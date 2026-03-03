#!/usr/bin/env python3

# Copyright 2026 Minh Nguyen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from typing import Any, Optional
from uuid import UUID, uuid4
from dataclasses import dataclass
import threading
from rdflib import Dataset, URIRef

import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from std_msgs.msg import Empty as EmptyMsg

from rdf_utils.models.common import ModelBase
from bdd_dsl.models.clauses import WhenBehaviourModel
from bdd_dsl.models.user_story import ScenarioVariantModel, UserStoryLoader
from bdd_dsl.models.urirefs import (
    URI_BHV_PRED_TARGET_AGN,
    URI_BHV_PRED_TARGET_OBJ,
    URI_BHV_TYPE_PICK,
    URI_BHV_TYPE_PLACE,
)
from bdd_dsl.models.variation import get_task_var_dicts
from bdd_dsl.models.observation import ObservationManager, trin_policy_and

from bdd_ros2_interfaces.action import Behaviour
from bdd_ros2_interfaces.msg import (
    Event,
    ParamValue,
    ScenarioStatusList,
    TrinaryStamped,
)
from bdd_exec_ros2.conversions import (
    from_trin_stamped_msg,
    from_uuid_msg,
    to_paramval_message,
    to_scenario_status_msg,
    to_uuid_msg,
)
from bdd_exec_ros2.observation import load_ros_topic_model
from bdd_exec_ros2.urirefs import (
    URI_ROS_PRED_MSG_TYPE,
    URI_ROS_PRED_TOPIC_NAME,
    URI_ROS_TYPE_TOPIC,
)


__DEFAULT_NODE_NAME = "test_coordinator"


def load_graph_models_in_yaml(models_yml: str) -> Dataset:
    from pathlib import Path
    import yaml
    from rdf_utils.resolver import install_resolver

    yml_p = Path(models_yml)

    if not yml_p.is_file():
        raise FileNotFoundError(
            f"YAML listing graph models is not a file: {models_yml}"
        )
    with open(yml_p) as yml_f:
        models_list = yaml.safe_load(yml_f)

    install_resolver()

    g = Dataset()
    for model_info in models_list:
        if model_info["format"] == "robbdd":
            raise ValueError("RobBDD model not yet handled")

        # assuming model can be loaded using rdflib
        try:
            g.parse(model_info["path"], format=model_info["format"])
        except Exception as e:
            raise RuntimeError(f"Caught {e} while processing '{model_info['path']}'")

    return g


def get_bhv_param_messages(
    when_bhv: WhenBehaviourModel, var_value_dict: dict[URIRef, Any]
) -> list[ParamValue]:
    param_vals = []
    if URI_BHV_TYPE_PICK or URI_BHV_TYPE_PLACE in when_bhv.types:
        obj_var_uri = when_bhv.get_attr(URI_BHV_PRED_TARGET_OBJ)
        assert obj_var_uri is not None
        assert obj_var_uri in var_value_dict, f"no value for '{obj_var_uri}'"
        param_vals.append(
            to_paramval_message(
                rel_uri=URI_BHV_PRED_TARGET_OBJ, val=var_value_dict[obj_var_uri]
            )
        )

        agn_var_uri = when_bhv.get_attr(URI_BHV_PRED_TARGET_AGN)
        assert agn_var_uri is not None
        assert agn_var_uri in var_value_dict, f"no value for '{agn_var_uri}'"
        param_vals.append(
            to_paramval_message(
                rel_uri=URI_BHV_PRED_TARGET_AGN, val=var_value_dict[agn_var_uri]
            )
        )

    return param_vals


@dataclass
class ScenarioContext:
    """Context for tracking scenario execution"""

    context_id: UUID
    obs_manager: ObservationManager
    variation_params: dict[URIRef, Any]
    # Useful for handling timeout, cancelation
    goal_handle: Optional[ClientGoalHandle] = None


class BddCoordNode(Node):
    timeout_sec: float
    graph: Dataset
    us_loader: UserStoryLoader

    _scenario_contexts: dict[UUID, ScenarioContext]
    _scr_lock: threading.Lock

    _obs_cb_group: MutuallyExclusiveCallbackGroup
    _topic_fpolicy_reg: dict[str, dict[UUID, set[URIRef]]]
    _fpolicy_subs: dict[str, Subscription]

    _action_client: ActionClient
    _evt_pub: Publisher
    _evt_sub: Subscription
    _scr_status_pub: Publisher

    def __init__(self, node_name: str, timeout_sec: float = 5.0) -> None:
        super().__init__(node_name)
        self.timeout_sec = timeout_sec

        self.declare_parameter("bhv_server_name", "bhv_server")
        self.declare_parameter("start_test_topic", "start")
        self.declare_parameter("status_timer_period", 0.5)
        self.declare_parameter("status_topic", "status")
        self.declare_parameter("event_topic", "")
        self.declare_parameter("graph_models", "")

        use_sim_time = self.get_parameter("use_sim_time").value
        self.get_logger().info(f"use_sim_time: {use_sim_time}")

        # Behaviour action server
        server_name = self.get_parameter("bhv_server_name").value
        self.get_logger().info(f"Behaviour server name: {server_name}")
        self._action_client = ActionClient(self, Behaviour, server_name)
        is_ready = self._action_client.wait_for_server(timeout_sec=self.timeout_sec)
        if not is_ready:
            raise RuntimeError(
                f"Timed out after {self.timeout_sec} secs waiting for server '{server_name}'"
            )

        # Ensure events and trinaries callbacks are handled in sequence
        self._obs_cb_group = MutuallyExclusiveCallbackGroup()

        # Test starting topic
        start_test_topic = self.get_parameter("start_test_topic").value
        self.get_logger().info(f"Start topic: {start_test_topic}")
        self._start_test_sub = self.create_subscription(
            msg_type=EmptyMsg,
            topic=start_test_topic,
            callback=self.start_test_cb,
            qos_profile=10,
        )

        # Topic for events
        self.event_topic = self.get_parameter("event_topic").value
        assert self.event_topic, f"{self.get_name()}: no 'event_topic' param specified"
        self.get_logger().info(f"Event topic: {self.event_topic}")

        self._evt_pub = self.create_publisher(
            msg_type=Event, topic=self.event_topic, qos_profile=10
        )

        self._evt_sub = self.create_subscription(
            msg_type=Event,
            topic=self.event_topic,
            callback=self.evt_sub_cb,
            callback_group=self._obs_cb_group,
            qos_profile=10,
        )

        # Timer & publisher for broadcasting scenario status
        status_topic = self.get_parameter("status_topic").value
        timer_period = self.get_parameter("status_timer_period").value
        self.timer = self.create_timer(timer_period, self._status_timer_callback)
        self._scr_status_pub = self.create_publisher(
            msg_type=ScenarioStatusList, topic=status_topic, qos_profile=10
        )

        # Load model graph
        g_models_yml = self.get_parameter("graph_models").value
        self.get_logger().info(f"YAML list of graph models: {g_models_yml}")
        self.graph = load_graph_models_in_yaml(models_yml=g_models_yml)
        self.us_loader = UserStoryLoader(graph=self.graph, shacl_check=True)

        # Set up context management for executing scenario variants
        self._scenario_contexts = {}
        self._scr_lock = threading.Lock()

        # Observation
        self._topic_fpolicy_reg = {}  # lock thread before modifying
        self._fpolicy_subs = {}

    def _send_event(self, evt_uri: URIRef, ctx_id: UUID) -> None:
        evt_msg = Event()
        evt_msg.scenario_context_id = to_uuid_msg(ctx_id)
        evt_msg.uri = evt_uri.toPython()
        evt_msg.stamp = self.get_clock().now().to_msg()
        self._evt_pub.publish(evt_msg)

    def _remove_context_topic_reg(self, context_id):
        for ctx_fc_dict in self._topic_fpolicy_reg.values():
            if context_id not in ctx_fc_dict:
                continue
            del ctx_fc_dict[context_id]

    def _update_fpolicy_assertion(self, topic_name: str, msg: TrinaryStamped):
        assert topic_name in self._topic_fpolicy_reg, (
            f"no policy registered for topic: {topic_name}"
        )

        trin_st, ctx_uuid = from_trin_stamped_msg(msg)

        with self._scr_lock:
            if ctx_uuid not in self._scenario_contexts:
                self.get_logger().error(
                    f"Trinary update from {topic_name}: Scenario context '{ctx_uuid.hex}' not found"
                )
                return

            if ctx_uuid not in self._topic_fpolicy_reg[topic_name]:
                self.get_logger().error(
                    f"Trinary update from {topic_name}: No fluent policy registered for context '{ctx_uuid.hex}'"
                )
                return

            ctx = self._scenario_contexts[ctx_uuid]
            policies = self._topic_fpolicy_reg[topic_name][ctx_uuid]
            for fpolicy_uri in policies:
                ctx.obs_manager.update_fpolicy_assertion(
                    fc_uri=fpolicy_uri, trin_st=trin_st
                )

    def _create_subscription(self, model: ModelBase, context_id: UUID):
        if URI_ROS_TYPE_TOPIC not in model.types:
            self.get_logger().warning(
                f"create_subscription: model {model.id} does not have ROSTopic type"
            )
            return

        topic_name = model.get_attr(key=URI_ROS_PRED_TOPIC_NAME)
        msg_type = model.get_attr(key=URI_ROS_PRED_MSG_TYPE)
        assert isinstance(topic_name, str) and msg_type is not None, (
            f"invalid attrs for {model.id}: topic={topic_name}, msg_type={msg_type}"
        )
        assert issubclass(msg_type, TrinaryStamped), (
            "currently only support TrinaryStamped policy assertions"
        )

        if topic_name not in self._topic_fpolicy_reg:
            self._topic_fpolicy_reg[topic_name] = {}
        if context_id not in self._topic_fpolicy_reg[topic_name]:
            self._topic_fpolicy_reg[topic_name][context_id] = set()
        self._topic_fpolicy_reg[topic_name][context_id].add(model.id)

        if topic_name in self._fpolicy_subs:
            self.get_logger().info(
                f"not creating new subscription for '{model.id.n3(self.graph.namespace_manager)}' on topic '{topic_name}'"
            )
            return

        self._fpolicy_subs[topic_name] = self.create_subscription(
            msg_type=msg_type,
            topic=topic_name,
            callback=lambda msg: self._update_fpolicy_assertion(
                topic_name=topic_name, msg=msg
            ),
            callback_group=self._obs_cb_group,
            qos_profile=10,
        )

    def _execute_scenario_variant(
        self, scr_var: ScenarioVariantModel, val_dict: dict[URIRef, Any]
    ):
        scr_context_id = uuid4()

        obs_manager = ObservationManager.from_scenario_variant(
            graph=self.graph,
            scr_var=scr_var,
            obs_loaders=[
                load_ros_topic_model,
                lambda graph, model, cid=scr_context_id, **kwargs: (
                    self._create_subscription(model=model, context_id=cid)
                ),
            ],
        )

        context = ScenarioContext(
            context_id=scr_context_id,
            variation_params=val_dict,
            obs_manager=obs_manager,
        )

        # Publish scenario start event
        self._send_event(
            evt_uri=context.obs_manager.scr_start_event, ctx_id=scr_context_id
        )

        goal_msg = Behaviour.Goal()
        goal_msg.scenario_context_id = to_uuid_msg(scr_context_id)
        goal_msg.parameters = get_bhv_param_messages(scr_var.when_bhv_model, val_dict)
        with self._scr_lock:
            self._scenario_contexts[context.context_id] = context

        # Send goal asynchronously
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.bhv_feedback_cb
        )
        send_goal_future.add_done_callback(
            callback=lambda future, cid=context.context_id: self.bhv_goal_resp_cb(
                future, context_id=cid
            )
        )

    def _status_timer_callback(self):
        if len(self._scenario_contexts) == 0:
            # no scenarios
            return

        now = self.get_clock().now()
        status_msg = ScenarioStatusList()
        status_msg.stamp = now.to_msg()
        status_msg.scenarios = []

        for ctx_id, scr_ctx in self._scenario_contexts.items():
            scr_status = to_scenario_status_msg(
                ctx_id=ctx_id,
                obs_manager=scr_ctx.obs_manager,
                now=now,
                trinaries_policy=trin_policy_and,
            )
            status_msg.scenarios.append(scr_status)

        self._scr_status_pub.publish(status_msg)

    def start_test_cb(self, _):
        us_var_dict = self.us_loader.get_us_scenario_variants()
        for scr_var_set in us_var_dict.values():
            for scr_var_id in scr_var_set:
                scr_var = self.us_loader.load_scenario_variant(
                    full_graph=self.graph, variant_id=scr_var_id
                )

                var_val_dicts = get_task_var_dicts(scr_var.task_variation)
                for val_dict in var_val_dicts:
                    self._execute_scenario_variant(
                        scr_var=scr_var,
                        val_dict=val_dict,
                    )

    def evt_sub_cb(self, msg: Event):
        self.get_logger().info(f"{msg.stamp.sec}: {msg.uri}")
        evt_uri = URIRef(msg.uri)
        evt_t = Time.from_msg(msg.stamp).to_datetime().timestamp()
        evt_ctx_uuid = from_uuid_msg(msg.scenario_context_id)
        with self._scr_lock:
            if evt_ctx_uuid not in self._scenario_contexts:
                self.get_logger().error(
                    f"Callback for event '{msg.uri}': Scenario context '{evt_ctx_uuid.hex}' not found"
                )
                return

            ctx = self._scenario_contexts[evt_ctx_uuid]
            try:
                ctx.obs_manager.on_event(evt_uri=evt_uri, evt_t=evt_t)
                # if finished remove from active scenarios
                if ctx.obs_manager.scr_end_time is not None:
                    self.get_logger().info(
                        f"Scenario {ctx.context_id} completed, removing..."
                    )
                    del self._scenario_contexts[evt_ctx_uuid]
            except ValueError as e:
                self.get_logger().error(f"error on_event {ctx.context_id}: {e}")

    def bhv_goal_resp_cb(self, future, context_id: UUID):
        goal_handle = future.result()

        assert context_id in self._scenario_contexts, "context ID not found"
        ctx = self._scenario_contexts[context_id]

        if not goal_handle.accepted:
            self.get_logger().error(f"Goal rejected for {context_id}, removing context")
            with self._scr_lock:
                self._send_event(
                    evt_uri=ctx.obs_manager.scr_end_event, ctx_id=context_id
                )
                self._remove_context_topic_reg(context_id=context_id)
            return

        self.get_logger().info(f"Goal accepted for {context_id}")
        with self._scr_lock:
            ctx.goal_handle = goal_handle

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(
            lambda future, cid=context_id: self.bhv_result_cb(future, context_id=cid)
        )

    def bhv_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Behaviour feedback: {feedback.status}")

    def bhv_result_cb(self, future, context_id: UUID):
        result = future.result().result
        result_uuid = from_uuid_msg(result.result.scenario_context_id)
        if context_id != result_uuid:
            self.get_logger().error(
                "Result callback: context ID doesn't match {context_id.hex} != {result_uuid.hex}"
            )
        self.get_logger().info(
            f"Result received for {context_id.hex}: {result.result.trinary.value}"
        )

        with self._scr_lock:
            if context_id not in self._scenario_contexts:
                self.get_logger().error(
                    f"Result callback: context {context_id} not found"
                )
                return
            ctx = self._scenario_contexts[context_id]
            self._remove_context_topic_reg(context_id=context_id)
            self._send_event(evt_uri=ctx.obs_manager.scr_end_event, ctx_id=context_id)


def main(args=None):
    if args is None:
        node_name = __DEFAULT_NODE_NAME
    else:
        node_name = getattr(args, "node_name", __DEFAULT_NODE_NAME)

    try:
        with rclpy.init(args=args):
            mockup_bhv_node = BddCoordNode(node_name=node_name)
            rclpy.spin(mockup_bhv_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
