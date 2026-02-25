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
from typing import Any, Iterable
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Empty as EmptyMsg
from rdflib import Dataset, URIRef
from bdd_dsl.models.clauses import FluentClauseModel, WhenBehaviourModel
from bdd_dsl.models.user_story import UserStoryLoader
from bdd_dsl.models.urirefs import (
    URI_BHV_PRED_TARGET_AGN,
    URI_BHV_PRED_TARGET_OBJ,
    URI_BHV_TYPE_PICK,
    URI_BHV_TYPE_PLACE,
    URI_OBS_TYPE_POLICY,
)
from bdd_dsl.models.variation import get_task_var_dicts
from bdd_ros2_interfaces.action import Behaviour
from bdd_ros2_interfaces.msg import Event, ParamValue, TrinaryStamped

from bdd_exec_ros2.observation import ObservationManager
from bdd_exec_ros2.urirefs import URI_ROS_PRED_MSG_TYPE, URI_ROS_PRED_TOPIC_NAME


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


def get_valid_paramval_message(rel_uri: URIRef, val: Any) -> ParamValue:
    param = ParamValue()
    param.param_rel_uri = rel_uri.toPython()
    if isinstance(val, URIRef):
        param.param_val_uris = [val.toPython()]
        return param

    if isinstance(val, Iterable):
        val_uris = []
        for uri in val:
            assert isinstance(uri, URIRef), f"not an Iterable of URIRef: {uri}"
            val_uris.append(uri.toPython())
        param.param_val_uris = val_uris
        return param

    raise RuntimeError(
        f"get_valid_paramval_message: unhandled types: (type={type(val)}) {val}"
    )


def get_bhv_param_messages(
    when_bhv: WhenBehaviourModel, var_value_dict: dict[URIRef, Any]
) -> list[ParamValue]:
    param_vals = []
    if URI_BHV_TYPE_PICK or URI_BHV_TYPE_PLACE in when_bhv.types:
        obj_var_uri = when_bhv.get_attr(URI_BHV_PRED_TARGET_OBJ)
        assert obj_var_uri is not None
        assert obj_var_uri in var_value_dict, f"no value for '{obj_var_uri}'"
        param_vals.append(
            get_valid_paramval_message(
                rel_uri=URI_BHV_PRED_TARGET_OBJ, val=var_value_dict[obj_var_uri]
            )
        )

        agn_var_uri = when_bhv.get_attr(URI_BHV_PRED_TARGET_AGN)
        assert agn_var_uri is not None
        assert agn_var_uri in var_value_dict, f"no value for '{agn_var_uri}'"
        param_vals.append(
            get_valid_paramval_message(
                rel_uri=URI_BHV_PRED_TARGET_AGN, val=var_value_dict[agn_var_uri]
            )
        )

    return param_vals


class BddCoordNode(Node):
    timeout_sec: float
    graph: Dataset
    us_loader: UserStoryLoader
    _obs_cb_group: MutuallyExclusiveCallbackGroup
    _fpolicy_by_topics: dict[str, set[URIRef]]
    _fpolicy_subs: dict[str, Subscription]
    _action_client: ActionClient
    _evt_pub: Publisher
    _evt_sub: Subscription

    def __init__(self, node_name: str, timeout_sec: float = 5.0) -> None:
        super().__init__(node_name)
        self._fpolicy_by_topics = {}
        self._fpolicy_subs = {}
        self._obs_cb_group = MutuallyExclusiveCallbackGroup()

        self.timeout_sec = timeout_sec

        self.declare_parameter("bhv_server_name", "bhv_server")
        self.declare_parameter("start_test_topic", "start")
        self.declare_parameter("event_topic", "")
        self.declare_parameter("graph_models", "")

        use_sim_time = self.get_parameter("use_sim_time").value
        self.get_logger().info(f"use_sim_time: {use_sim_time}")

        # Behaviour action server
        server_name = self.get_parameter("bhv_server_name").value
        self.get_logger().info(f"Behaviour server name: {server_name}")
        self._action_client = ActionClient(self, Behaviour, server_name)

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

        # Load model graph
        g_models_yml = self.get_parameter("graph_models").value
        self.get_logger().info(f"YAML list of graph models: {g_models_yml}")
        self.graph = load_graph_models_in_yaml(models_yml=g_models_yml)
        self.us_loader = UserStoryLoader(graph=self.graph, shacl_check=True)

    def start_test_cb(self, _):
        us_var_dict = self.us_loader.get_us_scenario_variants()
        for scr_var_set in us_var_dict.values():
            for scr_var_id in scr_var_set:
                scr_var = self.us_loader.load_scenario_variant(
                    full_graph=self.graph, variant_id=scr_var_id
                )
                obs_manager = ObservationManager()
                for fc in scr_var.fluent_clauses():
                    self.load_fluent_obs(obs_manager=obs_manager, fc=fc)

                var_val_dicts = get_task_var_dicts(scr_var.task_variation)
                for val_dict in var_val_dicts:
                    goal_msg = Behaviour.Goal()
                    goal_msg.parameters = get_bhv_param_messages(
                        scr_var.when_bhv_model, val_dict
                    )
                    send_goal_future = self._action_client.send_goal_async(
                        goal_msg, feedback_callback=self.bhv_feedback_cb
                    )
                    send_goal_future.add_done_callback(callback=self.bhv_goal_resp_cb)

    def load_fluent_obs(self, obs_manager: ObservationManager, fc: FluentClauseModel):
        if URI_OBS_TYPE_POLICY not in fc.types:
            self.get_logger().warning(f"no observation policy for fluent {fc.id}")
            return

        obs_manager.load_fluent_obs(fc=fc, graph=self.graph)

        topic_name = fc.get_attr(key=URI_ROS_PRED_TOPIC_NAME)
        msg_type = fc.get_attr(key=URI_ROS_PRED_MSG_TYPE)
        assert (
            isinstance(topic_name, str) and msg_type is not None
        ), f"invalid attrs for {fc.id}: topic={topic_name}, msg_type={msg_type}"
        assert issubclass(
            msg_type, TrinaryStamped
        ), "currently only support TrinaryStamped policy assertions"

        if topic_name not in self._fpolicy_by_topics:
            self._fpolicy_by_topics[topic_name] = set()
        self._fpolicy_by_topics[topic_name].add(fc.id)

        if topic_name in self._fpolicy_subs:
            self.get_logger().info(
                f"not creating new subscription for '{fc.id}' on topic '{topic_name}'"
            )
            return

        self._fpolicy_subs[topic_name] = self.create_subscription(
            msg_type=msg_type,
            topic=topic_name,
            callback=lambda msg: self.update_fpolicy_assertion(
                obs_manager=obs_manager, topic_name=topic_name, msg=msg
            ),
            callback_group=self._obs_cb_group,
            qos_profile=10,
        )

    def update_fpolicy_assertion(
        self, obs_manager: ObservationManager, topic_name: str, msg: TrinaryStamped
    ):
        assert (
            topic_name in self._fpolicy_by_topics
        ), f"no policy registered for topic: {topic_name}"
        for fpolicy_uri in self._fpolicy_by_topics[topic_name]:
            self.get_logger().info(f"new assertion for {fpolicy_uri}: {msg}")
            obs_manager.update_fpolicy_assertion(fc_uri=fpolicy_uri, trinary=msg)

    def evt_sub_cb(self, msg: Event):
        self.get_logger().info(f"{msg.stamp.sec}: {msg.uri}")

    def bhv_goal_resp_cb(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.bhv_result_cb)

    def bhv_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info("Behaviour feedback: {0}".format(feedback.status))

    def bhv_result_cb(self, future):
        result = future.result().result
        self.get_logger().info("Result: {0}".format(result.result))


def main(args=None):
    import rdf_utils

    print(rdf_utils.__path__)
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
