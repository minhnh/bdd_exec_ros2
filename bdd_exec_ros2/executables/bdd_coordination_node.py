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
import os
import threading
from collections import deque
from dataclasses import dataclass
from enum import StrEnum
from typing import Any
from uuid import UUID, uuid4

import rclpy
from ament_index_python import get_package_share_directory
from bdd_dsl.models.observation import (
    ObservationManager,
    TrinaryStamped,
    trin_policy_and,
)
from bdd_dsl.models.user_story import ScenarioVariantModel, UserStoryLoader
from bdd_dsl.models.variation import (
    collect_variable_scene_elements,
    get_task_var_dicts,
)
from bdd_dsl.representation import (
    ClauseRepBuilder,
    ScenarioVariantRep,
    get_str_tc_after_event,
    get_str_tc_before_event,
    get_str_tc_during_events,
    get_tmpl_bhv_pickplace,
    get_tmpl_fc_config,
    get_tmpl_fc_is_held,
    get_tmpl_fc_located_at,
    get_tmpl_fc_str_tmpl,
)
from bdd_ros2_interfaces.action import Behaviour
from bdd_ros2_interfaces.msg import (
    Event,
    ScenarioStatusList,
)
from bdd_ros2_interfaces.msg import (
    TrinaryStamped as TrinaryStampedMsg,
)
from geometry_msgs.msg import PoseStamped
from rclpy.action.client import ActionClient, ClientGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.time import Time
from rdf_utils.models.common import ModelBase
from rdf_utils.models.vocab import (
    URI_ROS_PRED_CHNL_NAME,
    URI_ROS_PRED_TYPE_NAME,
    URI_ROS_TYPE_SIM_ENTITY_STATE_PROVIDER,
    URI_ROS_TYPE_TOPIC,
    URI_ROS_TYPE_TRANSFORM_LISTENER,
)
from rdf_utils.uri import try_expand_curie
from rdflib import RDF, Graph, URIRef
from rdflib.namespace import NamespaceManager
from scene_dsl.rdf_parser.scenex import SceneInstanceModel
from scene_dsl.rdf_parser.sensors import get_update_rate
from std_msgs.msg import Empty as EmptyMsg
from unique_identifier_msgs.msg import UUID as UUIDMsg

from bdd_exec_ros2.conversions import (
    TRINARY_NAMES,
    format_time_msg,
    from_trin_stamped_msg,
    from_uuid_msg,
    get_bhv_param_messages,
    get_cfg_messages,
    ros_time_to_stamp,
    to_scenario_status_msg,
    to_uuid_msg,
)
from bdd_exec_ros2.observation import (
    load_ros_action_model,
    load_ros_topic_model,
)
from bdd_exec_ros2.sim_interfaces import PosePollingHandle, SimInterface
from bdd_exec_ros2.tf_provider import TfPollingHandle, TfProvider

__DEFAULT_NODE_NAME = "test_coordinator"


class SceneSetupMode(StrEnum):
    NONE = "none"
    SIMULATION = "simulation"


def _is_context_id_uninitialized(context_id: UUIDMsg) -> bool:
    return not any(context_id.uuid)


def load_graph_models_in_yaml(models_yml: str) -> Graph:
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

    g = Graph()
    for model_info in models_list:
        path = model_info.get("path", None)
        if path is None:
            raise ValueError(f"no 'path' in model entry: {model_info}")
        path_type = model_info.get("path_type", None)
        if path_type == "ros":
            pkg_name = model_info.get("package_name", None)
            if pkg_name is None:
                raise ValueError(f"no 'package_name' specified for ROS path: {path}")
            pkg_share_path = get_package_share_directory(package_name=pkg_name)
            path = os.path.join(pkg_share_path, path)

        if model_info["format"] == "robbdd":
            from robbdd.rdf.bdd import create_bdd_model_graph
            from robbdd.rdf.bddx import create_bddx_model_graph
            from textx import metamodel_for_file
            from textx.registration import language_for_file

            model = metamodel_for_file(path).model_from_file(path)
            lang = language_for_file(path).name

            if lang == "robbdd":
                create_bdd_model_graph(model=model, g=g)
            elif lang == "robbdd-exec":
                create_bddx_model_graph(model=model, g=g)
            else:
                raise ValueError(
                    f"unsupported language '{lang}' for RobBDD model '{path}'"
                )
            continue

        # assuming model can be loaded using rdflib
        g.parse(path, format=model_info["format"])

    return g


@dataclass
class ScenarioContext:
    """Context for tracking scenario execution"""

    context_id: UUID
    obs_manager: ObservationManager
    scr_rep: ScenarioVariantRep
    variation_params: dict[URIRef, Any]
    scene_inst: SceneInstanceModel
    simulation_observations: dict[URIRef, tuple[float, dict[URIRef, URIRef]]]
    transform_observations: dict[URIRef, float]
    # Useful for handling timeout, cancelation
    goal_handle: ClientGoalHandle | None = None
    end_event_sent: bool = False


class BddCoordNode(Node):
    timeout_sec: float
    graph: Graph
    _ns_manager: NamespaceManager
    us_loader: UserStoryLoader

    _use_sim_time: bool
    _scene_setup_mode: SceneSetupMode
    _scene_setup_active: bool
    _sim_interface: SimInterface | None
    _pending_scenarios: deque[tuple[ScenarioVariantModel, dict[URIRef, Any]]]
    _scenario_contexts: dict[UUID, ScenarioContext]
    _scr_lock: threading.Lock

    _clause_rep_builder: ClauseRepBuilder

    _obs_cb_group: MutuallyExclusiveCallbackGroup
    _topic_fpolicy_reg: dict[str, dict[UUID, set[URIRef]]]
    _fpolicy_subs: dict[str, Subscription]
    _topic_observation_reg: dict[tuple[str, type], dict[UUID, set[URIRef]]]
    _observation_subs: dict[tuple[str, type], Subscription]
    _simulation_observation_handles: dict[tuple[UUID, URIRef], PosePollingHandle]
    _transform_observation_handles: dict[tuple[UUID, URIRef], TfPollingHandle]
    _tf_provider: TfProvider | None

    _action_client: ActionClient
    _evt_pub: Publisher
    _evt_sub: Subscription
    _scr_status_pub: Publisher

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.declare_parameter("bhv_server_name", "bhv_server")
        self.declare_parameter("start_test_topic", "start")
        self.declare_parameter("status_timer_period", 0.5)
        self.declare_parameter("timeout", 5.0)
        self.declare_parameter("tf_max_age", 1.0)
        self.declare_parameter("status_topic", "status")
        self.declare_parameter("event_topic", "")
        self.declare_parameter("graph_models", "")
        self.declare_parameter("scene_setup_mode", SceneSetupMode.NONE.value)
        self.declare_parameter("simulation_service_namespace", "/")
        self.declare_parameter("world_entity_name", "world")

        use_sim_time = self.get_parameter("use_sim_time").value
        self.get_logger().info(f"use_sim_time: {use_sim_time}")
        if not isinstance(use_sim_time, bool):
            raise TypeError(f"use_sim_time not a bool, got: {use_sim_time}")
        self._use_sim_time = use_sim_time

        timeout_sec = self.get_parameter("timeout").value
        self.get_logger().info(f"Timeout (seconds): {timeout_sec}")
        if not isinstance(timeout_sec, float):
            raise TypeError(f"'timeout' not a float, got: {timeout_sec}")
        self.timeout_sec = timeout_sec

        tf_max_age = self.get_parameter("tf_max_age").value
        if not isinstance(tf_max_age, float) or tf_max_age <= 0:
            raise ValueError(
                f"'tf_max_age' must be a positive float, got: {tf_max_age}"
            )
        self.tf_max_age = tf_max_age

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
        if not isinstance(start_test_topic, str):
            raise TypeError(
                f"expected str for 'start_test_topic' param, got: {type(start_test_topic)}"
            )
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
        if not isinstance(status_topic, str):
            raise TypeError(
                f"expected str for 'status_topic' param, got: {type(status_topic)}"
            )
        timer_period = self.get_parameter("status_timer_period").value
        if not isinstance(timer_period, float):
            raise TypeError(
                f"expected float for 'timer_period' param, got: {type(timer_period)}"
            )
        self._scr_status_pub = self.create_publisher(
            msg_type=ScenarioStatusList, topic=status_topic, qos_profile=10
        )
        self.timer = self.create_timer(timer_period, self._status_timer_callback)

        # Load model graph
        g_models_yml = self.get_parameter("graph_models").value
        if not isinstance(g_models_yml, str):
            raise TypeError(
                f"expected str 'graph_models' file name param, got: {type(g_models_yml)}"
            )
        self.get_logger().info(f"YAML list of graph models: {g_models_yml}")
        self.graph = load_graph_models_in_yaml(models_yml=g_models_yml)
        self._tf_provider = (
            TfProvider(
                self,
                max_age_sec=self.tf_max_age,
                callback_group=self._obs_cb_group,
            )
            if any(self.graph.subjects(RDF.type, URI_ROS_TYPE_TRANSFORM_LISTENER))
            else None
        )
        self._ns_manager = self.graph.namespace_manager
        self.us_loader = UserStoryLoader(graph=self.graph, shacl_check=True)

        self._scene_setup_mode, self._sim_interface = self._resolve_scene_setup()

        # Add lock to manager objects that may be modified across threads/processes
        self._scr_lock = threading.Lock()
        self._topic_fpolicy_reg = {}
        self._scenario_contexts = {}
        self._pending_scenarios = deque()
        self._scene_setup_active = False
        self._clause_rep_builder = ClauseRepBuilder(
            tmpl_creators=[
                get_tmpl_fc_is_held,
                get_tmpl_fc_located_at,
                get_tmpl_bhv_pickplace,
                get_tmpl_fc_str_tmpl,
                lambda model, **kwargs: get_tmpl_fc_config(
                    model, ns_manager=self._ns_manager, **kwargs
                ),
            ],
            tc_str_gens=[
                get_str_tc_after_event,
                get_str_tc_before_event,
                get_str_tc_during_events,
            ],
        )

        # Observation
        self._fpolicy_subs = {}
        self._topic_observation_reg = {}
        self._observation_subs = {}
        self._simulation_observation_handles = {}
        self._transform_observation_handles = {}

    def _resolve_scene_setup(self) -> tuple[SceneSetupMode, SimInterface | None]:
        mode_value = self.get_parameter("scene_setup_mode").value
        if not isinstance(mode_value, str):
            raise TypeError("'scene_setup_mode' must be a string")
        try:
            mode = SceneSetupMode(mode_value)
        except ValueError as exc:
            supported = ", ".join(item.value for item in SceneSetupMode)
            raise ValueError(
                f"unsupported 'scene_setup_mode' {mode_value!r}; expected one of: {supported}"
            ) from exc

        if mode is SceneSetupMode.NONE:
            return mode, None

        if mode is SceneSetupMode.SIMULATION:
            simulation_service_namespace = self.get_parameter(
                "simulation_service_namespace"
            ).value
            if not isinstance(simulation_service_namespace, str):
                raise TypeError("'simulation_service_namespace' must be a string")
            world_entity_name = self.get_parameter("world_entity_name").value
            if not isinstance(world_entity_name, str):
                raise TypeError("'world_entity_name' must be a string")
            return mode, SimInterface(
                node=self,
                timeout=self.timeout_sec,
                ns=simulation_service_namespace,
                model_graph=self.graph,
                world_entity_name=world_entity_name,
            )

        raise AssertionError(f"unhandled scene setup mode: {mode}")

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
        for ctx_provider_dict in self._topic_observation_reg.values():
            ctx_provider_dict.pop(context_id, None)
        for key, handle in tuple(self._simulation_observation_handles.items()):
            if key[0] == context_id:
                handle.cancel()
                del self._simulation_observation_handles[key]
        for key, handle in tuple(self._transform_observation_handles.items()):
            if key[0] == context_id:
                handle.cancel()
                del self._transform_observation_handles[key]

    def _update_observation(self, topic_key: tuple[str, type], msg: Any) -> None:
        receipt_stamp = ros_time_to_stamp(self.get_clock().now())
        with self._scr_lock:
            routes = tuple(
                (self._scenario_contexts[context_id].obs_manager, provider_uri)
                for context_id, providers in self._topic_observation_reg.get(
                    topic_key, {}
                ).items()
                if context_id in self._scenario_contexts
                for provider_uri in providers
            )

        for obs_manager, provider_uri in routes:
            with self._scr_lock:
                _ = obs_manager.update_provider_observation(
                    provider_uri, msg, receipt_stamp
                )
                # for policy_uri, (updated, reason) in results.items():
                #     if not updated:
                #         self.get_logger().warning(
                #             f"Observation for policy '{policy_uri.n3(self._ns_manager)}' not added: {reason}"
                #         )

    def _update_fpolicy_assertion(self, topic_name: str, msg: TrinaryStampedMsg):
        with self._scr_lock:
            if topic_name not in self._topic_fpolicy_reg:
                self.get_logger().error(f"no policy registered for topic: {topic_name}")
                return

            trin_val = msg.trinary.value
            if trin_val not in TRINARY_NAMES:
                trin_rep = f"{trin_val} ({format_time_msg(msg=msg.stamp, use_sim_time=self._use_sim_time)})"
                self.get_logger().error(
                    f"Policy assertion callback: no name found for trinary value [{trin_rep}]"
                )
                return

            trin_rep = f"{TRINARY_NAMES[trin_val]} ({format_time_msg(msg=msg.stamp, use_sim_time=self._use_sim_time)})"
            if msg.reason:
                trin_rep += f", reason: {msg.reason}"

            forward_to_all = _is_context_id_uninitialized(msg.scenario_context_id)
            trin_st, ctx_uuid = from_trin_stamped_msg(msg)

            if forward_to_all:
                self.get_logger().warning(
                    f"Trinary [{trin_rep}] from {topic_name} has no scenario context; forwarding to all active scenarios"
                )
                context_ids = tuple(self._scenario_contexts)
            elif ctx_uuid not in self._scenario_contexts:
                self.get_logger().error(
                    f"Trinary [{trin_rep}] from {topic_name}: Scenario context '{ctx_uuid.hex}' not found"
                )
                return
            else:
                self.get_logger().info(f"received trinary [{trin_rep}]")
                context_ids = (ctx_uuid,)

            for context_id in context_ids:
                if context_id not in self._topic_fpolicy_reg[topic_name]:
                    if not forward_to_all:
                        self.get_logger().error(
                            f"Trinary [{trin_rep}] from {topic_name}: No fluent policy registered for context '{context_id.hex}'"
                        )
                    continue
                ctx = self._scenario_contexts[context_id]
                policies = self._topic_fpolicy_reg[topic_name][context_id]
                for fpolicy_uri in policies:
                    updated, reason = ctx.obs_manager.update_fpolicy_assertion(
                        policy_uri=fpolicy_uri, trin_st=trin_st
                    )
                    if not updated:
                        self.get_logger().warning(
                            f"Trinary [{trin_rep}] not added: {reason}"
                        )

    def _create_subscription(self, model: ModelBase, context_id: UUID):
        if URI_ROS_TYPE_TOPIC not in model.types:
            return

        topic_name = model.get_attr(key=URI_ROS_PRED_CHNL_NAME)
        msg_type = model.get_attr(key=URI_ROS_PRED_TYPE_NAME)
        assert isinstance(topic_name, str) and msg_type is not None, (
            f"invalid attrs for {model.id}: topic={topic_name}, msg_type={msg_type}"
        )
        assert issubclass(msg_type, TrinaryStampedMsg), (
            "currently only support TrinaryStamped policy assertions"
        )

        with self._scr_lock:
            if topic_name not in self._topic_fpolicy_reg:
                self._topic_fpolicy_reg[topic_name] = {}
            if context_id not in self._topic_fpolicy_reg[topic_name]:
                self._topic_fpolicy_reg[topic_name][context_id] = set()
            self._topic_fpolicy_reg[topic_name][context_id].add(model.id)

            if topic_name in self._fpolicy_subs:
                self.get_logger().info(
                    f"not creating new subscription for '{model.id.n3(self._ns_manager)}' on topic '{topic_name}'"
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

    def _create_observation_subscription(
        self, provider: ModelBase, context_id: UUID, obs_manager: ObservationManager
    ) -> None:
        if URI_ROS_TYPE_TOPIC not in provider.types:
            return

        topic_name = provider.get_attr(key=URI_ROS_PRED_CHNL_NAME)
        msg_type = provider.get_attr(key=URI_ROS_PRED_TYPE_NAME)
        assert isinstance(topic_name, str) and msg_type is not None, (
            f"invalid attrs for {provider.id}: topic={topic_name}, msg_type={msg_type}"
        )
        topic_key = (topic_name, msg_type)

        with self._scr_lock:
            providers = self._topic_observation_reg.setdefault(topic_key, {})
            providers.setdefault(context_id, set()).add(provider.id)
            if topic_key in self._observation_subs:
                return
            self._observation_subs[topic_key] = self.create_subscription(
                msg_type=msg_type,
                topic=topic_name,
                callback=lambda msg: self._update_observation(topic_key, msg),
                callback_group=self._obs_cb_group,
                qos_profile=10,
            )

    def _create_scenario_context(
        self, scr_var: ScenarioVariantModel, val_dict: dict[URIRef, Any]
    ) -> ScenarioContext:
        scr_context_id = uuid4()

        obs_manager = ObservationManager.from_scenario_variant(
            graph=self.graph,
            scr_var=scr_var,
            bhv_loaders=[load_ros_action_model],
            obs_loaders=[
                load_ros_topic_model,
                lambda graph, model, cid=scr_context_id, **kwargs: (
                    self._create_subscription(model=model, context_id=cid)
                ),
            ],
        )
        obs_manager.bind_observation_targets(val_dict)

        scene_inst = obs_manager.scenario_exec.scene_instance

        simulation_rates = {}
        transform_configs = {}
        for provider_uri, provider in obs_manager.providers.items():
            if URI_ROS_TYPE_TOPIC in provider.types:
                load_ros_topic_model(graph=self.graph, model=provider)
                self._create_observation_subscription(
                    provider=provider,
                    context_id=scr_context_id,
                    obs_manager=obs_manager,
                )
            elif URI_ROS_TYPE_SIM_ENTITY_STATE_PROVIDER in provider.types:
                simulation_rates[provider_uri] = get_update_rate(self.graph, provider)
            elif URI_ROS_TYPE_TRANSFORM_LISTENER in provider.types:
                transform_configs[provider_uri] = get_update_rate(self.graph, provider)

        scr_rep = ScenarioVariantRep(
            scr_var=scr_var,
            clause_rep_builder=self._clause_rep_builder,
            val_dict=val_dict,
            ns_manager=self._ns_manager,
        )

        context = ScenarioContext(
            context_id=scr_context_id,
            variation_params=val_dict,
            obs_manager=obs_manager,
            scr_rep=scr_rep,
            scene_inst=scene_inst,
            simulation_observations={},
            transform_observations={},
        )
        for provider_uri, rate in simulation_rates.items():
            modelled_elem_by_obs_target = {}
            for (
                observation_uri,
                observation_target,
            ) in obs_manager.observation_targets_for_provider(provider_uri).items():
                if observation_target is None:
                    raise ValueError(
                        f"simulation observation '{observation_uri}' has no modelled target"
                    )
                modelled_element = scene_inst.resolve_modelled_element_id(
                    observation_target
                )
                if modelled_element is None:
                    raise ValueError(
                        f"simulation observation '{observation_uri}' has no modelled target"
                    )
                modelled_elem_by_obs_target[observation_target] = modelled_element
            context.simulation_observations[provider_uri] = (
                rate,
                modelled_elem_by_obs_target,
            )
        context.transform_observations.update(transform_configs)
        return context

    def _update_simulation_observation(
        self,
        context_id: UUID,
        provider_uri: URIRef,
        poses: dict[URIRef, PoseStamped],
    ) -> None:
        with self._scr_lock:
            context = self._scenario_contexts.get(context_id)
            if context is None:
                return
            _, modelled_elem_by_obs_target = context.simulation_observations[
                provider_uri
            ]
        missing_targets = set(modelled_elem_by_obs_target.values()) - poses.keys()
        if missing_targets:
            self.get_logger().warning(
                f"simulation observation provider "
                f"'{provider_uri.n3(self._ns_manager)}' returned no state "
                f"for: {sorted(target.n3(self._ns_manager) for target in missing_targets)}",
                throttle_duration_sec=1.0,
            )
        provider_poses = {
            observed_target: poses[model_target]
            for observed_target, model_target in modelled_elem_by_obs_target.items()
            if model_target in poses
        }
        if not provider_poses:
            return
        receipt_stamp = ros_time_to_stamp(self.get_clock().now())
        with self._scr_lock:
            if self._scenario_contexts.get(context_id) is not context:
                return
            results = context.obs_manager.update_provider_observation(
                provider_uri, provider_poses, receipt_stamp
            )
        for policy_uri, (updated, reason) in results.items():
            if not updated:
                self.get_logger().warning(
                    f"simulation observation policy "
                    f"'{policy_uri.n3(self._ns_manager)}' rejected "
                    f"provider '{provider_uri.n3(self._ns_manager)}': {reason}",
                    throttle_duration_sec=1.0,
                )

    def _simulation_observation_error(
        self, provider_uri: URIRef, exc: Exception
    ) -> None:
        self.get_logger().warning(
            f"simulation observation provider "
            f"'{provider_uri.n3(self._ns_manager)}' failed: {exc}"
        )

    def _activate_simulation_observations(self, context: ScenarioContext) -> None:
        if not context.simulation_observations:
            return
        if self._sim_interface is None:
            raise RuntimeError("simulation observations require simulation scene setup")
        for provider_uri, (
            rate,
            modelled_elem_by_obs_target,
        ) in context.simulation_observations.items():
            key = (context.context_id, provider_uri)
            self._simulation_observation_handles[key] = (
                self._sim_interface.start_pose_polling(
                    context.scene_inst,
                    set(modelled_elem_by_obs_target.values()),
                    rate,
                    lambda poses, cid=context.context_id, pid=provider_uri: (
                        self._update_simulation_observation(cid, pid, poses)
                    ),
                    lambda exc, pid=provider_uri: self._simulation_observation_error(
                        pid, exc
                    ),
                )
            )

    def _update_transform_observation(
        self, context_id: UUID, provider_uri: URIRef, poses: dict[str, PoseStamped]
    ) -> None:
        receipt_stamp = ros_time_to_stamp(self.get_clock().now())
        with self._scr_lock:
            context = self._scenario_contexts.get(context_id)
            if context is None:
                return
            results = context.obs_manager.update_provider_observation(
                provider_uri,
                poses,
                receipt_stamp,
            )
        for policy_uri, (updated, reason) in results.items():
            if not updated:
                self.get_logger().warning(
                    f"transform observation policy '{policy_uri.n3(self._ns_manager)}' "
                    f"rejected provider '{provider_uri.n3(self._ns_manager)}': {reason}",
                    throttle_duration_sec=1.0,
                )

    def _activate_transform_observations(self, context: ScenarioContext) -> None:
        if context.transform_observations and self._tf_provider is None:
            raise RuntimeError("transform observations require a TF provider")
        if self._tf_provider is None:
            return
        for provider_uri, rate in context.transform_observations.items():
            key = (context.context_id, provider_uri)
            self._transform_observation_handles[key] = (
                self._tf_provider.start_pose_polling(
                    self.graph,
                    provider_uri,
                    rate,
                    lambda poses, cid=context.context_id, pid=provider_uri: (
                        self._update_transform_observation(cid, pid, poses)
                    ),
                )
            )

    def _start_scenario_variant(
        self, context: ScenarioContext, scr_var: ScenarioVariantModel
    ) -> None:
        scr_context_id = context.context_id
        val_dict = context.variation_params

        goal_msg = Behaviour.Goal()
        goal_msg.scenario_context_id = to_uuid_msg(scr_context_id)
        goal_msg.parameters = get_bhv_param_messages(scr_var.when_bhv_model, val_dict)
        goal_msg.configs = get_cfg_messages(scr_var=scr_var, var_value_dict=val_dict)
        with self._scr_lock:
            self._scenario_contexts[context.context_id] = context
        self._activate_simulation_observations(context)
        self._activate_transform_observations(context)

        # Publish scenario start event
        self._send_event(
            evt_uri=context.obs_manager.scenario_exec.start_event, ctx_id=scr_context_id
        )

        # Send goal asynchronously
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.bhv_feedback_cb
        )
        send_goal_future.add_done_callback(
            callback=lambda future, cid=context.context_id: self.bhv_goal_resp_cb(
                future, context_id=cid
            )
        )

    def _schedule_next_scenario(self) -> None:
        if self._scene_setup_mode is not SceneSetupMode.SIMULATION:
            return
        executor = self.executor
        if executor is None:
            raise RuntimeError("coordinator is not attached to an executor")

        with self._scr_lock:
            if (
                self._scene_setup_active
                or self._scenario_contexts
                or not self._pending_scenarios
            ):
                return
            self._scene_setup_active = True

        executor.create_task(self._prepare_next_scenario())

    async def _prepare_next_scenario(self) -> None:
        assert self._sim_interface is not None
        try:
            while True:
                with self._scr_lock:
                    if not self._pending_scenarios or self._scenario_contexts:
                        break
                    scr_var, val_dict = self._pending_scenarios.popleft()

                context = self._create_scenario_context(scr_var, val_dict)
                additional_elements: set[URIRef] = set()
                for value in val_dict.values():
                    collect_variable_scene_elements(
                        scr_var.scene, value, additional_elements
                    )

                try:
                    await self._sim_interface.setup_scene(
                        context.scene_inst,
                        additional_elements=additional_elements,
                    )
                except Exception as exc:  # noqa: BLE001 - skip failed setup and continue
                    with self._scr_lock:
                        self._remove_context_topic_reg(context.context_id)
                    self.get_logger().error(
                        f"Scene setup failed for '{scr_var.id}': {exc}"
                    )
                    continue

                self._start_scenario_variant(context, scr_var)
                break
        finally:
            with self._scr_lock:
                self._scene_setup_active = False

        # Scheduling is ignored while _scene_setup_active is true. Retry after
        # clearing it in case the started scenario already completed.
        self._schedule_next_scenario()

    def _status_timer_callback(self):
        with self._scr_lock:
            if not self._scenario_contexts:
                # no scenarios
                return

            now = self.get_clock().now()
            now_stamp = ros_time_to_stamp(now)
            status_msg = ScenarioStatusList()
            status_msg.stamp = now.to_msg()
            status_msg.scenarios = []

            finished_ids = set()
            for ctx_id, scr_ctx in self._scenario_contexts.items():
                if (
                    scr_ctx.obs_manager.scr_start_time is not None
                    and now_stamp < scr_ctx.obs_manager.scr_start_time
                ):
                    # this is needed to clear scenarios after a simulation reset
                    # since clock would again start from 0
                    if not self._use_sim_time:
                        raise RuntimeError(
                            f"not using simulation time but now ({now_stamp}) < scenario start"
                            f" ({scr_ctx.obs_manager.scr_start_time}) for '{scr_ctx.context_id}'"
                        )
                    finished_ids.add(ctx_id)
                    continue

                if (
                    not scr_ctx.end_event_sent
                    and (end_deadline := scr_ctx.obs_manager.pending_end_deadline)
                    is not None
                    and now_stamp >= end_deadline
                ):
                    self._send_event(
                        evt_uri=scr_ctx.obs_manager.scenario_exec.end_event,
                        ctx_id=ctx_id,
                    )
                    scr_ctx.end_event_sent = True

                scr_status = to_scenario_status_msg(
                    ctx_id=ctx_id,
                    obs_manager=scr_ctx.obs_manager,
                    scr_rep=scr_ctx.scr_rep,
                    now=now,
                    trinaries_policy=trin_policy_and,
                )
                status_msg.scenarios.append(scr_status)

                # if finished remove from active scenarios
                observation_end_times = [
                    policy.end_time
                    for policy in scr_ctx.obs_manager.obs_policies.values()
                    if policy.end_time is not None
                ]
                if scr_ctx.obs_manager.scr_end_time is not None and now_stamp >= max(
                    observation_end_times,
                    default=scr_ctx.obs_manager.scr_end_time,
                ):
                    finished_ids.add(ctx_id)

            for ctx_id in finished_ids:
                self.get_logger().info(f"Scenario {ctx_id.hex} finished, removing...")
                self._remove_context_topic_reg(context_id=ctx_id)
                del self._scenario_contexts[ctx_id]

        self._scr_status_pub.publish(status_msg)
        self._schedule_next_scenario()

    def start_test_cb(self, _):
        if self._scene_setup_mode is SceneSetupMode.SIMULATION:
            with self._scr_lock:
                if (
                    self._scene_setup_active
                    or self._pending_scenarios
                    or self._scenario_contexts
                ):
                    self.get_logger().warning(
                        "Ignoring start request while scenario execution is active"
                    )
                    return

        us_var_dict = self.us_loader.get_us_scenario_variants()
        for scr_var_set in us_var_dict.values():
            for scr_var_id in scr_var_set:
                scr_var = self.us_loader.load_scenario_variant(
                    full_graph=self.graph, variant_id=scr_var_id
                )

                var_val_dicts = get_task_var_dicts(scr_var.task_variation)
                for val_dict in var_val_dicts:
                    if self._scene_setup_mode is SceneSetupMode.SIMULATION:
                        with self._scr_lock:
                            self._pending_scenarios.append((scr_var, val_dict))
                    else:
                        context = self._create_scenario_context(scr_var, val_dict)
                        self._start_scenario_variant(context, scr_var)

        self._schedule_next_scenario()

    def evt_sub_cb(self, msg: Event):
        evt_uri = URIRef(msg.uri)
        evt_t = ros_time_to_stamp(Time.from_msg(msg.stamp))
        forward_to_all = _is_context_id_uninitialized(msg.scenario_context_id)
        evt_ctx_uuid = from_uuid_msg(msg.scenario_context_id)
        with self._scr_lock:
            evt_rep = try_expand_curie(
                ns_manager=self._ns_manager, curie_str=msg.uri, quiet=True
            )
            if evt_rep is None:
                evt_rep = f"<{msg.uri}>"
            evt_rep = f"{evt_rep} ({format_time_msg(msg=msg.stamp, use_sim_time=self._use_sim_time)})"
            self.get_logger().info(f"received event [{evt_rep}]")

            if forward_to_all:
                self.get_logger().warning(
                    f"Event [{evt_rep}] has no scenario context; forwarding to all active scenarios"
                )
                context_ids = tuple(self._scenario_contexts)
            elif evt_ctx_uuid not in self._scenario_contexts:
                self.get_logger().error(
                    f"Callback for event [{evt_rep}]: Scenario context '{evt_ctx_uuid.hex}' not found"
                )
                return
            else:
                context_ids = (evt_ctx_uuid,)

            for context_id in context_ids:
                ctx = self._scenario_contexts[context_id]
                try:
                    ctx.obs_manager.on_event(evt_uri=evt_uri, evt_t=evt_t)
                except ValueError as e:
                    self.get_logger().error(f"error on_event {ctx.context_id}: {e}")

    def bhv_goal_resp_cb(self, future, context_id: UUID):
        goal_handle = future.result()

        with self._scr_lock:
            if context_id not in self._scenario_contexts:
                self.get_logger().error(
                    f"Goal response callback received unknown context ID: {context_id}"
                )
                return

            ctx = self._scenario_contexts[context_id]

            if not goal_handle.accepted:
                self.get_logger().error(
                    f"Goal rejected for {context_id}, ending scenario"
                )
                ctx.obs_manager.update_bhv_result(
                    trin_st=TrinaryStamped(
                        stamp=ros_time_to_stamp(self.get_clock().now()),
                        trinary=False,
                        reason="goal rejected",
                    )
                )
                self._send_event(
                    evt_uri=ctx.obs_manager.scenario_exec.end_event, ctx_id=context_id
                )
                return

            self.get_logger().info(f"Goal accepted for {context_id}")
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
                f"Behaviour result callback: context ID doesn't match {context_id.hex} != {result_uuid.hex}"
            )

        trin_val = result.result.trinary.value
        if trin_val not in TRINARY_NAMES:
            self.get_logger().error(
                f"Behaviour result callback: invalid trinary value '{trin_val}' for '{context_id.hex}'"
            )
            trin_st = None
        else:
            trin_st, _ = from_trin_stamped_msg(result.result)
            trin_rep = f"{TRINARY_NAMES[trin_val]} ({format_time_msg(msg=result.result.stamp, use_sim_time=self._use_sim_time)})"
            self.get_logger().info(f"Result received for {context_id.hex}: {trin_rep}")

        with self._scr_lock:
            if context_id not in self._scenario_contexts:
                self.get_logger().error(
                    f"Result callback: context {context_id} not found"
                )
                return

            ctx = self._scenario_contexts[context_id]
            if trin_st is not None:
                ctx.obs_manager.update_bhv_result(trin_st=trin_st)


def main(args=None):
    if args is None:
        node_name = __DEFAULT_NODE_NAME
    else:
        node_name = getattr(args, "node_name", __DEFAULT_NODE_NAME)

    try:
        rclpy.init(args=args)
        mockup_bhv_node = BddCoordNode(node_name=node_name)
        rclpy.spin(mockup_bhv_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
