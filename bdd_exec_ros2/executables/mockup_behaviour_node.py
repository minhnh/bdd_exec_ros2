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
import time
from random import random

import rclpy
from bdd_dsl.models.observation import EntityObservation
from bdd_ros2_interfaces.action import Behaviour
from bdd_ros2_interfaces.msg import Collision, Event, Trinary, TrinaryStamped
from coord_dsl.event_loop import reconfig_event_buffers
from coord_dsl.fsm import FSMData, consume_event, fsm_step, produce_event
from rclpy.action.server import ActionServer, CancelResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rdf_utils.namespace import URL_SECORO_M
from rdflib import Graph, Namespace, URIRef
from vision_msgs.msg import Detection3D

from bdd_exec_ros2.behaviours.fsm_pickplace import (
    EVENT_URIS,
    EventID,
    StateID,
    create_fsm,
)
from bdd_exec_ros2.observation import (
    map_detection3d_entity_by_dict,
)

__DEFAULT_NODE_NAME = "mockup_behaviour"
TOPIC_LOCATED_PICK = "/obs_policy/located_at_pick_ws"
TOPIC_IS_HELD = "/obs_policy/is_held"
TOPIC_DETECTIONS_3D = "/obs_policy/detections_3d"
TOPIC_COLLISION = "/observations/collision"

NS_M_TMPL = Namespace(f"{URL_SECORO_M}/acceptance-criteria/bdd/templates/")
NS_M_ENV_SECORO = Namespace(f"{URL_SECORO_M}/environments/secorolab/")
NS_M_AGN_ISAAC = Namespace(f"{URL_SECORO_M}/agents/isaac-sim/")
NS_MANAGER = Graph().namespace_manager
NS_MANAGER.bind("env-secoro", NS_M_ENV_SECORO)
NS_MANAGER.bind("agn-isaac", NS_M_AGN_ISAAC)
NS_MANAGER.bind("tmpl", NS_M_TMPL)

MOCKUP_DETECTION3D_ENTITY_URIS = {
    entity_id: NS_M_ENV_SECORO[entity_id]
    for entity_id in (
        "tomato_soup_can",
        "mustard_bottle",
        "dex_cube",
        "box1_ws",
        "box2_ws",
    )
}
MOCKUP_ENTITY_DETECTION3D_IDS = {
    uri: entity_id for entity_id, uri in MOCKUP_DETECTION3D_ENTITY_URIS.items()
}
MOCKUP_COLLISION_WORKSPACE_BODY_IDS = {
    NS_M_ENV_SECORO["tomato_soup_can"]: "/spawned/soup_can",
    NS_M_ENV_SECORO["mustard_bottle"]: "/spawned/mustard",
    NS_M_ENV_SECORO["dex_cube"]: "/spawned/cube",
    NS_M_ENV_SECORO["box1_ws"]: "/spawned/box1",
    NS_M_ENV_SECORO["box2_ws"]: "/spawned/box2",
    NS_M_ENV_SECORO["table"]: "/background/table_low_327",
}


def map_detection3d_entity_mockup(
    observation: Detection3D,
    scene_instance: object = None,
    targets: list[URIRef] | None = None,
) -> list[EntityObservation]:
    del scene_instance
    del targets
    return map_detection3d_entity_by_dict(observation, MOCKUP_DETECTION3D_ENTITY_URIS)


EXPORTED_EVENTS = {
    EventID.EVT_PICK_START,
    EventID.EVT_PICK_END,
    EventID.EVT_PLACE_START,
    EventID.EVT_PLACE_END,
}


def random_in_range(lower, upper):
    return random() * (upper - lower) + lower


class UserData:
    start_time: float
    perceive_delay: float
    approach_delay: float
    pick_delay: float
    place_delay: float
    picking: bool
    placing: bool
    succeeded: Trinary

    def __init__(self, delay_lower, delay_upper) -> None:
        self.start_time = time.time()
        self.perceive_delay = random_in_range(delay_lower, delay_upper)
        self.approach_delay = random_in_range(delay_lower, delay_upper)
        self.pick_delay = random_in_range(delay_lower, delay_upper)
        self.place_delay = random_in_range(delay_lower, delay_upper)
        self.picking = False
        self.placing = False
        self.succeeded = Trinary()
        self.succeeded.value = Trinary.UNKNOWN

    def elapsed(self, cur_state: StateID) -> bool:
        if cur_state == StateID.S_PERCEIVE:
            delay = self.perceive_delay
        elif cur_state == StateID.S_APPROACH:
            delay = self.approach_delay
        elif cur_state == StateID.S_PICK:
            delay = self.pick_delay
        elif cur_state == StateID.S_PLACE:
            delay = self.place_delay
        else:
            raise ValueError(f"UserData.elapsed: unhandled state '{cur_state.name}'")

        cur_time = time.time()
        elapsed = self.start_time + delay < cur_time
        if elapsed:
            self.start_time = cur_time
        return elapsed


def fsm_mockup_bhv(fsm: FSMData, ud: UserData):
    current_state = StateID(fsm.current_state_index)

    if current_state == StateID.S_PERCEIVE:
        if not ud.elapsed(cur_state=current_state):
            return
        ud.picking = True
        produce_event(event_data=fsm.event_data, event_index=EventID.EVT_PERCEIVE_DONE)
        return

    if current_state == StateID.S_APPROACH:
        if not ud.elapsed(cur_state=current_state):
            return

        if ud.picking:
            assert not ud.placing, "both 'picking' & 'placing' are true in UserData"
            produce_event(fsm.event_data, event_index=EventID.EVT_PICK_APPROACH_DONE)
            return

        if ud.placing:
            assert not ud.picking, "both 'placing' & 'picking' are true in UserData"
            produce_event(fsm.event_data, event_index=EventID.EVT_PLACE_APPROACH_DONE)
            return

        raise AssertionError("Neither 'picking' or placing is true in approach state")

    if current_state == StateID.S_PICK:
        if not ud.elapsed(cur_state=current_state):
            return

        ud.picking = False
        ud.placing = True
        produce_event(fsm.event_data, event_index=EventID.EVT_PICK_END)
        return

    if current_state == StateID.S_PLACE:
        if not ud.elapsed(cur_state=current_state):
            return

        ud.placing = False
        ud.succeeded.value = Trinary.TRUE
        produce_event(fsm.event_data, event_index=EventID.EVT_PLACE_END)
        return


class MockupBhvNode(Node):
    event_topic: str
    loop_duration: float
    heartbeat_duration: float
    delay_lower: float
    delay_upper: float
    simulate_collision: bool
    server_name: str
    _action_server: ActionServer

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.declare_parameter("event_topic", "")
        self.declare_parameter("loop_duration", 0.01)
        self.declare_parameter("heartbeat_duration", 0.5)
        self.declare_parameter("delay_lower", 2.0)
        self.declare_parameter("delay_upper", 4.0)
        self.declare_parameter("bhv_server_name", "bhv_server")
        self.declare_parameter("simulate_collision", False)

        use_sim_time = self.get_parameter("use_sim_time").value
        self.get_logger().info(f"use_sim_time: {use_sim_time}")

        self.loop_duration = self.get_parameter("loop_duration").value
        self.heartbeat_duration = self.get_parameter("heartbeat_duration").value
        assert self.loop_duration > 0 and self.heartbeat_duration > 0, (
            f"Negative duration: hearbeat={self.heartbeat_duration}, loop={self.loop_duration}"
        )
        assert self.loop_duration * 3 < self.heartbeat_duration, (
            f"Hearbeat duration (hb={self.heartbeat_duration}) must be at least"
            f" 3 times loop duration (loop={self.loop_duration})"
        )
        self.get_logger().info(
            f"Duration: hearbeat={self.heartbeat_duration}, loop={self.loop_duration}"
        )

        self.delay_lower = self.get_parameter("delay_lower").value
        self.delay_upper = self.get_parameter("delay_upper").value
        self.simulate_collision = self.get_parameter("simulate_collision").value
        self.get_logger().info(f"Simulate collision: {self.simulate_collision}")
        assert self.delay_lower > 2 * self.heartbeat_duration, (
            f"Lower range for state delay (lower={self.delay_lower}) must be at least 2 times the hearbheat duration (hb={self.heartbeat_duration})"
        )
        assert self.delay_upper > self.delay_lower, (
            f"Upper range for state delay (upper={self.delay_upper}) must be greater than lower range (lower={self.delay_lower})"
        )
        self.get_logger().info(
            f"State duration range: [{self.delay_lower}, {self.delay_upper}]"
        )

        self.server_name = self.get_parameter("bhv_server_name").value
        self.get_logger().info(f"Behaviour server name: {self.server_name}")

        self._action_server = ActionServer(
            self,
            Behaviour,
            self.server_name,
            self.execute_callback,
            cancel_callback=self.cancel_callback,
        )

        self.event_topic = self.get_parameter("event_topic").value
        assert self.event_topic, f"{self.get_name()}: no 'event_topic' param specified"
        self.get_logger().info(f"Event topic: {self.event_topic}")

        self.evt_pub = self.create_publisher(
            msg_type=Event, topic=self.event_topic, qos_profile=10
        )

        self.located_pick_pub = self.create_publisher(
            msg_type=TrinaryStamped, topic=TOPIC_LOCATED_PICK, qos_profile=10
        )
        self.is_held_pub = self.create_publisher(
            msg_type=TrinaryStamped, topic=TOPIC_IS_HELD, qos_profile=10
        )
        self.detections_pub = self.create_publisher(
            msg_type=Detection3D, topic=TOPIC_DETECTIONS_3D, qos_profile=10
        )
        self.collision_pub = self.create_publisher(
            msg_type=Collision, topic=TOPIC_COLLISION, qos_profile=10
        )

    def _publish_detection(self, entity_uri: URIRef) -> None:
        detection_id = MOCKUP_ENTITY_DETECTION3D_IDS.get(entity_uri)
        if detection_id is None:
            self.get_logger().warning(f"No Detection3D ID for {entity_uri}")
            return
        detection = Detection3D()
        detection.header.stamp = self.get_clock().now().to_msg()
        detection.id = detection_id
        detection.bbox.center.orientation.w = 1.0
        self.detections_pub.publish(detection)

    def _publish_collision(
        self, object_uris: list[URIRef], workspace_uris: list[URIRef]
    ) -> None:
        msg = Collision()
        msg.stamp = self.get_clock().now().to_msg()
        if self.simulate_collision:
            msg.bodies = [
                MOCKUP_COLLISION_WORKSPACE_BODY_IDS.get(
                    entity_uri,
                    MOCKUP_ENTITY_DETECTION3D_IDS.get(entity_uri),
                )
                for entity_uri in [*object_uris, *workspace_uris]
                if entity_uri in MOCKUP_COLLISION_WORKSPACE_BODY_IDS
                or entity_uri in MOCKUP_ENTITY_DETECTION3D_IDS
            ]
        self.collision_pub.publish(msg)

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Canceling goal...")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info("Received goal:")
        ctx_id = goal_handle.request.scenario_context_id

        response = Behaviour.Result()
        response.result.scenario_context_id = ctx_id
        feedback = Behaviour.Feedback()
        feedback.scenario_context_id = ctx_id

        agn_str = None
        obj_str = None
        object_uris = []
        workspace_uris = []
        variable_names = {
            NS_M_TMPL["var-target_object"]: "target_object",
            NS_M_TMPL["var-place_ws"]: "place_ws",
            NS_M_TMPL["var-robot"]: "robot",
        }
        for param_val in goal_handle.request.parameters:
            variable_uri = URIRef(param_val.variable_uri)
            self.get_logger().info(
                f"- Behaviour variable {variable_names.get(variable_uri, variable_uri.n3(NS_MANAGER))}"
            )

            val_uris = []
            for val_uri_str in param_val.param_val_uris:
                val_uri = URIRef(val_uri_str)
                val_uris.append(val_uri)
                self.get_logger().info(f"  + Parameter value: {val_uri.n3(NS_MANAGER)}")

            if variable_uri == NS_M_TMPL["var-target_object"]:
                object_uris = val_uris
                obj_str = f"[{', '.join([uri.n3(NS_MANAGER) for uri in val_uris])}]"

            if variable_uri == NS_M_TMPL["var-place_ws"]:
                workspace_uris = val_uris

            if variable_uri == NS_M_TMPL["var-robot"]:
                agn_str = f"[{', '.join([uri.n3(NS_MANAGER) for uri in val_uris])}]"

        for cfg_msg in goal_handle.request.configs:
            self.get_logger().info(
                f"Config {cfg_msg.target}: {cfg_msg.name} = {cfg_msg.num_value}"
            )

        pp_fsm = create_fsm()
        ud = UserData(delay_lower=self.delay_lower, delay_upper=self.delay_upper)

        now = time.time()
        loop_timeout = now + self.loop_duration
        heartbeat_timeout = now + self.heartbeat_duration
        trinary_msg = TrinaryStamped()
        trinary_msg.scenario_context_id = ctx_id
        trinary_msg.trinary.value = Trinary.TRUE
        self._publish_collision([], [])
        while True:
            # Ensure loop rate & produce step event
            now = time.time()
            if now < loop_timeout:
                continue
            while loop_timeout < now:
                loop_timeout += self.loop_duration
            produce_event(pp_fsm.event_data, EventID.EVT_STEP)

            for evt in EXPORTED_EVENTS:
                if not consume_event(pp_fsm.event_data, evt):
                    continue
                evt_msg = Event()
                evt_msg.scenario_context_id = ctx_id
                evt_msg.stamp = self.get_clock().now().to_msg()
                evt_msg.uri = EVENT_URIS[evt]
                self.evt_pub.publish(evt_msg)

            response.result.stamp = self.get_clock().now().to_msg()
            response.result.trinary = ud.succeeded
            if pp_fsm.current_state_index == StateID.S_EXIT:
                goal_handle.succeed()
                for entity_uri in [*object_uris, *workspace_uris]:
                    self._publish_detection(entity_uri)
                return response

            if goal_handle.is_cancel_requested:
                self.get_logger().info(
                    f"Goal canceled at {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(now))}"
                )
                # produce_event(
                #     event_data=pp_fsm.event_data, event_index=EventID.E_PREEMPTED
                # )
                goal_handle.canceled()
                return response

            # Heartbeat timer for callback message
            if heartbeat_timeout < now:
                while heartbeat_timeout < now:
                    heartbeat_timeout += self.heartbeat_duration
                feedback.status = f"current state: {agn_str} {StateID(pp_fsm.current_state_index).name} {obj_str}"
                goal_handle.publish_feedback(feedback)

                trinary_msg.stamp = self.get_clock().now().to_msg()
                if pp_fsm.current_state_index == StateID.S_PERCEIVE:
                    self.located_pick_pub.publish(trinary_msg)
                elif pp_fsm.current_state_index == StateID.S_APPROACH and ud.placing:
                    self.is_held_pub.publish(trinary_msg)
                self._publish_collision(object_uris, workspace_uris)

            # execute behaviour
            fsm_mockup_bhv(fsm=pp_fsm, ud=ud)

            # State transitions
            reconfig_event_buffers(pp_fsm.event_data)
            fsm_step(pp_fsm)

        return response


def main(args=None):
    if args is None:
        node_name = __DEFAULT_NODE_NAME
    else:
        node_name = getattr(args, "node_name", __DEFAULT_NODE_NAME)

    try:
        rclpy.init(args=args)
        mockup_bhv_node = MockupBhvNode(node_name=node_name)
        rclpy.spin(mockup_bhv_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
