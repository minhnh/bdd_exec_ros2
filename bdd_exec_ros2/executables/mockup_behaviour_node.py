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
import time
from random import random

from bdd_dsl.models.urirefs import URI_BHV_PRED_TARGET_AGN, URI_BHV_PRED_TARGET_OBJ
from rdflib import Graph, Namespace, URIRef
from rdf_utils.uri import URL_SECORO_M
from coord_dsl.fsm import FSMData, consume_event, fsm_step, produce_event
from coord_dsl.event_loop import reconfig_event_buffers

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import ExternalShutdownException

from bdd_ros2_interfaces.msg import Event, Trinary, TrinaryStamped
from bdd_ros2_interfaces.action import Behaviour
from bdd_exec_ros2.behaviours.fsm_pickplace import EventID, StateID, create_fsm


__DEFAULT_NODE_NAME = "mockup_behaviour"
TOPIC_LOCATED_PICK = "/obs_policy/located_at_pick_ws"
TOPIC_LOCATED_PLACE = "/obs_policy/located_at_place_ws"

NS_M_TMPL = Namespace(f"{URL_SECORO_M}/acceptance-criteria/bdd/templates/")
NS_M_ENV_SECORO = Namespace(f"{URL_SECORO_M}/environments/secorolab/")
NS_M_AGN_ISAAC = Namespace(f"{URL_SECORO_M}/agents/isaac-sim/")
NS_MANAGER = Graph().namespace_manager
NS_MANAGER.bind("env-secoro", NS_M_ENV_SECORO)
NS_MANAGER.bind("agn-isaac", NS_M_AGN_ISAAC)
NS_MANAGER.bind("tmpl", NS_M_TMPL)


EXPORTED_EVENTS = {
    EventID.E_PICK_APPROACH_START: NS_M_TMPL["evt-pick-start"],
    EventID.E_PLACE_DONE: NS_M_TMPL["evt-place-end"],
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
        produce_event(event_data=fsm.event_data, event_index=EventID.E_PERCEIVE_DONE)
        return

    if current_state == StateID.S_APPROACH:
        if not ud.elapsed(cur_state=current_state):
            return

        if ud.picking:
            assert not ud.placing, "both 'picking' & 'placing' are true in UserData"
            produce_event(fsm.event_data, event_index=EventID.E_PICK_APPROACH_DONE)
            return

        if ud.placing:
            assert not ud.picking, "both 'placing' & 'picking' are true in UserData"
            produce_event(fsm.event_data, event_index=EventID.E_PLACE_APPROACH_DONE)
            return

        raise AssertionError("Neither 'picking' or placing is true in approach state")

    if current_state == StateID.S_PICK:
        if not ud.elapsed(cur_state=current_state):
            return

        ud.picking = False
        ud.placing = True
        produce_event(fsm.event_data, event_index=EventID.E_PICK_DONE)
        return

    if current_state == StateID.S_PLACE:
        if not ud.elapsed(cur_state=current_state):
            return

        ud.placing = False
        ud.succeeded.value = Trinary.TRUE
        produce_event(fsm.event_data, event_index=EventID.E_PLACE_DONE)
        return


class MockupBhvNode(Node):
    event_topic: str
    loop_duration: float
    heartbeat_duration: float
    delay_lower: float
    delay_upper: float
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
        self.located_place_pub = self.create_publisher(
            msg_type=TrinaryStamped, topic=TOPIC_LOCATED_PLACE, qos_profile=10
        )

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Canceling goal...")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        response = Behaviour.Result()
        feedback = Behaviour.Feedback()
        self.get_logger().info("Received goal:")
        agn_str = None
        obj_str = None
        for param_val in goal_handle.request.parameters:
            rel_uri = URIRef(param_val.param_rel_uri)
            self.get_logger().info(f"- Parameter relation: {rel_uri.n3(NS_MANAGER)}")

            val_uris = []
            for val_uri_str in param_val.param_val_uris:
                val_uri = URIRef(val_uri_str)
                val_uris.append(val_uri)
                self.get_logger().info(f"  + Parameter value: {val_uri.n3(NS_MANAGER)}")

            if rel_uri == URI_BHV_PRED_TARGET_OBJ:
                obj_str = f"[{', '.join([uri.n3(NS_MANAGER) for uri in val_uris])}]"

            if rel_uri == URI_BHV_PRED_TARGET_AGN:
                agn_str = f"[{', '.join([uri.n3(NS_MANAGER) for uri in val_uris])}]"

        pp_fsm = create_fsm()
        ud = UserData(delay_lower=self.delay_lower, delay_upper=self.delay_upper)

        now = time.time()
        loop_timeout = now + self.loop_duration
        heartbeat_timeout = now + self.heartbeat_duration
        trinary_msg = TrinaryStamped()
        trinary_msg.trinary.value = Trinary.TRUE
        while True:
            # Ensure loop rate & produce step event
            now = time.time()
            if now < loop_timeout:
                continue
            while loop_timeout < now:
                loop_timeout += self.loop_duration
            produce_event(pp_fsm.event_data, EventID.E_STEP)

            for evt, evt_uri in EXPORTED_EVENTS.items():
                if not consume_event(pp_fsm.event_data, evt):
                    continue
                evt_msg = Event()
                evt_msg.stamp = self.get_clock().now().to_msg()
                evt_msg.uri = evt_uri.toPython()
                self.evt_pub.publish(evt_msg)

            response.result.stamp = self.get_clock().now().to_msg()
            response.result.trinary = ud.succeeded
            if pp_fsm.current_state_index == StateID.S_EXIT:
                goal_handle.succeed()
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
                elif pp_fsm.current_state_index == StateID.S_PLACE:
                    self.located_place_pub.publish(trinary_msg)

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
        with rclpy.init(args=args):
            mockup_bhv_node = MockupBhvNode(node_name=node_name)
            rclpy.spin(mockup_bhv_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
