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

from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message

from trinary import Unknown
from rdflib import Graph, Literal, URIRef
from rdf_utils.models.common import ModelBase
from bdd_dsl.models.clauses import FluentClauseModel
from bdd_dsl.models.observation import FluentTimeline, TrinaryStamped
from bdd_ros2_interfaces.msg import (
    Event,
    TrinaryStamped as TrinaryStampedMsg,
    Trinary as TrinaryMsg,
)
from bdd_exec_ros2.urirefs import (
    URI_ROS_PRED_MSG_TYPE,
    URI_ROS_PRED_TOPIC_NAME,
    URI_ROS_TYPE_TOPIC,
)


def from_trin_stamped_msg(msg: TrinaryStampedMsg) -> TrinaryStamped:
    epoch_t = Time.from_msg(msg.stamp).to_datetime().timestamp()
    if msg.trinary.value == TrinaryMsg.FALSE:
        trin = False
    elif msg.trinary.value == TrinaryMsg.TRUE:
        trin = True
    elif msg.trinary.value == TrinaryMsg.UNKNOWN:
        trin = Unknown
    else:
        raise ValueError(f"Invalid trinary value in ROS message: {msg.trinary.value}")
    return TrinaryStamped(stamp=epoch_t, trinary=trin)


def load_ros_topic_model(model: ModelBase, graph: Graph):
    topic_name = graph.value(
        subject=model.id, predicate=URI_ROS_PRED_TOPIC_NAME, any=False
    )
    assert isinstance(
        topic_name, Literal
    ), f"'topic_name' of '{model.id}' not a Literal: {topic_name}"
    model.set_attr(key=URI_ROS_PRED_TOPIC_NAME, val=topic_name.toPython())

    msg_type_str = graph.value(
        subject=model.id, predicate=URI_ROS_PRED_MSG_TYPE, any=False
    )
    assert isinstance(
        msg_type_str, Literal
    ), f"'message-type' of '{model.id}' not a Literal: {msg_type_str}"

    msg_type = get_message(msg_type_str)
    model.set_attr(key=URI_ROS_PRED_MSG_TYPE, val=msg_type)


def insert_timestamp_in_order(stamp_list: list[Time], new_stamp: Time):
    # Find insertion point (from end)
    for i in range(len(stamp_list) - 1, -1, -1):
        if stamp_list[i] < new_stamp:
            stamp_list.insert(i + 1, new_stamp)
            return

    # Insert at beginning if smallest
    stamp_list.insert(0, new_stamp)


class ObservationManager(object):
    fluent_timelines: dict[URIRef, FluentTimeline]
    event_timelines: dict[URIRef, list[Time]]
    _fluent_event_registry: dict[URIRef, set[URIRef]]

    def __init__(self) -> None:
        self.fluent_timelines = {}
        self.event_timelines = {}
        self._fluent_event_registry = {}

    def _register_fluent_event(self, evt_uri: URIRef | None, fc_id: URIRef) -> None:
        if evt_uri is None:
            return

        if evt_uri not in self._fluent_event_registry:
            self._fluent_event_registry[evt_uri] = {fc_id}
            return

        self._fluent_event_registry[evt_uri].add(fc_id)

    def load_fluent_obs(self, fc: FluentClauseModel, graph: Graph):
        if fc.id not in self.fluent_timelines:
            f_tl = FluentTimeline(fc=fc)
            self.fluent_timelines[fc.id] = f_tl
            self._register_fluent_event(evt_uri=f_tl.start_event, fc_id=fc.id)
            self._register_fluent_event(evt_uri=f_tl.end_event, fc_id=fc.id)

        assert (
            URI_ROS_TYPE_TOPIC in fc.types
        ), "currently only support observation policy from trinary ROS topics"
        load_ros_topic_model(model=fc, graph=graph)

    def update_fpolicy_assertion(self, fc_uri: URIRef, trin_msg: TrinaryStampedMsg):
        assert fc_uri in self.fluent_timelines, f"No Timeline created for '{fc_uri}'"
        self.fluent_timelines[fc_uri].add_trinary(from_trin_stamped_msg(trin_msg))

    def on_event(self, evt_msg: Event):
        evt_uri = URIRef(evt_msg.uri)
        evt_t = Time.from_msg(evt_msg.stamp)
        if evt_uri not in self.event_timelines:
            self.event_timelines[evt_uri] = [evt_t]
        else:
            insert_timestamp_in_order(
                stamp_list=self.event_timelines[evt_uri], new_stamp=evt_t
            )

        if evt_uri not in self._fluent_event_registry:
            return

        for fc_uri in self._fluent_event_registry[evt_uri]:
            assert fc_uri in self.fluent_timelines
            self.fluent_timelines[fc_uri].on_event(
                evt_uri=evt_uri, evt_stamp=evt_t.to_datetime().timestamp()
            )

    def reset(self) -> None:
        self.fluent_timelines.clear()
        self.event_timelines.clear()
        self._fluent_event_registry.clear()
