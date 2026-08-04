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

from collections.abc import Mapping

from bdd_dsl.models.observation import EntityObservation, ObservationStamped
from bdd_dsl.models.urirefs import (
    URI_ROS_PRED_CHNL_NAME,
    URI_ROS_PRED_TYPE_NAME,
    URI_ROS_TYPE_ACTION,
    URI_ROS_TYPE_TOPIC,
)
from rclpy.time import Time
from rdf_utils.models.common import ModelBase
from rdflib import Graph, Literal, URIRef
from rosidl_runtime_py.utilities import get_action, get_message
from vision_msgs.msg import Detection3D

from bdd_exec_ros2.conversions import ros_time_to_stamp


def _load_ros_comm_specs(graph: Graph, model: ModelBase) -> tuple[str, str]:
    topic_name = graph.value(
        subject=model.id, predicate=URI_ROS_PRED_CHNL_NAME, any=False
    )
    if not isinstance(topic_name, Literal):
        raise TypeError(f"'channel-name' of '{model.id}' not a Literal: {topic_name}")
    msg_type_str = graph.value(
        subject=model.id, predicate=URI_ROS_PRED_TYPE_NAME, any=False
    )
    if not isinstance(msg_type_str, Literal):
        raise TypeError(f"'type-name' of '{model.id}' not a Literal: {msg_type_str}")
    return topic_name.toPython(), msg_type_str.toPython()


def load_ros_action_model(graph: Graph, model: ModelBase, **kwargs):
    if URI_ROS_TYPE_ACTION not in model.types:
        return

    action_name, action_type_str = _load_ros_comm_specs(graph=graph, model=model)
    model.set_attr(key=URI_ROS_PRED_CHNL_NAME, val=action_name)

    action_type = get_action(action_type_str)
    model.set_attr(key=URI_ROS_PRED_TYPE_NAME, val=action_type)


def load_ros_topic_model(graph: Graph, model: ModelBase, **kwargs):
    if URI_ROS_TYPE_TOPIC not in model.types:
        return

    topic_name, msg_type_str = _load_ros_comm_specs(graph=graph, model=model)
    model.set_attr(key=URI_ROS_PRED_CHNL_NAME, val=topic_name)

    msg_type = get_message(msg_type_str)
    model.set_attr(key=URI_ROS_PRED_TYPE_NAME, val=msg_type)


def detection3d_stamp(observation: Detection3D, _receipt_stamp: float) -> float:
    return ros_time_to_stamp(Time.from_msg(observation.header.stamp))


def map_detection3d_entity_by_dict(
    observation: Detection3D, entity_by_id: Mapping[str, URIRef]
) -> list[EntityObservation]:
    entity_uri = entity_by_id.get(observation.id)
    if entity_uri is None:
        return []
    return [EntityObservation(entity_uri, observation.bbox.center)]


def poses_are_collocated(observations: list[ObservationStamped]) -> bool:
    if len(observations) != 2:
        raise ValueError(f"expected two pose observations, got {len(observations)}")
    left, right = (sample.value.position for sample in observations)
    return (left.x - right.x) ** 2 + (left.y - right.y) ** 2 + (
        left.z - right.z
    ) ** 2 <= 0.01**2
