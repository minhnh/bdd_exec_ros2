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

from rosidl_runtime_py.utilities import get_action, get_message
from rdflib import Graph, Literal
from rdf_utils.models.common import ModelBase
from bdd_exec_ros2.urirefs import (
    URI_ROS_PRED_TYPE_NAME,
    URI_ROS_PRED_CHNL_NAME,
    URI_ROS_TYPE_ACTION,
    URI_ROS_TYPE_TOPIC,
)


def _load_ros_comm_specs(graph: Graph, model: ModelBase) -> tuple[str, str]:
    topic_name = graph.value(
        subject=model.id, predicate=URI_ROS_PRED_CHNL_NAME, any=False
    )
    if not isinstance(topic_name, Literal):
        raise ValueError(f"'channel-name' of '{model.id}' not a Literal: {topic_name}")
    msg_type_str = graph.value(
        subject=model.id, predicate=URI_ROS_PRED_TYPE_NAME, any=False
    )
    if not isinstance(msg_type_str, Literal):
        raise ValueError(f"'type-name' of '{model.id}' not a Literal: {msg_type_str}")
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
