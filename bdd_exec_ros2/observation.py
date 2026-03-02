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

from rosidl_runtime_py.utilities import get_message
from rdflib import Graph, Literal
from rdf_utils.models.common import ModelBase
from bdd_exec_ros2.urirefs import (
    URI_ROS_PRED_MSG_TYPE,
    URI_ROS_PRED_TOPIC_NAME,
    URI_ROS_TYPE_TOPIC,
)


def load_ros_topic_model(graph: Graph, model: ModelBase, **kwargs):
    if URI_ROS_TYPE_TOPIC not in model.types:
        return

    topic_name = graph.value(
        subject=model.id, predicate=URI_ROS_PRED_TOPIC_NAME, any=False
    )
    assert isinstance(topic_name, Literal), (
        f"'topic_name' of '{model.id}' not a Literal: {topic_name}"
    )
    model.set_attr(key=URI_ROS_PRED_TOPIC_NAME, val=topic_name.toPython())

    msg_type_str = graph.value(
        subject=model.id, predicate=URI_ROS_PRED_MSG_TYPE, any=False
    )
    assert isinstance(msg_type_str, Literal), (
        f"'message-type' of '{model.id}' not a Literal: {msg_type_str}"
    )

    msg_type = get_message(msg_type_str)
    model.set_attr(key=URI_ROS_PRED_MSG_TYPE, val=msg_type)
