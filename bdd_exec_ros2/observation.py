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
from math import hypot

from bdd_dsl.models.observation import (
    EntityObservation,
    ObservationPolicyEvaluator,
    ObservationStamped,
)
from bdd_dsl.models.urirefs import (
    URI_ROS_PRED_CHNL_NAME,
    URI_ROS_PRED_TYPE_NAME,
    URI_ROS_TYPE_ACTION,
    URI_ROS_TYPE_TOPIC,
)
from bdd_ros2_interfaces.msg import Collision
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.time import Time
from rdf_utils.models.common import ModelBase
from rdflib import Graph, Literal, URIRef
from rosidl_runtime_py.utilities import get_action, get_message
from scene_dsl.rdf_parser.kinematics import get_kinematic_mappings
from scene_dsl.rdf_parser.scenex import SceneInstanceModel
from std_msgs.msg import Header
from vision_msgs.msg import Detection3D, Detection3DArray

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


def map_detection3d_entity_by_dict(
    observation: Detection3D, entity_by_id: Mapping[str, URIRef]
) -> list[EntityObservation]:
    entity_uri = entity_by_id.get(observation.id)
    if entity_uri is None:
        return []
    position = observation.bbox.center.position
    return [EntityObservation(entity_uri, (position.x, position.y, position.z))]


def header_stamp(observation: object, receipt_stamp: float) -> float:
    header = getattr(observation, "header", None)
    if not isinstance(header, Header):
        raise TypeError(
            f"header_stamp expects a ROS message with a std_msgs/Header, got: {observation}"
        )
    stamp = ros_time_to_stamp(Time.from_msg(header.stamp))
    return stamp or receipt_stamp


def map_detection3d_array_by_uri(
    observation: Detection3DArray,
    scene_instance: SceneInstanceModel | None = None,
    targets: list[URIRef] | None = None,
) -> list[EntityObservation]:
    del scene_instance

    target_set = set(targets) if targets is not None else None
    mapped = []
    for detection in observation.detections:
        entity_uri = URIRef(detection.id) if detection.id else None
        if (
            entity_uri is None
            or not detection.results
            or (target_set is not None and entity_uri not in target_set)
        ):
            continue
        position = detection.results[0].pose.pose.position
        mapped.append(
            EntityObservation(entity_uri, (position.x, position.y, position.z))
        )
    return mapped


def latest_identified_pose_stamp(
    poses: Mapping[URIRef, PoseStamped], receipt_stamp: float
) -> float:
    source_stamps = (
        ros_time_to_stamp(Time.from_msg(pose.header.stamp)) for pose in poses.values()
    )
    return max((stamp for stamp in source_stamps if stamp), default=receipt_stamp)


def map_identified_pose_batch(
    poses: Mapping[URIRef, PoseStamped],
    scene_instance: SceneInstanceModel | None = None,
    targets: list[URIRef] | None = None,
) -> list[EntityObservation]:
    return [
        EntityObservation(
            entity_uri,
            (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
        )
        for entity_uri, pose in poses.items()
    ]


def collision_stamp(observation: Collision, receipt_stamp: float) -> float:
    stamp = ros_time_to_stamp(Time.from_msg(observation.stamp))
    return stamp or receipt_stamp


def collision_target_mapper(
    observation: Collision,
    scene_inst: SceneInstanceModel,
    targets: list[URIRef],
) -> list[EntityObservation]:
    group = frozenset(observation.bodies)
    mapped: list[EntityObservation] = []
    for target_uri in targets:
        element_uri = scene_inst.resolve_modelled_element_id(target_uri)
        if element_uri is None:
            mapped.append(EntityObservation(target_uri, group))
            continue
        models = scene_inst.object_models.get(
            element_uri
        ) or scene_inst.agent_models.get(element_uri)
        if not models:
            mapped.append(EntityObservation(target_uri, group))
            continue
        entities = frozenset(
            mapping.entity
            for model in models.values()
            for mapping in get_kinematic_mappings(model)
            if mapping.entity is not None
        )
        mapped.append(
            EntityObservation(target_uri, group & entities if entities else group)
        )
    return mapped


class TargetsDoNotCollideEvaluator(ObservationPolicyEvaluator):
    def __init__(self) -> None:
        super().__init__((True, "no collision recorded"))

    def _evaluate_samples(
        self, observations: list[ObservationStamped]
    ) -> tuple[bool, str]:
        if not observations:
            raise ValueError("expected at least one collision observation")
        collision_sets = [sample.value for sample in observations]
        if not all(isinstance(value, (set, frozenset)) for value in collision_sets):
            raise TypeError("collision evaluator expects sets of colliding bodies")
        affected_bodies = set().union(*collision_sets)
        if all(collision_sets):
            return False, f"collision affects {sorted(affected_bodies)}"
        if not affected_bodies:
            return True, "no active collision for any target"
        return True, "collision does not affect all targets"


class WrenchForceNormWithinLimitEvaluator(ObservationPolicyEvaluator):
    def __init__(self, max_force_n: float = 15.0) -> None:
        super().__init__()
        self.max_force_n = max_force_n

    def _evaluate_samples(
        self, observations: list[ObservationStamped]
    ) -> tuple[bool, str]:
        if len(observations) != 1:
            raise ValueError("wrench evaluator expects exactly one observation")
        message = observations[0].value
        if not isinstance(message, WrenchStamped):
            raise TypeError("wrench evaluator expects a WrenchStamped value")
        force = message.wrench.force
        norm = hypot(force.x, force.y, force.z)
        within_limit = norm <= self.max_force_n
        relation = "within" if within_limit else "exceeds"
        return (
            within_limit,
            f"force norm {norm:.3f} N {relation} {self.max_force_n:.3f} N limit",
        )
