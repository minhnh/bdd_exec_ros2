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

from collections import deque
from collections.abc import Mapping
from dataclasses import dataclass
from math import hypot, isfinite, sqrt

from bdd_dsl.models.observation import (
    EntityObservation,
    ObservationPolicyEvaluator,
    ObservationStamped,
)
from bdd_ros2_interfaces.msg import Collision
from geometry_msgs.msg import Pose, PoseStamped, WrenchStamped
from rclpy.time import Time
from rdf_utils.models.common import ModelBase
from rdf_utils.models.vocab import (
    URI_ROS_PRED_CHNL_NAME,
    URI_ROS_PRED_TYPE_NAME,
    URI_ROS_TYPE_ACTION,
    URI_ROS_TYPE_TOPIC,
)
from rdflib import Graph, Literal, URIRef
from rosidl_runtime_py.utilities import get_action, get_message
from scene_dsl.rdf_parser.kinematics import get_kinematic_mappings
from scene_dsl.rdf_parser.scenex import SceneInstanceModel
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header
from trinary import Trinary, Unknown
from vision_msgs.msg import Detection3D, Detection3DArray

from bdd_exec_ros2.conversions import ros_time_to_stamp


@dataclass
class DetectedEntityPose:
    entity_uri: URIRef
    pose: Pose

    def __iter__(self):
        """Keep compatibility with bdd-dsl distance evaluators.

        The evaluator in bdd-dsl passes values to math.dist(), which expects an Iterable.
        """
        position = self.pose.position
        return iter((position.x, position.y, position.z))


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
    return [
        EntityObservation(
            entity_uri, DetectedEntityPose(entity_uri, detection.bbox.center)
        )
        for detection in observation.detections
        if detection.id
        and (entity_uri := URIRef(detection.id))
        and (target_set is None or entity_uri in target_set)
    ]


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
        norm = _wrench_force_norm(observations)
        within_limit = norm <= self.max_force_n
        relation = "within" if within_limit else "exceeds"
        return (
            within_limit,
            f"force norm {norm:.3f} N {relation} {self.max_force_n:.3f} N limit",
        )


def _wrench_force_norm(observations: list[ObservationStamped]) -> float:
    if len(observations) != 1:
        raise ValueError("wrench evaluator expects exactly one observation")
    message = observations[0].value
    if not isinstance(message, WrenchStamped):
        raise TypeError("wrench evaluator expects a WrenchStamped value")
    force = message.wrench.force
    return hypot(force.x, force.y, force.z)


class WrenchPeakForceNormWithinLimitEvaluator(ObservationPolicyEvaluator):
    def __init__(self, max_force_n: float = 45.0) -> None:
        super().__init__()
        self.max_force_n = max_force_n
        self.peak_force_n: float | None = None

    def _evaluate_samples(
        self, observations: list[ObservationStamped]
    ) -> tuple[bool, str]:
        norm = _wrench_force_norm(observations)
        self.peak_force_n = max(norm, self.peak_force_n or 0.0)
        within_limit = self.peak_force_n <= self.max_force_n
        relation = "within" if within_limit else "exceeds"
        return (
            within_limit,
            f"peak force norm {self.peak_force_n:.3f} N {relation} {self.max_force_n:.3f} N limit",
        )


class WrenchRmsForceNormWithinLimitEvaluator(ObservationPolicyEvaluator):
    def __init__(self, max_force_n: float = 15.0, window_seconds: float = 0.25) -> None:
        super().__init__()
        if window_seconds <= 0.0:
            raise ValueError("RMS window must be positive")
        self.max_force_n = max_force_n
        self.window_seconds = window_seconds
        self._first_stamp: float | None = None
        self._samples: deque[tuple[float, float]] = deque()

    def _evaluate_samples(
        self, observations: list[ObservationStamped]
    ) -> tuple[bool | Trinary, str]:
        norm = _wrench_force_norm(observations)
        stamp = observations[0].stamp
        if self._first_stamp is None:
            self._first_stamp = stamp
        if self._samples and self._samples[-1][0] == stamp:
            self._samples[-1] = (stamp, norm * norm)
        else:
            self._samples.append((stamp, norm * norm))
        cutoff = stamp - self.window_seconds
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.popleft()
        if stamp - self._first_stamp < self.window_seconds:
            return (
                Unknown,
                f"RMS force window warming up ({stamp - self._first_stamp:.3f} s)",
            )

        rms = sqrt(sum(value for _, value in self._samples) / len(self._samples))
        within_limit = rms <= self.max_force_n
        relation = "within" if within_limit else "exceeds"
        return (
            within_limit,
            f"RMS force norm {rms:.3f} N {relation} {self.max_force_n:.3f} N limit over {self.window_seconds:.3f} s",
        )


class PlanarContainmentEvaluator(ObservationPolicyEvaluator):
    def __init__(
        self,
        object_uri: URIRef,
        boundary_uri: URIRef,
        boundary_size_xy: tuple[float, float],
        margin_m: float = 0.05,
        footprint_size_xy: tuple[float, float] | None = None,
        allowed_outside_ratio: float = 0.0,
    ) -> None:
        super().__init__()
        if any(size <= 0.0 for size in boundary_size_xy):
            raise ValueError("boundary dimensions must be positive")
        if margin_m < 0.0 or any(2.0 * margin_m >= size for size in boundary_size_xy):
            raise ValueError("margin must leave a positive boundary")
        if footprint_size_xy is not None and any(
            size <= 0.0 for size in footprint_size_xy
        ):
            raise ValueError("footprint dimensions must be positive")
        if not 0.0 <= allowed_outside_ratio <= 1.0:
            raise ValueError("allowed outside ratio must be between 0 and 1")
        self.object_uri = object_uri
        self.boundary_uri = boundary_uri
        self.boundary_size_xy = boundary_size_xy
        self.margin_m = margin_m
        self.footprint_size_xy = footprint_size_xy
        self.allowed_outside_ratio = allowed_outside_ratio

    @staticmethod
    def _rotation(pose: Pose) -> Rotation:
        quaternion = pose.orientation
        values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        if (
            not all(isfinite(value) for value in values)
            or sum(value * value for value in values) == 0.0
        ):
            raise ValueError("quaternion must be finite and non-zero")
        return Rotation.from_quat(values)

    def _evaluate_samples(
        self, observations: list[ObservationStamped]
    ) -> tuple[bool | Trinary, str]:
        if len(observations) != 2:
            raise ValueError("containment evaluator expects exactly two observations")
        values = [sample.value for sample in observations]
        if not all(isinstance(value, DetectedEntityPose) for value in values):
            raise TypeError("containment evaluator expects detected entity poses")
        poses = {value.entity_uri: value.pose for value in values}
        if set(poses) != {self.object_uri, self.boundary_uri}:
            raise ValueError("containment evaluator received unexpected entities")

        object_pose = poses[self.object_uri]
        boundary_pose = poses[self.boundary_uri]
        positions = (object_pose.position, boundary_pose.position)
        if not all(
            isfinite(component)
            for position in positions
            for component in (position.x, position.y, position.z)
        ):
            return Unknown, "containment pose has a non-finite position"
        try:
            boundary_inverse = self._rotation(boundary_pose).inv()
            object_rotation = (
                self._rotation(object_pose)
                if self.footprint_size_xy is not None
                else None
            )
        except ValueError as exc:
            return Unknown, f"invalid containment pose: {exc}"

        offset = [
            object_pose.position.x - boundary_pose.position.x,
            object_pose.position.y - boundary_pose.position.y,
            object_pose.position.z - boundary_pose.position.z,
        ]
        boundary_half_x = self.boundary_size_xy[0] / 2.0 - self.margin_m
        boundary_half_y = self.boundary_size_xy[1] / 2.0 - self.margin_m
        center = boundary_inverse.apply(offset)
        if self.footprint_size_xy is None:
            inside = bool(
                abs(center[0]) <= boundary_half_x + 1e-9
                and abs(center[1]) <= boundary_half_y + 1e-9
            )
            relation = "inside" if inside else "outside"
            return inside, f"center {relation} boundary"

        try:
            from shapely.geometry import Polygon, box
        except ImportError as exc:
            raise RuntimeError(
                "footprint containment requires the shapely package"
            ) from exc

        assert object_rotation is not None
        half_x, half_y = (size / 2.0 for size in self.footprint_size_xy)
        corners = (
            (-half_x, -half_y),
            (half_x, -half_y),
            (half_x, half_y),
            (-half_x, half_y),
        )
        footprint = Polygon(
            [
                tuple(
                    boundary_inverse.apply(object_rotation.apply([x, y, 0.0]) + offset)[
                        :2
                    ]
                )
                for x, y in corners
            ]
        )
        boundary = box(
            -boundary_half_x, -boundary_half_y, boundary_half_x, boundary_half_y
        )
        outside_ratio = max(
            0.0, min(1.0, footprint.difference(boundary).area / footprint.area)
        )
        inside = outside_ratio <= self.allowed_outside_ratio + 1e-9
        relation = "within" if inside else "exceeds"
        return (
            inside,
            f"footprint outside ratio {outside_ratio * 100.0:.3f}% {relation} {self.allowed_outside_ratio * 100.0:.3f}% limit",
        )
