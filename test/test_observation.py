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

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from bdd_dsl.models.observation import ObservationStamped
from bdd_dsl.models.urirefs import (
    URI_BHV_PRED_TARGET_AGN,
    URI_BHV_PRED_TARGET_OBJ,
    URI_BHV_PRED_TARGET_WS,
    URI_BHV_TYPE_PLACE,
)
from bdd_ros2_interfaces.msg import Collision
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.time import Time
from rdflib import URIRef
from trinary import Unknown
from vision_msgs.msg import Detection3D, Detection3DArray

from bdd_exec_ros2.conversions import ros_time_to_stamp
from bdd_exec_ros2.executables.mockup_behaviour_node import (
    MOCKUP_DETECTION3D_ENTITY_URIS,
    MockupBhvNode,
    map_detection3d_entity_mockup,
)
from bdd_exec_ros2.observation import (
    DetectedEntityPose,
    PlanarContainmentEvaluator,
    TargetsDoNotCollideEvaluator,
    WrenchForceNormWithinLimitEvaluator,
    WrenchPeakForceNormWithinLimitEvaluator,
    WrenchRmsForceNormWithinLimitEvaluator,
    collision_stamp,
    header_stamp,
    latest_identified_pose_stamp,
    map_detection3d_array_by_uri,
    map_detection3d_entity_by_dict,
    map_detection3d_pose_array_by_uri,
    map_identified_pose_batch,
)


def test_detection3d_adapter_extracts_stamp_and_mapped_position():
    detection = Detection3D()
    detection.header.stamp.sec = 3
    detection.header.stamp.nanosec = 500_000_000
    detection.id = "tomato_soup_can"
    detection.bbox.center.position.x = 0.2

    mapped = map_detection3d_entity_mockup(detection)

    assert header_stamp(detection, 99.0) == 3.5
    assert mapped[0].entity_uri == MOCKUP_DETECTION3D_ENTITY_URIS[detection.id]
    assert mapped[0].value == (0.2, 0.0, 0.0)
    detection.id = "unmapped"
    assert (
        map_detection3d_entity_by_dict(detection, MOCKUP_DETECTION3D_ENTITY_URIS) == []
    )


def test_detection3d_array_adapter_maps_target_uris_and_bbox_centers():
    target = URIRef("urn:test:target")
    detections = Detection3DArray()
    detections.header.stamp.sec = 3
    detections.header.stamp.nanosec = 500_000_000

    for entity_id, x in (
        (str(target), 1.0),
        ("urn:test:unrelated", 2.0),
        ("", 3.0),
    ):
        detection = Detection3D(id=entity_id)
        detection.bbox.center.position.x = x
        detections.detections.append(detection)

    mapped = map_detection3d_array_by_uri(detections, targets=[target])

    assert header_stamp(detections, 42.0) == 3.5
    assert [(item.entity_uri, item.value) for item in mapped] == [
        (target, (1.0, 0.0, 0.0))
    ]
    detections.header.stamp = Time().to_msg()
    assert header_stamp(detections, 42.0) == 42.0


def test_planar_containment_uses_full_detection_poses_and_inclusive_margin():
    drawer = URIRef("urn:test:drawer")
    workspace = URIRef("urn:test:workspace")
    detections = Detection3DArray()
    for entity in (drawer, workspace):
        detection = Detection3D(id=str(entity))
        detection.bbox.center.orientation.w = 1.0
        detections.detections.append(detection)
    detections.detections[0].bbox.center.position.x = 0.640

    samples = [
        ObservationStamped(
            URIRef(f"urn:test:observation:{index}"),
            URIRef("urn:test:provider"),
            1.0,
            mapped.value,
        )
        for index, mapped in enumerate(
            map_detection3d_pose_array_by_uri(detections, targets=[drawer, workspace])
        )
    ]
    assert all(isinstance(sample.value, DetectedEntityPose) for sample in samples)

    center = PlanarContainmentEvaluator(drawer, workspace, (1.6, 0.8))
    footprint = PlanarContainmentEvaluator(
        drawer,
        workspace,
        (1.6, 0.8),
        footprint_size_xy=(0.24, 0.20),
        allowed_outside_ratio=0.05,
    )
    assert center.evaluate(samples)[0] is True
    assert footprint.evaluate(samples)[0] is True

    detections.detections[0].bbox.center.position.x = 0.643
    moved = map_detection3d_pose_array_by_uri(detections, targets=[drawer, workspace])
    samples = [
        ObservationStamped(
            sample.observation_uri, sample.provider_uri, 2.0, value.value
        )
        for sample, value in zip(samples, moved, strict=True)
    ]
    result, reason = footprint.evaluate(samples)
    assert result is False
    assert "outside ratio" in reason
    assert "5.000% limit" in reason

    half_sqrt_two = 2**-0.5
    for detection in detections.detections:
        detection.bbox.center.orientation.z = half_sqrt_two
        detection.bbox.center.orientation.w = half_sqrt_two
    detections.detections[0].bbox.center.position.x = 0.0
    detections.detections[0].bbox.center.position.y = 0.640
    rotated = map_detection3d_pose_array_by_uri(detections, targets=[drawer, workspace])
    samples = [
        ObservationStamped(
            sample.observation_uri, sample.provider_uri, 3.0, value.value
        )
        for sample, value in zip(samples, rotated, strict=True)
    ]
    assert footprint.evaluate(samples)[0] is True


def test_wrench_force_norm_evaluator_uses_header_and_inclusive_limit():
    message = WrenchStamped()
    message.header.stamp.sec = 3
    message.header.stamp.nanosec = 500_000_000
    message.wrench.force.x = 3.0
    message.wrench.force.y = -4.0
    evaluator = WrenchForceNormWithinLimitEvaluator(max_force_n=5.0)
    sample = ObservationStamped(
        URIRef("urn:test:wrench-observation"),
        URIRef("urn:test:wrench-provider"),
        3.5,
        message,
    )

    assert header_stamp(message, 42.0) == 3.5
    assert evaluator.evaluate([sample])[0] is True

    message.header.stamp = Time().to_msg()
    message.wrench.force.x = -6.0
    message.wrench.force.y = -8.0
    result, reason = evaluator.evaluate([sample])
    assert header_stamp(message, 42.0) == 42.0
    assert result is False
    assert "10.000 N" in reason and "5.000 N limit" in reason
    assert evaluator.evaluate([]) == evaluator.default_result

    with pytest.raises(ValueError, match="exactly one"):
        evaluator.evaluate([sample, sample])
    with pytest.raises(TypeError, match="WrenchStamped"):
        evaluator.evaluate(
            [
                ObservationStamped(
                    sample.observation_uri,
                    sample.provider_uri,
                    sample.stamp,
                    object(),
                )
            ]
        )
    with pytest.raises(TypeError, match="std_msgs/Header"):
        header_stamp(object(), 42.0)


def test_peak_and_rms_force_evaluators_accumulate_and_warm_up():
    message = WrenchStamped()
    observation_uri = URIRef("urn:test:wrench-observation")
    provider_uri = URIRef("urn:test:wrench-provider")

    def sample(stamp: float, force: float) -> ObservationStamped:
        message.wrench.force.x = force
        return ObservationStamped(observation_uri, provider_uri, stamp, message)

    peak = WrenchPeakForceNormWithinLimitEvaluator()
    assert peak.evaluate([sample(0.0, 10.0)])[0] is True
    assert peak.evaluate([sample(0.1, 50.0)])[0] is False

    rms = WrenchRmsForceNormWithinLimitEvaluator()
    assert rms.evaluate([sample(0.0, 10.0)])[0] is Unknown
    assert rms.evaluate([sample(0.25, 10.0)])[0] is True
    assert rms.evaluate([sample(0.26, 20.0)])[0] is False


def test_simulation_snapshot_adapter_extracts_stamp_and_mapped_poses():
    first = URIRef("urn:test:first")
    second = URIRef("urn:test:second")
    poses = {first: PoseStamped(), second: PoseStamped()}
    source_time = Time(seconds=7.0)
    poses[first].header.stamp = source_time.to_msg()
    receipt_stamp = 42.0

    mapped = map_identified_pose_batch(poses)

    assert latest_identified_pose_stamp(poses, receipt_stamp) == ros_time_to_stamp(
        source_time
    )
    assert {observation.entity_uri: observation.value for observation in mapped} == {
        first: (0.0, 0.0, 0.0),
        second: (0.0, 0.0, 0.0),
    }
    poses[first].header.stamp = Time().to_msg()
    assert latest_identified_pose_stamp(poses, receipt_stamp) == receipt_stamp


def test_collision_evaluator_matches_one_current_group():
    msg = Collision()
    msg.stamp.sec = 4
    msg.bodies = ["object", "workspace"]
    assert collision_stamp(msg, 99.0) == 4.0

    group = frozenset(msg.bodies)
    observations = [
        ObservationStamped(
            URIRef("urn:test:object-observation"),
            URIRef("urn:test:collision-provider"),
            4.0,
            group,
        ),
        ObservationStamped(
            URIRef("urn:test:workspace-observation"),
            URIRef("urn:test:collision-provider"),
            4.0,
            group,
        ),
    ]
    result, reason = TargetsDoNotCollideEvaluator().evaluate(observations)
    assert result is False
    assert "object" in reason and "workspace" in reason

    observations.append(
        ObservationStamped(
            URIRef("urn:test:third-target"),
            URIRef("urn:test:collision-provider"),
            4.0,
            group,
        )
    )
    result, reason = TargetsDoNotCollideEvaluator().evaluate(observations)
    assert result is False
    assert all(body in reason for body in ("object", "workspace"))

    msg.bodies = []
    clear_group = frozenset()
    observations = [
        ObservationStamped(
            observation.observation_uri,
            observation.provider_uri,
            observation.stamp,
            clear_group,
        )
        for observation in observations
    ]
    result, reason = TargetsDoNotCollideEvaluator().evaluate(observations)
    assert result is True
    assert "no active collision" in reason


def test_collision_evaluator_defaults_to_no_collision():
    evaluator = TargetsDoNotCollideEvaluator()
    assert evaluator.evaluate([]) == (True, "no collision recorded")


def test_place_behaviour_forwards_workspace_parameter():
    from bdd_exec_ros2.conversions import get_bhv_param_messages

    object_var = URIRef("urn:test:object-var")
    agent_var = URIRef("urn:test:agent-var")
    workspace_var = URIRef("urn:test:workspace-var")
    when_bhv = SimpleNamespace(
        behaviour=SimpleNamespace(types={URI_BHV_TYPE_PLACE}),
        get_attr=lambda key: {
            URI_BHV_PRED_TARGET_OBJ: object_var,
            URI_BHV_PRED_TARGET_AGN: agent_var,
            URI_BHV_PRED_TARGET_WS: workspace_var,
        }[key],
    )
    values = {
        object_var: URIRef("urn:test:object"),
        agent_var: URIRef("urn:test:agent"),
        workspace_var: URIRef("urn:test:workspace"),
    }

    messages = get_bhv_param_messages(when_bhv, values)

    assert {message.variable_uri for message in messages} == {
        object_var.toPython(),
        agent_var.toPython(),
        workspace_var.toPython(),
    }


def test_mockup_publishes_a_stamped_detection_for_a_mapped_entity():
    stamp = object()
    publisher = Mock()
    logger = Mock()
    node = SimpleNamespace(
        get_clock=Mock(
            return_value=SimpleNamespace(
                now=Mock(return_value=SimpleNamespace(to_msg=Mock(return_value=stamp)))
            )
        ),
        detections_pub=publisher,
        get_logger=Mock(return_value=logger),
    )

    MockupBhvNode._publish_detection(
        node, MOCKUP_DETECTION3D_ENTITY_URIS["tomato_soup_can"]
    )

    detection = publisher.publish.call_args.args[0]
    assert detection.header.stamp is stamp
    assert detection.id == "tomato_soup_can"
    assert detection.bbox.center.orientation.w == 1.0

    MockupBhvNode._publish_detection(node, URIRef("urn:test:unknown"))
    logger.warning.assert_called_once()
