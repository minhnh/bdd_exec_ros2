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

from math import hypot
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
)
from bdd_exec_ros2.observation import (
    DetectedEntityPose,
    PlanarContainmentEvaluator,
    TargetsDoNotCollideEvaluator,
    WrenchForceNormWithinLimitEvaluator,
    WrenchRmsForceNormWithinLimitEvaluator,
    collision_stamp,
    header_stamp,
    latest_identified_pose_stamp,
    map_detection3d_array_by_uri,
    map_identified_entity_poses,
    map_identified_pose_batch,
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
    receipt_stamp = 42.0
    assert header_stamp(detections, receipt_stamp) == ros_time_to_stamp(
        Time.from_msg(detections.header.stamp)
    )
    assert [item.entity_uri for item in mapped] == [target]
    position = detections.detections[0].bbox.center.position
    assert list(mapped[0].value) == [position.x, position.y, position.z]
    detections.header.stamp = Time().to_msg()
    assert header_stamp(detections, receipt_stamp) == receipt_stamp


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
            map_detection3d_array_by_uri(detections, targets=[drawer, workspace])
        )
    ]
    assert all(isinstance(sample.value, DetectedEntityPose) for sample in samples)

    center = PlanarContainmentEvaluator(drawer, workspace, (1.6, 0.8))
    config = center.get_config()
    assert config == {
        "object_uri": str(drawer),
        "boundary_uri": str(workspace),
        "boundary_size_m": list(center.boundary_size_xy),
        "margin_m": center.margin_m,
        "footprint_size_m": None,
        "allowed_outside_ratio": center.allowed_outside_ratio,
    }
    config["boundary_size_m"].clear()
    assert center.get_config()["boundary_size_m"] == list(center.boundary_size_xy)
    footprint = PlanarContainmentEvaluator(
        drawer,
        workspace,
        (1.6, 0.8),
        footprint_size_xy=(0.24, 0.20),
        allowed_outside_ratio=0.05,
    )
    result, reason = center.evaluate(samples[:1])
    assert result is Unknown
    assert isinstance(reason["reason"], str) and reason["reason"]
    assert reason["observation_uris"] == [str(samples[0].observation_uri)]
    assert center.evaluate(samples)[0] is True
    assert footprint.evaluate(samples)[0] is True

    detections.detections[0].bbox.center.position.x = 0.643
    moved = map_detection3d_array_by_uri(detections, targets=[drawer, workspace])
    samples = [
        ObservationStamped(
            sample.observation_uri, sample.provider_uri, 2.0, value.value
        )
        for sample, value in zip(samples, moved, strict=True)
    ]
    result, reason = footprint.evaluate(samples)
    assert result is False
    assert isinstance(reason["reason"], str) and reason["reason"]
    assert reason["outside_ratio"] > footprint.get_config()["allowed_outside_ratio"]
    assert footprint.get_config().keys().isdisjoint(reason)

    half_sqrt_two = 2**-0.5
    for detection in detections.detections:
        detection.bbox.center.orientation.z = half_sqrt_two
        detection.bbox.center.orientation.w = half_sqrt_two
    detections.detections[0].bbox.center.position.x = 0.0
    detections.detections[0].bbox.center.position.y = 0.640
    rotated = map_detection3d_array_by_uri(detections, targets=[drawer, workspace])
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

    receipt_stamp = 42.0
    assert header_stamp(message, receipt_stamp) == sample.stamp
    result, details = evaluator.evaluate([sample])
    assert result is True
    assert details["force_norm_n"] == pytest.approx(
        evaluator.get_config()["max_force_n"]
    )

    message.header.stamp = Time().to_msg()
    message.wrench.force.x = -6.0
    message.wrench.force.y = -8.0
    result, reason = evaluator.evaluate([sample])
    assert header_stamp(message, receipt_stamp) == receipt_stamp
    assert result is False
    assert isinstance(reason["reason"], str) and reason["reason"]
    force = message.wrench.force
    assert reason["force_norm_n"] == pytest.approx(hypot(force.x, force.y, force.z))
    assert reason["force_norm_n"] > evaluator.get_config()["max_force_n"]
    assert reason["stamp"] == sample.stamp
    assert reason["observation_uri"] == str(sample.observation_uri)
    assert evaluator.get_config()["max_force_n"] == evaluator.max_force_n
    assert evaluator.get_config().keys().isdisjoint(reason)
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


def test_rms_force_evaluator_accumulates_and_warms_up():
    message = WrenchStamped()
    observation_uri = URIRef("urn:test:wrench-observation")
    provider_uri = URIRef("urn:test:wrench-provider")

    def sample(stamp: float, force: float) -> ObservationStamped:
        message.wrench.force.x = force
        return ObservationStamped(observation_uri, provider_uri, stamp, message)

    rms = WrenchRmsForceNormWithinLimitEvaluator()
    config = rms.get_config()
    assert config["max_force_n"] == rms.max_force_n
    assert config["window_seconds"] == rms.window_seconds
    below_limit = config["max_force_n"] / 2
    result, details = rms.evaluate([sample(0.0, below_limit)])
    assert result is None
    assert config.keys().isdisjoint(details)
    result, details = rms.evaluate([sample(config["window_seconds"], below_limit)])
    assert result is True
    assert details["rms_force_norm_n"] == pytest.approx(below_limit)
    result, details = rms.evaluate(
        [sample(2 * config["window_seconds"], 2 * config["max_force_n"])]
    )
    assert result is False
    assert config.keys().isdisjoint(details)
    assert details["rms_force_norm_n"] > config["max_force_n"]
    assert rms.get_config() == config


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
    poses[first].pose.orientation.x = 0.5
    full = map_identified_entity_poses(poses, targets=[first])
    assert [sample.entity_uri for sample in full] == [first]
    assert isinstance(full[0].value, DetectedEntityPose)
    assert full[0].value.pose == poses[first].pose
    poses[first].header.stamp = Time().to_msg()
    assert latest_identified_pose_stamp(poses, receipt_stamp) == receipt_stamp


def test_collision_evaluator_matches_one_current_group():
    msg = Collision()
    msg.stamp.sec = 4
    msg.bodies = ["object", "workspace"]
    assert collision_stamp(msg, 99.0) == ros_time_to_stamp(Time.from_msg(msg.stamp))

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
    assert isinstance(reason["reason"], str) and reason["reason"]
    assert len(reason["collisions_by_observation"]) == len(observations)
    assert all(set(bodies) == group for bodies in reason["collisions_by_observation"])

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
    assert all(set(bodies) == group for bodies in reason["collisions_by_observation"])

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
    assert all(not bodies for bodies in reason["collisions_by_observation"])


def test_collision_evaluator_defaults_to_no_collision():
    evaluator = TargetsDoNotCollideEvaluator()
    result, details = evaluator.evaluate([])
    assert result is True
    assert isinstance(details.get("reason"), str) and details["reason"]


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


def test_mockup_publishes_stamped_detection_batch():
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

    MockupBhvNode._publish_detections(
        node,
        [
            MOCKUP_DETECTION3D_ENTITY_URIS["tomato_soup_can"],
            URIRef("urn:test:unknown"),
        ],
    )

    detections = publisher.publish.call_args.args[0]
    assert detections.header.stamp is stamp
    assert len(detections.detections) == 1
    detection = detections.detections[0]
    assert detection.id == str(MOCKUP_DETECTION3D_ENTITY_URIS["tomato_soup_can"])
    assert detection.bbox.center.orientation.w == 1.0
    logger.warning.assert_called_once()
