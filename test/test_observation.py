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

from bdd_dsl.models.observation import ObservationStamped
from bdd_dsl.models.urirefs import (
    URI_BHV_PRED_TARGET_AGN,
    URI_BHV_PRED_TARGET_OBJ,
    URI_BHV_PRED_TARGET_WS,
    URI_BHV_TYPE_PLACE,
)
from bdd_ros2_interfaces.msg import Collision
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time
from rdflib import URIRef
from vision_msgs.msg import Detection3D

from bdd_exec_ros2.conversions import ros_time_to_stamp
from bdd_exec_ros2.executables.mockup_behaviour_node import (
    MOCKUP_DETECTION3D_ENTITY_URIS,
    MockupBhvNode,
    map_detection3d_entity_mockup,
)
from bdd_exec_ros2.observation import (
    PosesAreCollocatedEvaluator,
    TargetsDoNotCollideEvaluator,
    collision_stamp,
    detection3d_stamp,
    map_detection3d_entity_by_dict,
    map_simulation_pose_snapshot,
    simulation_pose_snapshot_stamp,
)


def test_detection3d_adapter_extracts_stamp_and_mapped_pose():
    detection = Detection3D()
    detection.header.stamp.sec = 3
    detection.header.stamp.nanosec = 500_000_000
    detection.id = "tomato_soup_can"
    detection.bbox.center.position.x = 0.2

    mapped = map_detection3d_entity_mockup(detection)

    assert detection3d_stamp(detection, 99.0) == 3.5
    assert mapped[0].entity_uri == MOCKUP_DETECTION3D_ENTITY_URIS[detection.id]
    assert mapped[0].value.position.x == 0.2
    detection.id = "unmapped"
    assert (
        map_detection3d_entity_by_dict(detection, MOCKUP_DETECTION3D_ENTITY_URIS) == []
    )


def test_simulation_snapshot_adapter_extracts_stamp_and_mapped_poses():
    first = URIRef("urn:test:first")
    second = URIRef("urn:test:second")
    poses = {first: PoseStamped(), second: PoseStamped()}
    source_time = Time(seconds=7.0)
    poses[first].header.stamp = source_time.to_msg()
    receipt_stamp = 42.0

    mapped = map_simulation_pose_snapshot(poses)

    assert simulation_pose_snapshot_stamp(poses, receipt_stamp) == ros_time_to_stamp(
        source_time
    )
    assert {observation.entity_uri: observation.value for observation in mapped} == {
        entity_uri: pose.pose for entity_uri, pose in poses.items()
    }
    poses[first].header.stamp = Time().to_msg()
    assert simulation_pose_snapshot_stamp(poses, receipt_stamp) == receipt_stamp


def test_pose_evaluator_requires_collocated_observations():
    left = Detection3D().bbox.center
    right = Detection3D().bbox.center
    right.position.x = 0.009
    observations = [
        ObservationStamped(URIRef("urn:test:left"), URIRef("urn:test:p"), 1.0, left),
        ObservationStamped(URIRef("urn:test:right"), URIRef("urn:test:p"), 1.0, right),
    ]

    result, reason = PosesAreCollocatedEvaluator().evaluate(observations)
    assert result
    assert "within collocation threshold" in reason
    right.position.x = 0.011
    result, reason = PosesAreCollocatedEvaluator().evaluate(observations)
    assert not result
    assert "exceeds collocation threshold" in reason


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
