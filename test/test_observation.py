from types import SimpleNamespace
from unittest.mock import Mock

from bdd_dsl.models.observation import ObservationStamped
from bdd_dsl.models.urirefs import (
    URI_BHV_PRED_TARGET_AGN,
    URI_BHV_PRED_TARGET_OBJ,
    URI_BHV_PRED_TARGET_WS,
    URI_BHV_TYPE_PLACE,
)
from rdflib import URIRef
from vision_msgs.msg import Detection3D

from bdd_exec_ros2.executables.mockup_behaviour_node import (
    MOCKUP_DETECTION3D_ENTITY_URIS,
    MockupBhvNode,
    map_detection3d_entity_mockup,
)
from bdd_exec_ros2.observation import (
    detection3d_stamp,
    map_detection3d_entity_by_dict,
    poses_are_collocated,
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


def test_pose_evaluator_requires_collocated_observations():
    left = Detection3D().bbox.center
    right = Detection3D().bbox.center
    right.position.x = 0.009
    observations = [
        ObservationStamped(URIRef("urn:test:left"), URIRef("urn:test:p"), 1.0, left),
        ObservationStamped(URIRef("urn:test:right"), URIRef("urn:test:p"), 1.0, right),
    ]

    assert poses_are_collocated(observations)
    right.position.x = 0.011
    assert not poses_are_collocated(observations)


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

    assert {message.param_rel_uri for message in messages} == {
        URI_BHV_PRED_TARGET_OBJ.toPython(),
        URI_BHV_PRED_TARGET_AGN.toPython(),
        URI_BHV_PRED_TARGET_WS.toPython(),
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
