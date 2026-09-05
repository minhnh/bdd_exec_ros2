from unittest.mock import Mock

from bdd_dsl.models.urirefs import (
    URI_BDD_PRED_HAS_STRING_ENTITY_MAPPING,
    URI_BDD_PRED_MAPPED_ENTITY,
    URI_BDD_PRED_STRING_VALUE,
    URI_BDD_TYPE_STRING_ENTITY_MAPPING,
)
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from rdf_utils.models.vocab import (
    URI_OBS_PRED_ENTITY_MAPPER,
    URI_OBS_PRED_PROVIDER,
    URI_ROS_PRED_REFERENCE_FRAME,
)
from rdflib import RDF, Graph, Literal, URIRef
from tf2_ros import TransformException

from bdd_exec_ros2.tf_provider import TfProvider


def test_provider_discovers_mapper_frames_and_maps_available_transforms():
    provider_uri = URIRef("urn:test:provider")
    drawer_observation = URIRef("urn:test:drawer-observation")
    robot_observation = URIRef("urn:test:robot-observation")
    drawer_mapper = URIRef("urn:test:drawer-mapper")
    robot_mapper = URIRef("urn:test:robot-mapper")
    drawer_mapping = URIRef("urn:test:drawer-mapping")
    robot_mapping = URIRef("urn:test:robot-mapping")
    graph = Graph()
    graph.add((provider_uri, URI_ROS_PRED_REFERENCE_FRAME, Literal("base_link")))
    for observation, mapper, mapping, frame, target in (
        (
            drawer_observation,
            drawer_mapper,
            drawer_mapping,
            "drawer_handle",
            URIRef("urn:test:drawer"),
        ),
        (
            robot_observation,
            robot_mapper,
            robot_mapping,
            "g_pinch",
            URIRef("urn:test:robot"),
        ),
    ):
        graph.add((observation, URI_OBS_PRED_PROVIDER, provider_uri))
        graph.add((observation, URI_OBS_PRED_ENTITY_MAPPER, mapper))
        graph.add((mapper, URI_BDD_PRED_HAS_STRING_ENTITY_MAPPING, mapping))
        graph.add((mapping, RDF.type, URI_BDD_TYPE_STRING_ENTITY_MAPPING))
        graph.add((mapping, URI_BDD_PRED_STRING_VALUE, Literal(frame)))
        graph.add((mapping, URI_BDD_PRED_MAPPED_ENTITY, target))

    transform = TransformStamped()
    transform.header.frame_id = "base_link"
    transform.transform.translation.x = 1.25
    provider = TfProvider.__new__(TfProvider)
    provider._buffer = Mock()
    provider._buffer.lookup_transform.side_effect = [
        transform,
        TransformException("not available"),
    ]
    provider._logger = Mock()
    provider._callback_group = None
    provider._max_age_sec = 1.0
    provider._node = Mock()
    callback = Mock()

    provider.start_pose_polling(graph, provider_uri, 20.0, callback)

    poses = callback.call_args.args[0]
    assert set(poses) == {"drawer_handle"}
    assert poses["drawer_handle"].header.frame_id == "base_link"
    assert poses["drawer_handle"].pose.position.x == 1.25
    provider._logger.warning.assert_called_once()


def test_provider_omits_stale_transform():
    transform = TransformStamped()
    transform.header.stamp = Time(seconds=8.0).to_msg()
    provider = TfProvider.__new__(TfProvider)
    provider._buffer = Mock()
    provider._buffer.lookup_transform.return_value = transform
    provider._logger = Mock()
    provider._max_age_sec = 1.0
    provider._node = Mock()
    provider._node.get_clock.return_value.now.return_value = Time(seconds=10.0)

    poses = provider.get_poses("base_link", ["drawer_handle"])

    assert poses == {}
