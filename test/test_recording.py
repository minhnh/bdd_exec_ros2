import subprocess
from types import SimpleNamespace
from unittest.mock import Mock, patch
from uuid import uuid4

import pytest
from rclpy.qos import DurabilityPolicy
from rdflib import RDF, Graph, URIRef
from rdflib.namespace import PROV, TIME

from bdd_exec_ros2.recording import BagRecorder, _record_bag, stop_recorder


def test_api_worker_configures_and_finalizes_recorder(tmp_path):
    connection = Mock()
    with (
        patch("bdd_exec_ros2.recording.rosbag2_py.Recorder") as factory,
        patch("bdd_exec_ros2.recording.signal.signal"),
    ):
        _record_bag(
            tmp_path / "bag", ["/data"], True, "recorder", "/static", connection
        )
        storage, options = factory.call_args.args
        assert storage.uri == str(tmp_path / "bag")
        assert options.use_sim_time and options.topics == ["/data"]
        assert options.include_hidden_topics and options.include_unpublished_topics
        assert (
            options.topic_qos_profile_overrides["/static"].durability
            == DurabilityPolicy.TRANSIENT_LOCAL
        )
        factory.return_value.record.assert_called_once_with()
        factory.return_value.stop.assert_called_once_with()
        factory.return_value.stop_spin.assert_called_once_with()
        connection.close.assert_called_once_with()


def test_recorder_start_finalize_and_provenance(tmp_path):
    execution_id = URIRef(uuid4().urn)
    recorder = BagRecorder(
        tmp_path / "bag",
        execution_id,
        {"/data", "/later", "/absent"},
        use_sim_time=True,
    )
    node = Mock()
    node.resolve_topic_name.side_effect = lambda name: name
    node.count_publishers.side_effect = lambda topic: topic != "/absent"
    checks = {"/data": 0, "/later": 0}

    def subscriptions(topic):
        checks[topic] += 1
        if topic == "/later" and checks[topic] == 1:
            return []
        return [SimpleNamespace(node_name=recorder.node_name)]

    node.get_subscriptions_info_by_topic.side_effect = subscriptions
    with patch("bdd_exec_ros2.recording.multiprocessing.get_context") as factory:
        context = factory.return_value
        parent, child = Mock(), Mock()
        context.Pipe.return_value = parent, child
        parent.recv.return_value = None
        process = context.Process.return_value
        process.is_alive.return_value = True
        recorder.start(node)
        process.start.assert_called_once_with()
        assert checks == {"/data": 1, "/later": 2}
        assert node.count_publishers.call_count == len(recorder.topics)
        child.close.assert_called_once_with()
        recorder.path.mkdir()
        (recorder.path / "metadata.yaml").write_text("finalized")
        (recorder.path / "storage.mcap").touch()
        process.is_alive.side_effect = [True, False]
        process.exitcode = 0
        recorder.stop()
        recorder.stop()
        parent.send.assert_called_once_with("stop")
        with pytest.raises(RuntimeError, match=str(execution_id)):
            recorder.start(node)
    graph, execution = Graph(), recorder.execution_id
    with patch("bdd_exec_ros2.recording.load_ros_pkg_prov") as package:
        package.return_value.purl = URIRef("pkg:ros/rosbag2_transport")
        recorder.add_prov(graph)
    bag = graph.value(predicate=RDF.type, object=PROV.Collection)
    activity = graph.value(bag, PROV.wasGeneratedBy)
    assert bag == recorder.bag_id
    assert activity == recorder.activity_id
    assert execution is execution_id
    assert len({bag, activity, execution}) == 3
    assert recorder.node_name.isidentifier()
    assert str(bag).startswith(str(execution))
    assert str(activity).startswith(str(execution))
    assert (execution, PROV.wasInfluencedBy, activity) in graph
    assert graph.value(activity, TIME.hasBeginning) is not None
    assert graph.value(activity, TIME.hasEnd) is not None
    members = set(graph.objects(bag, PROV.hadMember))
    assert len(members) == len(list(recorder.path.iterdir()))
    assert all((member, PROV.wasGeneratedBy, activity) in graph for member in members)


def test_shutdown_escalates():
    process = Mock()
    process.poll.return_value = None
    process.wait.side_effect = [
        subprocess.TimeoutExpired("recorder", 10),
        subprocess.TimeoutExpired("recorder", 5),
        0,
    ]
    stop_recorder(process)
    process.terminate.assert_called_once()
    process.kill.assert_called_once()


def test_existing_bag_is_preserved(tmp_path):
    execution_id = URIRef(uuid4().urn)
    with patch("bdd_exec_ros2.recording.multiprocessing.get_context") as launch:
        with pytest.raises(FileExistsError, match=str(execution_id)):
            BagRecorder(tmp_path, execution_id, {"/data"}, use_sim_time=False).start(
                Mock()
            )
        launch.assert_not_called()


@pytest.mark.parametrize("worker_error", [None, "storage unavailable"])
def test_start_failure_reports_execution_and_stops_recorder(tmp_path, worker_error):
    recorder = BagRecorder(
        tmp_path / "bag", URIRef(uuid4().urn), {"/data"}, use_sim_time=False
    )
    node = Mock()
    node.resolve_topic_name.side_effect = lambda name: name
    node.count_publishers.return_value = 1
    node.get_subscriptions_info_by_topic.return_value = []
    with patch("bdd_exec_ros2.recording.multiprocessing.get_context") as factory:
        parent, child = Mock(), Mock()
        factory.return_value.Pipe.return_value = parent, child
        parent.poll.return_value = worker_error is not None
        parent.recv.return_value = worker_error
        process = factory.return_value.Process.return_value
        process.is_alive.return_value = True
        error_type = TimeoutError if worker_error is None else RuntimeError
        with pytest.raises(error_type, match=str(recorder.execution_id)):
            recorder.start(node, timeout=0)
        process.terminate.assert_called_once_with()
        process.kill.assert_called_once_with()
        parent.close.assert_called_once_with()


@pytest.mark.parametrize("topics", [set(), {"relative"}])
def test_invalid_topics_are_rejected_before_start(tmp_path, topics):
    recorder = BagRecorder(
        tmp_path / "bag", URIRef(uuid4().urn), topics, use_sim_time=False
    )
    with patch("bdd_exec_ros2.recording.multiprocessing.get_context") as factory:
        with pytest.raises(ValueError, match=str(recorder.execution_id)):
            recorder.start(Mock())
        factory.assert_not_called()
    assert not list(tmp_path.iterdir())
