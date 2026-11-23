# SPDX-License-Identifier: Apache-2.0
"""ROS bag lifecycle and artifacts; callers wait outside coordinator locks."""

import multiprocessing
import signal
import subprocess
import time
from datetime import UTC, datetime
from pathlib import Path

import rosbag2_py
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rdf_utils.models.prov import (
    add_activity,
    add_directory_location,
    add_entity,
    add_file_entity,
    load_ros_pkg_prov,
)
from rdf_utils.models.time import add_wall_instant
from rdflib import RDF, Graph, URIRef
from rdflib.namespace import PROV


def stop_recorder(process: subprocess.Popen) -> None:
    """Finalize a recorder gracefully, escalating shutdown after bounded waits."""
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


def _record_bag(path, topics, use_sim_time, node_name, tf_static, connection):
    # Jazzy's Python Recorder initializes a global rclcpp context: isolate each instance.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    recorder = None
    try:
        storage = rosbag2_py.StorageOptions(
            uri=str(path), storage_id=rosbag2_py.get_default_storage_id()
        )
        options = rosbag2_py.RecordOptions()
        options.topics = topics
        options.use_sim_time = use_sim_time
        options.include_hidden_topics = True
        options.include_unpublished_topics = True
        options.disable_keyboard_controls = True
        options.topic_qos_profile_overrides = {
            tf_static: QoSProfile(
                depth=100,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
        }
        recorder = rosbag2_py.Recorder(storage, options, node_name=node_name)
        recorder.start_spin()
        recorder.record()
        connection.send(None)
        connection.recv()
    except Exception as exc:
        connection.send(str(exc))
        raise
    finally:
        try:
            if recorder is not None:
                try:
                    recorder.stop()
                finally:
                    recorder.stop_spin()
        finally:
            connection.close()


class BagRecorder:
    """Own one isolated rosbag2 API recorder and its finalized artifact provenance."""

    FINALIZE_TIMEOUT = 15.0  # seconds
    SHUTDOWN_TIMEOUT = 5.0  # seconds

    def __init__(
        self, path: Path, execution_id: URIRef, topics: set[str], *, use_sim_time: bool
    ):
        self.path = path
        self.execution_id = execution_id
        self.activity_id = URIRef(f"{self.execution_id}#bag-recording")
        self.bag_id = URIRef(f"{self.execution_id}#bag")
        self.topics = list(topics)
        self.use_sim_time = use_sim_time
        self.node_name = (
            f"bdd_bag_{execution_id.removeprefix('urn:uuid:').replace('-', '')}"
        )
        self._process = None
        self._connection = None
        self.started = None
        self.ended = None

    def start(self, node, *, timeout: float = 10.0) -> None:
        """Wait for selected published topics to be subscribed before behavior starts."""
        if self._process is not None:
            raise RuntimeError(
                f"Bag recorder for {self.execution_id} has already been started"
            )
        if self.path.exists():
            raise FileExistsError(
                f"Bag directory for {self.execution_id} already exists: {self.path}"
            )
        if not self.topics:
            raise ValueError(f"Bag recording for {self.execution_id} requires topics")
        pending = set()
        for topic in self.topics:
            if not topic.startswith("/"):
                raise ValueError(
                    f"Bag recording for {self.execution_id} requires absolute topic names"
                )
            if node.count_publishers(topic):
                pending.add(topic)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        context = multiprocessing.get_context("spawn")
        self._connection, child = context.Pipe()
        self._process = context.Process(
            target=_record_bag,
            args=(
                self.path,
                self.topics,
                self.use_sim_time,
                self.node_name,
                node.resolve_topic_name("/tf_static"),
                child,
            ),
        )
        self.started = datetime.now(UTC)
        self._process.start()
        child.close()
        try:
            deadline = time.monotonic() + timeout
            initialized = False
            while True:
                if self._connection.poll():
                    error = self._connection.recv()
                    if error is not None:
                        raise RuntimeError(
                            f"Bag recording for {self.execution_id} failed: {error}"
                        )
                    initialized = True
                if not self._process.is_alive():
                    raise RuntimeError(
                        f"Bag recorder for {self.execution_id} exited before scenario start"
                    )
                pending = {
                    topic
                    for topic in pending
                    if not any(
                        endpoint.node_name == self.node_name
                        for endpoint in node.get_subscriptions_info_by_topic(topic)
                    )
                }
                if initialized and not pending:
                    return
                if time.monotonic() >= deadline:
                    raise TimeoutError(
                        f"Bag recorder subscriptions for {self.execution_id} were not ready"
                    )
                time.sleep(0.05)
        except BaseException as exc:
            exc.add_note(f"Bag recorder execution: {self.execution_id}")
            try:
                self.stop()
            except Exception as cleanup_error:  # noqa: BLE001 - preserve the startup error
                exc.add_note(
                    f"Bag cleanup for {self.execution_id} failed: {cleanup_error}"
                )
            raise

    def stop(self) -> None:
        """Finalize within a bounded wait; report recorder and metadata failures."""
        if self._process is None or self.ended is not None:
            return

        assert self._connection is not None, (
            f"Bag recorder for {self.execution_id} has no connection"
        )
        try:
            if self._process.is_alive():
                try:
                    self._connection.send("stop")
                except (BrokenPipeError, EOFError):
                    pass
            self._process.join(self.FINALIZE_TIMEOUT)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(self.SHUTDOWN_TIMEOUT)
                if self._process.is_alive():
                    self._process.kill()
                    self._process.join(self.SHUTDOWN_TIMEOUT)
                raise TimeoutError(
                    f"Bag recorder for {self.execution_id} did not finalize in time"
                )
            if self._process.exitcode != 0:
                raise RuntimeError(
                    f"Bag recorder for {self.execution_id} exited with {self._process.exitcode}"
                )
            if not (self.path / "metadata.yaml").is_file():
                raise OSError(
                    f"Bag metadata for {self.execution_id} was not finalized: {self.path}"
                )
            self.ended = datetime.now(UTC)
        finally:
            self._connection.close()

    def add_prov(self, graph: Graph) -> None:
        """Describe a finalized bag and its member files in the execution bundle."""
        if self.started is None:
            raise RuntimeError(f"Bag recorder for {self.execution_id} has not started")
        if self.ended is None:
            raise RuntimeError(
                f"Cannot register unfinished bag for {self.execution_id}"
            )
        agent = load_ros_pkg_prov(graph, "rosbag2_transport")
        add_activity(
            graph,
            self.activity_id,
            (),
            (),
            agent.purl,
            add_wall_instant(graph, self.started),
            add_wall_instant(graph, self.ended),
        )
        bag = add_entity(graph, self.bag_id)
        location = add_directory_location(graph, self.path)
        graph.add((bag, RDF.type, PROV.Collection))
        graph.add((bag, PROV.atLocation, location))
        graph.add((bag, PROV.wasGeneratedBy, self.activity_id))
        graph.add((self.execution_id, PROV.wasInfluencedBy, self.activity_id))
        for path in sorted(self.path.rglob("*")):
            if path.is_file():
                member = add_file_entity(
                    graph,
                    location,
                    path.relative_to(self.path),
                    generated_by=self.activity_id,
                )
                graph.add((bag, PROV.hadMember, member))
