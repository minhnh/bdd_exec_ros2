# SPDX-License-Identifier: Apache-2.0
"""ROS execution evidence and bag ownership; graph updates use the coordinator lock."""

from bdd_dsl.prov import ScenarioExecRecorder
from rdf_utils.models.prov import (
    add_entity,
    load_py_pkg_prov,
    load_ros_pkg_prov,
)
from rdflib import Graph, Literal, URIRef
from rdflib.namespace import PROV, SDO

from bdd_exec_ros2.recording import BagRecorder


def load_bdd_ros_prov(graph: Graph) -> URIRef:
    """Record the coordinator's package views once in this graph and return its Python URI."""
    for _, py_purl in graph.subject_objects(PROV.alternateOf):
        if (py_purl, SDO.name, Literal("bdd_exec_ros2")) in graph:
            assert isinstance(py_purl, URIRef), f"Package URL not an URIRef: {py_purl}"
            return py_purl

    python = load_py_pkg_prov(graph, "bdd_exec_ros2")
    ros = load_ros_pkg_prov(graph, "bdd_exec_ros2")
    add_entity(graph, python.purl)
    add_entity(graph, ros.purl)
    graph.add((ros.purl, PROV.alternateOf, python.purl))
    for name in ("bdd_dsl", "rdf_utils", "robbdd", "scene_dsl"):
        if not any(graph.subjects(SDO.name, Literal(name))):
            load_py_pkg_prov(graph, name)
    return python.purl


class RosScenarioExecRecorder(ScenarioExecRecorder):
    """Own execution evidence and its optional bag; attach callbacks after initialization."""

    _bag: BagRecorder | None = None

    @classmethod
    def open(cls, session, context, variant):
        directory = session.output_dir / str(context.context_id)
        directory.mkdir(parents=True, exist_ok=False)
        recorder = cls(
            directory / "evidence.jsonl",
            context.context_id,
            session,
        )
        try:
            recorder.record_var_assignments(context.variation_params)
            for policy in context.obs_manager.obs_policies.values():
                recorder.record_policy_config(policy)
            recorder.record_metadata(
                variant.id, {"scene_instance": context.scene_inst.id}
            )
            session.save()
        except BaseException:
            session.dataset.remove_graph(recorder.bundle_id)
            session.dataset.default_graph.remove((recorder.bundle_id, None, None))
            recorder.close()
            raise
        context.obs_manager.prov_recorder = recorder
        for policy in context.obs_manager.obs_policies.values():
            policy.prov_recorder = recorder
        return recorder

    def start_bag(self, node, topics: set[str]):
        """Start the execution's bag outside the coordinator lock."""
        bag = BagRecorder(
            self.path.parent / "bag",
            self.execution_id,
            topics,
            use_sim_time=self.session.use_sim_time,
        )
        bag.start(node)
        self._bag = bag

    def finish(self, node, reason, *, lock, stamp=None):
        """Finalize the bag outside lock, then save its provenance and close evidence under lock."""
        from bdd_exec_ros2.conversions import ros_time_to_stamp

        try:
            if self._bag is not None:
                self._bag.stop()
        except Exception:
            reason = "bag_recording_failed"
            raise
        finally:
            with lock:
                try:
                    if self._bag is not None and self._bag.ended is not None:
                        self._bag.add_prov(self.graph)
                    if stamp is None:
                        stamp = ros_time_to_stamp(node.get_clock().now())
                    self.create_record(
                        self.execution_id, stamp, {"termination": reason}
                    )
                    self.close()
                    self.session.save()
                finally:
                    self.close()
