# SPDX-License-Identifier: Apache-2.0
"""Coordinator adapter for scenario evidence; callers hold the coordinator lock."""

import json

from bdd_dsl.prov import ScenarioExecRecorder
from rdf_utils.models.prov import (
    add_entity,
    load_py_pkg_prov,
    load_ros_pkg_prov,
)
from rdflib import Graph, Literal, URIRef
from rdflib.namespace import PROV, SDO


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
    """Record policy evidence; attach callbacks only after host initialization succeeds."""

    @classmethod
    def open(cls, session, node, context, variant, *, scene_setup_mode):
        parameters = {
            name: node.get_parameter(name).value
            for name in node.list_parameters([], depth=0).names
        }
        directory = session.output_dir / str(context.context_id)
        directory.mkdir(parents=True, exist_ok=False)
        recorder = cls(
            directory / "evidence.jsonl",
            context.context_id,
            session,
        )
        try:
            for policy in context.obs_manager.obs_policies.values():
                recorder.record_policy_config(policy)
            with (directory / "runtime.json").open("x") as stream:
                json.dump({"parameters": parameters}, stream, indent=2)
            recorder.record_metadata(
                variant.id,
                {
                    "scenario_configuration": {
                        "scene_instance": context.scene_inst.id,
                        "use_sim_time": session.use_sim_time,
                        "scene_setup_mode": scene_setup_mode,
                    }
                },
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

    def finish(self, node, reason, *, stamp=None):
        from bdd_exec_ros2.conversions import ros_time_to_stamp

        if stamp is None:
            stamp = ros_time_to_stamp(node.get_clock().now())
        self.create_record(self.execution_id, stamp, {"termination": reason})
        self.close()
