# SPDX-License-Identifier: Apache-2.0
from types import SimpleNamespace
from uuid import uuid4

import pytest
from bdd_dsl.prov import RecordingSession, ScenarioExecRecorder
from rdf_utils.models.prov import get_py_pkg_info
from rdflib import RDF, Dataset, URIRef
from rdflib.compare import isomorphic
from rdflib.namespace import PROV

from bdd_exec_ros2.prov import (
    RosScenarioExecRecorder,
)


def _open_recorder(session):
    context = SimpleNamespace(
        context_id=uuid4(),
        scene_inst=SimpleNamespace(id=URIRef("urn:scene")),
        variation_params={},
        obs_manager=SimpleNamespace(obs_policies={}),
    )
    return RosScenarioExecRecorder.open(
        session,
        context,
        SimpleNamespace(id=URIRef("urn:variant")),
    )


@pytest.mark.parametrize(
    "owner, method",
    [(RosScenarioExecRecorder, "__init__"), (ScenarioExecRecorder, "record_metadata")],
)
def test_failed_execution_initialization_preserves_shared_dataset(
    tmp_path, owner, method
):
    from unittest.mock import patch

    session = RecordingSession(tmp_path, URIRef("urn:test:agent"), use_sim_time=False)
    first = _open_recorder(session)
    before = session.path.read_bytes()
    with (
        patch.object(owner, method, side_effect=OSError("disk full")),
        pytest.raises(OSError),
    ):
        _open_recorder(session)
    assert session.path.read_bytes() == before
    bundles = list(session.dataset.default_graph.subjects(PROV.wasGeneratedBy, None))
    assert bundles == [first.bundle_id]
    first.close()


def test_package_metadata_is_recorded_once_in_the_shared_loading_bundle(tmp_path):
    import json
    from unittest.mock import patch

    from rdf_utils.models.prov import load_py_pkg_prov, load_ros_pkg_prov
    from rdflib.namespace import SDO

    from bdd_exec_ros2.executables.bdd_coordination_node import (
        load_graph_models_in_yaml,
    )

    source = tmp_path / "model.ttl"
    source.write_text("<urn:model> a <urn:Model> .")
    manifest = tmp_path / "models.yaml"
    manifest.write_text(json.dumps([{"path": str(source), "format": "turtle"}]))
    session = RecordingSession(
        tmp_path, get_py_pkg_info("bdd_exec_ros2").purl, use_sim_time=False
    )
    dataset, prov_id, model_id = session.dataset, session.prov_id, session.model_id
    model_provenance, model_g = session.prov_bundle, session.model_graph
    with (
        patch(
            "bdd_exec_ros2.prov.load_py_pkg_prov", wraps=load_py_pkg_prov
        ) as python_lookup,
        patch(
            "bdd_exec_ros2.prov.load_ros_pkg_prov", wraps=load_ros_pkg_prov
        ) as ros_lookup,
    ):
        load_graph_models_in_yaml(str(manifest), model_g, session.prov_bundle)
        assert session.dataset is dataset
        assert model_g.store is dataset.store
        model_g.add((URIRef("urn:added-after-session"), RDF.type, PROV.Entity))
        (ros,) = model_provenance.subjects(PROV.alternateOf, session.agent_id)
        link = (ros, PROV.alternateOf, session.agent_id)
        (shared,) = [graph for graph in session.dataset.graphs() if link in graph]
        for _ in range(2):
            recorder = _open_recorder(session)
            assert (
                recorder.bundle_id,
                PROV.wasAttributedTo,
                session.agent_id,
            ) in recorder.graph
            assert (
                recorder.execution_id,
                PROV.wasAssociatedWith,
                session.agent_id,
            ) in recorder.graph
            assert not list(recorder.graph.subjects(RDF.type, PROV.SoftwareAgent))
            assert not list(recorder.graph.triples((None, PROV.alternateOf, None)))
            assert (session.agent_id, RDF.type, PROV.SoftwareAgent) in shared
            recorder.close()
    assert (
        sum(call.args[1] == "bdd_exec_ros2" for call in python_lookup.call_args_list)
        == 1
    )
    restored = Dataset().parse(session.path, format=session.dataset_format)
    assert set(restored.graph(model_id)) == set(model_g)
    assert isomorphic(restored.graph(prov_id), model_provenance)
    assert ros_lookup.call_count == 1
    packages = list(shared.subjects(RDF.type, PROV.SoftwareAgent))
    assert {str(shared.value(package, SDO.name)) for package in packages} >= {
        "bdd_exec_ros2",
        "bdd_dsl",
        "rdf_utils",
        "robbdd",
    }


def test_execution_recorder_owns_bag_lifecycle(tmp_path):
    import threading
    from unittest.mock import Mock, patch

    from bdd_dsl.prov import read_journal

    session = RecordingSession(tmp_path, URIRef("urn:test:agent"), use_sim_time=True)
    recorder = _open_recorder(session)
    lock = threading.Lock()
    topics = {"/data"}
    node = Mock()
    with patch("bdd_exec_ros2.prov.BagRecorder") as factory:
        bag = factory.return_value

        def start_or_stop(*_):
            assert not lock.locked()

        def add_prov(graph):
            assert lock.locked()
            assert graph is recorder.graph

        bag.start.side_effect = start_or_stop
        bag.stop.side_effect = start_or_stop
        bag.add_prov.side_effect = add_prov
        recorder.start_bag(node, topics)
        factory.assert_called_once_with(
            recorder.path.parent / "bag",
            recorder.execution_id,
            topics,
            use_sim_time=True,
        )
        recorder.finish(node, "finished", lock=lock, stamp=1.0)
        bag.stop.assert_called_once_with()
        bag.add_prov.assert_called_once_with(recorder.graph)
    assert list(read_journal(recorder.path))[-1]["data"]["termination"] == "finished"
    with pytest.raises(OSError):
        recorder.create_record(recorder.execution_id, 2.0)
