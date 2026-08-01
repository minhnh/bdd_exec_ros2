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

import asyncio
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest
from builtin_interfaces.msg import Time
from rdf_utils.models.common import ModelBase
from rdf_utils.models.execution import get_attr_path
from rdf_utils.models.vocab import URI_EXEC_PRED_PATH
from rdflib import URIRef
from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.scenex import create_scenex_model_graph
from scene_dsl.rdf_parser.scenex import SceneInstanceModel
from scene_dsl.rdf_parser.vocab import URI_ROS_PRED_PACKAGE_NAME, URI_ROS_TYPE_PACKAGE
from simulation_interfaces.msg import Result, SimulationState, SimulatorFeatures

from bdd_exec_ros2.conversions import create_spawn_entity_entries, format_time_msg
from bdd_exec_ros2.sim_interfaces import (
    SUPPORTED_FORMAT_URIS,
    SimInterface,
    _service_name,
)


class _Future:
    def __init__(self, response):
        self.response = response

    def __await__(self):
        async def done():
            return self.response

        return done().__await__()

    def cancel(self):
        pass


class _Client:
    def __init__(self, response, available=True):
        self.response = response
        self.available = available
        self.requests = []

    def wait_for_service(self, timeout_sec):
        return self.available

    def call_async(self, request):
        self.request = request
        self.requests.append(request)
        return _Future(self.response)


class _Node:
    def create_timer(self, timeout, callback):
        return object()

    def destroy_timer(self, timer):
        pass


def _scene_with_models(models, scene_id="urn:test:scene"):
    return SimpleNamespace(id=scene_id, models=models)


def test_load_world_uses_requested_scene_format(monkeypatch):
    resource = ModelBase(
        node_id=URIRef("urn:test:world"),
        types={SUPPORTED_FORMAT_URIS["usd"]},
    )
    resource.set_attr(URI_EXEC_PRED_PATH, "world.usd")
    resource.types.add(URI_ROS_TYPE_PACKAGE)
    resource.set_attr(URI_ROS_PRED_PACKAGE_NAME, "world_pkg")
    monkeypatch.setattr(
        "bdd_exec_ros2.sim_interfaces.get_package_share_directory",
        lambda package_name: "/share/" + package_name,
    )
    scene = _scene_with_models({resource.id: resource})
    result = SimpleNamespace(result=Result.RESULT_OK, error_message="")
    load_client = _Client(SimpleNamespace(result=result))
    get_state_client = _Client(
        SimpleNamespace(
            result=result,
            state=SimulationState(state=SimulationState.STATE_PLAYING),
        )
    )
    set_state_client = _Client(SimpleNamespace(result=result))
    interface = SimInterface.__new__(SimInterface)
    interface.timeout = 1.0
    interface._node = _Node()
    interface._load_world_srv_client = load_client
    interface._get_sim_state_srv_client = get_state_client
    interface._set_sim_state_srv_client = set_state_client
    interface._sim_features = SimpleNamespace(
        features=[
            SimulatorFeatures.WORLD_LOADING,
            SimulatorFeatures.SIMULATION_STATE_GETTING,
            SimulatorFeatures.SIMULATION_STATE_SETTING,
        ],
        spawn_formats=["usd"],
    )
    path = asyncio.run(interface.load_world(scene))

    assert path == Path("/share/world_pkg/world.usd")
    assert set_state_client.request.state.state == SimulationState.STATE_STOPPED
    assert load_client.request.uri == "/share/world_pkg/world.usd"


def test_load_world_skips_missing_model():
    interface = SimInterface.__new__(SimInterface)
    interface._sim_features = SimpleNamespace(
        features=[SimulatorFeatures.WORLD_LOADING], spawn_formats=["usd"]
    )
    assert asyncio.run(interface.load_world(_scene_with_models({}))) is None


def test_load_world_rejects_ambiguous_models():
    models = {}
    for name in ("first", "second"):
        model = ModelBase(
            node_id=URIRef(f"urn:test:{name}"),
            types={SUPPORTED_FORMAT_URIS["usd"]},
        )
        models[model.id] = model
    interface = SimInterface.__new__(SimInterface)
    interface._sim_features = SimpleNamespace(
        features=[SimulatorFeatures.WORLD_LOADING], spawn_formats=["usd"]
    )
    with pytest.raises(ValueError, match="ambiguous supported scene models"):
        asyncio.run(interface.load_world(_scene_with_models(models)))


def _spawn_interface(features):
    model_path = Path(__file__).parents[1] / "models" / "robbdd" / "lab.scenex"
    model = scenex_metamodel().model_from_file(model_path)
    graph = create_scenex_model_graph(model)
    scene = SceneInstanceModel(model.scene_insts[0].uri, graph)
    interface = SimInterface.__new__(SimInterface)
    interface.timeout = 1.0
    interface._node = _Node()
    interface._logger = SimpleNamespace(warning=lambda message: None)
    interface._model_graph = graph
    interface.world_entity_name = "world"
    interface._sim_features = features
    return interface, scene


def test_create_spawn_entity_entries_converts_executable_scene_objects():
    features = SimpleNamespace(
        features=[SimulatorFeatures.SPAWNING], spawn_formats=["usd"]
    )
    interface, scene = _spawn_interface(features)

    entries = create_spawn_entity_entries(
        scene,
        interface._model_graph,
        {SUPPORTED_FORMAT_URIS["usd"]},
        world_entity_name="world",
    )
    assert {element_id for element_id, _ in entries} == set(scene.object_models)
    assert all(
        message.initial_pose.header.frame_id == "world" for _, message in entries
    )
    for element_id, message in entries:
        [resource] = scene.object_models[element_id].values()
        assert message.entity_resource.uri == get_attr_path(resource)


def test_create_spawn_entity_entries_rejects_missing_configured_world_mapping():
    features = SimpleNamespace(
        features=[SimulatorFeatures.SPAWNING], spawn_formats=["usd"]
    )
    interface, scene = _spawn_interface(features)

    with pytest.raises(ValueError, match="simulation-world"):
        create_spawn_entity_entries(
            scene,
            interface._model_graph,
            {SUPPORTED_FORMAT_URIS["usd"]},
            world_entity_name="simulation-world",
        )


def test_create_spawn_entity_entries_skips_resources_without_mappings(monkeypatch):
    features = SimpleNamespace(
        features=[SimulatorFeatures.SPAWNING], spawn_formats=["usd"]
    )
    interface, scene = _spawn_interface(features)
    warnings = []
    monkeypatch.setattr(
        "bdd_exec_ros2.conversions.get_kinematic_mappings", lambda *_: []
    )

    assert (
        create_spawn_entity_entries(
            scene,
            interface._model_graph,
            {SUPPORTED_FORMAT_URIS["usd"]},
            warn=warnings.append,
        )
        == []
    )
    assert any("has no mapping" in warning for warning in warnings)


def test_create_spawn_entity_entries_uses_loaded_objects_and_additional_elements():
    features = SimpleNamespace(
        features=[SimulatorFeatures.SPAWNING], spawn_formats=["usd"]
    )
    interface, scene = _spawn_interface(features)
    selected = URIRef("https://example.test/selected-cube")
    missing = URIRef("https://example.test/missing-cube")
    scene.object_models[selected] = next(iter(scene.object_models.values()))
    warnings = []
    default_elements = set(scene.scene_model.objects) & set(scene.object_models)

    assert {
        element_id
        for element_id, _ in create_spawn_entity_entries(
            scene,
            interface._model_graph,
            {SUPPORTED_FORMAT_URIS["usd"]},
        )
    } == default_elements
    assert (
        len(
            create_spawn_entity_entries(
                scene,
                interface._model_graph,
                {SUPPORTED_FORMAT_URIS["usd"]},
                additional_elements={selected, missing},
                warn=warnings.append,
            )
        )
        == len(default_elements) + 1
    )
    assert any("missing-cube" in warning for warning in warnings)


def test_spawn_entities_uses_legacy_service_when_batch_is_not_advertised():
    features = SimpleNamespace(
        features=[SimulatorFeatures.SPAWNING], spawn_formats=["usd"]
    )
    interface, scene = _spawn_interface(features)
    result = SimpleNamespace(result=Result.RESULT_OK, error_message="")
    interface._spawn_entity_srv_client = _Client(
        SimpleNamespace(result=result, entity_name="spawned")
    )
    interface._spawn_entities_srv_client = _Client(None)

    spawned = asyncio.run(interface.spawn_entities(scene))

    assert set(spawned) == set(scene.object_models)
    assert len(interface._spawn_entity_srv_client.requests) == len(scene.object_models)
    assert not interface._spawn_entities_srv_client.requests


def test_spawn_entities_prefers_batch_service():
    features = SimpleNamespace(
        features=[SimulatorFeatures.SPAWNING, SimulatorFeatures.SPAWNING_ENTITIES],
        spawn_formats=["usd"],
    )
    interface, scene = _spawn_interface(features)
    result = SimpleNamespace(result=Result.RESULT_OK, error_message="")
    interface._spawn_entities_srv_client = _Client(
        SimpleNamespace(
            result=result,
            results=[SimpleNamespace(result=result, entity_name="spawned")]
            * len(scene.object_models),
        )
    )
    interface._spawn_entity_srv_client = _Client(None)

    spawned = asyncio.run(interface.spawn_entities(scene))

    assert set(spawned) == set(scene.object_models)
    assert set(spawned.values()) == {"spawned"}
    assert len(interface._spawn_entities_srv_client.requests) == 1
    assert not interface._spawn_entity_srv_client.requests


def test_spawn_entities_rejects_inconsistent_batch_result():
    features = SimpleNamespace(
        features=[SimulatorFeatures.SPAWNING_ENTITIES], spawn_formats=["usd"]
    )
    interface, scene = _spawn_interface(features)
    ok = SimpleNamespace(result=Result.RESULT_OK, error_message="")
    failed = SimpleNamespace(
        result=Result.RESULT_OPERATION_FAILED, error_message="failed"
    )
    interface._spawn_entities_srv_client = _Client(
        SimpleNamespace(
            result=failed,
            results=[SimpleNamespace(result=ok, entity_name="spawned")]
            * len(scene.object_models),
        )
    )

    with pytest.raises(RuntimeError, match="aggregate result disagrees"):
        asyncio.run(interface.spawn_entities(scene))


def test_spawn_entities_falls_back_when_batch_service_is_unavailable():
    features = SimpleNamespace(
        features=[SimulatorFeatures.SPAWNING, SimulatorFeatures.SPAWNING_ENTITIES],
        spawn_formats=["usd"],
    )
    interface, scene = _spawn_interface(features)
    result = SimpleNamespace(result=Result.RESULT_OK, error_message="")
    interface._spawn_entities_srv_client = _Client(None, available=False)
    interface._spawn_entity_srv_client = _Client(
        SimpleNamespace(result=result, entity_name="spawned")
    )

    assert len(asyncio.run(interface.spawn_entities(scene))) == len(scene.object_models)
    assert len(interface._spawn_entity_srv_client.requests) == len(scene.object_models)


def test_setup_scene_passes_exact_instance_to_low_level_operations():
    features = SimpleNamespace(
        features=[SimulatorFeatures.SPAWNING], spawn_formats=["usd"]
    )
    interface, scene = _spawn_interface(features)
    interface.load_world = AsyncMock(return_value=Path("world.usd"))
    interface._send_spawn_entries = AsyncMock(return_value={})

    assert asyncio.run(interface.setup_scene(scene)) == {}
    interface.load_world.assert_awaited_once_with(scene)
    entries, sent_features = interface._send_spawn_entries.await_args.args
    assert {elem_id for elem_id, _ in entries} == set(scene.object_models)
    assert sent_features is features


def test_service_names_preserve_relative_and_explicit_namespaces():
    assert _service_name("", "spawn_entity") == "spawn_entity"
    assert _service_name("/", "spawn_entity") == "/spawn_entity"
    assert _service_name("/sim/", "spawn_entity") == "/sim/spawn_entity"


def test_format_time_msg_distinguishes_wall_and_sim_time():
    stamp = Time(sec=7_384, nanosec=123_999_999)

    assert format_time_msg(stamp, use_sim_time=True) == "02:03:04.123"
    assert (
        format_time_msg(stamp, use_sim_time=True, num_decimals=9)
        == "02:03:04.123999999"
    )
    assert format_time_msg(Time()) == "1970-01-01 00:00:00.000 UTC"

    for num_decimals in (0, 10):
        with pytest.raises(ValueError, match="between 1 and 9"):
            format_time_msg(stamp, num_decimals=num_decimals)
