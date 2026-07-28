import asyncio
from pathlib import Path
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from rdf_utils.models.common import ModelBase
from rdf_utils.models.vocab import URI_EXEC_PRED_PATH
from rdflib import URIRef
from scene_dsl.rdf_parser.vocab import URI_ROS_PRED_PACKAGE_NAME, URI_ROS_TYPE_PACKAGE
from simulation_interfaces.msg import Result, SimulationState, SimulatorFeatures

from bdd_exec_ros2.conversions import format_time_msg
from bdd_exec_ros2.sim_interfaces import (
    SUPPORTED_FORMAT_URIS,
    SceneFormat,
    SimInterface,
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
    def __init__(self, response):
        self.response = response

    def wait_for_service(self, timeout_sec):
        return True

    def call_async(self, request):
        self.request = request
        return _Future(self.response)


class _Node:
    def create_timer(self, timeout, callback):
        return object()

    def destroy_timer(self, timer):
        pass


def test_load_world_uses_requested_scene_format(monkeypatch):
    resource = ModelBase(
        node_id=URIRef("urn:test:world"),
        types={SUPPORTED_FORMAT_URIS[SceneFormat.USD]},
    )
    resource.set_attr(URI_EXEC_PRED_PATH, "world.usd")
    resource.types.add(URI_ROS_TYPE_PACKAGE)
    resource.set_attr(URI_ROS_PRED_PACKAGE_NAME, "world_pkg")
    monkeypatch.setattr(
        "bdd_exec_ros2.sim_interfaces.get_package_share_directory",
        lambda package_name: "/share/" + package_name,
    )
    scene = SimpleNamespace(models={resource.id: resource})
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

    assert asyncio.run(interface.load_world(SimpleNamespace(models={}))) is None


def test_load_world_rejects_ambiguous_models():
    models = {}
    for name in ("first", "second"):
        model = ModelBase(
            node_id=URIRef(f"urn:test:{name}"),
            types={SUPPORTED_FORMAT_URIS[SceneFormat.USD]},
        )
        models[model.id] = model
    interface = SimInterface.__new__(SimInterface)
    interface._sim_features = SimpleNamespace(
        features=[SimulatorFeatures.WORLD_LOADING], spawn_formats=["usd"]
    )

    with pytest.raises(ValueError, match="ambiguous supported scene models"):
        asyncio.run(
            interface.load_world(SimpleNamespace(id="urn:test:scene", models=models))
        )


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
