import asyncio
from pathlib import Path
from types import SimpleNamespace

from rdflib import URIRef
from simulation_interfaces.msg import Result, SimulationState, SimulatorFeatures

from bdd_exec_ros2.sim_interfaces import (
    SceneFormat,
    SimInterface,
    SUPPORTED_FORMAT_URIS,
)
from rdf_utils.models.common import ModelBase
from rdf_utils.models.vocab import URI_EXEC_PRED_PATH


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


def test_load_world_uses_requested_scene_format():
    resource = ModelBase(
        node_id=URIRef("urn:test:world"),
        types={SUPPORTED_FORMAT_URIS[SceneFormat.USD]},
    )
    resource.set_attr(URI_EXEC_PRED_PATH, "world.usd")
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

    assert path == Path("world.usd")
    assert set_state_client.request.state.state == SimulationState.STATE_STOPPED
    assert load_client.request.uri == "world.usd"
