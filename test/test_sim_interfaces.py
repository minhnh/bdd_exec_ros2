import asyncio
from pathlib import Path
from types import SimpleNamespace

from rdflib import URIRef
from simulation_interfaces.msg import Result, SimulatorFeatures

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
    def wait_for_service(self, timeout_sec):
        return True

    def call_async(self, request):
        self.request = request
        result = SimpleNamespace(result=Result.RESULT_OK, error_message="")
        return _Future(SimpleNamespace(result=result))


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
    client = _Client()
    interface = SimInterface.__new__(SimInterface)
    interface.timeout = 1.0
    interface._node = _Node()
    interface._load_world_srv_client = client
    interface._sim_features = SimpleNamespace(
        features=[SimulatorFeatures.WORLD_LOADING], spawn_formats=["usd"]
    )

    path = asyncio.run(interface.load_world(scene))

    assert path == Path("world.usd")
    assert client.request.uri == "world.usd"
