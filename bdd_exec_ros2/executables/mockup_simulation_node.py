# SPDX-License-Identifier: Apache-2.0
"""Minimal simulator setup services for the topic-based BDD mockup; no physics."""

from copy import deepcopy

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from simulation_interfaces.msg import Result, SimulationState, SimulatorFeatures
from simulation_interfaces.srv import (
    GetSimulationState,
    GetSimulatorFeatures,
    LoadWorld,
    ResetSimulation,
    SetSimulationState,
    SpawnEntity,
)


class MockupSimulationNode(Node):
    """Retain requested world, simulation state and spawn poses for mock setup."""

    def __init__(self, **kwargs):
        super().__init__("mockup_simulation", **kwargs)
        self.state = SimulationState.STATE_STOPPED
        self.world_uri = ""
        self.entities = {}
        for service_type, name, handler in (
            (GetSimulatorFeatures, "get_simulator_features", self.get_features),
            (GetSimulationState, "get_simulation_state", self.get_state),
            (SetSimulationState, "set_simulation_state", self.set_state),
            (LoadWorld, "load_world", self.load_world),
            (SpawnEntity, "spawn_entity", self.spawn_entity),
            (ResetSimulation, "reset_simulation", self.reset),
        ):
            self.create_service(service_type, name, handler)

    def get_features(self, request, response):
        response.features.features = [
            SimulatorFeatures.SPAWNING,
            SimulatorFeatures.WORLD_LOADING,
            SimulatorFeatures.SIMULATION_STATE_GETTING,
            SimulatorFeatures.SIMULATION_STATE_SETTING,
            SimulatorFeatures.SIMULATION_STATE_PAUSE,
            SimulatorFeatures.SIMULATION_RESET,
            SimulatorFeatures.SIMULATION_RESET_SPAWNED,
        ]
        response.features.spawn_formats = ["usd"]
        return response

    def get_state(self, request, response):
        response.state.state = self.state
        response.result.result = Result.RESULT_OK
        return response

    def set_state(self, request, response):
        if request.state.state not in (
            SimulationState.STATE_STOPPED,
            SimulationState.STATE_PLAYING,
            SimulationState.STATE_PAUSED,
        ):
            response.result.result = Result.RESULT_FEATURE_UNSUPPORTED
            return response
        self.state = request.state.state
        response.result.result = Result.RESULT_OK
        return response

    def load_world(self, request, response):
        if not request.uri and not request.resource_string:
            response.result.result = LoadWorld.Response.NO_RESOURCE
            return response
        self.world_uri = request.uri
        self.entities.clear()
        self.state = SimulationState.STATE_STOPPED
        response.result.result = Result.RESULT_OK
        return response

    def spawn_entity(self, request, response):
        if not request.uri and not request.resource_string:
            response.result.result = SpawnEntity.Response.NO_RESOURCE
            return response
        name = request.name
        if not name and not request.allow_renaming:
            response.result.result = SpawnEntity.Response.NAME_INVALID
            return response
        if name in self.entities and not request.allow_renaming:
            response.result.result = SpawnEntity.Response.NAME_NOT_UNIQUE
            return response
        base = name or "entity"
        index = 1
        while not name or name in self.entities:
            name = f"{base}_{index}"
            index += 1
        self.entities[name] = deepcopy(request)
        response.entity_name = name
        response.result.result = Result.RESULT_OK
        return response

    def reset(self, request, response):
        if request.scope not in (
            ResetSimulation.Request.SCOPE_DEFAULT,
            ResetSimulation.Request.SCOPE_ALL,
            ResetSimulation.Request.SCOPE_SPAWNED,
        ):
            response.result.result = Result.RESULT_FEATURE_UNSUPPORTED
            return response
        self.entities.clear()
        self.state = SimulationState.STATE_STOPPED
        response.result.result = Result.RESULT_OK
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockupSimulationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
