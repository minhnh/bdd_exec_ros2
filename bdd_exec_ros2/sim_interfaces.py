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

from enum import StrEnum
from pathlib import Path
from typing import Optional
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.client import Client
from rclpy.node import Node

from rdf_utils.models.vocab import URI_EXEC_PRED_PATH
from scene_dsl.rdf.scenex import URI_MJCF_MUJOCO, URI_URDF_ROBOT, URI_USD_STAGE
from scene_dsl.rdf_parser.scenex import SceneInstanceModel

from simulation_interfaces.msg import Result, SimulationState, SimulatorFeatures
from simulation_interfaces.srv import (
    GetSimulationState,
    GetSimulatorFeatures,
    LoadWorld,
    SetSimulationState,
)


FEATURE_NAMES = {
    value: name
    for name in dir(SimulatorFeatures)
    if name.isupper() and isinstance(value := getattr(SimulatorFeatures, name), int)
}


class SceneFormat(StrEnum):
    USD = "usd"
    URDF = "urdf"
    MJCF = "mjcf"


SUPPORTED_FORMAT_URIS = {
    SceneFormat.USD: URI_USD_STAGE,
    SceneFormat.URDF: URI_URDF_ROBOT,
    SceneFormat.MJCF: URI_MJCF_MUJOCO,
}


class SimInterface:
    ns: str
    timeout: float

    _logger: RcutilsLogger
    _node: Node
    _load_world_srv_client: Client
    _get_sim_state_srv_client: Client
    _set_sim_state_srv_client: Client
    _sim_feature_srv_client: Client
    _sim_features: Optional[SimulatorFeatures]

    def __init__(
        self,
        node: Node,
        timeout: float = 5.0,
        ns: str = "",
        sim_feature_srv_name: str = "get_simulator_features",
        load_world_srv_name: str = "load_world",
        get_sim_state_srv_name: str = "get_simulation_state",
        set_sim_state_srv_name: str = "set_simulation_state",
    ) -> None:
        self.ns = ns
        self.timeout = timeout
        self._node = node
        self._logger = node.get_logger()

        sim_feat_srv_full = f"{self.ns}/{sim_feature_srv_name}"
        self._sim_features = None
        self._logger.info(f"Listening to service: {sim_feat_srv_full}")
        self._sim_feature_srv_client = node.create_client(
            srv_type=GetSimulatorFeatures,
            srv_name=sim_feat_srv_full,
        )
        if not self._sim_feature_srv_client.wait_for_service(timeout_sec=self.timeout):
            raise TimeoutError(
                f"Timed out waiting for '{sim_feat_srv_full}' after {self.timeout}s"
            )

        load_world_srv_full = f"{self.ns}/{load_world_srv_name}"
        self._logger.info(f"Listening to service: {load_world_srv_full}")
        self._load_world_srv_client = node.create_client(
            srv_type=LoadWorld,
            srv_name=load_world_srv_full,
        )

        get_sim_state_srv_full = f"{self.ns}/{get_sim_state_srv_name}"
        self._logger.info(f"Listening to service: {get_sim_state_srv_full}")
        self._get_sim_state_srv_client = node.create_client(
            srv_type=GetSimulationState,
            srv_name=get_sim_state_srv_full,
        )

        set_sim_state_srv_full = f"{self.ns}/{set_sim_state_srv_name}"
        self._logger.info(f"Listening to service: {set_sim_state_srv_full}")
        self._set_sim_state_srv_client = node.create_client(
            srv_type=SetSimulationState,
            srv_name=set_sim_state_srv_full,
        )

    async def get_sim_features(self, quiet=True) -> Optional[SimulatorFeatures]:
        if self._sim_features is not None:
            return self._sim_features

        future = self._sim_feature_srv_client.call_async(GetSimulatorFeatures.Request())
        timer = self._node.create_timer(self.timeout, future.cancel)
        try:
            response = await future
        finally:
            self._node.destroy_timer(timer)
        if response is None:
            err_msg = f"get_simulator_features timed out after {self.timeout}s"
            if not quiet:
                raise TimeoutError(err_msg)

            self._logger.error(err_msg)
            return None

        self._sim_features = response.features
        return self._sim_features

    async def get_sim_state(self) -> SimulationState:
        features = await self.get_sim_features(quiet=False)
        if (
            features is None
            or SimulatorFeatures.SIMULATION_STATE_GETTING not in features.features
        ):
            raise RuntimeError("simulator does not support getting simulation state")
        if not self._get_sim_state_srv_client.wait_for_service(
            timeout_sec=self.timeout
        ):
            raise TimeoutError(
                f"Timed out waiting for get_simulation_state after {self.timeout}s"
            )

        future = self._get_sim_state_srv_client.call_async(GetSimulationState.Request())
        timer = self._node.create_timer(self.timeout, future.cancel)
        try:
            response = await future
        finally:
            self._node.destroy_timer(timer)

        if response is None:
            raise TimeoutError(f"get_simulation_state timed out after {self.timeout}s")
        if response.result.result != Result.RESULT_OK:
            raise RuntimeError(
                f"get_simulation_state failed ({response.result.result}): "
                f"{response.result.error_message}"
            )
        return response.state

    async def set_sim_state(self, state: SimulationState) -> None:
        features = await self.get_sim_features(quiet=False)
        if (
            features is None
            or SimulatorFeatures.SIMULATION_STATE_SETTING not in features.features
        ):
            raise RuntimeError("simulator does not support setting simulation state")
        if not self._set_sim_state_srv_client.wait_for_service(
            timeout_sec=self.timeout
        ):
            raise TimeoutError(
                f"Timed out waiting for set_simulation_state after {self.timeout}s"
            )

        request = SetSimulationState.Request()
        request.state = state
        future = self._set_sim_state_srv_client.call_async(request)
        timer = self._node.create_timer(self.timeout, future.cancel)
        try:
            response = await future
        finally:
            self._node.destroy_timer(timer)

        if response is None:
            raise TimeoutError(f"set_simulation_state timed out after {self.timeout}s")
        if response.result.result not in (
            Result.RESULT_OK,
            SetSimulationState.Response.ALREADY_IN_TARGET_STATE,
        ):
            raise RuntimeError(
                f"set_simulation_state failed ({response.result.result}): "
                f"{response.result.error_message}"
            )

    async def load_world(self, scene_inst: SceneInstanceModel) -> Path:
        features = await self.get_sim_features(quiet=False)
        if features is None or SimulatorFeatures.WORLD_LOADING not in features.features:
            raise RuntimeError("simulator does not support world loading")

        state = await self.get_sim_state()
        if state.state == SimulationState.STATE_PLAYING:
            await self.set_sim_state(
                SimulationState(state=SimulationState.STATE_STOPPED)
            )

        if not self._load_world_srv_client.wait_for_service(timeout_sec=self.timeout):
            raise TimeoutError(
                f"Timed out waiting for load_world after {self.timeout}s"
            )

        resource_types = {
            resource_type
            for format_name in features.spawn_formats
            if (resource_type := SUPPORTED_FORMAT_URIS.get(format_name)) is not None
        }
        if len(resource_types) < 1:
            raise RuntimeError(
                f"unhandled simulator spawn formats: {features.spawn_formats}"
            )

        resources = [
            model
            for model in scene_inst.models.values()
            if resource_types.intersection(model.types)
        ]
        if len(resources) != 1:
            raise ValueError(
                f"SceneInstance '{scene_inst.id}': expected one supported scene model format ({features.spawn_formats}),"
                f" found {len(resources)}"
            )

        path_str = resources[0].get_attr(URI_EXEC_PRED_PATH)
        if not isinstance(path_str, str):
            raise ValueError(f"scene model {resources[0].id} has no path")
        path = Path(path_str).expanduser()

        request = LoadWorld.Request()
        request.uri = str(path)
        future = self._load_world_srv_client.call_async(request)
        timer = self._node.create_timer(self.timeout, future.cancel)
        try:
            response = await future
        finally:
            self._node.destroy_timer(timer)

        if response is None:
            raise TimeoutError(f"load_world timed out after {self.timeout}s")
        if response.result.result != Result.RESULT_OK:
            raise RuntimeError(
                f"load_world failed ({response.result.result}): {response.result.error_message}"
            )
        return path
