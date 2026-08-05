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

from __future__ import annotations

import re
import threading
from collections.abc import Callable, Collection
from pathlib import Path

from ament_index_python import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.client import Client
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node
from rdf_utils.models.vocab import (
    URI_EXEC_PRED_PATH,
)
from rdf_utils.naming import get_valid_var_name
from rdflib import Graph, URIRef
from scene_dsl.rdf.scenex import URI_MJCF_MUJOCO, URI_URDF_ROBOT, URI_USD_STAGE
from scene_dsl.rdf_parser.scenex import (
    SceneInstanceModel,
    get_ros_pkg_path,
)
from simulation_interfaces.msg import (
    Result,
    SimulationState,
    SimulatorFeatures,
)
from simulation_interfaces.msg import (
    SpawnEntity as SpawnEntityMsg,
)
from simulation_interfaces.srv import (
    GetEntitiesStates,
    GetSimulationState,
    GetSimulatorFeatures,
    LoadWorld,
    ResetSimulation,
    SetSimulationState,
    SpawnEntities,
    SpawnEntity,
)

from bdd_exec_ros2.conversions import create_spawn_entity_entries

FEATURE_NAMES = {
    value: name
    for name in dir(SimulatorFeatures)
    if name.isupper() and isinstance(value := getattr(SimulatorFeatures, name), int)
}


SUPPORTED_FORMAT_URIS = {
    "usd": URI_USD_STAGE,
    "urdf": URI_URDF_ROBOT,
    "mjcf": URI_MJCF_MUJOCO,
}


def _service_name(namespace: str, name: str) -> str:
    return f"{namespace.rstrip('/')}/{name}" if namespace else name


def _require_supported_resource_types(features: SimulatorFeatures) -> set[URIRef]:
    resource_types = {
        resource_type
        for format_name in features.spawn_formats
        if (resource_type := SUPPORTED_FORMAT_URIS.get(format_name)) is not None
    }
    if not resource_types:
        raise RuntimeError(
            f"unhandled simulator spawn formats: {features.spawn_formats}"
        )
    return resource_types


class PosePollingHandle:
    """Own an immediate, periodic batch-pose poll and allow cancellation."""

    def __init__(
        self,
        interface: SimInterface,
        scene_inst: SceneInstanceModel,
        element_ids: Collection[URIRef],
        frequency: float,
        callback: Callable[[dict[URIRef, PoseStamped]], None],
        error_callback: Callable[[Exception], None] | None,
    ) -> None:
        if frequency <= 0:
            raise ValueError("pose polling frequency must be positive")
        executor = interface._node.executor
        if executor is None:
            raise RuntimeError("simulation node is not attached to an executor")
        self._interface = interface
        self._scene_inst = scene_inst
        self._element_ids = frozenset(element_ids)
        self._callback = callback
        self._error_callback = error_callback
        self._executor = executor
        self._lock = threading.Lock()
        self._pending = False
        self._cancelled = False
        self._timer = interface._node.create_timer(1.0 / frequency, self._schedule)
        self._schedule()

    def _schedule(self) -> None:
        with self._lock:
            if self._cancelled or self._pending:
                return
            self._pending = True
        self._executor.create_task(self._poll())

    async def _poll(self) -> None:
        try:
            poses = await self._interface.get_elements_poses(
                self._scene_inst, self._element_ids
            )
            with self._lock:
                cancelled = self._cancelled
            if not cancelled:
                self._callback(poses)
        except Exception as exc:  # noqa: BLE001 - polling continues after failures
            with self._lock:
                cancelled = self._cancelled
            if not cancelled:
                if self._error_callback is None:
                    self._interface._logger.warning(f"pose polling failed: {exc}")
                else:
                    self._error_callback(exc)
        finally:
            with self._lock:
                self._pending = False

    def cancel(self) -> None:
        """Stop future polls and discard results from an in-flight request."""
        with self._lock:
            if self._cancelled:
                return
            self._cancelled = True
        self._interface._node.destroy_timer(self._timer)


class SimInterface:
    """Async client for simulator scene setup, state, and entity poses.

    Use :meth:`get_elements_poses` for one filtered batch request.
    :meth:`get_element_pose` is its one-element compatibility wrapper, while
    :meth:`start_pose_polling` owns periodic polling and cancellation.
    """

    _load_world_srv_client: Client
    _get_sim_state_srv_client: Client
    _get_entities_states_srv_client: Client
    _set_sim_state_srv_client: Client
    _reset_simulation_srv_client: Client
    _sim_feature_srv_client: Client
    _sim_features: SimulatorFeatures | None

    def __init__(
        self,
        node: Node,
        timeout: float = 5.0,
        ns: str = "",
        sim_feature_srv_name: str = "get_simulator_features",
        load_world_srv_name: str = "load_world",
        get_sim_state_srv_name: str = "get_simulation_state",
        set_sim_state_srv_name: str = "set_simulation_state",
        reset_simulation_srv_name: str = "reset_simulation",
        spawn_entity_srv_name: str = "spawn_entity",
        spawn_entities_srv_name: str = "spawn_entities",
        model_graph: Graph | None = None,
        world_entity_name: str = "world",
        get_entities_states_srv_name: str = "get_entities_states",
    ) -> None:
        self.ns: str = ns
        self.timeout: float = timeout
        self.world_entity_name: str = world_entity_name

        self._node: Node = node
        self._logger: RcutilsLogger = node.get_logger()
        self._model_graph = model_graph
        self._sim_features = None
        self._active_scene_inst_id: URIRef | None = None

        sim_feat_srv_full = _service_name(self.ns, sim_feature_srv_name)
        self._logger.info(f"Listening to service: {sim_feat_srv_full}")
        self._sim_feature_srv_client = node.create_client(
            srv_type=GetSimulatorFeatures,
            srv_name=sim_feat_srv_full,
        )
        if not self._sim_feature_srv_client.wait_for_service(timeout_sec=self.timeout):
            raise TimeoutError(
                f"Timed out waiting for '{sim_feat_srv_full}' after {self.timeout}s"
            )

        load_world_srv_full = _service_name(self.ns, load_world_srv_name)
        self._logger.info(f"Listening to service: {load_world_srv_full}")
        self._load_world_srv_client = node.create_client(
            srv_type=LoadWorld,
            srv_name=load_world_srv_full,
        )

        get_sim_state_srv_full = _service_name(self.ns, get_sim_state_srv_name)
        self._logger.info(f"Listening to service: {get_sim_state_srv_full}")
        self._get_sim_state_srv_client = node.create_client(
            srv_type=GetSimulationState,
            srv_name=get_sim_state_srv_full,
        )

        self._get_entities_states_srv_client = node.create_client(
            srv_type=GetEntitiesStates,
            srv_name=_service_name(self.ns, get_entities_states_srv_name),
        )

        set_sim_state_srv_full = _service_name(self.ns, set_sim_state_srv_name)
        self._logger.info(f"Listening to service: {set_sim_state_srv_full}")
        self._set_sim_state_srv_client = node.create_client(
            srv_type=SetSimulationState,
            srv_name=set_sim_state_srv_full,
        )

        self._reset_simulation_srv_client = node.create_client(
            srv_type=ResetSimulation,
            srv_name=_service_name(self.ns, reset_simulation_srv_name),
        )

        self._spawn_entity_srv_client = node.create_client(
            srv_type=SpawnEntity,
            srv_name=_service_name(self.ns, spawn_entity_srv_name),
        )
        self._spawn_entities_srv_client = node.create_client(
            srv_type=SpawnEntities,
            srv_name=_service_name(self.ns, spawn_entities_srv_name),
        )

    async def get_sim_features(self, quiet=True) -> SimulatorFeatures | None:
        """Return cached simulator capabilities, or ``None`` on a quiet timeout."""
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
        """Return the current simulation state when the simulator supports it."""
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

    async def get_elements_poses(
        self,
        scene_inst: SceneInstanceModel,
        element_ids: Collection[URIRef],
    ) -> dict[URIRef, PoseStamped]:
        """Return stamped poses for resolvable requested elements in one service call.

        Missing or unresolvable elements are omitted. The simulator must advertise
        entity-state support and a configured model graph is required.
        """
        if self._model_graph is None:
            raise RuntimeError("getting element poses requires a model graph")
        features = await self.get_sim_features(quiet=False)
        if (
            features is None
            or SimulatorFeatures.ENTITY_STATE_GETTING not in features.features
        ):
            raise RuntimeError("simulator does not support getting entity state")

        element_ids_by_entity: dict[str, list[URIRef]] = {}
        resource_types = _require_supported_resource_types(features)
        for element_id in element_ids:
            resolved = scene_inst.resolve_element_root_frame(
                element_id, resource_types, self._model_graph
            )
            if resolved is None:
                continue
            _, mapping, _ = resolved
            entity = mapping.entity or get_valid_var_name(
                element_id.n3(self._model_graph.namespace_manager)
            )
            element_ids_by_entity.setdefault(entity, []).append(element_id)
        if not element_ids_by_entity:
            return {}

        if not self._get_entities_states_srv_client.wait_for_service(
            timeout_sec=self.timeout
        ):
            raise TimeoutError(
                f"Timed out waiting for get_entities_states after {self.timeout}s"
            )

        request = GetEntitiesStates.Request()
        request.filters.filter = (
            "^("
            + "|".join(re.escape(entity) for entity in sorted(element_ids_by_entity))
            + ")$"
        )
        future = self._get_entities_states_srv_client.call_async(request)
        timer = self._node.create_timer(self.timeout, future.cancel)
        try:
            response = await future
        finally:
            self._node.destroy_timer(timer)

        if response is None:
            raise TimeoutError(f"get_entities_states timed out after {self.timeout}s")
        if response.result.result == Result.RESULT_NOT_FOUND:
            return {}
        if response.result.result != Result.RESULT_OK:
            raise RuntimeError(
                f"get_entities_states failed ({response.result.result}): "
                f"{response.result.error_message}"
            )
        if len(response.entities) != len(response.states):
            raise RuntimeError(
                "get_entities_states returned different entity and state counts"
            )

        poses = {}
        for entity, state in zip(response.entities, response.states, strict=True):
            for element_id in element_ids_by_entity.get(entity, ()):
                poses[element_id] = PoseStamped(header=state.header, pose=state.pose)
        return poses

    async def get_element_pose(
        self, scene_inst: SceneInstanceModel, element_id: URIRef
    ) -> PoseStamped | None:
        """Return one element pose, or ``None`` when it has no reported state."""
        return (await self.get_elements_poses(scene_inst, [element_id])).get(element_id)

    def start_pose_polling(
        self,
        scene_inst: SceneInstanceModel,
        element_ids: Collection[URIRef],
        frequency: float,
        callback: Callable[[dict[URIRef, PoseStamped]], None],
        error_callback: Callable[[Exception], None] | None = None,
    ) -> PosePollingHandle:
        """Start immediate periodic batch-pose polling at a positive frequency.

        The returned handle cancels its timer and discards callbacks from in-flight
        requests after cancellation.
        """
        return PosePollingHandle(
            self,
            scene_inst,
            element_ids,
            frequency,
            callback,
            error_callback,
        )

    async def set_sim_state(self, state: SimulationState) -> None:
        """Set the simulator state, accepting an already-reached target state."""
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

    async def load_world(self, scene_inst: SceneInstanceModel) -> Path | None:
        """Load the single supported world resource for ``scene_inst``, if any."""
        features = await self.get_sim_features(quiet=False)
        if features is None or SimulatorFeatures.WORLD_LOADING not in features.features:
            raise RuntimeError("simulator does not support world loading")

        resource_types = _require_supported_resource_types(features)

        resources = tuple(
            model
            for model in scene_inst.models.values()
            if model.types & resource_types
        )
        if not resources:
            return None
        if len(resources) > 1:
            raise ValueError(
                f"SceneInstance '{scene_inst.id}': ambiguous supported scene models: "
                f"{[resource.id for resource in resources]}"
            )

        resource = resources[0]
        path_str = resource.get_attr(URI_EXEC_PRED_PATH)
        if not isinstance(path_str, str):
            raise TypeError(f"scene model {resource.id} has no path")
        ros_pkg_path = get_ros_pkg_path(resource)
        path = (
            Path(get_package_share_directory(ros_pkg_path[0])) / ros_pkg_path[1]
            if ros_pkg_path is not None
            else Path(path_str).expanduser()
        )

        state = await self.get_sim_state()
        if state.state == SimulationState.STATE_PLAYING:
            await self.set_sim_state(
                SimulationState(state=SimulationState.STATE_STOPPED)
            )

        if not self._load_world_srv_client.wait_for_service(timeout_sec=self.timeout):
            raise TimeoutError(
                f"Timed out waiting for load_world after {self.timeout}s"
            )

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

    async def reset_simulation(self) -> None:
        """Reset simulation state and leave the simulator stopped."""
        features = await self.get_sim_features(quiet=False)
        if (
            features is None
            or SimulatorFeatures.SIMULATION_RESET not in features.features
        ):
            raise RuntimeError("simulator does not support resetting simulation")
        if not self._reset_simulation_srv_client.wait_for_service(
            timeout_sec=self.timeout
        ):
            raise TimeoutError(
                f"Timed out waiting for reset_simulation after {self.timeout}s"
            )

        future = self._reset_simulation_srv_client.call_async(
            ResetSimulation.Request(scope=ResetSimulation.Request.SCOPE_DEFAULT)
        )
        timer = self._node.create_timer(self.timeout, future.cancel)
        try:
            response = await future
        finally:
            self._node.destroy_timer(timer)

        if response is None:
            raise TimeoutError(f"reset_simulation timed out after {self.timeout}s")
        if response.result.result != Result.RESULT_OK:
            raise RuntimeError(
                f"reset_simulation failed ({response.result.result}): "
                f"{response.result.error_message}"
            )

        state = await self.get_sim_state()
        if state.state == SimulationState.STATE_PLAYING:
            await self.set_sim_state(
                SimulationState(state=SimulationState.STATE_STOPPED)
            )

    async def _send_spawn_entries(
        self,
        entries: list[tuple[URIRef, SpawnEntityMsg]],
        features: SimulatorFeatures,
    ) -> dict[URIRef, str]:
        if not entries:
            return {}

        responses = []
        num_entries = len(entries)
        if (
            SimulatorFeatures.SPAWNING_ENTITIES in features.features
            and self._spawn_entities_srv_client.wait_for_service(
                timeout_sec=self.timeout
            )
        ):
            request = SpawnEntities.Request()
            request.spawn_requests = [msg for _, msg in entries]
            future = self._spawn_entities_srv_client.call_async(request)
            # Scale spawn_entities timeout with number of entities
            # Not necessary for sending single requests
            spawn_timeout = num_entries * self.timeout
            timer = self._node.create_timer(spawn_timeout, future.cancel)
            try:
                response = await future
            finally:
                self._node.destroy_timer(timer)
            if response is None:
                raise TimeoutError(f"spawn_entities timed out after {self.timeout}s")
            if len(response.results) != num_entries:
                raise RuntimeError(
                    f"spawn_entities returned {len(response.results)} results for {num_entries} requests"
                )
            responses = response.results
            aggregate_ok = response.result.result == Result.RESULT_OK
            results_ok = all(
                result.result.result == Result.RESULT_OK for result in responses
            )
            if aggregate_ok != results_ok:
                raise RuntimeError(
                    "spawn_entities aggregate result disagrees with entity results"
                )
        elif SimulatorFeatures.SPAWNING in features.features:
            if not self._spawn_entity_srv_client.wait_for_service(
                timeout_sec=self.timeout
            ):
                raise TimeoutError(
                    f"Timed out waiting for spawn_entity after {self.timeout}s"
                )
            for _, msg in entries:
                future = self._spawn_entity_srv_client.call_async(
                    SpawnEntity.Request(
                        name=msg.name,
                        allow_renaming=msg.allow_renaming,
                        uri=msg.entity_resource.uri,
                        resource_string=msg.entity_resource.resource_string,
                        entity_namespace=msg.entity_namespace,
                        initial_pose=msg.initial_pose,
                    )
                )
                timer = self._node.create_timer(self.timeout, future.cancel)
                try:
                    response = await future
                finally:
                    self._node.destroy_timer(timer)
                if response is None:
                    raise TimeoutError(f"spawn_entity timed out after {self.timeout}s")
                responses.append(response)
        else:
            raise RuntimeError("simulator does not support spawning entities")

        spawned = {}
        for (elem_id, _), response in zip(entries, responses):
            if response.result.result != Result.RESULT_OK:
                message = (
                    f"failed to spawn '{elem_id}' ({response.result.result}): "
                    f"{response.result.error_message}"
                )
                self._logger.warning(message)
                continue
            spawned[elem_id] = response.entity_name
        return spawned

    async def spawn_entities(
        self,
        scene_inst: SceneInstanceModel,
        *,
        additional_elements: set[URIRef] | None = None,
    ) -> dict[URIRef, str]:
        """Spawn modelled scene elements and return their simulator entity names."""
        if self._model_graph is None:
            raise RuntimeError("spawning requires a model graph")
        features = await self.get_sim_features(quiet=False)
        assert features is not None
        entries = create_spawn_entity_entries(
            scene_inst,
            self._model_graph,
            _require_supported_resource_types(features),
            world_entity_name=self.world_entity_name,
            additional_elements=additional_elements,
            warn=self._logger.warning,
        )
        return await self._send_spawn_entries(entries, features)

    async def setup_scene(
        self,
        scene_inst: SceneInstanceModel,
        *,
        additional_elements: set[URIRef] | None = None,
    ) -> dict[URIRef, str]:
        """Load or reset a scene, spawn selected elements, then start simulation."""
        if self._model_graph is None:
            raise RuntimeError("scene setup requires a model graph")
        features = await self.get_sim_features(quiet=False)
        assert features is not None
        entries = create_spawn_entity_entries(
            scene_inst,
            self._model_graph,
            _require_supported_resource_types(features),
            world_entity_name=self.world_entity_name,
            additional_elements=additional_elements,
            warn=self._logger.warning,
        )
        if self._active_scene_inst_id == scene_inst.id:
            await self.reset_simulation()
        else:
            await self.load_world(scene_inst)
            self._active_scene_inst_id = scene_inst.id
        spawned = await self._send_spawn_entries(entries, features)
        await self.set_sim_state(SimulationState(state=SimulationState.STATE_PLAYING))
        return spawned
