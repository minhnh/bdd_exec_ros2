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

from pathlib import Path
from typing import Optional
from rclpy.impl.rcutils_logger import RcutilsLogger
from rdflib import Graph, URIRef

from rclpy.client import Client
from rclpy.node import Node

from bdd_dsl.models.urirefs import URI_EXEC_PRED_PATH
from scene_dsl.rdf.scenex import URI_USD_STAGE
from scene_dsl.rdf_parser.scenex import SceneInstanceModel

from simulation_interfaces.msg import SimulatorFeatures
from simulation_interfaces.srv import GetSimulatorFeatures


FEATURE_NAMES = {
    value: name
    for name in dir(SimulatorFeatures)
    if name.isupper() and isinstance(value := getattr(SimulatorFeatures, name), int)
}


class SimInterface:
    ns: str
    timeout: float

    _logger: RcutilsLogger
    _node: Node
    _sim_feature_srv_client: Client
    _sim_features: Optional[SimulatorFeatures]

    def __init__(
        self,
        node: Node,
        timeout: float = 5.0,
        ns: str = "",
        sim_feature_srv_name: str = "get_simulator_features",
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


def get_scene_path(graph: Graph, scn_inst_id: URIRef) -> Path:
    """Read the single USD scene path from a generated Scene DSL graph."""
    scene = SceneInstanceModel(scn_inst_id=scn_inst_id, graph=graph)
    resources = [
        model for model in scene.models.values() if URI_USD_STAGE in model.types
    ]
    if len(resources) != 1:
        raise ValueError(f"expected one USD scene model, found {len(resources)}")
    path = resources[0].get_attr(URI_EXEC_PRED_PATH)
    if not isinstance(path, str):
        raise ValueError("USD scene model has no path")
    return Path(path).expanduser()
