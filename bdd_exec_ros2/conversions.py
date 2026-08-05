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

from collections.abc import Callable, Iterable
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Final
from uuid import UUID

import numpy as np
from ament_index_python import get_package_share_directory
from bdd_dsl.models.clauses import WhenBehaviourModel, get_clause_config
from bdd_dsl.models.observation import (
    ObservationManager,
    TrinariesPolicyProtocol,
    TrinaryStamped,
)
from bdd_dsl.models.urirefs import (
    URI_BDD_TYPE_CONFIG,
    URI_BHV_PRED_TARGET_AGN,
    URI_BHV_PRED_TARGET_OBJ,
    URI_BHV_PRED_TARGET_WS,
    URI_BHV_TYPE_PICK,
    URI_BHV_TYPE_PLACE,
)
from bdd_dsl.models.user_story import ScenarioVariantModel
from bdd_dsl.representation import ScenarioVariantRep
from bdd_ros2_interfaces.msg import (
    Configuration,
    FluentStatus,
    ParamValue,
    ScenarioStatus,
)
from bdd_ros2_interfaces.msg import (
    Trinary as TrinaryMsg,
)
from bdd_ros2_interfaces.msg import (
    TrinaryStamped as TrinaryStampedMsg,
)
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.time import Time
from rdf_utils.models.execution import get_attr_path
from rdf_utils.models.geom_coord import get_transform_between_frames
from rdf_utils.naming import get_valid_var_name
from rdflib import Graph, URIRef
from scene_dsl.rdf_parser.scenex import (
    SceneInstanceModel,
    get_ros_pkg_path,
)
from scipy.spatial.transform import RigidTransform
from simulation_interfaces.msg import SpawnEntity as SpawnEntityMsg
from trinary import Trinary, Unknown
from unique_identifier_msgs.msg import UUID as UUIDMsg

S_TO_NS: Final = 1000 * 1000 * 1000
TRINARY_NAMES = {
    TrinaryMsg.TRUE: "TRUE",
    TrinaryMsg.FALSE: "FALSE",
    TrinaryMsg.UNKNOWN: "UNKNOWN",
}


def create_spawn_entity_msg(
    name: str,
    resource_uri: str,
    transform: RigidTransform,
    frame_id: str,
) -> SpawnEntityMsg:
    msg = SpawnEntityMsg()
    msg.name = name
    msg.allow_renaming = False
    msg.entity_resource.uri = resource_uri
    msg.initial_pose.header.frame_id = frame_id
    msg.initial_pose.pose.position.x = float(transform.translation[0])
    msg.initial_pose.pose.position.y = float(transform.translation[1])
    msg.initial_pose.pose.position.z = float(transform.translation[2])
    quaternion = transform.rotation.as_quat()
    msg.initial_pose.pose.orientation.x = float(quaternion[0])
    msg.initial_pose.pose.orientation.y = float(quaternion[1])
    msg.initial_pose.pose.orientation.z = float(quaternion[2])
    msg.initial_pose.pose.orientation.w = float(quaternion[3])
    return msg


def create_spawn_entity_entries(
    scene_inst: SceneInstanceModel,
    graph: Graph,
    resource_types: set[URIRef],
    *,
    world_entity_name: str = "world",
    additional_elements: set[URIRef] | None = None,
    warn: Callable[[str], object] | None = None,
) -> list[tuple[URIRef, SpawnEntityMsg]]:
    """Create spawn messages and retain their source scene element IDs."""
    if not resource_types:
        raise ValueError("resource_types must not be empty")

    ns_manager = graph.namespace_manager
    world_body = scene_inst.get_body_for_resource_entity(world_entity_name, graph)
    if world_body is None:
        raise ValueError(
            f"SceneInstance '{scene_inst.id.n3(ns_manager)}' has no body mapped to "
            f"simulator world entity '{world_entity_name}'"
        )

    rng = np.random.default_rng()
    entries = []
    element_ids = (
        scene_inst.scene_model.objects
        | scene_inst.scene_model.agents
        | (additional_elements or set())
    )
    for elem_id in element_ids:
        resolved = scene_inst.resolve_element_root_frame(elem_id, resource_types, graph)
        if resolved is None:
            if warn is not None:
                warn(
                    f"element '{elem_id.n3(ns_manager)}' has no compatible mapped kinematics resource"
                )
            continue

        resource, mapping, root = resolved
        sim_entity = mapping.entity
        if sim_entity is None:
            sim_entity = get_valid_var_name(elem_id.n3(ns_manager))
            if warn is not None:
                warn(
                    f"no sim entity specified for mapping of {elem_id.n3(ns_manager)}, "
                    f"converted from URI: {sim_entity}"
                )

        transform = get_transform_between_frames(
            root.id, world_body.root_frame.id, graph, rng=rng
        )
        if transform is None:
            if warn is not None:
                warn(
                    f"element '{elem_id.n3(ns_manager)}' has no pose path to '{world_entity_name}'"
                )
            continue

        ros_path = get_ros_pkg_path(resource)
        if ros_path is not None:
            resource_uri = (
                Path(get_package_share_directory(ros_path[0])) / ros_path[1]
            ).as_uri()
        else:
            resource_uri = get_attr_path(resource)
            if not resource_uri.startswith(("http://", "https://")):
                resource_uri = Path(resource_uri).expanduser().resolve().as_uri()

        entries.append(
            (
                elem_id,
                create_spawn_entity_msg(
                    sim_entity,
                    resource_uri,
                    transform,
                    frame_id=world_entity_name,
                ),
            )
        )
    return entries


def ros_time_to_stamp(t: Time) -> float:
    """Time to timestamp conversion, copied from rolling"""
    return t.nanoseconds / S_TO_NS


def format_time_msg(
    msg: TimeMsg,
    format_str: str = "%Y-%m-%d %H:%M:%S.%f %Z",
    use_sim_time: bool = False,
    num_decimals: int = 3,
) -> str:
    if not 1 <= num_decimals <= 9:
        raise ValueError(f"num_decimals must be between 1 and 9, got {num_decimals}")

    fraction = f"{msg.nanosec:09d}"[:num_decimals]
    if use_sim_time:
        hours, remainder = divmod(msg.sec, 3600)
        minutes, seconds = divmod(remainder, 60)
        return f"{hours:02d}:{minutes:02d}:{seconds:02d}.{fraction}"

    return datetime.fromtimestamp(msg.sec, tz=timezone.utc).strftime(
        format_str.replace("%f", fraction)
    )


def to_uuid_msg(uuid: UUID) -> UUIDMsg:
    uuid_msg = UUIDMsg()
    uuid_msg.uuid = bytearray(uuid.bytes)
    return uuid_msg


def from_uuid_msg(uuid_msg: UUIDMsg) -> UUID:
    return UUID(bytes=bytes(uuid_msg.uuid))


def from_trin_stamped_msg(msg: TrinaryStampedMsg) -> tuple[TrinaryStamped, UUID]:
    epoch_t = ros_time_to_stamp(Time.from_msg(msg.stamp))
    if msg.trinary.value == TrinaryMsg.FALSE:
        trin = False
    elif msg.trinary.value == TrinaryMsg.TRUE:
        trin = True
    elif msg.trinary.value == TrinaryMsg.UNKNOWN:
        trin = Unknown
    else:
        raise ValueError(f"Invalid trinary value in ROS message: {msg.trinary.value}")

    return TrinaryStamped(stamp=epoch_t, trinary=trin), from_uuid_msg(
        msg.scenario_context_id
    )


def to_trin_msg(trin: Trinary | bool) -> TrinaryMsg:
    trin_msg = TrinaryMsg()
    if trin is True:
        trin_msg.value = TrinaryMsg.TRUE
    elif trin is False:
        trin_msg.value = TrinaryMsg.FALSE
    elif trin is Unknown:
        trin_msg.value = TrinaryMsg.UNKNOWN
    else:
        raise ValueError(f"Invalid trinary value: {trin}")
    return trin_msg


def to_trin_stamped_msg(trin_st: TrinaryStamped) -> TrinaryStampedMsg:
    trin_st_msg = TrinaryStampedMsg()
    trin_st_msg.stamp = Time(seconds=trin_st.stamp).to_msg()
    trin_st_msg.trinary = to_trin_msg(trin_st.trinary)
    return trin_st_msg


def to_paramval_message(rel_uri: URIRef, val: Any) -> ParamValue:
    param = ParamValue()
    param.param_rel_uri = rel_uri.toPython()
    if isinstance(val, URIRef):
        param.param_val_uris = [val.toPython()]
        return param

    if isinstance(val, Iterable):
        val_uris = []
        for uri in val:
            assert isinstance(uri, URIRef), f"not an Iterable of URIRef: {uri}"
            val_uris.append(uri.toPython())
        param.param_val_uris = val_uris
        return param

    raise RuntimeError(
        f"get_valid_paramval_message: unhandled types: (type={type(val)}) {val}"
    )


def get_bhv_param_messages(
    when_bhv: WhenBehaviourModel, var_value_dict: dict[URIRef, Any]
) -> list[ParamValue]:
    param_vals = []
    if (
        URI_BHV_TYPE_PICK in when_bhv.behaviour.types
        or URI_BHV_TYPE_PLACE in when_bhv.behaviour.types
    ):
        obj_var_uri = when_bhv.get_attr(URI_BHV_PRED_TARGET_OBJ)
        assert obj_var_uri is not None
        assert obj_var_uri in var_value_dict, f"no value for '{obj_var_uri}'"
        param_vals.append(
            to_paramval_message(
                rel_uri=URI_BHV_PRED_TARGET_OBJ, val=var_value_dict[obj_var_uri]
            )
        )

        agn_var_uri = when_bhv.get_attr(URI_BHV_PRED_TARGET_AGN)
        assert agn_var_uri is not None
        assert agn_var_uri in var_value_dict, f"no value for '{agn_var_uri}'"
        param_vals.append(
            to_paramval_message(
                rel_uri=URI_BHV_PRED_TARGET_AGN, val=var_value_dict[agn_var_uri]
            )
        )

    if URI_BHV_TYPE_PLACE in when_bhv.behaviour.types:
        ws_var_uri = when_bhv.get_attr(URI_BHV_PRED_TARGET_WS)
        assert ws_var_uri is not None
        assert ws_var_uri in var_value_dict, f"no value for '{ws_var_uri}'"
        param_vals.append(
            to_paramval_message(
                rel_uri=URI_BHV_PRED_TARGET_WS, val=var_value_dict[ws_var_uri]
            )
        )

    return param_vals


def get_cfg_messages(
    scr_var: ScenarioVariantModel, var_value_dict: dict[URIRef, Any]
) -> list[Configuration]:
    configs = []
    for cfg_clause in scr_var.config_clauses():
        target_uri, name, var_uri = get_clause_config(clause=cfg_clause)
        assert var_uri in var_value_dict, (
            f"get_cfg_messages: no value for var '{var_uri}'"
        )
        var_val = var_value_dict[var_uri]
        assert isinstance(var_val, float), (
            f"get_cfg_messages: only float config supported, got '{var_val}' ({type(var_val)})"
        )
        cfg_msg = Configuration()
        cfg_msg.target = target_uri.toPython()
        cfg_msg.name = name
        cfg_msg.num_value = var_val
        configs.append(cfg_msg)

    return configs


def to_scenario_status_msg(
    ctx_id: UUID,
    obs_manager: ObservationManager,
    scr_rep: ScenarioVariantRep,
    now: Time,
    trinaries_policy: TrinariesPolicyProtocol,
) -> ScenarioStatus:
    scr_status = ScenarioStatus()
    scr_status.representation = scr_rep.variant_rep
    scr_status.context_id = to_uuid_msg(ctx_id)

    if obs_manager.scr_start_time is not None:
        scr_status.start_time = Time(seconds=obs_manager.scr_start_time).to_msg()
    if obs_manager.scr_end_time is not None:
        scr_status.end_time = Time(seconds=obs_manager.scr_end_time).to_msg()

    now_msg = now.to_msg()
    now_stamp = ros_time_to_stamp(now)

    scr_status.behaviour.representation = scr_rep.bhv_rep
    if obs_manager.bhv_result is None:
        scr_status.behaviour.result.stamp = now_msg
    else:
        scr_status.behaviour.result = to_trin_stamped_msg(
            trin_st=obs_manager.bhv_result,
        )

    scr_status.fluents = []
    fluent_results = []
    for obs_pol in obs_manager.obs_policies.values():
        fl_res = TrinaryStamped(
            stamp=now_stamp,
            trinary=trinaries_policy(obs_pol.trinary_timeline),
        )
        # Always set config result to true for now
        if URI_BDD_TYPE_CONFIG in obs_pol.fluent_types:
            fl_res.trinary = True
        fluent_results.append(fl_res)

        fl_status = FluentStatus()
        fl_status.representation = scr_rep.clause_rep(clause_id=obs_pol.fluent_id)
        if obs_pol.start_time is not None:
            fl_status.start_time = Time(seconds=obs_pol.start_time).to_msg()
        if obs_pol.end_time is not None:
            fl_status.end_time = Time(seconds=obs_pol.end_time).to_msg()
        fl_status.trinaries = [
            to_trin_stamped_msg(trin_st) for trin_st in obs_pol.trinary_timeline
        ]

        fl_status.result = to_trin_stamped_msg(fl_res)

        scr_status.fluents.append(fl_status)

    scr_status.result.stamp = now_msg
    if obs_manager.bhv_result is None:
        bhv_result = TrinaryStamped(stamp=now_stamp, trinary=Unknown)
    else:
        bhv_result = obs_manager.bhv_result
    fluent_results.append(bhv_result)
    scr_status.result.trinary = to_trin_msg(trinaries_policy(fluent_results))
    return scr_status
