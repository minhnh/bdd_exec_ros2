#!/usr/bin/env python3

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

from typing import Any, Final, Iterable
from uuid import UUID
from datetime import datetime
from bdd_dsl.models.clauses import WhenBehaviourModel, get_clause_config
from bdd_dsl.models.urirefs import (
    URI_BDD_TYPE_CONFIG,
    URI_BHV_PRED_TARGET_AGN,
    URI_BHV_PRED_TARGET_OBJ,
    URI_BHV_TYPE_PICK,
    URI_BHV_TYPE_PLACE,
)
from bdd_dsl.models.user_story import ScenarioVariantModel
from trinary import Trinary, Unknown
from rdflib import URIRef

from rclpy.time import Time
from unique_identifier_msgs.msg import UUID as UUIDMsg
from builtin_interfaces.msg import Time as TimeMsg

from bdd_dsl.models.observation import (
    ObservationManager,
    TrinariesPolicyProtocol,
    TrinaryStamped,
)

from bdd_ros2_interfaces.msg import (
    Configuration,
    FluentStatus,
    ParamValue,
    ScenarioStatus,
    TrinaryStamped as TrinaryStampedMsg,
    Trinary as TrinaryMsg,
)


S_TO_NS: Final = 1000 * 1000 * 1000


def ros_time_to_stamp(t: Time) -> float:
    """Time to timestamp conversion, copied from rolling"""
    return t.nanoseconds / S_TO_NS


def format_time_msg(msg: TimeMsg, format_str: str = "%Y-%m-%d %H:%M:%S.%f") -> str:
    return datetime.fromtimestamp(ros_time_to_stamp(Time.from_msg(msg))).strftime(
        format_str
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
    if URI_BHV_TYPE_PICK or URI_BHV_TYPE_PLACE in when_bhv.types:
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
    now: Time,
    trinaries_policy: TrinariesPolicyProtocol,
) -> ScenarioStatus:
    scr_status = ScenarioStatus()
    scr_status.representation = obs_manager.scr_rep
    scr_status.context_id = to_uuid_msg(ctx_id)

    if obs_manager.scr_start_time is not None:
        scr_status.start_time = Time(seconds=obs_manager.scr_start_time).to_msg()
    if obs_manager.scr_end_time is not None:
        scr_status.end_time = Time(seconds=obs_manager.scr_end_time).to_msg()

    now_msg = now.to_msg()
    now_stamp = ros_time_to_stamp(now)

    scr_status.behaviour.representation = obs_manager.bhv_rep
    if obs_manager.bhv_result is None:
        scr_status.behaviour.result.stamp = now_msg
    else:
        scr_status.behaviour.result = to_trin_stamped_msg(
            trin_st=obs_manager.bhv_result,
        )

    scr_status.fluents = []
    fluent_results = []
    for fl_tl in obs_manager.fluent_timelines.values():
        fl_res = TrinaryStamped(
            stamp=now_stamp,
            trinary=trinaries_policy(fl_tl.trinary_timeline),
        )
        # Always set config result to true for now
        if URI_BDD_TYPE_CONFIG in fl_tl.fluent_types:
            fl_res.trinary = True
        fluent_results.append(fl_res)

        fl_status = FluentStatus()
        fl_status.representation = fl_tl.representation
        if fl_tl.start_time is not None:
            fl_status.start_time = Time(seconds=fl_tl.start_time).to_msg()
        if fl_tl.end_time is not None:
            fl_status.end_time = Time(seconds=fl_tl.end_time).to_msg()
        fl_status.trinaries = [
            to_trin_stamped_msg(trin_st) for trin_st in fl_tl.trinary_timeline
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
