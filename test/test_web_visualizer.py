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
from types import SimpleNamespace
from uuid import UUID

from aiohttp import web
from aiohttp.test_utils import make_mocked_request
from bdd_dsl.models.observation import TrinaryStamped as DslTrinaryStamped
from bdd_ros2_interfaces.msg import (
    Event,
    FluentStatus,
    ScenarioStatus,
    ScenarioStatusList,
    Trinary,
    TrinaryStamped,
)
from builtin_interfaces.msg import Time
from rclpy.time import Time as RclpyTime
from trinary import Unknown

from bdd_exec_ros2.conversions import to_scenario_status_msg, to_uuid_msg
from bdd_exec_ros2.executables.web_visualizer import (
    TimelineStore,
    _no_store_ui,
    event_dict,
    status_dict,
)

CONTEXT_ID = UUID("01234567-89ab-cdef-0123-456789abcdef")


def stamp(seconds: int) -> Time:
    return Time(sec=seconds)


def assertion(seconds: int, value: int = Trinary.TRUE) -> TrinaryStamped:
    result = TrinaryStamped()
    result.scenario_context_id = to_uuid_msg(CONTEXT_ID)
    result.stamp = stamp(seconds)
    result.trinary.value = value
    result.reason = "observed"
    return result


def status(*trinaries: TrinaryStamped) -> ScenarioStatusList:
    fluent = FluentStatus()
    fluent.representation = "the object is held"
    fluent.trinaries = list(trinaries)

    scenario = ScenarioStatus()
    scenario.context_id = to_uuid_msg(CONTEXT_ID)
    scenario.start_time = stamp(1)
    scenario.representation = "Pick the object"
    scenario.behaviour.representation = "pick"
    scenario.fluents = [fluent]

    message = ScenarioStatusList()
    message.stamp = stamp(3)
    message.scenarios = [scenario]
    return message


def test_ros_messages_are_serialized_to_the_browser_contract():
    message = status(assertion(2, Trinary.FALSE))

    serialized = status_dict(message)
    scenario = serialized["scenarios"][0]
    observed = scenario["fluents"][0]["trinaries"][0]

    assert scenario["context_id"] == str(CONTEXT_ID)
    assert scenario["start_time"] == {"sec": 1, "nanosec": 0}
    assert observed["value"] == "false"
    assert observed["reason"] == "observed"


def test_behaviour_result_stamp_is_unset_until_a_terminal_trinary_exists():
    obs_manager = SimpleNamespace(
        scr_start_time=1.0,
        scr_end_time=None,
        bhv_result=None,
        obs_policies={},
    )
    representation = SimpleNamespace(
        variant_rep="Pick the object",
        bhv_rep="pick",
    )

    def policy(_):
        return Unknown, ""

    running = to_scenario_status_msg(
        CONTEXT_ID, obs_manager, representation, RclpyTime(seconds=2), policy
    )
    assert running.behaviour.result.stamp == Time()

    expected = (
        (True, Trinary.TRUE),
        (False, Trinary.FALSE),
        (Unknown, Trinary.UNKNOWN),
    )
    for value, message_value in expected:
        obs_manager.bhv_result = DslTrinaryStamped(
            stamp=2.5,
            trinary=value,
            reason="finished",
        )
        terminal = to_scenario_status_msg(
            CONTEXT_ID, obs_manager, representation, RclpyTime(seconds=3), policy
        )
        assert terminal.behaviour.result.stamp == Time(sec=2, nanosec=500_000_000)
        assert terminal.behaviour.result.trinary.value == message_value


def test_behaviour_timeline_ignores_running_placeholder_and_keeps_terminal_unknown():
    store = TimelineStore()
    store.ingest_status(status())
    terminal = status()
    terminal.scenarios[0].behaviour.result = assertion(4, Trinary.UNKNOWN)
    store.ingest_status(terminal)

    behaviour = [
        record for record in store.records if record.get("lane_type") == "behaviour"
    ]
    assert [(record["value"], record["stamp"]) for record in behaviour] == [
        ("unknown", {"sec": 4, "nanosec": 0})
    ]


def test_timeline_history_is_deduplicated_and_marks_discarded_observations():
    store = TimelineStore()
    first = status(assertion(2))

    store.ingest_status(first)
    store.ingest_status(first)
    store.ingest_status(status())

    assert [record["kind"] for record in store.records] == [
        "scenario_start",
        "trinary",
        "trinary_discarded",
    ]
    assert store.records[-1]["target_id"] == store.records[1]["id"]
    assert [record["sequence"] for record in store.records] == [1, 2, 3]


def test_events_remain_context_scoped_and_are_deduplicated():
    message = Event()
    message.scenario_context_id = to_uuid_msg(CONTEXT_ID)
    message.stamp = stamp(4)
    message.uri = "urn:bdd:event:grasped"
    store = TimelineStore()

    store.ingest_event(message)
    store.ingest_event(message)

    assert event_dict(message)["context_id"] == str(CONTEXT_ID)
    assert len(store.records) == 1
    assert store.records[0]["label"] == message.uri


def test_ui_assets_are_not_cached():
    async def check() -> None:
        async def handler(_: web.Request) -> web.StreamResponse:
            return web.Response()

        asset = await _no_store_ui(
            make_mocked_request("GET", "/assets/app.mjs"), handler
        )
        health = await _no_store_ui(make_mocked_request("GET", "/healthz"), handler)

        assert asset.headers["Cache-Control"] == "no-store"
        assert "Cache-Control" not in health.headers

    asyncio.run(check())
