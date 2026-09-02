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
from rdflib import URIRef
from trinary import Unknown

from bdd_exec_ros2.conversions import to_scenario_status_msg, to_uuid_msg
from bdd_exec_ros2.executables.web_visualizer import (
    TimelineStore,
    _revalidate_ui,
    create_app,
    status_dict,
)

CONTEXT_ID = UUID("01234567-89ab-cdef-0123-456789abcdef")


def stamp(seconds: int) -> Time:
    return Time(sec=seconds)


def scenario_event(seconds: int, uri: str = "urn:bdd:event:grasped") -> Event:
    result = Event()
    result.scenario_context_id = to_uuid_msg(CONTEXT_ID)
    result.stamp = stamp(seconds)
    result.uri = uri
    return result


def assertion(seconds: int, value: int = Trinary.TRUE) -> TrinaryStamped:
    result = TrinaryStamped()
    result.scenario_context_id = to_uuid_msg(CONTEXT_ID)
    result.stamp = stamp(seconds)
    result.trinary.value = value
    result.reason = "observed"
    return result


def status(
    *trinaries: TrinaryStamped, events: tuple[Event, ...] = ()
) -> ScenarioStatusList:
    fluent = FluentStatus()
    fluent.uri = "urn:bdd:policy:held"
    fluent.representation = "the object is held"
    fluent.trinaries = list(trinaries)

    scenario = ScenarioStatus()
    scenario.context_id = to_uuid_msg(CONTEXT_ID)
    scenario.start_time = stamp(1)
    scenario.representation = "Pick the object"
    scenario.behaviour.representation = "pick"
    scenario.fluents = [fluent]
    scenario.events = list(events)

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
    assert scenario["fluents"][0]["uri"] == "urn:bdd:policy:held"
    assert observed["value"] == "false"
    assert observed["reason"] == "observed"


def test_behaviour_result_stamp_is_unset_until_a_terminal_trinary_exists():
    obs_manager = SimpleNamespace(
        scr_start_time=1.0,
        scr_end_time=None,
        bhv_result=None,
        obs_policies={},
        event_timelines={},
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


def test_policy_uri_distinguishes_identical_clause_representations():
    policy = SimpleNamespace(
        fluent_id=URIRef("urn:bdd:fluent:held"),
        fluent_types=set(),
        start_time=None,
        end_time=None,
        trinary_timeline=[],
        get_result=lambda **_: DslTrinaryStamped(2.0, Unknown, "no observations"),
    )
    obs_manager = SimpleNamespace(
        scr_start_time=1.0,
        scr_end_time=None,
        bhv_result=None,
        obs_policies={
            URIRef("urn:bdd:policy:ground-truth"): policy,
            URIRef("urn:bdd:policy:perception"): policy,
        },
        event_timelines={},
    )
    representation = SimpleNamespace(
        variant_rep="Pick the object",
        bhv_rep="pick",
        clause_rep=lambda clause_id: "Then the object is held",
    )

    scenario = to_scenario_status_msg(
        CONTEXT_ID,
        obs_manager,
        representation,
        RclpyTime(seconds=2),
        lambda _: (Unknown, ""),
    )

    assert [(fluent.uri, fluent.representation) for fluent in scenario.fluents] == [
        ("urn:bdd:policy:ground-truth", "Then the object is held"),
        ("urn:bdd:policy:perception", "Then the object is held"),
    ]

    store = TimelineStore()
    store.ingest_status(ScenarioStatusList(stamp=stamp(2), scenarios=[scenario]))
    policy_records = [
        record for record in store.records if record.get("lane_type") == "policy"
    ]
    assert [record["uri"] for record in policy_records] == [
        "urn:bdd:policy:ground-truth",
        "urn:bdd:policy:perception",
    ]


def test_status_conversion_embeds_events():
    obs_manager = SimpleNamespace(
        scr_start_time=1.0,
        scr_end_time=None,
        bhv_result=None,
        obs_policies={},
        event_timelines={
            URIRef("urn:bdd:event:z"): [3.0, 1.0, 3.0],
            URIRef("urn:bdd:event:a"): [3.0],
        },
    )
    representation = SimpleNamespace(
        variant_rep="Pick the object",
        bhv_rep="pick",
    )

    result = to_scenario_status_msg(
        CONTEXT_ID,
        obs_manager,
        representation,
        RclpyTime(seconds=4),
        lambda _: (Unknown, ""),
    )

    assert sorted(
        (event.uri, event.stamp.sec, event.stamp.nanosec) for event in result.events
    ) == [
        ("urn:bdd:event:a", 3, 0),
        ("urn:bdd:event:z", 1, 0),
        ("urn:bdd:event:z", 3, 0),
        ("urn:bdd:event:z", 3, 0),
    ]
    assert all(
        event.scenario_context_id == to_uuid_msg(CONTEXT_ID) for event in result.events
    )


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
    message = status(events=(scenario_event(4),))
    store = TimelineStore()

    store.ingest_status(message)
    store.ingest_status(message)

    serialized = status_dict(message)["scenarios"][0]["events"][0]
    event_records = [record for record in store.records if record["kind"] == "event"]
    assert serialized["context_id"] == str(CONTEXT_ID)
    assert len(event_records) == 1
    assert event_records[0]["label"] == "urn:bdd:event:grasped"


def test_ui_assets_are_revalidated():
    async def check() -> None:
        async def handler(_: web.Request) -> web.StreamResponse:
            return web.Response()

        asset = await _revalidate_ui(
            make_mocked_request("GET", "/assets/app.mjs"), handler
        )
        health = await _revalidate_ui(make_mocked_request("GET", "/healthz"), handler)

        assert asset.headers["Cache-Control"] == "no-cache"
        assert "Cache-Control" not in health.headers

    asyncio.run(check())


def test_only_runtime_assets_are_served():
    app = create_app()
    paths = {resource.canonical for resource in app.router.resources()}

    assert "event_topic" not in app
    assert "/assets/app.mjs" in paths
    assert "/assets/timeline.mjs" in paths
    assert "/assets/styles.css" in paths
    assert "/assets/timeline.test.mjs" not in paths
