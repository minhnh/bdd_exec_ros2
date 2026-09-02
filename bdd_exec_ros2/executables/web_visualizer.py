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

import argparse
import asyncio
import json
import threading
import uuid
from contextlib import suppress
from functools import partial
from pathlib import Path
from typing import Any

import rclpy
from aiohttp import WSMsgType, web
from bdd_ros2_interfaces.msg import ScenarioStatusList, Trinary
from builtin_interfaces.msg import Time
from rclpy.executors import ExternalShutdownException

from bdd_exec_ros2.conversions import from_uuid_msg

RUNTIME_ASSETS = ("styles.css", "app.mjs", "timeline.mjs")
ASSET_DIR = Path(__file__).parents[1] / "web"
TRINARY_NAMES = {
    Trinary.TRUE: "true",
    Trinary.FALSE: "false",
    Trinary.UNKNOWN: "unknown",
}


def time_dict(msg: Time) -> dict[str, int]:
    return {"sec": int(msg.sec), "nanosec": int(msg.nanosec)}


def has_time(msg: Time) -> bool:
    return bool(msg.sec or msg.nanosec)


def trinary_dict(msg) -> dict[str, Any]:
    return {
        "stamp": time_dict(msg.stamp),
        "value": TRINARY_NAMES.get(msg.trinary.value, f"invalid:{msg.trinary.value}"),
        "reason": msg.reason,
    }


def status_dict(msg: ScenarioStatusList) -> dict[str, Any]:
    return {
        "stamp": time_dict(msg.stamp),
        "scenarios": [
            {
                "context_id": str(from_uuid_msg(scenario.context_id)),
                "representation": scenario.representation,
                "start_time": time_dict(scenario.start_time),
                "end_time": time_dict(scenario.end_time),
                "result": trinary_dict(scenario.result),
                "behaviour": {
                    "representation": scenario.behaviour.representation,
                    "result": trinary_dict(scenario.behaviour.result),
                },
                "events": [
                    {
                        "context_id": str(from_uuid_msg(event.scenario_context_id)),
                        "stamp": time_dict(event.stamp),
                        "uri": event.uri,
                    }
                    for event in scenario.events
                ],
                "fluents": [
                    {
                        "uri": fluent.uri,
                        "representation": fluent.representation,
                        "start_time": time_dict(fluent.start_time),
                        "end_time": time_dict(fluent.end_time),
                        "result": trinary_dict(fluent.result),
                        "trinaries": [trinary_dict(item) for item in fluent.trinaries],
                    }
                    for fluent in scenario.fluents
                ],
            }
            for scenario in msg.scenarios
        ],
    }


def _time_key(msg: Time) -> tuple[int, int]:
    return int(msg.sec), int(msg.nanosec)


class TimelineStore:
    """Keep the current snapshot and lossless history for this process."""

    def __init__(self) -> None:
        self.snapshot: dict[str, Any] = {
            "stamp": {"sec": 0, "nanosec": 0},
            "scenarios": [],
        }
        self.records: list[dict[str, Any]] = []
        self.use_sim_time = False
        self._sequence = 0
        self._seen: dict[tuple[Any, ...], str] = {}
        self._active: dict[tuple[str, str], set[str]] = {}
        self._clients: set[asyncio.Queue[dict[str, Any]]] = set()

    def add_client(self) -> asyncio.Queue[dict[str, Any]]:
        queue: asyncio.Queue[dict[str, Any]] = asyncio.Queue(maxsize=32)
        self._clients.add(queue)
        queue.put_nowait(
            {
                "type": "hello",
                "protocol": 1,
                "mode": "live",
                "use_sim_time": self.use_sim_time,
            }
        )
        queue.put_nowait(self._snapshot_message(include_history=True))
        return queue

    def remove_client(self, queue: asyncio.Queue[dict[str, Any]]) -> None:
        self._clients.discard(queue)

    def ingest_status(self, msg: ScenarioStatusList) -> None:
        self.snapshot = status_dict(msg)
        new_records: list[dict[str, Any]] = []

        for scenario in msg.scenarios:
            context_id = str(from_uuid_msg(scenario.context_id))
            if has_time(scenario.start_time):
                self._add(
                    ("scenario_start", context_id, _time_key(scenario.start_time)),
                    new_records,
                    kind="scenario_start",
                    context_id=context_id,
                    stamp=time_dict(scenario.start_time),
                    label=scenario.representation,
                )
            for event in scenario.events:
                self._add(
                    ("event", context_id, event.uri, _time_key(event.stamp)),
                    new_records,
                    kind="event",
                    context_id=context_id,
                    stamp=time_dict(event.stamp),
                    uri=event.uri,
                )

            self._add_trinary(
                new_records,
                context_id,
                "behaviour",
                scenario.behaviour.representation,
                "result",
                scenario.behaviour.result,
            )

            for fluent in scenario.fluents:
                active_key = (context_id, fluent.uri)
                fluent_start = (
                    _time_key(fluent.start_time)
                    if has_time(fluent.start_time)
                    else None
                )
                fluent_end = (
                    _time_key(fluent.end_time) if has_time(fluent.end_time) else None
                )
                current: set[str] = set()
                for item in fluent.trinaries:
                    item_time = _time_key(item.stamp)
                    if fluent_start is not None and item_time < fluent_start:
                        continue
                    if fluent_end is not None and item_time > fluent_end:
                        continue
                    record_id = self._add_trinary(
                        new_records,
                        context_id,
                        "policy",
                        fluent.representation,
                        "assertion",
                        item,
                        uri=fluent.uri,
                    )
                    if record_id is not None:
                        current.add(record_id)

                self._add_trinary(
                    new_records,
                    context_id,
                    "policy",
                    fluent.representation,
                    "result",
                    fluent.result,
                    uri=fluent.uri,
                )

                for record_id in self._active.get(active_key, set()) - current:
                    self._add(
                        ("trinary_discarded", record_id),
                        new_records,
                        kind="trinary_discarded",
                        context_id=context_id,
                        stamp=time_dict(msg.stamp),
                        target_id=record_id,
                    )
                self._active[active_key] = current

            if has_time(scenario.end_time):
                result = trinary_dict(scenario.result)
                self._add(
                    ("scenario_end", context_id, _time_key(scenario.end_time)),
                    new_records,
                    kind="scenario_end",
                    context_id=context_id,
                    stamp=time_dict(scenario.end_time),
                    label=scenario.representation,
                    value=result["value"],
                    reason=result["reason"],
                )

        self._publish(self._snapshot_message())
        for record in new_records:
            self._publish({"type": "timeline_record", "record": record})

    def _add_trinary(
        self,
        output: list[dict[str, Any]],
        context_id: str,
        lane_type: str,
        label: str,
        role: str,
        msg,
        uri: str = "",
    ) -> str | None:
        if not has_time(msg.stamp):
            return None
        data = trinary_dict(msg)
        key = (
            "trinary",
            context_id,
            lane_type,
            uri or label,
            role,
            _time_key(msg.stamp),
            data["value"],
            data["reason"],
        )
        record = self._add(
            key,
            output,
            kind="trinary",
            context_id=context_id,
            stamp=data["stamp"],
            lane_type=lane_type,
            uri=uri,
            label=label,
            role=role,
            value=data["value"],
            reason=data["reason"],
        )
        return record["id"] if record is not None else self._seen[key]

    def _add(
        self,
        key: tuple[Any, ...],
        output: list[dict[str, Any]],
        **values: Any,
    ) -> dict[str, Any] | None:
        if key in self._seen:
            return None
        self._sequence += 1
        record = {"id": f"r{self._sequence}", "sequence": self._sequence, **values}
        self._seen[key] = record["id"]
        self.records.append(record)
        output.append(record)
        return record

    def _snapshot_message(self, *, include_history: bool = False) -> dict[str, Any]:
        message: dict[str, Any] = {"type": "snapshot", "snapshot": self.snapshot}
        if include_history:
            message["history"] = list(self.records)
        return message

    def _publish(self, message: dict[str, Any]) -> None:
        for queue in self._clients:
            try:
                queue.put_nowait(message)
            except asyncio.QueueFull:
                while not queue.empty():
                    queue.get_nowait()
                queue.put_nowait(self._snapshot_message(include_history=True))


@web.middleware
async def _revalidate_ui(request: web.Request, handler: Any) -> web.StreamResponse:
    response = await handler(request)
    if request.path == "/" or request.path.startswith("/assets/"):
        response.headers["Cache-Control"] = "no-cache"
    return response


async def _index(_: web.Request) -> web.FileResponse:
    return web.FileResponse(ASSET_DIR / "index.html")


async def _asset(_: web.Request, name: str) -> web.FileResponse:
    return web.FileResponse(ASSET_DIR / name)


async def _health(request: web.Request) -> web.Response:
    store: TimelineStore = request.app["store"]
    return web.json_response({"ok": True, "records": len(store.records)})


async def _send_messages(
    socket: web.WebSocketResponse, queue: asyncio.Queue[dict[str, Any]]
) -> None:
    while True:
        await socket.send_json(await queue.get())


async def _websocket(request: web.Request) -> web.WebSocketResponse:
    socket = web.WebSocketResponse(heartbeat=20)
    await socket.prepare(request)
    store: TimelineStore = request.app["store"]
    queue = store.add_client()
    sender = asyncio.create_task(_send_messages(socket, queue))
    try:
        async for message in socket:
            if message.type is WSMsgType.TEXT:
                try:
                    data = json.loads(message.data)
                except json.JSONDecodeError:
                    data = {}
                await socket.send_json(
                    {
                        "type": "error",
                        "message": f"unsupported client message: {data.get('type')!r}",
                    }
                )
    finally:
        store.remove_client(queue)
        sender.cancel()
        with suppress(asyncio.CancelledError):
            await sender
    return socket


def _spin(node) -> None:
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


async def _start_ros(app: web.Application) -> None:
    if not rclpy.ok():
        rclpy.init(args=app["ros_args"])
    node = rclpy.create_node(f"bdd_web_viz_{uuid.uuid4().hex[:8]}")
    store: TimelineStore = app["store"]
    store.use_sim_time = bool(node.get_parameter("use_sim_time").value)
    loop = asyncio.get_running_loop()

    def ingest_status(msg: ScenarioStatusList) -> None:
        loop.call_soon_threadsafe(store.ingest_status, msg)

    node.create_subscription(
        ScenarioStatusList,
        app["status_topic"],
        ingest_status,
        10,
    )
    node.get_logger().info(f"Web visualizer subscribing to {app['status_topic']}")
    thread = threading.Thread(target=_spin, args=(node,), daemon=True)
    thread.start()
    app["ros_node"] = node
    app["ros_thread"] = thread


async def _stop_ros(app: web.Application) -> None:
    if rclpy.ok():
        rclpy.shutdown()
    thread: threading.Thread | None = app.get("ros_thread")
    if thread is not None:
        thread.join(timeout=2)
    node = app.get("ros_node")
    if node is not None:
        node.destroy_node()


def create_app(
    status_topic: str = "/bdd/status",
    ros_args: list[str] | None = None,
) -> web.Application:
    app = web.Application(middlewares=[_revalidate_ui])
    app["store"] = TimelineStore()
    app["status_topic"] = status_topic
    app["ros_args"] = ros_args
    app.router.add_get("/", _index)
    app.router.add_get("/healthz", _health)
    app.router.add_get("/ws", _websocket)
    for name in RUNTIME_ASSETS:
        app.router.add_get(f"/assets/{name}", partial(_asset, name=name))
    app.on_startup.append(_start_ros)
    app.on_cleanup.append(_stop_ros)
    return app


def main() -> None:
    parser = argparse.ArgumentParser(
        description="BDD web visualization tool",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("-t", "--topic", default="/bdd/status")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8080, type=int)
    args, ros_args = parser.parse_known_args()
    web.run_app(
        create_app(status_topic=args.topic, ros_args=ros_args),
        host=args.host,
        port=args.port,
        print=lambda _: print(f"BDD timeline: http://{args.host}:{args.port}"),
    )


if __name__ == "__main__":
    main()
