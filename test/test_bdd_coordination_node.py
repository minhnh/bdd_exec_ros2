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
import threading
from collections import deque
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock, call, patch
from uuid import uuid4

from bdd_ros2_interfaces.msg import ScenarioStatus
from rdflib import URIRef

from bdd_exec_ros2.executables.bdd_coordination_node import (
    BddCoordNode,
    SceneSetupMode,
)


def _node(**attrs):
    node = BddCoordNode.__new__(BddCoordNode)
    node.__dict__.update(attrs)
    return node


def test_none_mode_keeps_immediate_variation_execution():
    variation = SimpleNamespace(task_variation=object())
    loader = Mock()
    loader.get_us_scenario_variants.return_value = {
        URIRef("urn:test:story"): [URIRef("urn:test:variation")]
    }
    loader.load_scenario_variant.return_value = variation
    node = _node(
        _scene_setup_mode=SceneSetupMode.NONE,
        us_loader=loader,
        graph=object(),
        _create_scenario_context=Mock(side_effect=["first", "second"]),
        _start_scenario_variant=Mock(),
        _schedule_next_scenario=Mock(),
    )
    values = [{URIRef("urn:test:variable"): URIRef("urn:test:first")}, {}]

    with patch(
        "bdd_exec_ros2.executables.bdd_coordination_node.get_task_var_dicts",
        return_value=values,
    ):
        node.start_test_cb(None)

    assert node._create_scenario_context.call_args_list == [
        call(variation, value) for value in values
    ]
    assert node._start_scenario_variant.call_args_list == [
        call(context, variation) for context in ["first", "second"]
    ]


def test_simulation_mode_queues_variations_and_ignores_another_start():
    variation = SimpleNamespace(task_variation=object())
    loader = Mock()
    loader.get_us_scenario_variants.return_value = {
        URIRef("urn:test:story"): [URIRef("urn:test:variation")]
    }
    loader.load_scenario_variant.return_value = variation
    logger = Mock()
    node = _node(
        _scene_setup_mode=SceneSetupMode.SIMULATION,
        _scene_setup_active=False,
        _pending_scenarios=deque(),
        _scenario_contexts={},
        _scr_lock=threading.Lock(),
        us_loader=loader,
        graph=object(),
        get_logger=Mock(return_value=logger),
        _schedule_next_scenario=Mock(),
    )
    values = [{}, {}]

    with patch(
        "bdd_exec_ros2.executables.bdd_coordination_node.get_task_var_dicts",
        return_value=values,
    ):
        node.start_test_cb(None)
        node.start_test_cb(None)

    assert list(node._pending_scenarios) == [
        (variation, values[0]),
        (variation, values[1]),
    ]
    node._schedule_next_scenario.assert_called_once_with()
    logger.warning.assert_called_once()


def test_none_mode_does_not_resolve_simulation_parameters():
    node = _node(
        get_parameter=Mock(
            return_value=SimpleNamespace(value=SceneSetupMode.NONE.value)
        )
    )

    mode, sim_interface = node._resolve_scene_setup()

    assert mode is SceneSetupMode.NONE
    assert sim_interface is None
    node.get_parameter.assert_called_once_with("scene_setup_mode")


def test_scene_setup_failure_cleans_up_and_continues():
    invariant = URIRef("urn:test:invariant")
    additional = URIRef("urn:test:additional")
    scene = SimpleNamespace(has_invariant_elem=lambda element: element == invariant)
    failed_variation = SimpleNamespace(id=URIRef("urn:test:failed"), scene=scene)
    started_variation = SimpleNamespace(id=URIRef("urn:test:started"), scene=scene)
    values = {URIRef("urn:test:variable"): [invariant, additional]}
    failed_context = SimpleNamespace(context_id=uuid4(), scene_inst=object())
    started_context = SimpleNamespace(context_id=uuid4(), scene_inst=object())
    sim_interface = SimpleNamespace(
        setup_scene=AsyncMock(side_effect=[RuntimeError("setup failed"), {}])
    )
    node = _node(
        _sim_interface=sim_interface,
        _pending_scenarios=deque(
            [(failed_variation, values), (started_variation, values)]
        ),
        _scenario_contexts={},
        _scene_setup_active=True,
        _scr_lock=threading.Lock(),
        _create_scenario_context=Mock(side_effect=[failed_context, started_context]),
        _remove_context_topic_reg=Mock(),
        _start_scenario_variant=Mock(),
        get_logger=Mock(return_value=Mock()),
    )

    asyncio.run(node._prepare_next_scenario())

    assert sim_interface.setup_scene.await_args_list == [
        call(failed_context.scene_inst, additional_elements={additional}),
        call(started_context.scene_inst, additional_elements={additional}),
    ]
    node._remove_context_topic_reg.assert_called_once_with(failed_context.context_id)
    node._start_scenario_variant.assert_called_once_with(
        started_context, started_variation
    )
    assert not node._scene_setup_active


def test_completed_context_advances_the_setup_queue():
    context_id = uuid4()
    context = SimpleNamespace(
        context_id=context_id,
        obs_manager=SimpleNamespace(scr_end_time=1.0),
        scr_rep=object(),
    )
    now = SimpleNamespace(to_msg=Mock(return_value=object()))
    node = _node(
        _scenario_contexts={context_id: context},
        _scr_lock=threading.Lock(),
        _scr_status_pub=Mock(),
        _schedule_next_scenario=Mock(),
        get_clock=Mock(return_value=SimpleNamespace(now=Mock(return_value=now))),
        get_logger=Mock(return_value=Mock()),
    )

    with patch(
        "bdd_exec_ros2.executables.bdd_coordination_node.to_scenario_status_msg",
        return_value=ScenarioStatus(),
    ):
        node._status_timer_callback()

    assert node._scenario_contexts == {}
    node._schedule_next_scenario.assert_called_once_with()
