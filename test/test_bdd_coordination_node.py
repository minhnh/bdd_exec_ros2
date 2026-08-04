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

from bdd_dsl.models.urirefs import (
    URI_ROS_PRED_CHNL_NAME,
    URI_ROS_PRED_TYPE_NAME,
    URI_ROS_TYPE_TOPIC,
)
from bdd_ros2_interfaces.msg import ScenarioStatus
from rdflib import URIRef
from vision_msgs.msg import Detection3D

from bdd_exec_ros2.conversions import TRINARY_NAMES
from bdd_exec_ros2.executables.bdd_coordination_node import (
    BddCoordNode,
    SceneSetupMode,
    _load_topic_observation_adapters,
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
        _schedule_next_scenario=Mock(),
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
    node._schedule_next_scenario.assert_called_once_with()


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
        _remove_context_topic_reg=Mock(),
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
    node._remove_context_topic_reg.assert_called_once_with(context_id=context_id)
    node._schedule_next_scenario.assert_called_once_with()


def test_behaviour_result_is_recorded_before_scenario_ends():
    context_id = uuid4()
    trinary = SimpleNamespace(value=next(iter(TRINARY_NAMES)))
    result = SimpleNamespace(
        result=SimpleNamespace(
            scenario_context_id=object(),
            trinary=trinary,
        )
    )
    obs_manager = SimpleNamespace(
        scenario_exec=SimpleNamespace(end_event=URIRef("urn:test:end")),
        update_bhv_result=Mock(),
    )
    node = _node(
        _scenario_contexts={context_id: SimpleNamespace(obs_manager=obs_manager)},
        _scr_lock=threading.Lock(),
        _send_event=Mock(
            side_effect=lambda **_: obs_manager.update_bhv_result.assert_called_once()
        ),
        get_logger=Mock(return_value=Mock()),
    )

    with (
        patch(
            "bdd_exec_ros2.executables.bdd_coordination_node.from_uuid_msg",
            return_value=context_id,
        ),
        patch(
            "bdd_exec_ros2.executables.bdd_coordination_node.from_trin_stamped_msg",
            return_value=(object(), context_id),
        ),
    ):
        node.bhv_result_cb(
            SimpleNamespace(result=Mock(return_value=SimpleNamespace(result=result))),
            context_id,
        )

    node._send_event.assert_called_once()


def test_topic_observation_forwards_raw_message_with_receipt_stamp():
    provider_uri = URIRef("urn:test:provider")
    context_id = uuid4()
    topic_key = ("observations", object)
    obs_manager = SimpleNamespace(update_provider_observation=Mock())
    node = _node(
        _scr_lock=threading.Lock(),
        _scenario_contexts={context_id: SimpleNamespace(obs_manager=obs_manager)},
        _topic_observation_reg={topic_key: {context_id: {provider_uri}}},
        get_clock=Mock(
            return_value=SimpleNamespace(
                now=Mock(return_value=SimpleNamespace(nanoseconds=7_000_000_000))
            )
        ),
    )
    msg = SimpleNamespace()

    node._update_observation(topic_key, msg)

    obs_manager.update_provider_observation.assert_called_once_with(
        provider_uri, msg, 7.0
    )


def test_topic_provider_uses_its_message_type_adapter():
    provider_uri = URIRef("urn:test:provider")
    context_id = uuid4()
    msg_type = type("TestMessage", (), {})
    timestamp_extractor = Mock()
    entity_mapper = Mock()
    obs_manager = SimpleNamespace(register_provider=Mock())
    provider = SimpleNamespace(
        id=provider_uri,
        types={URI_ROS_TYPE_TOPIC},
        get_attr=lambda key: {
            URI_ROS_PRED_CHNL_NAME: "observations",
            URI_ROS_PRED_TYPE_NAME: msg_type,
        }[key],
    )
    node = _node(
        _topic_observation_adapters={msg_type: (timestamp_extractor, entity_mapper)},
        _topic_observation_reg={},
        _observation_subs={},
        _scr_lock=threading.Lock(),
        _obs_cb_group=object(),
        create_subscription=Mock(return_value=object()),
    )

    node._create_observation_subscription(provider, context_id, obs_manager)

    obs_manager.register_provider.assert_called_once_with(
        provider_uri, timestamp_extractor, entity_mapper
    )


def test_topic_observation_adapter_module_attribute_loads_mockup_adapter():
    adapters = _load_topic_observation_adapters(
        "bdd_exec_ros2.executables.mockup_behaviour_node:TOPIC_OBSERVATION_ADAPTERS"
    )

    timestamp_extractor, entity_mapper = adapters[Detection3D]
    assert callable(timestamp_extractor)
    assert callable(entity_mapper)
