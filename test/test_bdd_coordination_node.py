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

from bdd_ros2_interfaces.msg import Event, ScenarioStatus
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time
from rdf_utils.models.vocab import URI_ROS_TYPE_SIM_ENTITY_STATE_PROVIDER
from rdflib import Graph, URIRef

from bdd_exec_ros2.conversions import TRINARY_NAMES, ros_time_to_stamp
from bdd_exec_ros2.executables.bdd_coordination_node import (
    BddCoordNode,
    SceneSetupMode,
)


def _node(**attrs):
    node = BddCoordNode.__new__(BddCoordNode)
    node.__dict__.update(attrs)
    return node


def test_empty_transform_snapshot_is_forwarded_for_cache_invalidation():
    context_id = uuid4()
    provider_uri = URIRef("urn:test:tf-provider")
    manager = Mock()
    manager.update_provider_observation.return_value = {}
    node = _node(
        _scr_lock=threading.Lock(),
        _scenario_contexts={context_id: SimpleNamespace(obs_manager=manager)},
        get_clock=Mock(
            return_value=SimpleNamespace(now=Mock(return_value=Time(seconds=2.0)))
        ),
    )

    node._update_transform_observation(context_id, provider_uri, {})

    manager.update_provider_observation.assert_called_once_with(
        provider_uri,
        {},
        2.0,
    )


def test_context_free_event_is_recorded_for_each_active_scenario():
    context_ids = (uuid4(), uuid4())
    managers = (Mock(), Mock())
    node = _node(
        _scr_lock=threading.Lock(),
        _scenario_contexts={
            context_id: SimpleNamespace(obs_manager=manager)
            for context_id, manager in zip(context_ids, managers, strict=True)
        },
        _ns_manager=Graph().namespace_manager,
        _use_sim_time=False,
        get_logger=Mock(return_value=Mock()),
    )
    message = Event()
    message.uri = "urn:test:event"
    message.stamp = Time(seconds=4).to_msg()

    node.evt_sub_cb(message)

    for manager in managers:
        manager.on_event.assert_called_once_with(
            evt_uri=URIRef("urn:test:event"),
            evt_t=4.0,
        )


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
    now = Time(seconds=2.0)
    context = SimpleNamespace(
        context_id=context_id,
        obs_manager=SimpleNamespace(
            scr_start_time=0.0,
            scr_end_time=1.0,
            obs_policies={},
            pending_end_deadline=None,
        ),
        scr_rep=object(),
        end_event_sent=False,
    )
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


def test_completed_context_is_retained_during_observation_horizon():
    context_id = uuid4()
    now = Time(seconds=2.0)
    context = SimpleNamespace(
        context_id=context_id,
        obs_manager=SimpleNamespace(
            scr_start_time=0.0,
            scr_end_time=1.0,
            obs_policies={URIRef("urn:test:policy"): SimpleNamespace(end_time=3.0)},
            pending_end_deadline=None,
        ),
        scr_rep=object(),
        end_event_sent=False,
    )
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

    assert node._scenario_contexts == {context_id: context}
    node._remove_context_topic_reg.assert_not_called()

    node.get_clock.return_value.now.return_value = Time(seconds=4.0)
    with patch(
        "bdd_exec_ros2.executables.bdd_coordination_node.to_scenario_status_msg",
        return_value=ScenarioStatus(),
    ):
        node._status_timer_callback()

    assert node._scenario_contexts == {}
    node._remove_context_topic_reg.assert_called_once_with(context_id=context_id)


def test_clock_reset_removes_stale_context_without_publishing_it():
    context_id = uuid4()
    context = SimpleNamespace(
        context_id=context_id,
        obs_manager=SimpleNamespace(
            scr_start_time=10.0,
            scr_end_time=20.0,
            obs_policies={},
            pending_end_deadline=None,
        ),
        scr_rep=object(),
        end_event_sent=True,
    )
    publisher = Mock()
    node = _node(
        _scenario_contexts={context_id: context},
        _scr_lock=threading.Lock(),
        _use_sim_time=True,
        _remove_context_topic_reg=Mock(),
        _scr_status_pub=publisher,
        _schedule_next_scenario=Mock(),
        get_clock=Mock(
            return_value=SimpleNamespace(now=Mock(return_value=Time(seconds=1.0)))
        ),
        get_logger=Mock(return_value=Mock()),
    )

    with patch(
        "bdd_exec_ros2.executables.bdd_coordination_node.to_scenario_status_msg"
    ) as to_status:
        node._status_timer_callback()

    assert node._scenario_contexts == {}
    to_status.assert_not_called()
    assert publisher.publish.call_args.args[0].scenarios == []


def test_behaviour_result_is_recorded_without_ending_scenario():
    context_id = uuid4()
    trinary = SimpleNamespace(value=next(iter(TRINARY_NAMES)))
    result = SimpleNamespace(
        result=SimpleNamespace(
            scenario_context_id=object(),
            trinary=trinary,
            stamp=Time().to_msg(),
        )
    )
    obs_manager = SimpleNamespace(
        scenario_exec=SimpleNamespace(end_event=URIRef("urn:test:end")),
        update_bhv_result=Mock(),
    )
    node = _node(
        _scenario_contexts={context_id: SimpleNamespace(obs_manager=obs_manager)},
        _scr_lock=threading.Lock(),
        _send_event=Mock(),
        _use_sim_time=False,
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

    obs_manager.update_bhv_result.assert_called_once()
    node._send_event.assert_not_called()


def test_scenario_ends_once_after_pending_observation_deadline():
    context_id = uuid4()
    end_event = URIRef("urn:test:end")
    context = SimpleNamespace(
        context_id=context_id,
        obs_manager=SimpleNamespace(
            scr_start_time=0.0,
            pending_end_deadline=3.0,
            scenario_exec=SimpleNamespace(end_event=end_event),
            scr_end_time=None,
            obs_policies={},
        ),
        scr_rep=object(),
        end_event_sent=False,
    )
    node = _node(
        _scenario_contexts={context_id: context},
        _scr_lock=threading.Lock(),
        _send_event=Mock(),
        _scr_status_pub=Mock(),
        _schedule_next_scenario=Mock(),
        get_clock=Mock(
            return_value=SimpleNamespace(now=Mock(return_value=Time(seconds=2.0)))
        ),
    )

    with patch(
        "bdd_exec_ros2.executables.bdd_coordination_node.to_scenario_status_msg",
        return_value=ScenarioStatus(),
    ):
        node._status_timer_callback()
        node.get_clock.return_value.now.return_value = Time(seconds=3.0)
        node._status_timer_callback()
        node._status_timer_callback()

    node._send_event.assert_called_once_with(evt_uri=end_event, ctx_id=context_id)


def test_rejected_behaviour_sets_scenario_false_before_ending():
    context_id = uuid4()
    end_event = URIRef("urn:test:end")
    obs_manager = SimpleNamespace(
        scenario_exec=SimpleNamespace(end_event=end_event),
        update_bhv_result=Mock(),
    )
    node = _node(
        _scenario_contexts={context_id: SimpleNamespace(obs_manager=obs_manager)},
        _scr_lock=threading.Lock(),
        _send_event=Mock(
            side_effect=lambda **_: obs_manager.update_bhv_result.assert_called_once()
        ),
        get_clock=Mock(return_value=SimpleNamespace(now=Mock(return_value=Time()))),
        get_logger=Mock(return_value=Mock()),
    )

    node.bhv_goal_resp_cb(
        SimpleNamespace(result=Mock(return_value=SimpleNamespace(accepted=False))),
        context_id,
    )

    result = obs_manager.update_bhv_result.call_args.kwargs["trin_st"]
    assert result.trinary is False
    node._send_event.assert_called_once_with(evt_uri=end_event, ctx_id=context_id)


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


def test_create_scenario_context_binds_and_resolves_simulation_targets():
    provider_uri = URIRef("urn:test:provider")
    observation_uri = URIRef("urn:test:observation")
    target_variable = URIRef("urn:test:target-variable")
    modelled_element = URIRef("urn:test:modelled-element")
    scene_instance = SimpleNamespace(
        resolve_modelled_element_id=Mock(return_value=modelled_element)
    )
    provider = SimpleNamespace(types={URI_ROS_TYPE_SIM_ENTITY_STATE_PROVIDER})
    obs_manager = SimpleNamespace(
        scenario_exec=SimpleNamespace(scene_instance=scene_instance),
        providers={provider_uri: provider},
        bind_observation_targets=Mock(),
        observation_targets_for_provider=Mock(
            return_value={observation_uri: target_variable}
        ),
    )
    variation = SimpleNamespace()
    bindings = {target_variable: URIRef("urn:test:bound-target")}
    node = _node(
        graph=Graph(),
        _ns_manager=Graph().namespace_manager,
        _clause_rep_builder=object(),
        _create_observation_subscription=Mock(),
    )

    with (
        patch(
            "bdd_exec_ros2.executables.bdd_coordination_node.ObservationManager.from_scenario_variant",
            return_value=obs_manager,
        ),
        patch(
            "bdd_exec_ros2.executables.bdd_coordination_node.ScenarioVariantRep",
            return_value=object(),
        ),
        patch(
            "bdd_exec_ros2.executables.bdd_coordination_node.get_update_rate",
            return_value=5.0,
        ),
    ):
        context = node._create_scenario_context(variation, bindings)

    obs_manager.bind_observation_targets.assert_called_once_with(bindings)
    scene_instance.resolve_modelled_element_id.assert_called_once_with(target_variable)
    assert context.scene_inst is scene_instance
    assert context.simulation_observations == {
        provider_uri: (5.0, {target_variable: modelled_element})
    }


def test_simulation_provider_forwards_pose_snapshot_atomically():
    context_id = uuid4()
    provider_uri = URIRef("urn:test:provider")
    routes = {
        URIRef("urn:test:first-observation"): URIRef("urn:test:first-target"),
        URIRef("urn:test:second-observation"): URIRef("urn:test:second-target"),
    }
    poses = {target: PoseStamped() for target in routes.values()}
    manager = SimpleNamespace(update_provider_observation=Mock(return_value={}))
    context = SimpleNamespace(
        scene_inst=object(),
        obs_manager=manager,
        simulation_observations={provider_uri: (object(), routes)},
    )
    receipt_time = Time(seconds=42.0)
    node = _node(
        _scr_lock=threading.Lock(),
        _scenario_contexts={context_id: context},
        get_clock=Mock(
            return_value=SimpleNamespace(now=Mock(return_value=receipt_time))
        ),
        get_logger=Mock(return_value=Mock()),
    )

    node._update_simulation_observation(context_id, provider_uri, poses)

    manager.update_provider_observation.assert_called_once_with(
        provider_uri,
        {
            observed_target: poses[model_target]
            for observed_target, model_target in routes.items()
        },
        ros_time_to_stamp(receipt_time),
    )


def test_context_cleanup_destroys_simulation_provider_timer():
    context_id = uuid4()
    key = (context_id, URIRef("urn:test:provider"))
    handle = Mock()
    node = _node(
        _topic_fpolicy_reg={},
        _topic_observation_reg={},
        _simulation_observation_handles={key: handle},
        _transform_observation_handles={},
    )

    node._remove_context_topic_reg(context_id)

    handle.cancel.assert_called_once_with()
    assert not node._simulation_observation_handles


def test_simulation_provider_reports_missing_target_states():
    context_id = uuid4()
    provider_uri = URIRef("urn:test:provider")
    target_uri = URIRef("urn:test:missing-target")
    logger = Mock()
    graph = Graph()
    graph.bind("test", "urn:test:")
    context = SimpleNamespace(
        scene_inst=object(),
        obs_manager=SimpleNamespace(update_provider_observation=Mock()),
        simulation_observations={
            provider_uri: (object(), {URIRef("urn:test:observation"): target_uri})
        },
    )
    node = _node(
        _scr_lock=threading.Lock(),
        _scenario_contexts={context_id: context},
        graph=graph,
        _ns_manager=graph.namespace_manager,
        get_clock=Mock(return_value=SimpleNamespace(now=Mock(return_value=Time()))),
        get_logger=Mock(return_value=logger),
    )

    node._update_simulation_observation(context_id, provider_uri, {})

    logger.warning.assert_called_once()
    assert target_uri.n3(graph.namespace_manager) in logger.warning.call_args.args[0]
    context.obs_manager.update_provider_observation.assert_not_called()


def test_simulation_provider_reports_policy_rejection():
    context_id = uuid4()
    provider_uri = URIRef("urn:test:provider")
    observation_uri = URIRef("urn:test:observation")
    target_uri = URIRef("urn:test:target")
    policy_uri = URIRef("urn:test:policy")
    logger = Mock()
    graph = Graph()
    graph.bind("test", "urn:test:")
    manager = SimpleNamespace(
        update_provider_observation=Mock(
            return_value={policy_uri: (False, "not active")}
        )
    )
    context = SimpleNamespace(
        scene_inst=object(),
        obs_manager=manager,
        simulation_observations={
            provider_uri: (object(), {observation_uri: target_uri})
        },
    )
    node = _node(
        _scr_lock=threading.Lock(),
        _scenario_contexts={context_id: context},
        graph=graph,
        _ns_manager=graph.namespace_manager,
        get_clock=Mock(return_value=SimpleNamespace(now=Mock(return_value=Time()))),
        get_logger=Mock(return_value=logger),
    )

    node._update_simulation_observation(
        context_id, provider_uri, {target_uri: PoseStamped()}
    )

    logger.warning.assert_called_once()
    assert policy_uri.n3(graph.namespace_manager) in logger.warning.call_args.args[0]
    assert "not active" in logger.warning.call_args.args[0]
