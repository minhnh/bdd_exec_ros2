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

import pytest
from bdd_dsl.prov import RecordingSession
from bdd_ros2_interfaces.msg import Event, ScenarioStatus
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time
from rdf_utils.models.prov import get_py_pkg_info
from rdf_utils.models.vocab import URI_ROS_TYPE_SIM_ENTITY_STATE_PROVIDER
from rdflib import RDF, Dataset, Graph, URIRef

from bdd_exec_ros2.conversions import TRINARY_NAMES, ros_time_to_stamp
from bdd_exec_ros2.executables.bdd_coordination_node import (
    BddCoordNode,
    SceneSetupMode,
)
from bdd_exec_ros2.prov import load_bdd_ros_prov


def _open_recorder(session):
    from bdd_exec_ros2.prov import RosScenarioExecRecorder

    context = SimpleNamespace(
        context_id=uuid4(),
        scene_inst=SimpleNamespace(id=URIRef("urn:scene")),
        variation_params={},
        obs_manager=SimpleNamespace(obs_policies={}),
    )
    return RosScenarioExecRecorder.open(
        session,
        context,
        SimpleNamespace(id=URIRef("urn:variant")),
    )


def _node(**attrs):
    node = BddCoordNode.__new__(BddCoordNode)
    output_dir = attrs.pop("_prov_output_dir", None)
    session = (
        RecordingSession(
            output_dir,
            get_py_pkg_info("bdd_exec_ros2").purl,
            use_sim_time=attrs.get("_use_sim_time", False),
        )
        if output_dir is not None
        else None
    )
    if session is not None:
        load_bdd_ros_prov(session.prov_bundle)
    node.__dict__.update(
        {
            "_user_story": None,
            "_use_sim_time": False,
            "_recorders": {},
            "_scr_lock": threading.Lock(),
            "graph": Graph(identifier=URIRef(uuid4().urn)),
            "_recording_session": session,
            "_scene_setup_mode": SceneSetupMode.NONE,
            "list_parameters": Mock(return_value=SimpleNamespace(names=[])),
            **attrs,
        }
    )
    if session is not None and isinstance(node.graph, Graph):
        session.model_graph.__iadd__(node.graph)
        node.graph = session.model_graph
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


@pytest.mark.parametrize("error", [ValueError, OSError])
def test_event_recording_errors_propagate(error):
    context_id = uuid4()
    recorder = Mock()
    recorder.create_record.side_effect = error("recording failed")
    node = _node(
        _scenario_contexts={
            context_id: SimpleNamespace(context_id=context_id, obs_manager=Mock())
        },
        _recorders={context_id: recorder},
        _ns_manager=Graph().namespace_manager,
        get_logger=Mock(return_value=Mock()),
    )
    message = Event()
    message.uri = "urn:test:event"
    message.stamp = Time(seconds=4).to_msg()
    with pytest.raises(error, match="recording failed"):
        node.evt_sub_cb(message)


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


def test_clock_reset_without_scene_setup_fails_and_cancels_active_scenario():
    context_id = uuid4()
    manager, goal, publisher = Mock(), Mock(), Mock()
    context = SimpleNamespace(
        context_id=context_id, obs_manager=manager, goal_handle=goal, scr_rep=object()
    )
    node = _node(
        _scene_setup_mode=SceneSetupMode.NONE,
        _scenario_contexts={context_id: context},
        _scr_lock=threading.Lock(),
        _pending_scenarios=deque([object()]),
        _remove_context_topic_reg=Mock(),
        _scr_status_pub=publisher,
        get_logger=Mock(return_value=Mock()),
    )
    with (
        patch(
            "bdd_exec_ros2.executables.bdd_coordination_node.to_scenario_status_msg",
            return_value=ScenarioStatus(),
        ),
        pytest.raises(RuntimeError, match="clock reset after scenario start"),
    ):
        node._handle_clock_reset((context_id,), 10_000_000_000)
    assert node._scenario_contexts == {}
    assert not node._pending_scenarios
    result = manager.update_bhv_result.call_args.args[0]
    assert result.trinary is False and "clock reset" in result.reason
    goal.cancel_goal_async.assert_called_once()
    assert len(publisher.publish.call_args.args[0].scenarios) == 1
    node._remove_context_topic_reg.assert_called_once_with(context_id=context_id)


def test_clock_reset_only_affects_contexts_active_at_reset():
    old_id, new_id = uuid4(), uuid4()
    node = _node(
        _scenario_contexts={new_id: object()},
        _scr_lock=threading.Lock(),
        _scr_status_pub=Mock(),
    )
    node._handle_clock_reset((old_id,), 10_000_000_000)
    assert list(node._scenario_contexts) == [new_id]
    node._scr_status_pub.publish.assert_not_called()


def test_goal_accepted_after_context_retirement_is_cancelled():
    goal = Mock(accepted=True)
    node = _node(
        _scenario_contexts={},
        _scr_lock=threading.Lock(),
        get_logger=Mock(return_value=Mock()),
    )
    node.bhv_goal_resp_cb(Mock(result=Mock(return_value=goal)), uuid4())
    goal.cancel_goal_async.assert_called_once()


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


def test_configured_story_does_not_start_imported_reference_scenarios():
    story = URIRef("urn:test:selected-story")
    selected = URIRef("urn:test:selected-variant")
    loader = Mock()
    loader.get_us_scenario_variants.return_value = {
        URIRef("urn:test:reference-story"): [URIRef("urn:test:reference-variant")],
        story: [selected],
    }
    loader.load_scenario_variant.return_value = SimpleNamespace(task_variation=object())
    node = _node(
        _user_story=story,
        _scene_setup_mode=SceneSetupMode.NONE,
        us_loader=loader,
        graph=object(),
        _create_scenario_context=Mock(),
        _start_scenario_variant=Mock(),
        _schedule_next_scenario=Mock(),
    )
    with patch(
        "bdd_exec_ros2.executables.bdd_coordination_node.get_task_var_dicts",
        return_value=[{}],
    ):
        node.start_test_cb(None)
    loader.load_scenario_variant.assert_called_once_with(
        full_graph=node.graph, variant_id=selected
    )
    node._start_scenario_variant.assert_called_once()


def test_clock_reset_ignores_context_without_start_time():
    context_id = uuid4()
    context = SimpleNamespace(obs_manager=SimpleNamespace(scr_start_time=None))
    executor = Mock()
    node = _node(_scenario_contexts={context_id: context}, executor=executor)
    node._on_clock_reset(10_000_000_000)
    executor.create_task.assert_not_called()
    assert node._scenario_contexts[context_id] is context


@pytest.mark.parametrize("reset_after_setup", [False, True])
def test_coordinator_waits_for_clock_progress_after_scene_setup(
    monkeypatch, reset_after_setup
):
    from rclpy.clock import ROSClock

    from bdd_exec_ros2.clock import SimulationClockSync

    monkeypatch.setattr("bdd_exec_ros2.clock.Future", asyncio.Future)

    async def run():
        clock = ROSClock()
        clock._set_ros_time_is_active(True)
        clock.set_ros_time_override(Time(seconds=100))
        clock_node = SimpleNamespace(
            get_clock=lambda: clock,
            create_timer=lambda *args, **kwargs: object(),
            destroy_timer=Mock(),
        )
        sync = SimulationClockSync(clock_node)
        context = SimpleNamespace(context_id=uuid4(), scene_inst=object())
        variant = SimpleNamespace(id=URIRef("urn:test:variant"), scene=object())
        node = _node(
            _use_sim_time=True,
            _sim_clock=sync,
            timeout_sec=1,
            _sim_interface=SimpleNamespace(setup_scene=AsyncMock(return_value={})),
            _pending_scenarios=deque([(variant, {})]),
            _scenario_contexts={},
            _scr_lock=threading.Lock(),
            _scene_setup_active=True,
            _create_scenario_context=Mock(return_value=context),
            _start_scenario_variant=Mock(),
            _schedule_next_scenario=Mock(),
        )
        try:
            setup = asyncio.create_task(node._prepare_next_scenario())
            await asyncio.sleep(0)
            node._sim_interface.setup_scene.assert_awaited_once()
            node._start_scenario_variant.assert_not_called()
            if reset_after_setup:
                clock.set_ros_time_override(Time())
                await asyncio.sleep(0)
                node._start_scenario_variant.assert_not_called()
            clock.set_ros_time_override(Time(seconds=0.1 if reset_after_setup else 101))
            await setup
            node._start_scenario_variant.assert_called_once_with(context, variant)
        finally:
            sync.close()

    asyncio.run(run())


@pytest.mark.parametrize("enabled", [False, True])
def test_recording_contexts_have_independent_snapshots_and_lifecycles(
    tmp_path, enabled
):
    from bdd_dsl.prov import read_journal

    model = Graph()
    model.add((URIRef("urn:model"), RDF.type, URIRef("urn:Model")))
    node = _node(
        _prov_output_dir=tmp_path if enabled else None,
        _scene_setup_mode=SceneSetupMode.NONE,
        graph=model,
        get_logger=Mock(return_value=Mock()),
    )
    contexts = [
        SimpleNamespace(
            context_id=uuid4(),
            scene_inst=SimpleNamespace(id=URIRef("urn:scene")),
            variation_params={},
            obs_manager=SimpleNamespace(obs_policies={}),
        )
        for _ in range(2)
    ]
    for context in contexts:
        with node._scr_lock:
            node._open_recording(context, SimpleNamespace(id=URIRef("urn:variant")))
            node._close_recording(context.context_id, "finished", 1.0)
    assert not node._recorders
    if not enabled:
        assert not list(tmp_path.iterdir())
        return
    assert len(list(tmp_path.iterdir())) == len(contexts) + 1
    from rdflib.namespace import PROV

    session = node._recording_session
    assert session is not None
    dataset = Dataset().parse(session.path, format=session.dataset_format)
    bundles = list(dataset.default_graph.subjects(PROV.wasGeneratedBy, None))
    assert len(set(bundles)) == len(contexts)
    assert set(dataset.graph(session.model_id)) == set(model)
    assert (
        len(
            [
                graph
                for graph in dataset.graphs()
                if (URIRef("urn:model"), RDF.type, URIRef("urn:Model")) in graph
            ]
        )
        == 1
    )
    for context in contexts:
        execution_id = URIRef(context.context_id.urn)
        bundle_id = dataset.default_graph.value(None, PROV.wasGeneratedBy, execution_id)
        assert bundle_id is not None and bundle_id != execution_id
        graph = dataset.graph(bundle_id)
        assert (bundle_id, RDF.type, PROV.Bundle) in graph
        assert (execution_id, RDF.type, PROV.Activity) in graph
        assert (bundle_id, PROV.wasAttributedTo, session.agent_id) in graph
        assert (bundle_id, RDF.type, PROV.Activity) not in graph
        assert list(graph.objects(execution_id, PROV.used))
    for context in contexts:
        directory = tmp_path / str(context.context_id)
        assert session.path.is_file()
        records = list(read_journal(directory / "evidence.jsonl"))
        assert {r["execution_id"] for r in records} == {URIRef(context.context_id.urn)}
        assert records[-1]["data"]["termination"] == "finished"
        before = (directory / "evidence.jsonl").read_bytes()
        with node._scr_lock, pytest.raises(FileExistsError):
            node._open_recording(context, SimpleNamespace(id=URIRef("urn:variant")))
        assert (directory / "evidence.jsonl").read_bytes() == before


def test_shutdown_records_incomplete_termination_under_the_context_lock(tmp_path):
    from bdd_dsl.prov import read_journal

    from bdd_exec_ros2.prov import RosScenarioExecRecorder

    context_id = uuid4()
    path = tmp_path / "evidence.jsonl"
    session = RecordingSession(
        tmp_path, get_py_pkg_info("bdd_exec_ros2").purl, use_sim_time=False
    )
    recorder = RosScenarioExecRecorder(path, context_id, session)
    node = _node(
        _sim_clock=None,
        _recorders={context_id: recorder},
        get_clock=Mock(return_value=SimpleNamespace(now=Mock(return_value=Time()))),
    )
    close = recorder.close

    def close_with_lock_check():
        assert node._scr_lock.locked()
        close()

    recorder.close = close_with_lock_check
    with patch("rclpy.node.Node.destroy_node", return_value=True):
        assert node.destroy_node()
    assert not node._recorders
    assert list(read_journal(path))[-1]["data"]["termination"] == "shutdown"


def test_status_reads_do_not_record_evaluations():
    context_id = uuid4()
    recorder = Mock()
    context = SimpleNamespace(
        obs_manager=SimpleNamespace(
            pending_end_deadline=None, scr_end_time=None, obs_policies={}
        ),
        scr_rep=object(),
        end_event_sent=False,
    )
    node = _node(
        _scenario_contexts={context_id: context},
        _recorders={context_id: recorder},
        _scr_status_pub=Mock(),
        _schedule_next_scenario=Mock(),
        get_clock=Mock(return_value=SimpleNamespace(now=Mock(return_value=Time()))),
    )
    with patch(
        "bdd_exec_ros2.executables.bdd_coordination_node.to_scenario_status_msg",
        return_value=ScenarioStatus(),
    ):
        node._status_timer_callback()
        node._status_timer_callback()
    assert node._scr_status_pub.publish.call_count == 2
    assert not recorder.mock_calls


def test_metadata_save_failure_never_attaches_callbacks_or_registers_recorder(
    tmp_path,
):
    policy = SimpleNamespace(evaluator=None, evaluator_id=None, prov_recorder=None)
    context = SimpleNamespace(
        context_id=uuid4(),
        scene_inst=SimpleNamespace(id=URIRef("urn:scene")),
        variation_params={},
        obs_manager=SimpleNamespace(obs_policies={URIRef("urn:policy"): policy}),
    )
    node = _node(
        _prov_output_dir=tmp_path,
        graph=Graph(),
        get_logger=Mock(return_value=Mock()),
    )
    with (
        patch(
            "bdd_dsl.prov.RecordingSession.save",
            side_effect=OSError("disk full"),
        ),
        node._scr_lock,
        pytest.raises(OSError, match="disk full"),
    ):
        node._open_recording(context, SimpleNamespace(id=URIRef("urn:variant")))
    assert not node._recorders
    assert policy.prov_recorder is None


def test_successful_recording_initialization_attaches_policy_callback(tmp_path):
    policy = SimpleNamespace(evaluator=None, evaluator_id=None, prov_recorder=None)
    context = SimpleNamespace(
        context_id=uuid4(),
        scene_inst=SimpleNamespace(id=URIRef("urn:scene")),
        variation_params={},
        obs_manager=SimpleNamespace(obs_policies={URIRef("urn:policy"): policy}),
    )
    node = _node(
        _prov_output_dir=tmp_path,
        graph=Graph(),
        _scene_setup_mode=SceneSetupMode.NONE,
        get_logger=Mock(return_value=Mock()),
    )
    with node._scr_lock:
        node._open_recording(context, SimpleNamespace(id=URIRef("urn:variant")))
        assert policy.prov_recorder is node._recorders[context.context_id]
        node._close_recording(context.context_id, "finished", 1.0)


def test_model_loading_records_sources_transformation_and_execution_use(tmp_path):
    import json

    from rdf_utils.models.vocab import URI_PROV_EXT_TYPE_TRANSFORMATION
    from rdflib.namespace import DCTERMS, PROV, TIME

    from bdd_exec_ros2.executables.bdd_coordination_node import (
        load_graph_models_in_yaml,
    )

    source = tmp_path / "model.jsonld"
    source.write_text(
        json.dumps({"@id": "urn:subject", "urn:predicate": {"@id": "urn:object"}})
    )
    manifest = tmp_path / "models.yaml"
    manifest.write_text(json.dumps([{"path": str(source), "format": "json-ld"}]))
    session = RecordingSession(
        tmp_path, get_py_pkg_info("bdd_exec_ros2").purl, use_sim_time=False
    )
    provenance, model_g = session.prov_bundle, session.model_graph
    load_graph_models_in_yaml(str(manifest), model_g, session.prov_bundle)
    model = model_g
    (activity,) = provenance.objects(model.identifier, PROV.wasGeneratedBy)
    assert (activity, RDF.type, URI_PROV_EXT_TYPE_TRANSFORMATION) in provenance
    (collection,) = (
        entity
        for entity in provenance.objects(activity, PROV.used)
        if (entity, RDF.type, PROV.Collection) in provenance
    )
    (loading,) = provenance.objects(model.identifier, PROV.wasInfluencedBy)

    assert (
        loading,
        PROV.wasAssociatedWith,
        get_py_pkg_info("rdflib").purl,
    ) in provenance
    (contribution,) = provenance.subjects(PROV.wasGeneratedBy, loading)
    assert contribution != model.identifier
    assert (activity, PROV.used, contribution) in provenance
    assert (model.identifier, PROV.wasDerivedFrom, contribution) in provenance
    files = set(provenance.objects(collection, PROV.hadMember))
    from rdf_utils.models.prov import get_file_path

    assert {get_file_path(provenance, file) for file in files} == {source, manifest}
    assert all(provenance.value(file, DCTERMS.identifier) is not None for file in files)
    for boundary in (TIME.hasBeginning, TIME.hasEnd):
        instant = provenance.value(activity, boundary)
        assert (instant, RDF.type, TIME.Instant) in provenance
    assert (model.identifier, PROV.wasGeneratedBy, activity) in provenance
    recorder = _open_recorder(session)
    assert (recorder.execution_id, PROV.used, model.identifier) in recorder.graph
    restored = Dataset().parse(session.path, format=session.dataset_format)
    assert set(restored.graph(model.identifier)) == set(model)
    # Bundle declarations live inside their named graphs.
    bundles = {
        graph.identifier
        for graph in restored.graphs()
        if (graph.identifier, RDF.type, PROV.Bundle) in graph
    }
    assert recorder.bundle_id in bundles
    assert any(
        (model.identifier, PROV.wasGeneratedBy, activity) in restored.graph(bundle)
        for bundle in bundles
    )
    recorder.close()


def test_model_loading_keeps_shared_graph_available_to_robbdd_converters(tmp_path):
    import json

    from bdd_exec_ros2.executables.bdd_coordination_node import (
        load_graph_models_in_yaml,
    )

    paths = [tmp_path / name for name in ("first.bdd", "second.bdd")]
    for path in paths:
        path.write_text("example")
    manifest = tmp_path / "models.yaml"
    manifest.write_text(
        json.dumps([{"path": str(path), "format": "robbdd"} for path in paths])
    )
    converted = []

    def convert(model, g):
        for previous in converted:
            assert (URIRef(previous._tx_filename), RDF.type, URIRef("urn:model")) in g
        converted.append(model)
        g.add((URIRef(model._tx_filename), RDF.type, URIRef("urn:model")))

    metamodel = SimpleNamespace(
        model_from_file=lambda path: SimpleNamespace(_tx_filename=path, imports=[])
    )
    graph = Graph(identifier=URIRef(uuid4().urn))
    with (
        patch("textx.metamodel_for_file", return_value=metamodel),
        patch(
            "textx.registration.language_for_file",
            return_value=SimpleNamespace(name="robbdd"),
        ),
        patch("robbdd.rdf.bdd.create_bdd_model_graph", side_effect=convert),
    ):
        load_graph_models_in_yaml(str(manifest), graph)
    assert set(graph.subjects(RDF.type, URIRef("urn:model"))) == {
        URIRef(str(path)) for path in paths
    }
