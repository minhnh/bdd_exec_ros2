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

"""
This is an auto-generated file. Do not edit it directly.

FSM: pickplace
FSM Description: Example of a simple FSM for pick-place application

Examples:

>>> from coord_dsl.fsm import fsm_step
>>> from coord_dsl.event_loop import reconfig_event_buffers
>>> from fsm_example import create_fsm
>>> fsm = create_fsm()
>>> while True:
...     if fsm.current_state_index == StateID.S_EXIT:
...         print("State machine completed successfully")
...         break
...     fsm_behavior(fsm, ud) # user-defined behaviour with user data
...     fsm_step(fsm)
...     reconfig_event_buffers(fsm.event_data)
"""

from enum import IntEnum, auto
from coord_dsl.event_loop import EventData
from coord_dsl.fsm import FSMData, Transition, EventReaction


# Event IDs
class EventID(IntEnum):
    EVT_PERCEIVE_DONE = 0
    EVT_PICK_START = auto()
    EVT_PICK_APPROACH_DONE = auto()
    EVT_PLACE_START = auto()
    EVT_PLACE_APPROACH_DONE = auto()
    EVT_PICK_END = auto()
    EVT_PLACE_END = auto()
    EVT_STEP = auto()


# State IDs
class StateID(IntEnum):
    S_START = 0
    S_PERCEIVE = auto()
    S_APPROACH = auto()
    S_PICK = auto()
    S_PLACE = auto()
    S_EXIT = auto()


# Transition IDs
class TransitionID(IntEnum):
    T_START_PERCEIVE = 0
    T_PERCEIVE_PERCEIVE = auto()
    T_PERCEIVE_APPROACH = auto()
    T_APPROACH_APPROACH = auto()
    T_APPROACH_PICK = auto()
    T_PICK_PICK = auto()
    T_PICK_APPROACH = auto()
    T_APPROACH_PLACE = auto()
    T_PLACE_PLACE = auto()
    T_PERCEIVE_EXIT = auto()
    T_APPROACH_EXIT = auto()
    T_PICK_EXIT = auto()
    T_PLACE_EXIT = auto()


# Event reaction IDs
class ReactionID(IntEnum):
    R_E_PERCEIVE_DONE = 0
    R_E_PICK_APPROACH_DONE = auto()
    R_E_PLACE_APPROACH_DONE = auto()
    R_E_PICK_DONE = auto()
    R_E_PLACE_DONE = auto()
    R_E_STEP_START = auto()
    R_E_STEP_PERCEIVE = auto()
    R_E_STEP_APPROACH = auto()
    R_E_STEP_PICK = auto()
    R_E_STEP_PLACE = auto()


# URI mappings
STATE_URIS: dict[StateID, str] = {
    StateID.S_START: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/S_START",
    StateID.S_PERCEIVE: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/S_PERCEIVE",
    StateID.S_APPROACH: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/S_APPROACH",
    StateID.S_PICK: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/S_PICK",
    StateID.S_PLACE: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/S_PLACE",
    StateID.S_EXIT: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/S_EXIT",
}

EVENT_URIS: dict[EventID, str] = {
    EventID.EVT_PERCEIVE_DONE: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/evt-perceive-done",
    EventID.EVT_PICK_START: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/evt-pick-start",
    EventID.EVT_PICK_APPROACH_DONE: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/evt-pick-approach-done",
    EventID.EVT_PLACE_START: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/evt-place-start",
    EventID.EVT_PLACE_APPROACH_DONE: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/evt-place-approach-done",
    EventID.EVT_PICK_END: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/evt-pick-end",
    EventID.EVT_PLACE_END: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/evt-place-end",
    EventID.EVT_STEP: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/evt-step",
}

TRANSITION_URIS: dict[TransitionID, str] = {
    TransitionID.T_START_PERCEIVE: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_START_PERCEIVE",
    TransitionID.T_PERCEIVE_PERCEIVE: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_PERCEIVE_PERCEIVE",
    TransitionID.T_PERCEIVE_APPROACH: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_PERCEIVE_APPROACH",
    TransitionID.T_APPROACH_APPROACH: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_APPROACH_APPROACH",
    TransitionID.T_APPROACH_PICK: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_APPROACH_PICK",
    TransitionID.T_PICK_PICK: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_PICK_PICK",
    TransitionID.T_PICK_APPROACH: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_PICK_APPROACH",
    TransitionID.T_APPROACH_PLACE: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_APPROACH_PLACE",
    TransitionID.T_PLACE_PLACE: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_PLACE_PLACE",
    TransitionID.T_PERCEIVE_EXIT: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_PERCEIVE_EXIT",
    TransitionID.T_APPROACH_EXIT: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_APPROACH_EXIT",
    TransitionID.T_PICK_EXIT: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_PICK_EXIT",
    TransitionID.T_PLACE_EXIT: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_PLACE_EXIT",
}

REACTION_URIS: dict[ReactionID, str] = {
    ReactionID.R_E_PERCEIVE_DONE: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_PERCEIVE_DONE",
    ReactionID.R_E_PICK_APPROACH_DONE: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_PICK_APPROACH_DONE",
    ReactionID.R_E_PLACE_APPROACH_DONE: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_PLACE_APPROACH_DONE",
    ReactionID.R_E_PICK_DONE: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_PICK_DONE",
    ReactionID.R_E_PLACE_DONE: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_PLACE_DONE",
    ReactionID.R_E_STEP_START: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_STEP_START",
    ReactionID.R_E_STEP_PERCEIVE: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_STEP_PERCEIVE",
    ReactionID.R_E_STEP_APPROACH: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_STEP_APPROACH",
    ReactionID.R_E_STEP_PICK: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_STEP_PICK",
    ReactionID.R_E_STEP_PLACE: "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_STEP_PLACE",
}


def create_fsm() -> FSMData:
    """Creates the FSM data structure."""
    # Transitions
    trans_dict = {
        TransitionID.T_START_PERCEIVE: Transition(StateID.S_START, StateID.S_PERCEIVE),
        TransitionID.T_PERCEIVE_PERCEIVE: Transition(
            StateID.S_PERCEIVE, StateID.S_PERCEIVE
        ),
        TransitionID.T_PERCEIVE_APPROACH: Transition(
            StateID.S_PERCEIVE, StateID.S_APPROACH
        ),
        TransitionID.T_APPROACH_APPROACH: Transition(
            StateID.S_APPROACH, StateID.S_APPROACH
        ),
        TransitionID.T_APPROACH_PICK: Transition(StateID.S_APPROACH, StateID.S_PICK),
        TransitionID.T_PICK_PICK: Transition(StateID.S_PICK, StateID.S_PICK),
        TransitionID.T_PICK_APPROACH: Transition(StateID.S_PICK, StateID.S_APPROACH),
        TransitionID.T_APPROACH_PLACE: Transition(StateID.S_APPROACH, StateID.S_PLACE),
        TransitionID.T_PLACE_PLACE: Transition(StateID.S_PLACE, StateID.S_PLACE),
        TransitionID.T_PERCEIVE_EXIT: Transition(StateID.S_PERCEIVE, StateID.S_EXIT),
        TransitionID.T_APPROACH_EXIT: Transition(StateID.S_APPROACH, StateID.S_EXIT),
        TransitionID.T_PICK_EXIT: Transition(StateID.S_PICK, StateID.S_EXIT),
        TransitionID.T_PLACE_EXIT: Transition(StateID.S_PLACE, StateID.S_EXIT),
    }
    trans_list = [trans_dict[i] for i in TransitionID]

    # Event Reactions
    evt_reaction_dict = {
        ReactionID.R_E_PERCEIVE_DONE: EventReaction(
            condition_event_index=EventID.EVT_PERCEIVE_DONE,
            transition_index=TransitionID.T_PERCEIVE_APPROACH,
            fired_event_indices=[
                EventID.EVT_PICK_START,
            ],
        ),
        ReactionID.R_E_PICK_APPROACH_DONE: EventReaction(
            condition_event_index=EventID.EVT_PICK_APPROACH_DONE,
            transition_index=TransitionID.T_APPROACH_PICK,
            fired_event_indices=[],
        ),
        ReactionID.R_E_PLACE_APPROACH_DONE: EventReaction(
            condition_event_index=EventID.EVT_PLACE_APPROACH_DONE,
            transition_index=TransitionID.T_APPROACH_PLACE,
            fired_event_indices=[],
        ),
        ReactionID.R_E_PICK_DONE: EventReaction(
            condition_event_index=EventID.EVT_PICK_END,
            transition_index=TransitionID.T_PICK_APPROACH,
            fired_event_indices=[
                EventID.EVT_PLACE_START,
            ],
        ),
        ReactionID.R_E_PLACE_DONE: EventReaction(
            condition_event_index=EventID.EVT_PLACE_END,
            transition_index=TransitionID.T_PLACE_EXIT,
            fired_event_indices=[],
        ),
        ReactionID.R_E_STEP_START: EventReaction(
            condition_event_index=EventID.EVT_STEP,
            transition_index=TransitionID.T_START_PERCEIVE,
            fired_event_indices=[],
        ),
        ReactionID.R_E_STEP_PERCEIVE: EventReaction(
            condition_event_index=EventID.EVT_STEP,
            transition_index=TransitionID.T_PERCEIVE_PERCEIVE,
            fired_event_indices=[],
        ),
        ReactionID.R_E_STEP_APPROACH: EventReaction(
            condition_event_index=EventID.EVT_STEP,
            transition_index=TransitionID.T_APPROACH_APPROACH,
            fired_event_indices=[],
        ),
        ReactionID.R_E_STEP_PICK: EventReaction(
            condition_event_index=EventID.EVT_STEP,
            transition_index=TransitionID.T_PICK_PICK,
            fired_event_indices=[],
        ),
        ReactionID.R_E_STEP_PLACE: EventReaction(
            condition_event_index=EventID.EVT_STEP,
            transition_index=TransitionID.T_PLACE_PLACE,
            fired_event_indices=[],
        ),
    }
    evt_reaction_list = [evt_reaction_dict[i] for i in ReactionID]

    # Events
    events = EventData(len(EventID))

    # Return FSM instance
    return FSMData(
        event_data=events,
        num_states=len(StateID),
        start_state_index=StateID.S_START,
        end_state_index=StateID.S_EXIT,
        transitions=trans_list,
        event_reactions=evt_reaction_list,
        current_state_index=StateID.S_START,
    )
