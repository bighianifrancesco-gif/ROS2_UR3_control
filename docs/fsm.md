# Peg-in-Hole FSM

This diagram documents the exact finite-state machine implemented in
`peg_insertion_fsm.py`.

The FSM is contact-driven and uses position control only.

```mermaid
stateDiagram-v2
    [*] --> IDLE

    IDLE --> MOVE_ABOVE : pre_insert_q defined
    IDLE --> GUARDED_INSERT : no pre_insert_q

    MOVE_ABOVE --> CLEAR_WAIT : reached pre_insert pose

    CLEAR_WAIT --> GUARDED_INSERT : just_finished_xy && no contact for clear_s
    CLEAR_WAIT --> SEARCH_XY : next spiral offset available
    CLEAR_WAIT --> FAILURE : spiral exhausted

    GUARDED_INSERT --> RETRACT : persistent contact (debounce_s)
    GUARDED_INSERT --> GO_UP : z <= z_target && no contact for settle_s
    GUARDED_INSERT --> GUARDED_INSERT : dz_step (no contact)

    RETRACT --> ORIENT_UPRIGHT : first retract && upright enabled
    RETRACT --> CLEAR_WAIT : retract complete

    ORIENT_UPRIGHT --> POST_UPRIGHT : wrist trajectory sent
    POST_UPRIGHT --> SEARCH_XY : next spiral offset

    SEARCH_XY --> RETRACT : contact during XY motion
    SEARCH_XY --> CLEAR_WAIT : target XY reached
    SEARCH_XY --> FAILURE : spiral exhausted

    GO_UP --> SUCCESS : go_up_distance reached

    SUCCESS --> [*]
    FAILURE --> [*]
