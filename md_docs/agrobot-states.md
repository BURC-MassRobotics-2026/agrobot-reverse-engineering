# Whole-robot operation — Level 1 state machine

- source_pdf: `pdf_docs/agrobot-states.pdf`
- workspace: `nucbox_archive/nucbox_archive/agrobot_ws` · `nucbox_archive/nucbox_archive/ingenia_overlay_ws` · `nucbox_archive/nucbox_archive/AgrobotV2`
- configuration: `agrobot_supervisor/launch/supervisor.launch.py` with its default arguments: start_position 0.05 m, settle_seconds 1.0, capture_timeout 3.0, min_track_age 2, min_confidence 0.30. `robot_stack/hw_moveit_stack.sh` does not include this node.
- verification: Source-derived from the NucBox archive. No runtime verification.
- state_owner: /agrobot_supervisor, `self.state` (enum State), `nucbox_archive/nucbox_archive/agrobot_ws/src/agrobot_supervisor/agrobot_supervisor/supervisor_node.py`
- view: Explicit. Each state and transition is in the code. A 10 Hz timer runs the handler for the current state.
- comparison: The repository copy `Agrobot/src` has no supervisor. The earlier Level 1 view was conceptual.

## Semantics

- Blue boxes are explicit values of the State enum. Gray boxes are context. Amber marks a missing capability.
- A solid arrow is a transition in the code. Labels use event [guard] / effect.
- The supervisor owns only the search run. Joint bridges, /commander, and perception keep their own state.
- Drive faults do not reach this machine. Only the rail service result does.
- DONE is terminal. The timer keeps running and the handler does nothing.

## Nodes

- `initial` | kind: initial | initial marker | The constructor sets IDLE.
- `IDLE` | kind: state | IDLE
  - Wait for /rail_mover/goto
- `STEP` | kind: state | STEP | step = 0.8 × frame width at the median tomato depth, 0.05–0.40 m. Fallback 0.15 m.
  - Call /rail_mover/goto
  - target = j0 + step
- `SETTLE` | kind: state | SETTLE
  - Dwell 1.0 s
- `CAPTURE` | kind: state | CAPTURE
  - Wait for a tracks message newer than the capture start
- `PROCESS_QUEUE` | kind: state | PROCESS_QUEUE
  - Parse, keep age ≥ 2 and confidence ≥ 0.30
  - Key the queue by persistent_id
- `END_REACHED` | kind: state | END_REACHED | celebrate() calls /anthro_mover/goto and blocks until each pose returns.
  - Log the queue
  - celebrate(): 7 anthro poses
- `RETURN` | kind: state | RETURN
  - Rail goto start_position
  - 0.05 m
- `DONE` | kind: state | DONE
  - Log the run total
  - No further action
- `ctx_rail` | kind: context | /rail_mover/goto
  - SERVICE agrobot_motion/srv/RailGoto
  - Clamps target to 0.05–1.30 m
  - clamped = true: end of rail
- `ctx_tracks` | kind: context | /agrobot/tomato_tracks
  - TOPIC from /agrobot/tomato_tracker
- `ctx_stub` | kind: missing capability | Not in this machine
  - No pick: PROCESS_QUEUE only queues tomatoes
  - No link to /commander hw_moveit_stack.sh does not include this node

## Edges

- `initial -> IDLE` | transition | node start
- `IDLE -> STEP` | transition | tick [rail service ready]
- `STEP -> SETTLE` | transition | rail result [success, not clamped] / start settle timer
- `SETTLE -> CAPTURE` | transition | tick [settle time elapsed] / start capture window
- `CAPTURE -> PROCESS_QUEUE` | transition | tick [fresh tracks message] / take snapshot
- `CAPTURE -> PROCESS_QUEUE` | transition | tick [3.0 s timeout] / empty snapshot
- `PROCESS_QUEUE -> STEP` | transition | tick / add new IDs, station + 1 (pick step is a stub)
- `STEP -> DONE` | transition | rail result [call failed or success = false]
- `STEP -> END_REACHED` | transition | rail result [clamped = true]
- `END_REACHED -> RETURN` | transition | tick / log queue, celebrate()
- `RETURN -> DONE` | transition | rail result (any)
