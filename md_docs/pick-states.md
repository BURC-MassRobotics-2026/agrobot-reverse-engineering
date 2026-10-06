# Pick task and target preparation — Level 2 state machines

- source_pdf: `pdf_docs/pick-states.pdf`
- workspace: `nucbox_archive/nucbox_archive/agrobot_ws` · `nucbox_archive/nucbox_archive/ingenia_overlay_ws` · `nucbox_archive/nucbox_archive/AgrobotV2`
- configuration: `robot_commander/launch/commander.launch.py`: /commander and /tomato_picker. Perception from `AgrobotV2/perception/launch/perception.launch.py`.
- verification: Source-derived from the NucBox archive. No runtime verification.
- comparison: The repository copy runs picks as a /agrobot/pick_sequence action goal with feedback and a Canceled result. The NucBox copy runs picks in a topic callback with no goal status.

## Page 1: Pick task in /commander

- state_owner: /commander, `is_picking_` and the loop index in pickTargetCallback, `nucbox_archive/nucbox_archive/agrobot_ws/src/robot_commander/src/commander.cpp`
- view: Explicit for Waiting and Picking. Inferred for the five steps.

### Semantics

- Waiting and Picking are values of is_picking_. The code has no phase variable.
- The five steps inside Picking follow the code order (inferred).
- A solid arrow is a transition in the code. Labels use event [guard] / effect.
- A step returns when planAndExecute() returns. The code does not check whether the motion succeeded.
- No initial or final marker applies to a single pick. The node starts in Waiting.

### Nodes

- `initial` | kind: initial | initial marker | Node start. is_picking_ is false.
- `waiting` | kind: state | Waiting
  - is_picking_ = false
- `picking` | kind: composite state | Picking
  - is_picking_ = true · steps inferred from code order
- `crouch` | kind: inferred phase | Crouch
  - named target crouch
- `approach` | kind: inferred phase | Approach
  - poses[i]
- `grasp` | kind: inferred phase | Grasp
  - poses[i+1]
- `retract` | kind: inferred phase | Retract
  - poses[i+2]
- `bin` | kind: inferred phase | Bin
  - named target bin
- `ctx_fail` | kind: context | Plan failure
  - Log only. The step counts as done.
- `ctx_safe` | kind: context | safe_to_pick
  - /tomato_detector also publishes it on each frame.
- `ctx_queue` | kind: context | During Picking
  - New pick_targets wait in the queue
  - (depth 10).
- `ctx_none` | kind: missing capability | Not in this task
  - No cancel, abort, gripper command, or result.

### Edges

- `initial -> waiting` | transition | node start
- `waiting -> waiting` | transition | pick_targets [count % 3 ≠ 0] / log, ignore
- `waiting -> crouch` | transition | pick_targets [count % 3 = 0] / safe_to_pick false
- `crouch -> approach` | transition | planAndExecute() returns
- `approach -> grasp` | transition | planAndExecute() returns
- `grasp -> retract` | transition | planAndExecute() returns
- `retract -> bin` | transition | planAndExecute() returns
- `bin -> crouch` | transition | planAndExecute() returns [more tomatoes] / i += 3
- `bin -> waiting` | transition | planAndExecute() returns [no more tomatoes] / safe_to_pick true

## Page 2: Target preparation in /tomato_picker

- state_owner: /tomato_picker, `safe_to_pick` (bool) and `picked_ids` (set), `nucbox_archive/nucbox_archive/agrobot_ws/src/robot_commander/src/tomato_picker.py`
- view: Explicit.

### Semantics

- Picking enabled and Picking paused are the two values of safe_to_pick.
- This machine runs independently of the pick task on page 1.
- A solid arrow is a transition in the code. A self-loop keeps the state and can change picked_ids.
- Gray boxes are context. Amber boxes are limitations that change the output.

### Nodes

- `initial` | kind: initial | initial marker | Node start. safe_to_pick is True.
- `enabled` | kind: state | Picking enabled
  - safe_to_pick = True
- `paused` | kind: state | Picking paused
  - safe_to_pick = False
  - Skips new batches only
- `ctx_sources` | kind: context | Publishers of /agrobot/safe_to_pick
  - /tomato_detector: true if a frame has a detection, false on watchdog
  - /commander: false at pick start, true at pick end
- `ctx_tf` | kind: limitation | Rail to camera transform
  - No publisher in the archive.
  - Without it, each batch ends with no output.
- `ctx_ids` | kind: limitation | picked_ids
  - Never cleared. It holds per-frame indexes. The node skips a reused index.
- `ctx_out` | kind: context | /agrobot/pick_targets
  - geometry_msgs/msg/PoseArray
  - Subscriber: /commander (page 1)

### Edges

- `initial -> enabled` | transition | node start
- `enabled -> paused` | transition | /agrobot/safe_to_pick False / warn
- `paused -> enabled` | transition | /agrobot/safe_to_pick True
- `enabled -> enabled` | transition | tomato_spatial batch [≥ 1 new, confident, reachable tomato] / publish PoseArray, add IDs to picked_ids
- `enabled -> enabled` | transition | tomato_spatial batch [bad JSON, no new tomato ≥ 0.6, none within 1.1 m, or transform failure] / no output
- `paused -> paused` | transition | tomato_spatial batch / skip (warn)
