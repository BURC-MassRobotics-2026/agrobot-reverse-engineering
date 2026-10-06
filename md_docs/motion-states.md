# Motion execution — Level 3 state machines

- source_pdf: `pdf_docs/motion-states.pdf`
- workspace: `nucbox_archive/nucbox_archive/agrobot_ws` · `nucbox_archive/nucbox_archive/ingenia_overlay_ws` · `nucbox_archive/nucbox_archive/AgrobotV2`
- configuration: Hardware stack: `EPOS2_ENABLE_J0=1 robot_stack/hw_moveit_stack.sh start` (EPOS2 J0 and J2–J6, Copley J1, arm_trajectory_fanout, move_group). hw_moveit_stack.sh sets /move_group trajectory_execution.allowed_execution_duration_scaling 10.0 and allowed_goal_duration_margin 5.0 at run time.
- verification: Source-derived from the NucBox archive. No runtime verification.
- comparison: The repository version shows /epos2_fjt_fanout as the parent and a streaming EPOS2 child. The NucBox parent is /arm_trajectory_fanout and the EPOS2 child prebuffers its FIFO.

## Page 1: Fan-out parent goal

- state_owner: /arm_trajectory_fanout, the ROS action goal status, `nucbox_archive/nucbox_archive/ingenia_overlay_ws/src/arm_trajectory_fanout/arm_trajectory_fanout/arm_trajectory_fanout.py`
- view: Inferred phases. Explicit outcomes.

### Semantics

- The node has no phase variable. The phase boxes follow the code order in _goal_cb and _execute (inferred).
- Outcome boxes are ROS action terminal states with their error_code.
- A solid arrow is a transition in the code. Labels use event [guard] / effect. A dotted line is context.
- The node never calls canceled(). A cancel request does not have its own outcome.

### Nodes

- `initial` | kind: initial | initial marker | A parent goal arrives from /move_group.
- `received` | kind: inferred phase | Goal received
  - _goal_cb
- `rejected` | kind: outcome | Rejected
  - Goal does not execute
- `executing` | kind: composite state | Executing
  - Synchronous callback
- `dispatching` | kind: inferred phase | Dispatching
  - For each mapped joint, in order: wait for server (5 s), split, skip a no-op child, send goal, wait for acceptance (5 s)
- `awaiting` | kind: inferred phase | Awaiting results
  - One child at a time, in joint order, T + 8 s maximum for each
- `aborted_exc` | kind: outcome | Aborted
  - PATH_TOLERANCE_VIOLATED error_string: exception text
- `aborted_child` | kind: outcome | Aborted
  - PATH_TOLERANCE_VIOLATED
  - "Child trajectory failures"
- `succeeded` | kind: outcome | Succeeded
  - SUCCESSFUL
  - "Fanout succeeded …"
- `ctx_monitor` | kind: context | MoveIt execution monitor
  - Run-time values: scaling 10.0, margin 5.0 s. MoveIt cancels the goal when the run is longer.
- `ctx_cancel` | kind: context | Cancel request
  - _cancel_cb forwards it to the child goals in the active list. The list is empty during Dispatching.
  - The parent then ends as Aborted or Succeeded.
- `ctx_children` | kind: context | Child servers
  - joint0, joint2–joint6: EPOS2 (page 2) joint1: Copley /copley_j1_bridge
  - No shared start time

### Edges

- `initial -> received` | transition | goal request
- `received -> rejected` | transition | [no joint names, no points, or no mapped joint] / REJECT
- `received -> dispatching` | transition | [else] / ACCEPT (unmapped joints ignored)
- `dispatching -> awaiting` | transition | [last mapped joint done] / store active list
- `dispatching -> aborted_exc` | transition | server not ready, send timeout, or rejection / abort
- `awaiting -> aborted_exc` | transition | result wait longer than T + 8 s / abort
- `awaiting -> aborted_child` | transition | all results [any error_code ≠ SUCCESSFUL] / abort
- `awaiting -> succeeded` | transition | all results [all SUCCESSFUL] / succeed
- `ctx_cancel -> awaiting` | context | cancel request reaches child goals

## Page 2: EPOS2 child goal

- state_owner: /epos2_jN_bridge, the ROS action goal status, `nucbox_archive/nucbox_archive/agrobot_ws/src/epos2_bridge/epos2_bridge/epos2_joint_bridge.py`
- view: Inferred phases. Explicit outcomes.

### Semantics

- Phases follow the code order in _execute_follow_joint_trajectory. Level 4 page 1 shows the IPM lifecycle FSM behind them.
- Outcome boxes are ROS action terminal states with their error_code.
- Each failure path runs _ipm_cleanup_after_abort before the result. Cleanup sends controlword 0x0F.
- A solid arrow is a transition in the code. Labels use event [guard] / effect.

### Nodes

- `initial` | kind: initial | initial marker | A child goal arrives from /arm_trajectory_fanout.
- `received` | kind: inferred phase | Goal received
  - _goal_callback
- `setup` | kind: inferred phase | Setup
  - Entry guard, enable, FIFO, prebuffer, activate 0x1F
  - IPM FSM PREPARING → ACTIVE
- `executing` | kind: inferred phase | Executing
  - IPM active. Wait ≥ T for the target, timeout T + 5 s. Feedback each 20 ms
  - IPM FSM ACTIVE
- `completing` | kind: inferred phase | Completing
  - disarm_ipm: controlword 0x0F
  - Check IPM inactive
  - IPM FSM SETTLING → DISARMING
- `rejected` | kind: outcome | Rejected
  - Goal does not execute
- `canceled` | kind: outcome | Canceled
  - error_code PATH_TOLERANCE_VIOLATED
  - "Goal canceled"
- `succeeded` | kind: outcome | Succeeded
  - error_code SUCCESSFUL
  - IPM inactive, mode 7, no Position
  - Mode hold
- `aborted` | kind: outcome | Aborted
  - INVALID_GOAL: setup failed
  - GOAL_TOLERANCE_VIOLATED: fault, timeout, or feeder error
  - PATH_TOLERANCE_VIOLATED: exception after activation
- `ctx_cancel` | kind: context | Cancel
  - Checked only in Executing.
  - Setup does not check it.
- `ctx_sync` | kind: context | Start time
  - header.stamp is not used.
- `ctx_l4` | kind: context | Level 4
  - IPM FSM and BridgeState: bridge-states

### Edges

- `initial -> received` | transition | goal request
- `received -> rejected` | transition | [joint_names ≠ [jointN] or no points] / REJECT
- `received -> setup` | transition | [else] / ACCEPT
- `setup -> executing` | transition | IPM active [FIFO verified]
- `executing -> completing` | transition | [t ≥ T, |error| ≤ 0.01 rad, |v| ≤ 0.10 rad/s]
- `completing -> succeeded` | transition | [IPM inactive] / succeed
- `executing -> canceled` | transition | cancel requested / cleanup, canceled()
- `setup -> aborted` | transition | failed check or exception / cleanup, abort (INVALID_GOAL)
- `executing -> aborted` | transition | drive fault, timeout, or feeder error / cleanup, abort (GOAL_TOLERANCE_VIOLATED)
- `completing -> aborted` | transition | exception, IPM still active / cleanup, abort (PATH_TOLERANCE_VIOLATED)
