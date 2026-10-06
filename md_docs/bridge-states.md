# EPOS2 joint bridge — Level 4 state machines

- source_pdf: `pdf_docs/bridge-states.pdf`
- workspace: `nucbox_archive/nucbox_archive/agrobot_ws` · `nucbox_archive/nucbox_archive/ingenia_overlay_ws` · `nucbox_archive/nucbox_archive/AgrobotV2`
- configuration: Hardware stack: `EPOS2_ENABLE_J0=1 robot_stack/hw_moveit_stack.sh start` (EPOS2 J0 and J2–J6, Copley J1, arm_trajectory_fanout, move_group). One /epos2_jN_bridge node per joint. Config `config/joints/j0.yaml` and `j2.yaml`–`j6.yaml`.
- verification: Source-derived from the NucBox archive. No runtime verification.
- comparison: The repository copy has only BridgeState and gates the direct IPM paths with allow_legacy_ipm_services. The NucBox copy adds the IPM lifecycle FSM and has no gate.

## Page 1: IPM lifecycle FSM

- state_owner: /epos2_jN_bridge, `ipm_lifecycle_state` (string), `nucbox_archive/nucbox_archive/agrobot_ws/src/epos2_bridge/epos2_bridge/epos2_joint_bridge.py`
- view: Explicit.

### Semantics

- Each box is an explicit value of ipm_lifecycle_state. _ipm_fsm_set() checks each change against IPM_FSM_ALLOWED.
- Only _execute_follow_joint_trajectory and its cleanup change this FSM.
- A solid arrow is a transition in the code. Labels use event [guard] / effect.
- An arrow that starts at a circle on the Goal in progress border applies to each state inside it.

### Nodes

- `initial` | kind: initial | initial marker | _ipm_fsm_get() returns READY before the first set.
- `READY` | kind: state | READY
  - No goal active
- `FAULTED` | kind: state | FAULTED
  - Next goal must recover
- `in_progress` | kind: composite state | Goal in progress
- `PREPARING` | kind: state | PREPARING
  - clear_fault, mode 7, enable
- `PREBUFFERING` | kind: state | PREBUFFERING
  - Clear FIFO, write
  - PVT records
- `PREBUFFERED` | kind: state | PREBUFFERED
  - FIFO loaded, IPM inactive
- `DISARMING` | kind: state | DISARMING
  - controlword 0x0F
- `SETTLING` | kind: state | SETTLING
  - Target reached
- `ACTIVE` | kind: state | ACTIVE
  - IPM active, wait loop
- `ctx_outside` | kind: limitation | Outside this FSM
  - arm_ipm and the direct moves change the drive without this FSM. A later goal then finds IPM active and fails at entry.
- `ctx_table` | kind: context | Transition table
  - IPM_FSM_ALLOWED. An illegal change raises RuntimeError.
  - UNINITIALIZED is in the table.
  - No code sets it.
- `ctx_snapshot` | kind: limitation | Fault snapshot
  - Cleanup calls read_fault_snapshot().
  - The NucBox code does not define it.
  - The error is caught and logged.

### Edges

- `initial -> READY` | transition | node start
- `READY -> PREPARING` | transition | FJT goal [IPM inactive]
- `FAULTED -> PREPARING` | transition | FJT goal [IPM inactive]
- `FAULTED -> READY` | transition | cleanup [no fault, IPM inactive]
- `PREPARING -> PREBUFFERING` | transition | enable ok [IPM inactive]
- `PREBUFFERING -> PREBUFFERED` | transition | [FIFO level ≥ records]
- `PREBUFFERED -> ACTIVE` | transition | controlword 0x1F [IPM active]
- `ACTIVE -> SETTLING` | transition | [t ≥ T, |error| ≤ 0.01 rad, |v| ≤ 0.10 rad/s]
- `SETTLING -> DISARMING` | transition | / disarm_ipm()
- `ACTIVE -> DISARMING` | transition | cancel, drive fault, timeout, or feeder error / cleanup
- `DISARMING -> READY` | transition | [IPM inactive, no fault]
- `in_progress -> FAULTED` | transition | failed step, exception, or cleanup with a fault or IPM active / _ipm_fsm_fault()

## Page 2: BridgeState

- state_owner: /epos2_jN_bridge, `self.bridge_state` (enum BridgeState), `nucbox_archive/nucbox_archive/agrobot_ws/src/epos2_bridge/epos2_bridge/epos2_joint_bridge.py`
- view: Explicit.

### Semantics

- Each box is an explicit value of the BridgeState enum. state_summary publishes it.
- BridgeState is a coarse summary. The IPM lifecycle FSM on page 1 controls goal execution.
- A solid arrow is a transition in the code. An arrow that starts at a circle on the border applies to each state.
- MOVING and the direct-move return to IPM_ARMED come from both the FJT path and the direct moves.

### Nodes

- `initial` | kind: initial | initial marker | __init__ sets IDLE.
- `bridge_node` | kind: composite state | BridgeState
  - self.bridge_state
- `IDLE` | kind: state | IDLE
  - Initial value
  - FJT setup or arm_ipm start
- `IPM_ARMED` | kind: state | IPM_ARMED
  - IPM active after arm_ipm, or FJT settling
- `MOVING` | kind: state | MOVING
  - FJT active, or a direct move
- `READY` | kind: state | READY
  - After disarm, cleanup, or clear_fault
- `FAULTED` | kind: state | FAULTED
  - A command step failed
  - Not set by telemetry
- `ctx_keepalive` | kind: context | Keepalive
  - Sends hold records only in IPM_ARMED with ipm_armed.
- `ctx_summary` | kind: context | Visibility
  - state_summary shows this value.
- `ctx_telemetry` | kind: context | Telemetry
  - The fault bit does not change
  - BridgeState.

### Edges

- `initial -> IDLE` | transition | node start
- `bridge_node -> IDLE` | transition | FJT goal / FSM PREPARING, or arm_ipm / start sequence
- `IDLE -> MOVING` | transition | FJT [IPM active]
- `IDLE -> IPM_ARMED` | transition | arm_ipm [IPM active, buffer enabled]
- `MOVING -> IPM_ARMED` | transition | FJT target reached, or direct move done, out of tolerance, or timeout
- `IPM_ARMED -> MOVING` | transition | direct move [ipm_armed]
- `IPM_ARMED -> READY` | transition | disarm_ipm [no fault]
- `bridge_node -> READY` | transition | clear_fault [ok], or cleanup [no fault, IPM inactive]
- `bridge_node -> FAULTED` | transition | clear_fault [fails], failed step, drive fault, or exception
