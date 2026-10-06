# EPOS2 motor drive — Level 5 state machine (CiA 402 as commanded)

- source_pdf: `pdf_docs/drive-states.pdf`
- workspace: `nucbox_archive/nucbox_archive/agrobot_ws` · `nucbox_archive/nucbox_archive/ingenia_overlay_ws` · `nucbox_archive/nucbox_archive/AgrobotV2`
- configuration: Hardware stack: `EPOS2_ENABLE_J0=1 robot_stack/hw_moveit_stack.sh start` (EPOS2 J0 and J2–J6, Copley J1, arm_trajectory_fanout, move_group). EPOS2 drives on can0: J0 node 10, J2 node 2, J3 node 7, J4 node 113, J5 node 3, J6 node 24.
- verification: Source-derived from the NucBox archive. No runtime verification.
- state_owner: The EPOS2 drive firmware. The bridge writes controlword 0x6040 and mode 0x6060 by SDO and reads statusword 0x6041 by TPDO1 or SDO. `nucbox_archive/nucbox_archive/agrobot_ws/src/epos2_bridge/epos2_bridge/epos2_joint_bridge.py`
- view: Simplified. Only the CiA 402 states and transitions that the bridge uses. CiA 402 names.
- comparison: The repository copy ends each goal in Maxon Position Mode 0xFF. The NucBox copy does not use mode 0xFF.

## Semantics

- Boxes are CiA 402 drive states. The bridge does not store them. It writes commands and reads the statusword.
- A dark solid arrow is a controlword write from the bridge (SDO 0x6040). A gray arrow is a drive-internal transition.
- A dashed arrow is an inferred effect. The drive response is not verified in the sources.
- The FJT path writes 0x80, 0x06, 0x0F at the start of each goal and 0x1F, then 0x0F, during the goal.

## Nodes

- `initial` | kind: initial | initial marker | Drive power-up.
- `not_ready` | kind: state | Not Ready to Switch On
  - Power-up self-test
- `switch_on_disabled` | kind: state | Switch On Disabled
  - After power-up or fault reset
- `ready_to_switch_on` | kind: state | Ready to Switch On
  - Power stage off
- `operation_enabled` | kind: composite state | Operation Enabled
  - statusword bit 2 set (or 0x0737) · mode 7 written by SDO before enable
- `ipm_active` | kind: state | IPM active
  - controlword 0x001F
  - 0x20C4:01 bit 15 set
  - PVT records from RPDO1
- `ipm_inactive` | kind: state | IPM inactive
  - controlword 0x000F, mode 7
  - 0x20C4:01 bit 15 low
  - FIFO cleared and enabled
- `fault_reaction` | kind: state | Fault Reaction Active
  - Drive stops per its settings
- `fault` | kind: state | Fault
  - statusword bit 3 set
- `ctx_mode` | kind: context | Mode and PDO use
  - 0x6060 = 7 by SDO at each goal
  - Maxon Position Mode 0xFF: not used
  - RPDO2 0x302: not sent
- `ctx_not` | kind: context | Not commanded
  - Quick stop 0x0002, disable voltage 0x0000, disable operation 0x0007
- `ctx_nmt` | kind: context | NMT and PDO setup
  - start_epos2_ordered.sh restores the RPDO1 0x20C1 map and sends NMT start before the bridges start
- `ctx_fault` | kind: context | Fault visibility
  - TPDO1 statusword bit 3 goes to /epos2/jN/fault.
  - The FJT wait aborts on it.
- `ctx_after` | kind: unresolved | After a goal
  - IPM inactive, mode 7.
  - Position hold by the drive: not verified

## Edges

- `initial -> not_ready` | transition | power-up
- `not_ready -> switch_on_disabled` | drive-internal | automatic
- `switch_on_disabled -> ready_to_switch_on` | transition | cw 0x0006 Shutdown (enable_operation)
- `ready_to_switch_on -> ipm_inactive` | transition | cw 0x000F Enable operation (passes Switched On)
- `ipm_inactive -> ipm_active` | transition | cw 0x001F [FIFO prebuffered and verified]
- `ipm_active -> ipm_inactive` | transition | cw 0x000F (disarm_ipm or cleanup)
- `operation_enabled -> switch_on_disabled` | inferred | cw 0x0080 (clear_fault at each goal). Bits 0–3 low match the Disable Voltage pattern. Drive response with bit 7 set: not verified
- `operation_enabled -> fault_reaction` | drive-internal | drive error, for example following error or IPM buffer error
- `fault_reaction -> fault` | drive-internal | reaction done
- `fault -> switch_on_disabled` | transition | cw 0x0080 Fault reset (clear_fault)
