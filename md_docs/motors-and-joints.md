# Agrobot motors and joints — relationship map, overview

- source_pdf: `pdf_docs/motors-and-joints.pdf`
- scope: Each actuator on the robot and the URDF joint that it moves.
- configuration: Deployed NucBox archive `nucbox_archive/nucbox_archive` (`agrobot_ws`, `robot_stack`, `ingenia_overlay_ws`). The last run logs are from 2026-05-28.
- basis: Source-derived from configuration files, start scripts, drive settings files, and run logs. No runtime verification.
- comparison: The repository copy `Agrobot/src/epos2_bridge/config/joints/j2..j6.yaml` and both README motor tables are older. They show different node IDs, drive models, and gear ratios. They also show an Ingenia drive on J0.

## Semantics

- Blue boxes on the left are drives or actuators. Blue boxes in the middle are URDF joints.
- Gray groups show the bus that connects each drive. Group membership does not show execution order.
- A solid arrow points from a drive to the joint that it moves.
- A dotted arrow points from a source joint to a mimic joint.
- A dotted line without an arrow attaches an amber note to a joint row.
- Amber notes show unresolved or conflicting facts.
- "Motor: not found" means that no motor part number is in the archive.
- Each scaling value shows encoder quadrature counts per motor revolution and gear ratio (motor rev per joint rev).

## Nodes

- `can_bus` | kind: group | CAN bus can0 | Members: `drv_j0`, `drv_j1`, `drv_j2`, `drv_j3`, `drv_j4`, `drv_j5`, `drv_j6`.
- `http_bus` | kind: group | HTTP · gripper-board.local | Members: `drv_grip`.
- `drv_j0` | kind: drive | Maxon EPOS2 (J0) | CAN node 10 · epos2_bridge
  - Motor: not found · 25600 qc/rev · 20:1
  - `linear_m_per_joint_rad`: 0.025
- `drv_j1` | kind: drive | Copley amplifier (J1) | CAN node 22 · copley_bridge
  - Motor: Harmonic Drive SHA32A (`SHA32A81SG-B12A200-10S17bA`) · 81:1
- `drv_j2` | kind: drive | Maxon EPOS2 70/10 (J2) | CAN node 2 · epos2_bridge
  - Motor: not found · 25600 qc/rev · 160:1
- `drv_j3` | kind: drive | Maxon EPOS2 70/10 (J3) | CAN node 7 · epos2_bridge
  - Motor: not found · 25600 qc/rev · 160:1
- `drv_j4` | kind: drive | Maxon EPOS2 50/5 (J4) | CAN node 113 · epos2_bridge
  - Motor: not found · 16384 qc/rev · 221.91:1
- `drv_j5` | kind: drive | Maxon EPOS2 50/5 (J5) | CAN node 3 · epos2_bridge
  - Motor: not found · 16384 qc/rev · 80:1
- `drv_j6` | kind: drive | Maxon EPOS2 50/5 (J6) | CAN node 24 · epos2_bridge
  - Motor: not found · 16384 qc/rev · 80:1
- `drv_grip` | kind: actuator | Gripper servo | Servo model: not found
  - Command: HTTP `servo/move?deg=<d>&ms=<ms>`. Close is 0°, open is 240°, time is 500 ms.
  - The servo is not in ros2_control.
- `joint0` | kind: joint | joint0 | prismatic · linear rail
- `joint1` | kind: joint | joint1 | revolute
- `joint2` | kind: joint | joint2 | revolute
- `joint3` | kind: joint | joint3 | revolute
- `joint4` | kind: joint | joint4 | revolute
- `joint5` | kind: joint | joint5 | revolute
- `joint6` | kind: joint | joint6 | revolute
- `fin_joint1` | kind: joint | fin_joint1 | mock state · fixed at 0
  - `mock_components/GenericSystem` holds the state. A filler script publishes 0. No gripper feedback.
- `fin_joint2` | kind: joint | fin_joint2 | mimic of fin_joint1
- `fin_joint3` | kind: joint | fin_joint3 | mimic of fin_joint1
- `note_j0` | kind: note, unresolved | Drive model not stated | Ingenia EVS-XCR-C drive retired on 2026-05-25
- `note_j1` | kind: note, conflict | Conflicting J1 facts
  - Amp: APZ-090-50 (yaml) vs APV (.ccx)
  - Encoder: 65536 (yaml) vs 131072 (.ccx)
- `note_j4` | kind: note, unresolved | Hand-scaled gear ratio | 221.912336 = 344 × 0.645

## Edges

- `drv_j0 -> joint0` | moves | Drive moves joint
- `drv_j1 -> joint1` | moves | Drive moves joint
- `drv_j2 -> joint2` | moves | Drive moves joint
- `drv_j3 -> joint3` | moves | Drive moves joint
- `drv_j4 -> joint4` | moves | Drive moves joint
- `drv_j5 -> joint5` | moves | Drive moves joint
- `drv_j6 -> joint6` | moves | Drive moves joint
- `drv_grip -> fin_joint1` | moves | Drive moves joint | The robot model does not get this motion.
- `fin_joint1 -> fin_joint2` | mimic | mimic ×1.0
- `fin_joint1 -> fin_joint3` | mimic | mimic ×1.0
- `joint0 -- note_j0` | note | The note applies to the joint0 row.
- `joint1 -- note_j1` | note | The note applies to the joint1 row.
- `joint4 -- note_j4` | note | The note applies to the joint4 row.
