# Joint limit and collision checks (2 pages: check coverage map, issues by severity)

- source_pdf: `pdf_docs/joint-limits-collision-checks.pdf`
- basis: `Agrobot_Joint_Limits_and_Collision_checks.txt` (source-derived review of the `Agrobot` repo). Spot checks against `Agrobot/src` agree.
- configuration: staged `epos2_fjt_fanout` configuration (J0 + J2)
- verification: source-derived. Not runtime-verified. Items marked `needs check on robot` are uncertain.

## Page 1: Joint limit and collision checks along the move chain

### Semantics

- Boxes are stages of the move request, context, or a gap in checks.
- `MOVE` edges show the direction of the move request. They are source-derived.
- `CONTEXT` edges use dotted lines. They show a data source or group membership. They do not show a call.
- Issue IDs (H, M, L) in a node refer to the issues on page 2.

### Nodes

- `commander` | kind: stage | Commander | `/commander`
  - Check: none.
  - Sets speed + acceleration scaling to 100%. Issue: M1.
- `moveit` | kind: stage | MoveIt | `/move_group`
  - Check: joint limits + collisions (robot with itself and rail).
  - Issues: H4, M2, M3, L1, L2.
- `fanout` | kind: stage | Fanout | `epos2_fjt_fanout`
  - Check: none. Issue: H5.
- `bridge` | kind: stage | Bridge | EPOS2 joint bridge
  - Check: joint name + non-empty goal only. Issues: H1, H2.
- `can_lib` | kind: stage | CAN library | `ros2_canopen`
  - Check: none.
- `drive` | kind: stage | Motor drive | EPOS2
  - Built-in limits: our code does not set them. Issues: H3, M3.
- `urdf` | kind: context | Robot model (URDF)
  - Defines joint limits. MoveIt config can override them (M2).
- `no_later_check` | kind: gap | No check after MoveIt
  - Later stages do not check limits or collisions again.
  - Exception: drive speed limit.
- `limits_table` | kind: reference | Joint limits (reference)
  - J0 | Rail | min 0 m | max 1.565 m | max speed 1.0 m/s (MoveIt value. Robot model value: 0.5 m/s, M2)
  - J1 | Shoulder turn | -180° | +180° | 0.40 rad/s
  - J2 | Shoulder pitch | -90° | +90° | 1.96 rad/s
  - J3 | Elbow | -258° | +86° | 3.65 rad/s
  - J4 | Wrist 1 | -180° | +180° | 2.00 rad/s
  - J5 | Wrist 2 | -286° | +286° | 3.50 rad/s
  - J6 | Wrist turn | 0° | +120° | 1.00 rad/s
  - Fins | Gripper | -14° | +10° | 1.00 rad/s

### Edges

- `commander -> moveit` | MOVE | target | 100% scaling
- `moveit -> fanout` | MOVE | planned trajectory
- `fanout -> bridge` | MOVE | J0 + J2 only | the fanout drops other joints
- `bridge -> can_lib` | MOVE | motor commands
- `can_lib -> drive` | MOVE | CAN
- `urdf -> moveit` | CONTEXT | limits
- `no_later_check -- fanout` | CONTEXT | group member: no later check
- `no_later_check -- bridge` | CONTEXT | group member: no later check
- `no_later_check -- can_lib` | CONTEXT | group member: no later check
- `no_later_check -- drive` | CONTEXT | group member: no later check
- `limits_table` has no edges.

## Page 2: Joint limit and collision check issues by severity

### Semantics

- Hierarchy: severity group, then issue. Severity: high, medium, low.
- Each issue has: affected component, problem, consequence, proposed fix.
- Proposed fixes are proposals from the source document. They are not done.
- `uncertain: needs check on robot` is separate from severity.
- Group order and list order do not show sequence or cause.

### Nodes

- `sev_high` | kind: group | High severity | 5 issues
- `sev_medium` | kind: group | Medium severity | 3 issues
- `sev_low` | kind: group | Low severity | 2 issues
- `H1` | kind: issue | J2–J6 are not homed | group: `sev_high` | affects: bridge, drives
  - Problem: Joint configs set `homing_enabled: false` and `allow_unhomed_motion: true`. No code reads these flags. The bridge has no homing code. `ros2_canopen` can home a drive, but no code calls it. The README calls homing optional.
  - Consequence: Zero can be at a different position after each power-on. All joint limits can then point at wrong angles.
  - Proposed fix: Add homing (home switch or absolute encoder). Block bridge motion until homing is complete. Needs EE/ME for hardware.
- `H2` | kind: issue | Bridge does not check position or speed | group: `sev_high` | affects: bridge
  - Problem: The goal check accepts a matching joint name and a non-empty goal. It does not check the target position or the speed.
  - Consequence: Any ROS program (test script, typo, bug) can send a bad target directly to the motor.
  - Proposed fix: Add min/max position checks and a max speed check in the bridge.
- `H3` | kind: issue | Our code does not set drive limits | group: `sev_high` | affects: drives (EPOS2) | uncertain: needs check on robot
  - Problem: Code does not set position (0x607D), speed (0x607F), acceleration (0x60C5), or following-error (0x6065) limits. Profile speed/accel (0x6081/0x6083) only give a warning. Saved file (one drive, March): position limit off, acceleration limit off, speed limit 1960 rpm (active: the drive faults and stops).
  - Consequence: The drive is the last safety net. With the position limit off, the drive goes to any commanded position.
  - Proposed fix: Read the settings from each drive. Then set real limits. Needs EE.
- `H4` | kind: issue | MoveIt does not know the environment | group: `sev_high` | affects: MoveIt
  - Problem: No code adds obstacles. MoveIt knows only the robot and the rail. Missing: floor, plants, wall, bin, camera mount, electronics box.
  - Consequence: MoveIt can plan a path through the floor or the plants.
  - Proposed fix: Add the floor, plants/wall, bin, and mounts at startup. Needs measurements from ME.
- `H5` | kind: issue | MoveIt checks one path, the robot follows another | group: `sev_high` | affects: fanout | uncertain: needs check on robot
  - Problem: The fanout in this repo moves only J0 and J2. For a 7-joint goal, it drops the other 5 joints and logs a warning. The README says that `arm_trajectory_fanout` (NucBox) is the daily fanout. Its behavior is not known.
  - Consequence: The collision check applies to a path that the robot does not follow.
  - Proposed fix: Set `allow_extra_goal_joints` to false, or plan only with the connected joints.
- `M1` | kind: issue | Commander runs at 100% speed | group: `sev_medium` | affects: commander
  - Problem: The MoveIt config default is 10%. The commander overrides it to 100%.
  - Consequence: Every move runs at full speed: J3 up to 3.65 rad/s (~209 deg/s), rail up to 1.0 m/s.
  - Proposed fix: Use 10–20% during tests.
- `M2` | kind: issue | Rail (J0) speed limit is double the model value | group: `sev_medium` | affects: MoveIt config
  - Problem: Robot model: 0.5 m/s. MoveIt config: 1.0 m/s (changed May 9. The backup from that day has 0.5). MoveIt uses 1.0 m/s.
  - Consequence: The rail carries the whole arm. The stopping distance at 1 m/s is not known.
  - Proposed fix: Ask ME for the max speed. Measure the stopping distance.
- `M3` | kind: issue | MoveIt speeds exceed the motor speed limit | group: `sev_medium` | affects: MoveIt, drives | uncertain: needs check on robot
  - Problem: The saved drive file caps the motor at 1960 rpm. After the gearbox, MoveIt limits need J2 3144 rpm, J3 3485 rpm, J5 3342 rpm (too fast). J4 (1910 rpm) and J6 (955 rpm) are OK.
  - Consequence: At 100% speed, a long fast move can exceed the cap. The drive then faults and the arm stops mid-move.
  - Proposed fix: When the team knows the real motor max speed, lower those joint speeds to match.
- `L1` | kind: issue | gripper_closed pose is out of range | group: `sev_low` | affects: MoveIt SRDF
  - Problem: The pose sets `fin_joint1` to 1.0. The max is 0.175. No code uses the pose yet (no gripper controller).
  - Consequence: No effect today.
  - Proposed fix: Use a value from -0.25 to 0.175.
- `L2` | kind: issue | Some link pairs skip collision checks | group: `sev_low` | affects: MoveIt SRDF
  - Problem: Skipped pairs: rail/link2, rail/link3, base/link2, base/link3, rail/link1. A check with the real collision shapes over the full joint range found no contact. Closest: link3 to rail, 16 mm, at J2 = +90°.
  - Consequence: Acceptable for now. The 16 mm margin holds only if the J2 limit is real (needs homing, H1) and the real rail matches the model.
  - Proposed fix: Check again after the team adds homing.

### Edges

- This page has no connectors. Group membership is in the `group` field of each issue.
