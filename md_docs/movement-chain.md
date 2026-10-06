# Movement chain: J3 command and feedback (control flow, Level 1 system overview)

- source_pdf: `pdf_docs/movement-chain.pdf`
- basis: `Agrobot_Movement_Chain_FINAL.docx`, checked against `Agrobot/src` (`robot_commander`, `epos2_bridge`, `moveit_config`)
- workspace: `Agrobot/src` (same files as the NucBox `agrobot_ws` copy)
- configuration: `epos2_arm_controller` + `epos2_j3_bridge`. The controller serves `/arm_controller/follow_joint_trajectory` for `joint3` only. `epos2_fjt_fanout` serves the same action name in a different configuration. This view does not show that configuration.
- verification: source-derived. Not runtime-verified.

## Semantics

- Boxes are ROS nodes, external inputs, hardware, or context.
- Kinds: `ROS node`, `external` (input, hardware, or service provider), `scope` (scope limit).
- Edge kinds: `TOPIC`, `ACTION`, `SERVICE`, `CAN` (hardware transport), `CONTEXT` (undirected, no call or data flow).
- All edges except `CONTEXT` are source-derived command or feedback paths. Arrows show the direction of the command or data.
- An action edge includes goal and result. A topic edge does not set an execution order.
- List order does not set execution order.

## Nodes

- `clients` | Command clients | kind: external | External movement requests
- `commander` | Command handling | kind: ROS node | `/commander`
- `move_group` | Motion planning | kind: ROS node | `/move_group`, planning group `"arm"`
- `arm_controller` | J3 trajectory control | kind: ROS node | `/epos2_arm_controller`
- `j3_bridge` | J3 bridge | kind: ROS node | `/epos2_j3_bridge` | Converts joint units to motor units and motor units to joint units
- `drive` | EPOS2 drive (J3) | kind: external | CAN hardware
- `canopen` | CANopen services | kind: external | `ros2_canopen`, setup and configuration
- `scope_j3` | Traced path: joint3 only | kind: scope
  - Other joints: not traced.
  - Rail (J0) hardware path: not traced.

## Edges

- `clients -> commander` | TOPICS | pose, joint, position, and pick-sequence commands (`/agrobot/*`)
- `commander -> move_group` | ACTIONS / SERVICE | plan + execute target
- `move_group -> arm_controller` | ACTION | `FollowJointTrajectory`, arm trajectory | result: succeed or abort
- `arm_controller -> j3_bridge` | TOPIC | `/epos2/j3/reduced_traj` | reduced PVT trajectory
- `arm_controller -> j3_bridge` | SERVICES | arm before motion, disarm after
- `j3_bridge -> drive` | CAN | motor commands
- `drive -> j3_bridge` | CAN | actual motor state
- `j3_bridge -> canopen` | SERVICES | SDO read / write
- `j3_bridge -> arm_controller` | TOPIC | `/joint_states` | current state: actual position + velocity
- `j3_bridge -> move_group` | TOPIC | `/joint_states` | current state: actual position + velocity
- `arm_controller -- scope_j3` | CONTEXT | the traced hardware path applies to joint3 only
