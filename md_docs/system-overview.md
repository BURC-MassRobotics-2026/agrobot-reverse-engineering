# System control flow — Level 1, system overview

- source_pdf: `pdf_docs/system-overview.pdf`
- workspace: `nucbox_archive/nucbox_archive/agrobot_ws` · `nucbox_archive/nucbox_archive/ingenia_overlay_ws` · `nucbox_archive/nucbox_archive/AgrobotV2`
- configuration: Hardware stack: `EPOS2_ENABLE_J0=1 robot_stack/hw_moveit_stack.sh start` (EPOS2 J0 and J2–J6, Copley J1, arm_trajectory_fanout, move_group). Perception: `AgrobotV2/perception/launch/perception.launch.py` with a point-cloud topic. `robot_commander/launch/commander.launch.py`. `agrobot_supervisor/launch/supervisor.launch.py` runs separately. hw_moveit_stack.sh does not include it.
- verification: Source-derived from the NucBox archive. No runtime verification.
- comparison: The repository copy `Agrobot/src` is older. Its /commander has the /agrobot/pick_sequence action and does not subscribe to /agrobot/pick_targets. It has no supervisor, motion, or Copley packages.

## Semantics

- Boxes are ROS nodes or collapsed node groups. Gray boxes are external drivers or hardware.
- An amber box marks an output with no consumer or a missing input.
- Each arrow points from the sender or client to the receiver or server.
- The edge kind is TOPIC, SERVICE, ACTION, or CAN. An arrow does not show execution order.
- Two nodes publish /agrobot/safe_to_pick: /tomato_detector and /commander.
- The supervisor and /commander are separate paths. The supervisor does not send pick targets.

## Nodes

- `camera` | Camera | external
  - RealSense driver
  - Started outside these files
- `perception` | Perception | node group | Nodes: /tomato_detector, /agrobot/tomato_spatial, /agrobot/tomato_tracker, /agrobot/qwen_vl.
  - AgrobotV2, 4 nodes detector, spatial, tracker, qwen_vl
- `supervisor` | Search supervisor | node | Steps the rail and queues tomatoes. It does not command picks.
  - /agrobot_supervisor
  - Pick step: stub
- `movers` | Rail and arm movers | node | MoveIt clients for the rail and anthro groups.
  - /rail_mover
  - /anthro_mover
- `pick_target_gap` | Unused pick output | unresolved | /agrobot/qwen_vl publishes geometry_msgs/msg/PoseStamped. No node in the archive subscribes.
  - TOPIC /agrobot/pick_target
  - No subscriber found
- `tomato_picker` | Target generation | node | No source in the archive publishes linear_rail_link to camera_color_optical_frame.
  - /tomato_picker
  - Camera TF source: not found
- `commander` | Pick and command handling | node
  - /commander
- `move_group` | Motion planning | node
  - /move_group
- `sdo_servers` | CANopen SDO servers | node group | ProxyDriver per EPOS2 node gives /node_N/sdo_read and /node_N/sdo_write.
  - ros2_canopen master, node 100
  - /node_22 shim for J1
- `bridges` | Joint bridges | node group | One bridge node per joint. Each serves one FollowJointTrajectory action.
  - EPOS2: J0, J2–J6
  - Copley: J1
- `fanout` | Trajectory dispatch | node
  - /arm_trajectory_fanout
- `drives` | Motor drives | hardware
  - CAN bus can0, 1 Mbit/s
  - EPOS2 ×6, Copley ×1

## Edges

- `camera -> perception` | TOPIC | color image, point cloud, camera info
- `perception -> supervisor` | TOPIC | /agrobot/tomato_tracks
- `supervisor -> movers` | SERVICE | /rail_mover/goto, /anthro_mover/goto
- `movers -> move_group` | ACTION | MoveIt plan and execute
- `perception -> tomato_picker` | TOPIC | /agrobot/tomato_spatial (JSON string)
- `perception -> tomato_picker` | TOPIC | /agrobot/safe_to_pick from /tomato_detector
- `perception -> pick_target_gap` | TOPIC | /agrobot/pick_target from /agrobot/qwen_vl | No subscriber found.
- `tomato_picker -> commander` | TOPIC | /agrobot/pick_targets (PoseArray)
- `commander -> tomato_picker` | TOPIC | /agrobot/safe_to_pick from /commander | False at pick start, true at pick end.
- `commander -> move_group` | ACTION | MoveIt plan and execute
- `move_group -> fanout` | ACTION | /arm_controller/follow_joint_trajectory
- `fanout -> bridges` | ACTION | per-joint FollowJointTrajectory
- `bridges -> move_group` | TOPIC | /joint_states
- `bridges -> sdo_servers` | SERVICE | /node_N/sdo_read, /node_N/sdo_write
- `sdo_servers -> drives` | CAN | SDO transfers
- `bridges -> drives` | CAN | PVT records and controlword (raw SocketCAN)
- `drives -> bridges` | CAN | TPDO drive state, heartbeat, EMCY
