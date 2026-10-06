# /commander node detail — Level 2, package / node detail

- source_pdf: `pdf_docs/commander-detail.pdf`
- workspace: `nucbox_archive/nucbox_archive/agrobot_ws` · `nucbox_archive/nucbox_archive/ingenia_overlay_ws` · `nucbox_archive/nucbox_archive/AgrobotV2`
- configuration: `robot_commander/launch/commander.launch.py`: node /commander with `moveit_config.to_dict()`. Default namespace. No remappings.
- verification: Source-derived from the NucBox archive. No runtime verification.
- source: `nucbox_archive/nucbox_archive/agrobot_ws/src/robot_commander/src/commander.cpp`
- comparison: The repository copy `Agrobot/src/robot_commander` has the /agrobot/pick_sequence action, /agrobot/pose_cmd, and /agrobot/proceed. The NucBox copy has none of these.

## Page 1: Topic dispatch

### Semantics

- Gray boxes on the left are topics that /commander subscribes to. No publisher of the three command topics is in the archive.
- Blue boxes inside the /commander boundary are callbacks and local functions.
- A solid arrow is a ROS message. A dashed arrow is a local function call.
- rclcpp::spin uses one thread. One callback runs at a time. A pick blocks the other callbacks until it ends.

### Nodes

- `t_named` | /agrobot/named_pose_cmd | topic
  - robot_interfaces/msg/PoseCommand
  - Publisher: not in archive
- `named_cb` | namedPoseCmdCallback | callback
  - Accept crouch, attention, vertical, or bin. Ignore others.
- `t_joint` | /agrobot/joint_cmd | topic
  - robot_interfaces/msg/JointCommand
  - Publisher: not in archive
- `joint_cb` | jointCmdCallback | callback
  - Read j0–j6 joint targets
- `t_position` | /agrobot/position_cmd | topic
  - robot_interfaces/msg/PositionCommand
  - Publisher: not in archive
- `position_cb` | positionCmdCallback | callback
  - Pose target, or Cartesian path when cartesian_path is true
- `t_pick` | /agrobot/pick_targets | topic
  - geometry_msgs/msg/PoseArray
  - Publisher: /tomato_picker
- `pick_cb` | pickTargetCallback | callback
  - Pick sequence: page 2
  - Returns after the last tomato
- `helpers` | Motion helpers | local functions
  - goToNamedTarget(name) goToJointTarget(j0–j6) goToPositionTarget(x … yaw, cartesian_path) goToPoseTarget(pose)
  - Each helper calls planAndExecute() or execute() on the shared
  - MoveGroupInterface (group arm).
  - MoveIt connections: page 3
- `safe_pub` | safe_to_pick publisher | publisher | Subscriber: /tomato_picker.
  - TOPIC /agrobot/safe_to_pick std_msgs/msg/Bool
- `commander_node` | group | /commander · subscriptions, callbacks, and publisher | contains: `named_cb`, `joint_cb`, `position_cb`, `pick_cb`, `helpers`, `safe_pub`
  - rclcpp::spin: one thread, one callback at a time

### Edges

- `t_named -> named_cb` | TOPIC | /agrobot/named_pose_cmd
- `t_joint -> joint_cb` | TOPIC | /agrobot/joint_cmd
- `t_position -> position_cb` | TOPIC | /agrobot/position_cmd
- `t_pick -> pick_cb` | TOPIC | /agrobot/pick_targets
- `named_cb -> helpers` | LOCAL | goToNamedTarget()
- `joint_cb -> helpers` | LOCAL | goToJointTarget()
- `position_cb -> helpers` | LOCAL | goToPositionTarget()
- `pick_cb -> helpers` | LOCAL | goToNamedTarget(), goToPoseTarget()
- `pick_cb -> safe_pub` | LOCAL | setSafeToPick(false) at start, setSafeToPick(true) at end

## Page 2: Pick sequence (pickTargetCallback)

### Semantics

- This page expands pickTargetCallback. Blue boxes are steps in the callback.
- A solid arrow is control flow inside the callback or a ROS message. A dashed arrow is a local call.
- The loop runs once for each group of three poses: approach, grasp, retract.
- Each motion step ends when planAndExecute() returns. A failed plan does not stop the sequence.

### Nodes

- `t_pick` | /agrobot/pick_targets | topic
  - geometry_msgs/msg/PoseArray
  - Publisher: /tomato_picker
- `busy_guard` | Busy guard | decision | With one spin thread the callback cannot run twice at the same time. The guard has no effect (inferred).
  - [is_picking_] → log, ignore message
  - Inferred: false at each entry
- `count_guard` | Pose count guard | decision
  - [count % 3 ≠ 0] → log, ignore message
- `start_pick` | Start pick | step
  - setSafeToPick(false)
  - is_picking_ = true
- `t_safe` | /agrobot/safe_to_pick | topic
  - std_msgs/msg/Bool
  - Subscriber: /tomato_picker
- `end_pick` | End pick | step
  - setSafeToPick(true)
  - is_picking_ = false
- `crouch` | Crouch | step
  - goToNamedTarget("crouch")
- `approach` | Approach | step
  - goToPoseTarget(poses[i])
- `grasp` | Grasp | step
  - goToPoseTarget(poses[i+1])
- `retract` | Retract | step
  - goToPoseTarget(poses[i+2])
- `bin` | Bin | step
  - goToNamedTarget("bin")
- `plan_execute` | planAndExecute() | local function
  - plan(), then execute() on success
  - Plan failure: log error, continue
  - execute() result: not checked
- `mgi` | MoveGroupInterface | library client
  - group arm, end effector link6
  - MoveIt connections: page 3
- `not_in_sequence` | Not in this sequence | missing capability
  - No gripper command
  - No cancel input
  - No result message to the sender
  - New messages wait in the queue (depth 10)
- `tomato_loop` | group | Per tomato loop · i = 0, 3, 6 … while i < pose count | contains: `crouch`, `approach`, `grasp`, `retract`, `bin`
  - Each step returns when planAndExecute() returns

### Edges

- `t_pick -> busy_guard` | TOPIC | receive PoseArray
- `busy_guard -> count_guard` | LOCAL | continue
- `count_guard -> start_pick` | LOCAL | continue
- `start_pick -> t_safe` | TOPIC | publish false
- `end_pick -> t_safe` | TOPIC | publish true
- `start_pick -> crouch` | LOCAL | enter loop
- `crouch -> approach` | LOCAL | next step
- `approach -> grasp` | LOCAL | next step
- `grasp -> retract` | LOCAL | next step
- `retract -> bin` | LOCAL | next step
- `bin -> crouch` | LOCAL | [more tomatoes] / i += 3
- `bin -> end_pick` | LOCAL | [no more tomatoes]
- `tomato_loop -> plan_execute` | LOCAL | each step calls
- `plan_execute -> mgi` | LOCAL | plan(), execute()

## Page 3: MoveIt connections

### Semantics

- The left boundary is the MoveGroupInterface object inside /commander. The right boundary is /move_group.
- Endpoint names are MoveIt defaults from the installed headers. They are not runtime observations.
- A solid arrow pair is one request and its response.
- Topic callbacks and pickTargetCallback share one MoveGroupInterface object.

### Nodes

- `planning` | Planning requests | client calls
  - planAndExecute() → plan(plan)
  - Named, joint, or pose targets
- `move_action` | ACTION /move_action | server interface
  - moveit_msgs/action/MoveGroup
- `cartesian` | Cartesian requests | client calls
  - goToPositionTarget(…, true) computeCartesianPath(step 0.01 m)
- `cartesian_srv` | SERVICE /compute_cartesian_path | server interface
  - moveit_msgs/srv/GetCartesianPath
- `execution` | Execution requests | client calls
  - execute(plan or trajectory)
  - After plan success or fraction = 1
- `execute_action` | ACTION /execute_trajectory | server interface
  - moveit_msgs/action/ExecuteTrajectory
- `fanout_ctx` | ACTION /arm_controller/follow_joint_trajectory | downstream interface
  - Server: /arm_trajectory_fanout
- `mgi_group` | group | /commander · shared MoveGroupInterface | contains: `planning`, `cartesian`, `execution`
- `move_group_node` | group | /move_group · MoveIt interfaces | contains: `move_action`, `cartesian_srv`, `execute_action`

### Edges

- `planning -> move_action` | ACTION | Goal
- `move_action -> planning` | ACTION | Result
- `cartesian -> cartesian_srv` | SERVICE | Request
- `cartesian_srv -> cartesian` | SERVICE | Response
- `execution -> execute_action` | ACTION | Goal
- `execute_action -> execution` | ACTION | Result: not checked
- `move_group_node -> fanout_ctx` | ACTION | trajectory execution
