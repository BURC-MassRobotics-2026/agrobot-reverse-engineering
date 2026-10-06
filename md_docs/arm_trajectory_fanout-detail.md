# /arm_trajectory_fanout node detail — Level 2, package / node detail

- source_pdf: `pdf_docs/arm_trajectory_fanout-detail.pdf`
- workspace: `nucbox_archive/nucbox_archive/agrobot_ws` · `nucbox_archive/nucbox_archive/ingenia_overlay_ws` · `nucbox_archive/nucbox_archive/AgrobotV2`
- configuration: Hardware stack: `EPOS2_ENABLE_J0=1 robot_stack/hw_moveit_stack.sh start` (EPOS2 J0 and J2–J6, Copley J1, arm_trajectory_fanout, move_group). `fanout.launch.py` loads `config/arm_trajectory_fanout.yaml` (working-tree copy): joint0–joint6 mapped, allow_unmapped_joints true.
- verification: Source-derived from the NucBox archive. No runtime verification.
- source: `nucbox_archive/nucbox_archive/ingenia_overlay_ws/src/arm_trajectory_fanout/arm_trajectory_fanout/arm_trajectory_fanout.py`
- replaces: `epos2_fjt_fanout-detail`. The start script refuses to run with epos2_fjt_fanout. /arm_trajectory_fanout is the deployed fan-out node.

## Page 1: Goal fan-out

### Semantics

- Gray boxes are the parent action and the child action servers. Blue boxes are callbacks in the node.
- A solid arrow is ROS communication. A dashed arrow is data that the callbacks in the node use together.
- _execute sends child goals one at a time, then waits for each result in joint order.
- The node copies the parent header to each child. It does not set a shared start time.

### Nodes

- `move_group` | Trajectory execution | client node
  - /move_group
  - MoveItSimpleControllerManager
- `parent_action` | ACTION /arm_controller/follow_joint_trajectory | action interface | The YAML key action_name is not a declared parameter. The default controller_action name applies.
  - control_msgs/action/FollowJointTrajectory
  - MoveIt goals list joint0–joint6
- `goal_cb` | _goal_cb | callback
  - Reject: no joint names or no points
  - Reject: no mapped joint
  - Unmapped joints: accept and ignore
- `cancel_cb` | _cancel_cb | callback
  - Forward a cancel to each child goal in the active list. Always accept.
- `execute` | _execute  ·  synchronous execute callback | callback
  - 1 For each mapped joint, in the order of the goal joint names:
    - a Wait for the child server, 5 s maximum
    - b Make a one-joint goal. Copy the header and each time_from_start
    - c Skip a no-op child: 2 or more points, fixed position, zero velocity
    - d Send the goal. Wait for acceptance, 5 s maximum
  - 2 Put the accepted child goals in the active list
  - 3 Wait for each result in joint order, T + 8 s maximum for each (T = trajectory duration)
  - 4 Make the parent result: page 2
- `c0` | joint0 | action server
  - /joint0_position_controller/…
  - /epos2_j0_bridge (node 10)
- `c1` | joint1 | action server
  - /joint1_position_controller/…
  - /copley_j1_bridge (node 22)
- `c2` | joint2 | action server
  - /j2_position_controller/…
  - /epos2_j2_bridge (node 2)
- `c36` | joint3 – joint6 | action server
  - /j3 … /j6_position_controller/…
  - /epos2_j3 … j6_bridge
- `fanout_node` | group | /arm_trajectory_fanout · action server and child clients | contains: `goal_cb`, `cancel_cb`, `execute`
  - MultiThreadedExecutor (4 threads) · one ReentrantCallbackGroup for the server and all child clients
- `children` | group | Child servers | contains: `c0`, `c1`, `c2`, `c36` | Each child is a FollowJointTrajectory action server.

### Edges

- `move_group -> parent_action` | ACTION | goal, cancel request
- `parent_action -> move_group` | ACTION | result
- `parent_action -> goal_cb` | ACTION | goal request
- `parent_action -> cancel_cb` | ACTION | cancel request
- `goal_cb -> execute` | LOCAL | accepted goal starts _execute
- `execute -> cancel_cb` | DATA | active child goal list
- `cancel_cb -> children` | ACTION | cancel_goal_async() for each active child
- `execute -> children` | ACTION | child goal
- `children -> execute` | ACTION | child result

## Page 2: Results and cancellation

### Semantics

- Left boxes are the conditions that _execute observes. Right boxes are the parent action results.
- A solid arrow goes from a condition to the result that it produces.
- The node never calls canceled(). A canceled parent goal ends as Aborted or Succeeded.
- No feedback goes to /move_group. The node does not forward child feedback.

### Nodes

- `exc` | Exception | condition
  - Child server not ready in 5 s
  - Send timeout (5 s) or goal rejected
  - Result wait longer than T + 8 s
- `child_bad` | Child failure | condition
  - A child error_code is not SUCCESSFUL
  - EPOS2 cancel and abort codes count here
- `child_ok` | All children successful | condition
  - Each accepted child: SUCCESSFUL
  - Skipped no-op children are not checked
- `parent_header` | ACTION result to /move_group on /arm_controller/follow_joint_trajectory | interface
- `aborted_exc` | Aborted | outcome
  - error_code PATH_TOLERANCE_VIOLATED error_string: the exception text
  - Child goals that run are not canceled
- `aborted_child` | Aborted | outcome
  - error_code PATH_TOLERANCE_VIOLATED
  - "Child trajectory failures: …"
- `succeeded` | Succeeded | outcome
  - error_code SUCCESSFUL
  - "Fanout succeeded for mapped joints …"
- `cancel_ctx` | Cancel request | limitation
  - _cancel_cb forwards the cancel only to child goals in the active list. The list stays empty until _execute sends all goals.
  - If each child still reports SUCCESSFUL, the parent ends as Succeeded.
- `conditions` | group | /arm_trajectory_fanout · conditions in _execute | contains: `exc`, `child_bad`, `child_ok`

### Edges

- `exc -> aborted_exc` | ACTION | abort at once
- `child_bad -> aborted_child` | ACTION | after all results
- `child_ok -> succeeded` | ACTION | after all results
