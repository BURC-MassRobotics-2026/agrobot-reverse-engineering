# /epos2_j2_bridge node detail — Level 2, package / node detail

- source_pdf: `pdf_docs/epos2_j2_bridge-detail.pdf`
- workspace: `nucbox_archive/nucbox_archive/agrobot_ws` · `nucbox_archive/nucbox_archive/ingenia_overlay_ws` · `nucbox_archive/nucbox_archive/AgrobotV2`
- configuration: Hardware stack: `EPOS2_ENABLE_J0=1 robot_stack/hw_moveit_stack.sh start` (EPOS2 J0 and J2–J6, Copley J1, arm_trajectory_fanout, move_group). `epos2_j2_generic.launch.py` starts executable epos2_joint_bridge as /epos2_j2_bridge with `config/joints/j2.yaml`: drive node 2, can0, prefix /epos2/j2, gear ratio 160.
- verification: Source-derived from the NucBox archive. No runtime verification.
- source: `nucbox_archive/nucbox_archive/agrobot_ws/src/epos2_bridge/epos2_bridge/epos2_joint_bridge.py`
- comparison: The repository copy prepares IPM, waits for header.stamp, streams records, and ends in Maxon Position Mode 0xFF. The NucBox copy uses a strict IPM lifecycle FSM, prebuffers the FIFO, and ends with IPM inactive. It has no Position Mode hold.

## Page 1: Entry points

### Semantics

- Gray boxes on the left are the ROS entry points. Gray boxes on the right are the drive and the SDO server.
- Amber marks entry points that move the drive with no configuration gate.
- A solid arrow is ROS communication or CAN. A dashed arrow is a local call.
- j2.yaml sets allow_legacy_ipm_services to false. The NucBox code does not declare or read this parameter.

### Nodes

- `e_action` | ACTION trajectory | action interface
  - /j2_position_controller/ follow_joint_trajectory
  - Client: /arm_trajectory_fanout
- `e_fault` | SERVICES and TOPICS | service and topic interfaces
  - /epos2/j2/clear_fault, arm_ipm, disarm_ipm (std_srvs/srv/Trigger) arm_ipm_now, disarm_ipm_now
  - (std_msgs/msg/Bool)
- `e_moves` | SERVICES and TOPICS | service and topic interfaces, not gated | Types: epos2_bridge_interfaces/srv/MoveDelta, MoveAbsolute, MoveAbsoluteTimed. Topics: JointState, Float64, Float64MultiArray.
  - /epos2/j2/move_delta, move_absolute, move_absolute_timed joint_target, test_move_rad, reduced_traj
- `fjt` | FJT action callbacks | callbacks
  - _goal_callback · _cancel_callback
  - _execute_follow_joint_trajectory
  - Details: page 2
- `fault_handlers` | Fault and IPM handlers | callbacks
  - clear_fault: controlword 0x80 arm_ipm: IPM active, hold records disarm_ipm: controlword 0x0F
- `move_handlers` | Direct move handlers | callbacks
  - Need ipm_armed and BridgeState IPM_ARMED or MOVING ipm_armed is true only after arm_ipm
- `access` | Drive access | local functions
  - SDO client
  - /node_2/sdo_read
  - /node_2/sdo_write call_async, wait ≤ 2 s
  - Group cb_sdo
  - CAN socket TX
  - RPDO1 0x202: PVT records
  - Raw SocketCAN on can0
  - RPDO2 0x302: not sent
- `drive` | EPOS2 drive | hardware
  - CANopen node 2
  - CAN bus can0
- `sdo_server` | SDO server | server node | `epos2_canopen_backend.launch.py` starts it with `bus_epos2_j2_j6.yml`.
  - /node_2/sdo_read, sdo_write ros2_canopen ProxyDriver
  - Master node 100
- `bridge_node` | group | /epos2_j2_bridge · handlers and drive access | contains: `fjt`, `fault_handlers`, `move_handlers`, `access`
  - MultiThreadedExecutor (4 threads) · action: cb_action (reentrant) · SDO clients: cb_sdo · others: default group

### Edges

- `e_action -> fjt` | ACTION | goal, cancel
- `e_fault -> fault_handlers` | SERVICE, TOPIC | call or message
- `e_moves -> move_handlers` | SERVICE, TOPIC | call or message
- `fjt -> access` | LOCAL | SDO writes and reads, PVT records
- `fault_handlers -> access` | LOCAL | SDO writes, hold records
- `move_handlers -> access` | LOCAL | SDO, PVT records
- `access -> drive` | CAN | RPDO1 frames
- `access -> sdo_server` | SERVICE | canopen_interfaces/srv/CORead, COWrite request
- `sdo_server -> access` | SERVICE | response
- `sdo_server -> drive` | CAN | SDO transfer

## Page 2: Trajectory execution

### Semantics

- This page expands the FollowJointTrajectory callbacks. T is the planned trajectory duration.
- A solid arrow is ROS communication. A dashed arrow is a local call or shared data.
- The callback checks the cancel flag only in step 7. Steps 1 to 6 do not check it.
- The callback does not use header.stamp. Each joint starts when its own activation succeeds.
- Blocking waits in the execute callback hold one of the 4 executor threads for the whole goal.

### Nodes

- `action` | ACTION /j2_position_controller/follow_joint_trajectory | action interface
  - control_msgs/action/FollowJointTrajectory · client /arm_trajectory_fanout
- `goal_cb` | _goal_callback | callback
  - Accept only [joint2] with at least one point
- `execute` | _execute_follow_joint_trajectory | callback
  - 1 Entry: IPM FSM READY or FAULTED, IPM inactive
  - 2 clear_fault (0x80), mode 7, enable (0x06, 0x0F)
  - 3 Clear and enable the FIFO. IPM stays inactive
  - 4 Make PVT: 40 ms start record, spline segments, 10 hold records of 100 ms at the target
  - 5 Prebuffer ≤ 62 records, else 48 plus low-water feed.
  - 2 ms gap, check FIFO level, 3 tries maximum
  - 6 controlword 0x1F (IPM active), 4 tries maximum
  - 7 Wait ≥ T until |error| ≤ 0.01 rad and |v| ≤ 0.10 rad/s.
  - Timeout T + 5 s (5 s minimum). Feedback each 20 ms
  - 8 disarm_ipm: controlword 0x0F. IPM FSM READY
- `cancel_cb` | _cancel_callback | callback
  - Always accepts the cancel request
- `cache` | Drive state cache | data
  - DriveState under state_lock
  - Written by CAN RX (page 3)
  - Fault, position, velocity
- `access2` | Drive access | local functions
  - SDO: steps 1, 2, 3, 5, 6, 8 (≤ 2 s each)
  - RPDO1 0x202: PVT records (steps 5, 7)
- `terminal` | Terminal status | result | Result codes: Level 3 motion-states page 2.
  - Succeeded after step 8
  - Canceled after cleanup (step 7)
  - Aborted: setup, wait, or exception
- `bridge_exec` | group | /epos2_j2_bridge · action server and execution | contains: `goal_cb`, `execute`, `cancel_cb`, `cache`, `access2`, `terminal`

### Edges

- `action -> goal_cb` | ACTION | goal request
- `action -> cancel_cb` | ACTION | cancel request
- `goal_cb -> execute` | LOCAL | accepted goal starts execution
- `cancel_cb -> execute` | DATA | is_cancel_requested read in step 7
- `execute -> action` | ACTION | feedback and result
- `cache -> execute` | DATA | read position, velocity, fault
- `execute -> access2` | LOCAL | SDO and PVT
- `execute -> terminal` | LOCAL | return status

## Page 3: State and telemetry

### Semantics

- This page shows how drive state enters the node and leaves as topics.
- A solid arrow is CAN, a ROS service, or a ROS topic. A dashed arrow is a local call or shared data.
- CAN RX runs in its own thread. The three timers share the default mutually exclusive group.
- The telemetry timer holds state_lock during its SDO reads. Other readers of the cache wait for it.
- Rates are code defaults. j2.yaml does not set them.

### Nodes

- `drive` | EPOS2 drive | hardware
  - CANopen node 2
  - CAN bus can0
- `sdo_server` | SDO server | server node
  - /node_2/sdo_read, sdo_write ros2_canopen ProxyDriver
- `rx` | CAN RX thread | thread
  - _can_rx_loop, own thread
  - Heartbeat 0x702 · EMCY 0x082
  - TPDO1 0x182: buffer, status, mode
  - TPDO2 0x282: position, velocity
- `cache` | Drive state cache | data
  - DriveState with TPDO times
  - Guarded by state_lock
- `telemetry` | Telemetry timer | timer
  - _poll_slow_state_and_publish
  - Every 0.1 s (10 Hz)
  - SDO fallback if TPDO > 200 ms old
  - 9 more SDO reads on each tick
  - Holds state_lock while it reads
- `joint_timer` | Joint-state timer | timer
  - _publish_joint_state
  - Every 20 ms (50 Hz)
  - Position, velocity, fault
- `sdo_client` | SDO client | client
  - sdo_read(): call_async
  - Caller waits ≤ 2 s per read
  - Group cb_sdo
- `startup` | Startup and keepalive | timers
  - Startup (0.5 s): runs once after SDO is ready. j2.yaml startup flags are false: no writes
  - Keepalive (50 ms): hold record only if ipm_armed and BridgeState IPM_ARMED
- `topics_js` | TOPICS | topics
  - /joint_states sensor_msgs/msg/JointState
  - /epos2/j2/fault std_msgs/msg/Bool
- `topics_tel` | TOPICS | topics | The diagnostics name is a fixed string in the code. Each joint bridge uses "epos2/j2".
  - /epos2/j2/state_raw
  - /epos2/j2/state_engineering
  - /epos2/j2/state_summary
  - /diagnostics
  - Status name: "epos2/j2"
- `bridge_state` | group | /epos2_j2_bridge · state, timers, and telemetry | contains: `rx`, `cache`, `telemetry`, `joint_timer`, `sdo_client`, `startup`

### Edges

- `drive -> rx` | CAN | TPDO1, TPDO2, heartbeat, EMCY frames
- `rx -> cache` | DATA | write
- `telemetry -> cache` | DATA | write SDO values
- `cache -> joint_timer` | DATA | read
- `joint_timer -> topics_js` | TOPIC | publish at 50 Hz
- `telemetry -> topics_tel` | TOPIC | publish at 10 Hz
- `telemetry -> sdo_client` | LOCAL | sdo_read()
- `startup -> sdo_client` | LOCAL | service_is_ready()
- `sdo_client -> sdo_server` | SERVICE | canopen_interfaces/srv/CORead
- `sdo_server -> drive` | CAN | SDO transfer
