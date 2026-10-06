# /tomato_picker node detail — Level 2, package / node detail

- source_pdf: `pdf_docs/tomato_picker-detail.pdf`
- workspace: `nucbox_archive/nucbox_archive/agrobot_ws` · `nucbox_archive/nucbox_archive/ingenia_overlay_ws` · `nucbox_archive/nucbox_archive/AgrobotV2`
- configuration: `robot_commander/launch/commander.launch.py`: node /tomato_picker, no parameters. This launch has no camera transform.
- verification: Source-derived from the NucBox archive. No runtime verification.
- source: `nucbox_archive/nucbox_archive/agrobot_ws/src/robot_commander/src/tomato_picker.py`
- comparison: The repository copy uses MAX_REACH = 1.5 m and its launch adds a static camera transform. The NucBox copy uses 1.1 m and has no camera transform.

## Semantics

- Gray boxes are topics outside the node. Blue boxes inside the boundary are callbacks and stored data.
- A solid arrow is a ROS message. A dashed arrow is a read or write of stored data.
- rclpy.spin uses one thread. Flag and transform updates run between batches, not during one batch.
- No source in the archive publishes a transform from linear_rail_link to camera_color_optical_frame. Without it, step 4 fails for each tomato and the node publishes nothing.

## Nodes

- `t_spatial` | /agrobot/tomato_spatial | topic
  - std_msgs/msg/String (JSON)
  - Publisher: /agrobot/tomato_spatial tomato_id: index in one frame
- `t_safe` | /agrobot/safe_to_pick | topic
  - std_msgs/msg/Bool
  - Publishers: /tomato_detector and /commander
- `t_tf` | /tf, /tf_static | topic, unresolved
  - tf2_msgs/msg/TFMessage
  - Rail to camera transform: no publisher in archive
- `spatial_cb` | spatial_callback  ·  once per message | callback
  - 1 Skip the batch if safe_to_pick is false
  - 2 Parse JSON. Skip the batch on error
  - 3 Keep confidence ≥ 0.6 and IDs not in picked_ids
  - 4 Transform to linear_rail_link (timeout 0.1 s)
  - 5 Drop on transform failure or distance > 1.1 m
  - 6 Sort by y
  - 7 Make approach, grasp, and retract poses
  - 8 Add IDs to picked_ids. Publish one PoseArray
- `safety_cb` | safety_callback | callback
  - Store the flag
  - No timeout, no reset
- `tf_listener` | TransformListener | callback
  - Fills tf_buffer
  - No dedicated spin thread
- `stored` | Stored state | data
  - safe_to_pick default true picked_ids never cleared tf_buffer received transforms
- `t_targets` | /agrobot/pick_targets | topic
  - geometry_msgs/msg/PoseArray
  - Frame linear_rail_link
  - 3 poses per tomato
- `commander` | /commander | subscriber node
  - pickTargetCallback
  - Subscriber, page 2 of commander-detail
- `id_reuse` | Reused tomato_id | limitation | tomato_id comes from the per-frame list index in the spatial node.
  - picked_ids keeps frame indexes.
  - The node skips a later tomato with a used index.
- `picker_node` | group | /tomato_picker · subscriptions, stored state, publisher | contains: `spatial_cb`, `safety_cb`, `tf_listener`, `stored`
  - rclpy.spin: one thread, one callback at a time

## Edges

- `t_spatial -> spatial_cb` | TOPIC | /agrobot/tomato_spatial
- `t_safe -> safety_cb` | TOPIC | /agrobot/safe_to_pick
- `t_tf -> tf_listener` | TOPIC | /tf, /tf_static
- `safety_cb -> stored` | DATA | write safe_to_pick
- `tf_listener -> stored` | DATA | write tf_buffer
- `spatial_cb -> stored` | DATA | read flag and tf_buffer, add IDs
- `spatial_cb -> t_targets` | TOPIC | publish /agrobot/pick_targets | Once per batch with at least one tomato.
- `t_targets -> commander` | TOPIC | subscription
- `stored -- id_reuse` | context | picked_ids limitation
