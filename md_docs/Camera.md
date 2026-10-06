# Camera-to-tomato positions in the archived Agrobot pipeline

Reviewed: 2026-10-03. Method: source inspection, reference comparison, and
existing preprocessing tests. This review ran no camera, model inference,
simulation, or robot motion.

## Findings

The code has **two different ways to produce 3D positions**:

1. **Point-cloud path:** the spatial node combines detection boxes with a
   RealSense point cloud, fits a sphere, and sends its center through tracking
   to Qwen's selected target. This is the path connected to target selection.
2. **Optional depth-image path:** the detector samples depth at each box center
   and publishes `/agrobot/detections_3d`. It is disabled by default and is not
   the input to the supplied tracker or Qwen node.

The code traces a path to a camera-frame target, but **alignment, capture-time
matching, and a calibrated robot-frame position are not fully established**.
The reference launch example pairs color detections with raw depth in the
optional path. That path also lacks depth-unit conversion. The point-cloud
path assumes compatible frames and combines detections with the latest cached
camera data instead of matching capture timestamps.
Sources: [spatial processing, lines 249–399][spatial-process],
[optional depth lookup, lines 546–607][depth-path], and
[reference launch example, lines 57–64][reference-detector].

This work primarily serves **Find produce**, with camera input supporting
**Identify crops** and **Check ripeness** in the FAST diagram
(`agrobot-functions.md`, not in this repo). [Model.md][models] documents the
model roles. This document discusses tracking and motion
only where their interfaces affect camera coordinates or observation age.

## Sources and camera setup

The primary source is the supplied `computer_vision.7z`, extracted under
`/tmp/agrobot-computer-vision-review/computer_vision/`. The review rechecked
the eight Python files for this camera trace against the existing workspace
copies. They match byte for byte. Source links below therefore use the workspace
copies. [Model.md][models] records the archive identity and the complete perception
comparison. In this repo, links point to the archive copy under
`nucbox_archive/nucbox_archive/AgrobotV2/`.

| Item | Evidence | What is established |
| --- | --- | --- |
| Camera model | Archive documentation names RealSense **D456** | Reported hardware. No one checked a device serial number or queried the live model. |
| Camera interface | Agrobot subscribes to ROS messages. Dockerfiles install `ros-jazzy-realsense2-camera` | The application uses a separately launched ROS camera driver |
| SDK snapshot | Bundled `librealsense/include/librealsense2/rs.h` declares **2.56.1** | Version of the bundled source, not proof of the runtime library version |
| Color profile | Reference command requests `640x480x30` | Requested 640 × 480 pixels at 30 frames/s. The actual negotiated rate is unmeasured. |
| Depth profile | Reference command leaves it unspecified | The document reports an 848 × 480 setup and trouble forcing 640 × 480. This is not a verified list of supported D456 modes |
| Alignment and point cloud | Reference command sets `align_depth.enable:=true` and `pointcloud.enable:=true` | Intended driver configuration. The Agrobot launch files do not themselves launch or configure the camera. |
| Capture synchronization | Reference command does not explicitly set `enable_sync` | Effective driver synchronization and capture-time offset remain unknown |

Sources: [reference camera command, lines 9–42][reference-camera],
[Docker camera dependency, lines 29–43][docker-camera],
[SDK version, lines 26–28][sdk-version], and
[perception launch, lines 293–321][launch-composition]. The ROS wrapper's exact
installed version and parameters were not recovered from a running system.

The [RealSense ROS documentation][realsense-ros] describes depth alignment and
frame synchronization as separate settings. Alignment publishes depth on
`/camera/camera/aligned_depth_to_color/image_raw`. Its documentation says the
point cloud uses aligned depth when that filter is enabled. These are upstream
interface descriptions, not observations of this robot's deployment.

## Data flow and topics

```text
RealSense color and depth sensors
  → librealsense + separately launched realsense2_camera ROS driver
      ├─ color Image → detector → /agrobot/detections ──┐
      ├─ PointCloud2 ──────────────────────────────────┤
      ├─ color CameraInfo ─────────────────────────────┤
      └─ color Image for crops ────────────────────────┤
                                                       ▼
                              tomato_spatial: clip points and fit spheres
                                → /agrobot/tomato_spatial, JSON positions
                                → tomato_tracker
                                → /agrobot/tomato_tracks, JSON tracks
                                → qwen_vl
                                → /agrobot/pick_target, camera-frame pose

Separate optional branch inside the detector:
  detection box + cached depth Image + depth CameraInfo
    → one depth sample at the box center
    → /agrobot/detections_3d
```

| Topic | Message type | Producer → consumer / meaning |
| --- | --- | --- |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | Camera driver → detector and spatial node. Native color image. |
| `/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | Driver → spatial node. Color calibration and dimensions. |
| `/camera/camera/depth/color/points` | `sensor_msgs/PointCloud2` | Driver → spatial node. XYZ coordinates, expected in meters. |
| `/agrobot/detections` | `vision_msgs/Detection2DArray` | Detector → spatial node. Boxes, labels, confidence, original color header. |
| `/agrobot/tomato_spatial` | `std_msgs/String` containing JSON | Spatial node → tracker. Sphere centers, dimensions, scores, and JPEG crops. |
| `/agrobot/tomato_tracks` | `std_msgs/String` containing JSON | Tracker → Qwen. Smoothed centers and persistent IDs. |
| `/agrobot/pick_target` | `geometry_msgs/PoseStamped` | Qwen output. Selected center labeled `camera_color_optical_frame`. |
| `/agrobot/detections_3d` | `vision_msgs/Detection3DArray` | Optional detector output. The supplied spatial/tracker/Qwen chain does not consume it. |

Sources: [launch topic wiring, lines 156–224][launch-topics],
[detector subscriptions/publications, lines 313–346][detector-topics],
[tracker serialization, lines 174–183][tracker-output], and
[Qwen publication, lines 654–689][qwen-output]. Running the detector alone uses
`/camera/image_raw`. The launch file remaps it to the RealSense color topic.
Both launch files leave the optional depth topics empty by default, while
including the spatial node with its point-cloud topic configured.
Sources: [standard defaults, lines 28–50][launch-depth] and
[GPU defaults, lines 29–51][gpu-depth].

## Color pixels become detection boxes

The detector converts the incoming color message to BGR with `cv_bridge`,
then converts to RGB, resizes with preserved aspect ratio, adds black padding,
and normalizes for the models. The model canvas is **518 × 518 pixels**.
Published box coordinates remain in that canvas even though the copied message
header comes from the original camera image. The detector uses masks internally,
but `/agrobot/detections` does not include them.
Sources: [preprocessing, lines 87–109][preprocess],
[image callback and box publication, lines 448–522][detector-publish].

For a native image of width `W_px` and height `H_px`, the code uses:

```text
scale = min(518 / W_px, 518 / H_px)
resized_width_px  = int(W_px × scale)
resized_height_px = int(H_px × scale)
pad_x_px = (518 − resized_width_px) // 2
pad_y_px = (518 − resized_height_px) // 2

u_model_px = u_color_px × scale + pad_x_px
v_model_px = v_color_px × scale + pad_y_px

u_color_px = (u_model_px − pad_x_px) / scale
v_color_px = (v_model_px − pad_y_px) / scale
```

For the documented 640 × 480 color profile, this gives scale **0.809375**,
resized dimensions **518 × 388**, and padding **0 pixels horizontally and
65 pixels vertically on each side**. The review checked these values arithmetically.
They are not a measurement of the connected camera.
Sources: [resize calculation, lines 53–65][resize] and
[inverse crop calculation, lines 281–295][crop].

The point-cloud and crop helpers hardcode 518. Changing only the detector's
input-size parameters would therefore make these stages disagree.
Sources: [cloud projection, lines 64–80][cloud-clip] and [crop helper][crop].

## Point-cloud path: estimate a tomato center

### From depth measurements to XYZ points

A depth image supplies distance along the camera's optical Z axis for each
valid pixel. With matching calibration and a pinhole model, deprojection is:

```text
Z_m = depth_m
X_m = (u_px − cx_px) × Z_m / fx_px
Y_m = (v_px − cy_px) × Z_m / fy_px
```

`fx_px` and `fy_px` are focal lengths in pixels. `cx_px` and `cy_px` locate the
principal point, where the optical axis meets the image. The SDK handles the
depth-to-point-cloud conversion upstream of Agrobot. Its bundled implementation
multiplies stored depth values by `depth_frame.get_units()` before deprojecting.
This unit scaling is present in the SDK cloud path and missing from Agrobot's
optional image path discussed below.
Sources: [SDK deprojection, lines 28–47][sdk-cloud] and
[SDK depth-unit definition, lines 178–181][sdk-units].

Depth-to-color alignment uses calibration between the two sensors. The bundled
SDK deprojects a depth pixel, transforms the 3D point into the other camera's
frame, and projects it onto that camera's image. This is more than resizing the
depth image. Source: [SDK alignment, lines 36–106][sdk-align].

### From XYZ points to one sphere per box

On each detection message, `tomato_spatial` performs the following operations:

1. Read the latest cached point cloud and the first cached color `CameraInfo`.
   If either is missing, return without a spatial publication.
2. Extract XYZ fields, skip NaNs, and retain points with **0.05 < Z < 5.0 m**.
   These are code filters, not verified camera limits or arm reach limits.
3. Project each point with `u = fx × X/Z + cx`, `v = fy × Y/Z + cy`, then map
   those pixels into the 518 × 518 canvas. Keep points inside the detection
   rectangle. **This uses the box, not the SAM2 mask.**
4. Require at least **15 points before depth filtering**. Retain points no more
   than **0.10 m behind the nearest point**. If this leaves fewer than four,
   the helper returns the original cluster.
5. Fit a sphere with RANSAC: sample candidate fits and find points close to
   their surfaces. Defaults are **60 iterations**, **0.015 m** surface-distance
   tolerance, and **10 inliers** for the helper's consensus criterion.
6. Accept a final radius of **0.015–0.12 m**. Publish the fitted sphere center as
   `centroid`, plus radius, XYZ extents, detector confidence, and a JPEG crop.

Sources: [spatial processing, lines 249–399][spatial-process],
[projection, lines 34–92][cloud-clip],
[nearest-depth filter, lines 97–124][depth-filter], and
[sphere fitting, lines 164–239][sphere-fit]. The radius check above follows the
executable condition. Its nearby log text incorrectly lists an upper limit
of 0.075 m.

The fitted center estimates the center of the fruit, while the visible depth
points lie on its surface. They are different quantities. `width`, `height`,
and `depth` are the axis-aligned extents of the returned point set, not a
guarantee of the full fruit dimensions. The spatial node copies the output score from
the detector. There is no separate localization uncertainty or fit-quality field.
Sources: [fit output][sphere-fit], [extent calculation, lines 244–256][extents],
and [spatial JSON, lines 361–399][spatial-output].

The fit has limitations worth preserving in the record. The nearest-depth
filter assumes the tomato is the nearest surface in the rectangle, so a
foreground leaf can invalidate that assumption. If RANSAC lacks consensus,
the helper falls back to a fit using all points and clamps its radius to a
permitted range. Passing the final radius check therefore does not prove a
good spherical fit. These are source-derived limitations, not measured errors.
Sources: [depth filter][depth-filter] and [fallback, lines 227–239][sphere-fit].

## Alignment assessment

| Relationship | Implemented behavior | Assessment |
| --- | --- | --- |
| Native color pixels ↔ model pixels | Cloud clipping and cropping reuse the resize and padding formulas | Mathematically consistent when native dimensions match and input size remains 518 |
| Depth sensor ↔ color sensor | Reference setup enables the driver's alignment filter | Intended, but actual driver parameters and resulting messages were not checked |
| Cloud coordinates ↔ color calibration | Spatial node projects incoming XYZ directly with color intrinsics | Assumes the cloud is already in the color optical frame. It does not check `frame_id` or transform points. |
| Calibration ↔ current color stream | Spatial node caches only the first `CameraInfo` | Does not update for a changed profile/calibration or check cached dimensions against color images |
| Distorted image ↔ pinhole projection | Helpers use `K` values only | The helpers ignore the distortion, rectification, ROI, and binning fields. The approximation needs a check for the active stream. |
| Camera frame ↔ robot frame | Archived perception publishes camera-frame positions | Requires a separately supplied, calibrated transform before motion can use them |

Sources: [cached inputs, lines 215–241][spatial-cache],
[projection helper][cloud-clip], [crop helper][crop], and
[Qwen frame label][qwen-output]. ROS `CameraInfo` distinguishes raw-image
intrinsics `K`, distortion `D`, and rectified projection `P`. A nonzero focal
length and appropriate calibration cannot be assumed merely because a message
arrived. The node checks the array length but does not check those numeric
properties. See the [CameraInfo definition][camera-info].

Color texture on a cloud is not itself proof that its XYZ coordinates are in
the color frame: the SDK separately calculates vertices and texture
coordinates. The documented aligned-depth setup is intended to make the
geometry compatible. Check the actual cloud header and calibration rather
than inferring the frame from `/depth/color/points` in the topic name.
Sources: [SDK cloud/texture processing, lines 248–275][sdk-texture] and
[RealSense alignment behavior][realsense-ros].

There is also a visualization mismatch: the detector debug publisher draws
518-space boxes directly on the original-resolution BGR image, without
undoing padding and scaling. Its overlay alone cannot establish correct
alignment on a 640 × 480 stream. Sources:
[debug publisher, lines 527–544][debug-publish] and
[overlay drawing, lines 134–144][overlay].

## Optional depth-image path: a different output with two gaps

When both depth topic parameters are provided, the detector stores the most
recent depth image with `desired_encoding="passthrough"` and caches its `K`
values. For each box it reverses letterboxing, rounds the native color center
to one pixel, samples `depth[v, u]`, rejects nonpositive/nonfinite samples, and
uses the deprojection formulas above. The detector stores the position in
`Detection3D.bbox.center.position`, with the original color header.
Source: [depth callback and publication, lines 546–607][depth-path].

**The code assumes spatial correspondence.** The reference command supplies
`/camera/camera/depth/image_rect_raw` and `/camera/camera/depth/camera_info`.
Those are the raw depth stream's topics, whereas aligned depth has separate
`aligned_depth_to_color` topics. Bounds checking does not make a raw depth
pixel correspond to the color box center. Correct correspondence would require
aligned depth with matching calibration, or explicit cross-camera projection.
This function performs neither alignment nor a sensor-to-sensor transform.
Sources: [reference parameters][reference-detector], [depth function][depth-path],
and [driver topic definitions][realsense-ros].

**Units are not converted or checked.** ROS depth conventions allow float
depth in meters and unsigned 16-bit depth in millimeters. Casting a stored
value to `float` does not convert its unit. If a received sample is 500 mm,
this function writes `z = 500`, not `z = 0.5`. Interpreting its position as
meters makes every coordinate 1,000 times too large. This is conditional on
the actual encoding, which was not observed. See [REP 118][depth-units] and
[sampling, lines 589–599][depth-path].

This branch also samples a single surface pixel instead of fitting a fruit
center, has no capture-time matching, and does not publish an empty 3D array
when every sample is rejected. Its output should not be treated as equivalent
to the sphere-center path. Source: [depth publication][depth-path].

## Timing and freshness assessment

**Geometric alignment does not make two observations simultaneous.** The
application has no capture-timestamp join between detection, cloud, and crop.
Its callbacks cache values and combine whichever are present when processing
occurs. Source: [spatial caches and detection callback, lines 215–274][spatial-cache].

| Stage | Time information actually used | Consequence |
| --- | --- | --- |
| Color → detections | Detection header copies the input color header | Capture stamp survives model inference, assuming the driver supplied it correctly |
| Cloud cache | Saves the cloud and local `time.monotonic()` callback time | Age measures time since callback receipt, not time since camera exposure |
| Detections + cloud | Uses latest cached cloud. Does not compare headers. | An old box may be applied to a newer scene |
| Cloud age check | Warns above **2.0 s** by default, then continues | This is not a stale-data rejection rule |
| JPEG crop | Uses latest cached BGR image. Retains no color timestamp. | Crop may come from a different exposure than the detection or cloud |
| Optional depth lookup | Uses detector's cached depth without timestamp comparison | Depth may predate the color frame. Its callback cannot update during blocking inference in that process |
| Spatial JSON → tracking JSON | Neither schema includes source timestamp or frame ID | Downstream consumers cannot recover observation time/frame from these records |
| Tracker | Smooths observations. The tracker counts age and expiry in received spatial batches. | `age >= 3` is not a bound on elapsed time or measurement freshness |
| Qwen pick target | Sets timestamp to `now()` and hardcodes the color optical frame | A recently published pose can contain an old observation |

Sources: [detector publication][detector-publish], [spatial caches][spatial-cache],
[spatial JSON][spatial-output], [tracker update/output, lines 149–183][tracker-update],
[track expiry, lines 370–395][tracker-expiry], and [Qwen publication][qwen-output].

An illustrative sequence, not a recorded run:

```text
t = 0 s:   Color image A enters slow model inference
t = 20 s:  A's detection arrives at the separate spatial node
           The spatial node has recently received cloud B and color image B
           It applies A's box to B's cloud and uses B for the JPEG crop
later:     Tracking/Qwen publish the selected position with a new timestamp
```

A recent cloud passes the receipt-age check even though it does not match the
box's exposure. A stationary camera and scene can conceal the discrepancy.
Moving the camera, fruit, or foreground objects makes it consequential. The
20-second interval above is only an example, not a measured latency.

The image/cloud subscriptions use best-effort delivery and a queue depth of
one. This limits queued sensor messages but does not synchronize them.
`CameraInfo`, detection, track, and Qwen subscriptions use separate queues.
Sources: [detector QoS][detector-qos], [spatial QoS/subscriptions][spatial-qos],
and [Qwen track subscription][qwen-subscription].

The detector runs inference inside its image callback and uses plain
`rclpy.spin(node)`. ROS Jazzy's default executor is single-threaded, so the
detector's depth and watchdog callbacks wait while inference is executing.
The watchdog defaults to **60,000 ms**, measures elapsed time from image
callback entry, and publishes empty 2D detections plus `safe_to_pick=False`.
It does not provide an independent deadline for stalled inference and does
nothing before the first image callback. Sources:
[detector defaults, lines 140–147][watchdog-default],
[timer and watchdog, lines 356–401][watchdog],
[detector main, lines 610–619][detector-main], and
[ROS Jazzy executor implementation][rclpy-spin].

Missing clouds, missing calibration, parse failures, or an empty cloud can
make the spatial node return without publishing. Tracker expiry runs only
when another spatial batch arrives, not on a wall-clock timer. Silence is
therefore not an explicit invalidation of the last target.
Sources: [spatial early returns][spatial-process] and [tracker expiry][tracker-expiry].

## From camera coordinates to positions the robot can use

The intended optical frame uses **X right, Y down, Z forward**, with distances
in meters. These differ from the usual ROS body axes. See [REP 103][frame-conventions].
Changing a frame label does not transform the point. The transform needs a calibrated rotation
and translation:

```text
point_robot_m = R_robot_from_camera × point_camera_m + translation_robot_m
```

If the camera moves relative to the robot base, the transform must correspond
to the observation time. Neither a sphere center nor Qwen's identity orientation
by itself supplies a grasp approach. The archive's perception nodes contain no
camera-to-robot TF lookup. Qwen simply copies the tracked center into a pose.
Source: [Qwen position publication][qwen-output].

The tracker has translation-only camera-motion compensation via
`/agrobot/camera_motion` (`geometry_msgs/Vector3`). It subtracts accumulated
displacement from existing tracks before matching. It carries no timestamp,
does not compensate rotation, and is not a robot-base transformation.
Sources: [motion prediction, lines 133–147][tracker-motion] and
[motion accumulation/application, lines 278–322][tracker-motion-input].

The club repository has a separate [tomato picker helper][club-picker] at
commit `1cb09e05e78b5562b56ffcd68a8dbd311a926e0f`. Its executable code subscribes
to `/agrobot/tomato_spatial`, constructs a point labeled
`camera_color_optical_frame`, and asks TF2 to transform it to
`linear_rail_link` with a 0.1-second timeout. It stamps the point with the
current time, then publishes a `PoseArray` on `/agrobot/pick_targets`.
This shows an intended transformation boundary. It does not prove the
calibration or live TF chain exists. It also does not subscribe to Qwen's
singular `/agrobot/pick_target`. Sources:
[helper subscriptions/publication, lines 69–85][club-picker-topics] and
[transform function, lines 179–202][club-picker-transform].

## Reference gaps and evidence still needed

The archived reproduction guide launches `perception.launch.py` and then
instructs the operator to launch spatial, tracker, and Qwen separately. The
current launch already includes all four perception nodes, so following that
sequence literally would launch duplicate consumers/publishers. This review
did not launch them. Sources: [reference guide][reference-camera] and
[launch composition][launch-composition].

The file named `HIL_RESULTS.md` contains an unfilled cycle log and an
“arm planner TBD” entry. It is a protocol/template, not evidence of measured
alignment, timing, or completed vision-guided picks.
Source: [archived HIL document, lines 12–79][hil].

| Unverified item | Evidence needed to settle it |
| --- | --- |
| Active camera and stream profiles | Actual model/serial, driver version, parameters, image dimensions, encoding, and observed rates |
| Color/depth geometric alignment | Depth and cloud headers, matching calibration, and known target correspondences across the image |
| Correct 3D units | Received depth encoding/scale and measured-distance comparison against reported XYZ |
| Capture-time agreement | Recorded color, depth, cloud, and detection stamps from the same run, including motion |
| Full target age | Capture-to-detection, spatial, tracking, Qwen, and publication timing from one trace |
| Robot-frame coordinates | Camera mounting/calibration and a valid TF chain at observation time, checked against a known robot-frame target |
| Sphere-center accuracy | Comparison with known fruit geometry, including occlusion and foreground clutter |

These remain open findings for the camera assignment. They were not inferred
from file names, example logs, or the availability of model weights.

## Checks performed

- Rechecked the eight camera-related Python source/launch files against the
  extracted archive: all matched.
- Ran the existing preprocessing suite: **12 passed**. It covers image channel
  order, padding/resizing, normalization, and tensor shape. It does not prove
  depth alignment, sphere accuracy, timestamp matching, or robot calibration.
  Executed from `/home/t1sun/agrobot`:

  ```bash
  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/home/t1sun/agrobot/src/agrobot_perception python3 -m pytest src/agrobot_perception/tests/test_image_utils.py -q -p no:cacheprovider
  ```

- Checked the documented 640 × 480 → 518 × 518 padding arithmetic. The review
  wrote no new tests. No point-cloud test file exists in the inspected
  perception test directory.
- This review claims no hardware or model inference results. The review
  inspected the quoted camera setup and launch instructions but did not run them.

[models]: Model.md
[spatial-process]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/tomato_spatial_node.py#L249
[spatial-cache]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/tomato_spatial_node.py#L215
[spatial-output]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/tomato_spatial_node.py#L361
[spatial-qos]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/tomato_spatial_node.py#L131
[depth-path]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/tomato_detector_node.py#L546
[detector-topics]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/tomato_detector_node.py#L313
[detector-publish]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/tomato_detector_node.py#L448
[detector-qos]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/tomato_detector_node.py#L104
[debug-publish]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/tomato_detector_node.py#L527
[watchdog-default]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/tomato_detector_node.py#L140
[watchdog]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/tomato_detector_node.py#L356
[detector-main]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/tomato_detector_node.py#L610
[preprocess]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/utils/image_utils.py#L87
[resize]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/utils/image_utils.py#L53
[overlay]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/utils/image_utils.py#L134
[cloud-clip]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/utils/pointcloud_utils.py#L34
[depth-filter]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/utils/pointcloud_utils.py#L97
[sphere-fit]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/utils/pointcloud_utils.py#L164
[extents]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/utils/pointcloud_utils.py#L244
[crop]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/utils/pointcloud_utils.py#L261
[tracker-output]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/tomato_tracker_node.py#L174
[tracker-update]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/tomato_tracker_node.py#L149
[tracker-expiry]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/tomato_tracker_node.py#L370
[tracker-motion]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/tomato_tracker_node.py#L133
[tracker-motion-input]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/tomato_tracker_node.py#L278
[qwen-output]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/qwen_vl_node.py#L654
[qwen-subscription]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/agrobot_perception/qwen_vl_node.py#L309
[launch-depth]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/launch/perception.launch.py#L28
[gpu-depth]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/launch/perception_gpu.launch.py#L29
[launch-topics]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/launch/perception.launch.py#L156
[launch-composition]: ../nucbox_archive/nucbox_archive/AgrobotV2/perception/launch/perception.launch.py#L293
[reference-camera]: ../nucbox_archive/nucbox_archive/AgrobotV2/REPRODUCE.md#L9
[reference-detector]: ../nucbox_archive/nucbox_archive/AgrobotV2/REPRODUCE.md#L57
[docker-camera]: ../nucbox_archive/nucbox_archive/AgrobotV2/deployment/docker/Dockerfile.rocm-gpu#L29
[hil]: ../nucbox_archive/nucbox_archive/AgrobotV2/docs/HIL_RESULTS.md#L12
[sdk-version]: ../nucbox_archive/nucbox_archive/librealsense/include/librealsense2/rs.h#L26
[sdk-cloud]: ../nucbox_archive/nucbox_archive/librealsense/src/proc/pointcloud.cpp#L28
[sdk-units]: ../nucbox_archive/nucbox_archive/librealsense/include/librealsense2/h/rs_frame.h#L178
[sdk-align]: ../nucbox_archive/nucbox_archive/librealsense/src/proc/align.cpp#L36
[sdk-texture]: ../nucbox_archive/nucbox_archive/librealsense/src/proc/pointcloud.cpp#L248
[realsense-ros]: https://github.com/realsenseai/realsense-ros/blob/ros2-master/README.md#post-processing-filters
[camera-info]: https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/CameraInfo.msg
[depth-units]: https://github.com/ros-infrastructure/rep/blob/master/rep-0118.rst
[frame-conventions]: https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst
[rclpy-spin]: https://github.com/ros2/rclpy/blob/jazzy/rclpy/rclpy/__init__.py
[club-picker]: https://github.com/BURC-MassRobotics-2026/agrobot-reverse-engineering/blob/1cb09e05e78b5562b56ffcd68a8dbd311a926e0f/Agrobot/src/robot_commander/src/tomato_picker.py
[club-picker-topics]: https://github.com/BURC-MassRobotics-2026/agrobot-reverse-engineering/blob/1cb09e05e78b5562b56ffcd68a8dbd311a926e0f/Agrobot/src/robot_commander/src/tomato_picker.py#L69-L85
[club-picker-transform]: https://github.com/BURC-MassRobotics-2026/agrobot-reverse-engineering/blob/1cb09e05e78b5562b56ffcd68a8dbd311a926e0f/Agrobot/src/robot_commander/src/tomato_picker.py#L179-L202
