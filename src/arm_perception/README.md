# arm_perception

Wrist-camera RGB-D color box detector. Deprojects detections to world poses and
serves them on demand.

## Working
- Synchronizes RGB, depth, and camera_info from the wrist camera.
- HSV segmentation per configured color, contour filtering, top-face depth
  filtering, centroid + yaw from minAreaRect.
- Deprojects pixel + depth in the camera optical frame with the pinhole model
  and transforms to `world` via TF (image timestamp, camera moves).
- Publishes detected poses continuously and answers the `/detect_objects`
  service with fresh synchronized results in both world and tcp frames.
- Optional debug image with drawn detections.

## Input Expectation
- Camera topics from the wrist RGB-D sensor bridged by `arm_bringup`
  (`enable_camera:=true`), depth 32FC1 in meters.
- TF chain `world -> ... -> wrist_camera_optical_frame` available
  (robot_state_publisher).
- Boxes sit flat on the table (roll/pitch identity, yaw detected).

## Topics

### Subscribed Topics

| Topic Variable/Name | Type | What It Does |
|---|---|---|
| image_topic -> /wrist_camera/image | sensor_msgs/msg/Image | RGB input. |
| depth_topic -> /wrist_camera/depth_image | sensor_msgs/msg/Image | Depth input (32FC1, meters). |
| camera_info_topic -> /wrist_camera/camera_info | sensor_msgs/msg/CameraInfo | Intrinsics for deprojection. |

### Published Topics

| Topic Variable/Name | Type | What It Does |
|---|---|---|
| detected_objects_topic -> /detected_objects | geometry_msgs/msg/PoseArray | Detected box top-center poses in `world`. |
| debug_image_topic -> /object_detector/debug_image | sensor_msgs/msg/Image | Detections drawn on the RGB image. |

### Services

| Service | Type | What It Does |
|---|---|---|
| detect_service -> /detect_objects | arm_interfaces/srv/DetectObjects | Returns ids + poses in world and tcp frames from a fresh synchronized frame. |

## Parameters (config/detector_params.yaml)
- topic names: `image_topic`, `depth_topic`, `camera_info_topic`,
  `detected_objects_topic`, `debug_image_topic`, `detect_service`
- frames: `target_frame`, `tcp_frame`, `camera_frame`,
  `use_image_header_frame`
- detection: `colors` (per-color `hsv_low`/`hsv_high`), `min_contour_area`,
  `min_top_face_area`, `top_depth_percentile`, `top_depth_tolerance`,
  `box_height`
- timing: `publish_rate`, `max_detection_age`
- debug: `publish_debug_image`, `show_debug_window`, `print_predictions`,
  `log_static_pick_box`, `static_pick_box`

## Build and Launch
```bash
colcon build --packages-select arm_perception --symlink-install
source install/setup.bash
ros2 launch arm_perception perception.launch.py
ros2 launch arm_perception perception.launch.py show_debug_window:=true
```
