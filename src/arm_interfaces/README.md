# arm_interfaces

Shared ROS interfaces for the perception/manipulation pipeline. Interface-only
package, no nodes.

## Services

### DetectObjects.srv

Request: empty.

Response:

| Field | Type | What It Is |
|---|---|---|
| success | bool | Whether a fresh synchronized detection was produced. |
| message | string | Debug info (result age, failure reason). |
| ids | string[] | Detected object ids, e.g. box_red_0. |
| tcp_poses | geometry_msgs/PoseArray | Box poses in the tcp frame. |
| world_poses | geometry_msgs/PoseArray | Box poses in the world frame. |

## Used By
- Server: `arm_perception` (`/detect_objects`).
- Clients: `arm_mtc` (`pick_place_mtc`), `arm_manipulation`
  (`pick_place_dynamic`, `tune_pick_place`).
