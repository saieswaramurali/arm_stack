# Wrist Camera Perception Root Cause Analysis

## Summary

The wrist camera object detector was producing stable detections, but the detected world coordinates were wrong by about 20 cm. The robot reached the correct detection pose, and the spawned box was at the expected table location. The root cause was incorrect camera intrinsics coming from the bridged `/wrist_camera/camera_info` topic.

The camera image was 640 by 480, but `camera_info.k` reported the optical center as `(160, 120)`, which corresponds to a 320 by 240 image. The detector trusted those values, so deprojection shifted every object away from its true position.

## Impact

The dynamic pick pipeline could move correctly to a perceived target, but the perceived target was not the real box center. This caused the robot to reach a wrong grasp point even when the image detection looked visually correct.

## Timeline

1. The wrist camera was added to the robot and detection started publishing colored box poses.
2. Static and random object spawning worked, and the debug image showed correct segmentation.
3. The detector returned world coordinates that did not match Gazebo spawn coordinates.
4. A center box calibration was run with one red box at `(0.000, 0.750, 0.360)`.
5. The camera pose was verified as centered over the box:

```text
actual_camera world: x=0.0003 y=0.7498 z=0.6519 rpy=(179.9 -0.3 -0.2)deg
```

6. The detector still reported:

```text
box_red_0 world_top: x=0.1676 y=0.6242 z=0.3601
```

7. A URDF sensor pose correction was tried and rejected because it made the Z estimate wrong.
8. The `/wrist_camera/camera_info` topic was inspected and showed incorrect intrinsics.
9. The detector was changed to compute intrinsics from the actual image size and configured horizontal FOV.
10. Center and off axis tests then passed.

## Symptoms

The center test spawned one box at:

```text
spawned box_r_0: center=(0.000, 0.750, 0.330) top_center=(0.000, 0.750, 0.360)
```

Before the fix, the detector reported:

```text
box_red_0 world_top: x=0.1676 y=0.6242 z=0.3601
```

The Z value was correct, but X and Y were shifted. That meant depth was valid and color segmentation was valid, while image to world projection was wrong.

## What Was Ruled Out

### Spawn Location

The spawner printed the true box top center, and the requested calibration position matched the detector pose target:

```text
spawn top center: x=0.000 y=0.750 z=0.360
detect pose: x=0.000 y=0.750 z=0.650
```

So the box was spawned under the intended camera pose.

### Robot Detection Pose

The robot reached the intended wrist camera pose:

```text
actual_camera world: x=0.0003 y=0.7498 z=0.6519 rpy=(179.9 -0.3 -0.2)deg
```

So the MoveIt pose target was not the source of the offset.

### Color Segmentation

The OpenCV debug window showed the correct red, green, and blue boxes with stable contours. The detector produced repeatable results at the same image positions, so HSV thresholding was not the root cause.

### Stale Perception Results

The detector service was updated to reject stale cached detections. The reported detection age was fresh:

```text
returned fresh synchronized detector result age=0.098000s
```

So stale data was not causing the coordinate error.

### URDF Sensor Pose

A sensor pose correction was tested in the URDF. It made the reported Z value wrong:

```text
box_red_0 world_top: x=0.3586 y=0.4784 z=0.4130
```

Since the original Z value was already correct, this showed that changing the physical sensor pose was the wrong fix. The URDF change was reverted.

## Root Cause

The bridged camera info was inconsistent with the actual image resolution.

Observed `/wrist_camera/camera_info`:

```text
height: 480
width: 640
k:
- 277.0
- 0.0
- 160.0
- 0.0
- 277.0
- 120.0
- 0.0
- 0.0
- 1.0
```

For a 640 by 480 image, the image center should be around:

```text
cx=320
cy=240
```

The camera info reported:

```text
cx=160
cy=120
```

That caused the deprojection formula to treat the real image center as an off axis point. As a result, a box directly below the camera was projected to the wrong X and Y world position.

## Fix

The detector now computes intrinsics from the actual image dimensions and known horizontal FOV instead of trusting the incorrect bridged camera info.

Effective values:

```text
width=640
height=480
horizontal_fov=1.21
cx=width / 2
cy=height / 2
fx=width / (2 * tan(horizontal_fov / 2))
fy=fx
```

The detector still uses the standard optical frame pinhole projection:

```text
x=(u-cx)*d/fx
y=(v-cy)*d/fy
z=d
```

The detector also keeps:

```text
camera_frame: wrist_camera_optical_frame
```

No compensating axis remap was added.

## Verification

### Center Box Test

Spawned:

```text
top_center=(0.000, 0.750, 0.360)
```

Detected after fix:

```text
box_red_0 world_top: x=0.0001 y=0.7499 z=0.3600
```

Result:

```text
position error is approximately 0.1 mm in X and 0.1 mm in Y
```

### Off Axis Test

Spawned:

```text
box_r_0 top_center=(0.150, 0.750, 0.360)
box_g_1 top_center=(0.000, 0.900, 0.360)
```

Detected:

```text
box_red_0 world_top: x=0.1465 y=0.7503 z=0.3600
box_green_0 world_top: x=-0.0000 y=0.8870 z=0.3600
```

Errors:

```text
red error is about 3.5 mm
green error is about 13 mm
```

Both are within the target tolerance for the current dynamic pick pipeline.

## Final State

The camera pose, object spawning, depth values, TF conversion, and deprojection are now consistent enough for dynamic pick and place testing.

The remaining tuning should focus on grasp offsets, approach height, gripper closure, and placement behavior. Perception frame correction should not be changed unless new calibration data shows a consistent error outside the current tolerance.

## Lessons Learned

1. Always compare `camera_info.k` against actual image width and height before debugging TF.
2. A correct Z value with wrong X and Y usually points to intrinsics or optical center, not depth.
3. A visible and stable debug contour does not prove correct 3D projection.
4. Do not add axis remaps to hide calibration errors.
5. One change per iteration made the real root cause clear.
