#!/usr/bin/env python3

import argparse
import math
import random
import subprocess
import time


TABLE_X = 0.0
TABLE_Y = 0.75
TABLE_TOP_Z = 0.30
TABLE_SIZE_X = 0.80
TABLE_SIZE_Y = 0.80
STATIC_BOX_X = 0.0
STATIC_BOX_Y = 0.75
SPAWN_RADIUS = 0.20
BOX_SIZE_X = 0.04
BOX_SIZE_Y = 0.04
BOX_SIZE_Z = 0.06
BOX_MASS = 0.05
POSE_MARGIN = 0.10
MIN_SEPARATION = 0.11
BOX_COLORS = [
    ("r", "1 0.05 0.05 1"),
    ("g", "0.05 0.8 0.1 1"),
    ("b", "0.05 0.25 1 1"),
]
BOX_COLOR_MAP = dict(BOX_COLORS)
DROP_BIN_NAME = "drop_bin"
FIXED_BOX_NAME = "pick_box"
BIN_SUPPORT_HEIGHT = TABLE_TOP_Z


def run_ign_service(world, service, reqtype, request, timeout=5.0, attempts=1):
    cmd = [
        "ign",
        "service",
        "-s",
        f"/world/{world}/{service}",
        "--reqtype",
        reqtype,
        "--reptype",
        "ignition.msgs.Boolean",
        "--timeout",
        str(int(timeout * 1000)),
        "--req",
        request,
    ]
    last_error = None
    for _ in range(attempts):
        result = subprocess.run(cmd, text=True, capture_output=True, check=False)
        if result.returncode == 0 and "data: true" in result.stdout:
            return True
        last_error = (result.stdout + result.stderr).strip()
        time.sleep(0.5)
    if last_error:
        print(last_error)
    return False


def inertia_box(mass, x, y, z):
    return (
        mass * (y * y + z * z) / 12.0,
        mass * (x * x + z * z) / 12.0,
        mass * (x * x + y * y) / 12.0,
    )


def box_sdf(name, color):
    ixx, iyy, izz = inertia_box(BOX_MASS, BOX_SIZE_X, BOX_SIZE_Y, BOX_SIZE_Z)
    return f"""
<sdf version="1.7">
  <model name="{name}">
    <static>false</static>
    <link name="box_link">
      <inertial>
        <mass>{BOX_MASS}</mass>
        <inertia>
          <ixx>{ixx:.10f}</ixx><iyy>{iyy:.10f}</iyy><izz>{izz:.10f}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="box_collision">
        <geometry><box><size>{BOX_SIZE_X} {BOX_SIZE_Y} {BOX_SIZE_Z}</size></box></geometry>
        <surface>
          <friction><ode><mu>1.5</mu><mu2>1.5</mu2></ode></friction>
        </surface>
      </collision>
      <visual name="box_visual">
        <geometry><box><size>{BOX_SIZE_X} {BOX_SIZE_Y} {BOX_SIZE_Z}</size></box></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
""".strip()


def drop_bin_sdf():
    support_z = BIN_SUPPORT_HEIGHT / 2.0
    base_z = BIN_SUPPORT_HEIGHT + 0.005
    wall_z = BIN_SUPPORT_HEIGHT + 0.055
    return f"""
<sdf version="1.7">
  <model name="drop_bin">
    <static>true</static>
    <link name="bin_link">
      <collision name="support_collision">
        <pose>0 0 {support_z:.3f} 0 0 0</pose>
        <geometry><box><size>0.24 0.20 {BIN_SUPPORT_HEIGHT:.3f}</size></box></geometry>
      </collision>
      <visual name="support_visual">
        <pose>0 0 {support_z:.3f} 0 0 0</pose>
        <geometry><box><size>0.24 0.20 {BIN_SUPPORT_HEIGHT:.3f}</size></box></geometry>
        <material><ambient>0.35 0.36 0.34 1</ambient><diffuse>0.35 0.36 0.34 1</diffuse></material>
      </visual>
      <collision name="base_collision">
        <pose>0 0 {base_z:.3f} 0 0 0</pose>
        <geometry><box><size>0.24 0.20 0.01</size></box></geometry>
      </collision>
      <visual name="base_visual">
        <pose>0 0 {base_z:.3f} 0 0 0</pose>
        <geometry><box><size>0.24 0.20 0.01</size></box></geometry>
        <material><ambient>0.9 0.75 0.05 1</ambient><diffuse>0.9 0.75 0.05 1</diffuse></material>
      </visual>
      <collision name="front_wall_collision">
        <pose>0 -0.105 {wall_z:.3f} 0 0 0</pose>
        <geometry><box><size>0.24 0.01 0.11</size></box></geometry>
      </collision>
      <visual name="front_wall_visual">
        <pose>0 -0.105 {wall_z:.3f} 0 0 0</pose>
        <geometry><box><size>0.24 0.01 0.11</size></box></geometry>
        <material><ambient>0.9 0.75 0.05 1</ambient><diffuse>0.9 0.75 0.05 1</diffuse></material>
      </visual>
      <collision name="back_wall_collision">
        <pose>0 0.105 {wall_z:.3f} 0 0 0</pose>
        <geometry><box><size>0.24 0.01 0.11</size></box></geometry>
      </collision>
      <visual name="back_wall_visual">
        <pose>0 0.105 {wall_z:.3f} 0 0 0</pose>
        <geometry><box><size>0.24 0.01 0.11</size></box></geometry>
        <material><ambient>0.9 0.75 0.05 1</ambient><diffuse>0.9 0.75 0.05 1</diffuse></material>
      </visual>
      <collision name="left_wall_collision">
        <pose>-0.125 0 {wall_z:.3f} 0 0 0</pose>
        <geometry><box><size>0.01 0.20 0.11</size></box></geometry>
      </collision>
      <visual name="left_wall_visual">
        <pose>-0.125 0 {wall_z:.3f} 0 0 0</pose>
        <geometry><box><size>0.01 0.20 0.11</size></box></geometry>
        <material><ambient>0.9 0.75 0.05 1</ambient><diffuse>0.9 0.75 0.05 1</diffuse></material>
      </visual>
      <collision name="right_wall_collision">
        <pose>0.125 0 {wall_z:.3f} 0 0 0</pose>
        <geometry><box><size>0.01 0.20 0.11</size></box></geometry>
      </collision>
      <visual name="right_wall_visual">
        <pose>0.125 0 {wall_z:.3f} 0 0 0</pose>
        <geometry><box><size>0.01 0.20 0.11</size></box></geometry>
        <material><ambient>0.9 0.75 0.05 1</ambient><diffuse>0.9 0.75 0.05 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
""".strip()


def list_models():
    result = subprocess.run(
        ["ign", "model", "--list"], text=True, capture_output=True, check=False
    )
    if result.returncode != 0:
        return None

    names = set()
    for line in result.stdout.splitlines():
        name = line.strip().lstrip("-").strip()
        if name:
            names.add(name)
    return names


def delete_model(world, name, attempts=1):
    request = f'name: "{name}" type: MODEL'
    return run_ign_service(
        world, "remove", "ignition.msgs.Entity", request, attempts=attempts
    )


def create_model(world, name, sdf, x, y, z, yaw):
    escaped_sdf = sdf.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")
    request = (
        f'name: "{name}" '
        f'sdf: "{escaped_sdf}" '
        f'pose {{ position {{ x: {x:.4f} y: {y:.4f} z: {z:.4f} }} '
        f'orientation {{ x: 0 y: 0 z: {math.sin(yaw / 2.0):.8f} w: {math.cos(yaw / 2.0):.8f} }} }}'
    )
    return run_ign_service(
        world, "create", "ignition.msgs.EntityFactory", request, attempts=20
    )


def random_table_poses(count):
    x_min = TABLE_X - TABLE_SIZE_X / 2.0 + POSE_MARGIN
    x_max = TABLE_X + TABLE_SIZE_X / 2.0 - POSE_MARGIN
    y_min = TABLE_Y - TABLE_SIZE_Y / 2.0 + POSE_MARGIN
    y_max = TABLE_Y + TABLE_SIZE_Y / 2.0 - POSE_MARGIN
    poses = []
    for _ in range(count):
        for _ in range(200):
            radius = SPAWN_RADIUS * math.sqrt(random.random())
            theta = random.uniform(-math.pi, math.pi)
            x = STATIC_BOX_X + radius * math.cos(theta)
            y = STATIC_BOX_Y + radius * math.sin(theta)
            if not (x_min <= x <= x_max and y_min <= y <= y_max):
                continue
            if all(math.hypot(x - px, y - py) >= MIN_SEPARATION for px, py, _ in poses):
                poses.append((x, y, random.uniform(-math.pi, math.pi)))
                break
        else:
            raise RuntimeError("could not find collision-free table poses")
    return poses


def csv_values(value, cast=str):
    if value is None:
        return None
    return [cast(item.strip()) for item in value.split(",") if item.strip()]


def value_at(values, index, default):
    if not values:
        return default
    if len(values) == 1:
        return values[0]
    return values[index]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", default="table_pick_ign")
    parser.add_argument("--count", type=int, default=3)
    parser.add_argument("--reset", action="store_true")
    parser.add_argument("--x", help="Fixed box x, or comma-separated x values")
    parser.add_argument("--y", help="Fixed box y, or comma-separated y values")
    parser.add_argument("--yaw", help="Fixed yaw, or comma-separated yaw values")
    parser.add_argument("--color", help="Fixed color key r/g/b, or comma-separated keys")
    args = parser.parse_args()

    if args.count < 1:
        raise SystemExit("--count must be positive")

    xs = csv_values(args.x, float)
    ys = csv_values(args.y, float)
    yaws = csv_values(args.yaw, float)
    color_keys = csv_values(args.color, str)
    for values, label in ((xs, "--x"), (ys, "--y"), (yaws, "--yaw"), (color_keys, "--color")):
        if values is not None and len(values) not in (1, args.count):
            raise SystemExit(f"{label} must have one value or exactly --count values")
    if color_keys:
        bad_colors = [color for color in color_keys if color not in BOX_COLOR_MAP]
        if bad_colors:
            raise SystemExit(f"unknown --color values: {', '.join(bad_colors)}")

    colors = [
        value_at(color_keys, index, BOX_COLORS[index % len(BOX_COLORS)][0])
        for index in range(args.count)
    ]
    names = [f"box_{colors[index]}_{index}" for index in range(args.count)]

    if args.reset:
        existing_models = list_models()
        reset_names = [FIXED_BOX_NAME, DROP_BIN_NAME] + [
            f"box_{color}_{index}"
            for index in range(max(args.count, 10))
            for color, _ in BOX_COLORS
        ]
        for name in reset_names:
            if existing_models is not None and name not in existing_models:
                continue
            delete_model(args.world, name, attempts=20 if name == FIXED_BOX_NAME else 1)

    if xs is not None or ys is not None:
        if xs is None or ys is None:
            raise SystemExit("--x and --y must be passed together")
        poses = [
            (
                value_at(xs, index, STATIC_BOX_X),
                value_at(ys, index, STATIC_BOX_Y),
                value_at(yaws, index, 0.0),
            )
            for index in range(args.count)
        ]
    else:
        poses = random_table_poses(args.count)
    box_z = TABLE_TOP_Z + BOX_SIZE_Z / 2.0
    for index, (name, pose) in enumerate(zip(names, poses)):
        color = BOX_COLOR_MAP[colors[index]]
        x, y, yaw = pose
        if not create_model(args.world, name, box_sdf(name, color), x, y, box_z, yaw):
            raise SystemExit(f"failed to spawn {name}")
        print(
            f"spawned {name}: center=({x:.3f}, {y:.3f}, {box_z:.3f}) "
            f"top_center=({x:.3f}, {y:.3f}, {box_z + BOX_SIZE_Z / 2.0:.3f}) "
            f"yaw={yaw:.2f}"
        )

    bin_x = 0.48
    bin_y = 0.35
    bin_z = 0.0
    if not create_model(args.world, DROP_BIN_NAME, drop_bin_sdf(), bin_x, bin_y, bin_z, 0.0):
        raise SystemExit(f"failed to spawn {DROP_BIN_NAME}")
    print(f"spawned {DROP_BIN_NAME} at x={bin_x:.3f}, y={bin_y:.3f}")


if __name__ == "__main__":
    main()
