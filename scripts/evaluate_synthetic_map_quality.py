#!/usr/bin/env python3
import csv
import json
import math
import sys


def build_world_points():
    points = []
    for iy in range(-20, 21):
        for iz in range(-5, 11):
            points.append((6.0, iy * 0.2, iz * 0.2, 80.0, "front_wall"))
    for ix in range(0, 31):
        for iz in range(-5, 11):
            points.append((2.0 + ix * 0.2, -4.0, iz * 0.2, 60.0, "side_wall"))
    for ix in range(0, 31):
        for iy in range(-20, 21):
            points.append((2.0 + ix * 0.2, iy * 0.2, -1.0, 40.0, "floor"))
    return points


def sensor_pose(scan_idx):
    x = 0.12 * scan_idx
    y = 0.15 * math.sin(0.35 * scan_idx)
    yaw = math.radians(0.8 * scan_idx)
    return x, y, yaw


def world_to_sensor(point, pose_x, pose_y, pose_yaw):
    px, py, pz, intensity, label = point
    dx = px - pose_x
    dy = py - pose_y
    cos_yaw = math.cos(pose_yaw)
    sin_yaw = math.sin(pose_yaw)
    sx = cos_yaw * dx + sin_yaw * dy
    sy = -sin_yaw * dx + cos_yaw * dy
    sz = pz
    return sx, sy, sz, intensity, label


def make_cloud(world_points, scan_idx):
    pose_x, pose_y, pose_yaw = sensor_pose(scan_idx)
    cloud = []
    for point in world_points:
        sx, sy, sz, intensity, label = world_to_sensor(point, pose_x, pose_y, pose_yaw)
        if sx <= 0.5:
            continue
        if sx * sx + sy * sy + sz * sz > 80.0 * 80.0:
            continue
        if abs(math.atan2(sy, sx)) > math.radians(85.0):
            continue
        cloud.append((sx, sy, sz, intensity, label))
    return cloud


def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def body_to_world(point, pose_x, pose_y, pose_yaw):
    sx, sy, sz, _, label = point
    cos_yaw = math.cos(pose_yaw)
    sin_yaw = math.sin(pose_yaw)
    wx = pose_x + cos_yaw * sx - sin_yaw * sy
    wy = pose_y + sin_yaw * sx + cos_yaw * sy
    wz = sz
    return wx, wy, wz, label


def residual_for_label(point):
    wx, wy, wz, label = point
    if label == "front_wall":
        return wx - 6.0
    if label == "side_wall":
        return wy + 4.0
    return wz + 1.0


def summarize(values):
    if not values:
        return {"count": 0, "mean_abs": None, "std": None, "max_abs": None}
    mean = sum(values) / len(values)
    var = sum((v - mean) ** 2 for v in values) / len(values)
    abs_vals = [abs(v) for v in values]
    return {
        "count": len(values),
        "mean_abs": sum(abs_vals) / len(abs_vals),
        "std": math.sqrt(var),
        "max_abs": max(abs_vals),
    }


def main():
    odom_csv = sys.argv[1]
    out_json = sys.argv[2]
    world_points = build_world_points()
    labels = {"front_wall": [], "side_wall": []}

    with open(odom_csv, newline="") as f:
        rows = list(csv.DictReader(f))

    if not rows:
        summary = {
            "trajectory_xy_error_mean": None,
            "trajectory_xy_error_max": None,
            "wall_thickness_proxy": {label: summarize(vals) for label, vals in labels.items()},
        }
        with open(out_json, 'w') as f:
            json.dump(summary, f, indent=2)
        return

    traj_xy_errors = []
    for idx, row in enumerate(rows, start=1):
        gt_x, gt_y, _ = sensor_pose(idx)
        est_x = float(row['field.pose.pose.position.x'])
        est_y = float(row['field.pose.pose.position.y'])
        est_z = float(row['field.pose.pose.position.z'])
        qx = float(row['field.pose.pose.orientation.x'])
        qy = float(row['field.pose.pose.orientation.y'])
        qz = float(row['field.pose.pose.orientation.z'])
        qw = float(row['field.pose.pose.orientation.w'])
        est_yaw = quat_to_yaw(qx, qy, qz, qw)

        traj_xy_errors.append(math.hypot(est_x - gt_x, est_y - gt_y))

        for point in make_cloud(world_points, idx):
            world_point = body_to_world(point, est_x, est_y, est_yaw)
            if world_point[3] in labels:
                labels[world_point[3]].append(residual_for_label(world_point))

    summary = {
        "trajectory_xy_error_mean": sum(traj_xy_errors) / len(traj_xy_errors),
        "trajectory_xy_error_max": max(traj_xy_errors),
        "wall_thickness_proxy": {label: summarize(vals) for label, vals in labels.items()},
    }

    with open(out_json, 'w') as f:
        json.dump(summary, f, indent=2)


if __name__ == "__main__":
    main()
