#!/usr/bin/env python3
import math
import random
import sys

import rospy
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header


def build_world_points():
    points = []
    random.seed(42)

    for iy in range(-20, 21):
        for iz in range(-5, 11):
            y = iy * 0.2
            z = iz * 0.2
            points.append((6.0, y, z, 80.0))

    for ix in range(0, 31):
        for iz in range(-5, 11):
            x = 2.0 + ix * 0.2
            z = iz * 0.2
            points.append((x, -4.0, z, 60.0))

    for ix in range(0, 31):
        for iy in range(-20, 21):
            x = 2.0 + ix * 0.2
            y = iy * 0.2
            points.append((x, y, -1.0, 40.0))

    return points


def sensor_pose(scan_idx, scan_dt):
    t = scan_idx * scan_dt
    x = 0.12 * scan_idx
    y = 0.15 * math.sin(0.35 * scan_idx)
    yaw = math.radians(0.8 * scan_idx)
    return t, x, y, yaw


def world_to_sensor(point, pose_x, pose_y, pose_yaw):
    px, py, pz, intensity = point
    dx = px - pose_x
    dy = py - pose_y
    cos_yaw = math.cos(pose_yaw)
    sin_yaw = math.sin(pose_yaw)
    sx = cos_yaw * dx + sin_yaw * dy
    sy = -sin_yaw * dx + cos_yaw * dy
    sz = pz
    return sx, sy, sz, intensity


def make_cloud(world_points, scan_idx, scan_dt):
    _, pose_x, pose_y, pose_yaw = sensor_pose(scan_idx, scan_dt)
    cloud = []
    for point in world_points:
        sx, sy, sz, intensity = world_to_sensor(point, pose_x, pose_y, pose_yaw)
        if sx <= 0.5:
            continue
        if sx * sx + sy * sy + sz * sz > 80.0 * 80.0:
            continue
        if abs(math.atan2(sy, sx)) > math.radians(85.0):
            continue
        cloud.append((sx, sy, sz, intensity))
    return cloud


def fill_imu(stamp):
    imu = Imu()
    imu.header.stamp = stamp
    imu.header.frame_id = "body"
    imu.orientation.w = 1.0
    imu.angular_velocity.x = 0.0
    imu.angular_velocity.y = 0.0
    imu.angular_velocity.z = 0.0
    imu.linear_acceleration.x = 0.0
    imu.linear_acceleration.y = 0.0
    imu.linear_acceleration.z = -9.81
    return imu


def main():
    duration_sec = float(sys.argv[1]) if len(sys.argv) > 1 else 14.0
    rospy.init_node("synthetic_fastlio_stream")
    cloud_pub = rospy.Publisher("/synthetic_points", PointCloud2, queue_size=2)
    imu_pub = rospy.Publisher("/synthetic_imu", Imu, queue_size=50)

    scan_rate = 10.0
    imu_rate = 200.0
    scan_dt = 1.0 / scan_rate
    imu_dt = 1.0 / imu_rate
    world_points = build_world_points()
    total_scans = int(duration_sec * scan_rate)

    start_time = rospy.Time.now() + rospy.Duration(0.5)
    loop_rate = rospy.Rate(imu_rate)
    imu_tick = 0
    scan_idx = 0

    while not rospy.is_shutdown():
        stamp = start_time + rospy.Duration.from_sec(imu_tick * imu_dt)
        imu_pub.publish(fill_imu(stamp))

        if scan_idx < total_scans:
            expected_scan_tick = int(round(scan_idx * imu_rate / scan_rate))
            if imu_tick >= expected_scan_tick:
                header = Header()
                header.stamp = stamp
                header.frame_id = "body"
                cloud = make_cloud(world_points, scan_idx, scan_dt)
                fields = [
                    PointField("x", 0, PointField.FLOAT32, 1),
                    PointField("y", 4, PointField.FLOAT32, 1),
                    PointField("z", 8, PointField.FLOAT32, 1),
                    PointField("intensity", 12, PointField.FLOAT32, 1),
                ]
                cloud_msg = point_cloud2.create_cloud(header, fields, cloud)
                cloud_msg.header = header
                cloud_pub.publish(cloud_msg)
                scan_idx += 1

        imu_tick += 1
        if scan_idx >= total_scans and imu_tick * imu_dt > duration_sec + 0.5:
            break
        loop_rate.sleep()


if __name__ == "__main__":
    main()
