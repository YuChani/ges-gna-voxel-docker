#!/usr/bin/env bash
set -euo pipefail

MODE="${1:?mode required: baseline|proposed|center}"
OUT_DIR="${2:?output dir required}"

if [[ "$MODE" != "baseline" && "$MODE" != "proposed" && "$MODE" != "center" ]]; then
  echo "Invalid mode: $MODE" >&2
  exit 1
fi

mkdir -p "$OUT_DIR"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"

source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

pkill -f fastlio_mapping >/dev/null 2>&1 || true
pkill -f synthetic_fastlio_stream.py >/dev/null 2>&1 || true
pkill -f roscore >/dev/null 2>&1 || true
pkill -f rosmaster >/dev/null 2>&1 || true
sleep 1

cleanup() {
  jobs -p | xargs -r kill >/dev/null 2>&1 || true
}
trap cleanup EXIT

roscore > "$OUT_DIR/roscore.log" 2>&1 &
sleep 3

rosparam load /root/catkin_ws/FAST_LIO/config/marsim.yaml
rosparam set common/lid_topic /synthetic_points
rosparam set common/imu_topic /synthetic_imu
rosparam set preprocess/lidar_type 4
rosparam set preprocess/scan_rate 10
rosparam set feature_extract_enable false
rosparam set point_filter_num 1
rosparam set max_iteration 3
rosparam set filter_size_surf 0.2
rosparam set filter_size_map 0.2
rosparam set cube_side_length 200
rosparam set runtime_pos_log_enable false
rosparam set mapping/primitive_log_enable true
rosparam set mapping/primitive_log_csv_path "$OUT_DIR/primitive_runtime.csv"

if [[ "$MODE" == "baseline" ]]; then
  rosparam set mapping/primitive_mode 0
elif [[ "$MODE" == "center" ]]; then
  rosparam set mapping/primitive_mode 2
else
  rosparam set mapping/primitive_mode 1
fi

python3 /root/catkin_ws/scripts/synthetic_fastlio_stream.py 14 > "$OUT_DIR/publisher.log" 2>&1 &
PUBLISHER_PID=$!

timeout 20s rosrun fast_lio fastlio_mapping > "$OUT_DIR/mapping.log" 2>&1 &
MAPPING_PID=$!

rostopic echo -p /Odometry > "$OUT_DIR/odometry.csv" 2>/dev/null &
ODOM_PID=$!

wait "$PUBLISHER_PID"
sleep 2
kill "$ODOM_PID" >/dev/null 2>&1 || true
wait "$MAPPING_PID" || true

python3 /root/catkin_ws/scripts/evaluate_synthetic_map_quality.py "$OUT_DIR/odometry.csv" "$OUT_DIR/map_quality.json"

rosnode list > "$OUT_DIR/rosnode.txt"
rostopic list > "$OUT_DIR/rostopic.txt"
