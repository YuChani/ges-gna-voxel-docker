#!/usr/bin/env bash
set -euo pipefail

MODE="${1:?mode required: baseline|proposed|center|hybrid}"
OUT_DIR="${2:?output dir required}"
BAG_PATH="${3:?bag path required}"

if [[ "$MODE" != "baseline" && "$MODE" != "proposed" && "$MODE" != "center" && "$MODE" != "hybrid" ]]; then
  echo "Invalid mode: $MODE" >&2
  exit 1
fi

mkdir -p "$OUT_DIR"
PRIMITIVE_LOG_CSV_PATH="$OUT_DIR/primitive_runtime.csv"
if [[ "$PRIMITIVE_LOG_CSV_PATH" != /* ]]; then
  PRIMITIVE_LOG_CSV_PATH="$(pwd)/$PRIMITIVE_LOG_CSV_PATH"
fi
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-127.0.0.1}"

source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

pkill -f roscore >/dev/null 2>&1 || true
pkill -f rosmaster >/dev/null 2>&1 || true
pkill -f fastlio_mapping >/dev/null 2>&1 || true
sleep 1

cleanup() {
  jobs -p | xargs -r kill >/dev/null 2>&1 || true
}
trap cleanup EXIT

if [[ "$MODE" == "baseline" ]]; then
  PRIMITIVE_MODE=0
elif [[ "$MODE" == "center" ]]; then
  PRIMITIVE_MODE=2
elif [[ "$MODE" == "hybrid" ]]; then
  PRIMITIVE_MODE=3
else
  PRIMITIVE_MODE=1
fi

printf start > "$OUT_DIR/01_start.txt"
roscore > "$OUT_DIR/roscore.log" 2>&1 &
sleep 3
printf roscore_ready > "$OUT_DIR/02_roscore_ready.txt"

timeout 110s roslaunch fast_lio mapping_avia_param.launch rviz:=false primitive_mode:="$PRIMITIVE_MODE" primitive_log_enable:=true primitive_log_csv_path:="$PRIMITIVE_LOG_CSV_PATH" pcd_save_en:=false > "$OUT_DIR/launch.log" 2>&1 &
LAUNCH_PID=$!
sleep 6
printf launch_ready > "$OUT_DIR/03_launch_ready.txt"

rostopic echo -p /Odometry > "$OUT_DIR/odometry.csv" 2>/dev/null &
ODOM_PID=$!
sleep 1

rosbag play "$BAG_PATH" > "$OUT_DIR/bag.log" 2>&1
sleep 3

rosnode list > "$OUT_DIR/rosnode.txt" 2>&1 || true
rostopic list > "$OUT_DIR/rostopic.txt" 2>&1 || true

kill "$ODOM_PID" >/dev/null 2>&1 || true
wait "$ODOM_PID" >/dev/null 2>&1 || true
wait "$LAUNCH_PID" >/dev/null 2>&1 || true

printf done > "$OUT_DIR/99_done.txt"
