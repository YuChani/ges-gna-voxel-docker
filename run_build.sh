#!/bin/bash
export PROJECT_NAME="ges-voxel"

read -p "Enter username: " USERNAME
read -p "Enter work folder name: " WORK_FOLDER

# Ensure environment variables are set for Docker Compose
export USERNAME
export WORK_FOLDER
export __ROS_MASTER_URI="http://127.0.0.1:11311"
export __ROS_HOSTNAME="127.0.0.1"

# Set the appropriate Docker Compose file based on the platform
COMPOSE_FILE="./docker-compose/${PROJECT_NAME}_compose.yml"

up_build_command=(
    "docker-compose"
    "--file"
    "${COMPOSE_FILE}"
    "up"
    "-d"
    "${PROJECT_NAME}"
)

exec_command=(
    "docker"
    "exec"
    "-it"
    "${PROJECT_NAME}"
    "/bin/bash"
    "-c"
    "source /opt/ros/noetic/setup.bash && cd ~/catkin_ws && catkin_make && source ~/catkin_ws/devel/setup.bash"
)

stop_command=(
    "docker"
    "stop"
    "${PROJECT_NAME}"
)

# Allow X11 display access (for rviz and other GUI nodes)
xhost +local:root

"${up_build_command[@]}"

# Wait for container to be ready
echo "Waiting for container to start..."
sleep 3

"${exec_command[@]}"

"${stop_command[@]}"
