#!/bin/bash
# ============================================================
# ROS 2 Network Environment — BASE STATION LAPTOP (Ground side)
# Laptop IP  : 192.168.88.100
# Jetson IP  : 192.168.88.10
# Subnet     : 192.168.88.0/24
#
# Source this on the laptop before running RViz2 / tools:
#   source ~/ros2_network/base_station_env.sh
#
# Or add to ~/.bashrc:
#   echo "source ~/ros2_network/base_station_env.sh" >> ~/.bashrc
# ============================================================

# ---- ROS 2 installation ----
source /opt/ros/humble/setup.bash

# ---- Workspace overlay ----
# Adjust path if workspace is not at ~/rover_ws
source ~/rover_ws/install/setup.bash

# ---- Network identity ----
export ROS_IP=192.168.88.100
export ROS_HOSTNAME=192.168.88.100

# ---- DDS multicast — disable localhost isolation ----
export ROS_LOCALHOST_ONLY=0

# ---- ROS Domain (MUST match Jetson) ----
export ROS_DOMAIN_ID=42

# ---- Fast-DDS ----
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ---- Optional: force unicast to Jetson ----
# Uncomment if multicast is blocked on your WiFi AP.
# export FASTRTPS_DEFAULT_PROFILES_FILE=~/ros2_network/fastdds_unicast.xml

# ---- Convenience aliases ----
alias rover_rviz='ros2 launch rover_navigation navigation_rviz.launch.py'
alias rover_topic='ros2 topic list'
alias rover_ssh='ssh ubuntu@192.168.88.10'
alias rover_teleop='ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel'

echo "[ROS2 ENV] Base station ready."
echo "  ROS_IP        : $ROS_IP"
echo "  Jetson        : 192.168.88.10"
echo "  ROS_DOMAIN_ID : $ROS_DOMAIN_ID"
echo "  RMW           : $RMW_IMPLEMENTATION"
