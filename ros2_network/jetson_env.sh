#!/bin/bash
# ============================================================
# ROS 2 Network Environment — JETSON (Rover Side)
# Jetson IP : 192.168.88.10
# Subnet    : 192.168.88.0/24
#
# Source this on the Jetson before launching any ROS 2 node:
#   source ~/ros2_network/jetson_env.sh
#
# Or add to ~/.bashrc:
#   echo "source ~/ros2_network/jetson_env.sh" >> ~/.bashrc
# ============================================================

# ---- ROS 2 installation ----
source /opt/ros/humble/setup.bash

# ---- Workspace overlay ----
# Adjust path if your workspace is in a different location.
source ~/rover_ws/install/setup.bash

# ---- Network identity ----
export ROS_IP=192.168.88.10
export ROS_HOSTNAME=192.168.88.10

# ---- DDS multicast — disable localhost isolation ----
# Allows discovery across the 192.168.88.0/24 subnet.
export ROS_LOCALHOST_ONLY=0

# ---- ROS Domain (keep matching on both machines) ----
export ROS_DOMAIN_ID=42

# ---- Fast-DDS (default in ROS 2 Humble) ----
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ---- Optional: force unicast to base station ----
# Uncomment if multicast is blocked on your WiFi AP.
# export FASTRTPS_DEFAULT_PROFILES_FILE=~/ros2_network/fastdds_unicast.xml

# ---- CAN interface (SocketCAN) ----
export CAN_INTERFACE=can0

# ---- Convenience aliases ----
alias rover_launch='ros2 launch rover_bringup full_autonomy.launch.py use_hardware:=true'
alias rover_hw='ros2 launch rover_bringup real_hardware.launch.py'

echo "[ROS2 ENV] Jetson rover node ready."
echo "  ROS_IP        : $ROS_IP"
echo "  ROS_DOMAIN_ID : $ROS_DOMAIN_ID"
echo "  RMW           : $RMW_IMPLEMENTATION"
