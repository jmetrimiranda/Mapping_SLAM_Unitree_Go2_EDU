#!/bin/bash
# =============================================================
# env_ros2.sh — Source this file in EVERY terminal before
# running any ROS 2 command (nodes, topic list, rviz2, etc.)
#
# USAGE:
#   source /env_ros2.sh
#
# IMPORTANT: Replace enx0c3796dc8be5 with YOUR network interface.
# Find yours on the HOST with: ip a | grep "192.168.123"
# =============================================================

# --- ROS 2 Foxy core ---
source /opt/ros/foxy/setup.bash

# --- CycloneDDS workspace ---
source /upgrade_dds_ws/install/setup.bash 2>/dev/null || true

# --- Go2 SDK workspace ---
source /go2_webrtc_ws/install/setup.bash

# --- Locale ---
export LC_ALL=C
export LC_NUMERIC="en_US.UTF-8"

# --- DDS Domain Isolation ---
export ROS_DOMAIN_ID=42

# --- DDS Implementation ---
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# --- CycloneDDS Configuration ---
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>enx0c3796dc8be5</NetworkInterfaceAddress><MaxMessageSize>65500</MaxMessageSize><FragmentSize>4000</FragmentSize></General><Internal><Watermarks><WhcHigh>500kB</WhcHigh></Watermarks></Internal></Domain></CycloneDDS>"

echo "ROS 2 environment configured:"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  RMW=$RMW_IMPLEMENTATION"
echo "  Interface=enx0c3796dc8be5"