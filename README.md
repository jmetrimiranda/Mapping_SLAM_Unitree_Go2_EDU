# Mapping SLAM — Unitree Go2 EDU

Source code for the WebRTC and Unitree Go2 EDU SDK, featuring custom camera calibrations, LiDAR processing, and DDS parameters. This setup has been fully validated and homologated for **ROS 2 Foxy**.

---

## Table of Contents

1. [Quick Start with Pre-built Image (Recommended)](#1-quick-start-with-pre-built-image-recommended)
   - 1.1 [Prerequisites](#11-prerequisites)
   - 1.2 [Host Preparation](#12-host-preparation)
   - 1.3 [Host Network Setup](#13-host-network-setup)
   - 1.4 [Pull the Pre-built Image](#14-pull-the-pre-built-image)
   - 1.5 [Create the Docker Compose File](#15-create-the-docker-compose-file)
   - 1.6 [Start the Container](#16-start-the-container)
2. [Building from Source (Alternative)](#2-building-from-source-alternative)
3. [Environment Setup Script (`env_ros2.sh`)](#3-environment-setup-script-env_ros2sh)
4. [Running the SLAM Pipeline](#4-running-the-slam-pipeline)
   - 4.1 [C++ High-Performance Mode — 2 Terminals (Recommended)](#41-c-high-performance-mode--2-terminals-recommended)
   - 4.2 [Python Mode — 2 Terminals (Legacy)](#42-python-mode--2-terminals-legacy)
   - 4.3 [Standard Mode — Multi-Terminal (5 Terminals)](#43-standard-mode--multi-terminal-5-terminals)
   - 4.4 [YAML-Tuned Mode — Industrial / Power Plant (Recommended)](#44-yaml-tuned-mode--industrial--power-plant-recommended)
5. [Camera / Video Stream](#5-camera--video-stream)
   - 5.1 [Enabling the Camera](#51-enabling-the-camera)
   - 5.2 [Published Topics](#52-published-topics)
   - 5.3 [RViz2 Camera Setup](#53-rviz2-camera-setup)
   - 5.4 [One-Command Launch Script](#54-one-command-launch-script)
6. [LiDAR L1 Tuning Guide](#6-lidar-l1-tuning-guide)
   - 6.1 [How the 2D Scan Works](#61-how-the-2d-scan-works)
   - 6.2 [Environment Profiles](#62-environment-profiles)
   - 6.3 [Practical LiDAR Test — Object Detection](#63-practical-lidar-test--object-detection)
   - 6.4 [Diagnostic Commands](#64-diagnostic-commands)
   - 6.5 [Common Noise Sources and Fixes](#65-common-noise-sources-and-fixes)
   - 6.6 [Quick Tuning Reference](#66-quick-tuning-reference)
7. [Architecture Overview](#7-architecture-overview)
   - 7.1 [C++ High-Performance Pipeline](#71-c-high-performance-pipeline)
   - 7.2 [Python Legacy Pipeline](#72-python-legacy-pipeline)
8. [Parameter Reference](#8-parameter-reference)
   - 8.1 [WebRTC Driver Parameters](#81-webrtc-driver-parameters)
   - 8.2 [PointCloud Slicer Parameters](#82-pointcloud-slicer-parameters)
   - 8.3 [SLAM Parameters](#83-slam-parameters)
   - 8.4 [TF Publisher Parameters](#84-tf-publisher-parameters)
9. [Docker Image Tags](#9-docker-image-tags)
10. [Saving Maps](#10-saving-maps)
11. [Autonomous Navigation (Nav2)](#11-autonomous-navigation-nav2)
    - 11.1 [How Nav2 Works With a Saved Map](#111-how-nav2-works-with-a-saved-map)
    - 11.2 [Prerequisites](#112-prerequisites)
    - 11.3 [Files Involved and What Each Does](#113-files-involved-and-what-each-does)
    - 11.4 [What to Change When Using a New Map Folder](#114-what-to-change-when-using-a-new-map-folder)
    - 11.5 [Running the Navigation Pipeline](#115-running-the-navigation-pipeline)
    - 11.6 [RViz2 Setup for Navigation](#116-rviz2-setup-for-navigation)
    - 11.7 [Performing Autonomous Navigation](#117-performing-autonomous-navigation)
    - 11.8 [Verifying the Pipeline is Healthy](#118-verifying-the-pipeline-is-healthy)
    - 11.9 [Confirmed Working Indicators](#119-confirmed-working-indicators)
    - 11.10 [Troubleshooting Nav2](#1110-troubleshooting-nav2)
12. [Segmented Mapping (Large Environments)](#12-segmented-mapping-large-environments)
    - 12.1 [How Segmented Mapping Works](#121-how-segmented-mapping-works)
    - 12.2 [Scripts Overview](#122-scripts-overview)
    - 12.3 [slam_pipeline_segments.sh](#123-slam_pipeline_segmentssh)
    - 12.4 [salvar_segmento.sh](#124-salvar_segmentosh)
    - 12.5 [Step-by-Step Workflow](#125-step-by-step-workflow)
    - 12.6 [Customizing Parameters](#126-customizing-parameters)
    - 12.7 [File Structure](#127-file-structure)
13. [YAML Parameter Reference — Advanced Tuning](#13-yaml-parameter-reference--advanced-tuning)
    - 13.1 [Why the YAML Mode Matters](#131-why-the-yaml-mode-matters)
    - 13.2 [Frames and Topics](#132-frames-and-topics)
    - 13.3 [Map Quality](#133-map-quality)
    - 13.4 [Scan Insertion](#134-scan-insertion)
    - 13.5 [Angular Drift Correction — Critical Parameters](#135-angular-drift-correction--critical-parameters)
    - 13.6 [Scan Matching Correlation Space](#136-scan-matching-correlation-space)
    - 13.7 [Loop Closure](#137-loop-closure)
    - 13.8 [Solver (Ceres)](#138-solver-ceres)
    - 13.9 [Response Expansion](#139-response-expansion)
    - 13.10 [Complete YAML — Power Plant Profile](#1310-complete-yaml--power-plant-profile)
14. [Troubleshooting](#14-troubleshooting)

---

## 1. Quick Start with Pre-built Image (Recommended)

A fully configured Docker image with ROS 2 Foxy, CycloneDDS, the Go2 SDK, and all dependencies pre-installed is available on Docker Hub. **No compilation needed — just pull and run.**

### 1.1 Prerequisites

- Linux (Ubuntu 20.04+ recommended)
- Docker Engine installed ([install guide](https://docs.docker.com/engine/install/ubuntu/))
- NVIDIA GPU + NVIDIA Container Toolkit (for RViz2 visualization)
- Wired Ethernet connection to the Unitree Go2 EDU robot

### 1.2 Host Preparation

Allow Docker to access the display and install the NVIDIA Container Toolkit:

```bash
# Allow GUI rendering
xhost +local:root

# Install NVIDIA Container Toolkit
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### 1.3 Host Network Setup

Before starting the container, configure the wired Ethernet connection to the robot **on the host**:

```bash
# Replace enx0c3796dc8be5 with YOUR adapter name (find with: ip a)
sudo ip link set enx0c3796dc8be5 up
sudo ip addr flush dev enx0c3796dc8be5
sudo ip addr add 192.168.123.100/24 dev enx0c3796dc8be5
sudo ip route add 192.168.123.0/24 dev enx0c3796dc8be5

# Test connection
ping -c 4 192.168.123.161
```

> ⚠️ **You must run this BEFORE starting any ROS 2 nodes.** If you see `No route to host` or `does not match an available interface`, the network is not configured.

### 1.4 Pull the Pre-built Image

```bash
docker pull mirandametri/unitree-go2-slam:yaml-tuned-v1
```

> This downloads ~8 GB (compressed). The full image is ~24 GB uncompressed and includes: ROS 2 Foxy, CycloneDDS, go2_robot_sdk, slam_toolbox, pointcloud_to_laserscan, wasmtime C API, voxel_decoder_cpp, PCL, and all Python/C++ dependencies.

### 1.5 Create the Docker Compose File

Create a folder for your project and the compose file:

```bash
mkdir -p ~/unitree_slam/workspace
cd ~/unitree_slam
nano docker-compose.yml
```

Paste the following content:

```yaml
version: '3.8'

services:
  ros2_foxy_dev:
    image: mirandametri/unitree-go2-slam:yaml-tuned-v1
    container_name: dev_unitree_go2
    network_mode: host      # Crucial for DDS/UDP traffic with the robot
    ipc: host               # Crucial for LiDAR shared memory
    pid: host
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
      - NVIDIA_DRIVER_CAPABILITIES=all
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw    # GUI rendering
      - ./workspace:/ros2_ws                # Persist maps on host
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu]
    command: tail -f /dev/null
```

Save with `Ctrl+O`, `Enter`, `Ctrl+X`.

### 1.6 Start the Container

```bash
cd ~/unitree_slam
sudo docker compose up -d
sudo docker exec -it dev_unitree_go2 bash
```

You are now inside the container with everything ready. Proceed to the **Environment Setup** section below.

---

## 2. Building from Source (Alternative)

If you prefer to build everything from scratch instead of using the pre-built image, use `osrf/ros:foxy-desktop` as the base image in the `docker-compose.yml`:

```yaml
    image: osrf/ros:foxy-desktop    # instead of mirandametri/unitree-go2-slam:yaml-tuned-v1
```

Then, inside the container, install dependencies and compile:

```bash
apt-get update
apt-get install -y python3-pip python3-colcon-common-extensions git python3-rosdep
apt-get install -y ros-foxy-rosidl-generator-dds-idl ros-foxy-fastrtps ros-foxy-rmw-fastrtps-cpp
apt-get install -y ros-foxy-pcl-ros ros-foxy-pcl-conversions ros-foxy-sensor-msgs-py ros-foxy-pointcloud-to-laserscan ros-foxy-slam-toolbox

cd /go2_webrtc_ws/src
git clone https://github.com/jmetrimiranda/Mapping_SLAM_Unitree_Go2_EDU.git go2_ros2_sdk
cd go2_ros2_sdk
git submodule update --init --recursive
pip3 install -r requirements.txt

cd /go2_webrtc_ws
source /opt/ros/foxy/setup.bash
colcon build
```

---

## 3. Environment Setup Script (`env_ros2.sh`)

Every terminal that runs a ROS 2 node **must** have the correct environment variables configured. Instead of typing 8 lines of exports every time, create a reusable script.

### Creating the script

Inside the container, run:

```bash
nano /env_ros2.sh
```

Paste the following content:

```bash
#!/bin/bash
# =============================================================
# env_ros2.sh — Source this file in EVERY terminal before
# running any ROS 2 command (nodes, topic list, rviz2, etc.)
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
# Domain 42 isolates our SLAM pipeline from the robot's internal traffic (domain 0).
export ROS_DOMAIN_ID=42

# --- DDS Implementation ---
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# --- CycloneDDS Configuration ---
# Auto-detect network interface: if the Ethernet cable to the robot is connected,
# bind CycloneDDS to that specific interface. Otherwise, leave it unset (local only).
# Replace enx0c3796dc8be5 with YOUR adapter name.
# Find yours with: ip a | grep "192.168.123" (on the host, before Docker)
if ip link show enx0c3796dc8be5 2>/dev/null | grep -q "state UP"; then
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>enx0c3796dc8be5</NetworkInterfaceAddress><MaxMessageSize>65500</MaxMessageSize><FragmentSize>4000</FragmentSize></General><Internal><Watermarks><WhcHigh>500kB</WhcHigh></Watermarks></Internal></Domain></CycloneDDS>"
    echo "  Interface=enx0c3796dc8be5 (cable connected)"
else
    export CYCLONEDDS_URI=""
    echo "  Interface=ANY (cable not detected — local only)"
fi

echo "ROS 2 environment configured:"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  RMW=$RMW_IMPLEMENTATION"
```

Save with `Ctrl+O`, `Enter`, `Ctrl+X`. Then make it executable:

```bash
chmod +x /env_ros2.sh
```

> ⚠️ **IMPORTANT:** Replace `enx0c3796dc8be5` with **your** network interface name. Find it on the **host** (not inside Docker) by running `ip a` and looking for the adapter with IP `192.168.123.x`.

### Usage

In every new terminal, before running any ROS 2 command:

```bash
source /env_ros2.sh
```

---

## 4. Running the SLAM Pipeline

### 4.1 C++ High-Performance Mode — 2 Terminals (Recommended)

Uses a native C++ decoder (`voxel_decoder_cpp`) that offloads LiDAR decoding from the Python driver, reducing CPU usage from ~75% to ~25% while increasing the scan rate from ~7 Hz to ~7.7 Hz.

> 💡 **When to use:** This is the recommended mode for all mapping sessions. The C++ decoder uses wasmtime to run the same `libvoxel.wasm` algorithm natively, freeing CPU for SLAM and other tasks.

#### Performance Comparison

| Metric | Python Pipeline | C++ Pipeline |
|---|---|---|
| Total CPU usage | ~75% | **~25%** |
| Driver CPU | ~75% | **~6%** |
| /scan frequency | ~7 Hz | **~7.7 Hz** |
| SLAM quality | Good | **Good** |

#### Terminal 1 — Pipeline

```bash
source /env_ros2.sh
export LD_LIBRARY_PATH=/opt/wasmtime/lib:$LD_LIBRARY_PATH

# Kill any leftover processes from previous sessions
pkill -f "go2_driver\|voxel_decoder\|pointcloud_to_laser\|slam_tool\|static_transform\|rviz2"
sleep 3

# 1. WebRTC Driver — connects to the robot, publishes raw compressed voxels
ros2 run go2_robot_sdk go2_driver_node --ros-args \
  -p conn_type:="webrtc" \
  -p robot_ip:="192.168.123.161" \
  -p enable_video:=false \
  -p decode_lidar:=false \
  -p publish_raw_voxel:=true &
sleep 8

# 2. C++ Voxel Decoder — decodes compressed voxels natively via wasmtime
ros2 run voxel_decoder_cpp voxel_decoder_node &
sleep 3

# 3. TF Publisher — defines the LiDAR's position on the robot body
ros2 run tf2_ros static_transform_publisher \
  0.289 0.0 0.08 0.0 0.0 0.0 base_link utlidar_lidar &
sleep 1

# 4. PointCloud Slicer — converts 3D point cloud into 2D laser scan
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -p target_frame:=utlidar_lidar \
  -p min_height:=0.20 \
  -p max_height:=0.60 \
  -p range_min:=0.5 \
  -p range_max:=8.0 \
  -r cloud_in:=/point_cloud2 &
sleep 1

# 5. SLAM — builds the 2D occupancy grid map
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  -p odom_frame:=odom \
  -p base_frame:=base_link \
  -p map_frame:=map \
  -p max_laser_range:=8.0 \
  -p resolution:=0.05 \
  -p minimum_travel_distance:=0.3 \
  -p minimum_travel_heading:=0.3 &
sleep 2

echo "C++ Pipeline running. Open RViz2 in Terminal 2."
wait
```

#### Terminal 2 — Visualization

Open a second terminal in the container:

```bash
docker exec -it dev_unitree_go2 bash
source /env_ros2.sh
rviz2
```

**RViz2 Setup (do this in order):**

1. Set **Fixed Frame** to `map` (top-left dropdown)
2. Click **Add** → **By topic** → `/map` → **Map** → OK (leave QoS as default: Reliable + Transient Local)
3. Click **Add** → **By display type** → **LaserScan** → OK, then:
   - Expand **Topic** → set **Reliability Policy** to **Best Effort**
   - Set Topic to `/scan`
4. (Optional) Click **Add** → **By display type** → **PointCloud2** → OK, then:
   - Expand **Topic** → set **Reliability Policy** to **Best Effort**
   - Set Topic to `/point_cloud2`
   - Set **Decay Time** to `999` to accumulate points as the robot moves
   - Set **Style** to `Points`, **Size** to `0.005` for a dense visualization

To stop the pipeline, press `Ctrl+C` in Terminal 1.

### 4.2 Python Mode — 2 Terminals (Legacy)

The original pipeline where the Python driver decodes the LiDAR internally. Uses more CPU (~75%) but requires no additional dependencies. Both pipelines coexist in the same Docker image — just change the launch parameters.

> 💡 **When to use:** Fallback if the C++ decoder has issues, or for comparison/debugging.

#### Terminal 1 — Pipeline

```bash
source /env_ros2.sh

# Kill any leftover processes from previous sessions
pkill -f "go2_driver\|voxel_decoder\|pointcloud_to_laser\|slam_tool\|static_transform\|rviz2"
sleep 3

# 1. WebRTC Driver — decodes LiDAR data internally (high CPU)
ros2 run go2_robot_sdk go2_driver_node --ros-args \
  -p conn_type:="webrtc" \
  -p robot_ip:="192.168.123.161" \
  -p enable_video:=false \
  -p decode_lidar:=true \
  -p publish_raw_voxel:=false &
sleep 8

# 2. TF Publisher
ros2 run tf2_ros static_transform_publisher \
  0.289 0.0 0.08 0.0 0.0 0.0 base_link utlidar_lidar &
sleep 1

# 3. PointCloud Slicer
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -p target_frame:=utlidar_lidar \
  -p min_height:=0.20 \
  -p max_height:=0.60 \
  -p range_min:=0.5 \
  -p range_max:=8.0 \
  -r cloud_in:=/point_cloud2 &
sleep 1

# 4. SLAM
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  -p odom_frame:=odom \
  -p base_frame:=base_link \
  -p map_frame:=map \
  -p max_laser_range:=8.0 \
  -p resolution:=0.05 \
  -p minimum_travel_distance:=0.3 \
  -p minimum_travel_heading:=0.3 &
sleep 2

echo "Python Pipeline running. Open RViz2 in Terminal 2."
wait
```

#### Terminal 2 — Visualization

Same as C++ mode:

```bash
docker exec -it dev_unitree_go2 bash
source /env_ros2.sh
rviz2
```

### 4.3 Standard Mode — Multi-Terminal (5 Terminals)

Each node runs in its own terminal for better debugging and control. Identical pipeline to Quick Mode, but you can monitor each node's output individually.

> 💡 **When to use:** Debugging, parameter tuning, or when you need to restart individual nodes without killing the whole pipeline.

**Run `source /env_ros2.sh` in ALL 5 terminals before starting.** For the C++ pipeline, also run `export LD_LIBRARY_PATH=/opt/wasmtime/lib:$LD_LIBRARY_PATH` in Terminals 1 and 2.

#### Terminal 1 — WebRTC Driver

```bash
# C++ mode (recommended):
ros2 run go2_robot_sdk go2_driver_node --ros-args \
  -p conn_type:="webrtc" \
  -p robot_ip:="192.168.123.161" \
  -p enable_video:=false \
  -p decode_lidar:=false \
  -p publish_raw_voxel:=true

# Python mode (legacy):
# ros2 run go2_robot_sdk go2_driver_node --ros-args \
#   -p conn_type:="webrtc" \
#   -p robot_ip:="192.168.123.161" \
#   -p enable_video:=false \
#   -p decode_lidar:=true \
#   -p publish_raw_voxel:=false
```

#### Terminal 2 — C++ Voxel Decoder (C++ mode only)

```bash
ros2 run voxel_decoder_cpp voxel_decoder_node
```

> Skip this terminal if using Python mode (`decode_lidar:=true`).

#### Terminal 3 — TF Publisher

```bash
ros2 run tf2_ros static_transform_publisher \
  0.289 0.0 0.08 0.0 0.0 0.0 base_link utlidar_lidar
```

#### Terminal 4 — PointCloud Slicer

```bash
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -p target_frame:=utlidar_lidar \
  -p min_height:=0.20 \
  -p max_height:=0.60 \
  -p range_min:=0.5 \
  -p range_max:=8.0 \
  -r cloud_in:=/point_cloud2
```

#### Terminal 5 — SLAM

```bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  -p odom_frame:=odom \
  -p base_frame:=base_link \
  -p map_frame:=map \
  -p max_laser_range:=8.0 \
  -p resolution:=0.05 \
  -p minimum_travel_distance:=0.3 \
  -p minimum_travel_heading:=0.3
```

#### Terminal 6 — RViz2

```bash
rviz2
```

Follow the same RViz2 setup instructions from the C++ mode section above.

---

### 4.4 YAML-Tuned Mode — Industrial / Power Plant (Recommended)

> 💡 **This is the most capable pipeline.** Instead of 6 inline parameters, it loads a complete YAML configuration file with 40+ parameters. This activates the full angular scan matching correction — essential for long industrial corridors, reflective metal floors, and environments where the robot's paws slip during turns.

#### What changed from the base pipeline and why it matters

| Component | Base Pipeline (`slam_pipeline.sh`) | YAML-Tuned (`slam_pipeline_v2.sh`) | Effect |
|---|---|---|---|
| slam_toolbox config | 6 inline params via `--ros-args -p` | Full YAML (40+ params) | Activates all scan matcher tuning |
| `angle_variance_penalty` | ~0.5 (internal default — not set) | **1.0** | 2× stronger rejection of wrong yaw from odometry |
| `coarse_search_angle_offset` | Not set (narrow internal default) | **0.349 rad (±20°)** | Searches correct pose in ±20° window — corrects paw slippage on smooth floors |
| `correlation_search_space_resolution` | Not set | **0.01 m (1 cm)** | 1 cm precision in pose search vs coarser default |
| `use_response_expansion` | Not set | **true** | Expands search when few features available (smooth metal floor) |
| `target_frame` (slicer) | `utlidar_lidar` | **`base_link`** | Scan plane stays horizontal even when robot body sways during turns |
| `min_height` / `max_height` | 0.5 / 0.8 m (from LiDAR) | **-0.1 / 0.4 m** (from body center) | Stable height reference — doesn't tilt with sensor |
| `range_max` | 8.0 m | **12.0 m** | More wall features for angular correction |
| `use_inf` | Not set (false) | **true** | Out-of-range → `inf` not `0` (no false obstacles at scan origin) |
| `minimum_time_interval` | 0.5 s (filtered many scans) | **0.0** | All scans processed — critical at ~4–5 Hz Python scan rate |

#### Quick start — Python mode (default, recommended for power plant sessions)

```bash
# 1. Ensure network is connected
./on_network.sh

# 2. Start the YAML-tuned pipeline
docker exec -it dev_unitree_go2 bash -c \
  "source /env_ros2.sh && bash /go2_webrtc_ws/slam_pipeline_v2.sh"

# 3. Open RViz2 in a second terminal
docker exec -it dev_unitree_go2 bash -c \
  "source /env_ros2.sh && rviz2"
```

#### Quick start — C++ mode (lower CPU, recommended for long sessions)

```bash
docker exec -it dev_unitree_go2 bash -c \
  "source /env_ros2.sh && bash /go2_webrtc_ws/slam_pipeline_v2.sh cpp"
```

#### Verify the pipeline is healthy

```bash
# Scan frequency: ~4-5 Hz (Python) or ~7 Hz (C++)
docker exec dev_unitree_go2 bash -c \
  "source /env_ros2.sh && ros2 topic hz /scan"

# Map being published every ~5 seconds
docker exec dev_unitree_go2 bash -c \
  "source /env_ros2.sh && ros2 topic hz /map"

# LiDAR TF — must show Translation z=0.080
docker exec dev_unitree_go2 bash -c \
  "source /env_ros2.sh && ros2 run tf2_ros tf2_echo base_link utlidar_lidar"

# All expected topics present
docker exec dev_unitree_go2 bash -c \
  "source /env_ros2.sh && ros2 topic list"
# Expected: /map, /scan, /point_cloud2, /odom, /imu, /tf, /tf_static
```

#### Confirmed working indicators (from pipeline log)

When the pipeline starts correctly, these lines appear in the log:

```
[INFO] [go2_driver_node]: Robot 0 validated and ready       ← WebRTC connected
[INFO] [slam_toolbox]: Node using stack size 40000000        ← YAML was read (stack_size_to_use param)
[INFO] [slam_toolbox]: Using solver plugin solver_plugins::CeresSolver   ← Ceres confirmed
[INFO] [slam_toolbox]: CeresSolver: Using SCHUR_JACOBI preconditioner.  ← YAML solver params active
[INFO] [pointcloud_to_laserscan]: Got a subscriber to laserscan...      ← Slicer receiving data
Registering sensor: [Custom Described Lidar]                            ← LiDAR registered
```

#### RViz2 setup

Same as other modes:

1. Set **Fixed Frame** → `map`
2. Add `/map` → Map → keep default QoS (Reliable + Transient Local)
3. Add `/scan` → LaserScan → set **Reliability Policy** to **Best Effort**
4. (Optional) Add `/point_cloud2` → PointCloud2 → **Best Effort**, Decay Time: 999

#### Adjusting the YAML for your specific environment

The YAML lives at `/go2_webrtc_ws/mapper_params_online_async.yaml` inside the container. Edit it with:

```bash
docker exec -it dev_unitree_go2 nano /go2_webrtc_ws/mapper_params_online_async.yaml
```

**After any change, restart the pipeline** — the YAML is read only at startup:

```bash
# Ctrl+C the running pipeline, then restart:
docker exec -it dev_unitree_go2 bash -c \
  "source /env_ros2.sh && bash /go2_webrtc_ws/slam_pipeline_v2.sh"
```

See [Section 13](#13-yaml-parameter-reference--advanced-tuning) for the complete parameter reference with explanations tailored for industrial environments.

---

## 5. Camera / Video Stream

The Go2 EDU has a front-facing camera accessible via the same WebRTC connection used for LiDAR and odometry. This section explains how to enable the video stream and visualize it alongside the SLAM pipeline.

### 5.1 Enabling the Camera

Set `enable_video:=true` on the driver node. This parameter activates the WebRTC video channel from the robot.

```bash
ros2 run go2_robot_sdk go2_driver_node --ros-args \
  -p conn_type:="webrtc" \
  -p robot_ip:="192.168.123.161" \
  -p enable_video:=true \
  -p decode_lidar:=false \
  -p publish_raw_voxel:=true
```

> ⚠️ **Use with the C++ pipeline** (`decode_lidar:=false`). The Python pipeline already consumes ~75% CPU; adding video will overload the system. The C++ pipeline uses only ~25% CPU, leaving headroom for video processing.

### 5.2 Published Topics

When `enable_video:=true`, the driver publishes two additional topics:

| Topic | Type | QoS | Description |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | Best Effort | Raw camera frames |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Best Effort | Calibration metadata (intrinsics, distortion) |

Both use **Best Effort** QoS (the driver prioritizes speed over guaranteed delivery).

### 5.3 RViz2 Camera Setup

In RViz2, add the camera display:

1. Click **Add** → **By display type** → **Image** → OK
2. Set **Topic** to `/camera/image_raw`
3. Expand **Topic** → set **Reliability Policy** to **Best Effort**

> ⚠️ If you leave the Reliability Policy at the default (Reliable), RViz2 will show **"No Image"**. This happens because the ROS 2 QoS system does not deliver messages when the subscriber is stricter than the publisher — a Reliable subscriber cannot receive from a Best Effort publisher.

### 5.4 One-Command Launch Script

A convenience script is provided to launch the full pipeline (SLAM + video) in a single command:

```bash
source /env_ros2.sh
export LD_LIBRARY_PATH=/opt/wasmtime/lib:$LD_LIBRARY_PATH
/go2_webrtc_ws/slam_pipeline.sh cpp video
```

Arguments:
- `cpp` — uses the C++ voxel decoder (recommended)
- `video` — enables the camera stream (`enable_video:=true`)

To launch without video: `/go2_webrtc_ws/slam_pipeline.sh cpp`

---

## 6. LiDAR L1 Tuning Guide

The Unitree Go2 EDU uses the **L1 LiDAR**, mounted **under the robot's chin** (front-bottom), approximately **8cm from the ground**. The LiDAR emits rays in all directions (up, down, and around) producing a full 3D point cloud. The `pointcloud_to_laserscan` node then filters this cloud, keeping only points within a specific height band to create the 2D scan used by SLAM.

### 6.1 How the 2D Scan Works

The LiDAR produces a 3D point cloud covering everything around the robot. The `pointcloud_to_laserscan` node slices a horizontal band from this cloud to create a 2D scan for SLAM:

![LiDAR Side View — Height Parameters](docs/side_view.png)
![LiDAR Top View — Horizontal Parameters](docs/top_view.png)

**Vertical parameters** (measured from the LiDAR position when `target_frame=utlidar_lidar`, or from body center when `target_frame=base_link`):

| Parameter | What it does | Too low | Too high |
|---|---|---|---|
| `min_height` | Bottom of scan band | Ground noise, robot legs visible | Misses low obstacles (steps, cables) |
| `max_height` | Top of scan band | Misses tall objects (chairs, shelves) | Ceiling reflections, more noise |

> ⚠️ **`min_height` must always be less than `max_height`.** If inverted (e.g., min=0.8, max=0.2), no points can pass the filter and the scan will be empty.

**Horizontal parameters** (distance from robot center):

| Parameter | What it does | Too low | Too high |
|---|---|---|---|
| `range_min` | Dead zone around robot | Sees own legs/body as obstacles | Misses nearby walls in corridors |
| `range_max` | Max detection distance | Misses distant walls | Multi-path reflections, ghost points |

> ⚠️ **Note on `target_frame`:** When using `target_frame: base_link` (YAML-Tuned pipeline), height values are measured from the **robot body center** (base_link), not the LiDAR. The LiDAR is ~8 cm above base_link, so values shift by ~0.08 m compared to `utlidar_lidar` mode. See Section 6.2 for per-environment values.

### 6.2 Environment Profiles

#### ⚡ Power Plant / Industrial (Usina, Refinaria, Petroquímica)

Long straight corridors, metal equipment, reflective floors, pipes at various heights, large open areas between structures. Typically floors are smooth painted concrete or metal grating — both cause odometry slippage during turns. Use the **YAML-Tuned pipeline** (Section 4.4) for all power plant sessions.

```bash
# slam_pipeline_v2.sh — slicer parameters (target_frame: base_link)
target_frame: base_link
min_height: -0.1      # 10cm below body center = captures obstacles at knee/pipe height
max_height:  0.5      # 50cm above body center = captures pipes, valve handles, barriers
range_min:   0.3      # 30cm dead zone — minimal blind spot for tight passages
range_max:   12.0     # 12m — good range without picking up reflective floor noise

# YAML parameters optimized for power plant:
resolution:              0.05    # 5cm/cell — sufficient detail for corridors and equipment
minimum_travel_distance: 0.2     # 20cm between nodes — dense map in long corridors
minimum_travel_heading:  0.5     # ~28° before a turn adds a new node
max_laser_range:         12.0    # must match range_max
do_loop_closing:         true    # essential — power plants have ring corridors around tanks
loop_search_maximum_distance: 5.0  # increased for large ring-shaped corridors
angle_variance_penalty:        1.0    # strong angular correction (see Section 13.5)
coarse_search_angle_offset:    0.349  # ±20° search — handles slippage on smooth floors
use_response_expansion:        true   # recovers when floor is too smooth for features
```

**Power plant specific notes:**
- Metal gratings, pipes, and tanks cause point cloud noise → keep `range_min ≥ 0.3`
- If corridors are wider than 8m (between tanks), increase `loop_search_maximum_distance` to `8.0`
- For ring corridors (walking around a tank), loop closure at return will snap the map — keep `do_loop_closing: true`
- For very long corridors (>50m): use [Segmented Mapping](#12-segmented-mapping-large-environments)
- Robot mode: use **Normal** (not Classic) for smoother gait — less body sway = more stable scans

#### 🅿️ Parking Garage / Underground Structure

```bash
target_frame: base_link
min_height: -0.1       # Reflective floor at ~-0.35m from base_link — safely excluded
max_height:  0.4       # Captures pillars and walls
range_min:   0.3
range_max:   12.0      # Floor reflections don't enter scan because they're below min_height
```

#### 🏭 Large Industrial (factory, warehouse)

Open areas, tall equipment (conveyors, tanks, machinery), long corridors.

```bash
# PointCloud Slicer
-p min_height:=0.20 -p max_height:=2.0 -p range_min:=0.5 -p range_max:=10.0

# SLAM
-p max_laser_range:=10.0 -p resolution:=0.10 -p minimum_travel_distance:=0.5 -p minimum_travel_heading:=0.5
```

- `max_height: 2.0` — Captures conveyors, tanks, tall machinery
- `resolution: 0.10` — 10cm/pixel, clean maps for large areas
- `minimum_travel_distance: 0.5` — Less frequent updates, cleaner result

#### 🏢 Medium Room (office, lab, empty room)

Regular walls, some furniture, moderate distances.

```bash
# PointCloud Slicer
-p min_height:=0.20 -p max_height:=0.80 -p range_min:=0.5 -p range_max:=8.0

# SLAM
-p max_laser_range:=8.0 -p resolution:=0.05 -p minimum_travel_distance:=0.3 -p minimum_travel_heading:=0.3
```

- `max_height: 0.80` — Captures desks, shelves, door frames
- `resolution: 0.05` — 5cm/pixel, good balance of detail and noise

#### 🏠 Small Cluttered Space (bedroom, storage room, workshop)

Many objects close together, narrow passages, lots of furniture at various heights.

```bash
# PointCloud Slicer
-p min_height:=0.25 -p max_height:=0.50 -p range_min:=0.3 -p range_max:=5.0

# SLAM
-p max_laser_range:=5.0 -p resolution:=0.03 -p minimum_travel_distance:=0.2 -p minimum_travel_heading:=0.2
```

- `min_height: 0.25` — Filters shoes, cables, floor clutter
- `range_min: 0.3` — Detects furniture in tight spaces
- `resolution: 0.03` — 3cm/pixel, fine detail for small areas

### 6.3 Practical LiDAR Test — Object Detection

Follow this procedure to verify your parameters are correctly tuned:

**1. Place a test object** (chair, box, trash can) **1.5 meters** in front of the robot.

**2. Start the pipeline** and open RViz2 with both LaserScan and PointCloud2 displays.

**3. Check the LaserScan display.** The object should appear as a cluster of points at ~1.5m. If not:

| Problem | Cause | Fix |
|---|---|---|
| Object not visible | Below `min_height` | Decrease `min_height` |
| Object not visible | Above `max_height` | Increase `max_height` |
| Object not visible | Inside `range_min` dead zone | Decrease `range_min` |
| Object partially visible | Only part is in the scan band | Widen `min_height`/`max_height` range |

**4. Walk the robot around the object.** The `/map` should draw the object's outline. With PointCloud2 `Decay Time = 999`, you'll see the 3D shape accumulating.

**5. Fine-tune without restarting everything** — kill and relaunch only the slicer:

```bash
pkill -f pointcloud_to_laser
sleep 1
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -p target_frame:=utlidar_lidar \
  -p min_height:=0.15 \
  -p max_height:=1.0 \
  -p range_min:=0.3 \
  -p range_max:=8.0 \
  -r cloud_in:=/point_cloud2 &
```

### 6.4 Diagnostic Commands

Run in a new terminal with `source /env_ros2.sh`:

```bash
# Check scan frequency (Python: ~4-5 Hz | C++: ~7.7 Hz)
ros2 topic hz /point_cloud2

# Check current scan parameters
ros2 topic echo /scan | grep -A2 "range_min"

# Check points per frame
ros2 topic echo /point_cloud2 | grep "width"

# Check CPU usage per node
top -bn1 | grep -E "voxel_deco|go2_driver|pointcloud|slam"

# List all active topics
ros2 topic list

# Check QoS compatibility
ros2 topic info /point_cloud2 --verbose
```

### 6.5 Common Noise Sources and Fixes

| Noise Source | Symptom | Fix |
|---|---|---|
| Robot's own legs | Points at ~30cm in all directions | Increase `range_min` to `0.5` |
| Ground reflections | Random points near robot at ground level | Increase `min_height` to `0.25` |
| Reflective metal floor | Inconsistent scans during turns | Switch to `target_frame: base_link` (YAML-Tuned pipeline) |
| Metal gratings (usina) | High scan noise near floor | Increase `min_height`, use `range_min: 0.3` |
| Ceiling reflections | Points appearing above real obstacles | Decrease `max_height` |
| Glass / mirrors | Ghost walls, duplicated room geometry | No LiDAR fix — avoid glass surfaces |
| Multi-path reflections | Scattered random points in small rooms | Decrease `range_max` |
| Moving people | Temporary blobs appearing in the map | Increase `minimum_travel_distance` |
| Shiny metal surfaces | Sporadic false points at random distances | Increase `min_height`, lower `range_max` |
| Rotational drift in curves | Corridors misaligned after turns | Use YAML-Tuned pipeline (Section 4.4) |

### 6.6 Quick Tuning Reference

A single-table reference for the most common tuning scenarios. When you see a symptom in RViz2, find it below and adjust the corresponding parameter.

#### "I see… I adjust…" — Tuning Cheat Sheet

| Symptom | Action | Parameter | Useful Range | Physical Limit | Unit |
|---|---|---|---|---|---|
| Ground noise / false obstacles at floor level | Raise the bottom of the scan band | `min_height` | 0.15 – 0.30 | ≥ −0.08 (ground) | m |
| Missing low obstacles (steps, cables, shoes) | Lower the bottom of the scan band | `min_height` | 0.10 – 0.20 | ≥ −0.08 (ground) | m |
| Missing tall objects (shelves, machinery) | Raise the top of the scan band | `max_height` | 0.80 – 2.00 | No hard limit | m |
| Ceiling reflections / noise above obstacles | Lower the top of the scan band | `max_height` | 0.50 – 0.80 | No hard limit | m |
| Robot legs/body appear as obstacles | Increase the dead zone radius | `range_min` | 0.3 – 0.8 | ≥ 0.0 | m |
| Missing nearby walls in tight corridors | Decrease the dead zone radius | `range_min` | 0.2 – 0.4 | ≥ 0.0 | m |
| Ghost points / multi-path reflections in small rooms | Reduce the maximum detection distance | `range_max` | 5.0 – 8.0 | L1 effective: ~8 m | m |
| Missing distant walls in large spaces | Increase the maximum detection distance | `range_max` | 8.0 – 12.0 | L1 effective: ~12 m | m |
| Map too noisy / too many scan insertions | Increase travel required between scans | `minimum_travel_distance` | 0.3 – 0.5 | — | m |
| Map missing detail in tight areas | Decrease travel required between scans | `minimum_travel_distance` | 0.1 – 0.2 | — | m |
| Map too coarse / blocky cells | Increase map resolution (smaller cell size) | `resolution` | 0.03 – 0.05 | — | m/cell |
| Map too noisy / computation too high | Decrease map resolution (larger cell size) | `resolution` | 0.05 – 0.10 | — | m/cell |
| Rotational drift after turns | Enable YAML-Tuned pipeline | See Section 4.4 | — | — | — |

#### Mandatory Rules

These constraints are enforced by the physics of the system. Violating them produces empty scans or invalid data:

1. **`min_height` < `max_height`** — Always. If inverted (e.g., min=0.8, max=0.2), no points pass the filter and the scan is empty.

2. **`min_height` ≥ −0.08 m** — The LiDAR is mounted 8cm above the ground. A `min_height` of −0.08 means "at ground level." Values below −0.08 attempt to scan underground, which only produces noise. In practice, keep `min_height` ≥ 0.10 to avoid ground reflections. *(This constraint applies when `target_frame: utlidar_lidar`. With `target_frame: base_link`, the equivalent lower limit is `min_height ≥ -0.16 m`.)*

3. **`range_min` < `range_max`** — Always. The dead zone must be smaller than the detection distance.

4. **`range_max` ≤ 12.0 m** — The L1 LiDAR has an effective range of approximately 8–12 meters depending on surface reflectivity. Setting `range_max` beyond 12m adds noise without meaningful detections.

5. **`max_laser_range` ≥ `range_max`** — The SLAM node's `max_laser_range` should match or exceed the slicer's `range_max`. Otherwise, SLAM discards valid scan data.

#### About the −0.08 m Limit

All height parameters (`min_height`, `max_height`) are measured **from the LiDAR position** when using `target_frame: utlidar_lidar`. Since the LiDAR is mounted at 8cm above the ground:

- `min_height = 0.0` → at LiDAR level (8cm above ground)
- `min_height = 0.20` → 20cm above LiDAR = 28cm above ground (default)
- `min_height = −0.08` → 8cm below LiDAR = ground level
- `min_height < −0.08` → below ground = physically impossible, only noise

When using `target_frame: base_link` (YAML-Tuned pipeline), heights are measured from the robot's center of mass (~16cm above ground). Ground level is approximately `min_height = -0.16`.

---

## 7. Architecture Overview

### 7.1 C++ High-Performance Pipeline

```
┌─────────────────────────────────────────────────────────────┐
│                    Unitree Go2 EDU                          │
│              (192.168.123.161, WebRTC)                       │
└─────────────┬───────────────────────────────────────────────┘
              │ Compressed Voxels via WebRTC
              ▼
┌─────────────────────────────────────────────────────────────┐
│  go2_driver_node (Python, decode_lidar:=false, ~6% CPU)     │
│  Does NOT decode — publishes raw compressed voxels          │
│  Publishes: /utlidar/voxel_map_compressed (Best Effort)     │
│  Also publishes: /odom, /imu, /tf, /joint_states            │
│  (With enable_video:=true: /camera/image_raw, camera_info)  │
└─────────────┬───────────────────────────────────────────────┘
              │ /utlidar/voxel_map_compressed (~7.7 Hz)
              ▼
┌─────────────────────────────────────────────────────────────┐
│  voxel_decoder_node (C++, wasmtime, ~19% CPU)               │
│  Decodes libvoxel.wasm natively via wasmtime C API          │
│  Converts raw bytes → XYZ + intensity (C++ pure, no numpy)  │
│  Publishes: /point_cloud2 (Best Effort)                     │
└─────────────┬───────────────────────────────────────────────┘
              │ /point_cloud2 (PointCloud2, ~7.7 Hz)
              ▼
┌─────────────────────────────────────────────────────────────┐
│  pointcloud_to_laserscan_node                               │
│  Slices 3D cloud → 2D laser scan                            │
│  Input: /point_cloud2  Output: /scan (Best Effort)          │
└─────────────┬───────────────────────────────────────────────┘
              │ /scan (LaserScan, ~7.7 Hz)
              ▼
┌─────────────────────────────────────────────────────────────┐
│  async_slam_toolbox_node                                    │
│  Builds occupancy grid from laser scans + odometry          │
│  Config: mapper_params_online_async.yaml (YAML-Tuned mode)  │
│  Output: /map (Reliable + Transient Local)                  │
└─────────────┬───────────────────────────────────────────────┘
              │ /map (OccupancyGrid)
              ▼
┌─────────────────────────────────────────────────────────────┐
│  RViz2 (Visualization)                                      │
│  Map: Reliable | LaserScan/PointCloud2: Best Effort         │
│  Camera Image: Best Effort                                  │
└─────────────────────────────────────────────────────────────┘
```

### 7.2 Python Legacy Pipeline

```
┌─────────────────────────────────────────────────────────────┐
│                    Unitree Go2 EDU                          │
│              (192.168.123.161, WebRTC)                       │
└─────────────┬───────────────────────────────────────────────┘
              │ Compressed Voxels via WebRTC
              ▼
┌─────────────────────────────────────────────────────────────┐
│  go2_driver_node (Python, decode_lidar:=true, ~75% CPU)     │
│  Decodes voxels → publishes /point_cloud2 (Best Effort)     │
│  Also publishes: /odom, /imu, /tf, /joint_states            │
└─────────────┬───────────────────────────────────────────────┘
              │ /point_cloud2 (PointCloud2, ~4-5 Hz)
              ▼
┌─────────────────────────────────────────────────────────────┐
│  pointcloud_to_laserscan_node → slam_toolbox → /map         │
└─────────────────────────────────────────────────────────────┘
```

---

## 8. Parameter Reference

### 8.1 WebRTC Driver Parameters

| Parameter | Default | Description |
|---|---|---|
| `conn_type` | `"webrtc"` | Connection protocol. Always `"webrtc"` for Go2 EDU. |
| `robot_ip` | `"192.168.123.161"` | Robot's IP on the wired Ethernet. Default for Go2 EDU. |
| `enable_video` | `false` | Enable camera stream. Set `true` for visual SLAM or object detection. Increases CPU usage. **Recommended with C++ pipeline only** (see [Section 5](#5-camera--video-stream)). |
| `decode_lidar` | `true` | Decode compressed voxels into PointCloud2 inside the driver. Set `false` for C++ pipeline. |
| `publish_raw_voxel` | `false` | Publish raw compressed voxels on `/utlidar/voxel_map_compressed`. Set `true` for C++ pipeline. |

### 8.2 PointCloud Slicer Parameters

These control how the 3D point cloud is "sliced" into a 2D laser scan for SLAM.

| Parameter | Base default | YAML-Tuned default | Description | Tuning Guide |
|---|---|---|---|---|
| `target_frame` | `utlidar_lidar` | `base_link` | TF frame for the output scan. `base_link` = stable horizontal plane during body sway. | Change to `base_link` for industrial environments. |
| `min_height` | `0.20` | `-0.1` | Minimum height to include (m from frame origin). | **Lower = more ground noise.** |
| `max_height` | `0.60` | `0.4` | Maximum height to include (m from frame origin). | **Higher = captures taller objects** but may include ceiling reflections. |
| `range_min` | `0.5` | `0.3` | Minimum distance (m) to accept a point. | **Raise to 0.6–0.8** if you see noise from the robot's own body. |
| `range_max` | `8.0` | `12.0` | Maximum distance (m). Points beyond this are discarded. | The Go2 LiDAR has ~8–12m effective range. |
| `use_inf` | not set | `true` | Out-of-range points → `inf` instead of `0`. | Prevents false obstacles at the scan origin. |

### 8.3 SLAM Parameters

#### Inline mode (slam_pipeline.sh) — 6 parameters:

| Parameter | Value | Description |
|---|---|---|
| `odom_frame` | `odom` | Odometry frame from the robot. |
| `base_frame` | `base_link` | Robot's base frame. |
| `map_frame` | `map` | Output map frame. |
| `max_laser_range` | `8.0` | Maximum range (meters) for SLAM scan matching. |
| `resolution` | `0.05` | Map resolution in meters/pixel. 0.05 = 5cm per cell. |
| `minimum_travel_distance` | `0.2` | Minimum distance (meters) the robot must travel before a new scan is added. |
| `minimum_travel_heading` | `0.5` | Minimum rotation (radians, ~28°) before a new scan is added. |

#### YAML-Tuned mode (slam_pipeline_v2.sh) — 40+ parameters from YAML file:

See [Section 13](#13-yaml-parameter-reference--advanced-tuning) for the complete reference.

### 8.4 TF Publisher Parameters

The 6 numbers represent the LiDAR's position relative to `base_link`: `x y z roll pitch yaw`

| Value | Meaning | Go2 Default |
|---|---|---|
| `0.289` | X offset — LiDAR is 28.9cm forward of center | Measured on Go2 EDU |
| `0.0` | Y offset — centered laterally | — |
| `0.08` | Z offset — LiDAR is 8cm above the base | Measured on Go2 EDU |
| `0.0 0.0 0.0` | No rotation (roll, pitch, yaw) | LiDAR is level |

---

## 9. Docker Image Tags

All images are available on [Docker Hub](https://hub.docker.com/r/mirandametri/unitree-go2-slam/tags):

| Tag | Description | When to use |
|---|---|---|
| `yaml-tuned-v1` | **Current recommended.** Full YAML SLAM config + C++ decoder. | Industrial, power plant, parking garage |
| `cpp-decoder-v1` | Previous stable. Inline params, C++ decoder. | Backup / before YAML upgrade |
| `pre-yaml-upgrade` | Snapshot taken 2026-04-05 before migration. | Full rollback if needed |
| `python-stable` | Python-only pipeline. No C++ decoder. | Simple environments, debugging |

### Switching Between Images

Edit `docker-compose.yml` and change the `image:` line:

```yaml
# Current recommended (YAML-tuned, industrial):
image: mirandametri/unitree-go2-slam:yaml-tuned-v1

# Previous stable (inline params):
image: mirandametri/unitree-go2-slam:cpp-decoder-v1

# Full rollback:
image: mirandametri/unitree-go2-slam:pre-yaml-upgrade
```

Then recreate the container:

```bash
docker compose down
sudo docker compose up -d
sudo docker exec -it dev_unitree_go2 bash
```

> **Note:** You do NOT need to switch images to alternate between C++ and Python pipelines. Both pipelines coexist in all images — just change the launch parameters (`decode_lidar:=true/false`).

---

## 10. Saving Maps

### Save current map as image

```bash
docker exec dev_unitree_go2 bash -c "
  source /env_ros2.sh &&
  mkdir -p /ros2_ws/mapa_garagem1 &&
  ros2 run nav2_map_server map_saver_cli -f /ros2_ws/mapa_garagem1/mapa1
"
```

> **Note:** Always save to `/ros2_ws` to ensure the files are persisted on your host machine (mounted via Docker volume at `~/unitree_slam/workspace/`).

This generates two files:
- `mapa1.pgm` — The occupancy grid image
- `mapa1.yaml` — Map metadata (resolution, origin, etc.)

---

### Save SLAM state (to continue mapping later)

```bash
docker exec dev_unitree_go2 bash -c "
  source /env_ros2.sh &&
  ros2 service call /slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph \
    \"{filename: '/ros2_ws/mapa_garagem1/mapa1'}\"
"
```

This generates two additional files:
- `mapa1.posegraph` — Full pose graph (~13 MB for a garage-sized map)
- `mapa1.data` — Associated scan data (~1.4 MB)

---

### Confirm all 4 files were created

```bash
docker exec dev_unitree_go2 ls -lh /ros2_ws/mapa_garagem1/
# Expected:
# mapa1.pgm        ~55 KB
# mapa1.yaml       ~144 B
# mapa1.posegraph  ~13 MB
# mapa1.data       ~1.4 MB
```

---

### Copy map to host for inspection

```bash
docker cp dev_unitree_go2:/ros2_ws/mapa_garagem1/mapa1.pgm ~/Downloads/mapa1.pgm
eog ~/Downloads/mapa1.pgm
```

---

### Load saved map and continue mapping

Add to the SLAM node launch:

```bash
-p map_file_name:=/ros2_ws/mapa_garagem1/mapa1 \
-p map_start_at_dock:=true
```

---

### Restart container before a new session

Always restart the container after saving to clear DDS memory and avoid ghost maps:

```bash
docker restart dev_unitree_go2
```

---

### Record raw data (rosbag)

```bash
ros2 bag record /point_cloud2 /scan /odom /tf /tf_static /map -o /ros2_ws/session_01
```

Replay later:

```bash
ros2 bag play /ros2_ws/session_01
```

---

## 11. Autonomous Navigation (Nav2)

This section explains how to use a previously saved map to perform **fully autonomous navigation** — where you give the robot a goal position and it plans and executes its own path, avoiding obstacles in real time.

> ⚠️ **Prerequisite:** You must have completed a mapping session using `slam_pipeline_v2.sh` (see [Section 4.4](#44-yaml-tuned-mode--industrial--power-plant-recommended)) and saved all 4 map files using the procedure in [Section 10](#10-saving-maps) before proceeding.

### 11.1 How Nav2 Works With a Saved Map

The key difference between **mapping mode** and **navigation mode** is the role of the slam_toolbox:

| Mode | slam_toolbox role | Map |
|---|---|---|
| Mapping (`slam_pipeline_v2.sh`) | **Builds** the map from scratch | Grows as robot walks |
| Navigation (`nav_pipeline_navigation.sh`) | **Localizes** the robot in the existing map | Fixed — read from disk |

In navigation mode, slam_toolbox switches from `mode: mapping` to `mode: localization`. It loads the `.posegraph` file saved in Section 10 and uses incoming LiDAR scans to continuously compute **where the robot is** within that fixed map. Nav2 uses this position to plan paths and send velocity commands (`/cmd_vel`) to the `go2_driver_node`, which translates them into physical movement.

```
Saved map on disk (.posegraph)
         │
         ▼
slam_toolbox (localization) ←── /scan (LiDAR)
         │
         │ TF: map → odom → base_link
         ▼
Nav2 Stack
  ├── global_costmap  (full map + obstacles)
  ├── local_costmap   (nearby obstacles, rolling window)
  ├── planner_server  (computes global path)
  ├── controller_server (executes path, avoids obstacles)
  └── bt_navigator    (high-level logic, recoveries)
         │
         │ /cmd_vel
         ▼
go2_driver_node ──→ Unitree Go2 EDU (moves physically)
```

### 11.2 Prerequisites

Before running the navigation pipeline, confirm all 4 map files exist:

```bash
docker exec dev_unitree_go2 ls -lh /ros2_ws/mapa_garagem1/
# Must show:
# mapa1.pgm        ← occupancy grid image
# mapa1.yaml       ← map metadata
# mapa1.posegraph  ← SLAM pose graph (loaded by localization)
# mapa1.data       ← associated scan data
```

If any file is missing, redo the mapping session (Section 4.4) and save steps (Section 10).

Also confirm the network is active before starting (on the Alienware host, outside Docker):

```bash
./on_network.sh
ping -c 2 192.168.123.161   # must succeed
```

### 11.3 Files Involved and What Each Does

The navigation pipeline uses **three configuration files** inside the container. Understanding each one is important — if you create a new map in a different folder, all three must be updated.

| File | Location | Purpose |
|---|---|---|
| `nav_pipeline_navigation.sh` | `/go2_webrtc_ws/` | Launch script: starts all nodes in the correct order |
| `localization_params.yaml` | `/go2_webrtc_ws/` | slam_toolbox config in `localization` mode — points to the `.posegraph` file |
| `nav2_params.yaml` | `/go2_webrtc_ws/` | Nav2 stack config — velocities, costmaps, planners, map server path |

#### `nav_pipeline_navigation.sh` — the main script

```bash
#!/bin/bash
ROBOT_IP="192.168.123.161"
MAP_FILE="/ros2_ws/mapa_garagem1/mapa1"          # ← path to map (without extension)
SLAM_PARAMS="/go2_webrtc_ws/localization_params.yaml"
NAV2_PARAMS="/go2_webrtc_ws/nav2_params.yaml"
```

The script starts 5 components in order:
1. `go2_driver_node` — connects to robot, receives LiDAR, listens to `/cmd_vel`
2. TF publishers — `base_link → utlidar_lidar` and `base_link → base_footprint`
3. `pointcloud_to_laserscan` — same slicer parameters as mapping
4. `slam_toolbox` in localization mode — loads posegraph, localizes robot
5. `nav2_bringup` — full Nav2 stack (planner + controller + bt_navigator)

#### `localization_params.yaml` — slam_toolbox localization config

This file is a copy of `mapper_params_online_async.yaml` with three lines changed:

```yaml
mode: localization               # was: mapping
map_file_name: /ros2_ws/mapa_garagem1/mapa1   # path to posegraph (without extension)
map_start_at_dock: true          # robot starts at the origin of the saved map
```

All other parameters (angular correction, scan matching, loop closure) remain the same as during mapping — this ensures localization uses the same quality settings that produced the map.

#### `nav2_params.yaml` — Nav2 configuration

Key sections relevant to the Go2 EDU:

```yaml
map_server:
  ros__parameters:
    yaml_filename: "/ros2_ws/mapa_garagem1/mapa1.yaml"  # ← .yaml map file

controller_server:
  ros__parameters:
    controller_frequency: 3.0    # conservative — avoids control loop overload
    FollowPath:
      max_vel_x: 0.3             # m/s — safe for Go2 EDU indoors
      max_vel_theta: 0.5         # rad/s — gentle turns
```

### 11.4 What to Change When Using a New Map Folder

Every time you create a new map in a new folder (e.g., `/ros2_ws/mapa_usina/mapa1`), you must update **3 locations** inside the container before running navigation. This is the most common source of errors.

#### Step 1 — Update `nav_pipeline_navigation.sh`

```bash
nano /go2_webrtc_ws/nav_pipeline_navigation.sh
```

Change the `MAP_FILE` line at the top:

```bash
# Example: changing from garage map to power plant map
MAP_FILE="/ros2_ws/mapa_usina/mapa1"    # ← update this
```

Or use `sed` to do it without opening the editor:

```bash
sed -i 's|MAP_FILE=.*|MAP_FILE="/ros2_ws/mapa_usina/mapa1"|' \
  /go2_webrtc_ws/nav_pipeline_navigation.sh

# Confirm:
grep MAP_FILE /go2_webrtc_ws/nav_pipeline_navigation.sh
```

#### Step 2 — Update `localization_params.yaml`

```bash
nano /go2_webrtc_ws/localization_params.yaml
```

Change the `map_file_name` line:

```yaml
map_file_name: /ros2_ws/mapa_usina/mapa1    # ← update this (no extension)
```

Or use `sed`:

```bash
sed -i 's|map_file_name:.*|map_file_name: /ros2_ws/mapa_usina/mapa1|' \
  /go2_webrtc_ws/localization_params.yaml

# Confirm:
grep map_file_name /go2_webrtc_ws/localization_params.yaml
```

#### Step 3 — Update `nav2_params.yaml`

```bash
nano /go2_webrtc_ws/nav2_params.yaml
```

Find the `map_server` section and change `yaml_filename`:

```yaml
map_server:
  ros__parameters:
    yaml_filename: "/ros2_ws/mapa_usina/mapa1.yaml"    # ← update this (WITH .yaml extension)
```

Or use `sed`:

```bash
sed -i 's|yaml_filename:.*|yaml_filename: "/ros2_ws/mapa_usina/mapa1.yaml"|' \
  /go2_webrtc_ws/nav2_params.yaml

# Confirm:
grep yaml_filename /go2_webrtc_ws/nav2_params.yaml
```

#### Summary table — what changes per file

| File | What to change | Extension |
|---|---|---|
| `nav_pipeline_navigation.sh` | `MAP_FILE` variable | none |
| `localization_params.yaml` | `map_file_name` parameter | none |
| `nav2_params.yaml` | `yaml_filename` in `map_server` | `.yaml` |

> ⚠️ **`localization_params.yaml` and `nav_pipeline_navigation.sh` use the path WITHOUT extension.** `nav2_params.yaml` uses the path WITH `.yaml` extension. This asymmetry is a common source of errors.

### 11.5 Running the Navigation Pipeline

#### Pré-requisito — on the Alienware host (outside Docker)

```bash
docker start dev_unitree_go2
./on_network.sh
```

#### Terminal 1 — Navigation pipeline

```bash
docker exec -it dev_unitree_go2 bash -c \
  "source /env_ros2.sh && bash /go2_webrtc_ws/nav_pipeline_navigation.sh"
```

The startup takes approximately 20–30 seconds. Wait for these confirmations in the log before proceeding:

```
[go2_driver_node]: Robot 0 validated and ready        ← robot connected
[slam_toolbox]: Node using stack size 40000000         ← localization_params.yaml was read
[slam_toolbox]: Using solver plugin solver_plugins::CeresSolver  ← Ceres active
Registering sensor: [Custom Described Lidar]           ← LiDAR registered
[bt_navigator]: Creating                               ← Nav2 active
[controller_server]: Controller Server has FollowPath  ← planner active
[lifecycle_manager]: All requested nodes are active    ← Nav2 fully up
```

> ⚠️ If you see `[bt_navigator] [FATAL]: cannot open shared object file` for any plugin, that plugin is not available in this version of Nav2/Foxy. Remove the offending line from the `plugin_lib_names` list in `nav2_params.yaml` and restart.

#### Terminal 2 — RViz2

```bash
docker exec -it dev_unitree_go2 bash -c "source /env_ros2.sh && rviz2"
```

### 11.6 RViz2 Setup for Navigation

Configure RViz2 in this exact order after it opens:

1. **Fixed Frame** → set to `map` (top-left dropdown)
2. **Add** → By topic → `/map` → **Map** → OK
   - Leave QoS as default: Reliable + Transient Local
   - The map may take 10–20 seconds to appear (posegraph loading)
3. **Add** → By display type → **LaserScan** → OK
   - Topic: `/scan`
   - Reliability Policy: **Best Effort**
4. **Add** → By display type → **RobotModel** → OK
   - Shows the physical model of the Go2 in the map
5. (Optional) **Add** → By display type → **Path** → OK
   - Topic: `/plan`
   - Shows the planned path when a goal is set

> ⚠️ The map will appear as it was when you saved it — static, not updating. This is correct. Only the robot's position (laser scan overlay) moves as the robot moves.

### 11.7 Performing Autonomous Navigation

#### Step 1 — Set initial pose (2D Pose Estimate)

The robot does not automatically know where it is in the map. You must tell it.

1. In the RViz2 toolbar, click **"2D Pose Estimate"**
2. Click on the map at the exact location where the robot **physically is right now**
3. Hold and drag the mouse in the direction the robot **is currently facing**
4. Release

You will see the laser scan (red/orange dots) overlay on the map. **The scan must align with the walls.** If the dots are floating in the middle of a room or far from the walls, the pose estimate is wrong — repeat this step.

> 💡 **Tip:** Position the robot at an easily recognizable feature in the map (a corner, a doorway, a pillar) before setting the pose estimate. This makes alignment much easier.

#### Step 2 — Verify alignment

After setting the pose estimate, check:

```bash
# In a third terminal — verify scan is being received
docker exec dev_unitree_go2 bash -c \
  "source /env_ros2.sh && ros2 topic hz /scan"
# Must show ~4-5 Hz (Python mode)

# Verify localization is publishing the TF
docker exec dev_unitree_go2 bash -c \
  "source /env_ros2.sh && ros2 run tf2_ros tf2_echo map base_link"
# Must show a transform — if it hangs, localization is not working
```

#### Step 3 — Set navigation goal (Nav2 Goal)

Once the laser scan aligns with the walls:

1. In the RViz2 toolbar, click **"Nav2 Goal"**
2. Click on a **white area** of the map (free space) — not on a wall (black) or unknown area (grey)
3. Hold and drag to set the robot's desired final orientation
4. Release

The robot will:
- Plan a global path (shown as a line in RViz2 if the Path display is added)
- Begin walking toward the goal
- Avoid obstacles detected by the LiDAR in real time

> ⚠️ **Safety:** Stay close to the robot during the first navigation attempt. Any of these behaviors indicate a problem that requires intervention: spinning in circles continuously, walking directly into a wall, or not moving at all. In all cases, the most likely cause is an incorrect 2D Pose Estimate — repeat Step 1 with more precision.

#### Step 4 — Reaching the goal

When the robot reaches the goal position and orientation, Nav2 will stop the robot and report success. You can then set a new goal.

### 11.8 Verifying the Pipeline is Healthy

Run these checks in a new terminal (`docker exec -it dev_unitree_go2 bash`, then `source /env_ros2.sh`):

```bash
# Scan arriving?
ros2 topic hz /scan
# Expected: ~4-5 Hz (Python) or ~7 Hz (C++)

# Map published?
ros2 topic hz /map
# Expected: ~0.2 Hz (once every 5 seconds)

# Nav2 receiving velocity commands?
ros2 topic hz /cmd_vel
# Expected: 0 when idle, ~3 Hz when navigating

# All Nav2 nodes active?
ros2 node list | grep -E "controller|planner|bt_nav|slam_toolbox"
# Expected: all 4 nodes listed

# Localization working (TF from map to base_link)?
ros2 run tf2_ros tf2_echo map base_link
# Expected: shows a transform. If it hangs → localization failed to load map.
```

### 11.9 Confirmed Working Indicators

When the full pipeline starts correctly, these lines appear in sequence in the log:

```
[go2_driver_node]: Robot 0 validated and ready         ← robot connected via WebRTC
[slam_toolbox]: Node using stack size 40000000          ← localization_params.yaml loaded
[slam_toolbox]: Using solver plugin solver_plugins::CeresSolver
[slam_toolbox]: CeresSolver: Using SCHUR_JACOBI preconditioner.
Registering sensor: [Custom Described Lidar]            ← LiDAR registered in localization
[controller_server]: Controller frequency set to 3.0Hz  ← nav2_params.yaml read
[planner_server]: Created global planner plugin GridBased
[bt_navigator]: Creating                                ← BT navigator active
[lifecycle_manager]: All requested nodes are active and running  ← Nav2 fully up
[global_costmap]: StaticLayer: Resizing costmap to NNN X MMM   ← map loaded into costmap
```

### 11.10 Troubleshooting Nav2

#### Robot does not move after Nav2 Goal

1. Check that `[lifecycle_manager]: All requested nodes are active` appeared in the log
2. Check that the 2D Pose Estimate was set and the scan aligns with walls
3. Check `/cmd_vel` is being published: `ros2 topic hz /cmd_vel` (should increase when goal is set)
4. Check that the goal was placed in **white** (free) space, not grey (unknown) or black (wall)

#### Map does not appear in RViz2 after 30 seconds

The posegraph is taking too long to deserialize, or the path is wrong. Verify:

```bash
docker exec dev_unitree_go2 bash -c \
  "source /env_ros2.sh && ros2 topic echo /map --once"
# If this hangs → map was never published → check slam_toolbox log for errors
```

Also verify the posegraph path is correct:

```bash
grep map_file_name /go2_webrtc_ws/localization_params.yaml
# Must point to a file that actually exists:
docker exec dev_unitree_go2 ls /ros2_ws/mapa_garagem1/mapa1.posegraph
```

#### `bt_navigator [FATAL]: cannot open shared object file`

A plugin listed in `nav2_params.yaml` does not exist in this version of Nav2/Foxy. Remove the offending line from `plugin_lib_names` in `nav2_params.yaml`:

```bash
nano /go2_webrtc_ws/nav2_params.yaml
# Find plugin_lib_names under bt_navigator and remove the line with the missing plugin
```

Restart the pipeline after editing.

#### Laser scan does not align with walls after 2D Pose Estimate

- The robot's physical position or orientation differs too much from what was clicked
- Try again: place the robot at a clear landmark (corner, pillar) and be precise
- If the map itself is distorted (common in long corridors), navigation may be unreliable — consider remapping

#### Robot spins in circles or tries to walk through walls

Both symptoms indicate incorrect initial pose. Repeat the **2D Pose Estimate** step with more precision. All Nav2 fault behaviors — spinning, wall collisions, no movement — trace back to either (1) wrong initial pose, (2) distorted map, or (3) control loop overload (reduce `controller_frequency` to `2.0` in `nav2_params.yaml`).

#### `map_file_name` or `yaml_filename` errors in log

You changed the map folder but forgot to update one of the three files. Check all three:

```bash
grep MAP_FILE /go2_webrtc_ws/nav_pipeline_navigation.sh
grep map_file_name /go2_webrtc_ws/localization_params.yaml
grep yaml_filename /go2_webrtc_ws/nav2_params.yaml
```

All three must point to the same map, consistently (see [Section 11.4](#114-what-to-change-when-using-a-new-map-folder)).

---

## 12. Segmented Mapping (Large Environments)

For large environments like parking garages, long corridors, power plants, or multi-floor buildings, it is often impractical to map everything in a single continuous run. The robot may need to be physically repositioned (picked up and turned) between straight-line segments. This section describes a **segmented mapping workflow** that splits the mapping session into manageable segments while building a single accumulated map.

### 12.1 How Segmented Mapping Works

The `slam_toolbox` serialization saves the **entire accumulated map** at the time of the call — not just the new portion. This is the key concept:

```
Segment 1: Walk corridor A → save as seg1
            seg1 contains: [corridor A]

Segment 2: Load seg1 → walk corridor B → save as seg2
            seg2 contains: [corridor A + corridor B]

Segment 3: Load seg2 → walk corridor C → save as seg3
            seg3 contains: [corridor A + corridor B + corridor C]

...and so on.
```

Each saved file is a **complete snapshot** of everything mapped so far. You always load only the **last saved segment** because it already contains all previous data. There is no need to "merge" files.

> 💡 **Between segments:** Stop the pipeline (`Ctrl+C`), physically reposition the robot (pick it up, turn it to face the next corridor), then restart the pipeline loading the previous segment. The SLAM node will resume from where the map left off.

### 12.2 Scripts Overview

Two scripts handle the segmented workflow. Both live in `/go2_webrtc_ws/` and follow the same structure as `slam_pipeline.sh` — configurable variables at the top, same node launch pattern.

| Script | Purpose |
|---|---|
| `slam_pipeline_segments.sh` | Starts the full pipeline for a given segment number. If segment=1, starts from scratch. If segment>1, loads the previous segment's map and continues. |
| `salvar_segmento.sh` | Saves the current SLAM state as the specified segment number. |

### 12.3 `slam_pipeline_segments.sh`

Create the script inside the container:

```bash
nano /go2_webrtc_ws/slam_pipeline_segments.sh
```

Paste the following content:

```bash
#!/bin/bash
# ============================================================
# SLAM Pipeline — Segmented Mapping
# Uso:
#   bash /go2_webrtc_ws/slam_pipeline_segments.sh 1          → Segment 1 (from scratch)
#   bash /go2_webrtc_ws/slam_pipeline_segments.sh 2          → Segment 2 (loads seg1)
#   bash /go2_webrtc_ws/slam_pipeline_segments.sh 3 cpp      → Segment 3 (loads seg2, C++ mode)
#   bash /go2_webrtc_ws/slam_pipeline_segments.sh 4 python   → Segment 4 (loads seg3, Python mode)
# ============================================================

SEG=${1:-""}
MODE=${2:-python}
ROBOT_IP="192.168.123.161"

# --- Directory where segment files are stored ---
SEG_DIR="/ros2_ws/garagem_segmentos"

# --- PointCloud Slicer Parameters ---
MIN_HEIGHT=0.5
MAX_HEIGHT=0.8
RANGE_MIN=0.5
RANGE_MAX=8.0

# --- SLAM Parameters ---
MAX_LASER_RANGE=8.0
RESOLUTION=0.05
TRAVEL_DIST=0.2
TRAVEL_HEADING=0.5
LOOP_CLOSING=true

# ============================================================
# Validation
# ============================================================

if [ -z "$SEG" ]; then
    echo "Uso: bash slam_pipeline_segments.sh SEGMENT_NUMBER [cpp|python]"
    echo ""
    echo "  SEGMENT_NUMBER  Segment to map (1 = from scratch, 2+ = loads previous)"
    echo "  MODE            cpp or python (default: python)"
    echo ""
    echo "Examples:"
    echo "  bash slam_pipeline_segments.sh 1          # Start fresh, Python mode"
    echo "  bash slam_pipeline_segments.sh 2 cpp      # Load seg1, continue with C++ mode"
    echo "  bash slam_pipeline_segments.sh 3           # Load seg2, continue with Python mode"
    exit 1
fi

# Create output directory if it doesn't exist
mkdir -p "$SEG_DIR"

# If segment > 1, verify that the previous segment file exists
if [ "$SEG" -gt 1 ]; then
    PREV=$((SEG - 1))
    if [ ! -f "${SEG_DIR}/seg${PREV}.posegraph" ]; then
        echo "ERROR: ${SEG_DIR}/seg${PREV}.posegraph not found!"
        echo "Save segment ${PREV} before starting segment ${SEG}."
        echo ""
        echo "To save:  bash /go2_webrtc_ws/salvar_segmento.sh ${PREV}"
        exit 1
    fi
fi

echo "================================================"
echo "SLAM Pipeline — Segment: $SEG | Mode: $MODE"
echo "  Slicer: min_h=$MIN_HEIGHT max_h=$MAX_HEIGHT range=[$RANGE_MIN, $RANGE_MAX]"
echo "  SLAM:   res=$RESOLUTION travel_dist=$TRAVEL_DIST loop_closing=$LOOP_CLOSING"
echo "  Output: $SEG_DIR"
echo "================================================"

# ============================================================
# Kill leftover processes
# ============================================================

pkill -f "go2_driver\|voxel_decoder\|pointcloud_to_laser\|slam_tool\|static_transform"
sleep 2

# ============================================================
# 1. WebRTC Driver
# ============================================================

if [ "$MODE" = "cpp" ]; then
    echo "[1/5] Starting driver (C++ mode, decode_lidar=false)..."
    export LD_LIBRARY_PATH=/opt/wasmtime/lib:$LD_LIBRARY_PATH
    ros2 run go2_robot_sdk go2_driver_node --ros-args \
        -p conn_type:="webrtc" -p robot_ip:="$ROBOT_IP" \
        -p enable_video:=false \
        -p decode_lidar:=false -p publish_raw_voxel:=true &
    sleep 10

    echo "[2/5] Starting C++ voxel decoder..."
    ros2 run voxel_decoder_cpp voxel_decoder_node &
    sleep 5
else
    echo "[1/5] Starting driver (Python mode, decode_lidar=true)..."
    ros2 run go2_robot_sdk go2_driver_node --ros-args \
        -p conn_type:="webrtc" -p robot_ip:="$ROBOT_IP" \
        -p enable_video:=false \
        -p decode_lidar:=true -p publish_raw_voxel:=false &
    sleep 10

    echo "[2/5] (Skipped — Python mode, no C++ decoder needed)"
fi

# ============================================================
# 3. TF Publisher
# ============================================================

echo "[3/5] Starting TF publisher..."
ros2 run tf2_ros static_transform_publisher \
    0.289 0.0 0.08 0.0 0.0 0.0 base_link utlidar_lidar &
sleep 2

# ============================================================
# 4. PointCloud Slicer
# ============================================================

echo "[4/5] Starting pointcloud_to_laserscan..."
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
    -p target_frame:=utlidar_lidar \
    -p min_height:=$MIN_HEIGHT -p max_height:=$MAX_HEIGHT \
    -p range_min:=$RANGE_MIN -p range_max:=$RANGE_MAX \
    -r cloud_in:=/point_cloud2 &
sleep 2

# ============================================================
# 5. SLAM — with or without map loading
# ============================================================

if [ "$SEG" -eq 1 ]; then
    echo "[5/5] Starting SLAM (segment 1 — from scratch)..."
    ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
        -p odom_frame:=odom -p base_frame:=base_link -p map_frame:=map \
        -p max_laser_range:=$MAX_LASER_RANGE -p resolution:=$RESOLUTION \
        -p minimum_travel_distance:=$TRAVEL_DIST \
        -p minimum_travel_heading:=$TRAVEL_HEADING \
        -p do_loop_closing:=$LOOP_CLOSING &
else
    PREV=$((SEG - 1))
    echo "[5/5] Starting SLAM (segment $SEG — loading seg${PREV} which contains segments 1-${PREV})..."
    ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
        -p odom_frame:=odom -p base_frame:=base_link -p map_frame:=map \
        -p max_laser_range:=$MAX_LASER_RANGE -p resolution:=$RESOLUTION \
        -p minimum_travel_distance:=$TRAVEL_DIST \
        -p minimum_travel_heading:=$TRAVEL_HEADING \
        -p do_loop_closing:=$LOOP_CLOSING \
        -p map_file_name:=${SEG_DIR}/seg${PREV} \
        -p map_start_at_dock:=true &
fi

sleep 3
echo "================================================"
echo "Segment $SEG running!"
echo ""
echo "Open RViz2 in another terminal:"
echo "  source /env_ros2.sh && rviz2"
echo ""
echo "When done walking, save with:"
echo "  bash /go2_webrtc_ws/salvar_segmento.sh $SEG"
echo "================================================"
wait
```

Save with `Ctrl+O`, `Enter`, `Ctrl+X`. Then make it executable:

```bash
chmod +x /go2_webrtc_ws/slam_pipeline_segments.sh
```

### 12.4 `salvar_segmento.sh`

Create the save script:

```bash
nano /go2_webrtc_ws/salvar_segmento.sh
```

Paste the following content:

```bash
#!/bin/bash
# ============================================================
# Save current SLAM state as a segment
# Uso:
#   bash /go2_webrtc_ws/salvar_segmento.sh 1    → Saves as seg1
#   bash /go2_webrtc_ws/salvar_segmento.sh 2    → Saves as seg2
# ============================================================

SEG=${1:-""}

# --- Directory where segment files are stored ---
SEG_DIR="/ros2_ws/garagem_segmentos"

if [ -z "$SEG" ]; then
    echo "Uso: bash salvar_segmento.sh SEGMENT_NUMBER"
    echo "  Saves the current SLAM map as seg<N> in $SEG_DIR"
    exit 1
fi

source /env_ros2.sh

mkdir -p "$SEG_DIR"

echo "Saving segment $SEG to ${SEG_DIR}/seg${SEG}..."

ros2 service call /slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph \
    "{filename: '${SEG_DIR}/seg${SEG}'}"

if [ $? -eq 0 ]; then
    echo "================================================"
    echo "Segment $SEG saved!"
    echo "  Files: ${SEG_DIR}/seg${SEG}.posegraph"
    echo "         ${SEG_DIR}/seg${SEG}.data"
    echo ""
    echo "This file contains ALL map data from segments 1 through $SEG."
    echo ""
    NEXT=$((SEG + 1))
    echo "To continue mapping:"
    echo "  1. Ctrl+C the pipeline"
    echo "  2. Reposition the robot"
    echo "  3. bash /go2_webrtc_ws/slam_pipeline_segments.sh $NEXT"
    echo "================================================"
else
    echo "ERROR: Failed to save segment $SEG. Is the SLAM node running?"
fi
```

Save and make executable:

```bash
chmod +x /go2_webrtc_ws/salvar_segmento.sh
```

### 12.5 Step-by-Step Workflow

This example maps a parking garage in 6 straight-line segments. Between each segment, the robot is physically picked up and turned to face the next corridor.

> ⚠️ **Prerequisites:** Host network configured ([Section 1.3](#13-host-network-setup)), container running, `source /env_ros2.sh` in all terminals.

#### Segment 1 — First corridor (from scratch)

**Terminal 1:**
```bash
source /env_ros2.sh
bash /go2_webrtc_ws/slam_pipeline_segments.sh 1
```

**Terminal 2:**
```bash
source /env_ros2.sh && rviz2
```

Walk the robot straight through the first corridor. When done, stop the robot.

**Terminal 3:**
```bash
source /env_ros2.sh
bash /go2_webrtc_ws/salvar_segmento.sh 1
```

Press `Ctrl+C` in Terminal 1 to stop the pipeline. Pick up the robot and turn it to face the next corridor.

#### Segment 2 — Second corridor (loads seg1)

**Terminal 1:**
```bash
source /env_ros2.sh
bash /go2_webrtc_ws/slam_pipeline_segments.sh 2
```

Walk the robot straight. Stop. Save:

**Terminal 3:**
```bash
bash /go2_webrtc_ws/salvar_segmento.sh 2
```

`Ctrl+C` in Terminal 1. Reposition the robot.

#### Segments 3–6 — Repeat

Continue the same pattern:

```bash
# Terminal 1: start segment N
bash /go2_webrtc_ws/slam_pipeline_segments.sh 3

# Terminal 3: save segment N (after walking)
bash /go2_webrtc_ws/salvar_segmento.sh 3

# Ctrl+C, reposition, then:
bash /go2_webrtc_ws/slam_pipeline_segments.sh 4
bash /go2_webrtc_ws/salvar_segmento.sh 4

# ...and so on until segment 6
bash /go2_webrtc_ws/slam_pipeline_segments.sh 6
bash /go2_webrtc_ws/salvar_segmento.sh 6
```

After saving segment 6, `seg6.posegraph` contains the complete map of all 6 corridors.

#### Export the final map as an image

```bash
source /env_ros2.sh
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/garagem_segmentos/mapa_final
```

### 12.6 Customizing Parameters

All tuning parameters are defined as variables at the top of `slam_pipeline_segments.sh`. Edit the file to adjust for your environment:

```bash
nano /go2_webrtc_ws/slam_pipeline_segments.sh
```

| Variable | Default | Description |
|---|---|---|
| `SEG_DIR` | `/ros2_ws/garagem_segmentos` | Directory for segment files. Change to organize different mapping sessions. |
| `MIN_HEIGHT` | `0.5` | Bottom of the scan band (meters above LiDAR). Higher than default to avoid ground noise in parking garages. |
| `MAX_HEIGHT` | `0.8` | Top of the scan band. Captures pillars and walls at waist height. |
| `RANGE_MIN` | `0.5` | Dead zone radius. Filters robot body noise. |
| `RANGE_MAX` | `8.0` | Maximum detection distance. |
| `RESOLUTION` | `0.05` | Map resolution (5cm/cell). |
| `TRAVEL_DIST` | `0.2` | Distance between scan insertions. Lower than default for more detail in straight corridors. |
| `TRAVEL_HEADING` | `0.5` | Rotation between scan insertions. Higher than default since the robot walks straight. |
| `LOOP_CLOSING` | `true` | Enable loop closure detection. Helps when segments revisit previously mapped areas. |

> 💡 These defaults are tuned for **parking garages and large indoor corridors** (`min_height:=0.5`, `max_height:=0.8`). For other environments, refer to the [Environment Profiles](#62-environment-profiles) in the LiDAR Tuning Guide.

### 12.7 File Structure

After mapping 6 segments, the output directory looks like:

```
/ros2_ws/garagem_segmentos/
├── seg1.posegraph          # Segment 1 only
├── seg1.data
├── seg2.posegraph          # Segments 1 + 2
├── seg2.data
├── seg3.posegraph          # Segments 1 + 2 + 3
├── seg3.data
├── seg4.posegraph          # Segments 1 + 2 + 3 + 4
├── seg4.data
├── seg5.posegraph          # Segments 1 + 2 + 3 + 4 + 5
├── seg5.data
├── seg6.posegraph          # Complete map (all 6 segments)
├── seg6.data
├── mapa_final.pgm          # Exported occupancy grid image
└── mapa_final.yaml         # Map metadata
```

> **Note:** Each `segN` file is a complete snapshot. You can safely delete intermediate files (seg1–seg5) after confirming the final map is correct. Keep `seg6` as it is the complete accumulated map.

> **Note:** Since the workspace is mounted via Docker volume (`./workspace:/ros2_ws`), all files in `/ros2_ws/` are persisted on your host machine at `~/unitree_slam/workspace/`.

---

## 13. YAML Parameter Reference — Advanced Tuning

This section documents all parameters in `mapper_params_online_async.yaml`, explaining what each one does and why it matters for industrial environments. The YAML-Tuned pipeline (Section 4.4) uses this file instead of the 6 inline parameters.

> **After any YAML change, restart the pipeline.** The file is read only at startup.

### 13.1 Why the YAML Mode Matters

The base `slam_pipeline.sh` passes only 6 parameters to slam_toolbox via `--ros-args -p`. All other parameters use internal defaults set inside the slam_toolbox package — defaults designed for general robotics use, not for specific industrial scenarios.

The critical missing parameters were:

| Parameter | Internal default | YAML value | Impact |
|---|---|---|---|
| `angle_variance_penalty` | ~0.5 | **1.0** | Scan matcher was accepting wrong angles from odometry |
| `coarse_search_angle_offset` | narrow | **0.349 rad** | Search window too small to correct paw slippage errors |
| `use_response_expansion` | false | **true** | No recovery when floor features were too sparse |
| `minimum_time_interval` | 0.5 s | **0.0** | Was discarding half the scans at 4–5 Hz |

These four parameters alone explain most of the rotational drift observed in parking garage and power plant corridors.

### 13.2 Frames and Topics

```yaml
odom_frame: odom          # Must match go2_driver's odometry frame — do not change
map_frame: map            # Output map frame — do not change
base_frame: base_link     # Robot body frame — do not change
scan_topic: /scan         # Must match the slicer's output topic — do not change
use_map_saver: true       # Enables the map saver service
mode: mapping             # mapping = build new map | localization = use existing map
map_start_pose: [0.0, 0.0, 0.0]  # Robot's starting position in the map
```

### 13.3 Map Quality

```yaml
resolution: 0.05
# Map cell size in meters. Each pixel in the occupancy grid represents this area.
# 0.05 = 5cm/cell — good for industrial corridors and rooms
# 0.03 = 3cm/cell — finer detail for small rooms, more memory/CPU
# 0.10 = 10cm/cell — coarser, for very large warehouses where memory matters

map_update_interval: 5.0
# How often the /map topic is published (seconds).
# Higher = less CPU usage but slower RViz2 updates.
# 5.0 is appropriate for most mapping sessions.

stack_size_to_use: 40000000
# Memory stack for serializing large maps (bytes = 40 MB).
# Increase if you get stack overflow errors on very large maps (>500m corridors).
```

### 13.4 Scan Insertion

```yaml
throttle_scans: 1
# Use 1 out of every N incoming scans. 1 = use all scans.
# Increase only if CPU is overloaded (e.g., on slower computers).

minimum_travel_distance: 0.2
# The robot must move at least this far (meters) before a new pose node
# is added to the SLAM graph. Lower = denser graph, more detail, more CPU.
# 0.2m is good for industrial corridors (adds a node every 20cm).

minimum_travel_heading: 0.5
# The robot must rotate at least this many radians (~28°) before a new
# pose node is added during rotation. 0.5 = adds a node roughly every
# quarter turn during slow rotation.

minimum_time_interval: 0.0
# Minimum time (seconds) between two accepted scans.
# 0.0 = accept every scan that arrives.
# CRITICAL: At 4–5 Hz (Python mode), setting this to 0.5 would discard
# half the scans. Always use 0.0 when scan rate is below 10 Hz.

transform_publish_period: 0.02
# How often to publish the map→odom TF (seconds). 0.02 = 50 Hz.
# Should be higher than the scan rate to ensure smooth visualization.

transform_timeout: 0.2
# How long to wait for a TF lookup before timing out (seconds).

tf_buffer_duration: 30.0
# How long to keep TF history in the buffer (seconds).
```

### 13.5 Angular Drift Correction — Critical Parameters

These are the most important parameters for industrial environments with smooth floors, long corridors, and turns where the robot's paws may slip.

```yaml
angle_variance_penalty: 1.0
# How strongly the scan matcher penalizes angular (yaw) errors.
# INTERNAL DEFAULT: ~0.5 — accepted odometry yaw even when slightly wrong.
# CURRENT VALUE: 1.0 — 2× stronger rejection of incorrect yaw.
#
# Physical meaning: when the robot turns a corner and its paws slip on a
# smooth metal or painted concrete floor, the odometry may report "you
# turned 90°" when the robot actually turned 87°. With penalty=0.5, the
# scan matcher accepts this 3° error and the next corridor appears rotated
# in the map. With penalty=1.0, it searches harder for the correct angle.

coarse_search_angle_offset: 0.349
# Angular search window (radians) for the coarse alignment phase.
# 0.349 rad ≈ 20°. This means: "search ±20° around the odometry estimate."
#
# INTERNAL DEFAULT: a narrower value that only searched a few degrees.
# With the narrow default, a 5° slippage error would be accepted because
# the correct pose was outside the search window.
# With 0.349: even if odometry is off by up to 20°, the scan matcher will
# find the correct pose.

coarse_angle_resolution: 0.0349
# Angular step size in the coarse search (radians ≈ 2°).
# 2° steps means 20 search positions across the ±20° window.

fine_search_angle_offset: 0.00349
# Angular search window for the fine refinement phase (radians ≈ 0.2°).
# After the coarse search finds an approximate angle, fine search refines
# it to 0.2° precision.

distance_variance_penalty: 0.5
# How strongly to penalize linear position errors (less critical than angular).

minimum_angle_penalty: 0.9
# Minimum acceptable angular match quality (0–1 scale).
# 0.9 = reject matches with less than 90% angular confidence.

minimum_distance_penalty: 0.5
# Minimum acceptable linear match quality (0–1 scale).
```

### 13.6 Scan Matching Correlation Space

These parameters define the search space used to align a new scan against the current map.

```yaml
correlation_search_space_dimension: 0.5
# Size of the search area for local scan matching (meters in each direction).
# 0.5m means the matcher searches ±0.5m around the estimated pose.

correlation_search_space_resolution: 0.01
# Precision of the local search grid (meters = 1 cm).
# 0.01m = 1 cm grid — fine enough for accurate alignment in tight corridors.
# Coarser values (0.05) would be faster but less accurate.

correlation_search_space_smear_deviation: 0.1
# Gaussian blur applied to the scan before matching.
# Slightly smears point positions to improve robustness against small
# calibration errors or noisy scans. 0.1m is a sensible default.

link_match_minimum_response_fine: 0.1
# Minimum quality threshold to link two consecutive scans (0–1).
# 0.1 is permissive — accepts even weak matches between consecutive scans.
# Increase to 0.3 if you see artifacts or "ghost" walls in the map.

link_scan_maximum_distance: 1.5
# Maximum distance (meters) between two scans for them to be linked.
# Scans further apart than this won't be directly matched.

scan_buffer_size: 10
# Number of recent scans kept in memory for local matching.

scan_buffer_maximum_scan_distance: 10.0
# Maximum distance between scans in the buffer.
```

### 13.7 Loop Closure

Loop closure detects when the robot returns to a previously mapped location and corrects accumulated drift by snapping the map together.

```yaml
do_loop_closing: true
# Enable loop closure detection. ESSENTIAL for any non-trivial mapping session.
# When the robot walks a ring corridor (around a tank, for example) and
# returns to the starting point, loop closure will correct all accumulated
# drift in a single optimization.

loop_search_maximum_distance: 3.0
# Search radius (meters) for loop closure candidates.
# 3.0m works for parking garages.
# For power plant ring corridors (large tanks), increase to 5.0 or 8.0.

loop_match_minimum_chain_size: 10
# Minimum number of pose nodes in the path for a loop candidate to be considered.
# Prevents false loop closures on very short detours.

loop_match_maximum_variance_coarse: 3.0
# Maximum covariance accepted in the coarse loop match phase.
# Higher = more permissive (accepts weaker loop candidates for refinement).

loop_match_minimum_response_coarse: 0.35
# Minimum quality threshold for coarse loop match (0–1).
# 0.35 = accept candidates with at least 35% match quality for fine refinement.

loop_match_minimum_response_fine: 0.45
# Minimum quality threshold for the final loop match (0–1).
# 0.45 = only apply loop closure if the fine match achieves ≥45% quality.
# Higher values = fewer false loop closures but may miss real loops.

loop_search_space_dimension: 8.0
# Search volume size for loop closure (meters).

loop_search_space_resolution: 0.05
# Grid resolution for loop closure search (meters).

loop_search_space_smear_deviation: 0.03
# Gaussian blur for loop closure matching (tighter than local matching).
```

### 13.8 Solver (Ceres)

The Ceres solver optimizes the pose graph — the collection of robot positions connected by scan matches.

```yaml
solver_plugin: solver_plugins::CeresSolver
ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
# Best linear solver for sparse graphs (like SLAM pose graphs).

ceres_preconditioner: SCHUR_JACOBI
# Preconditioner that accelerates convergence for pose graphs.

ceres_trust_strategy: LEVENBERG_MARQUARDT
# Optimization strategy. Levenberg-Marquardt is robust for nonlinear problems.

ceres_dogleg_type: TRADITIONAL_DOGLEG
ceres_loss_function: None
# No robust loss function — appropriate when scan matching quality is already
# filtered by the response thresholds above.
```

> Do not change these unless you have specific nonlinear optimization expertise.

### 13.9 Response Expansion

```yaml
use_response_expansion: true
# When the initial scan match returns a low-quality response (e.g., on a
# smooth metal floor with few geometric features), instead of accepting
# the odometry estimate, expand the search to a larger region.
#
# CRITICAL for power plants and parking garages:
# Smooth painted floors and reflective surfaces produce sparse scan features.
# Without response expansion, SLAM would accept a bad odometry pose.
# With it, SLAM searches more aggressively before giving up.
```

### 13.10 Complete YAML — Power Plant Profile

Ready to use for Usina / Power Plant environments. Copy this to `/go2_webrtc_ws/mapper_params_online_async.yaml`:

```yaml
slam_toolbox:
  ros__parameters:

    # Solver
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # Frames and topics
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    use_map_saver: true
    mode: mapping
    map_start_pose: [0.0, 0.0, 0.0]

    # Operation
    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.02
    map_update_interval: 5.0
    resolution: 0.05
    max_laser_range: 12.0          # Match range_max in slam_pipeline_v2.sh
    minimum_time_interval: 0.0     # Critical at 4-5 Hz (Python mode)
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000
    enable_interactive_mode: true

    # Scan insertion
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.2
    minimum_travel_heading: 0.5
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5

    # Loop closure — tuned for ring corridors around tanks
    loop_search_maximum_distance: 5.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    # Correlation (local matching precision)
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    # Angular drift correction — THE KEY PARAMETERS for industrial floors
    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0          # Strong angular correction
    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349    # ±20° search window
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true         # Recover on smooth floors
```

---

## 14. Troubleshooting

### `/scan` frequency too low (< 2 Hz when robot is stationary)

The `minimum_time_interval` in the YAML may be filtering out scans. Fix:

```bash
docker exec dev_unitree_go2 bash -c "
  sed -i 's/minimum_time_interval: 0.5/minimum_time_interval: 0.0/' \
  /go2_webrtc_ws/mapper_params_online_async.yaml &&
  grep minimum_time_interval /go2_webrtc_ws/mapper_params_online_async.yaml
"
```

Restart the pipeline.

### `No route to host` / `Failed to get robot public key`

The network connection to the robot is not established. **On the host** (not inside Docker):

```bash
sudo ip link set enx0c3796dc8be5 up
sudo ip addr flush dev enx0c3796dc8be5
sudo ip addr add 192.168.123.100/24 dev enx0c3796dc8be5
ping -c 4 192.168.123.161
```

If ping succeeds, restart the pipeline inside the container.

### `does not match an available interface supporting udp`

The Ethernet adapter is not connected or has a different name. Check on the host:

```bash
ip link show | grep enx
```

If the name is different, update `/env_ros2.sh` inside the container with the correct interface name.

### `tf2_echo base_link utlidar_lidar` hangs with "Waiting for transform" or returns no data

The pipeline is not running. The `base_link` frame only exists when `static_transform_publisher` is active (started by the pipeline). Start the pipeline first, then run `tf2_echo` in a second terminal.

### Map flickering / alternating between old and new maps

This happens when DDS retains cached map data from previous sessions (Transient Local durability). Restart the Docker container to clear all DDS memory:

```bash
# On the HOST (not inside Docker):
docker restart dev_unitree_go2
sleep 5
docker exec -it dev_unitree_go2 bash
```

**Always restart the container before a new mapping session** to avoid ghost maps.

### `enable_video:=true` causes crash or connection failure

Video requires a stable WebRTC connection and additional CPU. Common causes:

1. **Network not configured** → run the host network setup first
2. **Robot not ready** → wait for "Robot 0 validated and ready" in the logs before enabling video
3. **CPU overload (Python mode)** → use the **C++ pipeline** (`decode_lidar:=false`), which frees ~50% CPU for video processing

**Recommended:** use `enable_video:=true` with the C++ pipeline to avoid CPU overload.

### QoS warnings in the terminal

Messages like `New subscription discovered... incompatible QoS... RELIABILITY_QOS_POLICY` are **normal** and can be safely ignored. They appear when RViz2 auto-discovers topics before you manually set the Reliability Policy to Best Effort.

### `Message Filter dropping message: frame 'utlidar_lidar'` or `frame 'base_link'`

This is normal during the first 2–3 seconds of startup. The TF tree takes a moment to propagate. If it persists for more than 10 seconds, check that the `static_transform_publisher` is running:

```bash
docker exec dev_unitree_go2 bash -c "source /env_ros2.sh && ros2 node list | grep static"
```

### No data in RViz2

1. Make sure you ran `source /env_ros2.sh` in the RViz2 terminal
2. Check that `ros2 topic hz /point_cloud2` shows ~4 Hz (Python) or ~7 Hz (C++)
3. Verify Reliability Policy is set to **Best Effort** for LaserScan and PointCloud2 displays
4. The Map display should use the **default** QoS (Reliable + Transient Local) — do NOT change it to Best Effort

### Camera shows "No Image" in RViz2

The Reliability Policy for the Image display must be set to **Best Effort**. The driver publishes camera frames with Best Effort QoS, and ROS 2 will not deliver messages when the subscriber (RViz2 default: Reliable) is stricter than the publisher. See [Section 5.3](#53-rviz2-camera-setup).

### Unknown YAML parameter warning

```
[WARN] Parameter 'enable_interactive_mode' not declared
[WARN] Parameter 'minimum_time_interval' not declared
```

Some parameters were added in newer versions of slam_toolbox. If you see these warnings, comment out the offending lines in the YAML:

```yaml
# enable_interactive_mode: true   ← comment this out
# minimum_time_interval: 0.0      ← comment this out
```

The remaining parameters are still active. This does not affect mapping quality.

### Rotational drift persists after enabling YAML-Tuned pipeline

If the corridor after a 90° turn is still misaligned:

1. **Verify YAML is being loaded** — look for `Node using stack size 40000000` in the pipeline log (this parameter only comes from the YAML, not from inline args)
2. **Increase the angular search window**: `coarse_search_angle_offset: 0.523` (±30°)
3. **Increase the angular penalty**: `angle_variance_penalty: 1.5`
4. **Slow down turns** — Go2 paw slippage is worse at high angular velocity; use the controller to turn slowly
5. **Check `minimum_travel_heading`** — if set too high (e.g. 1.0 rad), SLAM won't add new nodes during turns and has less data to correct the angle
6. **Robot mode** — ensure the robot is in **Normal** mode, not Classic. Normal mode has less body sway, producing more stable scans during turns
