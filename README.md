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
11. [Segmented Mapping (Large Environments)](#11-segmented-mapping-large-environments)
    - 11.1 [How Segmented Mapping Works](#111-how-segmented-mapping-works)
    - 11.2 [Scripts Overview](#112-scripts-overview)
    - 11.3 [slam_pipeline_segments.sh](#113-slam_pipeline_segmentssh)
    - 11.4 [salvar_segmento.sh](#114-salvar_segmentosh)
    - 11.5 [Step-by-Step Workflow](#115-step-by-step-workflow)
    - 11.6 [Customizing Parameters](#116-customizing-parameters)
    - 11.7 [File Structure](#117-file-structure)
12. [Troubleshooting](#12-troubleshooting)

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
docker pull mirandametri/unitree-go2-slam:cpp-decoder-v1
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
    image: mirandametri/unitree-go2-slam:cpp-decoder-v1
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
    image: osrf/ros:foxy-desktop    # instead of mirandametri/unitree-go2-slam:cpp-decoder-v1
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

**Vertical parameters** (measured from the LiDAR position, not from the ground):

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

### 6.2 Environment Profiles

#### 🏭 Large Industrial (factory, warehouse, power plant)

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
# Check scan frequency (expect ~7.7 Hz)
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
| Ceiling reflections | Points appearing above real obstacles | Decrease `max_height` |
| Glass / mirrors | Ghost walls, duplicated room geometry | No LiDAR fix — avoid glass surfaces |
| Multi-path reflections | Scattered random points in small rooms | Decrease `range_max` |
| Moving people | Temporary blobs appearing in the map | Increase `minimum_travel_distance` |
| Shiny metal surfaces | Sporadic false points at random distances | Increase `min_height`, lower `range_max` |

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
| Missing distant walls in large spaces | Increase the maximum detection distance | `range_max` | 8.0 – 10.0 | L1 effective: ~8 m | m |
| Map too noisy / too many scan insertions | Increase travel required between scans | `minimum_travel_distance` | 0.3 – 0.5 | — | m |
| Map missing detail in tight areas | Decrease travel required between scans | `minimum_travel_distance` | 0.1 – 0.2 | — | m |
| Map too coarse / blocky cells | Increase map resolution (smaller cell size) | `resolution` | 0.03 – 0.05 | — | m/cell |
| Map too noisy / computation too high | Decrease map resolution (larger cell size) | `resolution` | 0.05 – 0.10 | — | m/cell |

#### Mandatory Rules

These constraints are enforced by the physics of the system. Violating them produces empty scans or invalid data:

1. **`min_height` < `max_height`** — Always. If inverted (e.g., min=0.8, max=0.2), no points pass the filter and the scan is empty.

2. **`min_height` ≥ −0.08 m** — The LiDAR is mounted 8cm above the ground. A `min_height` of −0.08 means "at ground level." Values below −0.08 attempt to scan underground, which only produces noise. In practice, keep `min_height` ≥ 0.10 to avoid ground reflections.

3. **`range_min` < `range_max`** — Always. The dead zone must be smaller than the detection distance.

4. **`range_max` ≤ 10.0 m** — The L1 LiDAR has an effective range of approximately 8 meters. Setting `range_max` beyond 10m adds noise without meaningful detections.

5. **`max_laser_range` ≥ `range_max`** — The SLAM node's `max_laser_range` should match or exceed the slicer's `range_max`. Otherwise, SLAM discards valid scan data.

#### About the −0.08 m Limit

All height parameters (`min_height`, `max_height`) are measured **from the LiDAR position**, not from the ground. Since the LiDAR is mounted at 8cm above the ground:

- `min_height = 0.0` → at LiDAR level (8cm above ground)
- `min_height = 0.20` → 20cm above LiDAR = 28cm above ground (default)
- `min_height = −0.08` → 8cm below LiDAR = ground level
- `min_height < −0.08` → below ground = physically impossible, only noise

See the [side view diagram](#61-how-the-2d-scan-works) for a visual reference.

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
              │ /point_cloud2 (PointCloud2, ~7 Hz)
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

| Parameter | Default | Description | Tuning Guide |
|---|---|---|---|
| `target_frame` | `utlidar_lidar` | TF frame for the output scan. Must match the TF publisher. | Don't change. |
| `min_height` | `0.20` | Minimum height (meters) above the LiDAR to include in the 2D slice. | **Lower = more ground noise.** Raise to 0.25 if you see false obstacles near the robot. |
| `max_height` | `0.60` | Maximum height (meters) above the LiDAR to include. | **Higher = captures taller objects** but may include ceiling reflections. For outdoor/industrial use, try 1.0–2.0. |
| `range_min` | `0.5` | Minimum distance (meters) to accept a point. | **Raise to 0.6–0.8** if you see noise from the robot's own body. Lower to 0.3 for tight spaces. |
| `range_max` | `8.0` | Maximum distance (meters). Points beyond this are discarded. | The Go2 LiDAR has ~8m effective range. Setting higher than 10 just adds noise. |

### 8.3 SLAM Parameters

| Parameter | Default | Description | Tuning Guide |
|---|---|---|---|
| `odom_frame` | `odom` | Odometry frame from the robot. | Don't change. |
| `base_frame` | `base_link` | Robot's base frame. | Don't change. |
| `map_frame` | `map` | Output map frame. | Don't change. |
| `max_laser_range` | `8.0` | Maximum range (meters) for SLAM scan matching. | Match or slightly exceed your `range_max`. |
| `resolution` | `0.05` | Map resolution in meters/pixel. 0.05 = 5cm per cell. | **Lower = more detail but more noise.** Try 0.03 for small rooms, 0.10 for large warehouses. |
| `minimum_travel_distance` | `0.3` | Minimum distance (meters) the robot must travel before a new scan is added. | **Higher = fewer scans, cleaner map, faster.** Lower = more dense but noisier. |
| `minimum_travel_heading` | `0.3` | Minimum rotation (radians, ~17°) before a new scan is added. | Same trade-off as above. |

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

| Tag | Description |
|---|---|
| `cpp-decoder-v1` | **Recommended.** Full pipeline with C++ voxel decoder + Python fallback. |
| `python-stable` | Python-only pipeline. Backup before C++ changes. |
| `latest` | Previous stable state. |

### Switching Between Images

Edit `docker-compose.yml` and change the `image:` line:

```yaml
# For C++ pipeline (recommended):
image: mirandametri/unitree-go2-slam:cpp-decoder-v1

# For Python-only fallback:
image: mirandametri/unitree-go2-slam:python-stable
```

Then recreate the container:

```bash
docker compose down
sudo docker compose up -d
sudo docker exec -it dev_unitree_go2 bash
```

> **Note:** You do NOT need to switch images to alternate between C++ and Python pipelines. Both pipelines coexist in `cpp-decoder-v1` — just change the launch parameters (`decode_lidar:=true/false`).

---

## 10. Saving Maps

### Save current map as image

```bash
docker exec -it dev_unitree_go2 bash
source /env_ros2.sh
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/my_map_name
```

> **Note:** Always save to `/ros2_ws` to ensure the files are persisted on your host machine (mounted via Docker volume).

This generates two files:
- `my_map_name.pgm` — The occupancy grid image
- `my_map_name.yaml` — Map metadata (resolution, origin, etc.)

### Save SLAM state (to continue mapping later)

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/ros2_ws/my_map'}"
```

### Load saved map and continue mapping

Add to the SLAM node launch:

```bash
-p map_file_name:=/ros2_ws/my_map \
-p map_start_at_dock:=true
```

### Record raw data (rosbag)

```bash
ros2 bag record /point_cloud2 /scan /odom /tf /tf_static /map -o /ros2_ws/session_01
```

Replay later:

```bash
ros2 bag play /ros2_ws/session_01
```

---

## 11. Segmented Mapping (Large Environments)

For large environments like parking garages, long corridors, or multi-floor buildings, it is often impractical to map everything in a single continuous run. The robot may need to be physically repositioned (picked up and turned) between straight-line segments. This section describes a **segmented mapping workflow** that splits the mapping session into manageable segments while building a single accumulated map.

### 11.1 How Segmented Mapping Works

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

### 11.2 Scripts Overview

Two scripts handle the segmented workflow. Both live in `/go2_webrtc_ws/` and follow the same structure as `slam_pipeline.sh` — configurable variables at the top, same node launch pattern.

| Script | Purpose |
|---|---|
| `slam_pipeline_segments.sh` | Starts the full pipeline for a given segment number. If segment=1, starts from scratch. If segment>1, loads the previous segment's map and continues. |
| `salvar_segmento.sh` | Saves the current SLAM state as the specified segment number. |

### 11.3 `slam_pipeline_segments.sh`

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

### 11.4 `salvar_segmento.sh`

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

### 11.5 Step-by-Step Workflow

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

### 11.6 Customizing Parameters

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

### 11.7 File Structure

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

## 12. Troubleshooting

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

### `Message Filter dropping message: frame 'utlidar_lidar'`

This is normal during the first 2-3 seconds of startup. The TF tree takes a moment to propagate. If it persists for more than 10 seconds, check that the `static_transform_publisher` is running.

### No data in RViz2

1. Make sure you ran `source /env_ros2.sh` in the RViz2 terminal
2. Check that `ros2 topic hz /point_cloud2` shows ~7 Hz
3. Verify Reliability Policy is set to **Best Effort** for LaserScan and PointCloud2 displays
4. The Map display should use the **default** QoS (Reliable + Transient Local) — do NOT change it to Best Effort

### Camera shows "No Image" in RViz2

The Reliability Policy for the Image display must be set to **Best Effort**. The driver publishes camera frames with Best Effort QoS, and ROS 2 will not deliver messages when the subscriber (RViz2 default: Reliable) is stricter than the publisher. See [Section 5.3](#53-rviz2-camera-setup).