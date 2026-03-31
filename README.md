# Mapping SLAM - Unitree Go2 EDU

Source code for the WebRTC and Unitree Go2 EDU SDK, featuring custom camera calibrations, LiDAR processing, and DDS parameters. This setup has been fully validated and homologated for **ROS 2 Foxy**.

---

## 🚀 Quick Start with Pre-built Image (Recommended)

A fully configured Docker image with ROS 2 Foxy, CycloneDDS, the Go2 SDK, and all dependencies pre-installed is available on Docker Hub. **No compilation needed — just pull and run.**

### Prerequisites

- Linux (Ubuntu 20.04+ recommended)
- Docker Engine installed ([install guide](https://docs.docker.com/engine/install/ubuntu/))
- NVIDIA GPU + NVIDIA Container Toolkit (for RViz2 visualization)
- Wired Ethernet connection to the Unitree Go2 EDU robot

### 1. Host Preparation

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

### 2. Pull the Pre-built Image

```bash
docker pull mirandametri/unitree-go2-slam:latest
```

> This downloads ~8 GB (compressed). The full image is ~24 GB uncompressed and includes: ROS 2 Foxy, CycloneDDS, go2_robot_sdk, slam_toolbox, pointcloud_to_laserscan, PCL, and all Python/C++ dependencies.

### 3. Create the Docker Compose File

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
    image: mirandametri/unitree-go2-slam:latest
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

### 4. Start the Container

```bash
cd ~/unitree_slam
sudo docker compose up -d
sudo docker exec -it dev_unitree_go2 bash
```

You are now inside the container with everything ready. Proceed to the **Environment Setup** section below.

---

## 🛠️ Building from Source (Alternative)

If you prefer to build everything from scratch instead of using the pre-built image, use `osrf/ros:foxy-desktop` as the base image in the `docker-compose.yml`:

```yaml
    image: osrf/ros:foxy-desktop    # instead of mirandametri/unitree-go2-slam:latest
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

## 🔧 Environment Setup Script (`env_ros2.sh`)

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
# Loads the base ROS 2 installation (rcl, rclpy, rclcpp, etc.)
source /opt/ros/foxy/setup.bash

# --- CycloneDDS workspace ---
# Loads the custom-compiled CycloneDDS and its RMW layer.
# This replaces the default FastRTPS with CycloneDDS for better
# compatibility with the Unitree Go2's internal DDS traffic.
source /upgrade_dds_ws/install/setup.bash 2>/dev/null || true

# --- Go2 SDK workspace ---
# Loads the go2_robot_sdk, lidar_processor, lidar_processor_cpp,
# go2_interfaces, and all custom message types.
source /go2_webrtc_ws/install/setup.bash

# --- Locale ---
# Forces the C locale to avoid decimal separator issues (comma vs dot)
# in European/Brazilian systems. LC_NUMERIC ensures that floating-point
# parameters like "0.05" are parsed correctly by ROS 2 nodes.
export LC_ALL=C
export LC_NUMERIC="en_US.UTF-8"

# --- DDS Domain Isolation ---
# The Unitree Go2 robot runs its own internal ROS 2 nodes on domain 0.
# We use domain 42 to isolate our SLAM pipeline from the robot's
# internal traffic. Without this, you get ghost nodes and topic conflicts.
export ROS_DOMAIN_ID=42

# --- DDS Implementation ---
# Switches from the default FastRTPS to CycloneDDS.
# CycloneDDS handles the Go2's large UDP packets (LiDAR data) much
# better than FastRTPS, which tends to fragment and drop them.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# --- CycloneDDS Configuration ---
# NetworkInterfaceAddress: The wired Ethernet adapter connected to the robot.
#   Find yours with: ip a | grep "192.168.123" (on the host, before Docker)
#   Common names: enx0c3796dc8be5, eth0, eno1
# MaxMessageSize: 65500 bytes (max UDP payload, prevents fragmentation)
# FragmentSize: 4000 bytes (CycloneDDS internal fragment size)
# WhcHigh: 500kB write-cache watermark (prevents buffer overflow at 7+ Hz)
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>enx0c3796dc8be5</NetworkInterfaceAddress><MaxMessageSize>65500</MaxMessageSize><FragmentSize>4000</FragmentSize></General><Internal><Watermarks><WhcHigh>500kB</WhcHigh></Watermarks></Internal></Domain></CycloneDDS>"

echo "ROS 2 environment configured:"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  RMW=$RMW_IMPLEMENTATION"
echo "  Interface=enx0c3796dc8be5"
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

## ⚡ Quick Mode — Single Script (2 Terminals)

The fastest way to start mapping. Everything runs from a single script in one terminal, and you open RViz2 in a second terminal.

> 💡 **When to use:** Quick mapping sessions, demos, or when you want minimal setup. The Python driver decodes the LiDAR internally at ~7 Hz.

### Terminal 1 — Pipeline

```bash
source /env_ros2.sh

# Kill any leftover processes from previous sessions
pkill -f "go2_driver\|pointcloud_to_laser\|slam_tool\|static_transform\|rviz2"
sleep 3

# 1. WebRTC Driver — connects to the robot and decodes LiDAR data
ros2 run go2_robot_sdk go2_driver_node --ros-args \
  -p conn_type:="webrtc" \
  -p robot_ip:="192.168.123.161" \
  -p enable_video:=false \
  -p decode_lidar:=true \
  -p publish_raw_voxel:=false &
sleep 8

# 2. TF Publisher — defines the LiDAR's position on the robot body
ros2 run tf2_ros static_transform_publisher \
  0.289 0.0 0.08 0.0 0.0 0.0 base_link utlidar_lidar &
sleep 1

# 3. PointCloud Slicer — converts 3D point cloud into 2D laser scan
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -p target_frame:=utlidar_lidar \
  -p min_height:=0.15 \
  -p max_height:=0.60 \
  -p range_min:=0.5 \
  -p range_max:=10.0 \
  -r cloud_in:=/point_cloud2 &
sleep 1

# 4. SLAM — builds the 2D occupancy grid map
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  -p odom_frame:=odom \
  -p base_frame:=base_link \
  -p map_frame:=map \
  -p max_laser_range:=8.0 \
  -p resolution:=0.05 \
  -p minimum_travel_distance:=0.3 \
  -p minimum_travel_heading:=0.3 &
sleep 2

echo "Pipeline running. Open RViz2 in Terminal 2."
wait
```

### Terminal 2 — Visualization

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

To stop the pipeline, press `Ctrl+C` in Terminal 1.

---

## 🐢 Standard Mode — Multi-Terminal (5 Terminals)

Each node runs in its own terminal for better debugging and control. Identical pipeline to Quick Mode, but you can monitor each node's output individually.

> 💡 **When to use:** Debugging, parameter tuning, or when you need to restart individual nodes without killing the whole pipeline.

**Run `source /env_ros2.sh` in ALL 5 terminals before starting.**

### Terminal 1 — WebRTC Driver

```bash
ros2 run go2_robot_sdk go2_driver_node --ros-args \
  -p conn_type:="webrtc" \
  -p robot_ip:="192.168.123.161" \
  -p enable_video:=false \
  -p decode_lidar:=true \
  -p publish_raw_voxel:=false
```

### Terminal 2 — TF Publisher

```bash
ros2 run tf2_ros static_transform_publisher \
  0.289 0.0 0.08 0.0 0.0 0.0 base_link utlidar_lidar
```

### Terminal 3 — PointCloud Slicer

```bash
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -p target_frame:=utlidar_lidar \
  -p min_height:=0.15 \
  -p max_height:=0.60 \
  -p range_min:=0.5 \
  -p range_max:=10.0 \
  -r cloud_in:=/point_cloud2
```

### Terminal 4 — SLAM

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

### Terminal 5 — RViz2

```bash
rviz2
```

Follow the same RViz2 setup instructions from Quick Mode above.

---

## 📐 Parameter Reference

### WebRTC Driver Parameters

| Parameter | Default | Description |
|---|---|---|
| `conn_type` | `"webrtc"` | Connection protocol. Always `"webrtc"` for Go2 EDU. |
| `robot_ip` | `"192.168.123.161"` | Robot's IP on the wired Ethernet. Default for Go2 EDU. |
| `enable_video` | `false` | Enable camera stream. Set `true` for visual SLAM or object detection. Increases CPU usage. |
| `decode_lidar` | `true` | Decode compressed voxels into PointCloud2 inside the driver. Must be `true` for the Python pipeline. |
| `publish_raw_voxel` | `false` | Publish raw compressed voxels on `/utlidar/voxel_map_compressed`. Only useful for custom C++ decoders. |

### PointCloud Slicer Parameters

These control how the 3D point cloud is "sliced" into a 2D laser scan for SLAM.

| Parameter | Default | Description | Tuning Guide |
|---|---|---|---|
| `target_frame` | `utlidar_lidar` | TF frame for the output scan. Must match the TF publisher. | Don't change. |
| `min_height` | `0.15` | Minimum height (meters) above the LiDAR to include in the 2D slice. | **Lower = more ground noise.** Raise to 0.20–0.25 if you see false obstacles near the robot. |
| `max_height` | `0.60` | Maximum height (meters) above the LiDAR to include. | **Higher = captures taller objects** but may include ceiling reflections. For outdoor use, try 1.0–2.0. |
| `range_min` | `0.5` | Minimum distance (meters) to accept a point. | **Raise to 0.6–0.8** if you see noise from the robot's own body. |
| `range_max` | `10.0` | Maximum distance (meters). Points beyond this are discarded. | The Go2 LiDAR has ~8m effective range. Setting higher than 10 just adds noise. |

**Example: Outdoor mapping with taller obstacles**
```bash
-p min_height:=0.20 -p max_height:=1.5 -p range_min:=0.6 -p range_max:=10.0
```

**Example: Tight indoor corridor**
```bash
-p min_height:=0.15 -p max_height:=0.40 -p range_min:=0.3 -p range_max:=5.0
```

### SLAM Parameters

| Parameter | Default | Description | Tuning Guide |
|---|---|---|---|
| `odom_frame` | `odom` | Odometry frame from the robot. | Don't change. |
| `base_frame` | `base_link` | Robot's base frame. | Don't change. |
| `map_frame` | `map` | Output map frame. | Don't change. |
| `max_laser_range` | `8.0` | Maximum range (meters) for SLAM scan matching. | Match or slightly exceed your `range_max`. |
| `resolution` | `0.05` | Map resolution in meters/pixel. 0.05 = 5cm per cell. | **Lower = more detail but more noise.** Try 0.03 for small rooms, 0.10 for large warehouses. |
| `minimum_travel_distance` | `0.3` | Minimum distance (meters) the robot must travel before a new scan is added. | **Higher = fewer scans, cleaner map, faster.** Lower = more dense but noisier. |
| `minimum_travel_heading` | `0.3` | Minimum rotation (radians, ~17°) before a new scan is added. | Same trade-off as above. |

### TF Publisher Parameters

The 6 numbers represent the LiDAR's position relative to `base_link`: `x y z roll pitch yaw`

| Value | Meaning | Go2 Default |
|---|---|---|
| `0.289` | X offset — LiDAR is 28.9cm forward of center | Measured on Go2 EDU |
| `0.0` | Y offset — centered laterally | — |
| `0.08` | Z offset — LiDAR is 8cm above the base | Measured on Go2 EDU |
| `0.0 0.0 0.0` | No rotation (roll, pitch, yaw) | LiDAR is level |

---

## ⚠️ Troubleshooting

### Map flickering / alternating between old and new maps

This happens when DDS retains cached map data from previous sessions (Transient Local durability). The solution is to restart the Docker container to clear all DDS memory:

```bash
# On the HOST (not inside Docker):
docker restart dev_unitree_go2
sleep 5
docker exec -it dev_unitree_go2 bash
```

Then start the pipeline fresh inside the new container session. **Always restart the container before a new mapping session** to avoid ghost maps.

### QoS warnings in the terminal

Messages like `New subscription discovered... incompatible QoS... RELIABILITY_QOS_POLICY` are **normal** and can be safely ignored. They appear when RViz2 auto-discovers topics before you manually set the Reliability Policy to Best Effort. The data still flows correctly for topics you've configured properly.

### `Message Filter dropping message: frame 'utlidar_lidar'`

This is normal during the first 2-3 seconds of startup. The TF tree takes a moment to propagate. If it persists for more than 10 seconds, check that the `static_transform_publisher` is running.

### No data in RViz2

1. Make sure you ran `source /env_ros2.sh` in the RViz2 terminal
2. Check that `ros2 topic hz /point_cloud2` shows ~7 Hz
3. Verify Reliability Policy is set to **Best Effort** for LaserScan and PointCloud2 displays
4. The Map display should use the **default** QoS (Reliable + Transient Local) — do NOT change it to Best Effort

---

## 💾 Saving the Map

Once your mapping session is complete and the map looks good in RViz2, open a new terminal:

```bash
docker exec -it dev_unitree_go2 bash
source /env_ros2.sh
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/my_map_name
```

> **Note:** Always save to `/ros2_ws` to ensure the files are persisted on your host machine (mounted via Docker volume).

This generates two files:
- `my_map_name.pgm` — The occupancy grid image
- `my_map_name.yaml` — Map metadata (resolution, origin, etc.)

---

## 📊 Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    Unitree Go2 EDU                          │
│              (192.168.123.161, WebRTC)                       │
└─────────────┬───────────────────────────────────────────────┘
              │ Compressed Voxels via WebRTC
              ▼
┌─────────────────────────────────────────────────────────────┐
│  go2_driver_node (Python, decode_lidar:=true)               │
│  Decodes voxels → publishes /point_cloud2 (Best Effort)     │
│  Also publishes: /odom, /imu, /tf, /joint_states            │
└─────────────┬───────────────────────────────────────────────┘
              │ /point_cloud2 (PointCloud2, ~7 Hz)
              ▼
┌─────────────────────────────────────────────────────────────┐
│  pointcloud_to_laserscan_node                               │
│  Slices 3D cloud → 2D laser scan                            │
│  Input: /point_cloud2  Output: /scan (Best Effort)          │
└─────────────┬───────────────────────────────────────────────┘
              │ /scan (LaserScan, ~7 Hz)
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
└─────────────────────────────────────────────────────────────┘
```