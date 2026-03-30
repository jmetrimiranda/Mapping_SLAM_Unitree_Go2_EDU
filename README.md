# Mapping SLAM - Unitree Go2 EDU

Source code for the WebRTC and Unitree Go2 EDU SDK, featuring custom camera calibrations, LiDAR processing, and DDS parameters. This setup has been fully validated and homologated for **ROS 2 Foxy**.

## 🐳 Running in Docker (Recommended)

To avoid dependency conflicts and keep your host machine clean, this repository is designed to run inside a Docker container with NVIDIA GPU passthrough for RViz2.

### 1. Host Preparation (Linux)

Allow Docker to access the host's X11 server for GUI rendering:

```bash
xhost +local:root
```

Install the NVIDIA Container Toolkit on your host:

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### 2. Docker Compose Configuration

Create a `docker-compose.yml` file on your host to persist your workspace:

```yaml
version: '3.8'

services:
  ros2_foxy_dev:
    image: osrf/ros:foxy-desktop
    container_name: dev_unitree_go2
    network_mode: host      # Crucial for DDS/UDP traffic
    ipc: host               # Crucial for LiDAR shared memory
    pid: host
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
      - NVIDIA_DRIVER_CAPABILITIES=all
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./workspace:/ros2_ws
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu]
    command: tail -f /dev/null
```

Start the container and enter the bash session:

```bash
sudo docker compose up -d
sudo docker exec -it dev_unitree_go2 bash
```

---

## 🛠️ Compiling the SDK (Inside Docker)

Once inside the container, install the Foxy dependencies and compile:

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

## ⚡ High-Performance Mode (C++ LiDAR Decoding - Recommended)

If you are using a wired connection and need maximum LiDAR frequency (up to ~8Hz+), use the C++ processor to offload the heavy math from Python. **This is the recommended way to run SLAM.**

> 💡 **Network Interface Tip:** Before running the mandatory setup, you need to know the name of your wired network interface connected to the robot. Open a terminal and run `ip a`. Look for the interface that has the IP `192.168.123.x` (usually starts with `enx`, `eth`, or `eno`).

**Mandatory Setup:** Run this block in **ALL 6 terminals** before executing any ROS 2 node. This isolates the host from the robot's internal DDS traffic (`ROS_DOMAIN_ID=42`) and increases the UDP buffer to prevent bottlenecks.

```bash
source /opt/ros/foxy/setup.bash
source /upgrade_dds_ws/install/setup.bash
source /go2_webrtc_ws/install/setup.bash

export LC_ALL=C
export LC_NUMERIC="en_US.UTF-8"
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>YOUR_NETWORK_INTERFACE</NetworkInterfaceAddress><MaxMessageSize>65500</MaxMessageSize><FragmentSize>4000</FragmentSize></General><Internal><Watermarks><WhcHigh>500kB</WhcHigh></Watermarks></Internal></Domain></CycloneDDS>"
```
*(Replace `YOUR_NETWORK_INTERFACE` with the name found using `ip a`, e.g., `enx0c3796dc8be5`)*

### Execution Order for C++ Mode (6 Terminals):

**🟢 Terminal 1: WebRTC Driver (Raw Data Extraction)**
```bash
ros2 run go2_robot_sdk go2_driver_node --ros-args -p conn_type:="webrtc" -p robot_ip:="192.168.123.161" -p enable_video:=false -p decode_lidar:=false -p publish_raw_voxel:=true
```

**🟢 Terminal 2: C++ LiDAR Processor (3D PointCloud Generator)**
```bash
ros2 run lidar_processor_cpp lidar_to_pointcloud_node --ros-args -p robot_ip:="192.168.123.161"
```

**🟢 Terminal 3: TF Publisher (Bridge to LiDAR Frame)**
```bash
ros2 run tf2_ros static_transform_publisher 0.289 0.0 0.08 0.0 0.0 0.0 base_link utlidar_lidar
```

**🟢 Terminal 4: PointCloud Slicer (3D to 2D Conversion)**
```bash
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -p target_frame:=utlidar_lidar -p min_height:=0.25 -p max_height:=0.40 -p range_min:=0.4 -p range_max:=10.0 -r cloud_in:=/point_cloud2
```

**🟢 Terminal 5: Async SLAM Toolbox (Cartographer)**
```bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args -p odom_frame:=odom -p base_frame:=base_link -p map_frame:=map -p max_laser_range:=5.0 -p resolution:=0.05
```

**🟢 Terminal 6: RViz2 (Visualization)**
To avoid QoS mismatch crashes during startup, launch RViz2 without loading corrupted cached configs:
```bash
rviz2 -d ""
```

> ⚠️ **Important RViz2 Setup:** Inside RViz2, add `PointCloud2` and `LaserScan` displays. **BEFORE** typing `/point_cloud2` or `/scan` in their respective Topic fields, you must expand the display properties and ensure the Reliability Policy is set to **Best Effort**. Otherwise, the connection will crash.

---

## 🐢 Legacy Mode (Python Decoding)

If the C++ node fails or you are running in a constrained environment, you can use the Python driver to decode the LiDAR directly. This requires only 5 terminals.

**Mandatory Setup:** Use the exact same export block as shown in the C++ section in all 5 terminals.

### Execution Order for Python Mode:

**🟢 Terminal 1: WebRTC Brain (Full Decode)**
```bash
ros2 run go2_robot_sdk go2_driver_node --ros-args -p conn_type:="webrtc" -p robot_ip:="192.168.123.161" -p enable_video:=false -p decode_lidar:=true -p publish_raw_voxel:=false
```

**🟢 Terminal 2: TF Publisher**
```bash
ros2 run tf2_ros static_transform_publisher 0.289 0.0 0.08 0.0 0.0 0.0 base_link utlidar_lidar
```

**🟢 Terminal 3: PointCloud Slicer**
```bash
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -p target_frame:=utlidar_lidar -p min_height:=0.25 -p max_height:=0.40 -p range_min:=0.4 -p range_max:=10.0 -r cloud_in:=/point_cloud2
```

**🟢 Terminal 4: Async SLAM Toolbox**
```bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args -p odom_frame:=odom -p base_frame:=base_link -p map_frame:=map -p max_laser_range:=5.0 -p resolution:=0.05
```

**🟢 Terminal 5: RViz2**
```bash
rviz2 -d ""
```
*(Remember to set Reliability to **Best Effort** for PointCloud2 and LaserScan before typing the topic).*

---

## 💾 Saving the Map

Once your mapping session is complete and the map looks good in RViz2, run the following command to save it.

> **Note:** Always save to `/ros2_ws` to ensure the files are persisted on your host machine. 

Open a new terminal, run the **Mandatory Setup**, and then execute:

```bash
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/my_map_name
```

This will generate two files on your host machine:
* `my_map_name.pgm`: The occupancy grid image.
* `my_map_name.yaml`: The map metadata (resolution, origin, etc).
