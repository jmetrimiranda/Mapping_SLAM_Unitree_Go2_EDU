# Mapping SLAM - Unitree Go2 EDU

Source code for the WebRTC and Unitree Go2 EDU SDK, featuring custom camera calibrations, LiDAR processing, and DDS parameters. This setup has been fully validated and homologated for **ROS 2 Foxy**.

---

## 🐳 Running in Docker (Recommended)

To avoid dependency conflicts and keep your host machine clean, this repository is designed to run inside a Docker container with NVIDIA GPU passthrough for RViz2.

### 1. Host Preparation (Linux)
Allow Docker to access the host's X11 server for GUI rendering:
    
    xhost +local:root

Install the NVIDIA Container Toolkit on your host:
    
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
      && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
        sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
        sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
    
    sudo apt-get update
    sudo apt-get install -y nvidia-container-toolkit
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker

### 2. Docker Compose Configuration
Create a `docker-compose.yml` file on your host to persist your workspace:

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

Start the container and enter the bash session:
    
    sudo docker compose up -d
    sudo docker exec -it dev_unitree_go2 bash

---

## 🛠️ Compiling the SDK (Inside Docker)

Once inside the container, install the Foxy dependencies and compile:

    apt-get update
    apt-get install -y python3-pip python3-colcon-common-extensions git python3-rosdep
    apt-get install -y ros-foxy-rosidl-generator-dds-idl ros-foxy-fastrtps ros-foxy-rmw-fastrtps-cpp
    apt-get install -y ros-foxy-pcl-ros ros-foxy-pcl-conversions ros-foxy-sensor-msgs-py
    
    cd /go2_webrtc_ws/src
    git clone https://github.com/jmetrimiranda/Mapping_SLAM_Unitree_Go2_EDU.git go2_ros2_sdk
    cd go2_ros2_sdk
    git submodule update --init --recursive
    pip3 install -r requirements.txt
    
    cd /go2_webrtc_ws
    source /opt/ros/foxy/setup.bash
    colcon build

---

## 🚀 Mapping Orchestration (5 Terminals)

Open 5 separate terminals inside the Docker container. 
**Mandatory Setup:** Run this block in ALL 5 terminals before anything else:
    
    source /opt/ros/foxy/setup.bash
    source /upgrade_dds_ws/install/setup.bash
    source /go2_webrtc_ws/install/setup.bash
    export LC_ALL=C
    export LC_NUMERIC="en_US.UTF-8"
    unset CYCLONEDDS_URI
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

### Execution Order:

**🟢 Terminal 1: WebRTC Brain**
    
    ros2 run go2_robot_sdk go2_driver_node --ros-args -p conn_type:="webrtc" -p robot_ip:="192.168.123.161" -p enable_video:=false

**🟢 Terminal 2: TF Publisher**
    
    ros2 run tf2_ros static_transform_publisher 0.289 0.0 0.08 0.0 0.0 0.0 base_link radar

**🟢 Terminal 3: PointCloud Slicer**
    
    ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -p target_frame:=radar -p min_height:=0.25 -p max_height:=0.40 -p range_min:=0.4 -p range_max:=10.0 -r cloud_in:=/point_cloud2

**🟢 Terminal 4: Async SLAM Toolbox**
    
    ros2 run slam_toolbox async_slam_toolbox_node --ros-args -p odom_frame:=odom -p base_frame:=base_link -p map_frame:=map -p max_laser_range:=5.0 -p resolution:=0.05

**🟢 Terminal 5: RViz2**
    
    rviz2
