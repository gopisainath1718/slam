# SLAM

This repository contains the beginnings of a robust real-time SLAM (Simultaneous Localization and Mapping) pipeline. The project is planned to implement various state-of-the-art methods, including:
* **VO** (Visual Odometry)
* **VIO** (Visual-Inertial Odometry)
* **LO** (Lidar Odometry)
* **LIO** (Lidar-Inertial Odometry)

Currently, the foundation of the pipeline focuses on efficiently loading and synchronizing diverse sensor streams from ROS 2 bag files.

## Requirements

To build and run this package, you will need the following libraries and dependencies installed on your system:

* **ROS 2** (rclcpp, rosbag2_cpp, rosbag2_storage, sensor_msgs)
* **Eigen3** (for linear algebra operations)
* **OpenCV** (for image processing and computer vision)
* **Pangolin** (for 3D visualization)

Alternatively, you can install all required dependencies using the provided script:
```bash
./install_dependencies.sh
```

### Supported Data and Datasets

This repository focuses on handling multiple modalities and is aimed at testing with the **Newer College Dataset**. The core pipeline architecture is designed to flexibly support:
* **Real-time sensors**: Live streaming connections directly from physical hardware.
* **ROS Bags**: Built-in support for reading offline `rosbag2` files.
* **Raw Data**: Reading of raw sensor data stored in specific, non-ROS structured formats on disk.

## Cloning and Execution

### 1. Clone the repository
```bash
git clone https://github.com/gopisainath1718/slam.git
cd slam
```

### 2. Build the project
Create a build directory and compile the source code using CMake:
```bash
mkdir build
cd build
cmake ..
make -j$(nproc)
```

### 3. Execution
The executables are outputted to the `bin/` directory. You can run them from the project root:

To inspect a ROS 2 bag file:
```bash
./bin/inspect_bag /path/to/ros2/bag
```

To run the dataloader pipeline and dump synchronized packets:
```bash
./bin/dump_packets /path/to/ros2/bag
```

## Data Loader Pipeline

The core of the current implementation is the data loader pipeline (`data_loader`), designed specifically to handle multi-modal asynchronous sensor data seamlessly. 

It reads sensor data stored in ROS 2 bag files using `rosbag2_cpp::Reader`. The central component is the `Packetizer` (found in `include/data_loader/packetizer.hpp`), which ensures accurate time-synchronization of multiple high-rate (IMU) and low-rate (Camera, Lidar) sensor streams.

**Key Features:**
* **Continuous Streaming & Buffering**: Utilizes `StreamBuffer` to safely store timestamped data for multiple cameras (front/side/left/right), Lidar, and IMU configurations (IMU-Cam, IMU-Lidar).
* **Time Synchronization**: The `Packetizer::pop_packet()` method intelligently retrieves a synchronized `DataPacket` triggered by new Lidar scans.
* **Association**: For every Lidar scan, it associates the nearest available camera frames across all viewing angles within a predefined time margin. It also collects all high-frequency IMU measurements bounded between the previous and current Lidar timestamps.
* **Safe Pruning**: As the pipeline progresses, it clears older unused data to maintain optimal memory footprint without losing unconsumed samples.

This robust synchronization ensures that the downstream VO, VIO, LO, and LIO algorithmic blocks accurately align spatial and temporal frames.
