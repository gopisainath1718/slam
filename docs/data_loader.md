# Data Loader Pipeline

The core of the current implementation is the data loader pipeline (`data_loader`), designed specifically to handle multi-modal asynchronous sensor data seamlessly. 

It reads sensor data stored in ROS 2 bag files using `rosbag2_cpp::Reader`. The central component is the `Packetizer` (found in `include/data_loader/packetizer.hpp`), which ensures accurate time-synchronization of multiple high-rate (IMU) and low-rate (Camera, Lidar) sensor streams.

**Key Features:**
* **Continuous Streaming & Buffering**: Utilizes `StreamBuffer` to safely store timestamped data for multiple cameras (front/side/left/right), Lidar, and IMU configurations (IMU-Cam, IMU-Lidar).
* **Time Synchronization**: The `Packetizer::pop_packet()` method intelligently retrieves a synchronized `DataPacket` triggered by new Lidar scans.
* **Association**: For every Lidar scan, it associates the nearest available camera frames across all viewing angles within a predefined time margin. It also collects all high-frequency IMU measurements bounded between the previous and current Lidar timestamps.
* **Safe Pruning**: As the pipeline progresses, it clears older unused data to maintain optimal memory footprint without losing unconsumed samples.

This robust synchronization ensures that the downstream VO, VIO, LO, and LIO algorithmic blocks accurately align spatial and temporal frames.
