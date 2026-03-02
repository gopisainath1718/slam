#ifndef TOPIC_CONFIG_HPP
#define TOPIC_CONFIG_HPP

#include<string>

namespace dataloader{

struct TopicConfig{
	std::string cam_front_right = "/alphasense_driver_ros/cam0/compressed";
	std::string cam_front_left  = "/alphasense_driver_ros/cam1/compressed";
	std::string cam_side_right  = "/alphasense_driver_ros/cam3/compressed";
	std::string cam_side_left   = "/alphasense_driver_ros/cam4/compressed";
	
	std::string imu_cam = "/alphasense_driver_ros/imu";
	std::string imu_lidar = "/os_cloud_node/imu";

	std::string lidar_points = "/os_cloud_node/points";
};

struct SyncParams{
	int64_t max_cam_dt = 40000000LL;
	int64_t watermark_margin_ns = 150000000LL;
	int64_t buffer_window_ns = 5LL * 1000000000LL;
};

}

#endif
