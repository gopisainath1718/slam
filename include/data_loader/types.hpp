#ifndef TYPES_HPP
#define TYPES_HPP

#include<opencv2/opencv.hpp>
#include<vector>
#include<Eigen/Dense>
#include "data_loader/time.hpp"

namespace dataloader{

struct ImageFrame{
	TimeNs t = 0;
	cv::Mat img;
};

struct ImuSample{
	TimeNs t = 0;
	Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
	Eigen::Vector3d accel = Eigen::Vector3d::Zero();
};

struct PointXYZI{
	float x = 0;
	float y = 0;
	float z = 0;
	float intensity = 0;
};

struct LidarSample{
	TimeNs t = 0;
	std::vector<PointXYZI> pts;
};

struct DataPacket{
	TimeNs t = 0;
	ImageFrame cam_front_right;
	ImageFrame cam_front_left;
	ImageFrame cam_side_right;
	ImageFrame cam_side_left;

	bool has_cam_front_right = false;
	bool has_cam_front_left = false;
	bool has_cam_side_right = false;
	bool has_cam_side_left = false;
	
	bool has_imu_cam = false;

	std::vector<ImuSample> imu_cam;
	std::vector<ImuSample> imu_lidar;

	LidarSample lidar;
};

}

#endif
