cmake_minimum_required(VERSION 3.16)
project(slam CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${Pangolin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_library(data_loader src/rosbag2_reader.cpp)
target_link_libraries(data_loader
	rclcpp::rclcpp
	rosbag2_storage::rosbag2_storage
	rosbag2_cpp::rosbag2_cpp	
)

add_executable(inspect_bag src/inspect_bag.cpp)
target_link_libraries(inspect_bag
	data_loader
	rclcpp::rclcpp
)

add_executable(dump_packets src/dump_packets.cpp)
target_link_libraries(dump_packets
	data_loader
	rclcpp::rclcpp
	${EIGEN3_LIBS}
	${OpenCV_LIBS}
	sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
	${Pangolin_LIBRARIES}
)
