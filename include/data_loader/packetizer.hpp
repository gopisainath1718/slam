#ifndef PACKETIZER_HPP
#define PACKETIZER_HPP

#include <deque>
#include <optional>

#include "data_loader/config.hpp"
#include "data_loader/stream_buffer.hpp"
#include "data_loader/types.hpp"

namespace dataloader{

class Packetizer{
public:
	Packetizer(TopicConfig& topics, SyncParams& params) : 
		topics_(topics),
		params_(params),
		cam_front_right_(params.buffer_window_ns),
		cam_front_left_(params.buffer_window_ns),
		cam_side_right_(params.buffer_window_ns),
		cam_side_left_(params.buffer_window_ns),
		imu_cam_(params.buffer_window_ns),
		imu_lidar_(params.buffer_window_ns){}
	
	void push_cam_front_right(ImageFrame& fr){
		cam_front_right_.push_sorted(fr);
		latest_cam_front_right_ = fr.t;
	}
	void push_cam_front_left(ImageFrame& fl){
		cam_front_left_.push_sorted(fl);
		latest_cam_front_left_ = fl.t;
	}
	void push_cam_side_right(ImageFrame& sr){
		cam_side_right_.push_sorted(sr);
		latest_cam_side_right_ = sr.t;
	}
	void push_cam_side_left(ImageFrame& sl){
		cam_side_left_.push_sorted(sl);
		latest_cam_side_left_ = sl.t;
	}
	void push_imu_cam(ImuSample& ic){
		imu_cam_.push_sorted(ic);
		latest_imu_cam_ = ic.t;
	}
	void push_imu_lidar(ImuSample& il){
		imu_lidar_.push_sorted(il);
		latest_imu_lidar_ = il.t;
	}

	void push_lidar(LidarSample& sample){
		lidar_.push_back(sample);
		latest_lidar_ = sample.t;
	}	
	
	std::optional<DataPacket> pop_packet(){
		if(lidar_.empty()) return std::nullopt;
	
		TimeNs t_safe = std::min({latest_cam_front_right_, latest_cam_front_left_, latest_cam_side_right_, latest_cam_side_left_, latest_imu_cam_, latest_imu_lidar_}) - params_.watermark_margin_ns;

		const auto& scan = lidar_.front();
		if(scan.t > t_safe) return std::nullopt;
		
		cam_front_right_.prune_older_than(scan.t);
		cam_front_left_.prune_older_than(scan.t);
		cam_side_right_.prune_older_than(scan.t);
		cam_side_left_.prune_older_than(scan.t);
		imu_cam_.prune_older_than(scan.t);
		imu_lidar_.prune_older_than(scan.t);

		DataPacket pkt;
		pkt.t = scan.t;
		pkt.lidar = scan;	
	
		auto fr = cam_front_right_.nearest(scan.t, params_.max_cam_dt);
		auto fl = cam_front_left_.nearest(scan.t, params_.max_cam_dt);
		auto sr = cam_side_right_.nearest(scan.t, params_.max_cam_dt);
		auto sl = cam_side_left_.nearest(scan.t, params_.max_cam_dt);
	
		if(fr){ pkt.has_cam_front_right = true; pkt.cam_front_right = *fr; }	
		if(fl){ pkt.has_cam_front_left = true; pkt.cam_front_left = *fl; }	
		if(sr){ pkt.has_cam_side_right = true; pkt.cam_side_right = *sr; }	
		if(sl){ pkt.has_cam_side_left = true; pkt.cam_side_left = *sl; }

		if(prev_lidar_t_ !=0){
			pkt.imu_cam = imu_cam_.range(prev_lidar_t_, scan.t);
			pkt.has_imu_cam = !imu_cam_.empty();
			pkt.imu_lidar = imu_lidar_.range(prev_lidar_t_, scan.t);
		}
		else pkt.has_imu_cam = false;	
		
		prev_lidar_t_ = scan.t;
		lidar_.pop_front();
		return pkt;				
	}

private:
	TopicConfig topics_;
	SyncParams params_;
	
	StreamBuffer<ImageFrame> cam_front_right_;
	StreamBuffer<ImageFrame> cam_front_left_;
	StreamBuffer<ImageFrame> cam_side_right_;
	StreamBuffer<ImageFrame> cam_side_left_;
	
	StreamBuffer<ImuSample> imu_cam_;
	StreamBuffer<ImuSample> imu_lidar_;
	
	std::deque<LidarSample> lidar_;
	
	TimeNs latest_cam_front_right_ = 0;
	TimeNs latest_cam_front_left_ = 0;
	TimeNs latest_cam_side_right_ = 0;
	TimeNs latest_cam_side_left_ = 0;
	TimeNs latest_imu_cam_ = 0;
	TimeNs latest_imu_lidar_ = 0;
	TimeNs latest_lidar_ = 0;
	TimeNs prev_lidar_t_ = 0;
};

}

#endif
