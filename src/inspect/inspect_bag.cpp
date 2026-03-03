#include <iostream>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "data_loader/rosbag2_reader.hpp"

struct Stats{
	int64_t count = 0;
	int64_t first_recv = 0;
	int64_t last_recv = 0;		
};

int main(int argc, char** argv){
	if(argc < 2){
		std::cout << "Usage: inspect_bag <bag_path>\n";
		return 1;
	}
	
	rclcpp::init(argc, argv);
	
	dataloader::RosBag2Reader reader(argv[1]);
	std::unordered_map<std::string, Stats> st;

	while(reader.has_next()){
		auto m = reader.read_next();
		auto& s = st[m->topic_name];
		s.count++;	
		if(s.count == 1) s.first_recv = (int64_t)m->time_stamp;
		s.last_recv = m->time_stamp;
	}
	
	for(const auto& [topic, s] : st){
		std::cout << topic
				  << " count = " << s.count
				  << " first_recv_ns = " << s.first_recv
				  << " last_recv_ns = " << s.last_recv
				  << "\n"; 
	}
	rclcpp::shutdown();
	return 0;
}
