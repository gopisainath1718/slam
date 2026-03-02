#ifndef ROSBAG2_READER_HP
#define ROSBAG2_READER_HPP

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"

namespace dataloader{

class RosBag2Reader{
public:
	explicit RosBag2Reader(const std::string& bag_path);
	bool has_next();
	std::shared_ptr<rosbag2_storage::SerializedBagMessage> read_next();
	const rosbag2_cpp::Reader& impl(){ return reader_; }	

private:
	rosbag2_cpp::Reader reader_;
};

}

#endif 
