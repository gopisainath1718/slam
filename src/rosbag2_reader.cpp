#include "data_loader/rosbag2_reader.hpp"
#include "rosbag2_storage/storage_options.hpp"

namespace dataloader{

RosBag2Reader::RosBag2Reader(const std::string& bag_path){
	rosbag2_storage::StorageOptions storage_options;
	storage_options.uri = bag_path;
	
	rosbag2_cpp::ConverterOptions converter_options;
	converter_options.input_serialization_format = "cdr";
	converter_options.output_serialization_format = "cdr";

	reader_.open(storage_options, converter_options);
}

bool RosBag2Reader::has_next() {
	return reader_.has_next();
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage> RosBag2Reader::read_next() {
	return reader_.read_next();
}

}
