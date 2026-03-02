#ifndef DATA_LOADER_TIME_HPP
#define DATA_LOADER_TIME_HPP

#include<cstdint>
#include"builtin_interfaces/msg/time.hpp"

namespace dataloader{

using TimeNs = int64_t;

inline TimeNs to_ns(const builtin_interfaces::msg::Time& t){
	return static_cast<TimeNs>(t.sec) * 1000000000LL + static_cast<TimeNs>(t.nanosec);
}

}

#endif
