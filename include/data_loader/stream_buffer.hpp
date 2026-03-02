#ifndef STREAM_BUFFER_HPP
#define STREAM_BUFFER_HPP

#include<deque>
#include "data_loader/time.hpp"

namespace dataloader{

template <typename T>
class StreamBuffer{
public:
	explicit StreamBuffer(TimeNs window) : window_(window){}
	
	void push_sorted(const T& data){
		if(buffer_.empty() || buffer_.back().t < data.t){
			buffer_.push_back(data);
		}
		else{
			auto it = std::lower_bound(buffer_.begin(), buffer_.end(), data.t,
				[](const T& a, TimeNs t){return a.t < t;});
			buffer_.insert(it, data);
		}
	}

	void prune_older_than(TimeNs t_ref){
		const TimeNs t_min = t_ref - window_;
		while(!buffer_.empty() && buffer_.front().t < t_min) buffer_.pop_front();	
	}

	bool empty() const { return buffer_.empty(); }
	TimeNs latest_time_or(TimeNs def){ return buffer_.empty() ? def : buffer_.back().t; }
	size_t size() const { return buffer_.size(); }
	
	std::optional<T> nearest(TimeNs t_ref, TimeNs max_dt){
		if(buffer_.empty()) return std::nullopt;
		
		auto it = std::lower_bound(buffer_.begin(), buffer_.end(),t_ref,
				[](const T& a, TimeNs t){ return a.t < t; });
		
		std::optional<T> best;
		TimeNs best_dt = (TimeNs)9e18;
		
		auto consider = [&](const T& a){
			TimeNs dt = (a.t > t_ref) ? (a.t - t_ref) : (t_ref - a.t);
			if(dt < best_dt){
				best_dt = dt;
				best = a;
			}
		};
		if(it != buffer_.end()) consider(*it);
		if(it != buffer_.begin()) consider(*std::prev(it));
		
		if(!best || best_dt > max_dt) { return std::nullopt; }
		return best;
	}

	std::vector<T> range(TimeNs from_exclusive, TimeNs to_inclusive) const {
		std::vector<T> out;
		
		if(buffer_.empty()) return out;
		auto it = std::upper_bound(buffer_.begin(), buffer_.end(), from_exclusive,
				[](TimeNs t, const T& a){ return t < a.t; });
		for(;it != buffer_.end() && it->t <= to_inclusive; ++it){
			out.push_back(*it);
		}
		return out;
	}
	
private:
	TimeNs window_;
	std::deque<T> buffer_;
};

}

#endif
