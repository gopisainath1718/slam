#ifndef TF_LOADER_HPP
#define TF_LOADER_HPP

#include <string>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>

namespace dataloader{

//TODO: combine all the functions for Transform struct into a single class
struct Transform {
	Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
};

Transform Inverse(const Transform& a);
Transform Compose(const Transform& a, const Transform& b);
Transform T_from_translation_xyzw(const std::vector<double>& t, const std::vector<double>& q_xyzw);

class StaticTfTree{
public:
	void add_edge(const std::string& parent, const std::string& child, const Transform& T_parent_child);
	std::optional<Transform> look_up(const std::string& target, const std::string& source) const;
private:
	std::unordered_map<std::string, std::vector<std::pair<std::string, Transform>>> adj_;
};

StaticTfTree load_tf_yaml(const std::string& path);

}

#endif
