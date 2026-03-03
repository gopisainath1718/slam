#include <deque>
#include <yaml-cpp/yaml.h>
#include <stdexcept>

#include "data_loader/tf_loader.hpp"

namespace dataloader{

Transform Inverse(const Transform& a){
	Transform out;
	out.T = a.T.inverse();
	return out;
}

Transform Compose(const Transform&a, const Transform& b){
	Transform out;
	out.T = a.T * b.T;
	return out;
}

Transform T_from_translation_xyzw(const std::vector<double>& t, const std::vector<double>& q_xyzw){
	if(t.size() != 3) throw std::runtime_error("translation must have 3 values");
	if(q_xyzw.size() != 4) throw std::runtime_error("quaternion must have 4 values");
	
	//TODO: can be optimized
	const double qx = q_xyzw[0];
	const double qy = q_xyzw[1];
	const double qz = q_xyzw[2];
	const double qw = q_xyzw[3];

	Eigen::Quaterniond q(qw, qx, qy, qz);
	
	Transform out;
	out.T.setIdentity();
	out.T.block<3,3>(0,0) = q.toRotationMatrix();
	out.T.block<3,1>(0,3) = Eigen::Vector3d(t[0], t[1], t[2]);
	
	return out;
}

void StaticTfTree::add_edge(const std::string& parent, const std::string& child, const Transform& T_parent_child){
	adj_[parent].push_back({child, T_parent_child});
	adj_[child].push_back({parent, Inverse(T_parent_child)});
}

std::optional<Transform> StaticTfTree::look_up(const std::string& target, const std::string& source) const {
	//TODO
  if (target == source) return Transform{};

  struct Node {
    std::string frame;
    Transform T_target_frame;
  };

  std::deque<Node> q;
  std::unordered_map<std::string, bool> vis;

  q.push_back({target, Transform{}});
  vis[target] = true;

  while (!q.empty()) {
    Node cur = q.front();
    q.pop_front();

    auto it = adj_.find(cur.frame);
    if (it == adj_.end()) continue;

    for (const auto& [nbr, T_cur_nbr] : it->second) {
      if (vis[nbr]) continue;

      Transform T_target_nbr = Compose(cur.T_target_frame, T_cur_nbr);
      if (nbr == source) return T_target_nbr;

      vis[nbr] = true;
      q.push_back({nbr, T_target_nbr});
    }
  }
  return std::nullopt;
}

StaticTfTree load_tf_yaml(const std::string& path){
	YAML::Node root = YAML::LoadFile(path);
	YAML::Node trs = root["transforms"];
	if(!trs || !trs.IsSequence()) throw std::runtime_error("Missing transforms, YAML file formate is incorrect.");
	
	StaticTfTree tree;
	for(const auto& e : trs){
		const std::string parent = e["parent"].as<std::string>();
		const std::string child = e["child"].as<std::string>();
		const std::vector<double> t = e["translation"].as<std::vector<double>>();		
		const std::vector<double> q = e["quaternion"].as<std::vector<double>>();
		tree.add_edge(parent, child, T_from_translation_xyzw(t, q));
	}
	return tree;
}

}
