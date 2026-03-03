#include <iostream>
#include <set>
#include <string>
#include <vector>

#include <pangolin/pangolin.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include "data_loader/tf_loader.hpp"

static pangolin::OpenGlMatrix ToGl(const Eigen::Matrix4d& T) {
  pangolin::OpenGlMatrix M;
  // OpenGL expects column-major
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      M.m[c * 4 + r] = T(r, c);
  return M;
}

static void DrawAxis(const Eigen::Matrix4d& T, float s) {
  glPushMatrix();
  pangolin::OpenGlMatrix M = ToGl(T);
  glMultMatrixd(M.m);

  glLineWidth(2.0f);
  glBegin(GL_LINES);
  // X (red)
  glColor3f(1.f, 0.f, 0.f); glVertex3f(0,0,0); glVertex3f(s,0,0);
  // Y (green)
  glColor3f(0.f, 1.f, 0.f); glVertex3f(0,0,0); glVertex3f(0,s,0);
  // Z (blue)
  glColor3f(0.f, 0.f, 1.f); glVertex3f(0,0,0); glVertex3f(0,0,s);
  glEnd();

  glPopMatrix();
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <tf_yaml> [root_frame]\n";
    return 1;
  }
  const std::string tf_path = argv[1];
  const std::string root = (argc >= 3) ? argv[2] : "base";

  // Load YAML, collect frames, build tree
  YAML::Node root_yaml = YAML::LoadFile(tf_path);
  auto trs = root_yaml["transforms"];
  if (!trs || !trs.IsSequence()) {
    std::cerr << "YAML missing transforms sequence\n";
    return 1;
  }

  dataloader::StaticTfTree tree;
  std::set<std::string> frames;
  frames.insert(root);

  for (const auto& e : trs) {
    const std::string parent = e["parent"].as<std::string>();
    const std::string child  = e["child"].as<std::string>();
    const auto t = e["translation"].as<std::vector<double>>();
    const auto q = e["quaternion"].as<std::vector<double>>();

    dataloader::Transform Tpc = dataloader::T_from_translation_xyzw(t, q);
    tree.add_edge(parent, child, Tpc);

    frames.insert(parent);
    frames.insert(child);
  }

  // Pangolin window + camera
  pangolin::CreateWindowAndBind("TF Frames", 1280, 720);
  glEnable(GL_DEPTH_TEST);

  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1280, 720, 800, 800, 640, 360, 0.1, 1000),
    pangolin::ModelViewLookAt(3, -3, 2, 0, 0, 0, pangolin::AxisZ)
  );

  pangolin::Handler3D handler(s_cam);
  pangolin::View& d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, 0.0, 1.0, -1280.0f/720.0f)
    .SetHandler(&handler);

  pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, 0.22);
  pangolin::Var<float> axis_scale("ui.axis_scale", 0.2f, 0.01f, 2.0f);
  pangolin::Var<bool> show_root("ui.show_root", true, true);

  // Precompute poses T_root_frame for all frames (static)
  struct FramePose { std::string name; Eigen::Matrix4d T_root_frame; };
  std::vector<FramePose> poses;
  poses.reserve(frames.size());

  for (const auto& f : frames) {
    auto T = tree.look_up(root, f); // returns T_root_f
    if (!T) continue;
    poses.push_back({f, T->T});
  }

  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);

    if (show_root) {
      DrawAxis(Eigen::Matrix4d::Identity(), axis_scale);
    }

    for (const auto& p : poses) {
      if (p.name == root) continue;
      DrawAxis(p.T_root_frame, axis_scale);
    }

    pangolin::FinishFrame();
  }

  return 0;
}
