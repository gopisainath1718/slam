#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include "data_loader/rosbag2_reader.hpp"
#include "data_loader/config.hpp"
#include "data_loader/packetizer.hpp"
#include "data_loader/time.hpp"
#include "data_loader/types.hpp"
#include <pangolin/pangolin.h>
#include <mutex>
#include <thread>

struct Pt { float x,y,z,i; };

static std::mutex g_mtx;
static std::vector<Pt> g_pts;

static void update_points(const std::vector<Pt>& pts) {
  std::lock_guard<std::mutex> lk(g_mtx);
  g_pts = pts; // copy; for speed use swap/move
}

static void draw_loop() {
  pangolin::CreateWindowAndBind("lidar", 1280, 720);
  glEnable(GL_DEPTH_TEST);

  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1280,720, 500,500, 640,360, 0.1, 2000),
    pangolin::ModelViewLookAt(0,-10,-2, 0,0,0, 0,-1,0)
  );

  pangolin::Handler3D handler(s_cam);
  pangolin::View& d_cam = pangolin::CreateDisplay()
      .SetBounds(0.0, 1.0, 0.0, 1.0, -1280.0f/720.0f)
      .SetHandler(&handler);

  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);

    // copy points under lock
    std::vector<Pt> local;
    {
      std::lock_guard<std::mutex> lk(g_mtx);
      local = g_pts;
    }

    glPointSize(2.0f);
    glBegin(GL_POINTS);
    for (size_t k = 0; k < local.size(); k += 3) { // draw every 3rd point (speed)
      const auto& p = local[k];
      // simple coloring by intensity
      float c = std::min(1.0f, std::max(0.0f, p.i / 255.0f));
      glColor3f(c, c, c);
      glVertex3f(p.x, p.y, p.z);
    }
    glEnd();

    pangolin::FinishFrame();
  }
}

static cv::Mat make_mosaic_2x2(const std::optional<dataloader::ImageFrame>& fr,
                              const std::optional<dataloader::ImageFrame>& fl,
                              const std::optional<dataloader::ImageFrame>& sr,
                              const std::optional<dataloader::ImageFrame>& sl)
{
  // pick a reference size from first available image
  cv::Size ref_sz(640, 480);
  auto pick_size = [&](const std::optional<dataloader::ImageFrame>& x) {
    if (x && !x->img.empty()) ref_sz = x->img.size();
  };
  pick_size(fr); pick_size(fl); pick_size(sr); pick_size(sl);

  auto norm = [&](const std::optional<dataloader::ImageFrame>& x) {
    cv::Mat out(ref_sz, CV_8UC1, cv::Scalar(0));
    if (x && !x->img.empty()) {
      if (x->img.size() == ref_sz) out = x->img;
      else cv::resize(x->img, out, ref_sz, 0, 0, cv::INTER_AREA);
    }
    return out;
  };

  cv::Mat a = norm(fl); // top-left
  cv::Mat b = norm(fr); // top-right
  cv::Mat c = norm(sl); // bottom-left
  cv::Mat d = norm(sr); // bottom-right

  cv::Mat top, bot, mosaic;
  cv::hconcat(a, b, top);
  cv::hconcat(c, d, bot);
  cv::vconcat(top, bot, mosaic);
  return mosaic;
}

static inline int field_offset(const sensor_msgs::msg::PointCloud2& pc2, const std::string& name) {
  for (const auto& f : pc2.fields) {
    if (f.name == name) return (int)f.offset;
  }
  return -1;
}

static inline bool field_is_float32(const sensor_msgs::msg::PointCloud2& pc2, const std::string& name) {
  for (const auto& f : pc2.fields) {
    if (f.name == name) return (f.datatype == sensor_msgs::msg::PointField::FLOAT32);
  }
  return false;
}

static dataloader::LidarSample extract_xyzi(const sensor_msgs::msg::PointCloud2& pc2) {
  dataloader::LidarSample scan;
  scan.t = dataloader::to_ns(pc2.header.stamp);

  const int ox = field_offset(pc2, "x");
  const int oy = field_offset(pc2, "y");
  const int oz = field_offset(pc2, "z");
  const int oi = field_offset(pc2, "intensity");

  if (ox < 0 || oy < 0 || oz < 0) return scan;
  if (!field_is_float32(pc2, "x") || !field_is_float32(pc2, "y") || !field_is_float32(pc2, "z")) return scan;

  const bool has_i = (oi >= 0) && field_is_float32(pc2, "intensity");

  const size_t npts = (size_t)pc2.width * (size_t)pc2.height;
  scan.pts.resize(npts);

  const uint8_t* base = pc2.data.data();
  const size_t step = pc2.point_step;

  for (size_t i = 0; i < npts; ++i) {
    const uint8_t* p = base + i * step;
    dataloader::PointXYZI q;
    q.x = *reinterpret_cast<const float*>(p + ox);
    q.y = *reinterpret_cast<const float*>(p + oy);
    q.z = *reinterpret_cast<const float*>(p + oz);
    q.intensity = has_i ? *reinterpret_cast<const float*>(p + oi) : 0.0f;
    scan.pts[i] = q;
  }
  return scan;
}

static dataloader::ImageFrame decode_compressed(const sensor_msgs::msg::CompressedImage& ci) {
  dataloader::ImageFrame f;
  f.t = dataloader::to_ns(ci.header.stamp);
  cv::Mat buf(1, (int)ci.data.size(), CV_8UC1, const_cast<uint8_t*>(ci.data.data()));
  // grayscale; change to IMREAD_COLOR if needed
  f.img = cv::imdecode(buf, cv::IMREAD_GRAYSCALE);
  return f;
}

static dataloader::ImuSample to_imu(const sensor_msgs::msg::Imu& imu) {
  dataloader::ImuSample s;
  s.t = dataloader::to_ns(imu.header.stamp);
  s.accel = Eigen::Vector3d(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
  s.gyro  = Eigen::Vector3d(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
  return s;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: dump_packets <bag_path> [max_packets]\n";
    return 1;
  }
  const int max_packets = (argc >= 3) ? std::stoi(argv[2]) : 200;

  rclcpp::init(argc, argv);
 
  cv::namedWindow("cams", cv::WINDOW_NORMAL);
 
  std::thread viz_thread(draw_loop);

  dataloader::TopicConfig topics;
  dataloader::SyncParams params;
  dataloader::Packetizer packetizer(topics, params);

  dataloader::RosBag2Reader reader(argv[1]);

  rclcpp::Serialization<sensor_msgs::msg::CompressedImage> ser_ci;
  rclcpp::Serialization<sensor_msgs::msg::Imu> ser_imu;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> ser_pc2;

  int emitted = 0;

  while (reader.has_next() && emitted < max_packets) {
    auto m = reader.read_next();

    const std::string& topic = m->topic_name;
    rclcpp::SerializedMessage smsg(*m->serialized_data);

    if (topic == topics.cam_front_right || topic == topics.cam_front_left ||
        topic == topics.cam_side_right || topic == topics.cam_side_left) {
      sensor_msgs::msg::CompressedImage ci;
      ser_ci.deserialize_message(&smsg, &ci);
      auto f = decode_compressed(ci);
      if (f.img.empty()) continue;

      if (topic == topics.cam_front_right) packetizer.push_cam_front_right(f);
      else if (topic == topics.cam_front_left) packetizer.push_cam_front_left(f);
      else if (topic == topics.cam_side_right) packetizer.push_cam_side_right(f);
      else if (topic == topics.cam_side_left) packetizer.push_cam_side_left(f);
    }
    else if (topic == topics.imu_cam || topic == topics.imu_lidar) {
      sensor_msgs::msg::Imu imu;
      ser_imu.deserialize_message(&smsg, &imu);
      auto s = to_imu(imu);
      if (topic == topics.imu_cam) packetizer.push_imu_cam(s);
      else packetizer.push_imu_lidar(s);
    }
    else if (topic == topics.lidar_points) {
      sensor_msgs::msg::PointCloud2 pc2;
      ser_pc2.deserialize_message(&smsg, &pc2);
      auto scan = extract_xyzi(pc2);
      packetizer.push_lidar(scan);
    }

    // Flush any packets that are now safe
    while (true) {
      auto pkt_opt = packetizer.pop_packet();
      if (!pkt_opt) break;
      const auto& pkt = *pkt_opt;

	  std::optional<dataloader::ImageFrame> fr, fl, sr, sl;
	  if (pkt.has_cam_front_right) fr = pkt.cam_front_right;
	  if (pkt.has_cam_front_left)  fl = pkt.cam_front_left;
	  if (pkt.has_cam_side_right)  sr = pkt.cam_side_right;
	  if (pkt.has_cam_side_left)   sl = pkt.cam_side_left;
	  
	  cv::Mat mosaic = make_mosaic_2x2(fr, fl, sr, sl);
	  
	  // annotate
	  std::string txt = "t_lidar(ns)=" + std::to_string(pkt.t);
	  cv::putText(mosaic, txt, cv::Point(20, 40),
	              cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255), 2);
	  
	  cv::imshow("cams", mosaic);
	  
	  // controls
	  int key = cv::waitKey(1);        // 1ms; use 0 for step-by-step
	  if (key == 27 || key == 'q') {   // ESC or q
	    rclcpp::shutdown();
	    return 0;
	  }
	  if (key == ' ') {                // space = pause
	    cv::waitKey(0);
	  }

	  std::vector<Pt> pts;
	  pts.reserve(pkt.lidar.pts.size());
	  for (const auto& p : pkt.lidar.pts) pts.push_back({p.x,p.y,p.z,p.intensity});
	  update_points(pts);

      std::cout
        << "pkt t_lidar=" << pkt.t
        << " pts=" << pkt.lidar.pts.size()
        << " camFR=" << (pkt.has_cam_front_right ? "Y" : "N")
        << " camFL=" << (pkt.has_cam_front_left ? "Y" : "N")
        << " camSR=" << (pkt.has_cam_side_right ? "Y" : "N")
        << " camSL=" << (pkt.has_cam_side_left  ? "Y" : "N")
       
	    <<" dtR_ms="<<(pkt.has_cam_front_right ? (std::llabs(pkt.cam_front_right.t - pkt.t) /1e6):-1)
        <<" dtL_ms="<<(pkt.has_cam_front_left ? (std::llabs(pkt.cam_front_left.t - pkt.t) / 1e6) :-1)
        <<" dtL_ms="<<(pkt.has_cam_side_right ? (std::llabs(pkt.cam_side_right.t - pkt.t) / 1e6) :-1)
        <<" dtL_ms="<<(pkt.has_cam_side_left ? (std::llabs(pkt.cam_side_left.t - pkt.t) / 1e6) : -1)
        <<" imuC=" << pkt.imu_cam.size()
		<<" imuL=" << pkt.imu_lidar.size()
        << "\n";

      emitted++;
      if (emitted >= max_packets) break;
    }
  }
  viz_thread.join();	  
  rclcpp::shutdown();
  return 0;
}
