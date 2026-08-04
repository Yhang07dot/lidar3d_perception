// ROS 2 node wrapper for the C++ surface detector.
//
// Drop-in replacement for lidar3d_bringup's surface_detector.py: same topics,
// same parameter names (so config/lidar_params.yaml works unchanged), same
// MarkerArray output format.
//
// Subscribes:  /patchworkpp/ground, /patchworkpp/nonground   (PointCloud2)
// Publishes:   /obstacles/boxes_3d_surface                   (MarkerArray, high conf)
//              /lidar/low_confidence_surface                 (MarkerArray, debug)
//              /lidar/pothole_markers                        (MarkerArray)

#include "lidar3d_perception_cpp/surface_detector.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <vector>

using lidar3d::Cloud;
using lidar3d::Point3;
using sensor_msgs::msg::PointCloud2;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace
{

// Port of _pc2_to_xyz(). Reads x/y/z straight out of the raw buffer using the
// field offsets, so any point_step / extra fields are handled.
Cloud pc2ToXyz(const PointCloud2 & msg)
{
  Cloud out;
  int off_x = -1, off_y = -1, off_z = -1;
  for (const auto & f : msg.fields) {
    if (f.name == "x") {off_x = static_cast<int>(f.offset);} else if (f.name == "y") {
      off_y = static_cast<int>(f.offset);
    } else if (f.name == "z") {off_z = static_cast<int>(f.offset);}
  }
  if (off_x < 0 || off_y < 0 || off_z < 0) {return out;}

  const size_t n = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
  out.reserve(n);
  const uint8_t * base = msg.data.data();
  for (size_t i = 0; i < n; ++i) {
    const uint8_t * p = base + i * msg.point_step;
    float x, y, z;
    std::memcpy(&x, p + off_x, sizeof(float));
    std::memcpy(&y, p + off_y, sizeof(float));
    std::memcpy(&z, p + off_z, sizeof(float));
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
      out.emplace_back(x, y, z);
    }
  }
  return out;
}

}  // namespace

class SurfaceDetectorNode : public rclcpp::Node
{
public:
  SurfaceDetectorNode()
  : Node("surface_detector")
  {
    // ==== 曲面模型 (Layer 1) ==== — parameter names match surface_detector.py
    declare_parameter("grid_dr_base", 0.10);
    declare_parameter("grid_dr_per_m", 0.02);
    declare_parameter("grid_dth_deg", 1.5);
    declare_parameter("smooth_sigma", 1.0);

    // ==== 残差分析 (Layer 2) ====
    declare_parameter("residual_th_near", 0.15);
    declare_parameter("residual_th_far", 0.40);
    declare_parameter("mad_factor", 3.0);
    declare_parameter("min_cluster_pts", 8);

    // ==== Ground离群点过滤 (方案A) ====
    declare_parameter("ground_outlier_factor", 2.0);

    // ==== 近场曲面不可靠修复 ====
    declare_parameter("nearfield_range", 6.0);

    // ==== 置信度 & 发布 ====
    declare_parameter("confidence_threshold", 0.35);
    declare_parameter("pothole_depth_m", 0.08);

    // ==== 时序追踪 ====
    declare_parameter("track_dist_thr", 2.0);
    declare_parameter("track_hist", 10);
    declare_parameter("track_max_lost", 3);
    declare_parameter("log_interval", 10);

    rclcpp::QoS qos(10);
    qos.best_effort();  // matches the Python node / Gazebo bridge

    sub_ground_ = create_subscription<PointCloud2>(
      "/patchworkpp/ground", qos,
      std::bind(&SurfaceDetectorNode::onGround, this, std::placeholders::_1));
    sub_nonground_ = create_subscription<PointCloud2>(
      "/patchworkpp/nonground", qos,
      std::bind(&SurfaceDetectorNode::onNonground, this, std::placeholders::_1));

    pub_boxes_ = create_publisher<MarkerArray>("/obstacles/boxes_3d_surface", 10);
    pub_low_ = create_publisher<MarkerArray>("/lidar/low_confidence_surface", 10);
    pub_pot_ = create_publisher<MarkerArray>("/lidar/pothole_markers", 10);

    RCLCPP_INFO(
      get_logger(),
      "Surface Detector (C++) ready — polar grid + surface model + residuals");
  }

private:
  void onGround(const PointCloud2::SharedPtr msg)
  {
    const Cloud g = pc2ToXyz(*msg);
    if (g.size() < 20) {return;}

    // build polar surface grid (with outlier filtering, 方案A)
    lidar3d::PolarGrid grid = lidar3d::buildPolarGrid(
      g, 0.5, 35.0,
      get_parameter("grid_dr_base").as_double(),
      get_parameter("grid_dr_per_m").as_double(),
      get_parameter("grid_dth_deg").as_double(),
      get_parameter("ground_outlier_factor").as_double());
    if (!grid.valid()) {return;}

    std::vector<double> S = lidar3d::fillAndSmooth(
      grid, get_parameter("smooth_sigma").as_double());

    // capture points mis-assigned to ground but significantly above the surface
    Cloud outliers;
    outliers.reserve(grid.outlier_indices.size());
    for (int idx : grid.outlier_indices) {outliers.push_back(g[idx]);}

    // ground reference height from the CLEAN surface (not pulled up by obstacles)
    std::vector<double> valid_z;
    valid_z.reserve(S.size());
    for (double v : S) {if (std::isfinite(v)) {valid_z.push_back(v);}}
    double ground_z;
    if (!valid_z.empty()) {
      std::nth_element(valid_z.begin(), valid_z.begin() + valid_z.size() / 2, valid_z.end());
      ground_z = valid_z[valid_z.size() / 2];
    } else {
      double sum = 0.0;
      for (const auto & p : g) {sum += p.z();}
      ground_z = sum / static_cast<double>(g.size());
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      grid_ = std::move(grid);
      surface_ = std::move(S);
      ground_outliers_ = std::move(outliers);
      ground_z_ = ground_z;
      have_surface_ = true;
    }

    // pothole detection
    const auto pots = lidar3d::detectPotholes(
      g, 0.2, get_parameter("pothole_depth_m").as_double());
    if (!pots.empty()) {
      MarkerArray ma;
      const auto now = this->now();
      for (size_t i = 0; i < pots.size(); ++i) {
        Marker m;
        m.header.frame_id = msg->header.frame_id;
        m.header.stamp = now;
        m.ns = "pothole";
        m.id = static_cast<int>(i);
        m.type = Marker::SPHERE;
        m.action = Marker::ADD;
        m.pose.position.x = pots[i].x;
        m.pose.position.y = pots[i].y;
        m.pose.position.z = -pots[i].depth / 2.0;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = 0.3;
        float r, gg, b, a;
        lidar3d::typeColor(lidar3d::TYPE_PASSABLE_HIGH, r, gg, b, a);
        m.color.r = r; m.color.g = gg; m.color.b = b; m.color.a = a;
        m.lifetime = rclcpp::Duration(0, 500000000);
        char buf[64];
        snprintf(buf, sizeof(buf), "pothole_%.0fcm", pots[i].depth * 100.0);
        m.text = buf;
        ma.markers.push_back(m);
      }
      pub_pot_->publish(ma);
    }
  }

  void onNonground(const PointCloud2::SharedPtr msg)
  {
    lidar3d::PolarGrid grid;
    std::vector<double> S;
    Cloud outliers;
    double ground_z = 0.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!have_surface_) {return;}
      grid = grid_;
      S = surface_;
      outliers = ground_outliers_;
      ground_z = ground_z_;
    }

    ++frame_count_;
    Cloud xyz = pc2ToXyz(*msg);

    // 方案A: merge points that Patchwork++ mis-assigned to ground
    xyz.insert(xyz.end(), outliers.begin(), outliers.end());
    if (xyz.size() < 20) {return;}

    // residual analysis with near-field unreliable-surface repair
    const double nearfield_range = get_parameter("nearfield_range").as_double();
    const double th_near = get_parameter("residual_th_near").as_double();
    const double th_far = get_parameter("residual_th_far").as_double();

    std::vector<double> residuals(xyz.size()), thresholds(xyz.size());
    int near_unreliable = 0;
    for (size_t i = 0; i < xyz.size(); ++i) {
      double s_z;
      int cnt;
      lidar3d::sampleSurface(S, grid, xyz[i].x(), xyz[i].y(), s_z, cnt);
      const double r = std::hypot(xyz[i].x(), xyz[i].y());
      // 近场曲面不可靠: Patchwork++盲区 + 16线近场打不到地面 → count≈0，
      // 曲面是插值填充的。用全局地面参考替换，使地面点残差≈0(净化聚类)、
      // 高大障碍物仍能触发检测。
      if (cnt < 3 && r < nearfield_range) {
        s_z = ground_z;
        ++near_unreliable;
      }
      residuals[i] = xyz[i].z() - s_z;
      thresholds[i] = lidar3d::adaptiveThreshold(r, th_near, th_far);
    }

    const auto clusters = lidar3d::clusterResidualPts(
      xyz, residuals, thresholds, get_parameter("min_cluster_pts").as_int());
    if (clusters.empty()) {return;}

    // classify each cluster
    std::vector<lidar3d::Classification> infos;
    std::vector<Point3> cents;
    std::vector<int> tids;
    std::vector<std::string> labels;
    infos.reserve(clusters.size());
    for (const auto & cl : clusters) {
      auto info = lidar3d::classifySurface(cl, ground_z);
      cents.push_back(info.centroid);
      tids.push_back(info.type_id);
      labels.push_back(info.label);
      infos.push_back(std::move(info));
    }

    const auto tracked = tracker_.match(
      cents, tids, labels,
      get_parameter("track_dist_thr").as_double(),
      static_cast<int>(get_parameter("track_hist").as_int()),
      static_cast<int>(get_parameter("track_max_lost").as_int()));

    const double conf_thr = get_parameter("confidence_threshold").as_double();
    const int log_interval = static_cast<int>(get_parameter("log_interval").as_int());
    const bool do_log = (log_interval > 0) && (frame_count_ % log_interval == 0);

    MarkerArray high, low;
    std::string log_line;
    const auto now = this->now();

    for (size_t i = 0; i < infos.size(); ++i) {
      const auto & info = infos[i];
      const int st = tracked[i].type_id;
      const std::string & sl = tracked[i].label;
      const int n_pts = static_cast<int>(clusters[i].size());

      const double conf = lidar3d::confidenceSurface(
        n_pts, info.verticality, info.edge_ratio, tracked[i].hist);
      const bool hi = conf >= conf_thr;

      if (do_log) {
        char buf[192];
        snprintf(
          buf, sizeof(buf), "[%s] N=%d c=%.2f H=%.2fm → %s | ",
          lidar3d::typeLabel(st), n_pts, conf, info.dims.z(), sl.c_str());
        log_line += buf;
      }

      Marker box;
      box.header.frame_id = msg->header.frame_id;
      box.header.stamp = now;
      box.ns = lidar3d::typeLabel(st);
      box.id = static_cast<int>(i);
      box.type = Marker::CUBE;
      box.action = Marker::ADD;
      box.pose.position.x = info.centroid.x();
      box.pose.position.y = info.centroid.y();
      box.pose.position.z = info.centroid.z();
      box.pose.orientation.w = 1.0;
      box.scale.x = info.dims.x();
      box.scale.y = info.dims.y();
      box.scale.z = info.dims.z();
      float r, g, b, a;
      lidar3d::typeColor(st, r, g, b, a);
      box.color.r = r; box.color.g = g; box.color.b = b;
      box.color.a = hi ? a : 0.25f;
      char tbuf[192];
      snprintf(tbuf, sizeof(tbuf), "%s c=%.2f", sl.c_str(), conf);
      box.text = tbuf;
      box.lifetime = rclcpp::Duration(0, 300000000);

      (hi ? high : low).markers.push_back(box);
    }

    pub_boxes_->publish(high);
    if (!low.markers.empty()) {pub_low_->publish(low);}
    if (do_log && !log_line.empty()) {
      RCLCPP_INFO(
        get_logger(), "F%d: %snear_unrel=%d",
        frame_count_, log_line.c_str(), near_unreliable);
    }
  }

  rclcpp::Subscription<PointCloud2>::SharedPtr sub_ground_, sub_nonground_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_boxes_, pub_low_, pub_pot_;

  std::mutex state_mutex_;
  lidar3d::PolarGrid grid_;
  std::vector<double> surface_;
  Cloud ground_outliers_;
  double ground_z_ = 0.0;
  bool have_surface_ = false;

  lidar3d::Tracker tracker_;
  int frame_count_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SurfaceDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
