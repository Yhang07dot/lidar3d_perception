// Patchwork++ and Patchwork classic
#include "patchwork/patchwork.h"
#include "patchwork/patchworkpp.h"

// Standard library
#include <string>
#include <variant>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

namespace patchworkpp_ros {

class GroundSegmentationServer : public rclcpp::Node {
 public:
  /// GroundSegmentationServer constructor
  GroundSegmentationServer() = delete;
  explicit GroundSegmentationServer(const rclcpp::NodeOptions &options);

 private:
  /// Register new frame
  void EstimateGround(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

  /// Stream the point clouds for visualization
  void PublishClouds(const Eigen::MatrixX3f &est_ground,
                     const Eigen::MatrixX3f &est_nonground,
                     const std_msgs::msg::Header header_msg);

  /// Parameter loaders — only the selected algorithm's loader is called
  patchwork::Params loadPlusplusParamsFromROS();
  patchwork::PatchworkParams loadClassicParamsFromROS();

 private:
  /// Data subscribers.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  /// Data publishers.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_publisher_;

  /// Algorithm implementation (patchworkpp or patchwork classic)
  using ImplVariant =
      std::variant<std::unique_ptr<patchwork::PatchWorkpp>, std::unique_ptr<patchwork::PatchWork>>;
  ImplVariant impl_;

  std::string base_frame_{"base_link"};
};

}  // namespace patchworkpp_ros
