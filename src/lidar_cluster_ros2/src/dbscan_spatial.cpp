// Non-grid (spatial) DBSCAN filter for point cloud data
// The DBSCAN (Density-Based Spatial Clustering of Applications with Noise) algorithm is a popular clustering algorithm in machine learning

#include <functional>
#include <memory>
#include <cmath>
#include <vector>
#include <queue>
#include <algorithm>
// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
// ROS package
#include "lidar_cluster/marker.hpp"
// Benchmarking
#include "benchmark.hpp"
// TBB
#include <tbb/tbb.h>

#include "cluster_outline.hpp"

// Point class, which stores the x and y coordinates of a point, the number of neighbors, whether it is a core point, and the cluster ID
class Point {
public:
  double x, y;
  int neighbor_pts = 0;
  bool core = false;
  int cluster_id = -1;
  /*
      cluster_id is the cluster ID of the point:
      -1: undecided values, not assigned to any cluster
       0: unassigned values: already visited, but not assigned to any cluster
       1: assigned to the first cluster
       2: assigned to the second cluster and so on
  */
};

// Function to calculate the Euclidean distance between two points
double distance(const Point &p1, const Point &p2) {
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return std::sqrt(dx * dx + dy * dy);
}

// For neighbor searching, instead of brute force, we use a KD-tree
// KD-tree node, which stores a point and pointers to left and right children
struct Node {
  Point point;
  Node* left;
  Node* right;
};

// Build a KD-tree from a given point cloud(vector of points)
Node* build_kdtree(std::vector<Point>::iterator begin, std::vector<Point>::iterator end, int depth = 0) {

  // If the point cloud is empty, return nullptr
  if (begin >= end) {
    return nullptr;
  }

  // Determine the axis based on the depth and find the middle point of the range
  int axis = depth % 2;
  auto mid = begin + (end - begin) / 2;

  // Use std::nth_element to partition the points around the median.
  // std::nth_element is a partial sorting algorithm that rearranges the elements in the range
  // such that the element at the 'mid' position is the one that would be there if the entire range was sorted.
  // All elements before 'mid' are no greater than mid, and all elements after 'mid' are no less than mid.
  // This allows us to find the median point in linear time, which is efficient for large datasets.
  std::nth_element(begin, mid, end, [axis](const Point& a, const Point& b) {
    if (axis == 0) {
      return a.x < b.x;  // Compare 'x' coordinates if axis is 0
    } else {
      return a.y < b.y;  // Compare 'y' coordinates if axis is 1
    }
  });

  // Create a new node and assign the median point
  Node* node = new Node;
  node->point = *mid;

  // Recursively build the left and right subtrees with a tbb::task_group
  // a tbb::task_group is a task scheduler that can run tasks in parallel
  // TODO there may be better and cleaner ways to parallelize this
  tbb::task_group tg;
  // the left children is started on a new thread
  tg.run([&]() {
    node->left = build_kdtree(begin, mid, depth + 1);
  });
  // the right children is started on the main thread
  node->right = build_kdtree(mid + 1, end, depth + 1);
  tg.wait(); // we wait for the left children to finish, before we continue

  return node;
}

// Search for neighbors within a radius eps of a given point
void radius_search(Node* node, const Point& search_point, double eps, std::vector<Point>& neighbors, int depth = 0) {
  if (node == nullptr || neighbors.size() >= 3) { // if we have enough neighbors, we can stop
    return;
  }

  // If the point is within eps of the search point, add it to the neighbors
  double dist = distance(node->point, search_point);
  if (dist <= eps) {
    neighbors.push_back(node->point);
  }

  // Recursively search the left and right subtrees
  int axis = depth % 2;
  double diff;
  if (axis == 0) {
    diff = search_point.x - node->point.x;
  } else {
    diff = search_point.y - node->point.y;
  }
  double eps_sqr = eps * eps;

  // Choose the near and far subtrees based on the search point
  // Basically, if the search point is to the left of the node, the near subtree is the left subtree
  Node* near;
  Node* far;
  if (diff <= 0) {
    near = node->left;
    far = node->right;
  } else {
    near = node->right;
    far = node->left;
  }

  // Recursively search the near subtree
  radius_search(near, search_point, eps, neighbors, depth + 1);

  // If the search point is within eps of the node, search the far subtree
  if (diff * diff < eps_sqr) {
    radius_search(far, search_point, eps, neighbors, depth + 1);
  }
}

// Find neighbors for each point in the point cloud
void find_neighbors(std::vector<Point>& points, double eps) {
  // Build a KD-tree from the points
  Node* root = build_kdtree(points.begin(), points.end());

  // Search for neighbors for each point in the point cloud
  // We use tbb::parallel_for to parallelize the loop, as the loop iterations are independent
  tbb::parallel_for(tbb::blocked_range<size_t>(0, points.size()), [&](const tbb::blocked_range<size_t>& r) {
    for (size_t i = r.begin(); i < r.end(); ++i) {
      Point& p = points[i];
      std::vector<Point> neighbors;
      radius_search(root, p, eps, neighbors);
      p.neighbor_pts = neighbors.size();
      p.core = p.neighbor_pts >= 3;
    }
  });

  // Free the KD-tree nodes
  std::function<void(Node*)> free_tree = [&](Node* node) {
    if (node != nullptr) {
      free_tree(node->left);
      free_tree(node->right);
      delete node;
    }
  };
  free_tree(root);
}

// Function to expand a cluster from a given point
void expand_cluster(Point& p, int cluster_id, double eps, std::vector<Point>& points) {
  // Create a queue to hold points that are part of the cluster
  std::queue<Point*> qu;
  // Add the initial point to the queue
  qu.push(&p);
  // Assign the cluster ID to the initial point
  p.cluster_id = cluster_id;

  // Continue until there are no more points in the cluster
  while (!qu.empty()) {
    // Get the next point from the queue
    Point* current = qu.front();
    // Remove the point from the queue
    qu.pop();
    
    // Iterate over all points
    for (Point &q : points) {
      // If the point has not been assigned to a cluster and is within 'eps' distance of the current point
      if (q.cluster_id == -1 && current->x >= q.x - eps && current->x <= q.x + eps && current->y >= q.y - eps && current->y <= q.y + eps) { // here, instead of the distance function, we use a bounding box to filter the points, as this is faster, while sacraficing minimal accuracy
        // Assign the point to the current cluster
        q.cluster_id = cluster_id;
        // If the point is a core point, add it to the queue, so so we can also expand the cluster from this point
        if (q.core) {
          qu.push(&q);
        }
      }
    }
  }
}

// Function to find all clusters in the given points
int find_clusters(std::vector<Point>& points, double eps) {
  // Initialize the cluster ID
  int actual_cluster_id = 0;

  // Iterate over all points
  for (Point &p : points) {
    // If the point is a core point and has not been assigned to a cluster
    if (p.core && p.cluster_id == -1) {
      // Expand a new cluster from the point
      expand_cluster(p, actual_cluster_id, eps, points);
      // Increment the cluster ID for the next cluster
      actual_cluster_id++;
    }
  }

  // Return the number of clusters found
  return actual_cluster_id;
}

class DbscanSpatial : public rclcpp::Node
{
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto &param : parameters)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Param update: " << param.get_name().c_str() << ": " << param.value_to_string().c_str());
      if (param.get_name() == "minX")
      {
        minX = param.as_double();
      }
      if (param.get_name() == "minY")
      {
        minY = param.as_double();
      }
      if (param.get_name() == "minZ")
      {
        minZ = param.as_double();
      }
      if (param.get_name() == "maxX")
      {
        maxX = param.as_double();
      }
      if (param.get_name() == "maxY")
      {
        maxY = param.as_double();
      }
      if (param.get_name() == "maxZ")
      {
        maxZ = param.as_double();
      }
      if (param.get_name() == "points_in_topic")
      {
        points_in_topic = param.as_string();
        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(points_in_topic, rclcpp::SensorDataQoS().keep_last(1), std::bind(&DbscanSpatial::lidar_callback, this, std::placeholders::_1));
      }
      if (param.get_name() == "points_out_topic")
      {
        points_out_topic = param.as_string();
        pub_lidar_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(points_out_topic, 10);
      }
      if (param.get_name() == "marker_out_topic")
      {
        marker_out_topic = param.as_string();
        pub_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_out_topic, 10);
      }
      if (param.get_name() == "verbose1")
      {
        verbose1 = param.as_bool();
      }
      if (param.get_name() == "verbose2")
      {
        verbose2 = param.as_bool();
      }
      if (param.get_name() == "pub_undecided")
      {
        pub_undecided = param.as_bool();
      }
    }
    return result;
  }

public:
  DbscanSpatial() : Node("dbscan_spatial"), count_(0)
  {
    this->declare_parameter<float>("minX", minX);
    this->declare_parameter<float>("minY", minY);
    this->declare_parameter<float>("minZ", minZ);
    this->declare_parameter<float>("maxX", maxX);
    this->declare_parameter<float>("maxY", maxY);
    this->declare_parameter<float>("maxZ", maxZ);
    this->declare_parameter<std::string>("points_in_topic", "/lexus3/os_center/points");
    this->declare_parameter<std::string>("points_out_topic", "clustered_points");
    this->declare_parameter<std::string>("marker_out_topic", "clustered_marker");
    this->declare_parameter<bool>("verbose1", verbose1);
    this->declare_parameter<bool>("verbose2", verbose2);
    this->declare_parameter<bool>("pub_undecided", pub_undecided);
    this->declare_parameter<double>("eps", eps);

    this->get_parameter("minX", minX);
    this->get_parameter("minY", minY);
    this->get_parameter("minZ", minZ);
    this->get_parameter("maxX", maxX);
    this->get_parameter("maxY", maxY);
    this->get_parameter("maxZ", maxZ);
    this->get_parameter("points_in_topic", points_in_topic);
    this->get_parameter("points_out_topic", points_out_topic);
    this->get_parameter("marker_out_topic", marker_out_topic);
    this->get_parameter("verbose1", verbose1);
    this->get_parameter("verbose2", verbose2);
    this->get_parameter("pub_undecided", pub_undecided);
    this->get_parameter("eps", eps);

    pub_lidar_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(points_out_topic, 10);
    pub_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_out_topic, 10);
    // TODO: QoS // rclcpp::SensorDataQoS().keep_last(1)
    sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(points_in_topic, 10, std::bind(&DbscanSpatial::lidar_callback, this, std::placeholders::_1));
    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&DbscanSpatial::parametersCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "DbscanSpatial node has been started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: '%s'", points_in_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: '%s' and '%s'", points_out_topic.c_str(), marker_out_topic.c_str());
  }

private:
  benchmark::Timer fullbenchmark;
  benchmark::Timer bench1;
  benchmark::Timer bench2;
  benchmark::Timer bench3;
  benchmark::Timer bench4;
  void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
  {
    fullbenchmark.start("fullbenchmark", verbose2);
    bench1.start("pcl data and cropbox", verbose2);

    visualization_msgs::msg::MarkerArray mark_array;
    // Convert to PCL data type
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input_msg, *cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    int original_size = cloud->width * cloud->height;

    // Filter out points outside of the box
    pcl::CropBox<pcl::PointXYZI> crop;
    crop.setInputCloud(cloud);
    crop.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    crop.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    crop.filter(*cloud);

    bench1.finish();

    bench2.start("find_neighbors", verbose2);

    if (verbose1)
    {
      // print the length of the pointcloud
      RCLCPP_INFO_STREAM(this->get_logger(), "PointCloud in: " << original_size << " reduced size before cluster: " << cloud->width * cloud->height);
    }

    // DBSCAN
    // find neighbors in cloud
    std::vector<Point> points;

    for (const pcl::PointXYZI &p : cloud->points)
    {
      {
        Point point;
        point.x = p.x;
        point.y = p.y;
        points.push_back(point);
      }
    }

    find_neighbors(points, eps);

    bench2.finish();

    bench3.start("find_clusters", verbose2);
    // find clusters in cloud
    int num_of_clusters = find_clusters(points, eps);

    bench3.finish();
    bench4.start("publishing", verbose2);

    // create a vector of points for each cluster
    std::vector<double> center_x(num_of_clusters + 1), center_y(num_of_clusters + 1);
    std::vector<int> count(num_of_clusters + 1);
    // init
    for (int i = 0; i <= num_of_clusters; i++)
    {
      center_x[i] = 0.0;
      center_y[i] = 0.0;
      count[i] = 0;
    }

    // convert to PointXYZI
    for (const Point &p : points)
    {
      // undecided and unassigned (cluster_id -1 and 0) points are not published if pub_undecided is false
      if (p.cluster_id > 0 or pub_undecided)
      {
        pcl::PointXYZI point;
        point.x = p.x;
        point.y = p.y;
        point.z = 0.0;
        point.intensity = p.cluster_id;
        center_x[p.cluster_id + 1] += p.x;
        center_y[p.cluster_id + 1] += p.y;
        count[p.cluster_id + 1]++;
        cloud_filtered->points.push_back(point);
      }
    }
    for (int i = 1; i <= num_of_clusters; i++)
    {
      if (count[i] > 0) {
        center_x[i] /= count[i];
        center_y[i] /= count[i];
        visualization_msgs::msg::Marker center_marker;
        init_center_marker(center_marker, center_x[i], center_y[i], i);
        center_marker.header.frame_id = input_msg->header.frame_id;
        center_marker.header.stamp = this->now();
        mark_array.markers.push_back(center_marker);
      }
    }

    // Add markers for clusters that are not present in the current frame to avoid ghost markers
    for(int i = num_of_clusters + 1; i <= max_clust_reached; i++) {
      visualization_msgs::msg::Marker center_marker;
      init_center_marker(center_marker, 0, 0, i);
      center_marker.header.frame_id = input_msg->header.frame_id;
      center_marker.header.stamp = this->now();
      center_marker.color.a = 0.0;
      mark_array.markers.push_back(center_marker);
    }

    cluster_outline.computeOutline(cloud_filtered, mark_array, 20, max_clust_reached, input_msg->header.frame_id);

    // Convert to ROS data type
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_filtered, output_msg);
    // Add the same frame_id as the input, it is not included in pcl PointXYZI
    output_msg.header.frame_id = input_msg->header.frame_id;
    // Publish the data as a ROS message
    pub_lidar_->publish(output_msg);
    pub_marker_->publish(mark_array);


    bench4.finish();

    fullbenchmark.finish();

  } // DbScanSpatial::lidar_callback

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  outline::ClusterOutline cluster_outline;
  float minX = -80.0, minY = -25.0, minZ = -2.0;
  float maxX = +80.0, maxY = +25.0, maxZ = -0.15;
  double eps = 3.5;
  bool verbose1 = false, verbose2 = false, pub_undecided = false;
  std::string points_in_topic, points_out_topic, marker_out_topic;
  size_t count_;
  int max_clust_reached = 0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DbscanSpatial>());
  rclcpp::shutdown();
  return 0;
}