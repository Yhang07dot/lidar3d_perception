cmake_minimum_required(VERSION 3.8)
project(lidar3d_perception_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-O3 -Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(surface_detector_node src/surface_detector_node.cpp)
target_include_directories(surface_detector_node PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(surface_detector_node
  rclcpp
  sensor_msgs
  visualization_msgs
  std_msgs
)

install(TARGETS surface_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
