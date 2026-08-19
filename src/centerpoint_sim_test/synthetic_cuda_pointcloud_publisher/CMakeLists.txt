cmake_minimum_required(VERSION 3.14)
project(synthetic_cuda_pointcloud_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cuda_blackboard REQUIRED)
find_package(CUDA REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})

ament_auto_find_build_dependencies()

ament_auto_add_library(synthetic_cuda_pointcloud_publisher_component SHARED
  src/synthetic_cuda_pointcloud_publisher.cpp
)

rclcpp_components_register_node(
  synthetic_cuda_pointcloud_publisher_component
  PLUGIN "synthetic_cuda_pointcloud_publisher::SyntheticCudaPointCloudPublisher"
  EXECUTABLE synthetic_cuda_pointcloud_publisher_node
)

ament_auto_package()
