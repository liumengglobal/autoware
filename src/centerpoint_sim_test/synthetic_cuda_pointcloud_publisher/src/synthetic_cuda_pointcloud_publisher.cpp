#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <cuda_blackboard/cuda_blackboard_publisher.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace synthetic_cuda_pointcloud_publisher
{

class SyntheticCudaPointCloudPublisher : public rclcpp::Node
{
public:
  explicit SyntheticCudaPointCloudPublisher(const rclcpp::NodeOptions & options)
  : Node("synthetic_cuda_pointcloud_publisher", options), rng_(42)
  {
    pub_ = std::make_unique<
      cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      *this, "/sensing/lidar/pointcloud");
    relay_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/sensing/lidar/pointcloud/relay", 10);
    pointcloud_msg_ = buildPointCloud();
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), [this]() { this->publishCloud(); });
    RCLCPP_INFO(
      this->get_logger(),
      "Synthetic CUDA pointcloud publisher on /sensing/lidar/pointcloud (10 Hz) + reliable relay on "
      "/sensing/lidar/pointcloud/relay (deterministic point cloud)");
  }

private:
  void publishCloud()
  {
    auto ros_msg = pointcloud_msg_;
    ros_msg.header.stamp = this->now();
    auto cuda_msg = std::make_unique<cuda_blackboard::CudaPointCloud2>(ros_msg);
    pub_->publish(std::move(cuda_msg));
    relay_pub_->publish(ros_msg);
  }

  float uniform(float lo, float hi)
  {
    std::uniform_real_distribution<float> d(lo, hi);
    return d(rng_);
  }

  void addPoint(
    std::vector<uint8_t> & buf, float x, float y, float z, uint8_t intensity, uint8_t ret,
    uint16_t ch)
  {
    // 16 bytes per point: x,y,z (float32) + intensity,return_type (uint8) + channel (uint16)
    uint8_t tmp[16];
    std::memcpy(tmp + 0, &x, 4);
    std::memcpy(tmp + 4, &y, 4);
    std::memcpy(tmp + 8, &z, 4);
    tmp[12] = intensity;
    tmp[13] = ret;
    std::memcpy(tmp + 14, &ch, 2);
    buf.insert(buf.end(), tmp, tmp + 16);
  }

  sensor_msgs::msg::PointCloud2 buildPointCloud()
  {
    std::vector<uint8_t> buf;

    // Car 1: box centered at (15, 0), ~4.5 x 1.8 x 1.5 m
    for (int i = 0; i < 2000; ++i) {
      addPoint(buf, uniform(12.75f, 17.25f), uniform(-0.9f, 0.9f), uniform(0.0f, 1.5f), 120, 1, 0);
    }
    // Car 2: box centered at (30, 3.5)
    for (int i = 0; i < 8000; ++i) {
      addPoint(buf, uniform(28.0f, 32.5f), uniform(2.6f, 4.4f), uniform(0.0f, 1.5f), 120, 1, 0);
    }
    // Ground ring points
    for (int i = 0; i < 3000; ++i) {
      const float theta = uniform(0.0f, 2.0f * M_PI);
      const float r = uniform(5.0f, 40.0f);
      addPoint(buf, r * std::cos(theta), r * std::sin(theta), uniform(-0.05f, 0.05f), 30, 0, 0);
    }
    // Random noise points
    for (int i = 0; i < 500; ++i) {
      addPoint(buf, uniform(-40.0f, 40.0f), uniform(-40.0f, 40.0f), uniform(-1.0f, 3.0f), 10, 0, 0);
    }

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "lidar_link";
    msg.height = 1;
    msg.width = static_cast<uint32_t>(buf.size() / 16);
    msg.is_bigendian = false;
    msg.point_step = 16;
    msg.row_step = msg.point_step * msg.width;
    msg.is_dense = true;

    sensor_msgs::msg::PointField f;
    f.name = "x"; f.offset = 0; f.datatype = sensor_msgs::msg::PointField::FLOAT32; f.count = 1;
    msg.fields.push_back(f);
    f.name = "y"; f.offset = 4; f.datatype = sensor_msgs::msg::PointField::FLOAT32; f.count = 1;
    msg.fields.push_back(f);
    f.name = "z"; f.offset = 8; f.datatype = sensor_msgs::msg::PointField::FLOAT32; f.count = 1;
    msg.fields.push_back(f);
    f.name = "intensity"; f.offset = 12; f.datatype = sensor_msgs::msg::PointField::UINT8; f.count = 1;
    msg.fields.push_back(f);
    f.name = "return_type"; f.offset = 13; f.datatype = sensor_msgs::msg::PointField::UINT8; f.count = 1;
    msg.fields.push_back(f);
    f.name = "channel"; f.offset = 14; f.datatype = sensor_msgs::msg::PointField::UINT16; f.count = 1;
    msg.fields.push_back(f);

    msg.data = buf;
    return msg;
  }

  std::unique_ptr<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>> pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr relay_pub_;
  sensor_msgs::msg::PointCloud2 pointcloud_msg_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mt19937 rng_;
};

}  // namespace synthetic_cuda_pointcloud_publisher

RCLCPP_COMPONENTS_REGISTER_NODE(
  synthetic_cuda_pointcloud_publisher::SyntheticCudaPointCloudPublisher)
