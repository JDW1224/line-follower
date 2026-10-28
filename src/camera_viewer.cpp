#include <algorithm>
#include <chrono>

#include "line_follower/camera_viewer.hpp"

using namespace std::chrono_literals;

namespace line_follower
{

CameraViewer::CameraViewer()
: rclcpp::Node("camera_viewer")
{
  subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image_raw",
    rclcpp::SensorDataQoS(),
    std::bind(&CameraViewer::handle_image, this, std::placeholders::_1));

  publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
    "/camera/image_upright",
    rclcpp::SensorDataQoS());

  RCLCPP_INFO(get_logger(), "Listening on /camera/image_raw and publishing /camera/image_upright");
}

void CameraViewer::handle_image(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (msg->data.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Empty camera frame received.");
    return;
  }

  auto flipped = std::make_shared<sensor_msgs::msg::Image>(*msg);
  flipped->data.resize(msg->data.size());

  const std::size_t stride = static_cast<std::size_t>(msg->step);
  const std::size_t height = static_cast<std::size_t>(msg->height);

  for (std::size_t row = 0; row < height; ++row) {
    auto src_begin = msg->data.cbegin() + row * stride;
    auto src_end = src_begin + stride;
    auto dst_begin = flipped->data.begin() + (height - 1 - row) * stride;
    std::reverse_copy(src_begin, src_end, dst_begin);
  }

  publisher_->publish(*flipped);
  // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    // "Published upright image %ux%u", msg->width, msg->height);
}

}  // namespace line_follower

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<line_follower::CameraViewer>());
  rclcpp::shutdown();
  return 0;
}