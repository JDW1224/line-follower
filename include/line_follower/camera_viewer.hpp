#ifndef LINE_FOLLOWER__CAMERA_VIEWER_HPP_
#define LINE_FOLLOWER__CAMERA_VIEWER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace line_follower
{

class CameraViewer : public rclcpp::Node
{
public:
  CameraViewer();

private:
  void handle_image(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER__CAMERA_VIEWER_HPP_