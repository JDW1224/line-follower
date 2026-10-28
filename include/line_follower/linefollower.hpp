#ifndef LINEFOLLOWER_HPP
#define LINEFOLLOWER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class LineFollower : public rclcpp::Node
{
public:
    LineFollower();
    ~LineFollower();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void processImage(const cv::Mat& image);
    double computeAngularCommand(double signed_error, bool valid);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr roi_debug_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr line_mask_publisher_;
    rclcpp::TimerBase::SharedPtr gui_timer_;
    rclcpp::TimerBase::SharedPtr start_timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bird_eye_debug_publisher_;

    // 이전 프레임에서의 중심 (fallback)
    double prev_x_;
    double prev_y_;

    double max_linear_speed_;

    // Lateral (angular) PID
    double lat_kp_;
    double lat_ki_;
    double lat_kd_;
    double lat_integral_;
    double lat_prev_error_;
    double lat_integral_limit_;
    double max_angular_speed_;
    double last_angular_cmd_;
    int binary_threshold_;
    int blur_kernel_size_;
    bool use_gui_;
    int lost_frame_count_;
    int reverse_frame_limit_;
    double last_corner_direction_;
    double reverse_speed_;
    double lost_forward_speed_;
    double recovery_turn_gain_;
    double corner_detect_threshold_;
    
    // 90도 회전 관련 변수
    rclcpp::Time start_time_;
    rclcpp::Time rotation_start_time_;
    double initial_yaw_;
    double current_yaw_;
    double target_yaw_;
    bool is_rotating_;
    bool rotation_triggered_;
    bool initial_yaw_set_;
    int rotation_stage_;
    
    // HSV controls
    int h_min_;
    int h_max_;
    int s_min_;
    int s_max_;
    int v_min_;
    int v_max_;
    std::string control_window_name_;

    // Trackbar callbacks
    static void onHMinChange(int pos, void* userdata);
    static void onHMaxChange(int pos, void* userdata);
    static void onSMinChange(int pos, void* userdata);
    static void onSMaxChange(int pos, void* userdata);
    static void onVMinChange(int pos, void* userdata);
    static void onVMaxChange(int pos, void* userdata);
    static void onBinaryThresholdChange(int pos, void* userdata);
    static void onBlurKernelChange(int pos, void* userdata);
};

#endif // LINEFOLLOWER_HPP
