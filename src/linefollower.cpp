#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "line_follower/linefollower.hpp"

static const double MAX_LINEAR_SPEED = 0.26;   // TurtleBot3 spec
static const double MAX_ANGULAR_SPEED = 1.82;  // TurtleBot3 spec
static const int ROI_HEIGHT         = 100;
static const int ROI_BOTTOM_WIDTH   = 960;
static const int ROI_TOP_WIDTH      = 200;
static const double corner_speed    = 0.5;

/*ToDo List
1. PID 튜닝
2. 직선구간 감지시 속도 2배
3. 코너 인식 시 일반 직선 속도 복귀
4. 코너링 속도 조절
*/

LineFollower::LineFollower() : Node("line_follower")
{
    prev_x_ = 480.0; // 초기값 (가운데 근처)
    prev_y_ = 270.0;
    lost_frame_count_ = 0;
    reverse_frame_limit_ = 8;       // frames to back off when line is lost
    last_corner_direction_ = 0.0;   // sign of last strong turn
    reverse_speed_ = -0.05;         // slow reverse speed during recovery
    lost_forward_speed_ = 0.08;     // crawl forward after reversing
    recovery_turn_gain_ = 0.6;      // steer toward last corner direction
    corner_detect_threshold_ = 0.35; // angular magnitude considered a corner
    
    // 90도 회전 관련 초기화
    start_time_ = this->now();
    rotation_start_time_ = this->now();
    initial_yaw_ = 0.0;
    current_yaw_ = 0.0;
    target_yaw_ = 0.0;
    is_rotating_ = false;
    rotation_triggered_ = false;
    initial_yaw_set_ = false;
    rotation_stage_ = 0;

    max_linear_speed_ = MAX_LINEAR_SPEED;

    lat_kp_ = 4.0;
    lat_ki_ = 0.0;
    lat_kd_ = 2.2;
    lat_integral_ = 0.0;
    lat_prev_error_ = 0.0;
    lat_integral_limit_ = 1.0;
    max_angular_speed_ = MAX_ANGULAR_SPEED;
    last_angular_cmd_ = 0.0;
    binary_threshold_ = 157;
    blur_kernel_size_ = 10; // odd size required for GaussianBlur

    auto qos = rclcpp::SensorDataQoS();
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_upright", qos,
        std::bind(&LineFollower::imageCallback, this, std::placeholders::_1));
    
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&LineFollower::odomCallback, this, std::placeholders::_1));

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    roi_debug_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/line_follower/roi_debug", rclcpp::SensorDataQoS());
    line_mask_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/line_follower/line_mask", rclcpp::SensorDataQoS());
    RCLCPP_INFO(this->get_logger(), "Line follower with angular PID ready for TurtleBot3 limits");

    // Initialize HSV control values
    control_window_name_ = "HSV Controls";
    h_min_ = 0;
    h_max_ = 54;
    s_min_ = 0;
    s_max_ = 117;
    v_min_ = 0;
    v_max_ = 157; // focus on darker value range (wood floor + black line)

    // Create OpenCV control window and trackbars
    cv::namedWindow(control_window_name_, cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("H min", control_window_name_, nullptr, 179, onHMinChange, this);
    cv::createTrackbar("H max", control_window_name_, nullptr, 179, onHMaxChange, this);
    cv::createTrackbar("S min", control_window_name_, nullptr, 255, onSMinChange, this);
    cv::createTrackbar("S max", control_window_name_, nullptr, 255, onSMaxChange, this);
    cv::createTrackbar("V min", control_window_name_, nullptr, 255, onVMinChange, this);
    cv::createTrackbar("V max", control_window_name_, nullptr, 255, onVMaxChange, this);
    cv::createTrackbar("Binary thresh", control_window_name_, nullptr, 255, onBinaryThresholdChange, this);
    cv::createTrackbar("Blur ksize", control_window_name_, nullptr, 15, onBlurKernelChange, this);

    // Set initial positions
    cv::setTrackbarPos("H min", control_window_name_, h_min_);
    cv::setTrackbarPos("H max", control_window_name_, h_max_);
    cv::setTrackbarPos("S min", control_window_name_, s_min_);
    cv::setTrackbarPos("S max", control_window_name_, s_max_);
    cv::setTrackbarPos("V min", control_window_name_, v_min_);
    cv::setTrackbarPos("V max", control_window_name_, v_max_);
    cv::setTrackbarPos("Binary thresh", control_window_name_, binary_threshold_);
    cv::setTrackbarPos("Blur ksize", control_window_name_, blur_kernel_size_);

    // Timer for GUI event processing
    gui_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
        cv::waitKey(1);
    });
}

// Trackbar callback definitions
void LineFollower::onHMinChange(int pos, void* userdata) {
    static_cast<LineFollower*>(userdata)->h_min_ = pos;
}
void LineFollower::onHMaxChange(int pos, void* userdata) {
    static_cast<LineFollower*>(userdata)->h_max_ = pos;
}
void LineFollower::onSMinChange(int pos, void* userdata) {
    static_cast<LineFollower*>(userdata)->s_min_ = pos;
}
void LineFollower::onSMaxChange(int pos, void* userdata) {
    static_cast<LineFollower*>(userdata)->s_max_ = pos;
}
void LineFollower::onVMinChange(int pos, void* userdata) {
    static_cast<LineFollower*>(userdata)->v_min_ = pos;
}
void LineFollower::onVMaxChange(int pos, void* userdata) {
    static_cast<LineFollower*>(userdata)->v_max_ = pos;
}
void LineFollower::onBinaryThresholdChange(int pos, void* userdata) {
    static_cast<LineFollower*>(userdata)->binary_threshold_ = pos;
}
void LineFollower::onBlurKernelChange(int pos, void* userdata) {
    // Ensure odd and at least 1
    int k = std::max(1, pos | 1);
    static_cast<LineFollower*>(userdata)->blur_kernel_size_ = k;
}

LineFollower::~LineFollower() {
    cv::destroyWindow(control_window_name_);
}

void LineFollower::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Quaternion을 Yaw로 변환
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_yaw_ = yaw;
    
    // 처음 출발 시 방향 저장 (한 번만)
    if (!initial_yaw_set_) {
        initial_yaw_ = yaw;
        initial_yaw_set_ = true;
        RCLCPP_INFO(this->get_logger(), "Initial yaw set to: %.2f rad", initial_yaw_);
    }
}

void LineFollower::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        cv_bridge::CvImagePtr cv_ptr;
        cv::Mat bgr_image;
        if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
            cv::cvtColor(cv_ptr->image, bgr_image, cv::COLOR_GRAY2BGR);
        } else {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            bgr_image = cv_ptr->image;
        }
        processImage(bgr_image);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void LineFollower::processImage(const cv::Mat &image){
    if (image.empty()) return;

    int height = image.rows;
    int width  = image.cols;

    int roi_h = std::min(ROI_HEIGHT, height);
    cv::Rect roi(0, height - roi_h, width, roi_h);
    cv::Mat roi_img = image(roi);

    const int bottom_width = std::min(width, ROI_BOTTOM_WIDTH);
    const int top_width = std::min(width, ROI_TOP_WIDTH);
    const int bottom_offset = std::max(0, (width - bottom_width) / 2);
    const int top_offset = std::max(0, (width - top_width) / 2);

    std::array<cv::Point, 4> trapezoid_pts = {
        cv::Point(bottom_offset, roi_h - 1),
        cv::Point(width - bottom_offset - 1, roi_h - 1),
        cv::Point(width - top_offset - 1, 0),
        cv::Point(top_offset, 0)
    };

    cv::Mat trapezoid_mask = cv::Mat::zeros(roi_h, width, CV_8UC1);
    cv::fillConvexPoly(trapezoid_mask, std::vector<cv::Point>(trapezoid_pts.begin(), trapezoid_pts.end()), cv::Scalar(255));

    cv::Mat masked_roi;
    cv::bitwise_and(roi_img, roi_img, masked_roi, trapezoid_mask);

    // Smooth small floor textures before thresholding
    cv::Mat blurred;
    cv::GaussianBlur(masked_roi, blurred, cv::Size(blur_kernel_size_, blur_kernel_size_), 0);

    // Convert to HSV and produce binary mask using HSV ranges
    cv::Mat roi_hsv;
    cv::cvtColor(blurred, roi_hsv, cv::COLOR_BGR2HSV);
    cv::Mat binary;
    cv::Scalar lower(h_min_, s_min_, v_min_);
    cv::Scalar upper(h_max_, s_max_, v_max_);
    cv::inRange(roi_hsv, lower, upper, binary);
    cv::bitwise_and(binary, trapezoid_mask, binary);

    // Intensity-based binary mask with adjustable threshold (Value channel friendly)
    cv::Mat gray;
    cv::cvtColor(blurred, gray, cv::COLOR_BGR2GRAY);
    cv::Mat binary_thresh;
    cv::threshold(gray, binary_thresh, binary_threshold_, 255, cv::THRESH_BINARY_INV);
    cv::bitwise_and(binary_thresh, trapezoid_mask, binary_thresh);
    cv::bitwise_and(binary, binary_thresh, binary);

    if (roi_debug_publisher_ && roi_debug_publisher_->get_subscription_count() > 0) {
        cv::Mat debug_img = masked_roi.clone();
        const cv::Scalar outline_color(0, 255, 0);
        for (size_t i = 0; i < trapezoid_pts.size(); ++i) {
            const cv::Point &p1 = trapezoid_pts[i];
            const cv::Point &p2 = trapezoid_pts[(i + 1) % trapezoid_pts.size()];
            cv::line(debug_img, p1, p2, outline_color, 2);
        }
        auto roi_msg = cv_bridge::CvImage(std_msgs::msg::Header(),
                                          sensor_msgs::image_encodings::BGR8,
                                          debug_img).toImageMsg();
        roi_msg->header.stamp = this->now();
        roi_msg->header.frame_id = "line_roi";
        roi_debug_publisher_->publish(*roi_msg);
    }

    // line_mask publisher handled later to include annotations

    cv::Moments m = cv::moments(binary, true);
    double cx = prev_x_;
    bool valid = false;
    if (m.m00 > 0) {
        cx = m.m10 / m.m00; // x 중심 (열 방향)
        valid = true;
        prev_x_ = cx;
        prev_y_ = (m.m01 / m.m00) + (height - roi_h);
    }

    double signed_error = 0.0;
    double offset = 0.0; // 조향 오프셋 필요시 설정
    if (valid) {
        signed_error = (static_cast<double>(width) / 2.0 - cx) / (static_cast<double>(width) / 2.0);
        signed_error += offset;
        signed_error = std::clamp(signed_error, -1.0, 1.0);
    }

    double angular_cmd = computeAngularCommand(signed_error, valid);
    if (valid) {
        last_angular_cmd_ = angular_cmd;
        lost_frame_count_ = 0;
        if (std::abs(angular_cmd) > corner_detect_threshold_) {
            last_corner_direction_ = angular_cmd > 0.0 ? 1.0 : -1.0;
        }
        rotation_triggered_ = false;
        rotation_stage_ = 0;
    } else if (!is_rotating_) {  // 회전 중이 아닐 때만 카운트 증가
        lost_frame_count_++;
    } else {  // 회전 중에는 리셋
        lost_frame_count_ = 0;
    }

    geometry_msgs::msg::Twist twist;
    
    // 33초 이후에 line_lost가 8프레임 이상이면 90도 회전 트리거 (한 번만)
    double elapsed_time = (this->now() - start_time_).seconds();
    
    if (!is_rotating_ && initial_yaw_set_ && elapsed_time > 33.0 && lost_frame_count_ >= 8 && !rotation_triggered_) {
        rotation_start_time_ = this->now();  // 회전 시작 시간 기록
        if (rotation_stage_ == 0) {
            rotation_stage_ = 1;
            rotation_triggered_ = true;
            is_rotating_ = true;
            target_yaw_ = initial_yaw_;
            while (target_yaw_ > M_PI) target_yaw_ -= 2.0 * M_PI;
            while (target_yaw_ < -M_PI) target_yaw_ += 2.0 * M_PI;
            RCLCPP_INFO(this->get_logger(), "Line lost after 33s. Stage 1 rotate to initial direction. Current: %.2f, Target: %.2f", current_yaw_, target_yaw_);
        } else if (rotation_stage_ == 1) {
            rotation_stage_ = 2;
            rotation_triggered_ = true;
            is_rotating_ = true;
            target_yaw_ = initial_yaw_ + M_PI;
            while (target_yaw_ > M_PI) target_yaw_ -= 2.0 * M_PI;
            while (target_yaw_ < -M_PI) target_yaw_ += 2.0 * M_PI;
            RCLCPP_INFO(this->get_logger(), "Line still lost. Stage 2 rotate clockwise to opposite direction. Current: %.2f, Target: %.2f", current_yaw_, target_yaw_);
        }
    }
    
    
    // 90도 회전 중이면 회전 명령만 실행하고 다른 로직은 무시
    if (is_rotating_) {
        // 현재 yaw와 목표 yaw의 차이 계산
        double yaw_diff = target_yaw_ - current_yaw_;
        // 최단 경로로 회전하도록 정규화
        while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
        while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
        
        // 회전 시간 체크 (10도 오차 허용 또는 5초 이상 경과 시 강제 종료)
        double rotation_elapsed = (this->now() - rotation_start_time_).seconds();
        bool rotation_complete = (std::abs(yaw_diff) < 0.174); // 10도 = 0.174 라디안
        bool rotation_timeout = (rotation_elapsed > 5.0); // 5초 타임아웃
        
        if (rotation_complete || rotation_timeout) {
            is_rotating_ = false;
            rotation_triggered_ = false;
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Rotation stage %d complete. yaw_diff: %.2f rad, elapsed: %.2f s", rotation_stage_, yaw_diff, rotation_elapsed);
        } else {
            // 회전 실행 (선속도 0으로 제자리 회전)
            twist.linear.x = 0.0;
            double angular_speed_cmd = 0.0;
            if (rotation_stage_ == 2) {
                angular_speed_cmd = -1.8; // 시계방향 회전
            } else {
                angular_speed_cmd = std::copysign(1.8, yaw_diff == 0.0 ? 1.0 : yaw_diff);
            }
            twist.angular.z = std::clamp(angular_speed_cmd, -max_angular_speed_, max_angular_speed_);
            RCLCPP_INFO(this->get_logger(), "Rotating (stage %d)... yaw_diff: %.2f rad, elapsed: %.2f s", rotation_stage_, yaw_diff, rotation_elapsed);
        }
    } else if (valid) {
        twist.angular.z = angular_cmd;
        bool is_cornering = std::abs(angular_cmd) > 0.6;
        twist.linear.x = is_cornering ? max_linear_speed_ * corner_speed : max_linear_speed_;
    } else {
        // 회전 절차 이전에만 후진 로직 실행
        if (rotation_stage_ == 0) {
            bool reversing = lost_frame_count_ <= reverse_frame_limit_;
            twist.linear.x = reversing ? reverse_speed_ : lost_forward_speed_;
            double recovery_turn = last_corner_direction_ == 0.0
                                     ? last_angular_cmd_
                                     : last_corner_direction_ * max_angular_speed_ * recovery_turn_gain_;
            twist.angular.z = std::clamp(recovery_turn, -max_angular_speed_, max_angular_speed_);
        } else {
            // 회전 단계를 진행 중이거나 완료했으나 라인을 찾지 못한 경우 대기
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
        }
    }
    
    cmd_vel_publisher_->publish(twist);

    if (line_mask_publisher_ && line_mask_publisher_->get_subscription_count() > 0) {
        cv::Mat mask_bgr;
        cv::cvtColor(binary, mask_bgr, cv::COLOR_GRAY2BGR);
        const std::string text = valid ? "TRACK" : "LOST";
        const cv::Scalar color = valid ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        cv::putText(mask_bgr, text, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX,
                    0.9, color, 2, cv::LINE_AA);
        auto mask_msg = cv_bridge::CvImage(std_msgs::msg::Header(),
                           sensor_msgs::image_encodings::BGR8,
                           mask_bgr).toImageMsg();
        mask_msg->header.stamp = this->now();
        mask_msg->header.frame_id = "camera_line_mask";
        line_mask_publisher_->publish(*mask_msg);
    }

    RCLCPP_DEBUG(this->get_logger(),
                 "cx=%.1f err=%.2f linOut=%.2f ang=%.2f valid=%d",
                 cx, signed_error, twist.linear.x, twist.angular.z, valid);
}

double LineFollower::computeAngularCommand(double signed_error, bool valid)
{
    if (!valid) {
        lat_integral_ = 0.0;
        lat_prev_error_ = 0.0;
        return 0.0;
    }

    lat_integral_ += signed_error;
    lat_integral_ = std::clamp(lat_integral_, -lat_integral_limit_, lat_integral_limit_);
    double derivative = signed_error - lat_prev_error_;
    lat_prev_error_ = signed_error;

    double cmd = lat_kp_ * signed_error + lat_ki_ * lat_integral_ + lat_kd_ * derivative;
    return std::clamp(cmd, -max_angular_speed_, max_angular_speed_);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LineFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
