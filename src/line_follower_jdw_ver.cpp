#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

static const double MAX_LINEAR_SPEED = 0.26;   // TurtleBot3 spec
static const double MAX_ANGULAR_SPEED = 1.82;  // TurtleBot3 spec
static const int ROI_HEIGHT = 180;
static const int ROI_BOTTOM_WIDTH = 960;
static const int ROI_TOP_WIDTH = 220;

class LineFollowerWithCurve : public rclcpp::Node
{
public:
    LineFollowerWithCurve() : Node("line_follower_curve")
    {
        // 초기값
        prev_x_ = 480.0;
        prev_y_ = 270.0;
        
        max_linear_speed_ = MAX_LINEAR_SPEED;
        max_angular_speed_ = MAX_ANGULAR_SPEED;
        
        // PID 파라미터
        lat_kp_ = 7.5;
        lat_ki_ = 0.0;
        lat_kd_ = 0.8;
        lat_integral_ = 0.0;
        lat_prev_error_ = 0.0;
        lat_integral_limit_ = 1.0;
        
        // HSV 파라미터
        h_min_ = 0;
        h_max_ = 179;
        s_min_ = 0;
        s_max_ = 255;
        v_min_ = 0;
        v_max_ = 120;  // 어두운 영역 집중
        
        binary_threshold_ = 128;
        blur_kernel_size_ = 5;
        
        // 커브 감지 상태
        curve_detected_ = false;
        curve_direction_ = 0;  // -1: left, 0: none, 1: right
        is_turning_ = false;
        turn_delay_ = 1.0;  // 1초
        
        // 구독자
        auto qos = rclcpp::SensorDataQoS();
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_upright", qos,
            std::bind(&LineFollowerWithCurve::imageCallback, this, std::placeholders::_1));
        
        // 발행자
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        debug_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/line_follower/debug_image", qos);
        line_mask_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/line_follower/line_mask", qos);
        
        RCLCPP_INFO(this->get_logger(), "Line Follower with Curve Detection initialized");
    }
    
private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
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
            
            processImage(bgr_image, msg->header);
            
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
    
    void processImage(const cv::Mat &image, const std_msgs::msg::Header &header)
    {
        if (image.empty()) return;
        
        int height = image.rows;
        int width = image.cols;
        
        // ROI 추출
        int roi_h = std::min(ROI_HEIGHT, height);
        cv::Rect roi(0, height - roi_h, width, roi_h);
        cv::Mat roi_img = image(roi);
        
        // 사다리꼴 마스크
        const int bottom_width = std::min(width, ROI_BOTTOM_WIDTH);
        const int top_width = std::min(width, ROI_TOP_WIDTH);
        const int bottom_offset = std::max(0, (width - bottom_width) / 2);
        const int top_offset = std::max(0, (width - top_width) / 2);
        
        std::vector<cv::Point> trapezoid_pts = {
            cv::Point(bottom_offset, roi_h - 1),
            cv::Point(width - bottom_offset - 1, roi_h - 1),
            cv::Point(width - top_offset - 1, 0),
            cv::Point(top_offset, 0)
        };
        
        cv::Mat trapezoid_mask = cv::Mat::zeros(roi_h, width, CV_8UC1);
        cv::fillConvexPoly(trapezoid_mask, trapezoid_pts, cv::Scalar(255));
        
        cv::Mat masked_roi;
        cv::bitwise_and(roi_img, roi_img, masked_roi, trapezoid_mask);
        
        // 블러 처리
        cv::Mat blurred;
        cv::GaussianBlur(masked_roi, blurred, cv::Size(blur_kernel_size_, blur_kernel_size_), 0);
        
        // HSV 변환 및 이진화
        cv::Mat roi_hsv;
        cv::cvtColor(blurred, roi_hsv, cv::COLOR_BGR2HSV);
        cv::Mat binary;
        cv::Scalar lower(h_min_, s_min_, v_min_);
        cv::Scalar upper(h_max_, s_max_, v_max_);
        cv::inRange(roi_hsv, lower, upper, binary);
        cv::bitwise_and(binary, trapezoid_mask, binary);
        
        // 그레이스케일 이진화 (추가)
        cv::Mat gray;
        cv::cvtColor(blurred, gray, cv::COLOR_BGR2GRAY);
        cv::Mat binary_thresh;
        cv::threshold(gray, binary_thresh, binary_threshold_, 255, cv::THRESH_BINARY_INV);
        cv::bitwise_and(binary_thresh, trapezoid_mask, binary_thresh);
        cv::bitwise_and(binary, binary_thresh, binary);
        
        // 직선 검출 (Hough Transform)
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(binary, lines, 1, CV_PI/180, 30, 20, 30);
        
        // 좌우 차선 분리
        std::vector<cv::Vec4i> left_lines, right_lines;
        for (const auto& line : lines) {
            int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
            if (x2 - x1 == 0) continue;
            
            double slope = static_cast<double>(y2 - y1) / (x2 - x1);
            if (std::abs(slope) < 0.3) continue;  // 수평선 제거
            
            if (slope < 0) {
                left_lines.push_back(line);
            } else {
                right_lines.push_back(line);
            }
        }
        
        // 커브 감지 로직
        auto now = this->now();
        
        if (!is_turning_) {
            // 왼쪽 차선만 보임 → 좌회전 커브
            if (!left_lines.empty() && right_lines.empty()) {
                if (!curve_detected_) {
                    curve_detected_ = true;
                    curve_direction_ = -1;  // left
                    curve_detect_time_ = now;
                    RCLCPP_INFO(this->get_logger(), "Left curve detected! Will turn in 1 sec...");
                }
            }
            // 오른쪽 차선만 보임 → 우회전 커브
            else if (!right_lines.empty() && left_lines.empty()) {
                if (!curve_detected_) {
                    curve_detected_ = true;
                    curve_direction_ = 1;  // right
                    curve_detect_time_ = now;
                    RCLCPP_INFO(this->get_logger(), "Right curve detected! Will turn in 1 sec...");
                }
            }
        }
        
        // 커브 감지 후 1초 경과 → 회전 시작
        if (curve_detected_ && !is_turning_) {
            double elapsed = (now - curve_detect_time_).seconds();
            if (elapsed >= turn_delay_) {
                is_turning_ = true;
                RCLCPP_INFO(this->get_logger(), "Starting turn!");
            }
        }
        
        // 제어 명령 계산
        geometry_msgs::msg::Twist twist;
        
        // 회전 중
        if (is_turning_) {
            twist.linear.x = max_linear_speed_ * 0.5;  // 속도 감소
            twist.angular.z = curve_direction_ * 0.5;  // 좌(-) 또는 우(+) 회전
            
            // 양쪽 차선이 다시 보이면 회전 종료
            if (!left_lines.empty() && !right_lines.empty()) {
                is_turning_ = false;
                curve_detected_ = false;
                curve_direction_ = 0;
                RCLCPP_INFO(this->get_logger(), "Turn completed. Back to lane following.");
            }
        }
        // 정상 주행 (양쪽 차선 보임)
        else if (!left_lines.empty() && !right_lines.empty()) {
            // 차선 중심 계산 (모멘트 사용)
            cv::Moments m = cv::moments(binary, true);
            bool valid = false;
            double cx = prev_x_;
            
            if (m.m00 > 0) {
                cx = m.m10 / m.m00;
                valid = true;
                prev_x_ = cx;
                prev_y_ = (m.m01 / m.m00) + (height - roi_h);
            }
            
            if (valid) {
                double signed_error = (static_cast<double>(width) / 2.0 - cx) / 
                                     (static_cast<double>(width) / 2.0);
                signed_error = std::clamp(signed_error, -1.0, 1.0);
                
                double angular_cmd = computeAngularCommand(signed_error, true);
                
                twist.linear.x = max_linear_speed_;
                twist.angular.z = angular_cmd;
            } else {
                twist.linear.x = 0.03;
                twist.angular.z = 0.0;
            }
        }
        // 차선 없음 → 정지
        else {
            if (!is_turning_) {
                twist.linear.x = 0.03;
                twist.angular.z = 0.0;
            }
        }
        
        // 속도 명령 발행
        cmd_vel_publisher_->publish(twist);
        
        // 디버그 이미지 생성
        cv::Mat debug_image = masked_roi.clone();
        
        // 사다리꼴 테두리
        for (size_t i = 0; i < trapezoid_pts.size(); ++i) {
            cv::line(debug_image, trapezoid_pts[i], 
                    trapezoid_pts[(i + 1) % trapezoid_pts.size()],
                    cv::Scalar(255, 0, 0), 2);
        }
        
        // 검출된 직선 그리기
        for (const auto& line : left_lines) {
            cv::line(debug_image, cv::Point(line[0], line[1]),
                    cv::Point(line[2], line[3]), cv::Scalar(0, 0, 255), 2);
        }
        for (const auto& line : right_lines) {
            cv::line(debug_image, cv::Point(line[0], line[1]),
                    cv::Point(line[2], line[3]), cv::Scalar(0, 255, 0), 2);
        }
        
        // 상태 텍스트
        std::string status;
        cv::Scalar color;
        if (is_turning_) {
            status = curve_direction_ < 0 ? "TURNING LEFT" : "TURNING RIGHT";
            color = cv::Scalar(0, 165, 255);
        } else if (curve_detected_) {
            double remaining = turn_delay_ - (now - curve_detect_time_).seconds();
            status = "CURVE: " + std::to_string(remaining).substr(0, 3) + "s";
            color = cv::Scalar(0, 255, 255);
        } else {
            status = "FOLLOWING";
            color = cv::Scalar(0, 255, 0);
        }
        
        cv::putText(debug_image, status, cv::Point(10, 20),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
        
        char info[100];
        snprintf(info, sizeof(info), "L:%zu R:%zu", left_lines.size(), right_lines.size());
        cv::putText(debug_image, info, cv::Point(10, 40),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
        
        // 디버그 이미지 발행
        if (debug_publisher_->get_subscription_count() > 0) {
            auto debug_msg = cv_bridge::CvImage(header, 
                sensor_msgs::image_encodings::BGR8, debug_image).toImageMsg();
            debug_publisher_->publish(*debug_msg);
        }
        
        // 마스크 이미지 발행
        if (line_mask_publisher_->get_subscription_count() > 0) {
            cv::Mat mask_bgr;
            cv::cvtColor(binary, mask_bgr, cv::COLOR_GRAY2BGR);
            auto mask_msg = cv_bridge::CvImage(header,
                sensor_msgs::image_encodings::BGR8, mask_bgr).toImageMsg();
            line_mask_publisher_->publish(*mask_msg);
        }
    }
    
    double computeAngularCommand(double signed_error, bool valid)
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
    
    // 멤버 변수
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr line_mask_publisher_;
    
    double prev_x_, prev_y_;
    double max_linear_speed_, max_angular_speed_;
    double lat_kp_, lat_ki_, lat_kd_;
    double lat_integral_, lat_prev_error_, lat_integral_limit_;
    
    int h_min_, h_max_, s_min_, s_max_, v_min_, v_max_;
    int binary_threshold_, blur_kernel_size_;
    
    bool curve_detected_;
    int curve_direction_;  // -1: left, 0: none, 1: right
    bool is_turning_;
    double turn_delay_;
    rclcpp::Time curve_detect_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LineFollowerWithCurve>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}