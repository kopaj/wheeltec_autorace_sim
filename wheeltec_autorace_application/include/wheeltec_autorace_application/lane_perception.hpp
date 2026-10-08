#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

namespace wheeltec_autorace_application
{

class LanePerception final : public rclcpp::Node
{
public:
  LanePerception();

private:
  struct PolynomialLane
  {
    bool valid{false};
    cv::Vec3d coeff{0.0, 0.0, 0.0};  // x(y) = a*y^2 + b*y + c
    std::size_t support{0};
  };

  void imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr msg);

  void speedCallback(
    const std_msgs::msg::Float64::ConstSharedPtr msg);

  PolynomialLane fitPolynomial(
    const std::vector<cv::Point> & points) const;

  static double evaluatePolynomial(
    const cv::Vec3d & coeff,
    double y);

  static cv::Vec3d shiftedPolynomial(
    const cv::Vec3d & coeff,
    double x_shift);

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr birdseye_debug_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr turn_preview_debug_pub_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr degraded_pub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_px_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_normalized_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr turn_preview_pub_;

  // Topics
  std::string input_topic_;
  std::string current_speed_topic_;
  std::string birdseye_debug_topic_;
  std::string turn_preview_debug_topic_;

  std::string valid_topic_;
  std::string degraded_topic_;
  std::string error_px_topic_;
  std::string error_normalized_topic_;
  std::string heading_error_topic_;
  std::string control_error_topic_;
  std::string turn_preview_topic_;

  // White segmentation
  int white_min_value_;
  int white_max_saturation_;
  int gaussian_kernel_;

  // Far-field turn preview. This Hough branch does not steer the vehicle;
  // it only publishes a 0..1 risk score used for anticipatory speed limiting.
  bool turn_preview_enabled_;
  int turn_preview_canny_low_;
  int turn_preview_canny_high_;
  int turn_preview_hough_threshold_;
  double turn_preview_hough_min_line_length_;
  double turn_preview_hough_max_line_gap_;

  double turn_preview_roi_top_ratio_;
  double turn_preview_roi_bottom_ratio_;
  double turn_preview_roi_top_width_ratio_;
  double turn_preview_roi_bottom_width_ratio_;

  double turn_preview_angle_start_deg_;
  double turn_preview_angle_full_deg_;
  double turn_preview_min_evidence_length_px_;
  double turn_preview_attack_alpha_;
  double turn_preview_release_alpha_;

  // Perspective transform source trapezoid (image-space ratios).
  double perspective_top_ratio_;
  double perspective_bottom_ratio_;
  double perspective_top_left_ratio_;
  double perspective_top_right_ratio_;
  double perspective_bottom_left_ratio_;
  double perspective_bottom_right_ratio_;

  // Sliding window / polynomial tracking.
  int sliding_window_count_;
  int sliding_window_margin_px_;
  int sliding_window_minpix_;
  int min_lane_support_pixels_;
  double histogram_start_ratio_;
  double fallback_histogram_start_ratio_;
  double fallback_histogram_end_ratio_;
  int fallback_histogram_min_peak_;

  // Lane geometry in bird's-eye image.
  double evaluation_near_ratio_;
  double evaluation_far_ratio_;
  double control_lookahead_ratio_;
  double control_near_weight_;
  double min_lane_width_ratio_;
  double max_lane_width_ratio_;
  double lane_width_filter_alpha_;

  // Learned lane-width model in bird's-eye view.
  // Instead of assuming a constant pixel width, keep a polynomial
  // w(y) = right_x(y) - left_x(y). This remains usable even when the
  // perspective transform is not perfectly calibrated.
  cv::Vec3d last_lane_width_coeff_{0.0, 0.0, 0.0};
  bool have_lane_width_{false};

  double filtered_turn_preview_{0.0};
  bool have_turn_preview_{false};
  double current_speed_mps_{0.0};
  bool have_current_speed_{false};
};

}  // namespace wheeltec_autorace_application
