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
  struct BoundaryEstimate
  {
    bool valid{false};

    double x_near{0.0};
    double x_far{0.0};
  };


  void imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr msg);


  BoundaryEstimate fitBoundary(
    const std::vector<cv::Point> & points,
    int y_near,
    int y_far) const;


  // ----------------------------------------------------------
  // ROS interfaces
  // ----------------------------------------------------------

  rclcpp::Subscription<
    sensor_msgs::msg::Image>::SharedPtr image_sub_;

  rclcpp::Publisher<
    sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;

  rclcpp::Publisher<
    sensor_msgs::msg::Image>::SharedPtr edges_image_pub_;

  rclcpp::Publisher<
    std_msgs::msg::Bool>::SharedPtr valid_pub_;

  rclcpp::Publisher<
    std_msgs::msg::Float64>::SharedPtr error_px_pub_;

  rclcpp::Publisher<
    std_msgs::msg::Float64>::SharedPtr error_normalized_pub_;

  rclcpp::Publisher<
    std_msgs::msg::Float64>::SharedPtr heading_error_pub_;

  rclcpp::Publisher<
    sensor_msgs::msg::Image>::SharedPtr mask_image_pub_;


  // ----------------------------------------------------------
  // Topics
  // ----------------------------------------------------------

  std::string input_topic_;

  std::string debug_image_topic_;
  std::string edges_image_topic_;

  std::string valid_topic_;

  std::string error_px_topic_;
  std::string error_normalized_topic_;
  std::string heading_error_topic_;

  std::string mask_image_topic_;


  // ----------------------------------------------------------
  // Image processing parameters
  // ----------------------------------------------------------

  int white_min_value_;
  int white_max_saturation_;

  int gaussian_kernel_;

  int canny_low_;
  int canny_high_;

  int hough_threshold_;

  double hough_min_line_length_;
  double hough_max_line_gap_;

  double min_abs_slope_;


  // ----------------------------------------------------------
  // ROI parameters
  // ----------------------------------------------------------

  double roi_top_ratio_;

  double roi_top_left_ratio_;
  double roi_top_right_ratio_;

  double roi_bottom_ratio_;

  // ----------------------------------------------------------
  // Lane evaluation parameters
  // ----------------------------------------------------------

  double evaluation_near_ratio_;
  double evaluation_far_ratio_;

  double min_lane_width_ratio_;
  double max_lane_width_ratio_;
};

}  // namespace wheeltec_autorace_application