#include "wheeltec_autorace_application/lane_perception.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc.hpp>

#include <sensor_msgs/image_encodings.hpp>


namespace wheeltec_autorace_application
{

LanePerception::LanePerception()
: Node("lane_perception")
{
  // ----------------------------------------------------------
  // Topic parameters
  // ----------------------------------------------------------

  input_topic_ =
    declare_parameter<std::string>(
      "input_topic",
      "/camera/image_raw");


  mask_image_topic_ =
    declare_parameter<std::string>(
      "mask_image_topic",
      "/lane_detection/white_mask");

  debug_image_topic_ =
    declare_parameter<std::string>(
      "debug_image_topic",
      "/lane_detection/debug_image");


  edges_image_topic_ =
    declare_parameter<std::string>(
      "edges_image_topic",
      "/lane_detection/edges");


  valid_topic_ =
    declare_parameter<std::string>(
      "valid_topic",
      "/lane_detection/valid");


  error_px_topic_ =
    declare_parameter<std::string>(
      "error_px_topic",
      "/lane_detection/cross_track_error_px");


  error_normalized_topic_ =
    declare_parameter<std::string>(
      "error_normalized_topic",
      "/lane_detection/cross_track_error_normalized");


  heading_error_topic_ =
    declare_parameter<std::string>(
      "heading_error_topic",
      "/lane_detection/heading_error");


  // ----------------------------------------------------------
  // Image processing parameters
  // ----------------------------------------------------------


  white_min_value_ =
    declare_parameter<int>(
      "white_min_value",
      185);


  white_max_saturation_ =
    declare_parameter<int>(
      "white_max_saturation",
      80);


  gaussian_kernel_ =
    declare_parameter<int>(
      "gaussian_kernel",
      5);


  canny_low_ =
    declare_parameter<int>(
      "canny_low",
      50);


  canny_high_ =
    declare_parameter<int>(
      "canny_high",
      150);


  hough_threshold_ =
    declare_parameter<int>(
      "hough_threshold",
      20);


  hough_min_line_length_ =
    declare_parameter<double>(
      "hough_min_line_length",
      20.0);


  hough_max_line_gap_ =
    declare_parameter<double>(
      "hough_max_line_gap",
      35.0);


  min_abs_slope_ =
    declare_parameter<double>(
      "min_abs_slope",
      0.15);


  // ----------------------------------------------------------
  // Region Of Interest
  // ----------------------------------------------------------

  roi_top_ratio_ =
    declare_parameter<double>(
      "roi_top_ratio",
      0.40);


  roi_top_left_ratio_ =
    declare_parameter<double>(
      "roi_top_left_ratio",
      0.10);


  roi_top_right_ratio_ =
    declare_parameter<double>(
      "roi_top_right_ratio",
      0.90);


  roi_bottom_ratio_ =
    declare_parameter<double>(
      "roi_bottom_ratio",
      0.92);

  // ----------------------------------------------------------
  // Lane geometry
  // ----------------------------------------------------------

  evaluation_near_ratio_ =
    declare_parameter<double>(
      "evaluation_near_ratio",
      0.90);


  evaluation_far_ratio_ =
    declare_parameter<double>(
      "evaluation_far_ratio",
      0.55);


  min_lane_width_ratio_ =
    declare_parameter<double>(
      "min_lane_width_ratio",
      0.15);


  max_lane_width_ratio_ =
    declare_parameter<double>(
      "max_lane_width_ratio",
      0.95);


  // Gaussian kernel must be positive and odd.

  gaussian_kernel_ =
    std::max(
      gaussian_kernel_,
      1);

  if ((gaussian_kernel_ % 2) == 0)
  {
    gaussian_kernel_ += 1;
  }


  // ----------------------------------------------------------
  // ROS communication
  // ----------------------------------------------------------

  image_sub_ =
    create_subscription<sensor_msgs::msg::Image>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &LanePerception::imageCallback,
        this,
        std::placeholders::_1));


  mask_image_pub_ =
    create_publisher<sensor_msgs::msg::Image>(
      mask_image_topic_,
      rclcpp::SensorDataQoS());

  debug_image_pub_ =
    create_publisher<sensor_msgs::msg::Image>(
      debug_image_topic_,
      rclcpp::SensorDataQoS());


  edges_image_pub_ =
    create_publisher<sensor_msgs::msg::Image>(
      edges_image_topic_,
      rclcpp::SensorDataQoS());


  valid_pub_ =
    create_publisher<std_msgs::msg::Bool>(
      valid_topic_,
      10);


  error_px_pub_ =
    create_publisher<std_msgs::msg::Float64>(
      error_px_topic_,
      10);


  error_normalized_pub_ =
    create_publisher<std_msgs::msg::Float64>(
      error_normalized_topic_,
      10);


  heading_error_pub_ =
    create_publisher<std_msgs::msg::Float64>(
      heading_error_topic_,
      10);

  RCLCPP_INFO(
    get_logger(),
    "Lane perception started. Input: %s",
    input_topic_.c_str());
}


LanePerception::BoundaryEstimate
LanePerception::fitBoundary(
  const std::vector<cv::Point> & points,
  const int y_near,
  const int y_far) const
{
  BoundaryEstimate result;


  // At least two line segments -> four points.
  if (points.size() < 4)
  {
    return result;
  }


  cv::Vec4f fitted_line;

  cv::fitLine(
    points,
    fitted_line,
    cv::DIST_L2,
    0.0,
    0.01,
    0.01);


  const double vx =
    static_cast<double>(fitted_line[0]);

  const double vy =
    static_cast<double>(fitted_line[1]);

  const double x0 =
    static_cast<double>(fitted_line[2]);

  const double y0 =
    static_cast<double>(fitted_line[3]);


  // We express x as a function of image y.
  if (std::abs(vy) < 1e-6)
  {
    return result;
  }


  const auto x_at_y =
    [vx, vy, x0, y0](const double y)
    {
      return x0 +
        ((y - y0) * vx / vy);
    };


  result.x_near =
    x_at_y(
      static_cast<double>(y_near));


  result.x_far =
    x_at_y(
      static_cast<double>(y_far));


  result.valid = true;

  return result;
}


void LanePerception::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;


  try
  {
    cv_ptr =
      cv_bridge::toCvShare(
        msg,
        sensor_msgs::image_encodings::BGR8);
  }
  catch (const cv_bridge::Exception & exception)
  {
    RCLCPP_ERROR(
      get_logger(),
      "cv_bridge conversion failed: %s",
      exception.what());

    return;
  }


  const cv::Mat & frame =
    cv_ptr->image;


  if (frame.empty())
  {
    return;
  }


  const int width =
    frame.cols;

  const int height =
    frame.rows;


// ==========================================================
// 1. White lane segmentation
// ==========================================================

cv::Mat hsv;

cv::cvtColor(
  frame,
  hsv,
  cv::COLOR_BGR2HSV);


cv::Mat white_mask;

cv::inRange(
  hsv,
  cv::Scalar(
    0,
    0,
    white_min_value_),
  cv::Scalar(
    179,
    white_max_saturation_,
    255),
  white_mask);


// ==========================================================
// 2. Clean up the segmentation mask
// ==========================================================

cv::Mat morphology_kernel =
  cv::getStructuringElement(
    cv::MORPH_RECT,
    cv::Size(3, 3));


cv::morphologyEx(
  white_mask,
  white_mask,
  cv::MORPH_CLOSE,
  morphology_kernel);


// ==========================================================
// 3. Gaussian blur
// ==========================================================

cv::Mat blurred;

cv::GaussianBlur(
  white_mask,
  blurred,
  cv::Size(
    gaussian_kernel_,
    gaussian_kernel_),
  0.0);


// ==========================================================
// 4. Canny only on the segmented white markings
// ==========================================================

cv::Mat edges;

cv::Canny(
  blurred,
  edges,
  canny_low_,
  canny_high_);

  // ==========================================================
  // 4. Region Of Interest
  // ==========================================================

  const int roi_top_y =
    static_cast<int>(
      std::round(
        height * roi_top_ratio_));


  const int roi_top_left_x =
    static_cast<int>(
      std::round(
        width * roi_top_left_ratio_));


  const int roi_top_right_x =
    static_cast<int>(
      std::round(
        width * roi_top_right_ratio_));

  const int roi_bottom_y =
    static_cast<int>(
      std::round(
        height * roi_bottom_ratio_));


  const std::vector<cv::Point> roi_polygon{
    cv::Point(
      0,
      roi_bottom_y),

    cv::Point(
      roi_top_left_x,
      roi_top_y),

    cv::Point(
      roi_top_right_x,
      roi_top_y),

    cv::Point(
      width - 1,
      roi_bottom_y)
  };


  cv::Mat roi_mask =
    cv::Mat::zeros(
      edges.size(),
      CV_8UC1);


  cv::fillConvexPoly(
    roi_mask,
    roi_polygon,
    cv::Scalar(255));


  cv::Mat roi_edges;

  cv::bitwise_and(
    edges,
    roi_mask,
    roi_edges);


  // ==========================================================
  // 5. Probabilistic Hough Transform
  // ==========================================================

  std::vector<cv::Vec4i> hough_lines;


  cv::HoughLinesP(
    roi_edges,
    hough_lines,
    1.0,
    CV_PI / 180.0,
    hough_threshold_,
    hough_min_line_length_,
    hough_max_line_gap_);


  // ==========================================================
  // 6. Separate left / right lane candidates
  // ==========================================================

  std::vector<cv::Point> left_points;
  std::vector<cv::Point> right_points;


  left_points.reserve(
    hough_lines.size() * 2);

  right_points.reserve(
    hough_lines.size() * 2);


  cv::Mat debug =
    frame.clone();


  const std::vector<std::vector<cv::Point>>
    roi_contours{
      roi_polygon
    };


  // ROI is drawn in orange.
  cv::polylines(
    debug,
    roi_contours,
    true,
    cv::Scalar(
      0,
      165,
      255),
    2);


  const double image_center_x =
    static_cast<double>(width) / 2.0;


  int left_segment_count = 0;
  int right_segment_count = 0;


  for (const auto & line : hough_lines)
  {
    const int x1 = line[0];
    const int y1 = line[1];

    const int x2 = line[2];
    const int y2 = line[3];


    const double dx =
      static_cast<double>(x2 - x1);

    const double dy =
      static_cast<double>(y2 - y1);


    double slope;

    if (std::abs(dx) < 1e-6)
    {
        continue;
    }

    slope = dy / dx;


    if (std::abs(slope) < min_abs_slope_)
    {
        continue;
    }

    const double midpoint_x =
    0.5 *
    static_cast<double>(
        x1 + x2);


    if (
    midpoint_x < image_center_x &&
    slope < 0.0)
    {
    left_points.emplace_back(
        x1,
        y1);

    left_points.emplace_back(
        x2,
        y2);

    left_segment_count++;

    cv::line(
        debug,
        cv::Point(x1, y1),
        cv::Point(x2, y2),
        cv::Scalar(0, 0, 255),
        2);
    }
    else if (
    midpoint_x > image_center_x &&
    slope > 0.0)
    {
    right_points.emplace_back(
        x1,
        y1);

    right_points.emplace_back(
        x2,
        y2);

    right_segment_count++;

    cv::line(
        debug,
        cv::Point(x1, y1),
        cv::Point(x2, y2),
        cv::Scalar(255, 0, 0),
        2);
    }
  }


  // ==========================================================
  // 7. Fit one boundary to each side
  // ==========================================================

  int y_near =
    static_cast<int>(
      std::round(
        height *
        evaluation_near_ratio_));


  int y_far =
    static_cast<int>(
      std::round(
        height *
        evaluation_far_ratio_));


  y_near =
    std::max(
      roi_top_y + 1,
      std::min(
        height - 1,
        y_near));


  y_far =
    std::max(
      roi_top_y,
      std::min(
        y_near - 1,
        y_far));


  const BoundaryEstimate left =
    fitBoundary(
      left_points,
      y_near,
      y_far);


  const BoundaryEstimate right =
    fitBoundary(
      right_points,
      y_near,
      y_far);



  std::string invalid_reason = "OK";

  bool detection_valid =
    left.valid &&
    right.valid;


  if (!left.valid)
  {
    invalid_reason = "LEFT MISSING";
  }
  else if (!right.valid)
  {
    invalid_reason = "RIGHT MISSING";
  }

  /*
  RCLCPP_INFO_THROTTLE(
  get_logger(),
  *get_clock(),
  1000,
  "left.valid=%s right.valid=%s | "
  "L near=%.1f far=%.1f | "
  "R near=%.1f far=%.1f",
  left.valid ? "true" : "false",
  right.valid ? "true" : "false",
  left.x_near,
  left.x_far,
  right.x_near,
  right.x_far);
  */

  if (detection_valid)
  {
    if (
      left.x_near >= right.x_near ||
      left.x_far >= right.x_far)
    {
      detection_valid = false;
      invalid_reason = "LANE ORDER";
    }
  }

  if (detection_valid)
  {
    const double lane_width_ratio =
      (
        right.x_near -
        left.x_near
      ) /
      static_cast<double>(width);


    if (
      lane_width_ratio < min_lane_width_ratio_ ||
      lane_width_ratio > max_lane_width_ratio_)
    {
      detection_valid = false;
      invalid_reason = "LANE WIDTH";
    }
  }

  

  if (detection_valid)
    {
    const double margin =
        static_cast<double>(width) * 0.20;

    const bool left_reasonable =
        left.x_near >= -margin &&
        left.x_near <= width + margin &&
        left.x_far >= -margin &&
        left.x_far <= width + margin;

    const bool right_reasonable =
        right.x_near >= -margin &&
        right.x_near <= width + margin &&
        right.x_far >= -margin &&
        right.x_far <= width + margin;

    detection_valid =
        left_reasonable &&
        right_reasonable;
    }


  if (detection_valid)
    {
    const double lane_width_near =
        right.x_near -
        left.x_near;

    const double lane_width_far =
        right.x_far -
        left.x_far;

    const double lane_width_ratio_near =
        lane_width_near /
        static_cast<double>(width);

    const double lane_width_ratio_far =
        lane_width_far /
        static_cast<double>(width);

     
    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Lane width: near=%.1f px (%.3f), "
        "far=%.1f px (%.3f)",
        lane_width_near,
        lane_width_ratio_near,
        lane_width_far,
        lane_width_ratio_far);
    

    detection_valid =
        lane_width_ratio_near >= min_lane_width_ratio_ &&
        lane_width_ratio_near <= max_lane_width_ratio_;
    }


  // ==========================================================
  // 8. Calculate errors
  // ==========================================================

  double cross_track_error_px = 0.0;
  double cross_track_error_normalized = 0.0;
  double heading_error = 0.0;


  if (detection_valid)
  {
    const double lane_center_near =
      0.5 *
      (
        left.x_near +
        right.x_near
      );


    const double lane_center_far =
      0.5 *
      (
        left.x_far +
        right.x_far
      );


    // Positive error:
    // detected lane center is to the RIGHT of camera center.
    //
    // Negative error:
    // detected lane center is to the LEFT.

    cross_track_error_px =
      lane_center_near -
      image_center_x;


    cross_track_error_normalized =
      cross_track_error_px /
      (
        static_cast<double>(width)
        / 2.0
      );


    cross_track_error_normalized =
      std::clamp(
        cross_track_error_normalized,
        -1.0,
        1.0);


    // Heading error in image space.
    //
    // Positive:
    // lane direction points towards the right.
    //
    // Negative:
    // lane direction points towards the left.

    heading_error =
      std::atan2(
        lane_center_far -
        lane_center_near,

        static_cast<double>(
          y_near -
          y_far));


    // --------------------------------------------------------
    // Visualization
    // --------------------------------------------------------

    const cv::Point left_near_point(
      static_cast<int>(
        std::round(
          left.x_near)),
      y_near);


    const cv::Point left_far_point(
      static_cast<int>(
        std::round(
          left.x_far)),
      y_far);


    const cv::Point right_near_point(
      static_cast<int>(
        std::round(
          right.x_near)),
      y_near);


    const cv::Point right_far_point(
      static_cast<int>(
        std::round(
          right.x_far)),
      y_far);


    const cv::Point center_near_point(
      static_cast<int>(
        std::round(
          lane_center_near)),
      y_near);


    const cv::Point center_far_point(
      static_cast<int>(
        std::round(
          lane_center_far)),
      y_far);


    // Fitted left boundary.
    cv::line(
      debug,
      left_near_point,
      left_far_point,
      cv::Scalar(
        0,
        0,
        255),
      4);


    // Fitted right boundary.
    cv::line(
      debug,
      right_near_point,
      right_far_point,
      cv::Scalar(
        255,
        0,
        0),
      4);


    // Estimated lane center.
    cv::line(
      debug,
      center_near_point,
      center_far_point,
      cv::Scalar(
        0,
        255,
        255),
      4);


    // Lookahead / center target.
    cv::circle(
      debug,
      center_far_point,
      8,
      cv::Scalar(
        255,
        0,
        255),
      -1);


    // Image / robot center reference.
    cv::line(
      debug,
      cv::Point(
        static_cast<int>(
          std::round(
            image_center_x)),
        y_near),

      cv::Point(
        static_cast<int>(
          std::round(
            image_center_x)),
        y_far),

      cv::Scalar(
        255,
        255,
        255),
      2);


    // --------------------------------------------------------
    // Publish numeric perception output
    // --------------------------------------------------------

    std_msgs::msg::Float64 error_px_msg;
    error_px_msg.data =
      cross_track_error_px;

    error_px_pub_->publish(
      error_px_msg);


    std_msgs::msg::Float64 error_normalized_msg;
    error_normalized_msg.data =
      cross_track_error_normalized;

    error_normalized_pub_->publish(
      error_normalized_msg);


    std_msgs::msg::Float64 heading_msg;
    heading_msg.data =
      heading_error;

    heading_error_pub_->publish(
      heading_msg);
  }


  // ==========================================================
  // 9. Detection state + text visualization
  // ==========================================================

  std_msgs::msg::Bool valid_msg;
  valid_msg.data =
    detection_valid;

  valid_pub_->publish(
    valid_msg);


  if (detection_valid)
  {
    cv::putText(
      debug,
      "LANE: VALID",
      cv::Point(
        20,
        30),
      cv::FONT_HERSHEY_SIMPLEX,
      0.8,
      cv::Scalar(
        0,
        255,
        0),
      2);


    std::ostringstream error_text;

    error_text
      << std::fixed
      << std::setprecision(3)
      << "CTE norm: "
      << cross_track_error_normalized;


    cv::putText(
      debug,
      error_text.str(),
      cv::Point(
        20,
        60),
      cv::FONT_HERSHEY_SIMPLEX,
      0.65,
      cv::Scalar(
        0,
        255,
        0),
      2);


    std::ostringstream heading_text;

    heading_text
      << std::fixed
      << std::setprecision(3)
      << "Heading: "
      << heading_error
      << " rad";


    cv::putText(
      debug,
      heading_text.str(),
      cv::Point(
        20,
        90),
      cv::FONT_HERSHEY_SIMPLEX,
      0.65,
      cv::Scalar(
        0,
        255,
        0),
      2);
  }
  if (!detection_valid)
  {
    cv::putText(
      debug,
      "LANE: NOT DETECTED - " + invalid_reason,
      cv::Point(20, 30),
      cv::FONT_HERSHEY_SIMPLEX,
      0.65,
      cv::Scalar(0, 0, 255),
      2);
  }


  std::ostringstream segment_text;

  segment_text
    << "Hough L/R: "
    << left_segment_count
    << "/"
    << right_segment_count;


  cv::putText(
    debug,
    segment_text.str(),
    cv::Point(
      20,
      height - 20),
    cv::FONT_HERSHEY_SIMPLEX,
    0.55,
    cv::Scalar(
      255,
      255,
      255),
    1);


  // ==========================================================
  // 10. Publish visualization topics
  // ==========================================================

  auto debug_msg =
    cv_bridge::CvImage(
      msg->header,
      sensor_msgs::image_encodings::BGR8,
      debug)
    .toImageMsg();


  debug_image_pub_->publish(
    *debug_msg);


  auto edges_msg =
    cv_bridge::CvImage(
      msg->header,
      sensor_msgs::image_encodings::MONO8,
      roi_edges)
    .toImageMsg();


  edges_image_pub_->publish(
    *edges_msg);

  auto mask_msg =
    cv_bridge::CvImage(
      msg->header,
      sensor_msgs::image_encodings::MONO8,
      white_mask)
    .toImageMsg();


  mask_image_pub_->publish(
    *mask_msg);
}

}  // namespace wheeltec_autorace_application


int main(
  int argc,
  char ** argv)
{
  rclcpp::init(
    argc,
    argv);


  rclcpp::spin(
    std::make_shared<
      wheeltec_autorace_application::
      LanePerception>());


  rclcpp::shutdown();

  return 0;
}