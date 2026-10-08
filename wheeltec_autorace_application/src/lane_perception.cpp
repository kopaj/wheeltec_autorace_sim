#include "wheeltec_autorace_application/lane_perception.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace wheeltec_autorace_application
{

LanePerception::LanePerception()
: Node("lane_perception")
{
  input_topic_ = declare_parameter<std::string>("input_topic", "/camera/image_raw");
  current_speed_topic_ =
    declare_parameter<std::string>(
      "current_speed_topic",
      "/control/pure_pursuit/current_speed");
  birdseye_debug_topic_ = declare_parameter<std::string>("birdseye_debug_topic", "/lane_detection/birdseye_debug");
  turn_preview_debug_topic_ =
    declare_parameter<std::string>(
      "turn_preview_debug_topic",
      "/lane_detection/turn_preview_debug");

  valid_topic_ = declare_parameter<std::string>("valid_topic", "/lane_detection/valid");
  degraded_topic_ = declare_parameter<std::string>("degraded_topic", "/lane_detection/degraded");
  error_px_topic_ = declare_parameter<std::string>("error_px_topic", "/lane_detection/cross_track_error_px");
  error_normalized_topic_ = declare_parameter<std::string>("error_normalized_topic", "/lane_detection/cross_track_error_normalized");
  heading_error_topic_ = declare_parameter<std::string>("heading_error_topic", "/lane_detection/heading_error");
  control_error_topic_ = declare_parameter<std::string>("control_error_topic", "/lane_detection/control_error_normalized");
  turn_preview_topic_ =
    declare_parameter<std::string>(
      "turn_preview_topic",
      "/lane_detection/turn_preview");

  white_min_value_ = declare_parameter<int>("white_min_value", 185);
  white_max_saturation_ = declare_parameter<int>("white_max_saturation", 80);
  gaussian_kernel_ = declare_parameter<int>("gaussian_kernel", 5);

  turn_preview_enabled_ =
    declare_parameter<bool>("turn_preview_enabled", true);
  turn_preview_canny_low_ =
    declare_parameter<int>("turn_preview_canny_low", 50);
  turn_preview_canny_high_ =
    declare_parameter<int>("turn_preview_canny_high", 150);
  turn_preview_hough_threshold_ =
    declare_parameter<int>("turn_preview_hough_threshold", 15);
  turn_preview_hough_min_line_length_ =
    declare_parameter<double>("turn_preview_hough_min_line_length", 15.0);
  turn_preview_hough_max_line_gap_ =
    declare_parameter<double>("turn_preview_hough_max_line_gap", 20.0);

  turn_preview_roi_top_ratio_ =
    declare_parameter<double>("turn_preview_roi_top_ratio", 0.10);
  turn_preview_roi_bottom_ratio_ =
    declare_parameter<double>("turn_preview_roi_bottom_ratio", 0.62);
  turn_preview_roi_top_width_ratio_ =
    declare_parameter<double>("turn_preview_roi_top_width_ratio", 0.24);
  turn_preview_roi_bottom_width_ratio_ =
    declare_parameter<double>("turn_preview_roi_bottom_width_ratio", 0.92);

  // In the ORIGINAL camera image, normal straight lane boundaries converge
  // toward the horizon and are therefore already diagonal.  A tight corner
  // appearing far ahead produces significantly more HORIZONTAL local
  // segments.  We therefore score lines by angle from the horizontal:
  //   0 deg  -> strong corner cue
  //   90 deg -> vertical / straight-ahead cue
  turn_preview_angle_start_deg_ =
    declare_parameter<double>("turn_preview_angle_start_deg", 38.0);
  turn_preview_angle_full_deg_ =
    declare_parameter<double>("turn_preview_angle_full_deg", 12.0);
  turn_preview_min_evidence_length_px_ =
    declare_parameter<double>("turn_preview_min_evidence_length_px", 80.0);
  turn_preview_attack_alpha_ =
    declare_parameter<double>("turn_preview_attack_alpha", 0.72);
  turn_preview_release_alpha_ =
    declare_parameter<double>("turn_preview_release_alpha", 0.12);

  perspective_top_ratio_ = declare_parameter<double>("perspective_top_ratio", 0.34);
  perspective_bottom_ratio_ = declare_parameter<double>("perspective_bottom_ratio", 0.90);
  perspective_top_left_ratio_ = declare_parameter<double>("perspective_top_left_ratio", 0.08);
  perspective_top_right_ratio_ = declare_parameter<double>("perspective_top_right_ratio", 0.92);
  perspective_bottom_left_ratio_ = declare_parameter<double>("perspective_bottom_left_ratio", 0.00);
  perspective_bottom_right_ratio_ = declare_parameter<double>("perspective_bottom_right_ratio", 1.00);

  sliding_window_count_ = declare_parameter<int>("sliding_window_count", 10);
  sliding_window_margin_px_ = declare_parameter<int>("sliding_window_margin_px", 65);
  sliding_window_minpix_ = declare_parameter<int>("sliding_window_minpix", 30);
  min_lane_support_pixels_ = declare_parameter<int>("min_lane_support_pixels", 90);
  histogram_start_ratio_ = declare_parameter<double>("histogram_start_ratio", 0.55);
  fallback_histogram_start_ratio_ =
    declare_parameter<double>("fallback_histogram_start_ratio", 0.18);
  fallback_histogram_end_ratio_ =
    declare_parameter<double>("fallback_histogram_end_ratio", 0.72);
  fallback_histogram_min_peak_ =
    declare_parameter<int>("fallback_histogram_min_peak", 3);

  evaluation_near_ratio_ = declare_parameter<double>("evaluation_near_ratio", 0.86);
  evaluation_far_ratio_ = declare_parameter<double>("evaluation_far_ratio", 0.58);
  control_lookahead_ratio_ = declare_parameter<double>("control_lookahead_ratio", 0.42);
  control_near_weight_ = declare_parameter<double>("control_near_weight", 0.20);
  min_lane_width_ratio_ = declare_parameter<double>("min_lane_width_ratio", 0.08);
  max_lane_width_ratio_ = declare_parameter<double>("max_lane_width_ratio", 1.30);
  lane_width_filter_alpha_ = declare_parameter<double>("lane_width_filter_alpha", 0.15);

  gaussian_kernel_ = std::max(gaussian_kernel_, 1);
  if ((gaussian_kernel_ % 2) == 0) {
    ++gaussian_kernel_;
  }

  sliding_window_count_ = std::max(sliding_window_count_, 3);
  sliding_window_margin_px_ = std::max(sliding_window_margin_px_, 10);
  sliding_window_minpix_ = std::max(sliding_window_minpix_, 1);
  min_lane_support_pixels_ = std::max(min_lane_support_pixels_, 10);

  histogram_start_ratio_ = std::clamp(histogram_start_ratio_, 0.0, 0.95);
  fallback_histogram_start_ratio_ =
    std::clamp(fallback_histogram_start_ratio_, 0.0, 0.90);
  fallback_histogram_end_ratio_ =
    std::clamp(
      fallback_histogram_end_ratio_,
      fallback_histogram_start_ratio_ + 0.05,
      1.0);
  fallback_histogram_min_peak_ =
    std::max(fallback_histogram_min_peak_, 1);
  evaluation_near_ratio_ = std::clamp(evaluation_near_ratio_, 0.0, 1.0);
  evaluation_far_ratio_ = std::clamp(evaluation_far_ratio_, 0.0, evaluation_near_ratio_);
  control_lookahead_ratio_ = std::clamp(control_lookahead_ratio_, 0.0, evaluation_near_ratio_);
  control_near_weight_ = std::clamp(control_near_weight_, 0.0, 1.0);
  lane_width_filter_alpha_ = std::clamp(lane_width_filter_alpha_, 0.0, 1.0);

  turn_preview_canny_low_ = std::max(turn_preview_canny_low_, 0);
  turn_preview_canny_high_ =
    std::max(turn_preview_canny_high_, turn_preview_canny_low_ + 1);
  turn_preview_hough_threshold_ =
    std::max(turn_preview_hough_threshold_, 1);
  turn_preview_hough_min_line_length_ =
    std::max(turn_preview_hough_min_line_length_, 1.0);
  turn_preview_hough_max_line_gap_ =
    std::max(turn_preview_hough_max_line_gap_, 0.0);

  turn_preview_roi_top_ratio_ =
    std::clamp(turn_preview_roi_top_ratio_, 0.0, 0.95);
  turn_preview_roi_bottom_ratio_ =
    std::clamp(
      turn_preview_roi_bottom_ratio_,
      turn_preview_roi_top_ratio_ + 0.02,
      1.0);
  turn_preview_roi_top_width_ratio_ =
    std::clamp(turn_preview_roi_top_width_ratio_, 0.05, 1.0);
  turn_preview_roi_bottom_width_ratio_ =
    std::clamp(
      turn_preview_roi_bottom_width_ratio_,
      turn_preview_roi_top_width_ratio_,
      1.0);

  turn_preview_angle_start_deg_ =
    std::clamp(turn_preview_angle_start_deg_, 1.0, 89.0);
  turn_preview_angle_full_deg_ =
    std::clamp(
      turn_preview_angle_full_deg_,
      0.0,
      turn_preview_angle_start_deg_ - 1.0);
  turn_preview_min_evidence_length_px_ =
    std::max(turn_preview_min_evidence_length_px_, 1.0);
  turn_preview_attack_alpha_ =
    std::clamp(turn_preview_attack_alpha_, 0.0, 1.0);
  turn_preview_release_alpha_ =
    std::clamp(turn_preview_release_alpha_, 0.0, 1.0);

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    input_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&LanePerception::imageCallback, this, std::placeholders::_1));

  speed_sub_ = create_subscription<std_msgs::msg::Float64>(
    current_speed_topic_,
    10,
    std::bind(&LanePerception::speedCallback, this, std::placeholders::_1));

  birdseye_debug_pub_ = create_publisher<sensor_msgs::msg::Image>(birdseye_debug_topic_, rclcpp::SensorDataQoS());
  turn_preview_debug_pub_ =
    create_publisher<sensor_msgs::msg::Image>(
      turn_preview_debug_topic_,
      rclcpp::SensorDataQoS());

  valid_pub_ = create_publisher<std_msgs::msg::Bool>(valid_topic_, 10);
  degraded_pub_ = create_publisher<std_msgs::msg::Bool>(degraded_topic_, 10);
  error_px_pub_ = create_publisher<std_msgs::msg::Float64>(error_px_topic_, 10);
  error_normalized_pub_ = create_publisher<std_msgs::msg::Float64>(error_normalized_topic_, 10);
  heading_error_pub_ = create_publisher<std_msgs::msg::Float64>(heading_error_topic_, 10);
  control_error_pub_ = create_publisher<std_msgs::msg::Float64>(control_error_topic_, 10);
  turn_preview_pub_ =
    create_publisher<std_msgs::msg::Float64>(turn_preview_topic_, 10);

  RCLCPP_INFO(
    get_logger(),
    "Lane perception started: BEV sliding-window + RAW Hough preview. Retained debug: birdseye_debug, turn_preview_debug. Input: %s",
    input_topic_.c_str());
}

void LanePerception::speedCallback(
  const std_msgs::msg::Float64::ConstSharedPtr msg)
{
  current_speed_mps_ = msg->data;
  have_current_speed_ = true;
}

LanePerception::PolynomialLane LanePerception::fitPolynomial(
  const std::vector<cv::Point> & points) const
{
  PolynomialLane result;
  result.support = points.size();

  if (static_cast<int>(points.size()) < min_lane_support_pixels_) {
    return result;
  }

  cv::Mat a(static_cast<int>(points.size()), 3, CV_64F);
  cv::Mat b(static_cast<int>(points.size()), 1, CV_64F);

  for (std::size_t i = 0; i < points.size(); ++i) {
    const double y = static_cast<double>(points[i].y);
    a.at<double>(static_cast<int>(i), 0) = y * y;
    a.at<double>(static_cast<int>(i), 1) = y;
    a.at<double>(static_cast<int>(i), 2) = 1.0;
    b.at<double>(static_cast<int>(i), 0) = static_cast<double>(points[i].x);
  }

  cv::Mat coeff;
  if (!cv::solve(a, b, coeff, cv::DECOMP_QR)) {
    return result;
  }

  result.coeff = cv::Vec3d(
    coeff.at<double>(0, 0),
    coeff.at<double>(1, 0),
    coeff.at<double>(2, 0));

  result.valid =
    std::isfinite(result.coeff[0]) &&
    std::isfinite(result.coeff[1]) &&
    std::isfinite(result.coeff[2]);

  return result;
}

double LanePerception::evaluatePolynomial(
  const cv::Vec3d & coeff,
  const double y)
{
  return coeff[0] * y * y + coeff[1] * y + coeff[2];
}

cv::Vec3d LanePerception::shiftedPolynomial(
  const cv::Vec3d & coeff,
  const double x_shift)
{
  cv::Vec3d shifted = coeff;
  shifted[2] += x_shift;
  return shifted;
}

void LanePerception::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;

  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & exception) {
    RCLCPP_ERROR(get_logger(), "cv_bridge conversion failed: %s", exception.what());
    return;
  }

  const cv::Mat & frame = cv_ptr->image;
  if (frame.empty()) {
    return;
  }

  const int width = frame.cols;
  const int height = frame.rows;
  const double image_center_x = static_cast<double>(width) / 2.0;

  // ==========================================================
  // 1. White segmentation in the original camera image
  // ==========================================================

  cv::Mat hsv;
  cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

  cv::Mat white_mask;
  cv::inRange(
    hsv,
    cv::Scalar(0, 0, white_min_value_),
    cv::Scalar(179, white_max_saturation_, 255),
    white_mask);

  const cv::Mat morphology_kernel =
    cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

  cv::morphologyEx(
    white_mask,
    white_mask,
    cv::MORPH_CLOSE,
    morphology_kernel);

  // ==========================================================
  // 2. Perspective transform (bird's-eye view)
  // ==========================================================

  const float top_y = static_cast<float>(height * perspective_top_ratio_);
  const float bottom_y = static_cast<float>(height * perspective_bottom_ratio_);

  std::vector<cv::Point2f> src_points{
    cv::Point2f(static_cast<float>(width * perspective_top_left_ratio_), top_y),
    cv::Point2f(static_cast<float>(width * perspective_top_right_ratio_), top_y),
    cv::Point2f(static_cast<float>(width * perspective_bottom_right_ratio_), bottom_y),
    cv::Point2f(static_cast<float>(width * perspective_bottom_left_ratio_), bottom_y)
  };

  std::vector<cv::Point2f> dst_points{
    cv::Point2f(0.0F, 0.0F),
    cv::Point2f(static_cast<float>(width - 1), 0.0F),
    cv::Point2f(static_cast<float>(width - 1), static_cast<float>(height - 1)),
    cv::Point2f(0.0F, static_cast<float>(height - 1))
  };

  const cv::Mat perspective_matrix =
    cv::getPerspectiveTransform(src_points, dst_points);

  cv::Mat birdseye_mask;
  cv::warpPerspective(
    white_mask,
    birdseye_mask,
    perspective_matrix,
    frame.size(),
    cv::INTER_NEAREST,
    cv::BORDER_CONSTANT,
    cv::Scalar(0));

  cv::morphologyEx(
    birdseye_mask,
    birdseye_mask,
    cv::MORPH_CLOSE,
    morphology_kernel);

  // ==========================================================
  // 3. Far-field Hough turn preview in the ORIGINAL camera image
  // ==========================================================
  //
  // The BEV transform is excellent for lane-center estimation near/medium
  // range, but for *earlier* corner anticipation it throws away some of the
  // raw camera's long-range context.
  //
  // Therefore this preview branch works directly in the source image:
  //   white mask -> Canny -> trapezoid ROI ahead of the robot -> Hough
  //
  // It still NEVER changes the steering target. It only publishes a
  // 0..1 turn-preview score for anticipatory speed limiting.

  cv::Mat turn_preview_edges =
    cv::Mat::zeros(frame.size(), CV_8UC1);

  std::vector<cv::Vec4i> turn_preview_lines;
  std::vector<double> turn_preview_line_scores;

  double raw_turn_preview = 0.0;

  const int preview_top =
    std::clamp(
      static_cast<int>(
        std::round(height * turn_preview_roi_top_ratio_)),
      0,
      height - 1);

  const int preview_bottom =
    std::clamp(
      static_cast<int>(
        std::round(height * turn_preview_roi_bottom_ratio_)),
      preview_top + 1,
      height);

  const int preview_center_x =
    static_cast<int>(std::round(image_center_x));

  const int preview_top_half_width =
    std::clamp(
      static_cast<int>(
        std::round(
          0.5 * width * turn_preview_roi_top_width_ratio_)),
      1,
      width / 2);

  const int preview_bottom_half_width =
    std::clamp(
      static_cast<int>(
        std::round(
          0.5 * width * turn_preview_roi_bottom_width_ratio_)),
      preview_top_half_width,
      width / 2);

  std::vector<cv::Point> turn_preview_roi{
    cv::Point(
      std::max(0, preview_center_x - preview_bottom_half_width),
      preview_bottom),
    cv::Point(
      std::max(0, preview_center_x - preview_top_half_width),
      preview_top),
    cv::Point(
      std::min(width - 1, preview_center_x + preview_top_half_width),
      preview_top),
    cv::Point(
      std::min(width - 1, preview_center_x + preview_bottom_half_width),
      preview_bottom)
  };

  if (turn_preview_enabled_) {
    cv::Mat preview_blurred;
    cv::GaussianBlur(
      white_mask,
      preview_blurred,
      cv::Size(gaussian_kernel_, gaussian_kernel_),
      0.0);

    cv::Mat preview_all_edges;
    cv::Canny(
      preview_blurred,
      preview_all_edges,
      turn_preview_canny_low_,
      turn_preview_canny_high_);

    cv::Mat preview_roi_mask =
      cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::fillConvexPoly(
      preview_roi_mask,
      turn_preview_roi,
      cv::Scalar(255));

    cv::bitwise_and(
      preview_all_edges,
      preview_roi_mask,
      turn_preview_edges);

    cv::HoughLinesP(
      turn_preview_edges,
      turn_preview_lines,
      1.0,
      CV_PI / 180.0,
      turn_preview_hough_threshold_,
      turn_preview_hough_min_line_length_,
      turn_preview_hough_max_line_gap_);

    double weighted_score_sum = 0.0;
    double evidence_length_sum = 0.0;

    turn_preview_line_scores.reserve(turn_preview_lines.size());

    for (const auto & line : turn_preview_lines) {
      const double dx =
        static_cast<double>(line[2] - line[0]);
      const double dy =
        static_cast<double>(line[3] - line[1]);
      const double length =
        std::hypot(dx, dy);

      if (length < 1e-6) {
        turn_preview_line_scores.push_back(0.0);
        continue;
      }

      // Fold Hough angle to [0, 90]:
      //   0  deg = horizontal
      //   90 deg = vertical
      double angle_from_horizontal_deg =
        std::abs(
          std::atan2(dy, dx) *
          180.0 / CV_PI);

      if (angle_from_horizontal_deg > 90.0) {
        angle_from_horizontal_deg =
          180.0 - angle_from_horizontal_deg;
      }

      // Far, nearly horizontal lane-marking fragments are the early cue for
      // a strong bend in the raw perspective image.
      //
      // start_deg: first angle that begins contributing
      // full_deg:  angle at/below which contribution is maximal
      const double line_score =
        std::clamp(
          (turn_preview_angle_start_deg_ - angle_from_horizontal_deg) /
          std::max(
            1e-6,
            turn_preview_angle_start_deg_ -
            turn_preview_angle_full_deg_),
          0.0,
          1.0);

      // Farther lines (higher in the image) should matter more, because they
      // provide earlier anticipation. Weight them slightly higher.
      const double midpoint_y =
        0.5 * static_cast<double>(line[1] + line[3]);

      const double roi_height =
        std::max(
          1.0,
          static_cast<double>(preview_bottom - preview_top));

      const double normalized_depth =
        std::clamp(
          (midpoint_y - static_cast<double>(preview_top)) /
          roi_height,
          0.0,
          1.0);

      const double far_weight =
        std::clamp(
          1.0 - 0.75 * normalized_depth,
          0.25,
          1.0);

      turn_preview_line_scores.push_back(line_score);

      if (line_score > 0.0) {
        weighted_score_sum +=
          length * far_weight * line_score;
        evidence_length_sum +=
          length * far_weight;
      }
    }

    if (evidence_length_sum > 1e-6) {
      const double average_corner_score =
        weighted_score_sum / evidence_length_sum;

      const double evidence_factor =
        std::clamp(
          evidence_length_sum /
          turn_preview_min_evidence_length_px_,
          0.0,
          1.0);

      raw_turn_preview =
        std::clamp(
          average_corner_score * evidence_factor,
          0.0,
          1.0);
    }
  }

  if (!have_turn_preview_) {
    filtered_turn_preview_ = raw_turn_preview;
    have_turn_preview_ = true;
  } else {
    const double alpha =
      raw_turn_preview > filtered_turn_preview_
      ? turn_preview_attack_alpha_
      : turn_preview_release_alpha_;

    filtered_turn_preview_ =
      alpha * raw_turn_preview +
      (1.0 - alpha) * filtered_turn_preview_;
  }

  filtered_turn_preview_ =
    std::clamp(filtered_turn_preview_, 0.0, 1.0);

  std_msgs::msg::Float64 turn_preview_msg;
  turn_preview_msg.data =
    turn_preview_enabled_ ? filtered_turn_preview_ : 0.0;
  turn_preview_pub_->publish(turn_preview_msg);

  // Dedicated turn-preview debug image.
  // IMPORTANT: this image is a clone of /camera/image_raw, NOT bird's-eye.
  cv::Mat turn_preview_debug = frame.clone();

  cv::polylines(
    turn_preview_debug,
    std::vector<std::vector<cv::Point>>{turn_preview_roi},
    true,
    cv::Scalar(255, 0, 255),
    3);

  for (std::size_t i = 0; i < turn_preview_lines.size(); ++i) {
    const auto & line = turn_preview_lines[i];
    const double line_score =
      i < turn_preview_line_scores.size()
      ? turn_preview_line_scores[i]
      : 0.0;

    const cv::Scalar color =
      line_score > 0.05
      ? cv::Scalar(0, 0, 255)
      : cv::Scalar(160, 160, 160);

    const int thickness =
      line_score > 0.50 ? 4 : 2;

    cv::line(
      turn_preview_debug,
      cv::Point(line[0], line[1]),
      cv::Point(line[2], line[3]),
      color,
      thickness,
      cv::LINE_AA);
  }

  {
    std::ostringstream text;
    text
      << std::fixed << std::setprecision(2)
      << "RAW CAMERA TURN PREVIEW  raw/filt="
      << raw_turn_preview
      << "/"
      << filtered_turn_preview_
      << "  lines="
      << turn_preview_lines.size();

    cv::putText(
      turn_preview_debug,
      text.str(),
      cv::Point(18, 32),
      cv::FONT_HERSHEY_SIMPLEX,
      0.62,
      filtered_turn_preview_ >= 0.20
        ? cv::Scalar(0, 0, 255)
        : cv::Scalar(255, 0, 255),
      2);
  }

  // ==========================================================
  // 4. Histogram seeds in the lower part of bird's-eye mask
  // ==========================================================

  const int histogram_start_y =
    std::clamp(
      static_cast<int>(std::round(height * histogram_start_ratio_)),
      0,
      height - 1);

  std::vector<int> histogram(width, 0);

  for (int y = histogram_start_y; y < height; ++y) {
    const uint8_t * row = birdseye_mask.ptr<uint8_t>(y);
    for (int x = 0; x < width; ++x) {
      if (row[x] > 0) {
        ++histogram[x];
      }
    }
  }

  const int midpoint = width / 2;

  const auto left_peak_it =
    std::max_element(histogram.begin(), histogram.begin() + midpoint);
  const auto right_peak_it =
    std::max_element(histogram.begin() + midpoint, histogram.end());

  int left_current = static_cast<int>(std::distance(histogram.begin(), left_peak_it));
  int right_current = static_cast<int>(std::distance(histogram.begin(), right_peak_it));

  bool left_seed_valid =
    (left_peak_it != histogram.begin() + midpoint) &&
    (*left_peak_it > 0);

  bool right_seed_valid =
    (right_peak_it != histogram.end()) &&
    (*right_peak_it > 0);

  bool left_seed_from_fallback = false;
  bool right_seed_from_fallback = false;

  // In a tight bend one of the lane boundaries may not reach the lower
  // histogram band at all, even though it is clearly visible farther ahead.
  // The old implementation then forced a one-sided estimate.  If a bottom
  // seed is missing, search a wider mid/upper vertical band before declaring
  // that boundary absent.  The sliding windows still run bottom-to-top; they
  // simply keep this fallback x-position until they reach the visible stripe.
  const int fallback_start_y =
    std::clamp(
      static_cast<int>(
        std::round(
          height *
          fallback_histogram_start_ratio_)),
      0,
      height - 1);

  const int fallback_end_y =
    std::clamp(
      static_cast<int>(
        std::round(
          height *
          fallback_histogram_end_ratio_)),
      fallback_start_y + 1,
      height);

  if (!left_seed_valid || !right_seed_valid) {
    std::vector<int> fallback_histogram(width, 0);

    for (int y = fallback_start_y; y < fallback_end_y; ++y) {
      const uint8_t * row =
        birdseye_mask.ptr<uint8_t>(y);

      for (int x = 0; x < width; ++x) {
        if (row[x] > 0) {
          ++fallback_histogram[x];
        }
      }
    }

    if (!left_seed_valid) {
      const auto fallback_left_it =
        std::max_element(
          fallback_histogram.begin(),
          fallback_histogram.begin() + midpoint);

      if (
        fallback_left_it !=
        fallback_histogram.begin() + midpoint &&
        *fallback_left_it >=
        fallback_histogram_min_peak_)
      {
        left_current =
          static_cast<int>(
            std::distance(
              fallback_histogram.begin(),
              fallback_left_it));

        left_seed_valid = true;
        left_seed_from_fallback = true;
      }
    }

    if (!right_seed_valid) {
      const auto fallback_right_it =
        std::max_element(
          fallback_histogram.begin() + midpoint,
          fallback_histogram.end());

      if (
        fallback_right_it !=
        fallback_histogram.end() &&
        *fallback_right_it >=
        fallback_histogram_min_peak_)
      {
        right_current =
          static_cast<int>(
            std::distance(
              fallback_histogram.begin(),
              fallback_right_it));

        right_seed_valid = true;
        right_seed_from_fallback = true;
      }
    }
  }

  // ==========================================================
  // 5. Sliding-window tracing of curved lane boundaries
  // ==========================================================

  std::vector<cv::Point> nonzero_points;
  cv::findNonZero(birdseye_mask, nonzero_points);

  std::vector<cv::Point> left_lane_points;
  std::vector<cv::Point> right_lane_points;
  left_lane_points.reserve(nonzero_points.size() / 2);
  right_lane_points.reserve(nonzero_points.size() / 2);

  cv::Mat birdseye_debug;
  cv::cvtColor(birdseye_mask, birdseye_debug, cv::COLOR_GRAY2BGR);

  const int window_height = std::max(1, height / sliding_window_count_);

  int left_windows_hit = 0;
  int right_windows_hit = 0;

  for (int window = 0; window < sliding_window_count_; ++window) {
    const int win_y_high = height - window * window_height;
    const int win_y_low = std::max(0, win_y_high - window_height);

    auto collect_window =
      [&](const int center_x,
          std::vector<cv::Point> & output,
          const cv::Scalar & color,
          int & new_center,
          int & windows_hit)
      {
        const int x_low = std::max(0, center_x - sliding_window_margin_px_);
        const int x_high = std::min(width - 1, center_x + sliding_window_margin_px_);

        cv::rectangle(
          birdseye_debug,
          cv::Point(x_low, win_y_low),
          cv::Point(x_high, std::min(height - 1, win_y_high)),
          color,
          1);

        long x_sum = 0;
        int count = 0;

        for (const auto & p : nonzero_points) {
          if (
            p.y >= win_y_low && p.y < win_y_high &&
            p.x >= x_low && p.x <= x_high)
          {
            output.push_back(p);
            x_sum += p.x;
            ++count;
          }
        }

        if (count >= sliding_window_minpix_) {
          new_center = static_cast<int>(x_sum / count);
          ++windows_hit;
        }
      };

    if (left_seed_valid) {
      collect_window(
        left_current,
        left_lane_points,
        cv::Scalar(0, 0, 255),
        left_current,
        left_windows_hit);
    }

    if (right_seed_valid) {
      collect_window(
        right_current,
        right_lane_points,
        cv::Scalar(255, 0, 0),
        right_current,
        right_windows_hit);
    }
  }

  PolynomialLane left_lane = fitPolynomial(left_lane_points);
  PolynomialLane right_lane = fitPolynomial(right_lane_points);

  const bool raw_left_valid = left_lane.valid;
  const bool raw_right_valid = right_lane.valid;

  const int y_near =
    std::clamp(static_cast<int>(std::round(height * evaluation_near_ratio_)), 0, height - 1);
  const int y_far =
    std::clamp(static_cast<int>(std::round(height * evaluation_far_ratio_)), 0, y_near - 1);
  const int y_lookahead =
    std::clamp(static_cast<int>(std::round(height * control_lookahead_ratio_)), 0, y_near - 1);


  double raw_width_near_ratio = std::numeric_limits<double>::quiet_NaN();
  double raw_width_lookahead_ratio = std::numeric_limits<double>::quiet_NaN();

  if (raw_left_valid && raw_right_valid) {
    const double raw_width_near =
      evaluatePolynomial(right_lane.coeff, y_near) -
      evaluatePolynomial(left_lane.coeff, y_near);

    const double raw_width_lookahead =
      evaluatePolynomial(right_lane.coeff, y_lookahead) -
      evaluatePolynomial(left_lane.coeff, y_lookahead);

    raw_width_near_ratio =
      raw_width_near / static_cast<double>(width);

    raw_width_lookahead_ratio =
      raw_width_lookahead / static_cast<double>(width);
  }

  bool detection_valid = false;
  bool degraded_detection = false;
  bool left_estimated = false;
  bool right_estimated = false;
  std::string state_text = "LOST";
  std::string invalid_reason = "NO LANES";

  auto lane_pair_reasonable =
    [&](const PolynomialLane & left_candidate,
        const PolynomialLane & right_candidate,
        std::string & reason)
    {
      if (!left_candidate.valid || !right_candidate.valid) {
        reason = "BOUNDARY INVALID";
        return false;
      }

      const double min_width =
        min_lane_width_ratio_ * static_cast<double>(width);

      const double max_width =
        max_lane_width_ratio_ * static_cast<double>(width);

      // Do not validate lane width at only one or two extrapolated points.
      // Sample the full control region. The current BEV is intentionally
      // tolerant of imperfect perspective calibration, so lane width may
      // still vary with y.
      constexpr int sample_count = 7;

      for (int i = 0; i < sample_count; ++i) {
        const double t =
          static_cast<double>(i) /
          static_cast<double>(sample_count - 1);

        const double y =
          static_cast<double>(y_lookahead) +
          t * static_cast<double>(y_near - y_lookahead);

        const double left_x =
          evaluatePolynomial(left_candidate.coeff, y);

        const double right_x =
          evaluatePolynomial(right_candidate.coeff, y);

        if (!std::isfinite(left_x) || !std::isfinite(right_x)) {
          reason = "NONFINITE";
          return false;
        }

        const double lane_width =
          right_x - left_x;

        if (lane_width <= 0.0) {
          reason = "LANE ORDER";
          return false;
        }

        if (lane_width < min_width || lane_width > max_width) {
          reason = "LANE WIDTH";
          return false;
        }
      }

      reason = "OK";
      return true;
    };

  auto learn_lane_width_model =
    [&](const PolynomialLane & left_candidate,
        const PolynomialLane & right_candidate)
    {
      cv::Vec3d measured_width_coeff;

      for (int i = 0; i < 3; ++i) {
        measured_width_coeff[i] =
          right_candidate.coeff[i] -
          left_candidate.coeff[i];
      }

      if (!have_lane_width_) {
        last_lane_width_coeff_ =
          measured_width_coeff;

        have_lane_width_ = true;
      } else {
        for (int i = 0; i < 3; ++i) {
          last_lane_width_coeff_[i] =
            lane_width_filter_alpha_ *
            measured_width_coeff[i] +
            (1.0 - lane_width_filter_alpha_) *
            last_lane_width_coeff_[i];
        }
      }
    };

  auto reconstruct_right_from_left =
    [&](const PolynomialLane & left_candidate)
    {
      PolynomialLane reconstructed =
        left_candidate;

      for (int i = 0; i < 3; ++i) {
        reconstructed.coeff[i] =
          left_candidate.coeff[i] +
          last_lane_width_coeff_[i];
      }

      reconstructed.valid = true;
      return reconstructed;
    };

  auto reconstruct_left_from_right =
    [&](const PolynomialLane & right_candidate)
    {
      PolynomialLane reconstructed =
        right_candidate;

      for (int i = 0; i < 3; ++i) {
        reconstructed.coeff[i] =
          right_candidate.coeff[i] -
          last_lane_width_coeff_[i];
      }

      reconstructed.valid = true;
      return reconstructed;
    };

  if (raw_left_valid && raw_right_valid) {
    if (lane_pair_reasonable(left_lane, right_lane, invalid_reason)) {
      detection_valid = true;
      state_text = "FULL";

      // Learn the complete lane-width shape w(y), not a single scalar.
      learn_lane_width_model(
        left_lane,
        right_lane);
    } else if (have_lane_width_) {
      // If both fits exist but the pair is inconsistent, prefer the boundary
      // that is observed through more independent sliding windows.  A thick
      // inner stripe can contain more raw pixels while actually being visible
      // over a shorter part of the image, so pixel count alone is a poor
      // confidence measure in tight bends.
      bool keep_left = false;

      if (left_windows_hit != right_windows_hit) {
        keep_left = left_windows_hit > right_windows_hit;
      } else {
        keep_left = left_lane.support >= right_lane.support;
      }

      if (keep_left) {
        right_lane =
          reconstruct_right_from_left(
            left_lane);

        right_estimated = true;
      } else {
        left_lane =
          reconstruct_left_from_right(
            right_lane);

        left_estimated = true;
      }

      if (lane_pair_reasonable(left_lane, right_lane, invalid_reason)) {
        detection_valid = true;
        degraded_detection = true;
        state_text =
          left_estimated ?
          "DEGRADED - LEFT EST" :
          "DEGRADED - RIGHT EST";
      }
    }
  } else if (raw_left_valid && have_lane_width_) {
    right_lane =
      reconstruct_right_from_left(
        left_lane);

    right_estimated = true;

    if (lane_pair_reasonable(left_lane, right_lane, invalid_reason)) {
      detection_valid = true;
      degraded_detection = true;
      state_text = "DEGRADED - RIGHT EST";
    }
  } else if (raw_right_valid && have_lane_width_) {
    left_lane =
      reconstruct_left_from_right(
        right_lane);

    left_estimated = true;

    if (lane_pair_reasonable(left_lane, right_lane, invalid_reason)) {
      detection_valid = true;
      degraded_detection = true;
      state_text = "DEGRADED - LEFT EST";
    }
  }

  double cross_track_error_px = 0.0;
  double cross_track_error_normalized = 0.0;
  double heading_error = 0.0;
  double control_error_normalized = 0.0;

  if (detection_valid) {
    const double left_near = evaluatePolynomial(left_lane.coeff, y_near);
    const double right_near = evaluatePolynomial(right_lane.coeff, y_near);
    const double left_far = evaluatePolynomial(left_lane.coeff, y_far);
    const double right_far = evaluatePolynomial(right_lane.coeff, y_far);
    const double left_lookahead = evaluatePolynomial(left_lane.coeff, y_lookahead);
    const double right_lookahead = evaluatePolynomial(right_lane.coeff, y_lookahead);

    const double center_near = 0.5 * (left_near + right_near);
    const double center_far = 0.5 * (left_far + right_far);
    const double center_lookahead = 0.5 * (left_lookahead + right_lookahead);

    cross_track_error_px = center_near - image_center_x;
    cross_track_error_normalized =
      std::clamp(cross_track_error_px / (static_cast<double>(width) / 2.0), -1.0, 1.0);

    const double control_center =
      control_near_weight_ * center_near +
      (1.0 - control_near_weight_) * center_lookahead;

    control_error_normalized =
      std::clamp(
        (control_center - image_center_x) / (static_cast<double>(width) / 2.0),
        -1.0,
        1.0);

    heading_error =
      std::atan2(
        center_far - center_near,
        static_cast<double>(y_near - y_far));

    // Draw fitted curves and centerline in bird's-eye debug image.
    std::vector<cv::Point> left_curve;
    std::vector<cv::Point> right_curve;
    std::vector<cv::Point> center_curve;

    for (int y = 0; y < height; y += 6) {
      const double lx = evaluatePolynomial(left_lane.coeff, y);
      const double rx = evaluatePolynomial(right_lane.coeff, y);
      const double cx = 0.5 * (lx + rx);

      if (std::isfinite(lx) && lx > -width && lx < 2.0 * width) {
        left_curve.emplace_back(static_cast<int>(std::round(lx)), y);
      }
      if (std::isfinite(rx) && rx > -width && rx < 2.0 * width) {
        right_curve.emplace_back(static_cast<int>(std::round(rx)), y);
      }
      if (std::isfinite(cx) && cx > -width && cx < 2.0 * width) {
        center_curve.emplace_back(static_cast<int>(std::round(cx)), y);
      }
    }

    if (left_curve.size() > 1) {
      cv::polylines(
        birdseye_debug,
        std::vector<std::vector<cv::Point>>{left_curve},
        false,
        left_estimated ? cv::Scalar(255, 0, 255) : cv::Scalar(0, 0, 255),
        3);
    }
    if (right_curve.size() > 1) {
      cv::polylines(
        birdseye_debug,
        std::vector<std::vector<cv::Point>>{right_curve},
        false,
        right_estimated ? cv::Scalar(255, 0, 255) : cv::Scalar(255, 0, 0),
        3);
    }
    if (center_curve.size() > 1) {
      cv::polylines(
        birdseye_debug,
        std::vector<std::vector<cv::Point>>{center_curve},
        false,
        cv::Scalar(0, 255, 255),
        3);
    }

    cv::circle(
      birdseye_debug,
      cv::Point(static_cast<int>(std::round(center_lookahead)), y_lookahead),
      8,
      cv::Scalar(255, 0, 255),
      -1);

    cv::line(
      birdseye_debug,
      cv::Point(static_cast<int>(image_center_x), y_near),
      cv::Point(static_cast<int>(image_center_x), y_lookahead),
      cv::Scalar(255, 255, 255),
      2);

    std_msgs::msg::Float64 error_px_msg;
    error_px_msg.data = cross_track_error_px;
    error_px_pub_->publish(error_px_msg);

    std_msgs::msg::Float64 error_normalized_msg;
    error_normalized_msg.data = cross_track_error_normalized;
    error_normalized_pub_->publish(error_normalized_msg);

    std_msgs::msg::Float64 heading_msg;
    heading_msg.data = heading_error;
    heading_error_pub_->publish(heading_msg);

    std_msgs::msg::Float64 control_error_msg;
    control_error_msg.data = control_error_normalized;
    control_error_pub_->publish(control_error_msg);
  }

  std_msgs::msg::Bool valid_msg;
  valid_msg.data = detection_valid;
  valid_pub_->publish(valid_msg);

  std_msgs::msg::Bool degraded_msg;
  degraded_msg.data = degraded_detection;
  degraded_pub_->publish(degraded_msg);

  const cv::Scalar status_color =
    detection_valid ?
    (degraded_detection ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 255, 0)) :
    cv::Scalar(0, 0, 255);

  std::ostringstream bird_text;
  bird_text
    << state_text
    << "  L/R px: " << left_lane_points.size()
    << "/" << right_lane_points.size()
    << "  win=" << left_windows_hit
    << "/" << right_windows_hit;

  if (left_seed_from_fallback || right_seed_from_fallback) {
    bird_text
      << "  seed="
      << (left_seed_from_fallback ? "F" : "B")
      << "/"
      << (right_seed_from_fallback ? "F" : "B");
  }

  cv::putText(
    birdseye_debug,
    bird_text.str(),
    cv::Point(15, 28),
    cv::FONT_HERSHEY_SIMPLEX,
    0.60,
    status_color,
    2);


  if (std::isfinite(raw_width_near_ratio) &&
      std::isfinite(raw_width_lookahead_ratio))
  {
    std::ostringstream raw_width_text;
    raw_width_text
      << std::fixed << std::setprecision(2)
      << "RAW W near/look="
      << raw_width_near_ratio
      << "/"
      << raw_width_lookahead_ratio;

    cv::putText(
      birdseye_debug,
      raw_width_text.str(),
      cv::Point(15, 55),
      cv::FONT_HERSHEY_SIMPLEX,
      0.50,
      cv::Scalar(0, 255, 255),
      2);
  }

  if (detection_valid) {
    std::ostringstream control_text;
    control_text
      << std::fixed << std::setprecision(3)
      << "CTE=" << cross_track_error_normalized
      << "  CTRL=" << control_error_normalized
      << "  HDG=" << heading_error;

    cv::putText(
      birdseye_debug,
      control_text.str(),
      cv::Point(15, 82),
      cv::FONT_HERSHEY_SIMPLEX,
      0.52,
      status_color,
      2);

    const double debug_width_near =
      evaluatePolynomial(right_lane.coeff, y_near) -
      evaluatePolynomial(left_lane.coeff, y_near);

    const double debug_width_lookahead =
      evaluatePolynomial(right_lane.coeff, y_lookahead) -
      evaluatePolynomial(left_lane.coeff, y_lookahead);

    std::ostringstream width_text;
    width_text
      << std::fixed << std::setprecision(2)
      << "W near/look="
      << debug_width_near / static_cast<double>(width)
      << "/"
      << debug_width_lookahead / static_cast<double>(width);

    cv::putText(
      birdseye_debug,
      width_text.str(),
      cv::Point(15, 109),
      cv::FONT_HERSHEY_SIMPLEX,
      0.50,
      status_color,
      2);
  }

  {
    std::ostringstream speed_text;
    speed_text << std::fixed << std::setprecision(2);

    if (have_current_speed_) {
      speed_text
        << "CMD_V="
        << current_speed_mps_
        << " m/s  ("
        << current_speed_mps_ * 3.6
        << " km/h)";
    } else {
      speed_text << "CMD_V=--.-- m/s";
    }

    cv::putText(
      birdseye_debug,
      speed_text.str(),
      cv::Point(15, 136),
      cv::FONT_HERSHEY_SIMPLEX,
      0.52,
      cv::Scalar(255, 220, 0),
      2);
  }

  auto turn_preview_debug_msg =
    cv_bridge::CvImage(
      msg->header,
      sensor_msgs::image_encodings::BGR8,
      turn_preview_debug).toImageMsg();
  turn_preview_debug_pub_->publish(*turn_preview_debug_msg);

  auto bird_debug_msg =
    cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, birdseye_debug).toImageMsg();
  birdseye_debug_pub_->publish(*bird_debug_msg);
}

}  // namespace wheeltec_autorace_application

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<wheeltec_autorace_application::LanePerception>());
  rclcpp::shutdown();
  return 0;
}
