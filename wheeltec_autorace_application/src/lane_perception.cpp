#include "wheeltec_autorace_application/lane_perception.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <numeric>
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
  debug_image_topic_ = declare_parameter<std::string>("debug_image_topic", "/lane_detection/debug_image");
  edges_image_topic_ = declare_parameter<std::string>("edges_image_topic", "/lane_detection/edges");
  mask_image_topic_ = declare_parameter<std::string>("mask_image_topic", "/lane_detection/white_mask");
  birdseye_mask_topic_ = declare_parameter<std::string>("birdseye_mask_topic", "/lane_detection/birdseye_mask");
  birdseye_debug_topic_ = declare_parameter<std::string>("birdseye_debug_topic", "/lane_detection/birdseye_debug");

  valid_topic_ = declare_parameter<std::string>("valid_topic", "/lane_detection/valid");
  degraded_topic_ = declare_parameter<std::string>("degraded_topic", "/lane_detection/degraded");
  error_px_topic_ = declare_parameter<std::string>("error_px_topic", "/lane_detection/cross_track_error_px");
  error_normalized_topic_ = declare_parameter<std::string>("error_normalized_topic", "/lane_detection/cross_track_error_normalized");
  heading_error_topic_ = declare_parameter<std::string>("heading_error_topic", "/lane_detection/heading_error");
  control_error_topic_ = declare_parameter<std::string>("control_error_topic", "/lane_detection/control_error_normalized");

  white_min_value_ = declare_parameter<int>("white_min_value", 185);
  white_max_saturation_ = declare_parameter<int>("white_max_saturation", 80);
  gaussian_kernel_ = declare_parameter<int>("gaussian_kernel", 5);

  canny_low_ = declare_parameter<int>("canny_low", 50);
  canny_high_ = declare_parameter<int>("canny_high", 150);
  hough_threshold_ = declare_parameter<int>("hough_threshold", 20);
  hough_min_line_length_ = declare_parameter<double>("hough_min_line_length", 20.0);
  hough_max_line_gap_ = declare_parameter<double>("hough_max_line_gap", 35.0);

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

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    input_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&LanePerception::imageCallback, this, std::placeholders::_1));

  debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, rclcpp::SensorDataQoS());
  edges_image_pub_ = create_publisher<sensor_msgs::msg::Image>(edges_image_topic_, rclcpp::SensorDataQoS());
  mask_image_pub_ = create_publisher<sensor_msgs::msg::Image>(mask_image_topic_, rclcpp::SensorDataQoS());
  birdseye_mask_pub_ = create_publisher<sensor_msgs::msg::Image>(birdseye_mask_topic_, rclcpp::SensorDataQoS());
  birdseye_debug_pub_ = create_publisher<sensor_msgs::msg::Image>(birdseye_debug_topic_, rclcpp::SensorDataQoS());

  valid_pub_ = create_publisher<std_msgs::msg::Bool>(valid_topic_, 10);
  degraded_pub_ = create_publisher<std_msgs::msg::Bool>(degraded_topic_, 10);
  error_px_pub_ = create_publisher<std_msgs::msg::Float64>(error_px_topic_, 10);
  error_normalized_pub_ = create_publisher<std_msgs::msg::Float64>(error_normalized_topic_, 10);
  heading_error_pub_ = create_publisher<std_msgs::msg::Float64>(heading_error_topic_, 10);
  control_error_pub_ = create_publisher<std_msgs::msg::Float64>(control_error_topic_, 10);

  RCLCPP_INFO(
    get_logger(),
    "Lane perception started: BEV sliding-window control + Canny/Hough debug. Input: %s",
    input_topic_.c_str());
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
  // 3. Canny + Hough retained as a diagnostic/thesis output
  // ==========================================================

  cv::Mat blurred;
  cv::GaussianBlur(
    white_mask,
    blurred,
    cv::Size(gaussian_kernel_, gaussian_kernel_),
    0.0);

  cv::Mat edges;
  cv::Canny(blurred, edges, canny_low_, canny_high_);

  cv::Mat hough_roi_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
  std::vector<cv::Point> hough_roi{
    cv::Point(static_cast<int>(width * perspective_bottom_left_ratio_), static_cast<int>(bottom_y)),
    cv::Point(static_cast<int>(width * perspective_top_left_ratio_), static_cast<int>(top_y)),
    cv::Point(static_cast<int>(width * perspective_top_right_ratio_), static_cast<int>(top_y)),
    cv::Point(static_cast<int>(width * perspective_bottom_right_ratio_), static_cast<int>(bottom_y))
  };
  cv::fillConvexPoly(hough_roi_mask, hough_roi, cv::Scalar(255));

  cv::Mat roi_edges;
  cv::bitwise_and(edges, hough_roi_mask, roi_edges);

  std::vector<cv::Vec4i> hough_lines;
  cv::HoughLinesP(
    roi_edges,
    hough_lines,
    1.0,
    CV_PI / 180.0,
    hough_threshold_,
    hough_min_line_length_,
    hough_max_line_gap_);

  cv::Mat debug = frame.clone();
  cv::polylines(
    debug,
    std::vector<std::vector<cv::Point>>{hough_roi},
    true,
    cv::Scalar(0, 165, 255),
    2);

  for (const auto & line : hough_lines) {
    cv::line(
      debug,
      cv::Point(line[0], line[1]),
      cv::Point(line[2], line[3]),
      cv::Scalar(0, 255, 255),
      1);
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

  const std::string camera_status =
    detection_valid ? ("LANE: " + state_text) : ("LANE: LOST - " + invalid_reason);

  cv::putText(
    debug,
    camera_status,
    cv::Point(20, 30),
    cv::FONT_HERSHEY_SIMPLEX,
    0.68,
    status_color,
    2);

  cv::putText(
    debug,
    "CONTROL: BEV + SLIDING WINDOW (Hough debug only)",
    cv::Point(20, 58),
    cv::FONT_HERSHEY_SIMPLEX,
    0.48,
    cv::Scalar(255, 255, 255),
    1);

  std::ostringstream hough_text;
  hough_text << "Hough segments: " << hough_lines.size();
  cv::putText(
    debug,
    hough_text.str(),
    cv::Point(20, height - 20),
    cv::FONT_HERSHEY_SIMPLEX,
    0.50,
    cv::Scalar(255, 255, 255),
    1);

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

  auto debug_msg =
    cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, debug).toImageMsg();
  debug_image_pub_->publish(*debug_msg);

  auto edges_msg =
    cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, roi_edges).toImageMsg();
  edges_image_pub_->publish(*edges_msg);

  auto mask_msg =
    cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, white_mask).toImageMsg();
  mask_image_pub_->publish(*mask_msg);

  auto bird_mask_msg =
    cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, birdseye_mask).toImageMsg();
  birdseye_mask_pub_->publish(*bird_mask_msg);

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
