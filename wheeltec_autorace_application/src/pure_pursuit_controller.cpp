#include "wheeltec_autorace_application/pure_pursuit_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

namespace wheeltec_autorace_application
{

PurePursuitController::PurePursuitController()
: Node("pure_pursuit_controller")
{
  error_topic_ = declare_parameter<std::string>(
    "error_topic", "/lane_detection/control_error_normalized");
  valid_topic_ = declare_parameter<std::string>(
    "valid_topic", "/lane_detection/valid");
  degraded_topic_ = declare_parameter<std::string>(
    "degraded_topic", "/lane_detection/degraded");
  turn_preview_topic_ = declare_parameter<std::string>(
    "turn_preview_topic", "/lane_detection/turn_preview");
  cmd_vel_topic_ = declare_parameter<std::string>(
    "cmd_vel_topic", "/cmd_vel");
  current_speed_topic_ = declare_parameter<std::string>(
    "current_speed_topic", "/control/pure_pursuit/current_speed");

  lateral_target_topic_ = declare_parameter<std::string>(
    "lateral_target_topic", "/control/pure_pursuit/lateral_target_m");
  alpha_topic_ = declare_parameter<std::string>(
    "alpha_topic", "/control/pure_pursuit/alpha");
  curvature_topic_ = declare_parameter<std::string>(
    "curvature_topic", "/control/pure_pursuit/curvature");
  steering_angle_topic_ = declare_parameter<std::string>(
    "steering_angle_topic", "/control/pure_pursuit/steering_angle");
  output_topic_ = declare_parameter<std::string>(
    "output_topic", "/control/pure_pursuit/output");
  active_topic_ = declare_parameter<std::string>(
    "active_topic", "/control/pure_pursuit/active");

  wheel_base_m_ = declare_parameter<double>("wheel_base_m", 0.262);
  lookahead_distance_m_ =
    declare_parameter<double>("lookahead_distance_m", 0.55);
  straight_lookahead_distance_m_ =
    declare_parameter<double>("straight_lookahead_distance_m", 0.72);
  normalized_lateral_scale_m_ =
    declare_parameter<double>("normalized_lateral_scale_m", 0.70);
  max_steering_angle_rad_ =
    declare_parameter<double>("max_steering_angle_rad", 0.40);
  steering_sign_ =
    declare_parameter<double>("steering_sign", -1.0);

  error_filter_alpha_ =
    declare_parameter<double>("error_filter_alpha", 0.20);
  error_deadband_ =
    declare_parameter<double>("error_deadband", 0.012);
  normal_error_limit_ =
    declare_parameter<double>("normal_error_limit", 0.45);
  degraded_error_limit_ =
    declare_parameter<double>("degraded_error_limit", 0.25);

  perception_timeout_sec_ =
    declare_parameter<double>("perception_timeout_sec", 0.15);
  control_frequency_hz_ =
    declare_parameter<double>("control_frequency_hz", 40.0);

  max_linear_speed_ =
    declare_parameter<double>("max_linear_speed", 1.00);
  min_linear_speed_ =
    declare_parameter<double>("min_linear_speed", 0.25);
  degraded_linear_speed_ =
    declare_parameter<double>("degraded_linear_speed", 0.18);

  max_lateral_accel_mps2_ =
    declare_parameter<double>("max_lateral_accel_mps2", 0.20);

  max_acceleration_ =
    declare_parameter<double>("max_acceleration", 0.18);
  max_deceleration_ =
    declare_parameter<double>("max_deceleration", 2.50);

  angular_rate_limit_ =
    declare_parameter<double>("angular_rate_limit", 3.0);

  turn_preview_enabled_ =
    declare_parameter<bool>("turn_preview_enabled", true);
  turn_preview_timeout_sec_ =
    declare_parameter<double>("turn_preview_timeout_sec", 0.25);
  turn_preview_min_speed_ =
    declare_parameter<double>("turn_preview_min_speed", 0.18);

  wheel_base_m_ = std::max(wheel_base_m_, 1e-3);
  lookahead_distance_m_ = std::max(lookahead_distance_m_, 0.05);
  straight_lookahead_distance_m_ =
    std::max(straight_lookahead_distance_m_, lookahead_distance_m_);
  normalized_lateral_scale_m_ =
    std::max(normalized_lateral_scale_m_, 1e-3);
  max_steering_angle_rad_ =
    std::clamp(std::abs(max_steering_angle_rad_), 0.01, 1.2);

  error_filter_alpha_ =
    std::clamp(error_filter_alpha_, 0.0, 1.0);
  error_deadband_ = std::max(error_deadband_, 0.0);
  normal_error_limit_ =
    std::clamp(std::abs(normal_error_limit_), 0.01, 1.0);
  degraded_error_limit_ =
    std::clamp(
      std::abs(degraded_error_limit_),
      0.01,
      normal_error_limit_);

  perception_timeout_sec_ =
    std::max(perception_timeout_sec_, 0.01);
  control_frequency_hz_ =
    std::max(control_frequency_hz_, 1.0);

  max_linear_speed_ = std::max(max_linear_speed_, 0.0);
  min_linear_speed_ =
    std::clamp(min_linear_speed_, 0.0, max_linear_speed_);
  degraded_linear_speed_ =
    std::clamp(degraded_linear_speed_, 0.0, max_linear_speed_);

  max_lateral_accel_mps2_ =
    std::max(max_lateral_accel_mps2_, 1e-4);
  max_acceleration_ = std::max(max_acceleration_, 0.0);
  max_deceleration_ = std::max(max_deceleration_, 0.0);
  angular_rate_limit_ = std::max(angular_rate_limit_, 0.0);

  turn_preview_timeout_sec_ =
    std::max(turn_preview_timeout_sec_, 0.01);
  turn_preview_min_speed_ =
    std::clamp(turn_preview_min_speed_, 0.0, max_linear_speed_);

  enabled_ = declare_parameter<bool>("enabled_on_start", false);

  error_sub_ = create_subscription<std_msgs::msg::Float64>(
    error_topic_, 10,
    std::bind(
      &PurePursuitController::errorCallback,
      this,
      std::placeholders::_1));

  valid_sub_ = create_subscription<std_msgs::msg::Bool>(
    valid_topic_, 10,
    std::bind(
      &PurePursuitController::validCallback,
      this,
      std::placeholders::_1));

  degraded_sub_ = create_subscription<std_msgs::msg::Bool>(
    degraded_topic_, 10,
    std::bind(
      &PurePursuitController::degradedCallback,
      this,
      std::placeholders::_1));

  turn_preview_sub_ = create_subscription<std_msgs::msg::Float64>(
    turn_preview_topic_, 10,
    std::bind(
      &PurePursuitController::turnPreviewCallback,
      this,
      std::placeholders::_1));

  cmd_vel_pub_ =
    create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10);

  current_speed_pub_ =
    create_publisher<std_msgs::msg::Float64>(
      current_speed_topic_, 10);

  lateral_target_pub_ =
    create_publisher<std_msgs::msg::Float64>(lateral_target_topic_, 10);
  alpha_pub_ =
    create_publisher<std_msgs::msg::Float64>(alpha_topic_, 10);
  curvature_pub_ =
    create_publisher<std_msgs::msg::Float64>(curvature_topic_, 10);
  steering_angle_pub_ =
    create_publisher<std_msgs::msg::Float64>(steering_angle_topic_, 10);
  output_pub_ =
    create_publisher<std_msgs::msg::Float64>(output_topic_, 10);
  active_pub_ =
    create_publisher<std_msgs::msg::Bool>(active_topic_, 10);

  enable_service_ = create_service<std_srvs::srv::SetBool>(
    "/pure_pursuit_controller/enable",
    std::bind(
      &PurePursuitController::enableCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  const auto period =
    std::chrono::duration<double>(1.0 / control_frequency_hz_);

  control_timer_ =
    create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(
        &PurePursuitController::controlLoop,
        this));

  last_control_time_ = SteadyClock::now();

  RCLCPP_INFO(
    get_logger(),
    "Pure Pursuit v3: Ld=%.3f..%.3f m, vmax=%.3f m/s, alat=%.3f m/s^2",
    lookahead_distance_m_,
    straight_lookahead_distance_m_,
    max_linear_speed_,
    max_lateral_accel_mps2_);
}

void PurePursuitController::errorCallback(
  const std_msgs::msg::Float64::ConstSharedPtr msg)
{
  latest_error_ = msg->data;
  have_error_ = true;
  last_error_time_ = SteadyClock::now();
}

void PurePursuitController::validCallback(
  const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  lane_valid_ = msg->data;
}

void PurePursuitController::degradedCallback(
  const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  lane_degraded_ = msg->data;
}

void PurePursuitController::turnPreviewCallback(
  const std_msgs::msg::Float64::ConstSharedPtr msg)
{
  latest_turn_preview_ = std::clamp(msg->data, 0.0, 1.0);
  have_turn_preview_ = true;
  last_turn_preview_time_ = SteadyClock::now();
}

void PurePursuitController::enableCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  enabled_ = request->data;
  resetController();

  if (enabled_) {
    response->success = true;
    response->message = "Pure Pursuit controller enabled";
    RCLCPP_INFO(get_logger(), "Pure Pursuit controller ENABLED");
  } else {
    publishCommand(0.0, 0.0);
    response->success = true;
    response->message =
      "Pure Pursuit controller disabled and vehicle stopped";
    RCLCPP_INFO(get_logger(), "Pure Pursuit controller DISABLED");
  }
}

void PurePursuitController::resetController()
{
  filtered_error_ = 0.0;
  have_filtered_error_ = false;
  last_commanded_angular_z_ = 0.0;
  last_commanded_linear_x_ = 0.0;
}

void PurePursuitController::publishCommand(
  const double linear_x,
  const double angular_z)
{
  geometry_msgs::msg::Twist command;
  command.linear.x = linear_x;
  command.angular.z = angular_z;
  cmd_vel_pub_->publish(command);

  std_msgs::msg::Float64 speed_msg;
  speed_msg.data = linear_x;
  current_speed_pub_->publish(speed_msg);
}

void PurePursuitController::publishDebug(
  const double lateral_target_m,
  const double alpha_rad,
  const double curvature,
  const double steering_angle_rad,
  const double angular_command,
  const bool active)
{
  std_msgs::msg::Float64 value;

  value.data = lateral_target_m;
  lateral_target_pub_->publish(value);

  value.data = alpha_rad;
  alpha_pub_->publish(value);

  value.data = curvature;
  curvature_pub_->publish(value);

  value.data = steering_angle_rad;
  steering_angle_pub_->publish(value);

  value.data = angular_command;
  output_pub_->publish(value);

  std_msgs::msg::Bool active_msg;
  active_msg.data = active;
  active_pub_->publish(active_msg);
}

double PurePursuitController::limitAngularRate(
  const double target,
  const double dt)
{
  if (angular_rate_limit_ <= 0.0) {
    last_commanded_angular_z_ = target;
    return target;
  }

  const double max_delta =
    angular_rate_limit_ * dt;

  const double next =
    std::clamp(
      target,
      last_commanded_angular_z_ - max_delta,
      last_commanded_angular_z_ + max_delta);

  last_commanded_angular_z_ = next;
  return next;
}

double PurePursuitController::limitLinearSpeed(
  const double target,
  const double dt)
{
  const double delta =
    target - last_commanded_linear_x_;

  if (delta >= 0.0) {
    last_commanded_linear_x_ +=
      std::min(delta, max_acceleration_ * dt);
  } else {
    last_commanded_linear_x_ +=
      std::max(delta, -max_deceleration_ * dt);
  }

  last_commanded_linear_x_ =
    std::clamp(
      last_commanded_linear_x_,
      0.0,
      max_linear_speed_);

  return last_commanded_linear_x_;
}

void PurePursuitController::controlLoop()
{
  const auto now = SteadyClock::now();

  double dt =
    std::chrono::duration<double>(
      now - last_control_time_).count();

  last_control_time_ = now;

  if (dt <= 0.0 || dt > 0.5) {
    dt = 1.0 / control_frequency_hz_;
  }

  if (!enabled_) {
    return;
  }

  bool perception_stale = !have_error_;

  if (have_error_) {
    const double error_age =
      std::chrono::duration<double>(
        now - last_error_time_).count();

    perception_stale =
      error_age > perception_timeout_sec_;
  }

  if (!lane_valid_ || perception_stale) {
    resetController();
    publishCommand(0.0, 0.0);
    publishDebug(0.0, 0.0, 0.0, 0.0, 0.0, false);
    return;
  }

  if (!have_filtered_error_) {
    filtered_error_ = latest_error_;
    have_filtered_error_ = true;
  } else {
    filtered_error_ =
      error_filter_alpha_ * latest_error_
      + (1.0 - error_filter_alpha_) * filtered_error_;
  }

  const double error_limit =
    lane_degraded_
    ? degraded_error_limit_
    : normal_error_limit_;

  double normalized_error =
    std::clamp(filtered_error_, -error_limit, error_limit);

  if (std::abs(normalized_error) < error_deadband_) {
    normalized_error = 0.0;
  }

  const double lateral_target_m =
    normalized_error * normalized_lateral_scale_m_;

  // Continuous preview-adaptive lookahead:
  // straight: longer lookahead for calm steering
  // corner: smoothly returns to the base lookahead
  double preview_risk = 1.0;
  bool preview_fresh = false;

  if (turn_preview_enabled_ && have_turn_preview_) {
    const double preview_age =
      std::chrono::duration<double>(
        now - last_turn_preview_time_).count();

    preview_fresh =
      preview_age <= turn_preview_timeout_sec_;

    if (preview_fresh) {
      preview_risk =
        std::clamp(latest_turn_preview_, 0.0, 1.0);
    }
  }

  if (lane_degraded_) {
    preview_risk = 1.0;
  }

  const double effective_lookahead_m =
    lookahead_distance_m_
    + (1.0 - preview_risk) *
      (straight_lookahead_distance_m_ - lookahead_distance_m_);

  const double target_distance_m =
    std::hypot(
      effective_lookahead_m,
      lateral_target_m);

  const double alpha_rad =
    std::atan2(
      lateral_target_m,
      effective_lookahead_m);

  double curvature = 0.0;

  if (target_distance_m > 1e-6) {
    curvature =
      2.0 * std::sin(alpha_rad) / target_distance_m;
  }

  double steering_angle_rad =
    std::atan(wheel_base_m_ * curvature);

  steering_angle_rad =
    std::clamp(
      steering_angle_rad,
      -max_steering_angle_rad_,
      max_steering_angle_rad_);

  curvature =
    std::tan(steering_angle_rad) / wheel_base_m_;

  // Curvature based speed limit:
  // a_lat = v^2 * |curvature|
  // v = sqrt(a_lat_max / |curvature|)
  double curvature_speed_limit =
    max_linear_speed_;

  const double abs_curvature =
    std::abs(curvature);

  if (abs_curvature > 1e-4) {
    curvature_speed_limit =
      std::sqrt(
        max_lateral_accel_mps2_ /
        abs_curvature);
  }

  curvature_speed_limit =
    std::clamp(
      curvature_speed_limit,
      min_linear_speed_,
      max_linear_speed_);

  // Continuous RAW-camera turn-preview speed limit.
  // No threshold / full-brake threshold / exponent:
  // 0.0 -> max speed
  // 1.0 -> preview minimum speed
  double preview_speed_limit =
    max_linear_speed_;

  if (turn_preview_enabled_ && preview_fresh) {
    preview_speed_limit =
      max_linear_speed_
      - preview_risk *
        (max_linear_speed_ - turn_preview_min_speed_);
  }

  double target_speed =
    std::min(
      curvature_speed_limit,
      preview_speed_limit);

  if (lane_degraded_) {
    target_speed =
      std::min(
        target_speed,
        degraded_linear_speed_);
  }

  const double commanded_speed =
    limitLinearSpeed(
      target_speed,
      dt);

  // No redundant yaw-rate clamp:
  // steering_angle_rad is already physically limited to 0.4 rad.
  const double target_angular =
    steering_sign_ *
    commanded_speed *
    curvature;

  const double angular_command =
    limitAngularRate(
      target_angular,
      dt);

  publishCommand(
    commanded_speed,
    angular_command);

  publishDebug(
    lateral_target_m,
    alpha_rad,
    curvature,
    steering_angle_rad,
    angular_command,
    true);
}

}  // namespace wheeltec_autorace_application

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<
      wheeltec_autorace_application::PurePursuitController>());
  rclcpp::shutdown();
  return 0;
}
