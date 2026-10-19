#include "wheeltec_autorace_application/pid_lane_controller.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace wheeltec_autorace_application::pid_speed
{
struct PreviewLimit
{
  double risk;
  double speed_cap;
};

// Preview is a dimensionless evidence score, not distance to the corner.
// Configuration is validated by the node. Missing/invalid/stale input may
// reduce the allowed speed, but must never grant permission to accelerate.
inline PreviewLimit previewLimit(const bool enabled, const bool fresh,
  const double score, const double activation, const double vmax, const double corner_speed)
{
  if (!enabled) { return {0.0, vmax}; }
  const bool valid = fresh && std::isfinite(score) && score >= 0.0 && score <= 1.0;
  const double risk = valid ? std::clamp((score - activation) / (1.0 - activation), 0.0, 1.0) : 1.0;
  return {risk, vmax - risk * (vmax - corner_speed)};
}
}  // namespace wheeltec_autorace_application::pid_speed

namespace wheeltec_autorace_application
{

PidLaneController::PidLaneController()
: Node("pid_lane_controller")
{
  error_topic_ = declare_parameter<std::string>("error_topic", "/lane_detection/control_error_normalized");
  valid_topic_ = declare_parameter<std::string>("valid_topic", "/lane_detection/valid");
  degraded_topic_ = declare_parameter<std::string>("degraded_topic", "/lane_detection/degraded");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  turn_preview_topic_ = declare_parameter<std::string>("turn_preview_topic", "/lane_detection/turn_preview");
  current_speed_topic_ = declare_parameter<std::string>("current_speed_topic", "/control/pid/current_speed");

  p_term_topic_ = declare_parameter<std::string>("p_term_topic", "/control/pid/p_term");
  i_term_topic_ = declare_parameter<std::string>("i_term_topic", "/control/pid/i_term");
  d_term_topic_ = declare_parameter<std::string>("d_term_topic", "/control/pid/d_term");
  output_topic_ = declare_parameter<std::string>("output_topic", "/control/pid/output");
  active_topic_ = declare_parameter<std::string>("active_topic", "/control/pid/active");
  recovery_active_topic_ = declare_parameter<std::string>(
    "recovery_active_topic", "/control/pid/recovery_active");

  kp_ = declare_parameter<double>("kp", 1.75);
  // Error-dependent proportional gain.
  soft_center_enabled_ = declare_parameter<bool>("soft_center_enabled", true);
  // Linear fraction of the P law: 0 is quadratic, 1 is linear.
  soft_center_min_gain_ = declare_parameter<double>("soft_center_min_gain", 0.35);
  ki_ = declare_parameter<double>("ki", 0.0);
  kd_ = declare_parameter<double>("kd", 0.28);
  integral_limit_ = declare_parameter<double>("integral_limit", 0.40);
  derivative_filter_alpha_ = declare_parameter<double>("derivative_filter_alpha", 0.18);
  error_filter_alpha_ = declare_parameter<double>("error_filter_alpha", 0.40);
  error_deadband_ = declare_parameter<double>("error_deadband", 0.012);
  normal_error_limit_ = declare_parameter<double>("normal_error_limit", 0.45);
  degraded_error_limit_ = declare_parameter<double>("degraded_error_limit", 0.25);
  degraded_kp_scale_ = declare_parameter<double>("degraded_kp_scale", 0.60);

  max_angular_z_ = declare_parameter<double>("max_angular_z", 0.70);
  degraded_max_angular_z_ = declare_parameter<double>("degraded_max_angular_z", 0.40);
  max_linear_speed_ = declare_parameter<double>("max_linear_speed", 0.50);
  min_linear_speed_ = declare_parameter<double>("min_linear_speed", 0.16);
  degraded_linear_speed_ = declare_parameter<double>("degraded_linear_speed", 0.12);
  steering_sign_ = declare_parameter<double>("steering_sign", -1.0);
  use_turn_preview_ = declare_parameter<bool>("use_turn_preview", true);
  preview_speed_ = declare_parameter<double>("preview_speed", 0.25);
  preview_activation_score_ = declare_parameter<double>("preview_activation_score", 0.60);
  preview_timeout_sec_ = declare_parameter<double>("preview_timeout_sec", 0.30);
  // Longitudinal command rate limits.
  max_acceleration_ = declare_parameter<double>("max_acceleration", 0.45);
  max_deceleration_ = declare_parameter<double>("max_deceleration", 1.50);
  // Exponent of the steering-demand speed curve.
  steering_speed_exponent_ = declare_parameter<double>("steering_speed_exponent", 0.65);
  // Error tolerance for speed planning; steering uses the full error.
  full_speed_error_ = declare_parameter<double>("full_speed_error", 0.02);

  steering_rise_rate_ = declare_parameter<double>("steering_rise_rate", 4.0);
  steering_center_rate_ = declare_parameter<double>("steering_center_rate", 5.0);
  steering_reverse_rate_ = declare_parameter<double>("steering_reverse_rate", 1.4);

  // Optional fixed-command recovery after loss of lane evidence.
  recovery_enabled_ = declare_parameter<bool>("recovery_enabled", false);
  recovery_timeout_sec_ = declare_parameter<double>("recovery_timeout_sec", 0.60);
  recovery_entry_grace_sec_ = declare_parameter<double>("recovery_entry_grace_sec", 0.25);
  recovery_entry_error_ = declare_parameter<double>("recovery_entry_error", 0.10);
  recovery_entry_angular_z_ = declare_parameter<double>("recovery_entry_angular_z", 0.20);
  recovery_linear_speed_ = declare_parameter<double>("recovery_linear_speed", 0.025);
  recovery_angular_z_ = declare_parameter<double>("recovery_angular_z", 0.35);

  control_frequency_hz_ = declare_parameter<double>("control_frequency_hz", 40.0);
  perception_timeout_sec_ = declare_parameter<double>("perception_timeout_sec", 0.15);
  enabled_ = declare_parameter<bool>("enabled_on_start", false);

  control_frequency_hz_ = std::max(control_frequency_hz_, 1.0);
  derivative_filter_alpha_ = std::clamp(derivative_filter_alpha_, 0.0, 1.0);
  error_filter_alpha_ = std::clamp(error_filter_alpha_, 0.0, 1.0);
  error_deadband_ = std::max(error_deadband_, 0.0);
  normal_error_limit_ = std::clamp(std::abs(normal_error_limit_), 0.01, 1.0);
  degraded_error_limit_ = std::clamp(std::abs(degraded_error_limit_), 0.01, normal_error_limit_);
  degraded_kp_scale_ = std::clamp(degraded_kp_scale_, 0.0, 1.0);

  max_angular_z_ = std::max(std::abs(max_angular_z_), 1e-6);
  degraded_max_angular_z_ = std::clamp(std::abs(degraded_max_angular_z_), 1e-6, max_angular_z_);
  max_linear_speed_ = std::max(max_linear_speed_, 0.0);
  min_linear_speed_ = std::clamp(min_linear_speed_, 0.0, max_linear_speed_);
  degraded_linear_speed_ = std::clamp(degraded_linear_speed_, 0.0, max_linear_speed_);

  if (!std::isfinite(max_linear_speed_) || !std::isfinite(preview_speed_) ||
    preview_speed_ < 0.0 || preview_speed_ > max_linear_speed_ ||
    !std::isfinite(preview_activation_score_) || preview_activation_score_ < 0.0 ||
    preview_activation_score_ >= 1.0 || !std::isfinite(preview_timeout_sec_) ||
    preview_timeout_sec_ <= 0.0 || !std::isfinite(max_acceleration_) ||
    max_acceleration_ <= 0.0 || !std::isfinite(max_deceleration_) || max_deceleration_ <= 0.0 ||
    !std::isfinite(steering_speed_exponent_) || steering_speed_exponent_ <= 0.0 ||
    steering_speed_exponent_ > 1.0)
  {
    throw std::invalid_argument("Invalid PID preview/speed parameters; steering_speed_exponent must be in (0, 1]");
  }

  if (!std::isfinite(full_speed_error_) || full_speed_error_ < 0.0 ||
    full_speed_error_ >= degraded_error_limit_)
  {
    throw std::invalid_argument("full_speed_error must be finite, nonnegative and below degraded_error_limit");
  }

  if (!std::isfinite(soft_center_min_gain_) || soft_center_min_gain_ < 0.0 ||
    soft_center_min_gain_ > 1.0)
  {
    throw std::invalid_argument("soft_center_min_gain must be finite and in [0, 1]");
  }

  steering_rise_rate_ = std::max(steering_rise_rate_, 0.0);
  steering_center_rate_ = std::max(steering_center_rate_, 0.0);
  steering_reverse_rate_ = std::max(steering_reverse_rate_, 0.0);

  recovery_timeout_sec_ = std::max(recovery_timeout_sec_, 0.0);
  recovery_entry_grace_sec_ = std::max(recovery_entry_grace_sec_, 0.0);
  recovery_entry_error_ = std::max(recovery_entry_error_, 0.0);
  recovery_entry_angular_z_ = std::max(recovery_entry_angular_z_, 0.0);
  recovery_linear_speed_ = std::clamp(recovery_linear_speed_, 0.0, max_linear_speed_);
  recovery_angular_z_ = std::clamp(std::abs(recovery_angular_z_), 0.0, max_angular_z_);

  error_sub_ = create_subscription<std_msgs::msg::Float64>(
    error_topic_, 10, std::bind(&PidLaneController::errorCallback, this, std::placeholders::_1));

  valid_sub_ = create_subscription<std_msgs::msg::Bool>(
    valid_topic_, 10, std::bind(&PidLaneController::validCallback, this, std::placeholders::_1));

  degraded_sub_ = create_subscription<std_msgs::msg::Bool>(
    degraded_topic_, 10, std::bind(&PidLaneController::degradedCallback, this, std::placeholders::_1));

  preview_sub_ = create_subscription<std_msgs::msg::Float64>(
    turn_preview_topic_, rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&PidLaneController::previewCallback, this, std::placeholders::_1));

  status_pub_ = create_publisher<std_msgs::msg::String>("/control/pid/status", 10);
  speed_pub_ = create_publisher<std_msgs::msg::Float64>(current_speed_topic_, 10);

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  p_term_pub_ = create_publisher<std_msgs::msg::Float64>(p_term_topic_, 10);
  i_term_pub_ = create_publisher<std_msgs::msg::Float64>(i_term_topic_, 10);
  d_term_pub_ = create_publisher<std_msgs::msg::Float64>(d_term_topic_, 10);
  output_pub_ = create_publisher<std_msgs::msg::Float64>(output_topic_, 10);
  active_pub_ = create_publisher<std_msgs::msg::Bool>(active_topic_, 10);
  recovery_active_pub_ = create_publisher<std_msgs::msg::Bool>(recovery_active_topic_, 10);

  enable_service_ = create_service<std_srvs::srv::SetBool>(
    "/pid_lane_controller/enable",
    std::bind(&PidLaneController::enableCallback, this, std::placeholders::_1, std::placeholders::_2));

  const int period_ms = std::max(1, static_cast<int>(std::round(1000.0 / control_frequency_hz_)));

  const auto now = SteadyClock::now();
  last_control_time_ = now;
  last_valid_tracking_time_ = now;
  recovery_start_time_ = now;

  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(period_ms), std::bind(&PidLaneController::controlLoop, this));

  RCLCPP_INFO(
    get_logger(),
    "PID controller started %s. Kp=%.2f Kd=%.2f max_w=%.2f recovery=%s",
    enabled_ ? "ENABLED" : "DISABLED", kp_, kd_, max_angular_z_, recovery_enabled_ ? "ON" : "OFF");
  RCLCPP_INFO(get_logger(),
    "PID steering: soft_center=%s min_gain=%.3f error_deadband=%.4f; reversal rate persists through zero",
    soft_center_enabled_ ? "ON" : "OFF", soft_center_min_gain_, error_deadband_);
  RCLCPP_INFO(get_logger(),
    "PID speed policy: vmax=%.3f preview=%s activation=%.3f preview_speed=%.3f "
    "preview_timeout_wall_s=%.3f command_accel=%.3f command_decel=%.3f speed_topic=%s steering_speed_exponent=%.3f",
    max_linear_speed_, use_turn_preview_ ? "ON" : "OFF", preview_activation_score_,
    preview_speed_, preview_timeout_sec_, max_acceleration_, max_deceleration_,
    current_speed_topic_.c_str(), steering_speed_exponent_);
  RCLCPP_INFO(get_logger(), "PID full-speed CTRL error band: +/-%.4f (speed planning only)",
    full_speed_error_);
}

void PidLaneController::errorCallback(
  const std_msgs::msg::Float64::ConstSharedPtr msg)
{
  if (!std::isfinite(msg->data)) {
    diagnostic_nonfinite_input_ = true;
    diagnostic_target_speed_ = 0.0;
    diagnostic_speed_limit_ = "STOP";
    // Discard invalid input and clear recovery history.
    latest_error_ = 0.0;
    have_error_ = false;
    lane_valid_ = false;
    have_tracking_history_ = false;
    recovery_active_ = false;
    resetPid();
    last_commanded_angular_z_ = 0.0;
    last_commanded_linear_x_ = 0.0;
    if (enabled_) {
      publishCommand(0.0, 0.0);
      publishDebug(0.0, 0.0, 0.0, 0.0, false);
      publishRecoveryState(false);
      publishStatus("NONFINITE_ERROR", -1.0, -1.0);
    }
    return;
  }

  diagnostic_nonfinite_input_ = false;
  latest_error_ = std::clamp(msg->data, -1.0, 1.0);
  have_error_ = true;
  last_error_time_ = SteadyClock::now();
}

void PidLaneController::validCallback(
  const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  lane_valid_ = msg->data;
}

void PidLaneController::degradedCallback(
  const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  lane_degraded_ = msg->data;
}

void PidLaneController::previewCallback(
  const std_msgs::msg::Float64::ConstSharedPtr msg)
{
  have_preview_ = true;
  preview_valid_ = std::isfinite(msg->data) && msg->data >= 0.0 && msg->data <= 1.0;
  latest_preview_ = preview_valid_ ? msg->data : 0.0;
  last_preview_time_ = SteadyClock::now();
}

void PidLaneController::enableCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  enabled_ = request->data;
  diagnostic_target_speed_ = 0.0;
  diagnostic_speed_limit_ = "STOP";
  diagnostic_nonfinite_input_ = false;
  resetPid();
  recovery_active_ = false;
  last_commanded_angular_z_ = 0.0;
  last_commanded_linear_x_ = 0.0;
  previous_lane_degraded_ = false;
  publishRecoveryState(false);

  if (enabled_) {
    have_error_ = false;
    have_preview_ = false;
    preview_valid_ = false;
    lane_valid_ = false;
    lane_degraded_ = false;
    have_tracking_history_ = false;

    response->success = true;
    response->message = "PID lane controller enabled";
    RCLCPP_INFO(get_logger(), "PID controller ENABLED");
    publishStatus("WAITING_ERROR", -1.0, -1.0);
  } else {
    publishCommand(0.0, 0.0);
    response->success = true;
    response->message = "PID lane controller disabled and vehicle stopped";
    RCLCPP_INFO(get_logger(), "PID controller DISABLED");
    publishStatus("DISABLED", -1.0, -1.0);
  }
}

void PidLaneController::resetPid()
{
  integral_ = 0.0;
  previous_error_ = 0.0;
  filtered_error_ = 0.0;
  filtered_derivative_ = 0.0;
  have_filtered_error_ = false;
  have_previous_error_ = false;
  last_steering_direction_ = 0;
  steering_reversal_active_ = false;
  diagnostic_control_error_ = 0.0;
  diagnostic_p_scale_ = soft_center_enabled_ ? soft_center_min_gain_ : 1.0;
  diagnostic_speed_error_ = 0.0;
}

void PidLaneController::publishCommand(
  const double linear_x,
  const double angular_z)
{
  geometry_msgs::msg::Twist command;
  command.linear.x = linear_x;
  command.angular.z = angular_z;
  cmd_vel_pub_->publish(command);
  publishSpeed(linear_x);
}

void PidLaneController::publishSpeed(const double linear_x)
{
  diagnostic_command_speed_ = linear_x;
  std_msgs::msg::Float64 speed;
  speed.data = linear_x;
  speed_pub_->publish(speed);
}

void PidLaneController::publishDebug(
  const double p_term,
  const double i_term,
  const double d_term,
  const double control_output,
  const bool active)
{
  std_msgs::msg::Float64 p_msg;
  p_msg.data = p_term;
  p_term_pub_->publish(p_msg);

  std_msgs::msg::Float64 i_msg;
  i_msg.data = i_term;
  i_term_pub_->publish(i_msg);

  std_msgs::msg::Float64 d_msg;
  d_msg.data = d_term;
  d_term_pub_->publish(d_msg);

  std_msgs::msg::Float64 output_msg;
  output_msg.data = control_output;
  output_pub_->publish(output_msg);

  std_msgs::msg::Bool active_msg;
  active_msg.data = active;
  active_pub_->publish(active_msg);
}

void PidLaneController::publishRecoveryState(const bool active)
{
  std_msgs::msg::Bool msg;
  msg.data = active;
  recovery_active_pub_->publish(msg);
}

void PidLaneController::publishStatus(
  const char * reason,
  const double error_age,
  const double raw_dt)
{
  // Report transitions and a one-second wall-time heartbeat.
  const auto now = SteadyClock::now();
  if (last_status_reason_ == reason &&
    std::chrono::duration<double>(now - last_status_time_).count() < 1.0)
  {
    return;
  }
  last_status_reason_ = reason;
  last_status_time_ = now;
  std::ostringstream text;
  text << std::fixed << std::setprecision(6)
       << reason
       << " enabled=" << enabled_
       << " valid=" << lane_valid_
       << " degraded=" << lane_degraded_
       << " have_error=" << have_error_
       << " nonfinite_input=" << diagnostic_nonfinite_input_
       << " error_rx_age_s=" << error_age
       << " error_stale=" << (!have_error_ ? 1 : error_age < 0.0 ? -1 :
    static_cast<int>(error_age > perception_timeout_sec_))
       << " raw_dt_s=" << raw_dt
       << " timeout_s=" << perception_timeout_sec_
       << " command_speed=" << diagnostic_command_speed_
       << " target_speed=" << diagnostic_target_speed_
       << " speed_limit=" << diagnostic_speed_limit_
       << " preview_state=" << diagnostic_preview_state_
       << " preview_score=" << latest_preview_
       << " preview_risk=" << diagnostic_preview_risk_
       << " preview_cap=" << diagnostic_preview_cap_ << " preview_rx_age_s=" << diagnostic_preview_age_;
  text << " soft_center=" << soft_center_enabled_
       << " soft_center_min_gain=" << soft_center_min_gain_
       << " filtered_error=" << filtered_error_
       << " error_deadband=" << error_deadband_
       << " control_error=" << diagnostic_control_error_
       << " p_scale=" << diagnostic_p_scale_
       << " reversal_active=" << steering_reversal_active_
       << " command_omega=" << last_commanded_angular_z_
       << " steering_speed_exponent=" << steering_speed_exponent_
       << " full_speed_error=" << full_speed_error_ << " speed_error=" << diagnostic_speed_error_;
  std_msgs::msg::String msg;
  msg.data = text.str();
  status_pub_->publish(msg);
}

double PidLaneController::limitSteeringRate(
  const double target,
  const double dt)
{
  const double current = last_commanded_angular_z_;
  const int target_direction = (target > 0.0) - (target < 0.0);
  if (last_steering_direction_ == 0 && current != 0.0) {
    last_steering_direction_ = (current > 0.0) - (current < 0.0);
  }
  if (target_direction != 0 && last_steering_direction_ != 0) {
    // Remember reversal across a zero output. Cancel if target returns to
    // the original side. A momentary zero target does not lose the history.
    steering_reversal_active_ = target_direction != last_steering_direction_;
  }

  if (target * current < 0.0) {
    // Unwind first; do not spend this cycle's remaining time building a
    // command on the opposite side.
    const double max_delta = steering_center_rate_ * dt;
    last_commanded_angular_z_ = current > 0.0 ?
      std::max(0.0, current - max_delta) : std::min(0.0, current + max_delta);
    return last_commanded_angular_z_;
  }

  const bool moving_toward_zero = std::abs(target) < std::abs(current);
  const double rate = moving_toward_zero ? steering_center_rate_ :
    steering_reversal_active_ ? steering_reverse_rate_ : steering_rise_rate_;
  const double max_delta = rate * dt;
  const double next = std::clamp(target, current - max_delta, current + max_delta);
  last_commanded_angular_z_ = next;
  if (next != 0.0 && last_steering_direction_ == 0) {
    last_steering_direction_ = (next > 0.0) - (next < 0.0);
  }
  if (target_direction != 0 && next == target) {
    last_steering_direction_ = target_direction;
    steering_reversal_active_ = false;
  }
  return next;
}

void PidLaneController::controlLoop()
{
  const auto now = SteadyClock::now();

  double dt = std::chrono::duration<double>(now - last_control_time_).count();
  last_control_time_ = now;
  const double diagnostic_raw_dt = dt;

  diagnostic_target_speed_ = 0.0;
  diagnostic_speed_limit_ = "STOP";
  diagnostic_preview_age_ = have_preview_ ?
    std::chrono::duration<double>(now - last_preview_time_).count() : -1.0;
  const bool preview_fresh = have_preview_ && preview_valid_ &&
    diagnostic_preview_age_ >= 0.0 && diagnostic_preview_age_ <= preview_timeout_sec_;
  diagnostic_preview_state_ = !use_turn_preview_ ? "OFF" :
    !have_preview_ ? "MISSING" : !preview_valid_ ? "INVALID" : preview_fresh ? "FRESH" : "STALE";
  const auto preview_limit = pid_speed::previewLimit(use_turn_preview_, preview_fresh,
    latest_preview_, preview_activation_score_, max_linear_speed_, preview_speed_);
  diagnostic_preview_cap_ = preview_limit.speed_cap;
  diagnostic_preview_risk_ = preview_limit.risk;

  if (!enabled_) {
    publishSpeed(0.0);  // Debug only: disabled PID does not publish /cmd_vel.
    publishDebug(0.0, 0.0, 0.0, 0.0, false);
    publishRecoveryState(false);
    publishStatus("DISABLED", -1.0, diagnostic_raw_dt);
    return;
  }

  bool perception_stale = !have_error_;
  double error_age = -1.0;
  if (have_error_) {
    error_age = std::chrono::duration<double>(now - last_error_time_).count();
    perception_stale = error_age > perception_timeout_sec_;
  }

  if (lane_valid_ && !perception_stale) {
    if (recovery_active_) {
      recovery_active_ = false;
      resetPid();
      RCLCPP_INFO(get_logger(), "Lane reacquired -> leaving RECOVERY mode");
    }

    if (dt <= 0.0 || dt > 0.5) {
      dt = 1.0 / control_frequency_hz_;
    }

    // Filter frame-to-frame jumps before evaluating the PID.
    if (!have_filtered_error_) {
      filtered_error_ = latest_error_;
      have_filtered_error_ = true;
    } else {
      filtered_error_ = error_filter_alpha_ * latest_error_ + (1.0 - error_filter_alpha_) * filtered_error_;
    }

    // Reset derivative history when the boundary estimate changes mode.
    if (lane_degraded_ != previous_lane_degraded_) {
      filtered_derivative_ = 0.0;
      have_previous_error_ = false;
      previous_lane_degraded_ = lane_degraded_;
    }

    const double error_limit = lane_degraded_ ? degraded_error_limit_ : normal_error_limit_;

    double error = std::clamp(filtered_error_, -error_limit, error_limit);
    if (std::abs(error) < error_deadband_) {
      error = 0.0;
    }

    const double effective_kp = lane_degraded_ ? kp_ * degraded_kp_scale_ : kp_;

    // Blend linear and quadratic P; both agree at the error limit.
    const double linear_p_term = effective_kp * error;
    const double p_scale = soft_center_enabled_ ?
      soft_center_min_gain_ + (1.0 - soft_center_min_gain_) *
      std::clamp(std::abs(error) / error_limit, 0.0, 1.0) : 1.0;
    const double p_term = linear_p_term * p_scale;
    diagnostic_control_error_ = error;
    diagnostic_p_scale_ = p_scale;

    integral_ += error * dt;
    integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
    const double i_term = ki_ * integral_;

    double raw_derivative = 0.0;
    if (have_previous_error_) {
      raw_derivative = (error - previous_error_) / dt;
    }

    filtered_derivative_ =
      derivative_filter_alpha_ * raw_derivative + (1.0 - derivative_filter_alpha_) * filtered_derivative_;

    const double d_term = kd_ * filtered_derivative_;

    previous_error_ = error;
    have_previous_error_ = true;

    const double active_max_angular = lane_degraded_ ? degraded_max_angular_z_ : max_angular_z_;

    const double raw_output = p_term + i_term + d_term;
    const double limited_output = std::clamp(raw_output, -active_max_angular, active_max_angular);

    const double target_angular = steering_sign_ * limited_output;

    const double angular_command = limitSteeringRate(target_angular, dt);

    // Apply the full-speed band to the speed-planning P term, retaining I and D.
    const double speed_error = full_speed_error_ == 0.0 ? error :
      std::copysign(error_limit * std::clamp(
        (std::abs(error) - full_speed_error_) / (error_limit - full_speed_error_), 0.0, 1.0), error);
    diagnostic_speed_error_ = speed_error;
    const double speed_linear_p = effective_kp * speed_error;
    // Bound steering demand using the linear and quadratic endpoints.
    const double speed_p = speed_linear_p * (soft_center_enabled_ ?
      std::clamp(std::abs(speed_error) / error_limit, 0.0, 1.0) : 1.0);
    const double speed_linear_output = std::clamp(
      speed_linear_p + i_term + d_term, -active_max_angular, active_max_angular);
    const double speed_shaped_output = std::clamp(
      speed_p + i_term + d_term, -active_max_angular, active_max_angular);
    const double speed_steering_demand = std::max(
      std::abs(steering_sign_ * speed_linear_output), std::abs(steering_sign_ * speed_shaped_output));
    const double steering_demand_ratio = std::clamp(speed_steering_demand / max_angular_z_, 0.0, 1.0);

    const double speed_ratio = std::pow(steering_demand_ratio, steering_speed_exponent_);

    double target_speed = max_linear_speed_ - speed_ratio * (max_linear_speed_ - min_linear_speed_);

    diagnostic_speed_limit_ = speed_ratio > 0.0 ? "STEERING" : "MAX_SPEED";
    if (lane_degraded_) {
      if (degraded_linear_speed_ < target_speed) { diagnostic_speed_limit_ = "DEGRADED"; }
      target_speed = std::min(target_speed, degraded_linear_speed_);
    }

    if (preview_limit.speed_cap < target_speed) {
      diagnostic_speed_limit_ = preview_fresh ? "PREVIEW" : "PREVIEW_FALLBACK";
      target_speed = preview_limit.speed_cap;
    }
    diagnostic_target_speed_ = target_speed;

    const double speed_delta = target_speed - last_commanded_linear_x_;

    double commanded_speed = last_commanded_linear_x_;

    if (speed_delta >= 0.0) {
      const double max_delta = max_acceleration_ * dt;

      commanded_speed += std::min(speed_delta, max_delta);
    }
    else {
      const double max_delta = max_deceleration_ * dt;

      commanded_speed += std::max(speed_delta, -max_delta);
    }

    commanded_speed = std::clamp(commanded_speed, 0.0, max_linear_speed_);

    if (lane_degraded_) {
      commanded_speed = std::min(commanded_speed, degraded_linear_speed_);
    }

    // Start the next acceleration ramp from the published speed.
    last_commanded_linear_x_ = commanded_speed;

    publishCommand(commanded_speed, angular_command);
    publishDebug(p_term, i_term, d_term, angular_command, true);
    publishRecoveryState(false);

    last_valid_error_ = error;
    last_tracking_angular_z_ = angular_command;
    last_valid_tracking_time_ = now;
    have_tracking_history_ = true;
    publishStatus(lane_degraded_ ? "RUNNING_DEGRADED" : "RUNNING_NORMAL", error_age, diagnostic_raw_dt);
    return;
  }

  // Optional recovery, bounded by its timeout and command limits.
  const double since_last_valid = std::chrono::duration<double>(now - last_valid_tracking_time_).count();

  if (!recovery_active_ && recovery_enabled_ && have_tracking_history_) {
    const bool recent_loss = since_last_valid <= recovery_entry_grace_sec_;
    const bool was_turning =
      std::abs(last_tracking_angular_z_) >= recovery_entry_angular_z_ ||
      std::abs(last_valid_error_) >= recovery_entry_error_;

    if (recent_loss && was_turning) {
      double turn_reference = last_tracking_angular_z_;
      if (std::abs(turn_reference) < 1e-6) {
        turn_reference = steering_sign_ * last_valid_error_;
      }

      if (std::abs(turn_reference) >= 1e-6) {
        recovery_turn_sign_ = turn_reference > 0.0 ? 1.0 : -1.0;
        recovery_start_time_ = now;
        recovery_active_ = true;
        resetPid();
      }
    }
  }

  if (recovery_active_) {
    const double recovery_age = std::chrono::duration<double>(now - recovery_start_time_).count();

    if (recovery_age <= recovery_timeout_sec_) {
      diagnostic_target_speed_ = recovery_linear_speed_;
      diagnostic_speed_limit_ = "RECOVERY";
      const double target = recovery_turn_sign_ * recovery_angular_z_;
      const double angular = limitSteeringRate(target, std::max(dt, 1.0 / control_frequency_hz_));
      publishCommand(recovery_linear_speed_, angular);
      publishDebug(0.0, 0.0, 0.0, angular, true);
      publishRecoveryState(true);
      publishStatus("RECOVERY", error_age, diagnostic_raw_dt);
      return;
    }

    recovery_active_ = false;
    publishRecoveryState(false);
  }

  resetPid();
  last_commanded_angular_z_ = 0.0;
  // The next acceleration ramp must start from the stop command we publish.
  last_commanded_linear_x_ = 0.0;
  publishCommand(0.0, 0.0);
  publishDebug(0.0, 0.0, 0.0, 0.0, false);
  publishRecoveryState(false);
  const char * stop_reason = diagnostic_nonfinite_input_ ? "NONFINITE_ERROR" :
    !have_error_ ? "WAITING_ERROR" : !lane_valid_ ? "INVALID_LANE" : "ERROR_RECEIPT_TIMEOUT";
  publishStatus(stop_reason, error_age, diagnostic_raw_dt);
}

}  // namespace wheeltec_autorace_application

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wheeltec_autorace_application::PidLaneController>());
  rclcpp::shutdown();
  return 0;
}
