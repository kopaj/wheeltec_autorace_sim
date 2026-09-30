#include "wheeltec_autorace_application/pid_lane_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

namespace wheeltec_autorace_application
{

PidLaneController::PidLaneController()
: Node("pid_lane_controller")
{
  error_topic_ = declare_parameter<std::string>(
    "error_topic", "/lane_detection/control_error_normalized");
  valid_topic_ = declare_parameter<std::string>(
    "valid_topic", "/lane_detection/valid");
  degraded_topic_ = declare_parameter<std::string>(
    "degraded_topic", "/lane_detection/degraded");
  cmd_vel_topic_ = declare_parameter<std::string>(
    "cmd_vel_topic", "/cmd_vel");

  p_term_topic_ = declare_parameter<std::string>(
    "p_term_topic", "/control/pid/p_term");
  i_term_topic_ = declare_parameter<std::string>(
    "i_term_topic", "/control/pid/i_term");
  d_term_topic_ = declare_parameter<std::string>(
    "d_term_topic", "/control/pid/d_term");
  output_topic_ = declare_parameter<std::string>(
    "output_topic", "/control/pid/output");
  active_topic_ = declare_parameter<std::string>(
    "active_topic", "/control/pid/active");
  recovery_active_topic_ = declare_parameter<std::string>(
    "recovery_active_topic", "/control/pid/recovery_active");

  kp_ = declare_parameter<double>("kp", 1.25);
  ki_ = declare_parameter<double>("ki", 0.0);
  kd_ = declare_parameter<double>("kd", 0.18);
  integral_limit_ = declare_parameter<double>("integral_limit", 0.40);
  derivative_filter_alpha_ = declare_parameter<double>("derivative_filter_alpha", 0.15);
  error_filter_alpha_ = declare_parameter<double>("error_filter_alpha", 0.30);
  error_deadband_ = declare_parameter<double>("error_deadband", 0.018);
  normal_error_limit_ = declare_parameter<double>("normal_error_limit", 0.45);
  degraded_error_limit_ = declare_parameter<double>("degraded_error_limit", 0.30);
  degraded_kp_scale_ = declare_parameter<double>("degraded_kp_scale", 0.70);

  max_angular_z_ = declare_parameter<double>("max_angular_z", 0.70);
  degraded_max_angular_z_ = declare_parameter<double>("degraded_max_angular_z", 0.45);
  max_linear_speed_ = declare_parameter<double>("max_linear_speed", 0.09);
  min_linear_speed_ = declare_parameter<double>("min_linear_speed", 0.035);
  degraded_linear_speed_ = declare_parameter<double>("degraded_linear_speed", 0.03);
  steering_sign_ = declare_parameter<double>("steering_sign", -1.0);

  steering_rise_rate_ = declare_parameter<double>("steering_rise_rate", 2.8);
  steering_center_rate_ = declare_parameter<double>("steering_center_rate", 4.0);
  steering_reverse_rate_ = declare_parameter<double>("steering_reverse_rate", 1.2);

  // Recovery is intentionally OFF by default in this tuning step. The
  // previous fixed-angle recovery could keep turning after the perception
  // had already started to reacquire the track and amplified overshoot.
  recovery_enabled_ = declare_parameter<bool>("recovery_enabled", false);
  recovery_timeout_sec_ = declare_parameter<double>("recovery_timeout_sec", 0.60);
  recovery_entry_grace_sec_ = declare_parameter<double>("recovery_entry_grace_sec", 0.25);
  recovery_entry_error_ = declare_parameter<double>("recovery_entry_error", 0.10);
  recovery_entry_angular_z_ = declare_parameter<double>("recovery_entry_angular_z", 0.20);
  recovery_linear_speed_ = declare_parameter<double>("recovery_linear_speed", 0.025);
  recovery_angular_z_ = declare_parameter<double>("recovery_angular_z", 0.35);

  control_frequency_hz_ = declare_parameter<double>("control_frequency_hz", 20.0);
  perception_timeout_sec_ = declare_parameter<double>("perception_timeout_sec", 0.25);
  enabled_ = declare_parameter<bool>("enabled_on_start", false);

  control_frequency_hz_ = std::max(control_frequency_hz_, 1.0);
  derivative_filter_alpha_ = std::clamp(derivative_filter_alpha_, 0.0, 1.0);
  error_filter_alpha_ = std::clamp(error_filter_alpha_, 0.0, 1.0);
  error_deadband_ = std::max(error_deadband_, 0.0);
  normal_error_limit_ = std::clamp(std::abs(normal_error_limit_), 0.01, 1.0);
  degraded_error_limit_ = std::clamp(std::abs(degraded_error_limit_), 0.01, normal_error_limit_);
  degraded_kp_scale_ = std::clamp(degraded_kp_scale_, 0.0, 1.0);

  max_angular_z_ = std::max(std::abs(max_angular_z_), 1e-6);
  degraded_max_angular_z_ =
    std::clamp(std::abs(degraded_max_angular_z_), 1e-6, max_angular_z_);
  max_linear_speed_ = std::max(max_linear_speed_, 0.0);
  min_linear_speed_ = std::clamp(min_linear_speed_, 0.0, max_linear_speed_);
  degraded_linear_speed_ = std::clamp(degraded_linear_speed_, 0.0, max_linear_speed_);

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
    error_topic_, 10,
    std::bind(&PidLaneController::errorCallback, this, std::placeholders::_1));

  valid_sub_ = create_subscription<std_msgs::msg::Bool>(
    valid_topic_, 10,
    std::bind(&PidLaneController::validCallback, this, std::placeholders::_1));

  degraded_sub_ = create_subscription<std_msgs::msg::Bool>(
    degraded_topic_, 10,
    std::bind(&PidLaneController::degradedCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  p_term_pub_ = create_publisher<std_msgs::msg::Float64>(p_term_topic_, 10);
  i_term_pub_ = create_publisher<std_msgs::msg::Float64>(i_term_topic_, 10);
  d_term_pub_ = create_publisher<std_msgs::msg::Float64>(d_term_topic_, 10);
  output_pub_ = create_publisher<std_msgs::msg::Float64>(output_topic_, 10);
  active_pub_ = create_publisher<std_msgs::msg::Bool>(active_topic_, 10);
  recovery_active_pub_ = create_publisher<std_msgs::msg::Bool>(recovery_active_topic_, 10);

  enable_service_ = create_service<std_srvs::srv::SetBool>(
    "/pid_lane_controller/enable",
    std::bind(
      &PidLaneController::enableCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  const int period_ms = std::max(
    1,
    static_cast<int>(std::round(1000.0 / control_frequency_hz_)));

  const auto now = SteadyClock::now();
  last_control_time_ = now;
  last_valid_tracking_time_ = now;
  recovery_start_time_ = now;

  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&PidLaneController::controlLoop, this));

  RCLCPP_INFO(
    get_logger(),
    "PID controller started %s. Kp=%.2f Kd=%.2f max_w=%.2f recovery=%s",
    enabled_ ? "ENABLED" : "DISABLED",
    kp_, kd_, max_angular_z_, recovery_enabled_ ? "ON" : "OFF");
}

void PidLaneController::errorCallback(
  const std_msgs::msg::Float64::ConstSharedPtr msg)
{
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

void PidLaneController::enableCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  enabled_ = request->data;
  resetPid();
  recovery_active_ = false;
  last_commanded_angular_z_ = 0.0;
  previous_lane_degraded_ = false;
  publishRecoveryState(false);

  if (enabled_) {
    have_error_ = false;
    lane_valid_ = false;
    lane_degraded_ = false;
    have_tracking_history_ = false;

    response->success = true;
    response->message = "PID lane controller enabled";
    RCLCPP_INFO(get_logger(), "PID controller ENABLED");
  } else {
    publishCommand(0.0, 0.0);
    response->success = true;
    response->message = "PID lane controller disabled and vehicle stopped";
    RCLCPP_INFO(get_logger(), "PID controller DISABLED");
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
}

void PidLaneController::publishCommand(
  const double linear_x,
  const double angular_z)
{
  geometry_msgs::msg::Twist command;
  command.linear.x = linear_x;
  command.angular.z = angular_z;
  cmd_vel_pub_->publish(command);
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

double PidLaneController::limitSteeringRate(
  const double target,
  const double dt)
{
  const double current = last_commanded_angular_z_;

  double rate = steering_rise_rate_;

  const bool same_sign = target * current >= 0.0;
  const bool moving_toward_zero =
    same_sign && std::abs(target) < std::abs(current);

  if (moving_toward_zero) {
    rate = steering_center_rate_;
  } else if (!same_sign && std::abs(current) > 1e-4) {
    // First unwind the old steering direction. Crossing immediately to a
    // large opposite command is the main source of post-corner snake motion.
    const double max_delta = steering_center_rate_ * dt;
    const double next =
      current > 0.0 ? std::max(0.0, current - max_delta) : std::min(0.0, current + max_delta);
    last_commanded_angular_z_ = next;
    return next;
  } else if (!same_sign) {
    rate = steering_reverse_rate_;
  }

  const double max_delta = rate * dt;
  const double next = std::clamp(target, current - max_delta, current + max_delta);
  last_commanded_angular_z_ = next;
  return next;
}

void PidLaneController::controlLoop()
{
  const auto now = SteadyClock::now();

  double dt = std::chrono::duration<double>(now - last_control_time_).count();
  last_control_time_ = now;

  if (!enabled_) {
    publishDebug(0.0, 0.0, 0.0, 0.0, false);
    publishRecoveryState(false);
    return;
  }

  bool perception_stale = !have_error_;
  if (have_error_) {
    const double error_age =
      std::chrono::duration<double>(now - last_error_time_).count();
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

    // Smooth the image-space control error before the PID sees it. This is
    // deliberately mild: enough to suppress one-frame jumps, but still fast
    // enough for the tight first corner.
    if (!have_filtered_error_) {
      filtered_error_ = latest_error_;
      have_filtered_error_ = true;
    } else {
      filtered_error_ =
        error_filter_alpha_ * latest_error_ +
        (1.0 - error_filter_alpha_) * filtered_error_;
    }

    // Changing FULL <-> DEGRADED can cause a large derivative kick because
    // an estimated boundary moves the centerline slightly. Reset only the D
    // memory on that transition; keep the filtered steering objective.
    if (lane_degraded_ != previous_lane_degraded_) {
      filtered_derivative_ = 0.0;
      have_previous_error_ = false;
      previous_lane_degraded_ = lane_degraded_;
    }

    const double error_limit =
      lane_degraded_ ? degraded_error_limit_ : normal_error_limit_;

    double error = std::clamp(filtered_error_, -error_limit, error_limit);
    if (std::abs(error) < error_deadband_) {
      error = 0.0;
    }

    const double effective_kp =
      lane_degraded_ ? kp_ * degraded_kp_scale_ : kp_;

    const double p_term = effective_kp * error;

    integral_ += error * dt;
    integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
    const double i_term = ki_ * integral_;

    double raw_derivative = 0.0;
    if (have_previous_error_) {
      raw_derivative = (error - previous_error_) / dt;
    }

    filtered_derivative_ =
      derivative_filter_alpha_ * raw_derivative +
      (1.0 - derivative_filter_alpha_) * filtered_derivative_;

    const double d_term = kd_ * filtered_derivative_;

    previous_error_ = error;
    have_previous_error_ = true;

    const double active_max_angular =
      lane_degraded_ ? degraded_max_angular_z_ : max_angular_z_;

    const double raw_output = p_term + i_term + d_term;
    const double limited_output =
      std::clamp(raw_output, -active_max_angular, active_max_angular);

    const double target_angular = steering_sign_ * limited_output;
    const double angular_command = limitSteeringRate(target_angular, dt);

    const double steering_ratio =
      std::clamp(std::abs(angular_command) / max_angular_z_, 0.0, 1.0);

    double commanded_speed =
      max_linear_speed_ -
      steering_ratio * (max_linear_speed_ - min_linear_speed_);

    if (lane_degraded_) {
      commanded_speed = std::min(commanded_speed, degraded_linear_speed_);
    }

    publishCommand(commanded_speed, angular_command);
    publishDebug(p_term, i_term, d_term, angular_command, true);
    publishRecoveryState(false);

    last_valid_error_ = error;
    last_tracking_angular_z_ = angular_command;
    last_valid_tracking_time_ = now;
    have_tracking_history_ = true;
    return;
  }

  // Optional bounded recovery. It is disabled in the default YAML while the
  // closed-loop controller is being tuned; keeping the code here lets us
  // re-enable it later in a controlled experiment.
  const double since_last_valid =
    std::chrono::duration<double>(now - last_valid_tracking_time_).count();

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
    const double recovery_age =
      std::chrono::duration<double>(now - recovery_start_time_).count();

    if (recovery_age <= recovery_timeout_sec_) {
      const double target = recovery_turn_sign_ * recovery_angular_z_;
      const double angular = limitSteeringRate(target, std::max(dt, 1.0 / control_frequency_hz_));
      publishCommand(recovery_linear_speed_, angular);
      publishDebug(0.0, 0.0, 0.0, angular, true);
      publishRecoveryState(true);
      return;
    }

    recovery_active_ = false;
    publishRecoveryState(false);
  }

  resetPid();
  last_commanded_angular_z_ = 0.0;
  publishCommand(0.0, 0.0);
  publishDebug(0.0, 0.0, 0.0, 0.0, false);
  publishRecoveryState(false);
}

}  // namespace wheeltec_autorace_application

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<wheeltec_autorace_application::PidLaneController>());
  rclcpp::shutdown();
  return 0;
}
