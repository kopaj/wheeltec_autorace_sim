#include "wheeltec_autorace_application/pure_pursuit_controller.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <iomanip>
#include <sstream>

namespace wheeltec_autorace_application
{

PurePursuitController::PurePursuitController() : Node("pure_pursuit_controller")
{
  wheelbase_ = declare_parameter<double>("wheel_base_m", 0.3187);
  steering_limit_ = declare_parameter<double>("steering_limit_rad", 0.4);
  vmax_ = declare_parameter<double>("max_linear_speed", 0.5);
  preview_speed_ = declare_parameter<double>("preview_speed", 0.18);
  degraded_speed_ = declare_parameter<double>("degraded_speed", 0.12);
  a_lat_ = declare_parameter<double>("max_lateral_acceleration", 0.20);
  acceleration_ = declare_parameter<double>("max_acceleration", 0.15);
  deceleration_ = declare_parameter<double>("max_deceleration", 1.0);
  timeout_ = declare_parameter<double>("perception_timeout_sec", 0.20);
  frequency_ = declare_parameter<double>("control_frequency_hz", 40.0);
  preview_activation_score_ = declare_parameter<double>("preview_activation_score", 0.75);
  enabled_ = declare_parameter<bool>("enabled_on_start", false);
  for (double value : {wheelbase_, steering_limit_, vmax_, preview_speed_, degraded_speed_,
    a_lat_, acceleration_, deceleration_, timeout_, frequency_})
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("Pure Pursuit parameters must be finite and positive");
    }
  }
  if (!std::isfinite(preview_activation_score_) || preview_activation_score_ < 0.0 ||
    preview_activation_score_ >= 1.0 || steering_limit_ > 0.4 || preview_speed_ > vmax_ || degraded_speed_ > vmax_ ||
    acceleration_ >= deceleration_ || frequency_ < 1.0 || frequency_ > 500.0)
  {
    throw std::invalid_argument("Invalid speed envelope, steering limit or frequency");
  }

  // Fixed, documented interfaces can be changed with ROS remappings.
  const auto qos = rclcpp::QoS(1).reliable();
  // Use the latest perception sample with a separate watchdog for each input.
  const auto perception_qos = rclcpp::SensorDataQoS().keep_last(1);
  command_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos);
  auto scalar = [this, &qos](const std::string & name) {
      return create_publisher<std_msgs::msg::Float64>("/control/pure_pursuit/" + name, qos);
    };
  alpha_pub_ = scalar("alpha");
  curvature_pub_ = scalar("curvature");
  steering_pub_ = scalar("steering_angle");
  speed_pub_ = scalar("current_speed");  // Command, not measured odometry.
  omega_pub_ = scalar("output");
  active_pub_ = create_publisher<std_msgs::msg::Bool>("/control/pure_pursuit/active", qos);

  status_pub_ = create_publisher<std_msgs::msg::String>("/control/pure_pursuit/status", qos);

  target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
    "/lane_detection/target", perception_qos,
    [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
      const rclcpp::Time stamp(msg->header.stamp);
      const double age = (now() - stamp).seconds();
      const bool advancing = stamp > target_stamp_;
      // A reset/replayed/duplicate frame cannot refresh the watchdog.
      if (!advancing) {
        target_ok_ = false;
        target_problem_ = "TARGET_TIMESTAMP_NOT_ADVANCING";
        if (enabled_) { stop(target_problem_); }
      }
      if (stamp < target_stamp_) { target_stamp_ = stamp; }
      if (!advancing) { return; }
      target_stamp_ = stamp;
      target_time_ = Clock::now();
      target_ = {msg->point.x, msg->point.y};
      target_ok_ = msg->header.frame_id == "base_link" && stamp.nanoseconds() > 0 &&
        age >= 0.0 && age <= timeout_ && std::isfinite(target_.x) &&
        std::isfinite(target_.y) && target_.x > 0.0 &&
        std::isfinite(msg->point.z) && std::abs(msg->point.z) < 1e-9 &&
        std::isfinite(std::hypot(target_.x, target_.y)) && std::hypot(target_.x, target_.y) >= 0.05;
      // Classify invalid targets for diagnostics.
      target_problem_ = "TARGET_INVALID_GEOMETRY";
      if (msg->header.frame_id != "base_link") { target_problem_ = "TARGET_WRONG_FRAME"; }
      else if (stamp.nanoseconds() <= 0) { target_problem_ = "TARGET_ZERO_TIMESTAMP"; }
      else if (age < 0.0) {
        target_problem_ = "TARGET_IN_FUTURE age_sec=" + std::to_string(age);
      } else if (age > timeout_) {
        target_problem_ = "TARGET_TOO_OLD age_sec=" + std::to_string(age);
      } else if (!std::isfinite(target_.x) || !std::isfinite(target_.y)) {
        target_problem_ = "TARGET_NONFINITE";
      }
      if (!target_ok_ && enabled_) { stop(target_problem_); }
    });
  valid_sub_ = create_subscription<std_msgs::msg::Bool>("/lane_detection/valid", perception_qos,
    [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
      valid_ = msg->data; have_valid_ = true; valid_time_ = Clock::now();
      if (!valid_ && enabled_) { stop("LANE_INVALID"); }
    });
  degraded_sub_ = create_subscription<std_msgs::msg::Bool>("/lane_detection/target_degraded", perception_qos,
    [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
      degraded_ = msg->data; have_degraded_ = true; degraded_time_ = Clock::now();
    });
  preview_sub_ = create_subscription<std_msgs::msg::Float64>("/lane_detection/turn_preview", perception_qos,
    [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
      preview_ok_ = std::isfinite(msg->data) && msg->data >= 0.0 && msg->data <= 1.0;
      preview_ = preview_ok_ ? msg->data : 1.0; preview_time_ = Clock::now();
    });
  enable_service_ = create_service<std_srvs::srv::SetBool>("/pure_pursuit_controller/enable",
    std::bind(&PurePursuitController::enableCallback, this, std::placeholders::_1, std::placeholders::_2));
  loop_stamp_ = now();
  timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / frequency_)), std::bind(&PurePursuitController::controlLoop, this));
  stop(enabled_ ? "WAITING_FOR_TARGET" : "DISABLED");
  RCLCPP_INFO(get_logger(),
    "PP corner-support build: perception_qos=BEST_EFFORT depth=1, use_sim_time=%s, timeout=%.6f s, frequency=%.3f Hz, "
    "vmax=%.3f, preview_speed=%.3f, degraded_speed=%.3f, acceleration=%.3f, deceleration=%.3f, preview_activation=%.3f",
    get_parameter("use_sim_time").as_bool() ? "true" : "false",
    timeout_, frequency_, vmax_, preview_speed_, degraded_speed_, acceleration_, deceleration_,
    preview_activation_score_);
}

bool PurePursuitController::fresh(const Clock::time_point time) const
{
  return std::chrono::duration<double>(Clock::now() - time).count() <= timeout_;
}

void PurePursuitController::enableCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  enabled_ = request->data;
  target_ok_ = false; have_valid_ = false; have_degraded_ = false; preview_ok_ = false;
  loop_stamp_ = now();
  target_problem_ = "WAITING_FOR_TARGET";
  stop(enabled_ ? "WAITING_FOR_TARGET" : "DISABLED");
  response->success = true;
  response->message = enabled_ ? "Enabled; waiting for fresh perception" : "Disabled; zero Twist sent";
}

void PurePursuitController::publish(const double speed, const double omega,
  const pp::Solution & solution, const bool active)
{
  geometry_msgs::msg::Twist command;
  command.linear.x = speed;
  command.angular.z = omega;
  command_pub_->publish(command);
  auto scalar = [](const ScalarPub & publisher, const double value) {
      std_msgs::msg::Float64 msg; msg.data = value; publisher->publish(msg);
    };
  scalar(alpha_pub_, solution.alpha);
  scalar(curvature_pub_, solution.curvature);
  scalar(steering_pub_, solution.steering);
  scalar(speed_pub_, speed);
  scalar(omega_pub_, omega);
  std_msgs::msg::Bool state; state.data = active; active_pub_->publish(state);
  if (active) {
    std::ostringstream detail;
    detail << std::fixed << std::setprecision(4)
           << (degraded_ ? "RUNNING_DEGRADED" : "RUNNING_NORMAL")
           << " target_distance_m=" << std::hypot(target_.x, target_.y)
           << " requested_curvature=" << solution.requested_curvature
           << " curvature=" << solution.curvature
           << " saturated=" << (std::abs(solution.requested_curvature - solution.curvature) > 1e-9)
           << " speed=" << speed;
    publishStatus(detail.str());
  }
}

void PurePursuitController::publishStatus(const std::string & reason)
{
  std_msgs::msg::String message;
  message.data = reason;
  status_pub_->publish(message);
  // Log state transitions as well: a short stop may be replaced by RUNNING
  // before a depth-one diagnostic subscriber observes its topic message.
  const auto code = reason.substr(0, reason.find(' '));
  if (code != last_status_code_) {
    RCLCPP_INFO(get_logger(), "PP status: %s", reason.c_str());
    last_status_code_ = code;
  }
}

void PurePursuitController::stop(const std::string & reason)
{
  speed_ = 0.0;
  publish(0.0, 0.0, {0.0, 0.0, 0.0, 0.0}, false);
  const auto steady_now = Clock::now();
  auto age = [steady_now](Clock::time_point time) {
      return time == Clock::time_point{} ? -1.0 :
        std::chrono::duration<double>(steady_now - time).count();
    };
  std::ostringstream detail;
  detail << std::fixed << std::setprecision(6) << reason
         << " source_age_s=" << (now() - target_stamp_).seconds()
         << " target_rx_age_s=" << age(target_time_)
         << " valid_rx_age_s=" << age(valid_time_)
         << " degraded_rx_age_s=" << age(degraded_time_)
         << " control_dt_s=" << last_loop_dt_
         << " timeout_s=" << timeout_ << " target_x=" << target_.x << " target_y=" << target_.y;
  publishStatus(detail.str());
}

void PurePursuitController::controlLoop()
{
  const auto current = now();
  const double elapsed = (current - loop_stamp_).seconds();
  last_loop_dt_ = elapsed;
  loop_stamp_ = current;
  // Wall-clock watchdog still runs if simulation time stops. All essential
  // input streams need their own freshness checks, not only the target.
  const double image_age = (current - target_stamp_).seconds();
  // Report the first failing input check.
  if (!enabled_) { stop("DISABLED"); return; }
  if (!target_ok_) { stop(target_problem_); return; }
  if (!have_valid_) { stop("WAITING_FOR_VALID"); return; }
  if (!valid_) { stop("LANE_INVALID"); return; }
  if (!have_degraded_) { stop("WAITING_FOR_DEGRADED"); return; }
  if (!fresh(target_time_)) { stop("TARGET_RECEIPT_TIMEOUT"); return; }
  if (!fresh(valid_time_)) { stop("VALID_RECEIPT_TIMEOUT"); return; }
  if (!fresh(degraded_time_)) { stop("DEGRADED_RECEIPT_TIMEOUT"); return; }
  if (image_age < 0.0) {
    stop("TARGET_IN_FUTURE age_sec=" + std::to_string(image_age)); return;
  }
  if (image_age > timeout_) {
    stop("TARGET_TOO_OLD age_sec=" + std::to_string(image_age)); return;
  }
  if (elapsed < 0.0) { stop("CLOCK_MOVED_BACKWARD"); return; }
  if (elapsed > timeout_) { stop("CONTROL_TIME_GAP"); return; }
  if (elapsed == 0.0) { publishStatus("WAITING_FOR_CLOCK_ADVANCE"); return; }
  const double dt = std::min(elapsed, 2.0 / frequency_);
  const auto solution = pp::solve(target_, wheelbase_, steering_limit_);
  // Missing/invalid preview is conservative, never permission to accelerate.
  const double risk = preview_ok_ && fresh(preview_time_) ?
    pp::previewRisk(preview_, preview_activation_score_) : 1.0;
  const double desired = pp::speedLimit(solution.curvature, risk, degraded_,
    vmax_, preview_speed_, degraded_speed_, a_lat_);
  speed_ = pp::rateLimit(speed_, desired, dt, acceleration_, deceleration_);
  // Twist angular.z is yaw rate [rad/s], NOT steering angle [rad].
  // No independent yaw-rate slew limiter: omega/v must remain curvature.
  publish(speed_, speed_ * solution.curvature, solution, true);
}
}  // namespace wheeltec_autorace_application

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wheeltec_autorace_application::PurePursuitController>());
  rclcpp::shutdown();
  return 0;
}
