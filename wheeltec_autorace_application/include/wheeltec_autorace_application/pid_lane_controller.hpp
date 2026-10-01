#pragma once

#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace wheeltec_autorace_application
{

class PidLaneController final : public rclcpp::Node
{
public:
  PidLaneController();

private:
  using SteadyClock = std::chrono::steady_clock;

  void errorCallback(const std_msgs::msg::Float64::ConstSharedPtr msg);
  void validCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void degradedCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void controlLoop();

  void enableCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  void resetPid();
  void publishCommand(double linear_x, double angular_z);
  void publishDebug(
    double p_term,
    double i_term,
    double d_term,
    double control_output,
    bool active);
  void publishRecoveryState(bool active);

  double limitSteeringRate(double target, double dt);

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr error_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr valid_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr degraded_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr p_term_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr i_term_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr d_term_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr output_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recovery_active_pub_;

  // Service and timer
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Topic parameters
  std::string error_topic_;
  std::string valid_topic_;
  std::string degraded_topic_;
  std::string cmd_vel_topic_;
  std::string p_term_topic_;
  std::string i_term_topic_;
  std::string d_term_topic_;
  std::string output_topic_;
  std::string active_topic_;
  std::string recovery_active_topic_;

  // PID parameters
  double kp_;
  double ki_;
  double kd_;
  double integral_limit_;
  double derivative_filter_alpha_;
  double error_filter_alpha_;
  double error_deadband_;
  double normal_error_limit_;
  double degraded_error_limit_;
  double degraded_kp_scale_;

  // Vehicle command parameters
  double max_angular_z_;
  double degraded_max_angular_z_;
  double max_linear_speed_;
  double min_linear_speed_;
  double degraded_linear_speed_;
  double steering_sign_;

  // Steering-rate limiting.
  // Build steering into a curve, return quickly toward zero, but cross zero
  // more slowly so a single noisy frame cannot start a snake motion.
  double steering_rise_rate_;
  double steering_center_rate_;
  double steering_reverse_rate_;

  // Recovery state parameters
  bool recovery_enabled_;
  double recovery_timeout_sec_;
  double recovery_entry_grace_sec_;
  double recovery_entry_error_;
  double recovery_entry_angular_z_;
  double recovery_linear_speed_;
  double recovery_angular_z_;

  double control_frequency_hz_;
  double perception_timeout_sec_;

  // Controller state
  bool enabled_{false};
  bool lane_valid_{false};
  bool lane_degraded_{false};
  bool previous_lane_degraded_{false};
  bool have_error_{false};
  bool have_filtered_error_{false};
  bool have_previous_error_{false};
  bool have_tracking_history_{false};
  bool recovery_active_{false};

  double latest_error_{0.0};
  double filtered_error_{0.0};
  double previous_error_{0.0};
  double integral_{0.0};
  double filtered_derivative_{0.0};
  double last_commanded_angular_z_{0.0};
  double last_valid_error_{0.0};
  double last_tracking_angular_z_{0.0};
  double recovery_turn_sign_{0.0};
  double last_commanded_linear_x_{0.0};

  SteadyClock::time_point last_error_time_;
  SteadyClock::time_point last_control_time_;
  SteadyClock::time_point last_valid_tracking_time_;
  SteadyClock::time_point recovery_start_time_;
};

}  // namespace wheeltec_autorace_application
