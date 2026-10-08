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

class PurePursuitController final : public rclcpp::Node
{
public:
  PurePursuitController();

private:
  using SteadyClock = std::chrono::steady_clock;

  void errorCallback(const std_msgs::msg::Float64::ConstSharedPtr msg);
  void validCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void degradedCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void turnPreviewCallback(const std_msgs::msg::Float64::ConstSharedPtr msg);

  void enableCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  void controlLoop();

  void resetController();
  void publishCommand(double linear_x, double angular_z);
  void publishDebug(
    double lateral_target_m,
    double alpha_rad,
    double curvature,
    double steering_angle_rad,
    double angular_command,
    bool active);

  double limitAngularRate(double target, double dt);
  double limitLinearSpeed(double target, double dt);

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr error_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr valid_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr degraded_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr turn_preview_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr current_speed_pub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lateral_target_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr alpha_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr curvature_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_angle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr output_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::string error_topic_;
  std::string valid_topic_;
  std::string degraded_topic_;
  std::string turn_preview_topic_;
  std::string cmd_vel_topic_;
  std::string current_speed_topic_;

  std::string lateral_target_topic_;
  std::string alpha_topic_;
  std::string curvature_topic_;
  std::string steering_angle_topic_;
  std::string output_topic_;
  std::string active_topic_;

  double wheel_base_m_;
  double lookahead_distance_m_;
  double straight_lookahead_distance_m_;
  double normalized_lateral_scale_m_;
  double max_steering_angle_rad_;
  double steering_sign_;

  double error_filter_alpha_;
  double error_deadband_;
  double normal_error_limit_;
  double degraded_error_limit_;
  double perception_timeout_sec_;
  double control_frequency_hz_;

  double max_linear_speed_;
  double min_linear_speed_;
  double degraded_linear_speed_;
  double max_lateral_accel_mps2_;
  double max_acceleration_;
  double max_deceleration_;

  double angular_rate_limit_;

  bool turn_preview_enabled_;
  double turn_preview_timeout_sec_;
  double turn_preview_min_speed_;

  bool enabled_{false};
  bool lane_valid_{false};
  bool lane_degraded_{false};
  bool have_error_{false};
  bool have_filtered_error_{false};
  bool have_turn_preview_{false};

  double latest_error_{0.0};
  double filtered_error_{0.0};
  double latest_turn_preview_{0.0};
  double last_commanded_angular_z_{0.0};
  double last_commanded_linear_x_{0.0};

  SteadyClock::time_point last_error_time_;
  SteadyClock::time_point last_turn_preview_time_;
  SteadyClock::time_point last_control_time_;
};

}  // namespace wheeltec_autorace_application
