#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "wheeltec_autorace_application/pure_pursuit_core.hpp"

namespace wheeltec_autorace_application
{

class PurePursuitController final : public rclcpp::Node
{
public:
  PurePursuitController();
private:
  using Clock = std::chrono::steady_clock;
  using ScalarPub = rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr;
  void controlLoop();
  void stop(const std::string & reason);
  void publishStatus(const std::string & reason);
  void publish(double speed, double omega, const pp::Solution & solution, bool active);
  void enableCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  bool fresh(Clock::time_point time) const;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr valid_sub_, degraded_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr preview_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  ScalarPub alpha_pub_, curvature_pub_, steering_pub_, speed_pub_, omega_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::string target_problem_{"WAITING_FOR_TARGET"};
  std::string last_status_code_;
  double last_loop_dt_{0.0};
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  double wheelbase_, steering_limit_, vmax_, preview_speed_, degraded_speed_;
  double a_lat_, acceleration_, deceleration_, timeout_, frequency_;
  double preview_activation_score_;
  bool enabled_{false}, target_ok_{false}, valid_{false}, degraded_{true};
  bool have_valid_{false}, have_degraded_{false}, preview_ok_{false};
  pp::Point target_;
  double preview_{1.0}, speed_{0.0};
  Clock::time_point target_time_{}, valid_time_{}, degraded_time_{}, preview_time_{};
  rclcpp::Time target_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time loop_stamp_{0, 0, RCL_ROS_TIME};
};

}  // namespace wheeltec_autorace_application
