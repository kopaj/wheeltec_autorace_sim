// SPDX-License-Identifier: Apache-2.0
// Read-only adapter: preserve the Pose_V envelope's simulation stamp.
// Only the named top-level model is valid; link poses are parent-relative.
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class GroundTruthBridge final : public rclcpp::Node
{
public:
  GroundTruthBridge() : Node("evaluation_ground_truth_bridge")
  {
    world_ = declare_parameter<std::string>("world_name", "wheeltec_autorace");
    model_ = declare_parameter<std::string>("model_name", "roboworks");
    const auto topic = "/world/" + world_ + "/dynamic_pose/info";
    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/evaluation/ground_truth_pose", rclcpp::QoS(100).reliable());
    if (!transport_.Subscribe(topic, &GroundTruthBridge::receive, this)) {
      throw std::runtime_error("Cannot subscribe to " + topic);
    }
    RCLCPP_INFO(get_logger(), "Read-only: %s; model=%s; using Pose_V simulation stamps",
      topic.c_str(), model_.c_str());
  }
  ~GroundTruthBridge() override
  {
    transport_.Unsubscribe("/world/" + world_ + "/dynamic_pose/info");
  }
private:
  void receive(const ignition::msgs::Pose_V &msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const ignition::msgs::Pose *selected = nullptr;
    for (const auto &p : msg.pose()) {
      if (p.name() != model_) continue;
      if (selected) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
          "Ambiguous model name; refusing to publish ground truth");
        return;
      }
      selected = &p;
    }
    if (!selected || !msg.has_header() || !msg.header().has_stamp()) return;
    const auto &stamp = msg.header().stamp();
    if (stamp.sec() < 0 || stamp.sec() > 2147483647LL ||
        stamp.nsec() < 0 || stamp.nsec() >= 1000000000) return;
    geometry_msgs::msg::PoseStamped out;
    out.header.frame_id = "world/" + world_;
    out.header.stamp.sec = static_cast<int32_t>(stamp.sec());
    out.header.stamp.nanosec = static_cast<uint32_t>(stamp.nsec());
    out.pose.position.x = selected->position().x();
    out.pose.position.y = selected->position().y();
    out.pose.position.z = selected->position().z();
    out.pose.orientation.x = selected->orientation().x();
    out.pose.orientation.y = selected->orientation().y();
    out.pose.orientation.z = selected->orientation().z();
    out.pose.orientation.w = selected->orientation().w();
    pub_->publish(out);
  }
  std::string world_, model_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  std::mutex mutex_;
  ignition::transport::Node transport_;
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<GroundTruthBridge>());
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("evaluation_ground_truth_bridge"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
