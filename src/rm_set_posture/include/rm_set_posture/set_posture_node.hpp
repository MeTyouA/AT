#ifndef RM_SET_POSTURE__SET_POSTURE_NODE_HPP_
#define RM_SET_POSTURE__SET_POSTURE_NODE_HPP_

#include <chrono>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rm_decision_interfaces/msg/sentry_decision.hpp"
#include "rm_decision_interfaces/srv/set_sentry_posture.hpp"

using SentryDecision = rm_decision_interfaces::msg::SentryDecision;
using SetSentryPosture = rm_decision_interfaces::srv::SetSentryPosture;

enum Posture : uint8_t
{
  POSTURE_ATTACK = 1,
  POSTURE_DEFENSE = 2,
  POSTURE_MOVE = 3,
};

class SetPostureNode : public rclcpp::Node
{
public:
  explicit SetPostureNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onDecision(const SentryDecision::SharedPtr msg);
  void checkPostureTimeout();
  void restorePosture();
  void sendPosture(uint8_t posture, const std::string & reason);

  std::map<std::string, uint8_t> mode_to_posture_;
  uint8_t last_posture_ = 0;
  bool override_mode_;

  // 姿态超时切换
  double posture_timeout_sec_ = 160.0;
  double temp_switch_sec_ = 3.0;
  rclcpp::Time posture_start_time_;
  uint8_t saved_posture_ = 0;
  bool in_temp_switch_ = false;
  rclcpp::TimerBase::SharedPtr posture_check_timer_;
  rclcpp::TimerBase::SharedPtr temp_switch_timer_;

  rclcpp::Client<SetSentryPosture>::SharedPtr client_;
  rclcpp::Subscription<SentryDecision>::SharedPtr sub_;
};

#endif  // RM_SET_POSTURE__SET_POSTURE_NODE_HPP_
