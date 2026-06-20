#include "rm_set_posture/set_posture_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

SetPostureNode::SetPostureNode(const rclcpp::NodeOptions & options)
: Node("set_posture_node", options)
{
  mode_to_posture_ = {
    {"ATTACK",      POSTURE_ATTACK},
    {"RETREAT",     POSTURE_MOVE},
    {"SUPPLY",      POSTURE_MOVE},
    {"PATROL",      POSTURE_MOVE},
    {"DEFEND_BASE", POSTURE_DEFENSE},
    {"IDLE",        POSTURE_DEFENSE},
  };

  declare_parameter("service_name", std::string("/set_sentry_posture"));
  declare_parameter("override_mode", false);
  declare_parameter("posture_timeout_sec", 160.0);
  declare_parameter("temp_switch_sec", 3.0);

  std::string service_name = get_parameter("service_name").as_string();
  override_mode_ = get_parameter("override_mode").as_bool();
  posture_timeout_sec_ = get_parameter("posture_timeout_sec").as_double();
  temp_switch_sec_ = get_parameter("temp_switch_sec").as_double();

  client_ = create_client<SetSentryPosture>(service_name);

  sub_ = create_subscription<SentryDecision>(
    "/sentry_decision", rclcpp::QoS(10),
    [this](const SentryDecision::SharedPtr msg) {
      onDecision(msg);
    });

  posture_start_time_ = now();
  posture_check_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&SetPostureNode::checkPostureTimeout, this));

  RCLCPP_INFO(get_logger(),
    "SetPostureNode 已启动，订阅 /sentry_decision，服务: %s，姿态超时: %.0fs，临时切换: %.1fs",
    service_name.c_str(), posture_timeout_sec_, temp_switch_sec_);
}

void SetPostureNode::sendPosture(uint8_t posture, const std::string & reason)
{
  if (!client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "服务 %s 未就绪，等待中...", client_->get_service_name());
    return;
  }

  auto request = std::make_shared<SetSentryPosture::Request>();
  request->posture = posture;
  request->override_mode = override_mode_;

  RCLCPP_INFO(get_logger(), "姿态切换请求: %s → 姿态 %d", reason.c_str(), static_cast<int>(posture));

  client_->async_send_request(request,
    [this, posture, reason](rclcpp::Client<SetSentryPosture>::SharedFuture future) {
      auto response = future.get();
      if (response->accepted) {
        last_posture_ = posture;
        posture_start_time_ = now();
        RCLCPP_INFO(get_logger(), "姿态切换成功: %s → %d", reason.c_str(), static_cast<int>(posture));
      } else {
        RCLCPP_WARN(get_logger(), "姿态切换被拒绝: %s (原因: %s)",
                    reason.c_str(), response->message.c_str());
      }
    });
}

void SetPostureNode::onDecision(const SentryDecision::SharedPtr msg)
{
  // 如果正在临时切换中，且收到新的姿态请求，取消临时切换
  if (in_temp_switch_) {
    auto it = mode_to_posture_.find(msg->mode);
    if (it != mode_to_posture_.end() && it->second != POSTURE_DEFENSE) {
      RCLCPP_INFO(get_logger(), "临时切换期间收到新决策 '%s'，取消回切定时器", msg->mode.c_str());
      temp_switch_timer_->cancel();
      in_temp_switch_ = false;
    } else {
      return;
    }
  }

  auto it = mode_to_posture_.find(msg->mode);
  if (it == mode_to_posture_.end()) {
    RCLCPP_WARN(get_logger(), "未知模式: '%s'，跳过", msg->mode.c_str());
    return;
  }

  uint8_t posture = it->second;
  if (posture == last_posture_) {
    return;
  }

  sendPosture(posture, "模式 " + msg->mode);
}

void SetPostureNode::checkPostureTimeout()
{
  if (in_temp_switch_ || last_posture_ == 0) {
    return;
  }

  double elapsed = (now() - posture_start_time_).seconds();
  if (elapsed < posture_timeout_sec_) {
    return;
  }

  saved_posture_ = last_posture_;
  in_temp_switch_ = true;

  RCLCPP_INFO(get_logger(),
    "姿态 %d 已持续 %.0f 秒未改变，临时切换到 DEFENSE %.1f 秒",
    static_cast<int>(saved_posture_), elapsed, temp_switch_sec_);

  sendPosture(POSTURE_DEFENSE, "超时临时切换");

  temp_switch_timer_ = create_wall_timer(
    std::chrono::duration<double>(temp_switch_sec_),
    [this]() {
      temp_switch_timer_->cancel();
      restorePosture();
    });
}

void SetPostureNode::restorePosture()
{
  if (!in_temp_switch_) {
    return;
  }

  RCLCPP_INFO(get_logger(),
    "临时切换结束，恢复原姿态 %d", static_cast<int>(saved_posture_));

  in_temp_switch_ = false;
  sendPosture(saved_posture_, "超时恢复原姿态");
}

RCLCPP_COMPONENTS_REGISTER_NODE(SetPostureNode)
