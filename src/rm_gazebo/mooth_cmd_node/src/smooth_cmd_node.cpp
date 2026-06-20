/**
 * @file smooth_cmd_node.cpp
 * @brief 从 /cmd_vel_chassis 读取速度，平滑后输出到 /cmd_vel
 *
 * 参数：
 *  - smooth_mode (string): "average" 或 "exponential"
 *  - window_size (int): 滑动平均窗口大小
 *  - alpha (double): 指数加权系数 (0~1)
 *  - publish_rate (int): 输出频率（Hz）
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <deque>
#include <string>

class SmoothCmdNode : public rclcpp::Node
{
public:
  SmoothCmdNode() : Node("smooth_cmd_node")
  {
    // --- 参数声明 ---
    this->declare_parameter<std::string>("smooth_mode", "average");
    this->declare_parameter<int>("window_size", 5);
    this->declare_parameter<double>("alpha", 0.7);
    this->declare_parameter<int>("publish_rate", 20);

    // --- 参数读取 ---
    smooth_mode_ = this->get_parameter("smooth_mode").as_string();
    window_size_ = this->get_parameter("window_size").as_int();
    alpha_ = this->get_parameter("alpha").as_double();
    publish_rate_ = this->get_parameter("publish_rate").as_int();

    // --- 订阅与发布 ---
    sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_chassis", 10,
      std::bind(&SmoothCmdNode::cmdCallback, this, std::placeholders::_1));

    pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // --- 定时发布 ---
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / publish_rate_),
      std::bind(&SmoothCmdNode::publishSmoothedVel, this));

    RCLCPP_INFO(this->get_logger(),
                "SmoothCmdNode started: mode=%s, window=%d, alpha=%.2f, rate=%dHz",
                smooth_mode_.c_str(), window_size_, alpha_, publish_rate_);
  }

private:
  // 平滑函数
  geometry_msgs::msg::Twist smooth(const geometry_msgs::msg::Twist &new_vel)
  {
    geometry_msgs::msg::Twist result;

    if (smooth_mode_ == "average")
    {
      // 滑动平均
      buffer_.push_back(new_vel);
      if ((int)buffer_.size() > window_size_)
        buffer_.pop_front();

      result.linear.x = 0.0;
      result.linear.y = 0.0;
      result.angular.z = 0.0;
      for (const auto &v : buffer_)
      {
        result.linear.x += v.linear.x;
        result.linear.y += v.linear.y;
        result.angular.z += v.angular.z;
      }
      result.linear.x /= buffer_.size();
      result.linear.y /= buffer_.size();
      result.angular.z /= buffer_.size();
    }
    else if (smooth_mode_ == "exponential")
    {
      // 指数加权
      smoothed_.linear.x = alpha_ * new_vel.linear.x + (1 - alpha_) * smoothed_.linear.x;
      smoothed_.linear.y = alpha_ * new_vel.linear.y + (1 - alpha_) * smoothed_.linear.y;
      smoothed_.angular.z = alpha_ * new_vel.angular.z + (1 - alpha_) * smoothed_.angular.z;
      result = smoothed_;
    }
    else
    {
      // 无滤波
      result = new_vel;
    }

    return result;
  }

  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_input_ = *msg;
    new_data_received_ = true;
  }

  void publishSmoothedVel()
  {
    if (!new_data_received_)
      return;

    auto smoothed = smooth(latest_input_);
    pub_cmd_->publish(smoothed);
  }

  // --- 成员变量 ---
  std::string smooth_mode_;
  int window_size_;
  double alpha_;
  int publish_rate_;

  std::deque<geometry_msgs::msg::Twist> buffer_;
  geometry_msgs::msg::Twist smoothed_{};
  geometry_msgs::msg::Twist latest_input_{};
  bool new_data_received_ = false;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SmoothCmdNode>());
  rclcpp::shutdown();
  return 0;
}

