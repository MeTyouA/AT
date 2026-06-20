// Copyright 2025 RoboMaster-A&T
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RMOSS_GZ_BASE__GIMBAL_CONTROLLER_HPP_
#define RMOSS_GZ_BASE__GIMBAL_CONTROLLER_HPP_

#include <memory>
#include <geometry_msgs/msg/twist.hpp>
#include <rmoss_interfaces/msg/gimbal_cmd.hpp>
#include <rmoss_interfaces/msg/chassis_cmd.hpp>
#include <string>

#include "hardware_interface.hpp"
#include "pid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmoss_interfaces/msg/gimbal.hpp"
#include "rmoss_interfaces/msg/gimbal_cmd.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace rmoss_gz_base {

class GimbalController {
public:
    GimbalController(
        rclcpp::Node::SharedPtr node,
        Actuator<rmoss_interfaces::msg::Gimbal>::SharedPtr gimbal_vel_actuator,
        Sensor<rmoss_interfaces::msg::Gimbal>::SharedPtr gimbal_pos_sensor,
        const std::string& controller_name = "gimbal_controller",
        const std::string& yaw_name = "gimbal_yaw_joint",
        const std::string& pitch_name = "gimbal_pitch_joint");
    ~GimbalController() { }

public:
    void set_yaw_pid(struct PidParam pid_param);
    void set_pitch_pid(struct PidParam pid_param);
    // set gimbal's motor limit (TODO)
    // void set_yaw_motor_limit(double min, double max) {}
    // void set_pitch_motor_limit(double min, double max) {}
    void reset();

private:
    void gimbal_cb(const rmoss_interfaces::msg::GimbalCmd::SharedPtr msg);
    void gimbal_joint_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
    void update();
    void gimbal_state_timer_cb();

private:
    rclcpp::Node::SharedPtr node_;
    // ros pub and sub
    // … in gimbal_controller.hpp 添加成员
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr chassis_twist_sub_;
    rclcpp::Subscription<rmoss_interfaces::msg::Gimbal>::SharedPtr gimbal_big_state_sub_;
    rclcpp::Subscription<rmoss_interfaces::msg::GimbalCmd>::SharedPtr rmoss_gimbal_cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr ros_gimbal_cmd_sub_;
    rclcpp::Publisher<rmoss_interfaces::msg::Gimbal>::SharedPtr rmoss_gimbal_state_pub_;
    rclcpp::TimerBase::SharedPtr controller_timer_;
    rclcpp::TimerBase::SharedPtr gimbal_state_timer_;
    // control interface
    Actuator<rmoss_interfaces::msg::Gimbal>::SharedPtr gimbal_vel_actuator_;
    Sensor<rmoss_interfaces::msg::Gimbal>::SharedPtr gimbal_pos_sensor_;
    std::string controller_name_;
    // joint name
    std::string yaw_name_;
    std::string pitch_name_;
    // target data
    double target_pitch_ { 0 };
    double target_yaw_ { 0 };
    double cur_pitch_ { 0 };
    double cur_yaw_ { 0 };
    // pid and pid parameter
    PidParam picth_pid_param_;
    PidParam yaw_pid_param_;
    ignition::math::PID picth_pid_;
    ignition::math::PID yaw_pid_;
    std::chrono::nanoseconds pid_period_;
    // flag
    bool update_pid_flag_ { true };
    // topic list
    std::string cmd_topic_;
    std::string state_topic_;
    std::string joint_topic_;
    // vel
    bool use_velocity_mode_ = false;
    float desired_vel_yaw_ = 0.0f;
    float desired_vel_pitch_ = 0.0f;
    // 初始变量
    bool initialized_ { false };
    bool follow_offset_initialized_ = false;
    double yaw_follow_offset_ = 0.0;
    double pitch_follow_offset_ = 0.0;
};

} // namespace rmoss_gz_base

#endif // RMOSS_GZ_BASE__GIMBAL_CONTROLLER_HPP_
