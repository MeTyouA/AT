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

#include "rmoss_gz_base_lu_version/gimbal_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>

namespace rmoss_gz_base {

GimbalController::GimbalController(
    rclcpp::Node::SharedPtr node,
    Actuator<rmoss_interfaces::msg::Gimbal>::SharedPtr gimbal_vel_actuator,
    Sensor<rmoss_interfaces::msg::Gimbal>::SharedPtr gimbal_pos_sensor,
    const std::string& controller_name,
    const std::string& yaw_name,
    const std::string& pitch_name)
    : node_(node)
    , gimbal_vel_actuator_(gimbal_vel_actuator)
    , gimbal_pos_sensor_(gimbal_pos_sensor)
    , controller_name_(controller_name)
    , yaw_name_(yaw_name)
    , pitch_name_(pitch_name)
    , cmd_topic_(std::string("robot_base/") + controller_name + "_cmd")
    , state_topic_(std::string("robot_base/") + controller_name + "_state")
    , joint_topic_(std::string("cmd_") + controller_name + "_joint")
{

    // parameter
    std::string robot_name;
    node_->get_parameter("robot_name", robot_name);
    state_topic_ = "/" + robot_name + "/robot_base/" + controller_name_ + "_state";
    cmd_topic_ = "/" + robot_name + "/robot_base/" + controller_name_ + "_cmd";
    joint_topic_ = "/" + robot_name + "/cmd_" + controller_name + "_joint";
    declare_pid_parameter(node_, controller_name_ + ".pitch_pid");
    declare_pid_parameter(node_, controller_name_ + ".yaw_pid");
    get_pid_parameter(node_, controller_name_ + ".pitch_pid", picth_pid_param_);
    get_pid_parameter(node_, controller_name_ + ".yaw_pid", yaw_pid_param_);
    set_pitch_pid(picth_pid_param_);
    set_yaw_pid(yaw_pid_param_);
    // sensor callback
    gimbal_pos_sensor->add_callback(
        [this](const rmoss_interfaces::msg::Gimbal& data, const rclcpp::Time& /*stamp*/) {
            cur_yaw_ = data.yaw;
            cur_pitch_ = data.pitch;
        });
    // ros pub and sub
    using namespace std::placeholders;
    rmoss_gimbal_state_pub_ = node_->create_publisher<rmoss_interfaces::msg::Gimbal>(
        state_topic_, 10);
    rmoss_gimbal_cmd_sub_ = node_->create_subscription<rmoss_interfaces::msg::GimbalCmd>(
        cmd_topic_, 10, std::bind(&GimbalController::gimbal_cb, this, _1));
    ros_gimbal_cmd_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        joint_topic_, 10, std::bind(&GimbalController::gimbal_joint_cb, this, _1));

    chassis_twist_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "/" + robot_name + "/cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            if (controller_name_ == "gimbal_big_controller") {
                // 进入速度模式，用反向底盘角速度来锁定世界朝向
                use_velocity_mode_ = true;
                desired_vel_pitch_ = 0.0; // pitch 轴不补偿
                desired_vel_yaw_ = -msg->angular.z; // 反向底盘角速度
            } else {
                target_yaw_ = 0.0;
                use_velocity_mode_ = false;
            }
        });

    // timer
    int pid_rate = 100;
    pid_period_ = std::chrono::milliseconds(1000 / pid_rate);
    controller_timer_ = node_->create_wall_timer(
        pid_period_,
        std::bind(&GimbalController::update, this));
    int publish_rate = 10;
    gimbal_state_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(1000 / publish_rate),
        std::bind(&GimbalController::gimbal_state_timer_cb, this));
    // 构造里，最后加入
    // 仅在左右小云台控制器里执行
    if (controller_name_ == "gimbal_left_controller" || controller_name_ == "gimbal_right_controller") {
        std::string big_topic = "/" + robot_name + "/robot_base/gimbal_big_controller_state";

        gimbal_big_state_sub_ = node_->create_subscription<rmoss_interfaces::msg::Gimbal>(
            big_topic, 10,
            [this](const rmoss_interfaces::msg::Gimbal::SharedPtr msg) {
                /* ---------- 跟随策略 ----------
                 * yaw  : 0    → 枪口朝向与大云台一致
                 * pitch: msg→ 同步俯仰（如果只想固定俯仰，可继续用 0）
                 */

                target_yaw_ = msg->yaw; // 相对大云台的角度 = 0
                target_pitch_ = 0.0; // 把大云台的俯仰同步给小云台
                                     // 若仍固定俯仰，可改为 0.0

                use_velocity_mode_ = false; // 切回角度‑PID 模式
                yaw_pid_.Reset(); // 清空上一模式的误差积分
                picth_pid_.Reset();

                if (!use_velocity_mode_) {
                    // 大云台的yaw角度通过独立控制
                    target_yaw_ = msg->yaw; // 大云台的yaw由独立的命令来控制
                    target_pitch_ = 0.0; // 同步俯仰（如果需要固定俯仰，可改为 0.0）
                }
            });
    }
}

void GimbalController::update()
{
    rmoss_interfaces::msg::Gimbal cmd;
    if (use_velocity_mode_) {

        // 直接用期望速度
        cmd.yaw = 0.0;
        cmd.pitch = 0.0;
    } else {
        // pid for pitch
        double pitch_err = cur_pitch_ - target_pitch_;
        cmd.pitch = picth_pid_.Update(pitch_err, pid_period_);
        // pid for yaw
        double yaw_err = cur_yaw_ - target_yaw_;
        cmd.yaw = yaw_pid_.Update(yaw_err, pid_period_);
    }
    // set CMD
    gimbal_vel_actuator_->set(cmd);
}

void GimbalController::gimbal_state_timer_cb()
{
    rmoss_interfaces::msg::Gimbal gimbal_pos;
    gimbal_pos.pitch = cur_pitch_;
    gimbal_pos.yaw = cur_yaw_;
    rmoss_gimbal_state_pub_->publish(gimbal_pos);
}

void GimbalController::gimbal_joint_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Using ABSOLUTE_ANGLE
    if (msg->name.size() != msg->position.size()) {
        RCLCPP_ERROR(
            node_->get_logger(), "JointState message name and position arrays are of different sizes");
        return;
    }
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (!pitch_name_.empty() && msg->name[i] == pitch_name_) {
            target_pitch_ = msg->position[i];
        } else if (msg->name[i] == yaw_name_) {
            target_yaw_ = msg->position[i];
        }
    }
    // limitation for pitch
    if (!pitch_name_.empty()) {
        target_pitch_ = std::clamp(target_pitch_, -1.0, 1.0);
    } else {
        target_pitch_ = 0.0;
    }
}

void GimbalController::gimbal_cb(const rmoss_interfaces::msg::GimbalCmd::SharedPtr msg)
{
    // for pitch
    if (msg->pitch_type == msg->ABSOLUTE_ANGLE) {
        target_pitch_ = msg->position.pitch;
    } else if (msg->pitch_type == msg->RELATIVE_ANGLE) {
        target_pitch_ = cur_pitch_ + msg->position.pitch;
    } else {
        RCLCPP_WARN(node_->get_logger(), "pitch cmd type[%d] isn't supported!", msg->pitch_type);
    }
    // limitation for pitch
    target_pitch_ = std::min(target_pitch_, 1.0);
    target_pitch_ = std::max(target_pitch_, -1.0);
    // for yaw
    if (msg->yaw_type == msg->ABSOLUTE_ANGLE) {
        target_yaw_ = msg->position.yaw;
    } else if (msg->yaw_type == msg->RELATIVE_ANGLE) {
        target_yaw_ = cur_yaw_ + msg->position.yaw;
    } else if (msg->yaw_type == msg->VELOCITY) {
        use_velocity_mode_ = true;
        desired_vel_yaw_ = msg->velocity.yaw;
    } else {
        RCLCPP_WARN(node_->get_logger(), "yaw cmd type[%d] isn't supported!", msg->yaw_type);
    }
}

void GimbalController::set_yaw_pid(struct PidParam pid_param)
{
    yaw_pid_.Init(
        pid_param.p, pid_param.i, pid_param.d, pid_param.imax,
        pid_param.imin, pid_param.cmdmax, pid_param.cmdmin, pid_param.offset);
}
void GimbalController::set_pitch_pid(struct PidParam pid_param)
{
    picth_pid_.Init(
        pid_param.p, pid_param.i, pid_param.d, pid_param.imax,
        pid_param.imin, pid_param.cmdmax, pid_param.cmdmin, pid_param.offset);
}

void GimbalController::reset()
{
    target_pitch_ = 0;
    target_yaw_ = 0;
}

} // namespace rmoss_gz_base
