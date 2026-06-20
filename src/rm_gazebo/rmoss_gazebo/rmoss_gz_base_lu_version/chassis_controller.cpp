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
#include "rmoss_gz_base_lu_version/chassis_controller.hpp"

#include <memory>
#include <string>


namespace rmoss_gz_base
{
ChassisController::ChassisController(
  rclcpp::Node::SharedPtr node,
  Actuator<geometry_msgs::msg::Twist>::SharedPtr chassis_actuator,
  const std::string & controller_name)
: node_(node), chassis_actuator_(chassis_actuator) 
{ std::string robot_name;
  node_->get_parameter("robot_name",robot_name);
  declare_pid_parameter(node_, controller_name + ".follow_yaw_pid");
  get_pid_parameter(node_, controller_name + ".follow_yaw_pid", chassis_pid_param_);
  set_chassis_pid(chassis_pid_param_);
  std::string chassis_cmd_topic = "/"+ robot_name +"/robot_base/chassis_cmd";
  std::string robot_cmd_vel_topic = "/"+ robot_name +"/cmd_vel";
  using namespace std::placeholders;
  ros_chassis_cmd_sub_ = node_->create_subscription<rmoss_interfaces::msg::ChassisCmd>(
  chassis_cmd_topic , 10, std::bind(&ChassisController::chassis_cb, this, _1));
  ros_cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
  robot_cmd_vel_topic  , 10, std::bind(&ChassisController::cmd_vel_cb, this, _1));
  // timer and set_parameters callback
  auto period = std::chrono::milliseconds(10);
  controller_timer_ = node_->create_wall_timer(
    period, std::bind(&ChassisController::update, this));
  
}

void ChassisController::update()
{
  static const auto DT = std::chrono::milliseconds(10);
  geometry_msgs::msg::Twist result_vel;

  result_vel = target_vel_;
  // publish CMD
  chassis_actuator_->set(result_vel);
}

void ChassisController::chassis_cb(const rmoss_interfaces::msg::ChassisCmd::SharedPtr msg)
{
  if (msg->type == msg->VELOCITY) {
    target_vel_ = msg->twist;
  } else {
    RCLCPP_WARN(node_->get_logger(), "chassis type[%d] isn't supported!", msg->type);
  }
}

void ChassisController::cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  target_vel_ = *msg;
}

void ChassisController::set_chassis_pid(PidParam pid_param)
{
  chassis_pid_.Init(
    pid_param.p, pid_param.i, pid_param.d, pid_param.imax,
    pid_param.imin, pid_param.cmdmax, pid_param.cmdmin, pid_param.offset);
}

void ChassisController::reset()
{
  target_vel_.linear.x = 0;
  target_vel_.linear.y = 0;
  target_vel_.angular.z = 0;
  chassis_pid_.Reset();
}

}  // namespace rmoss_gz_base
