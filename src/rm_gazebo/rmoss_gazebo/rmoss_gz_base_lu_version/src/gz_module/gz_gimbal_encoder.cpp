// Copyright 2025 RoboMaster-OSS
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

// gz_gimbal_encoder.cpp

#include "rmoss_gz_base_lu_version/gz_gimbal_encoder.hpp"

namespace rmoss_gz_base {

IgnGimbalEncoder::IgnGimbalEncoder(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<ignition::transport::Node> gz_node,
    const std::string& gz_joint_state_topic,
    const std::vector<std::string>& yaw_joint_names,
    const std::vector<std::string>& pitch_joint_names)
    : node_(node)
    , gz_node_(gz_node)
    , yaw_set_(yaw_joint_names.begin(), yaw_joint_names.end()) // ← 正确成员名
    , pitch_set_(pitch_joint_names.begin(), pitch_joint_names.end())
{
    gz_node_->Subscribe(gz_joint_state_topic,
        &IgnGimbalEncoder::gz_Joint_state_cb, this);

    position_sensor_ = std::make_shared<DataSensor<rmoss_interfaces::msg::Gimbal>>();
    velocity_sensor_ = std::make_shared<DataSensor<rmoss_interfaces::msg::Gimbal>>();
}

void IgnGimbalEncoder::gz_Joint_state_cb(const ignition::msgs::Model& msg)
{
    if (!enable_)
        return;

    rmoss_interfaces::msg::Gimbal pos {}, vel {};

    for (int i = 0; i < msg.joint_size(); ++i) {
        const auto& j = msg.joint(i);
        const auto& name = j.name();

        // 只在真正匹配 yaw 时打印一次
        if (yaw_set_.count(name)) {
            pos.yaw += j.axis1().position();
            vel.yaw += j.axis1().velocity();
            if (!first_yaw_logged_) {
                RCLCPP_INFO(node_->get_logger(),
                    "matched YAW joint = %s",
                     name.c_str());
                first_yaw_logged_ = true;
            }
        }

        if (pitch_set_.count(name)) {
            pos.pitch += j.axis1().position();
            vel.pitch += j.axis1().velocity();
            if (!first_pitch_logged_) {
                RCLCPP_INFO(node_->get_logger(),
                    "matched PITCH joint = %s",
                     name.c_str());
                first_pitch_logged_ = true;
            }
        }
    }

    // 汇总完后统一发布
    const auto now = node_->get_clock()->now();
    position_sensor_->update(pos, now);
    velocity_sensor_->update(vel, now);
}

} // namespace rmoss_gz_base
