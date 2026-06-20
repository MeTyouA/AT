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

#ifndef RMOSS_GZ_BASE__GZ_GIMBAL_ENCODER_HPP_
#define RMOSS_GZ_BASE__GZ_GIMBAL_ENCODER_HPP_

#include "hardware_interface.hpp"
#include "ignition/transport/Node.hh"
#include "rclcpp/rclcpp.hpp"
#include "rmoss_interfaces/msg/gimbal.hpp"
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

namespace rmoss_gz_base {

// C++17 写法；若编译器支持 C++20，可直接用 std::string::ends_with
inline bool ends_with(const std::string& full, const std::string& suffix)
{
    return full.size() >= suffix.size() && full.compare(full.size() - suffix.size(), suffix.size(), suffix) == 0;
}

class IgnGimbalEncoder {
public:
    IgnGimbalEncoder(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<ignition::transport::Node> gz_node,
        const std::string& gz_joint_state_topic,
        const std::vector<std::string>& yaw_joint_names,
        const std::vector<std::string>& pitch_joint_names);
    ~IgnGimbalEncoder() { }

    void enable(bool enable) { enable_ = enable; }
    Sensor<rmoss_interfaces::msg::Gimbal>::SharedPtr get_position_sensor() { return position_sensor_; }
    Sensor<rmoss_interfaces::msg::Gimbal>::SharedPtr get_velocity_sensor() { return velocity_sensor_; }

private:
    void gz_Joint_state_cb(const ignition::msgs::Model& msg);

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<ignition::transport::Node> gz_node_;
    bool enable_ { false };
    bool first_logged_ { false };
    bool first_yaw_logged_ { false };
    bool first_pitch_logged_ { false };

    // info
    std::shared_ptr<DataSensor<rmoss_interfaces::msg::Gimbal>> position_sensor_;
    std::shared_ptr<DataSensor<rmoss_interfaces::msg::Gimbal>> velocity_sensor_;
    std::vector<std::string> yaw_joints_;
    std::vector<std::string> pitch_joints_;
    std::unordered_set<std::string> yaw_set_;
    std::unordered_set<std::string> pitch_set_;
};

} // namespace rmoss_gz_base

#endif // RMOSS_GZ_BASE__GZ_GIMBAL_ENCODER_HPP_
