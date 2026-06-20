#include <rclcpp/rclcpp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <Eigen/Dense>
#include <execution>
#include <vector>
#include <numeric>

#include "rm_deskew_node/imu_predictor.hpp"

class DeskewNode : public rclcpp::Node {
public:
    DeskewNode() : Node("rm_deskew_node") {
        // 预计算静态外参逆矩阵
        R_ext_ = Eigen::Quaterniond::Identity();
        P_ext_ << -0.011, -0.02329, 0.04412;
        R_ext_inv_ = R_ext_.inverse();

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                imu_predictor_.feedImu(
                    msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9,
                    Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z),
                    Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z));
            });

        lidar_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            "/livox/lidar", rclcpp::SensorDataQoS(),
            [this](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) { processLidar(msg); });

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/deskewed_cloud", 10);
    }

private:
    void processLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
        if (msg->points.empty()) return;

        double frame_start_time = msg->timebase * 1e-9;
        double frame_end_time = frame_start_time + msg->points.back().offset_time * 1e-9;

        // 1. 获取 IMU 快照（临界区结束）
        auto snapshot = imu_predictor_.getTrajectorySnapshot(frame_start_time, frame_end_time);
        if (snapshot.empty()) return;

        // 2. 获取参考位姿（通常以帧末尾或开头为基准）
        Eigen::Quaterniond q_end; Eigen::Vector3d p_end;
        ImuPredictor::interpolate(snapshot, frame_end_time, q_end, p_end);
        Eigen::Matrix3d R_end_inv = q_end.toRotationMatrix().transpose();

        // 3. 预准备输出消息（直接操作 buffer）
        auto out_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        out_msg->header = msg->header;
        sensor_msgs::PointCloud2Modifier modifier(*out_msg);
        modifier.setPointCloud2Fields(4, 
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(msg->point_num);

        // 获取迭代器
        sensor_msgs::PointCloud2Iterator<float> it_x(*out_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> it_y(*out_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> it_z(*out_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> it_i(*out_msg, "intensity");

        // 4. 并行去畸变处理
        std::vector<size_t> indices(msg->point_num);
        std::iota(indices.begin(), indices.end(), 0);

        std::for_each(std::execution::par, indices.begin(), indices.end(), [&](size_t idx) {
            const auto& p_in = msg->points[idx];
            double pt_time = frame_start_time + p_in.offset_time * 1e-9;

            Eigen::Quaterniond q_i; Eigen::Vector3d p_i;
            if (!ImuPredictor::interpolate(snapshot, pt_time, q_i, p_i)) {
                // 如果插值失败，标记为无效点
                *(it_x + idx) = *(it_y + idx) = *(it_z + idx) = std::numeric_limits<float>::quiet_NaN();
                return;
            }

            // 优化后的数学运算：LiDAR -> IMU -> World -> End_IMU -> End_LiDAR
            Eigen::Matrix3d R_rel = R_end_inv * q_i.toRotationMatrix();
            Eigen::Vector3d t_rel = R_end_inv * (p_i - p_end);

            // 合并变换矩阵降低计算量
            // p_out = R_ext_inv * (R_rel * (R_ext * p_in + P_ext) + t_rel - P_ext)
            Eigen::Vector3d p_raw(p_in.x, p_in.y, p_in.z);
            Eigen::Vector3d p_deskewed = R_ext_inv_ * (R_rel * (R_ext_ * p_raw + P_ext_) + t_rel - P_ext_);

            // 距离过滤
            float r2 = p_deskewed.squaredNorm();
            if (r2 < 0.09f || r2 > 100.0f) {
                *(it_x + idx) = *(it_y + idx) = *(it_z + idx) = std::numeric_limits<float>::quiet_NaN();
            } else {
                *(it_x + idx) = p_deskewed.x();
                *(it_y + idx) = p_deskewed.y();
                *(it_z + idx) = p_deskewed.z();
                *(it_i + idx) = p_in.reflectivity;
            }
        });

        cloud_pub_->publish(*out_msg);
    }

    ImuPredictor imu_predictor_;
    Eigen::Quaterniond R_ext_, R_ext_inv_;
    Eigen::Vector3d P_ext_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DeskewNode>());
    rclcpp::shutdown();
    return 0;
}