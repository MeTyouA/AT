#pragma once
#include <deque>
#include <mutex>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>

struct ImuState {
    double timestamp;
    Eigen::Quaterniond q; 
    Eigen::Vector3d p;    
    Eigen::Vector3d v;    
};

class ImuPredictor {
public:
    ImuPredictor() {
        current_state_.q.setIdentity();
        current_state_.p.setZero();
        current_state_.v.setZero();
        current_state_.timestamp = -1.0;
    }

    void feedImu(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro) {
        std::lock_guard<std::mutex> lock(imu_mtx_);

        if (current_state_.timestamp < 0) {
            current_state_.timestamp = timestamp;
            imu_buffer_.push_back(current_state_);
            return;
        }

        double dt = timestamp - current_state_.timestamp;
        if (dt <= 0 || dt > 0.1) return; 

        Eigen::Vector3d delta_angle = gyro * dt;
        double angle = delta_angle.norm();
        Eigen::Quaterniond dq = (angle < 1e-6) ? 
            Eigen::Quaterniond::Identity() : 
            Eigen::Quaterniond(Eigen::AngleAxisd(angle, delta_angle / angle));

        current_state_.q = (current_state_.q * dq).normalized();
        current_state_.p = current_state_.p + current_state_.v * dt + 0.5 * acc * dt * dt;
        current_state_.v = current_state_.v + acc * dt;
        current_state_.timestamp = timestamp;

        imu_buffer_.push_back(current_state_);
        while (imu_buffer_.size() > 2000) imu_buffer_.pop_front();
    }

    // ⭐ 关键优化：提取快照，避免并行循环中竞争锁
    std::vector<ImuState> getTrajectorySnapshot(double start_time, double end_time) {
        std::lock_guard<std::mutex> lock(imu_mtx_);
        if (imu_buffer_.empty() || end_time < imu_buffer_.front().timestamp || start_time > imu_buffer_.back().timestamp) {
            return {};
        }

        auto it_start = std::lower_bound(imu_buffer_.begin(), imu_buffer_.end(), start_time,
            [](const ImuState& s, double t) { return s.timestamp < t; });
        if (it_start != imu_buffer_.begin()) std::advance(it_start, -1);

        auto it_end = std::upper_bound(it_start, imu_buffer_.end(), end_time,
            [](double t, const ImuState& s) { return t < s.timestamp; });
        if (it_end != imu_buffer_.end()) std::advance(it_end, 1);

        return std::vector<ImuState>(it_start, it_end);
    }

    // 无锁版本的插值函数，用于并行计算
    static bool interpolate(const std::vector<ImuState>& snapshot, double t, Eigen::Quaterniond& q, Eigen::Vector3d& p) {
        if (snapshot.size() < 2) return false;
        
        auto it = std::lower_bound(snapshot.begin(), snapshot.end(), t,
            [](const ImuState& s, double t_val) { return s.timestamp < t_val; });

        if (it == snapshot.begin() || it == snapshot.end()) {
            auto const& s = (it == snapshot.begin()) ? snapshot.front() : snapshot.back();
            q = s.q; p = s.p;
            return true;
        }

        auto prev = std::prev(it);
        double ratio = (t - prev->timestamp) / (it->timestamp - prev->timestamp);
        q = prev->q.slerp(ratio, it->q);
        p = prev->p + ratio * (it->p - prev->p);
        return true;
    }

private:
    std::mutex imu_mtx_;
    std::deque<ImuState> imu_buffer_;
    ImuState current_state_;
};