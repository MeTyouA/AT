#ifndef FAST_PLANNER__FAST_PLANNER_SMOOTHER_HPP_
#define FAST_PLANNER__FAST_PLANNER_SMOOTHER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_core/smoother.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "grad_replanner/PlannerInterface.h"

class RcEsdfMap;

namespace fast_planner {

class FastPlannerSmoother : public nav2_core::Smoother {
public:
  FastPlannerSmoother() = default;
  ~FastPlannerSmoother() override;

  void
  configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
            std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
            std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub)
      override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  bool smooth(nav_msgs::msg::Path &path,
              const rclcpp::Duration &max_time) override;

private:
  void
  obstacleCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  bool convertPathToPlannerPoints(
      const nav_msgs::msg::Path &path,
      std::vector<Planner::PathPoint> &planner_path) const;

  bool buildObstacleList(const std::string &target_frame,
                         std::vector<Planner::ObstacleInfo> &obstacles);

  void
  overwritePath(nav_msgs::msg::Path &path,
                const std::vector<Planner::PathPoint> &planner_result) const;

  bool configureRcEsdfFootprint(const std::vector<double> &footprint_values);
  bool isPathRcSafe(const nav_msgs::msg::Path &path,
                    const std::vector<Planner::ObstacleInfo> &obstacles) const;
  void publishDebugVisualization(
      const nav_msgs::msg::Path &input_path,
      const nav_msgs::msg::Path &optimized_path,
      const std::vector<Planner::ObstacleInfo> &obstacles);
  void
  publishObstacleCloud(const std_msgs::msg::Header &header,
                       const std::vector<Planner::ObstacleInfo> &obstacles);
  void publishEsdfGrid(const std_msgs::msg::Header &header);
  void publishRcEsdfGrid();
  void publishRcEsdfFootprint();
  void publishCachedDebugMaps();
  double getPathYaw(const nav_msgs::msg::Path &path, size_t pose_index) const;

  static geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);
  static double distance2d(const Planner::PathPoint &a,
                           const Planner::PathPoint &b);
  static int distanceToOccupancy(double distance, double max_distance);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<Planner::PlannerInterface> planner_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_obstacle_cloud_;
  std::mutex obstacle_mutex_;

  rclcpp::Logger logger_{rclcpp::get_logger("fast_planner_smoother")};
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;

  double max_vel_{2.5};
  double max_acc_{4.5};
  double map_resolution_{0.1};
  double map_x_size_{10.0};
  double map_y_size_{10.0};
  double map_z_size_{0.5};
  double inflate_value_{0.25};
  double path_sample_interval_{0.1};
  double esdf_distance_weight_{8.0};
  double esdf_safe_distance_{0.3};
  bool rc_esdf_enabled_{true};
  bool rc_esdf_hard_check_enabled_{true};
  double rc_esdf_lambda_{120.0};
  double rc_esdf_margin_{0.05};
  double rc_esdf_map_width_{2.0};
  double rc_esdf_map_height_{2.0};
  double rc_esdf_resolution_{0.02};
  double rc_esdf_query_radius_{1.2};
  double obstacle_min_z_{-2.0};
  double obstacle_max_z_{2.0};
  double obstacle_min_intensity_{0.1};
  double obstacle_max_intensity_{2.0};
  int obstacle_sample_step_{1};
  int max_obstacle_points_{4000};
  int rc_esdf_obstacle_sample_step_{3};
  int rc_esdf_max_obstacles_per_pose_{120};
  int rc_esdf_pose_sample_step_{2};
  std::string obstacle_topic_{"terrain_map_ext"};
  bool debug_visualization_enabled_{false};
  double debug_map_publish_period_{1.0};
  double debug_esdf_z_{0.2};
  double debug_esdf_max_distance_{1.0};
  double debug_rc_esdf_max_distance_{0.5};
  std::string debug_rc_esdf_frame_{"gimbal_yaw_fake"};
  nav_msgs::msg::OccupancyGrid last_esdf_grid_;
  bool has_last_esdf_grid_{false};
  bool rc_esdf_ready_{false};
  std::vector<double> rc_esdf_footprint_values_{0.35,  0.35,  -0.35, 0.35,
                                                -0.35, -0.35, 0.35,  -0.35};
  std::vector<Eigen::Vector2d> rc_esdf_footprint_;
  std::unique_ptr<RcEsdfMap> rc_esdf_map_;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
      debug_input_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
      debug_optimized_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      debug_obstacles_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
      debug_esdf_grid_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
      debug_rc_esdf_grid_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::
      SharedPtr debug_rc_esdf_footprint_pub_;
  rclcpp::TimerBase::SharedPtr debug_map_timer_;
};

} // namespace fast_planner

#endif // FAST_PLANNER__FAST_PLANNER_SMOOTHER_HPP_
