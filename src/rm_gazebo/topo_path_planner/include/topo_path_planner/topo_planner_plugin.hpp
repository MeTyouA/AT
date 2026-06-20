/**
 * @file topo_planner_plugin.hpp
 * @brief Nav2 GlobalPlanner plugin interface for TopoPlanner
 */

#ifndef TOPO_PATH_PLANNER_TOPO_PLANNER_PLUGIN_HPP_
#define TOPO_PATH_PLANNER_TOPO_PLANNER_PLUGIN_HPP_

#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "topo_path_planner/topo_graph.hpp"

namespace topo_path_planner
{

/**
 * @class TopoPlannerPlugin
 * @brief Nav2 GlobalPlanner plugin wrapping TopoPlanner
 */
class TopoPlannerPlugin : public nav2_core::GlobalPlanner
{
public:
  /**
   * @brief Constructor
   */
  TopoPlannerPlugin();

  /**
   * @brief Destructor
   */
  ~TopoPlannerPlugin() override;

  /**
   * @brief Configure the plugin (called during on_configure state)
   * @param parent Parent node pointer
   * @param name Plugin name
   * @param tf TF buffer pointer
   * @param costmap_ros Costmap ROS wrapper pointer
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Activate the plugin (called during on_activate state)
   */
  void activate() override;

  /**
   * @brief Deactivate the plugin (called during on_deactivate state)
   */
  void deactivate() override;

  /**
   * @brief Cleanup resources (called during on_cleanup state)
   */
  void cleanup() override;

  /**
   * @brief Create a global plan from start to goal
   * @param start Starting pose
   * @param goal Goal pose
   * @return Generated path
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // Core planner instance
  TopoPlanner planner_;
  bool planner_initialized_;

  // ROS2 interfaces
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_shared_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string global_frame_;

  // Planner parameters
  double sample_distance_;
  int max_samples_;
  bool use_skeleton_;
  bool enable_smoothing_;
  int smoothing_iterations_;

  /**
   * @brief Convert TopoPath to nav_msgs::msg::Path
   */
  nav_msgs::msg::Path toNavPath(const TopoPath& topo_path);

  /**
   * @brief Convert PoseStamped to Eigen::Vector2d
   */
  Eigen::Vector2d poseToEigen(const geometry_msgs::msg::PoseStamped& pose);

  /**
   * @brief Declare and get parameters
   */
  void getParameters();
};

}  // namespace topo_path_planner

#endif  // TOPO_PATH_PLANNER_TOPO_PLANNER_PLUGIN_HPP_
