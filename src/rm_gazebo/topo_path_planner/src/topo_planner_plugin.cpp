/**
 * @file topo_planner_plugin.cpp
 * @brief Nav2 GlobalPlanner plugin implementation for TopoPlanner
 */

#include "topo_path_planner/topo_planner_plugin.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace topo_path_planner
{

TopoPlannerPlugin::TopoPlannerPlugin()
  : planner_initialized_(false),
    costmap_(nullptr),
    sample_distance_(0.5),
    max_samples_(500),
    use_skeleton_(true),
    enable_smoothing_(true),
    smoothing_iterations_(5)
{
}

TopoPlannerPlugin::~TopoPlannerPlugin()
{
  cleanup();
}

void TopoPlannerPlugin::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // Store parent node
  node_ = parent;
  node_shared_ = node_.lock();
  if (!node_shared_) {
    throw std::runtime_error("Failed to lock parent node!");
  }

  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();

  // Declare and get parameters
  getParameters();

  // Initialize TopoPlanner with costmap
  planner_.setSampleDistance(sample_distance_);
  planner_.setUseSkeleton(use_skeleton_);
  planner_.setMaxSamples(max_samples_);
  planner_.setSmoothing(enable_smoothing_);

  if (!planner_.initialize(costmap_)) {
    RCLCPP_ERROR(
      node_shared_->get_logger(),
      "TopoPlanner initialization failed!");
    throw std::runtime_error("TopoPlanner initialization failed!");
  }

  planner_initialized_ = true;

  RCLCPP_INFO(
    node_shared_->get_logger(),
    "TopoPlannerPlugin configured: %s, frame: %s",
    name_.c_str(), global_frame_.c_str());
}

void TopoPlannerPlugin::activate()
{
  if (!node_shared_) {
    node_shared_ = node_.lock();
  }

  if (planner_initialized_) {
    RCLCPP_INFO(
      node_shared_->get_logger(),
      "TopoPlannerPlugin '%s' activated", name_.c_str());
  }
}

void TopoPlannerPlugin::deactivate()
{
  RCLCPP_INFO(
    node_shared_->get_logger(),
    "TopoPlannerPlugin '%s' deactivated", name_.c_str());
}

void TopoPlannerPlugin::cleanup()
{
  planner_initialized_ = false;
  RCLCPP_INFO(
    node_shared_->get_logger(),
    "TopoPlannerPlugin '%s' cleaned up", name_.c_str());
}

nav_msgs::msg::Path TopoPlannerPlugin::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  // Validate planner is initialized
  if (!planner_initialized_) {
    RCLCPP_ERROR(
      node_shared_->get_logger(),
      "Planner not initialized!");
    return global_path;
  }

  // Validate frame IDs
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_shared_->get_logger(),
      "Start pose must be in %s frame, got %s",
      global_frame_.c_str(), start.header.frame_id.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_shared_->get_logger(),
      "Goal pose must be in %s frame, got %s",
      global_frame_.c_str(), goal.header.frame_id.c_str());
    return global_path;
  }

  // Convert poses to Eigen vectors
  Eigen::Vector2d start_pos = poseToEigen(start);
  Eigen::Vector2d goal_pos = poseToEigen(goal);

  RCLCPP_INFO(
    node_shared_->get_logger(),
    "Planning: from (%.2f, %.2f) to (%.2f, %.2f)",
    start_pos.x(), start_pos.y(), goal_pos.x(), goal_pos.y());

  // Execute topological planning
  TopoPlanResult result = planner_.plan(start_pos, goal_pos, 5);

  if (!result.success) {
    RCLCPP_ERROR(
      node_shared_->get_logger(),
      "Planning failed: %s", result.message.c_str());
    return global_path;
  }

  RCLCPP_INFO(
    node_shared_->get_logger(),
    "Planning successful! Path length: %.2f, cost: %.2f, nodes: %zu",
    result.best_path.length, result.best_path.cost,
    result.best_path.node_ids.size());

  // Convert TopoPath to nav_msgs::msg::Path
  global_path = toNavPath(result.best_path);

  return global_path;
}

nav_msgs::msg::Path TopoPlannerPlugin::toNavPath(const TopoPath& topo_path)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = node_shared_->now();

  for (const auto& pt : topo_path.points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = node_shared_->now();

    pose.pose.position.x = pt.x();
    pose.pose.position.y = pt.y();
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;  // Default orientation

    path.poses.push_back(pose);
  }

  return path;
}

Eigen::Vector2d TopoPlannerPlugin::poseToEigen(
  const geometry_msgs::msg::PoseStamped& pose)
{
  return Eigen::Vector2d(pose.pose.position.x, pose.pose.position.y);
}

void TopoPlannerPlugin::getParameters()
{
  // Declare parameters with default values using auto_declare
  node_shared_->declare_parameter(name_ + ".sample_distance", 0.5);
  node_shared_->declare_parameter(name_ + ".max_samples", 500);
  node_shared_->declare_parameter(name_ + ".use_skeleton", true);
  node_shared_->declare_parameter(name_ + ".enable_smoothing", true);
  node_shared_->declare_parameter(name_ + ".smoothing_iterations", 5);

  // Get parameter values
  node_shared_->get_parameter(name_ + ".sample_distance", sample_distance_);
  node_shared_->get_parameter(name_ + ".max_samples", max_samples_);
  node_shared_->get_parameter(name_ + ".use_skeleton", use_skeleton_);
  node_shared_->get_parameter(name_ + ".enable_smoothing", enable_smoothing_);
  node_shared_->get_parameter(name_ + ".smoothing_iterations", smoothing_iterations_);

  RCLCPP_INFO(
    node_shared_->get_logger(),
    "TopoPlannerPlugin parameters: sample_dist=%.2f, max_samples=%d, "
    "use_skeleton=%s, smoothing=%s",
    sample_distance_, max_samples_,
    use_skeleton_ ? "true" : "false",
    enable_smoothing_ ? "true" : "false");
}

}  // namespace topo_path_planner

// Export plugin for pluginlib
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  topo_path_planner::TopoPlannerPlugin,
  nav2_core::GlobalPlanner)
