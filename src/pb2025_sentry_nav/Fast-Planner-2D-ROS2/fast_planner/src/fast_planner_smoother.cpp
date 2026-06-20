#include "fast_planner/fast_planner_smoother.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rc_esdf.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fast_planner {

using nav2_util::declare_parameter_if_not_declared;

FastPlannerSmoother::~FastPlannerSmoother() = default;

void FastPlannerSmoother::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>) {
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock smoother_server node");
  }

  node_ = parent;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  declare_parameter_if_not_declared(node, name + ".max_vel",
                                    rclcpp::ParameterValue(max_vel_));
  declare_parameter_if_not_declared(node, name + ".max_acc",
                                    rclcpp::ParameterValue(max_acc_));
  declare_parameter_if_not_declared(node, name + ".map_resolution",
                                    rclcpp::ParameterValue(map_resolution_));
  declare_parameter_if_not_declared(node, name + ".map_x_size",
                                    rclcpp::ParameterValue(map_x_size_));
  declare_parameter_if_not_declared(node, name + ".map_y_size",
                                    rclcpp::ParameterValue(map_y_size_));
  declare_parameter_if_not_declared(node, name + ".map_z_size",
                                    rclcpp::ParameterValue(map_z_size_));
  declare_parameter_if_not_declared(node, name + ".inflate_value",
                                    rclcpp::ParameterValue(inflate_value_));
  declare_parameter_if_not_declared(
      node, name + ".path_sample_interval",
      rclcpp::ParameterValue(path_sample_interval_));
  declare_parameter_if_not_declared(
      node, name + ".esdf_distance_weight",
      rclcpp::ParameterValue(esdf_distance_weight_));
  declare_parameter_if_not_declared(
      node, name + ".esdf_safe_distance",
      rclcpp::ParameterValue(esdf_safe_distance_));
  declare_parameter_if_not_declared(node, name + ".rc_esdf_enabled",
                                    rclcpp::ParameterValue(rc_esdf_enabled_));
  declare_parameter_if_not_declared(
      node, name + ".rc_esdf_hard_check_enabled",
      rclcpp::ParameterValue(rc_esdf_hard_check_enabled_));
  declare_parameter_if_not_declared(node, name + ".rc_esdf_lambda",
                                    rclcpp::ParameterValue(rc_esdf_lambda_));
  declare_parameter_if_not_declared(node, name + ".rc_esdf_margin",
                                    rclcpp::ParameterValue(rc_esdf_margin_));
  declare_parameter_if_not_declared(node, name + ".rc_esdf_map_width",
                                    rclcpp::ParameterValue(rc_esdf_map_width_));
  declare_parameter_if_not_declared(
      node, name + ".rc_esdf_map_height",
      rclcpp::ParameterValue(rc_esdf_map_height_));
  declare_parameter_if_not_declared(
      node, name + ".rc_esdf_resolution",
      rclcpp::ParameterValue(rc_esdf_resolution_));
  declare_parameter_if_not_declared(
      node, name + ".rc_esdf_query_radius",
      rclcpp::ParameterValue(rc_esdf_query_radius_));
  declare_parameter_if_not_declared(
      node, name + ".rc_esdf_obstacle_sample_step",
      rclcpp::ParameterValue(rc_esdf_obstacle_sample_step_));
  declare_parameter_if_not_declared(
      node, name + ".rc_esdf_max_obstacles_per_pose",
      rclcpp::ParameterValue(rc_esdf_max_obstacles_per_pose_));
  declare_parameter_if_not_declared(
      node, name + ".rc_esdf_pose_sample_step",
      rclcpp::ParameterValue(rc_esdf_pose_sample_step_));
  declare_parameter_if_not_declared(
      node, name + ".rc_esdf_footprint",
      rclcpp::ParameterValue(rc_esdf_footprint_values_));
  declare_parameter_if_not_declared(node, name + ".obstacle_topic",
                                    rclcpp::ParameterValue(obstacle_topic_));
  declare_parameter_if_not_declared(node, name + ".obstacle_min_z",
                                    rclcpp::ParameterValue(obstacle_min_z_));
  declare_parameter_if_not_declared(node, name + ".obstacle_max_z",
                                    rclcpp::ParameterValue(obstacle_max_z_));
  declare_parameter_if_not_declared(
      node, name + ".obstacle_min_intensity",
      rclcpp::ParameterValue(obstacle_min_intensity_));
  declare_parameter_if_not_declared(
      node, name + ".obstacle_max_intensity",
      rclcpp::ParameterValue(obstacle_max_intensity_));
  declare_parameter_if_not_declared(
      node, name + ".obstacle_sample_step",
      rclcpp::ParameterValue(obstacle_sample_step_));
  declare_parameter_if_not_declared(
      node, name + ".max_obstacle_points",
      rclcpp::ParameterValue(max_obstacle_points_));
  declare_parameter_if_not_declared(
      node, name + ".debug_visualization_enabled",
      rclcpp::ParameterValue(debug_visualization_enabled_));
  declare_parameter_if_not_declared(
      node, name + ".debug_map_publish_period",
      rclcpp::ParameterValue(debug_map_publish_period_));
  declare_parameter_if_not_declared(node, name + ".debug_esdf_z",
                                    rclcpp::ParameterValue(debug_esdf_z_));
  declare_parameter_if_not_declared(
      node, name + ".debug_esdf_max_distance",
      rclcpp::ParameterValue(debug_esdf_max_distance_));
  declare_parameter_if_not_declared(
      node, name + ".debug_rc_esdf_max_distance",
      rclcpp::ParameterValue(debug_rc_esdf_max_distance_));
  declare_parameter_if_not_declared(
      node, name + ".debug_rc_esdf_frame",
      rclcpp::ParameterValue(debug_rc_esdf_frame_));

  node->get_parameter(name + ".max_vel", max_vel_);
  node->get_parameter(name + ".max_acc", max_acc_);
  node->get_parameter(name + ".map_resolution", map_resolution_);
  node->get_parameter(name + ".map_x_size", map_x_size_);
  node->get_parameter(name + ".map_y_size", map_y_size_);
  node->get_parameter(name + ".map_z_size", map_z_size_);
  node->get_parameter(name + ".inflate_value", inflate_value_);
  node->get_parameter(name + ".path_sample_interval", path_sample_interval_);
  node->get_parameter(name + ".esdf_distance_weight", esdf_distance_weight_);
  node->get_parameter(name + ".esdf_safe_distance", esdf_safe_distance_);
  node->get_parameter(name + ".rc_esdf_enabled", rc_esdf_enabled_);
  node->get_parameter(name + ".rc_esdf_hard_check_enabled",
                      rc_esdf_hard_check_enabled_);
  node->get_parameter(name + ".rc_esdf_lambda", rc_esdf_lambda_);
  node->get_parameter(name + ".rc_esdf_margin", rc_esdf_margin_);
  node->get_parameter(name + ".rc_esdf_map_width", rc_esdf_map_width_);
  node->get_parameter(name + ".rc_esdf_map_height", rc_esdf_map_height_);
  node->get_parameter(name + ".rc_esdf_resolution", rc_esdf_resolution_);
  node->get_parameter(name + ".rc_esdf_query_radius", rc_esdf_query_radius_);
  node->get_parameter(name + ".rc_esdf_obstacle_sample_step",
                      rc_esdf_obstacle_sample_step_);
  node->get_parameter(name + ".rc_esdf_max_obstacles_per_pose",
                      rc_esdf_max_obstacles_per_pose_);
  node->get_parameter(name + ".rc_esdf_pose_sample_step",
                      rc_esdf_pose_sample_step_);
  node->get_parameter(name + ".rc_esdf_footprint", rc_esdf_footprint_values_);
  node->get_parameter(name + ".obstacle_topic", obstacle_topic_);
  node->get_parameter(name + ".obstacle_min_z", obstacle_min_z_);
  node->get_parameter(name + ".obstacle_max_z", obstacle_max_z_);
  node->get_parameter(name + ".obstacle_min_intensity",
                      obstacle_min_intensity_);
  node->get_parameter(name + ".obstacle_max_intensity",
                      obstacle_max_intensity_);
  node->get_parameter(name + ".obstacle_sample_step", obstacle_sample_step_);
  node->get_parameter(name + ".max_obstacle_points", max_obstacle_points_);
  node->get_parameter(name + ".debug_visualization_enabled",
                      debug_visualization_enabled_);
  node->get_parameter(name + ".debug_map_publish_period",
                      debug_map_publish_period_);
  node->get_parameter(name + ".debug_esdf_z", debug_esdf_z_);
  node->get_parameter(name + ".debug_esdf_max_distance",
                      debug_esdf_max_distance_);
  node->get_parameter(name + ".debug_rc_esdf_max_distance",
                      debug_rc_esdf_max_distance_);
  node->get_parameter(name + ".debug_rc_esdf_frame", debug_rc_esdf_frame_);

  obstacle_sample_step_ = std::max(1, obstacle_sample_step_);
  max_obstacle_points_ = std::max(0, max_obstacle_points_);
  path_sample_interval_ = std::max(0.02, path_sample_interval_);
  map_resolution_ = std::max(0.02, map_resolution_);
  esdf_distance_weight_ = std::max(0.0, esdf_distance_weight_);
  esdf_safe_distance_ = std::max(0.0, esdf_safe_distance_);
  rc_esdf_lambda_ = std::max(0.0, rc_esdf_lambda_);
  rc_esdf_margin_ = std::max(0.0, rc_esdf_margin_);
  rc_esdf_map_width_ = std::max(0.1, rc_esdf_map_width_);
  rc_esdf_map_height_ = std::max(0.1, rc_esdf_map_height_);
  rc_esdf_resolution_ = std::max(0.005, rc_esdf_resolution_);
  rc_esdf_query_radius_ = std::max(0.0, rc_esdf_query_radius_);
  rc_esdf_obstacle_sample_step_ = std::max(1, rc_esdf_obstacle_sample_step_);
  rc_esdf_max_obstacles_per_pose_ =
      std::max(0, rc_esdf_max_obstacles_per_pose_);
  rc_esdf_pose_sample_step_ = std::max(1, rc_esdf_pose_sample_step_);
  debug_map_publish_period_ = std::max(0.1, debug_map_publish_period_);
  debug_esdf_max_distance_ = std::max(0.01, debug_esdf_max_distance_);
  debug_rc_esdf_max_distance_ = std::max(0.01, debug_rc_esdf_max_distance_);
  rc_esdf_ready_ =
      rc_esdf_enabled_ && configureRcEsdfFootprint(rc_esdf_footprint_values_);

  planner_ = std::make_shared<Planner::PlannerInterface>();
  planner_->initParam(max_vel_, max_acc_);
  planner_->configureEsdfPenalty(esdf_distance_weight_, esdf_safe_distance_);
  planner_->initEsdfMap(map_x_size_, map_y_size_, map_z_size_, map_resolution_,
                        Eigen::Vector3d::Zero(), inflate_value_);
  planner_->configureRcEsdf(
      rc_esdf_enabled_ && rc_esdf_ready_, rc_esdf_lambda_, rc_esdf_margin_,
      rc_esdf_map_width_, rc_esdf_map_height_, rc_esdf_resolution_,
      rc_esdf_query_radius_, rc_esdf_max_obstacles_per_pose_,
      rc_esdf_obstacle_sample_step_, rc_esdf_footprint_);

  obstacle_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      obstacle_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FastPlannerSmoother::obstacleCloudCallback, this,
                std::placeholders::_1));

  debug_input_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
      "fast_planner/input_path", rclcpp::QoS(1));
  debug_optimized_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
      "fast_planner/optimized_path", rclcpp::QoS(1));
  debug_obstacles_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "fast_planner/obstacles", rclcpp::QoS(1));
  debug_esdf_grid_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "fast_planner/esdf_grid", rclcpp::QoS(1).transient_local());
  debug_rc_esdf_grid_pub_ =
      node->create_publisher<nav_msgs::msg::OccupancyGrid>(
          "fast_planner/rc_esdf_grid", rclcpp::QoS(1).transient_local());
  debug_rc_esdf_footprint_pub_ =
      node->create_publisher<visualization_msgs::msg::MarkerArray>(
          "fast_planner/rc_esdf_footprint", rclcpp::QoS(1).transient_local());

  debug_map_timer_ = node->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(debug_map_publish_period_)),
      std::bind(&FastPlannerSmoother::publishCachedDebugMaps, this));

  RCLCPP_INFO(
      logger_,
      "Fast-Planner smoother configured. obstacle_topic=%s, "
      "map=%.2fx%.2fx%.2f, res=%.2f, rc_esdf=%s, debug_visualization=%s",
      obstacle_topic_.c_str(), map_x_size_, map_y_size_, map_z_size_,
      map_resolution_, (rc_esdf_enabled_ && rc_esdf_ready_) ? "on" : "off",
      debug_visualization_enabled_ ? "on" : "off");
}

void FastPlannerSmoother::cleanup() {
  debug_map_timer_.reset();
  obstacle_sub_.reset();
  debug_input_path_pub_.reset();
  debug_optimized_path_pub_.reset();
  debug_obstacles_pub_.reset();
  debug_esdf_grid_pub_.reset();
  debug_rc_esdf_grid_pub_.reset();
  debug_rc_esdf_footprint_pub_.reset();
  planner_.reset();
  latest_obstacle_cloud_.reset();
  has_last_esdf_grid_ = false;
  rc_esdf_map_.reset();
  rc_esdf_ready_ = false;
}

void FastPlannerSmoother::activate() {
  if (!debug_visualization_enabled_) {
    return;
  }

  debug_input_path_pub_->on_activate();
  debug_optimized_path_pub_->on_activate();
  debug_obstacles_pub_->on_activate();
  debug_esdf_grid_pub_->on_activate();
  debug_rc_esdf_grid_pub_->on_activate();
  debug_rc_esdf_footprint_pub_->on_activate();

  publishRcEsdfGrid();
  publishRcEsdfFootprint();
}

void FastPlannerSmoother::deactivate() {
  if (debug_input_path_pub_) {
    debug_input_path_pub_->on_deactivate();
  }
  if (debug_optimized_path_pub_) {
    debug_optimized_path_pub_->on_deactivate();
  }
  if (debug_obstacles_pub_) {
    debug_obstacles_pub_->on_deactivate();
  }
  if (debug_esdf_grid_pub_) {
    debug_esdf_grid_pub_->on_deactivate();
  }
  if (debug_rc_esdf_grid_pub_) {
    debug_rc_esdf_grid_pub_->on_deactivate();
  }
  if (debug_rc_esdf_footprint_pub_) {
    debug_rc_esdf_footprint_pub_->on_deactivate();
  }
}

void FastPlannerSmoother::obstacleCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(obstacle_mutex_);
  latest_obstacle_cloud_ = msg;
}

bool FastPlannerSmoother::smooth(nav_msgs::msg::Path &path,
                                 const rclcpp::Duration &max_time) {
  if (!planner_) {
    RCLCPP_ERROR(logger_, "Fast-Planner smoother is not configured");
    return false;
  }

  if (path.poses.size() < 2) {
    return true;
  }

  const auto start_time = clock_->now();
  const nav_msgs::msg::Path original_path = path;

  std::vector<Planner::PathPoint> planner_path;
  if (!convertPathToPlannerPoints(path, planner_path)) {
    RCLCPP_WARN(
        logger_,
        "Input path is too short for Fast-Planner; keeping original path");
    return true;
  }

  std::vector<Planner::ObstacleInfo> obstacles;
  buildObstacleList(path.header.frame_id, obstacles);

  Planner::PathPoint current_pose = planner_path.front();
  planner_->setCurrentPose(current_pose);
  planner_->setPathPoint(planner_path);
  planner_->setObstacles(obstacles);
  planner_->makePlan();

  std::vector<Planner::PathPoint> planner_result;
  planner_->getLocalPlanTrajResults(planner_result);

  if (planner_result.size() < 2) {
    publishDebugVisualization(original_path, nav_msgs::msg::Path(), obstacles);
    RCLCPP_WARN(
        logger_,
        "Fast-Planner returned an empty trajectory; keeping original path");
    return true;
  }

  nav_msgs::msg::Path optimized_path = path;
  overwritePath(optimized_path, planner_result);
  publishDebugVisualization(original_path, optimized_path, obstacles);

  path = optimized_path;
  if (rc_esdf_enabled_ && rc_esdf_hard_check_enabled_ && rc_esdf_ready_ &&
      !isPathRcSafe(path, obstacles)) {
    path = original_path;
    RCLCPP_WARN(logger_,
                "Fast-Planner RC-ESDF hard check rejected the optimized path");
    return false;
  }

  const auto elapsed = clock_->now() - start_time;
  if (elapsed > max_time) {
    RCLCPP_WARN(
        logger_,
        "Fast-Planner smoothing exceeded requested duration: %.3f s > %.3f s",
        elapsed.seconds(), max_time.seconds());
  }

  return true;
}

bool FastPlannerSmoother::convertPathToPlannerPoints(
    const nav_msgs::msg::Path &path,
    std::vector<Planner::PathPoint> &planner_path) const {
  planner_path.clear();

  Planner::PathPoint first;
  first.x = path.poses.front().pose.position.x;
  first.y = path.poses.front().pose.position.y;
  first.z = 0.0;
  first.v = 0.0;
  planner_path.push_back(first);

  for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
    Planner::PathPoint start;
    start.x = path.poses[i].pose.position.x;
    start.y = path.poses[i].pose.position.y;
    start.z = 0.0;
    start.v = 0.0;

    Planner::PathPoint end;
    end.x = path.poses[i + 1].pose.position.x;
    end.y = path.poses[i + 1].pose.position.y;
    end.z = 0.0;
    end.v = 0.0;

    const double segment_length = distance2d(start, end);
    if (segment_length < 1e-6) {
      continue;
    }

    const int steps = std::max(
        1, static_cast<int>(std::ceil(segment_length / path_sample_interval_)));
    for (int step = 1; step <= steps; ++step) {
      const double ratio =
          static_cast<double>(step) / static_cast<double>(steps);
      Planner::PathPoint point;
      point.x = start.x + ratio * (end.x - start.x);
      point.y = start.y + ratio * (end.y - start.y);
      point.z = 0.0;
      point.v = 0.0;
      planner_path.push_back(point);
    }
  }

  if (planner_path.size() >= 4) {
    return true;
  }

  const auto start = planner_path.front();
  const auto goal_pose = path.poses.back().pose.position;
  Planner::PathPoint goal;
  goal.x = goal_pose.x;
  goal.y = goal_pose.y;
  goal.z = 0.0;
  goal.v = 0.0;

  if (distance2d(start, goal) < 1e-6) {
    return false;
  }

  planner_path.clear();
  for (int i = 0; i < 4; ++i) {
    const double ratio = static_cast<double>(i) / 3.0;
    Planner::PathPoint point;
    point.x = start.x + ratio * (goal.x - start.x);
    point.y = start.y + ratio * (goal.y - start.y);
    point.z = 0.0;
    point.v = 0.0;
    planner_path.push_back(point);
  }

  return true;
}

bool FastPlannerSmoother::buildObstacleList(
    const std::string &target_frame,
    std::vector<Planner::ObstacleInfo> &obstacles) {
  obstacles.clear();

  sensor_msgs::msg::PointCloud2::SharedPtr cloud;
  {
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    cloud = latest_obstacle_cloud_;
  }

  if (!cloud) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 3000,
                         "No obstacle cloud received on %s yet",
                         obstacle_topic_.c_str());
    return false;
  }

  const std::string source_frame = cloud->header.frame_id;
  const bool need_transform =
      !source_frame.empty() && source_frame != target_frame;
  geometry_msgs::msg::TransformStamped transform;

  if (need_transform) {
    try {
      transform =
          tf_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 3000,
                           "Cannot transform obstacle cloud from %s to %s: %s",
                           source_frame.c_str(), target_frame.c_str(),
                           ex.what());
      return false;
    }
  }

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>>
        iter_intensity;

    try {
      iter_intensity =
          std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(
              *cloud, "intensity");
    } catch (const std::runtime_error &) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000,
                           "Obstacle cloud has no intensity field; using xyz "
                           "points without intensity filtering");
    }

    int index = 0;
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++index) {
      double intensity = 0.0;
      if (iter_intensity) {
        intensity = **iter_intensity;
        ++(*iter_intensity);
      }

      if (index % obstacle_sample_step_ != 0) {
        continue;
      }

      if (iter_intensity && (intensity < obstacle_min_intensity_ ||
                             intensity > obstacle_max_intensity_)) {
        continue;
      }

      geometry_msgs::msg::PointStamped source_point;
      source_point.header = cloud->header;
      source_point.point.x = *iter_x;
      source_point.point.y = *iter_y;
      source_point.point.z = *iter_z;

      if (!std::isfinite(source_point.point.x) ||
          !std::isfinite(source_point.point.y) ||
          !std::isfinite(source_point.point.z)) {
        continue;
      }

      geometry_msgs::msg::PointStamped target_point = source_point;
      if (need_transform) {
        tf2::doTransform(source_point, target_point, transform);
      }

      if (target_point.point.z < obstacle_min_z_ ||
          target_point.point.z > obstacle_max_z_) {
        continue;
      }

      Planner::ObstacleInfo obstacle;
      obstacle.x = target_point.point.x;
      obstacle.y = target_point.point.y;
      obstacle.z = target_point.point.z;
      obstacles.push_back(obstacle);

      if (max_obstacle_points_ > 0 &&
          static_cast<int>(obstacles.size()) >= max_obstacle_points_) {
        break;
      }
    }
  } catch (const std::runtime_error &ex) {
    RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 3000,
        "Obstacle cloud cannot be parsed as xyz PointCloud2: %s", ex.what());
    return false;
  }

  return true;
}

void FastPlannerSmoother::overwritePath(
    nav_msgs::msg::Path &path,
    const std::vector<Planner::PathPoint> &planner_result) const {
  path.poses.clear();
  path.poses.reserve(planner_result.size());

  for (size_t i = 0; i < planner_result.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = planner_result[i].x;
    pose.pose.position.y = planner_result[i].y;
    pose.pose.position.z = 0.0;

    double yaw = 0.0;
    if (i + 1 < planner_result.size()) {
      yaw = std::atan2(planner_result[i + 1].y - planner_result[i].y,
                       planner_result[i + 1].x - planner_result[i].x);
    } else if (i > 0) {
      yaw = std::atan2(planner_result[i].y - planner_result[i - 1].y,
                       planner_result[i].x - planner_result[i - 1].x);
    }

    pose.pose.orientation = yawToQuaternion(yaw);
    path.poses.push_back(pose);
  }
}

bool FastPlannerSmoother::configureRcEsdfFootprint(
    const std::vector<double> &footprint_values) {
  rc_esdf_footprint_.clear();

  if (footprint_values.size() < 6 || footprint_values.size() % 2 != 0) {
    RCLCPP_ERROR(logger_, "rc_esdf_footprint must contain at least 3 xy pairs; "
                          "disabling RC-ESDF");
    rc_esdf_map_.reset();
    return false;
  }

  rc_esdf_footprint_.reserve(footprint_values.size() / 2);
  for (size_t i = 0; i + 1 < footprint_values.size(); i += 2) {
    rc_esdf_footprint_.emplace_back(footprint_values[i],
                                    footprint_values[i + 1]);
  }

  rc_esdf_map_ = std::make_unique<RcEsdfMap>();
  rc_esdf_map_->initialize(rc_esdf_map_width_, rc_esdf_map_height_,
                           rc_esdf_resolution_);
  rc_esdf_map_->generateFromPolygon(rc_esdf_footprint_);
  return true;
}

bool FastPlannerSmoother::isPathRcSafe(
    const nav_msgs::msg::Path &path,
    const std::vector<Planner::ObstacleInfo> &obstacles) const {
  if (!rc_esdf_map_ || path.poses.empty() || obstacles.empty()) {
    return true;
  }

  const double query_radius_sq =
      rc_esdf_query_radius_ > 0.0
          ? rc_esdf_query_radius_ * rc_esdf_query_radius_
          : std::numeric_limits<double>::infinity();

  for (size_t pose_id = 0; pose_id < path.poses.size();
       pose_id += static_cast<size_t>(rc_esdf_pose_sample_step_)) {
    const auto &pose = path.poses[pose_id].pose;
    const Eigen::Vector2d center(pose.position.x, pose.position.y);
    const double yaw = getPathYaw(path, pose_id);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    Eigen::Matrix2d body_to_world;
    body_to_world << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;
    const Eigen::Matrix2d world_to_body = body_to_world.transpose();

    int used_obstacles = 0;
    for (size_t obs_id = 0; obs_id < obstacles.size();
         obs_id += static_cast<size_t>(rc_esdf_obstacle_sample_step_)) {
      const Eigen::Vector2d obstacle(obstacles[obs_id].x, obstacles[obs_id].y);
      const Eigen::Vector2d diff_world = obstacle - center;
      if (diff_world.squaredNorm() > query_radius_sq) {
        continue;
      }

      double dist = 0.0;
      Eigen::Vector2d grad;
      if (!rc_esdf_map_->query(world_to_body * diff_world, dist, grad)) {
        continue;
      }

      if (dist < rc_esdf_margin_) {
        RCLCPP_WARN(logger_,
                    "RC-ESDF collision risk: pose=%zu obstacle=%zu "
                    "clearance=%.3f margin=%.3f",
                    pose_id, obs_id, dist, rc_esdf_margin_);
        return false;
      }

      ++used_obstacles;
      if (rc_esdf_max_obstacles_per_pose_ > 0 &&
          used_obstacles >= rc_esdf_max_obstacles_per_pose_) {
        break;
      }
    }
  }

  return true;
}

void FastPlannerSmoother::publishDebugVisualization(
    const nav_msgs::msg::Path &input_path,
    const nav_msgs::msg::Path &optimized_path,
    const std::vector<Planner::ObstacleInfo> &obstacles) {
  if (!debug_visualization_enabled_ || input_path.header.frame_id.empty()) {
    return;
  }

  if (debug_input_path_pub_ && debug_input_path_pub_->is_activated()) {
    auto stamped_path = input_path;
    stamped_path.header.stamp = clock_->now();
    debug_input_path_pub_->publish(stamped_path);
  }

  if (debug_optimized_path_pub_ && debug_optimized_path_pub_->is_activated() &&
      !optimized_path.poses.empty()) {
    auto stamped_path = optimized_path;
    stamped_path.header.stamp = clock_->now();
    debug_optimized_path_pub_->publish(stamped_path);
  }

  std_msgs::msg::Header header = input_path.header;
  header.stamp = clock_->now();
  publishObstacleCloud(header, obstacles);
  publishEsdfGrid(header);
  publishRcEsdfGrid();
  publishRcEsdfFootprint();
}

void FastPlannerSmoother::publishObstacleCloud(
    const std_msgs::msg::Header &header,
    const std::vector<Planner::ObstacleInfo> &obstacles) {
  if (!debug_obstacles_pub_ || !debug_obstacles_pub_->is_activated()) {
    return;
  }

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
      sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
      sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(obstacles.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");
  for (const auto &obstacle : obstacles) {
    *iter_x = obstacle.x;
    *iter_y = obstacle.y;
    *iter_z = obstacle.z;
    *iter_intensity = 1.0f;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_intensity;
  }

  debug_obstacles_pub_->publish(cloud);
}

void FastPlannerSmoother::publishEsdfGrid(const std_msgs::msg::Header &header) {
  if (!planner_ || !debug_esdf_grid_pub_ ||
      !debug_esdf_grid_pub_->is_activated()) {
    return;
  }

  Planner::EsdfSlice slice;
  if (!planner_->getLocalEsdfSlice(slice, debug_esdf_z_)) {
    return;
  }

  nav_msgs::msg::OccupancyGrid grid;
  grid.header = header;
  grid.info.resolution = slice.resolution;
  grid.info.width = static_cast<uint32_t>(slice.width);
  grid.info.height = static_cast<uint32_t>(slice.height);
  grid.info.origin.position.x = slice.origin_x;
  grid.info.origin.position.y = slice.origin_y;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;
  grid.data.resize(slice.samples.size(), 0);

  for (size_t i = 0; i < slice.samples.size(); ++i) {
    const auto &sample = slice.samples[i];
    grid.data[i] =
        sample.occupancy > 0
            ? 100
            : distanceToOccupancy(sample.distance, debug_esdf_max_distance_);
  }

  last_esdf_grid_ = grid;
  has_last_esdf_grid_ = true;
  debug_esdf_grid_pub_->publish(grid);
}

void FastPlannerSmoother::publishCachedDebugMaps() {
  if (!debug_visualization_enabled_) {
    return;
  }

  publishRcEsdfGrid();
  publishRcEsdfFootprint();

  if (has_last_esdf_grid_ && debug_esdf_grid_pub_ &&
      debug_esdf_grid_pub_->is_activated()) {
    last_esdf_grid_.header.stamp = clock_->now();
    debug_esdf_grid_pub_->publish(last_esdf_grid_);
  }
}

void FastPlannerSmoother::publishRcEsdfGrid() {
  if (!rc_esdf_map_ || !debug_rc_esdf_grid_pub_ ||
      !debug_rc_esdf_grid_pub_->is_activated()) {
    return;
  }

  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = clock_->now();
  grid.header.frame_id = debug_rc_esdf_frame_;
  grid.info.resolution = rc_esdf_resolution_;
  grid.info.width = static_cast<uint32_t>(
      std::ceil(rc_esdf_map_width_ / rc_esdf_resolution_));
  grid.info.height = static_cast<uint32_t>(
      std::ceil(rc_esdf_map_height_ / rc_esdf_resolution_));
  grid.info.origin.position.x = -0.5 * rc_esdf_map_width_;
  grid.info.origin.position.y = -0.5 * rc_esdf_map_height_;
  grid.info.origin.orientation.w = 1.0;
  grid.data.resize(static_cast<size_t>(grid.info.width * grid.info.height), 0);

  for (uint32_t y = 0; y < grid.info.height; ++y) {
    for (uint32_t x = 0; x < grid.info.width; ++x) {
      const Eigen::Vector2d pos(
          grid.info.origin.position.x +
              (static_cast<double>(x) + 0.5) * rc_esdf_resolution_,
          grid.info.origin.position.y +
              (static_cast<double>(y) + 0.5) * rc_esdf_resolution_);
      double distance = 0.0;
      Eigen::Vector2d grad;
      if (rc_esdf_map_->query(pos, distance, grad)) {
        grid.data[static_cast<size_t>(y * grid.info.width + x)] =
            distanceToOccupancy(distance, debug_rc_esdf_max_distance_);
      }
    }
  }

  debug_rc_esdf_grid_pub_->publish(grid);
}

void FastPlannerSmoother::publishRcEsdfFootprint() {
  if (rc_esdf_footprint_.empty() || !debug_rc_esdf_footprint_pub_ ||
      !debug_rc_esdf_footprint_pub_->is_activated()) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker footprint;
  footprint.header.stamp = clock_->now();
  footprint.header.frame_id = debug_rc_esdf_frame_;
  footprint.ns = "fast_planner_rc_esdf";
  footprint.id = 0;
  footprint.type = visualization_msgs::msg::Marker::LINE_STRIP;
  footprint.action = visualization_msgs::msg::Marker::ADD;
  footprint.pose.orientation.w = 1.0;
  footprint.scale.x = 0.03;
  footprint.color.r = 1.0;
  footprint.color.g = 0.8;
  footprint.color.b = 0.0;
  footprint.color.a = 1.0;
  footprint.points.reserve(rc_esdf_footprint_.size() + 1);

  for (const auto &vertex : rc_esdf_footprint_) {
    geometry_msgs::msg::Point point;
    point.x = vertex.x();
    point.y = vertex.y();
    point.z = 0.03;
    footprint.points.push_back(point);
  }
  geometry_msgs::msg::Point first_point;
  first_point.x = rc_esdf_footprint_.front().x();
  first_point.y = rc_esdf_footprint_.front().y();
  first_point.z = 0.03;
  footprint.points.push_back(first_point);

  markers.markers.push_back(footprint);
  debug_rc_esdf_footprint_pub_->publish(markers);
}

double FastPlannerSmoother::getPathYaw(const nav_msgs::msg::Path &path,
                                       size_t pose_index) const {
  if (path.poses.size() < 2) {
    return 0.0;
  }

  size_t next_index = pose_index + 1;
  size_t prev_index = pose_index;
  if (next_index >= path.poses.size()) {
    next_index = pose_index;
    prev_index = pose_index - 1;
  }

  const auto &prev = path.poses[prev_index].pose.position;
  const auto &next = path.poses[next_index].pose.position;
  return std::atan2(next.y - prev.y, next.x - prev.x);
}

geometry_msgs::msg::Quaternion
FastPlannerSmoother::yawToQuaternion(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);

  geometry_msgs::msg::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

double FastPlannerSmoother::distance2d(const Planner::PathPoint &a,
                                       const Planner::PathPoint &b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

int FastPlannerSmoother::distanceToOccupancy(double distance,
                                             double max_distance) {
  if (!std::isfinite(distance)) {
    return 0;
  }

  if (distance <= 0.0) {
    return 100;
  }

  const double normalized = std::clamp(distance / max_distance, 0.0, 1.0);
  return static_cast<int>(std::round((1.0 - normalized) * 100.0));
}

} // namespace fast_planner

PLUGINLIB_EXPORT_CLASS(fast_planner::FastPlannerSmoother, nav2_core::Smoother)
