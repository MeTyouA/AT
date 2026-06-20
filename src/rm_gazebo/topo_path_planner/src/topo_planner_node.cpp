/**
 * @file topo_planner_node.cpp
 * @brief 拓扑路径规划器ROS2节点实现
 */

#include "topo_path_planner/topo_planner_node.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace topo_path_planner
{

TopoPlannerNode::TopoPlannerNode(const rclcpp::NodeOptions& options)
  : Node("topo_planner_node", options),
    costmap_(nullptr),
    costmap_owned_(false),
    planner_initialized_(false),
    global_frame_("map"),
    sample_distance_(0.5),
    max_samples_(500),
    use_skeleton_(true),
    enable_smoothing_(true),
    smoothing_iterations_(5)
{
  declareParameters();
  loadParameters();

  // 初始化 TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
    *tf_buffer_, this, false);

  // 订阅代价地图话题
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_topic_,
    rclcpp::QoS(10).reliable().transient_local(),
    std::bind(&TopoPlannerNode::costmapCallback, this, std::placeholders::_1)
  );

  // 创建目标点订阅者
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_pose_topic_,
    rclcpp::QoS(10),
    std::bind(&TopoPlannerNode::goalCallback, this, std::placeholders::_1)
  );

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "/topo_plan_path",
    rclcpp::QoS(10)
  );

  graph_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/topo_graph_visualization",
    rclcpp::QoS(10)
  );

  skeleton_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    "/topo_skeleton_image",
    rclcpp::QoS(10)
  );

  // 可视化定时器
  viz_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&TopoPlannerNode::visualizationCallback, this)
  );

  // 初始化规划器参数
  planner_.setSampleDistance(sample_distance_);
  planner_.setUseSkeleton(use_skeleton_);
  planner_.setMaxSamples(max_samples_);
  planner_.setSmoothing(enable_smoothing_);

  RCLCPP_INFO(this->get_logger(), "TopoPlannerNode initialized, waiting for costmap...");
}

void TopoPlannerNode::declareParameters()
{
  this->declare_parameter("global_frame", "map");
  this->declare_parameter("sample_distance", 0.5);
  this->declare_parameter("max_samples", 500);
  this->declare_parameter("use_skeleton", true);
  this->declare_parameter("enable_smoothing", true);
  this->declare_parameter("smoothing_iterations", 5);
  this->declare_parameter("costmap_topic", "global_costmap/global_costmap");
  this->declare_parameter("tf_name", "gimbal_yaw_fake");
  this->declare_parameter("goal_pose_topic", "goal_pose");
}

void TopoPlannerNode::loadParameters()
{
  this->get_parameter("global_frame", global_frame_);
  this->get_parameter("sample_distance", sample_distance_);
  this->get_parameter("max_samples", max_samples_);
  this->get_parameter("use_skeleton", use_skeleton_);
  this->get_parameter("enable_smoothing", enable_smoothing_);
  this->get_parameter("smoothing_iterations", smoothing_iterations_);
  this->get_parameter("costmap_topic", costmap_topic_);
  this->get_parameter("tf_name", tf_name_);
  this->get_parameter("goal_pose_topic", goal_pose_topic_);


  RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
  RCLCPP_INFO(this->get_logger(), "  global_frame: %s", global_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "  sample_distance: %.2f", sample_distance_);
  RCLCPP_INFO(this->get_logger(), "  max_samples: %d", max_samples_);
  RCLCPP_INFO(this->get_logger(), "  use_skeleton: %s", use_skeleton_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  enable_smoothing: %s", enable_smoothing_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  smoothing_iterations: %d", smoothing_iterations_);
  RCLCPP_INFO(this->get_logger(), "  costmap_topic: %s", costmap_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  tf_name: %s", tf_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "  goal_pose_topic: %s", goal_pose_topic_.c_str());
}

void TopoPlannerNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // 如果已经有 costmap 且不是自己拥有的，就不处理（使用外部提供的）
  if (costmap_ && !costmap_owned_) {
    return;
  }

  // 删除旧的 costmap（如果有）
  if (costmap_ && costmap_owned_) {
    delete costmap_;
  }

  // 从 OccupancyGrid 创建 Costmap2D
  costmap_ = new nav2_costmap_2d::Costmap2D(
    msg->info.width,
    msg->info.height,
    msg->info.resolution,
    msg->info.origin.position.x,
    msg->info.origin.position.y
  );

  // 复制代价数据
  for (size_t i = 0; i < msg->data.size(); ++i) {
    unsigned char cost = static_cast<unsigned char>(msg->data[i]);
    // 将 OccupancyGrid 值 (0-100, -1) 转换为 costmap 值 (0-254)
    if (msg->data[i] == -1) {
      cost = nav2_costmap_2d::NO_INFORMATION;
    } else if (msg->data[i] == 0) {
      cost = nav2_costmap_2d::FREE_SPACE;
    } else {
      cost = static_cast<unsigned char>(msg->data[i] * 254 / 100);
    }
    size_t x = i % msg->info.width;
    size_t y = (msg->info.height - 1) - (i / msg->info.width);  // 翻转 y 轴
    costmap_->setCost(x, y, cost);
  }

  costmap_owned_ = true;

  // 初始化规划器
  if (!planner_initialized_ && costmap_) {
    planner_.initialize(costmap_);
    planner_initialized_ = true;
    RCLCPP_INFO(this->get_logger(),
      "Planner initialized with costmap: %dx%d, resolution: %.3f",
      costmap_->getSizeInCellsX(),
      costmap_->getSizeInCellsY(),
      costmap_->getResolution());
  }
}

void TopoPlannerNode::goalCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received goal: (%.2f, %.2f)",
    msg->pose.position.x, msg->pose.position.y);

  // 检查规划器是否已初始化
  if (!planner_initialized_) {
    RCLCPP_ERROR(this->get_logger(),
      "Planner not initialized: waiting for costmap on topic '%s'",
      costmap_topic_.c_str());
    return;
  }

  // 获取机器人当前位置
  Eigen::Vector2d start;
  if (!getRobotPose(start)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get robot pose");
    return;
  }

  Eigen::Vector2d goal(msg->pose.position.x, msg->pose.position.y);
  last_start_ = start;
  last_goal_ = goal;

  // 执行规划
  RCLCPP_INFO(this->get_logger(), "Planning from (%.2f, %.2f) to (%.2f, %.2f)...",
    start.x(), start.y(), goal.x(), goal.y());

  last_result_ = planner_.plan(start, goal, 5);

  if (last_result_.success) {
    RCLCPP_INFO(this->get_logger(), "%s", last_result_.message.c_str());
    RCLCPP_INFO(this->get_logger(), "Best path length: %.2f, cost: %.2f",
      last_result_.best_path.length, last_result_.best_path.cost);

    // 发布最优路径
    publishPath(last_result_.best_path);

    // 发布拓扑图可视化
    publishGraphVisualization();
  } else {
    RCLCPP_ERROR(this->get_logger(), "Planning failed: %s",
      last_result_.message.c_str());
  }
}

void TopoPlannerNode::visualizationCallback()
{
  // 定期发布可视化信息
  if (last_result_.success) {
    publishGraphVisualization();
  }
}

void TopoPlannerNode::publishPath(const TopoPath& path)
{
  nav_msgs::msg::Path nav_path = toNavPath(path);
  path_pub_->publish(nav_path);
}

void TopoPlannerNode::publishGraphVisualization()
{
  visualization_msgs::msg::MarkerArray markers;

  auto graph = planner_.getGraph();
  if (!graph || graph->nodeCount() == 0) {
    return;
  }

  int marker_id = 0;

  // 发布守卫点
  for (auto* guard : graph->getAllGuards()) {
    auto marker = createGuardMarker(guard->position, guard->id, global_frame_);
    markers.markers.push_back(marker);
    ++marker_id;
  }

  // 发布连接点
  for (auto* conn : graph->getAllConnections()) {
    auto* g1 = graph->getGuard(conn->guard1_id);
    auto* g2 = graph->getGuard(conn->guard2_id);

    if (g1 && g2) {
      auto marker = createConnectionMarker(
        g1->position, g2->position, conn->id, global_frame_);
      markers.markers.push_back(marker);
      ++marker_id;
    }
  }

  // 发布找到的所有路径
  for (size_t i = 0; i < last_result_.paths.size(); ++i) {
    std::array<float, 3> color = {0.0f, 1.0f, 0.0f};  // 绿色
    if (i == 0) {
      color = {0.0f, 0.0f, 1.0f};  // 最优路径用蓝色
    }

    auto marker = createPathMarker(
      last_result_.paths[i].points,
      marker_id,
      "path_" + std::to_string(i),
      color,
      global_frame_);
    markers.markers.push_back(marker);
    ++marker_id;
  }

  graph_viz_pub_->publish(markers);
}

visualization_msgs::msg::Marker TopoPlannerNode::createGuardMarker(
  const Eigen::Vector2d& position,
  int id,
  const std::string& frame_id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = this->now();
  marker.ns = "guards";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = position.x();
  marker.pose.position.y = position.y();
  marker.pose.position.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 0.8;

  marker.lifetime = rclcpp::Duration::from_seconds(1.0);

  return marker;
}

visualization_msgs::msg::Marker TopoPlannerNode::createConnectionMarker(
  const Eigen::Vector2d& p1,
  const Eigen::Vector2d& p2,
  int id,
  const std::string& frame_id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = this->now();
  marker.ns = "connections";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;

  geometry_msgs::msg::Point pt1, pt2;
  pt1.x = p1.x();
  pt1.y = p1.y();
  pt1.z = 0.0;
  pt2.x = p2.x();
  pt2.y = p2.y();
  pt2.z = 0.0;

  marker.points.push_back(pt1);
  marker.points.push_back(pt2);

  marker.scale.x = 0.05;

  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 0.5;

  marker.lifetime = rclcpp::Duration::from_seconds(1.0);

  return marker;
}

visualization_msgs::msg::Marker TopoPlannerNode::createPathMarker(
  const std::vector<Eigen::Vector2d>& points,
  int id,
  const std::string& ns,
  const std::array<float, 3>& color,
  const std::string& frame_id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = this->now();
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;

  for (const auto& pt : points) {
    geometry_msgs::msg::Point p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = 0.1;
    marker.points.push_back(p);
  }

  marker.scale.x = 0.08;

  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = 0.8;

  marker.lifetime = rclcpp::Duration::from_seconds(1.0);

  return marker;
}

nav_msgs::msg::Path TopoPlannerNode::toNavPath(const TopoPath& topo_path)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = this->now();

  for (const auto& pt : topo_path.points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = this->now();

    pose.pose.position.x = pt.x();
    pose.pose.position.y = pt.y();
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;

    path.poses.push_back(pose);
  }

  return path;
}

bool TopoPlannerNode::getRobotPose(Eigen::Vector2d& pose)
{
  try {
    // 从 global_frame_ (通常是 "map") 到 "gimbal_yaw_fake" 获取变换
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      global_frame_, tf_name_, tf2::TimePointZero,
      std::chrono::milliseconds(100));

    pose.x() = transform.transform.translation.x;
    pose.y() = transform.transform.translation.y;
    return true;
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(),
      "Failed to get robot pose from TF: %s", ex.what());
    return false;
  }
}

}  // namespace topo_path_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(topo_path_planner::TopoPlannerNode)
