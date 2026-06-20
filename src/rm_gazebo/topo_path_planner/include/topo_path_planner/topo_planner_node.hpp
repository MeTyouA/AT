/**
 * @file topo_planner_node.hpp
 * @brief 拓扑路径规划器ROS2节点
 */

#ifndef TOPO_PATH_PLANNER_TOPO_PLANNER_NODE_HPP_
#define TOPO_PATH_PLANNER_TOPO_PLANNER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

#include "topo_path_planner/topo_graph.hpp"

namespace topo_path_planner
{

/**
 * @class TopoPlannerNode
 * @brief 拓扑路径规划器ROS2节点
 */
class TopoPlannerNode : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数（用于rclcpp_components）
   */
  explicit TopoPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  ~TopoPlannerNode() = default;

private:
  // 核心组件
  TopoPlanner planner_;

  // ROS2相关
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_viz_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr skeleton_pub_;

  rclcpp::TimerBase::SharedPtr viz_timer_;

  // 代价地图
  nav2_costmap_2d::Costmap2D* costmap_;
  bool costmap_owned_;  // 是否由本节点创建
  bool planner_initialized_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 参数
  std::string global_frame_;
  double sample_distance_;
  int max_samples_;
  bool use_skeleton_;
  bool enable_smoothing_;
  int smoothing_iterations_;
  std::string costmap_topic_;
  std::string tf_name_;
  std::string goal_pose_topic_;

  // 可视化数据
  TopoPlanResult last_result_;
  Eigen::Vector2d last_start_;
  Eigen::Vector2d last_goal_;

  /**
   * @brief 声明参数
   */
  void declareParameters();

  /**
   * @brief 从参数服务器读取参数
   */
  void loadParameters();

  /**
   * @brief 目标点回调
   */
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /**
   * @brief 代价地图回调
   */
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  /**
   * @brief 可视化定时器回调
   */
  void visualizationCallback();

  /**
   * @brief 发布路径
   */
  void publishPath(const TopoPath& path);

  /**
   * @brief 发布拓扑图可视化
   */
  void publishGraphVisualization();

  /**
   * @brief 发布骨架图像
   */
  void publishSkeletonImage(const cv::Mat& skeleton);

  /**
   * @brief 创建守卫点标记
   */
  visualization_msgs::msg::Marker createGuardMarker(
    const Eigen::Vector2d& position,
    int id,
    const std::string& frame_id);

  /**
   * @brief 创建连接线标记
   */
  visualization_msgs::msg::Marker createConnectionMarker(
    const Eigen::Vector2d& p1,
    const Eigen::Vector2d& p2,
    int id,
    const std::string& frame_id);

  /**
   * @brief 创建路径标记
   */
  visualization_msgs::msg::Marker createPathMarker(
    const std::vector<Eigen::Vector2d>& points,
    int id,
    const std::string& ns,
    const std::array<float, 3>& color,
    const std::string& frame_id);

  /**
   * @brief 将TopoPath转换为nav_msgs::msg::Path
   */
  nav_msgs::msg::Path toNavPath(const TopoPath& topo_path);

  /**
   * @brief 获取机器人当前位置
   */
  bool getRobotPose(Eigen::Vector2d& pose);
};

}  // namespace topo_path_planner

#endif  // TOPO_PATH_PLANNER_TOPO_PLANNER_NODE_HPP_
