/**
 * @file visibility_checker.hpp
 * @brief 可见性判断模块 - 判断两点之间是否可见（无障碍物）
 */

#ifndef TOPO_PATH_PLANNER_VISIBILITY_CHECKER_HPP_
#define TOPO_PATH_PLANNER_VISIBILITY_CHECKER_HPP_

#include <Eigen/Core>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <memory>
#include <vector>

namespace topo_path_planner
{

/**
 * @class VisibilityChecker
 * @brief 可见性判断器 - 检查两点间是否有障碍物和高差
 */
class VisibilityChecker
{
public:
  /**
   * @brief 默认构造函数
   */
  VisibilityChecker();

  /**
   * @brief 构造函数
   * @param costmap 代价地图指针
   */
  explicit VisibilityChecker(nav2_costmap_2d::Costmap2D* costmap);

  ~VisibilityChecker() = default;

  /**
   * @brief 设置代价地图
   */
  void setCostmap(nav2_costmap_2d::Costmap2D* costmap);

  /**
   * @brief 获取代价地图
   */
  nav2_costmap_2d::Costmap2D* getCostmap() { return costmap_; }

  /**
   * @brief 检查两点之间是否可见
   * @param p1 第一个点
   * @param p2 第二个点
   * @return 如果可见返回true
   */
  bool isVisible(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) const;

  /**
   * @brief 检查两个带高程的点之间是否可见
   * @param p1 第一个点 (x, y, height)
   * @param p2 第二个点 (x, y, height)
   * @return 如果可见返回true
   */
  bool isVisibleWithHeight(
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2) const;

  /**
   * @brief 设置最大高差阈值
   */
  void setMaxHeightDiff(double max_diff) { max_height_diff_ = max_diff; }

  /**
   * @brief 设置步进采样距离
   */
  void setStepSize(double step) { step_size_ = step; }

  /**
   * @brief 获取两点之间的障碍物代价
   */
  double getLineCost(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) const;

  /**
   * @brief 检查点是否在自由空间
   */
  bool isFree(const Eigen::Vector2d& point) const;

  /**
   * @brief 检查点是否在地图范围内
   */
  bool isInMap(const Eigen::Vector2d& point) const;

private:
  nav2_costmap_2d::Costmap2D* costmap_;  ///< 代价地图
  double max_height_diff_;                ///< 最大允许高差
  double step_size_;                      ///< 步进采样距离

  /**
   * @brief 步进采样检查两点间是否有障碍
   * @param p1 起点
   * @param p2 终点
   * @return 如果无障碍返回true
   */
  bool checkLineOfSight(
    const Eigen::Vector2d& p1,
    const Eigen::Vector2d& p2) const;

  /**
   * @brief 检查高差是否在允许范围内
   */
  bool checkHeightDifference(double h1, double h2) const;

  /**
   * @brief 世界坐标转地图坐标
   */
  bool worldToMap(const Eigen::Vector2d& world, unsigned int& mx, unsigned int& my) const;

  /**
   * @brief 线性插值获取点
   */
  Eigen::Vector2d interpolate(
    const Eigen::Vector2d& p1,
    const Eigen::Vector2d& p2,
    double t) const;
};

}  // namespace topo_path_planner

#endif  // TOPO_PATH_PLANNER_VISIBILITY_CHECKER_HPP_
