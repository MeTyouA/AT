/**
 * @file visibility_checker.cpp
 * @brief 可见性判断模块实现
 */

#include "topo_path_planner/visibility_checker.hpp"
#include <cmath>
#include <algorithm>

namespace topo_path_planner
{

VisibilityChecker::VisibilityChecker()
  : costmap_(nullptr),
    max_height_diff_(0.3),  // 默认最大高差0.3m
    step_size_(0.05)        // 默认步进5cm
{
}

VisibilityChecker::VisibilityChecker(nav2_costmap_2d::Costmap2D* costmap)
  : costmap_(costmap),
    max_height_diff_(0.3),  // 默认最大高差0.3m
    step_size_(0.05)        // 默认步进5cm
{
}

void VisibilityChecker::setCostmap(nav2_costmap_2d::Costmap2D* costmap)
{
  costmap_ = costmap;
}

bool VisibilityChecker::isVisible(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) const
{
  // 检查两点是否都在地图内
  if (!isInMap(p1) || !isInMap(p2)) {
    return false;
  }

  // 检查起点和终点是否是自由空间
  if (!isFree(p1) || !isFree(p2)) {
    return false;
  }

  // 步进采样检查
  return checkLineOfSight(p1, p2);
}

bool VisibilityChecker::isVisibleWithHeight(
  const Eigen::Vector3d& p1,
  const Eigen::Vector3d& p2) const
{
  Eigen::Vector2d pos1 = p1.head<2>();
  Eigen::Vector2d pos2 = p2.head<2>();

  // 先检查2D可见性
  if (!isVisible(pos1, pos2)) {
    return false;
  }

  // 再检查高差
  return checkHeightDifference(p1.z(), p2.z());
}

bool VisibilityChecker::checkLineOfSight(
  const Eigen::Vector2d& p1,
  const Eigen::Vector2d& p2) const
{
  if (!costmap_) {
    return false;
  }

  // 计算两点距离
  double dist = (p2 - p1).norm();
  if (dist < 1e-6) {
    return true;  // 两点重合
  }

  // 计算采样点数
  int num_steps = static_cast<int>(std::ceil(dist / step_size_));

  // 步进采样检查
  for (int i = 1; i < num_steps; ++i) {
    double t = static_cast<double>(i) / num_steps;
    Eigen::Vector2d sample_point = interpolate(p1, p2, t);

    if (!isFree(sample_point)) {
      return false;  // 发现障碍物
    }
  }

  return true;
}

bool VisibilityChecker::checkHeightDifference(double h1, double h2) const
{
  return std::abs(h1 - h2) <= max_height_diff_;
}

double VisibilityChecker::getLineCost(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) const
{
  if (!costmap_) {
    return std::numeric_limits<double>::infinity();
  }

  double dist = (p2 - p1).norm();
  if (dist < 1e-6) {
    return 0.0;
  }

  int num_steps = static_cast<int>(std::ceil(dist / step_size_));
  double total_cost = 0.0;

  for (int i = 0; i <= num_steps; ++i) {
    double t = (num_steps > 0) ? static_cast<double>(i) / num_steps : 0.0;
    Eigen::Vector2d sample_point = interpolate(p1, p2, t);

    unsigned int mx, my;
    if (worldToMap(sample_point, mx, my)) {
      unsigned char cost = costmap_->getCost(mx, my);
      // 根据代价类型累加
      if (cost >= 253) {  // LETHAL_OBSTACLE (254) or INSCRIBED_INFLATED_OBSTACLE (253)
        if (cost == 254) {
          return std::numeric_limits<double>::infinity();
        }
        total_cost += 100.0;
      } else if (cost == 0) {  // FREE_SPACE
        total_cost += 0.0;
      } else {
        total_cost += static_cast<double>(cost);
      }
    } else {
      // 点在地图外
      total_cost += 200.0;
    }
  }

  return total_cost / (num_steps + 1);
}

bool VisibilityChecker::isFree(const Eigen::Vector2d& point) const
{
  if (!costmap_) {
    return false;
  }

  unsigned int mx, my;
  if (!worldToMap(point, mx, my)) {
    return false;
  }

  unsigned char cost = costmap_->getCost(mx, my);
  return cost < 253;  // INSCRIBED_INFLATED_OBSTACLE = 253
}

bool VisibilityChecker::isInMap(const Eigen::Vector2d& point) const
{
  if (!costmap_) {
    return false;
  }

  unsigned int mx, my;
  return worldToMap(point, mx, my);
}

bool VisibilityChecker::worldToMap(
  const Eigen::Vector2d& world,
  unsigned int& mx,
  unsigned int& my) const
{
  if (!costmap_) {
    return false;
  }

  return costmap_->worldToMap(world.x(), world.y(), mx, my);
}

Eigen::Vector2d VisibilityChecker::interpolate(
  const Eigen::Vector2d& p1,
  const Eigen::Vector2d& p2,
  double t) const
{
  return p1 + t * (p2 - p1);
}

}  // namespace topo_path_planner
