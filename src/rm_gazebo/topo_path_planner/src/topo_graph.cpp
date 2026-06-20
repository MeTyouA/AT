/**
 * @file topo_graph.cpp
 * @brief 拓扑路径规划器核心实现
 */

#include "topo_path_planner/topo_graph.hpp"
#include <algorithm>
#include <limits>
#include <cmath>

namespace topo_path_planner
{

// ============================================================================
// TopoGraphBuilder
// ============================================================================

TopoGraphBuilder::TopoGraphBuilder()
  : graph_(std::make_shared<TopoGraph>()),
    visibility_checker_(nullptr),
    sample_distance_(0.5),
    max_samples_(500),
    use_skeleton_(true),
    rng_(std::random_device{}())
{
}

void TopoGraphBuilder::setCostmap(nav2_costmap_2d::Costmap2D* costmap)
{
  visibility_checker_.setCostmap(costmap);
}

bool TopoGraphBuilder::buildGraph(
  const Eigen::Vector2d& start,
  const Eigen::Vector2d& goal)
{
  if (!visibility_checker_.isInMap(start) || !visibility_checker_.isInMap(goal)) {
    return false;
  }

  // 清空旧图
  graph_->clear();

  // 添加起点和终点作为守卫点
  int start_id = graph_->addGuard(start);
  int goal_id = graph_->addGuard(goal);

  // 提取骨架点（如果启用）
  if (use_skeleton_) {
    skeleton_points_ = sampleFromSkeleton(start, goal);
  } else {
    skeleton_points_ = sampleFromFreeSpace(start, goal);
  }

  // 处理所有采样点
  for (const auto& point : skeleton_points_) {
    processSamplePoint(point);
  }

  // 尝试直接连接起点和终点
  if (visibility_checker_.isVisible(start, goal)) {
    graph_->addConnection((start + goal) / 2, start_id, goal_id);
  }

  return graph_->guardCount() >= 2;
}

std::vector<Eigen::Vector2d> TopoGraphBuilder::sampleFromSkeleton(
  const Eigen::Vector2d& start,
  const Eigen::Vector2d& goal)
{
  // 获取代价地图
  auto* costmap = visibility_checker_.getCostmap();
  if (!costmap) {
    return {};
  }

  // 提取骨架
  SkeletonExtractor extractor;
  std::vector<Eigen::Vector2d> skeleton = extractor.extractSkeleton(costmap);

  // 添加起点和终点
  std::vector<Eigen::Vector2d> samples = {start, goal};

  // 沿骨架采样
  for (const auto& point : skeleton) {
    // 确保采样点在自由空间
    if (visibility_checker_.isFree(point)) {
      samples.push_back(point);
    }
  }

  return samples;
}

std::vector<Eigen::Vector2d> TopoGraphBuilder::sampleFromFreeSpace(
  const Eigen::Vector2d& start,
  const Eigen::Vector2d& goal)
{
  std::vector<Eigen::Vector2d> samples = {start, goal};

  auto* costmap = visibility_checker_.getCostmap();
  if (!costmap) {
    return samples;
  }

  // 获取地图边界
  double origin_x = costmap->getOriginX();
  double origin_y = costmap->getOriginY();
  double size_x = costmap->getSizeInMetersX();
  double size_y = costmap->getSizeInMetersY();

  std::uniform_real_distribution<double> dist_x(origin_x, origin_x + size_x);
  std::uniform_real_distribution<double> dist_y(origin_y, origin_y + size_y);

  // 随机采样
  for (int i = 0; i < max_samples_; ++i) {
    Eigen::Vector2d sample(dist_x(rng_), dist_y(rng_));
    if (visibility_checker_.isFree(sample)) {
      samples.push_back(sample);
    }
  }

  return samples;
}

std::vector<int> TopoGraphBuilder::checkVisibilityToGuards(
  const Eigen::Vector2d& point)
{
  std::vector<int> visible_guards;

  auto guards = graph_->getAllGuards();
  for (auto* guard : guards) {
    if (visibility_checker_.isVisible(point, guard->position)) {
      visible_guards.push_back(guard->id);
    }
  }

  return visible_guards;
}

int TopoGraphBuilder::processSamplePoint(const Eigen::Vector2d& point)
{
  // 检查与所有守卫点的可见性
  std::vector<int> visible_guards = checkVisibilityToGuards(point);

  if (visible_guards.empty()) {
    // 与任何守卫点都不相连，作为新守卫点
    return graph_->addGuard(point);
  }

  if (visible_guards.size() == 1) {
    // 只与一个守卫点相连
    // 检查距离，如果太近可能不需要新节点
    int guard_id = visible_guards[0];
    const auto* guard = graph_->getGuard(guard_id);
    if (guard && (point - guard->position).norm() > sample_distance_ * 0.5) {
      return graph_->addGuard(point);
    }
    return -1;
  }

  // 与两个或多个守卫点可见
  // 检查是否需要创建连接点
  for (size_t i = 0; i < visible_guards.size(); ++i) {
    for (size_t j = i + 1; j < visible_guards.size(); ++j) {
      int g1 = visible_guards[i];
      int g2 = visible_guards[j];

      // 检查是否已经连接
      int existing_conn = graph_->areGuardsConnected(g1, g2);
      if (existing_conn < 0) {
        // 未连接，创建新的连接点
        Eigen::Vector2d mid_point = (point + graph_->getGuard(g1)->position +
                                     graph_->getGuard(g2)->position) / 3.0;
        return graph_->addConnection(mid_point, g1, g2);
      }
    }
  }

  return -1;
}

// ============================================================================
// TopoPathFinder
// ============================================================================

TopoPathFinder::TopoPathFinder()
  : costmap_(nullptr),
    enable_smoothing_(true),
    smoothing_iterations_(5)
{
}

void TopoPathFinder::setGraph(TopoGraph::Ptr graph)
{
  graph_ = graph;
}

void TopoPathFinder::setCostmap(nav2_costmap_2d::Costmap2D* costmap)
{
  costmap_ = costmap;
  cost_checker_.setCostmap(costmap);
}

TopoPlanResult TopoPathFinder::findPaths(
  const Eigen::Vector2d& start,
  const Eigen::Vector2d& goal,
  size_t max_paths)
{
  TopoPlanResult result;

  if (!graph_) {
    result.success = false;
    result.message = "Graph not set";
    return result;
  }

  // 找到最接近起点和终点的守卫点
  auto guards = graph_->getAllGuards();
  if (guards.size() < 2) {
    result.success = false;
    result.message = "Not enough guards in graph";
    return result;
  }

  int start_guard = -1, goal_guard = -1;
  double min_start_dist = std::numeric_limits<double>::infinity();
  double min_goal_dist = std::numeric_limits<double>::infinity();

  for (auto* guard : guards) {
    double d_start = (guard->position - start).norm();
    double d_goal = (guard->position - goal).norm();

    if (d_start < min_start_dist) {
      min_start_dist = d_start;
      start_guard = guard->id;
    }
    if (d_goal < min_goal_dist) {
      min_goal_dist = d_goal;
      goal_guard = guard->id;
    }
  }

  // 查找所有路径
  std::vector<std::vector<int>> all_paths = graph_->findAllPaths(start_guard, goal_guard);

  if (all_paths.empty()) {
    result.success = false;
    result.message = "No path found";
    return result;
  }

  // 计算每条路径的代价并排序
  std::vector<std::pair<double, size_t>> path_costs;
  for (size_t i = 0; i < all_paths.size(); ++i) {
    TopoPath path;
    path.node_ids = all_paths[i];
    path.points = interpolatePath(all_paths[i]);
    path.length = 0.0;
    for (size_t j = 1; j < path.points.size(); ++j) {
      path.length += (path.points[j] - path.points[j-1]).norm();
    }
    path.cost = computePathCost(path);
    path_costs.push_back({path.cost, i});
  }

  std::sort(path_costs.begin(), path_costs.end());

  // 返回前max_paths条路径
  size_t num_return = std::min(max_paths, all_paths.size());
  result.paths.resize(num_return);
  for (size_t i = 0; i < num_return; ++i) {
    size_t idx = path_costs[i].second;
    TopoPath path;
    path.node_ids = all_paths[idx];
    path.points = interpolatePath(all_paths[idx]);
    path.length = 0.0;
    for (size_t j = 1; j < path.points.size(); ++j) {
      path.length += (path.points[j] - path.points[j-1]).norm();
    }
    path.cost = path_costs[i].first;

    if (enable_smoothing_) {
      smoothPath(path);
    }

    result.paths[i] = path;
  }

  result.best_path = result.paths[0];
  result.success = true;
  result.message = "Found " + std::to_string(result.paths.size()) + " paths";

  return result;
}

TopoPath TopoPathFinder::findBestPath(
  const Eigen::Vector2d& start,
  const Eigen::Vector2d& goal)
{
  auto result = findPaths(start, goal, 1);
  return result.best_path;
}

double TopoPathFinder::computePathCost(const TopoPath& path)
{
  // 代价 = 路径长度 + 障碍物代价
  double cost = path.length;

  if (costmap_) {
    for (size_t i = 1; i < path.points.size(); ++i) {
      cost += cost_checker_.getLineCost(path.points[i-1], path.points[i]);
    }
  }

  return cost;
}

void TopoPathFinder::smoothPath(TopoPath& path)
{
  if (path.points.size() < 3) return;

  std::vector<Eigen::Vector2d> smoothed = path.points;

  for (int iter = 0; iter < smoothing_iterations_; ++iter) {
    std::vector<Eigen::Vector2d> new_points = smoothed;

    for (size_t i = 1; i < smoothed.size() - 1; ++i) {
      // 计算前后点的平均值
      Eigen::Vector2d avg = (smoothed[i-1] + smoothed[i+1]) / 2.0;

      // 检查平均点是否可见
      if (cost_checker_.isVisible(smoothed[i-1], smoothed[i+1])) {
        new_points[i] = avg;
      }
    }

    smoothed = new_points;
  }

  path.points = smoothed;
}

std::vector<Eigen::Vector2d> TopoPathFinder::interpolatePath(
  const std::vector<int>& node_ids)
{
  std::vector<Eigen::Vector2d> path;

  for (int id : node_ids) {
    const auto* node = graph_->getNode(id);
    if (node) {
      path.push_back(node->position);
    }
  }

  return path;
}

// ============================================================================
// TopoPlanner
// ============================================================================

TopoPlanner::TopoPlanner()
  : initialized_(false)
{
}

bool TopoPlanner::initialize(nav2_costmap_2d::Costmap2D* costmap)
{
  if (!costmap) {
    return false;
  }

  builder_.setCostmap(costmap);
  finder_.setCostmap(costmap);
  finder_.setGraph(builder_.getGraph());

  initialized_ = true;
  return true;
}

TopoPlanResult TopoPlanner::plan(
  const Eigen::Vector2d& start,
  const Eigen::Vector2d& goal,
  size_t max_paths)
{
  if (!initialized_) {
    TopoPlanResult result;
    result.success = false;
    result.message = "Planner not initialized";
    return result;
  }

  // 构建拓扑图
  if (!builder_.buildGraph(start, goal)) {
    TopoPlanResult result;
    result.success = false;
    result.message = "Failed to build topology graph";
    return result;
  }

  // 查找路径
  return finder_.findPaths(start, goal, max_paths);
}

}  // namespace topo_path_planner
