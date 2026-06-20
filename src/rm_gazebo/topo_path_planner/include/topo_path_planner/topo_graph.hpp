/**
 * @file topo_graph.hpp
 * @brief 拓扑路径规划器核心类 - 整合所有模块实现完整的拓扑路径搜索
 */

#ifndef TOPO_PATH_PLANNER_TOPO_GRAPH_HPP_
#define TOPO_PATH_PLANNER_TOPO_GRAPH_HPP_

#include <nav2_costmap_2d/costmap_2d.hpp>
#include "topo_path_planner/topo_node.hpp"
#include "topo_path_planner/visibility_checker.hpp"
#include "topo_path_planner/skeleton_extractor.hpp"
#include <memory>
#include <vector>
#include <random>

namespace topo_path_planner
{

/**
 * @struct TopoPath
 * @brief 拓扑路径结构
 */
struct TopoPath
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::vector<int> node_ids;            ///< 节点ID序列
  std::vector<Eigen::Vector2d> points;  ///< 实际路径点
  double length;                        ///< 路径总长度
  double cost;                          ///< 路径代价

  TopoPath() : length(0.0), cost(0.0) {}

  void clear()
  {
    node_ids.clear();
    points.clear();
    length = 0.0;
    cost = 0.0;
  }
};

/**
 * @struct TopoPlanResult
 * @brief 拓扑规划结果
 */
struct TopoPlanResult
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool success;
  std::string message;
  std::vector<TopoPath> paths;  ///< 找到的所有路径（按代价排序）
  TopoPath best_path;           ///< 最优路径

  TopoPlanResult() : success(false), message("") {}
};

/**
 * @class TopoGraphBuilder
 * @brief 拓扑图构建器 - 实现文档描述的拓扑图构建算法
 */
class TopoGraphBuilder
{
public:
  /**
   * @brief 构造函数
   */
  TopoGraphBuilder();

  ~TopoGraphBuilder() = default;

  /**
   * @brief 设置代价地图
   */
  void setCostmap(nav2_costmap_2d::Costmap2D* costmap);

  /**
   * @brief 构建拓扑图
   * @param start 起始位置
   * @param goal 目标位置
   * @return 构建是否成功
   */
  bool buildGraph(const Eigen::Vector2d& start, const Eigen::Vector2d& goal);

  /**
   * @brief 获取构建的拓扑图
   */
  TopoGraph::Ptr getGraph() { return graph_; }

  /**
   * @brief 获取骨架点
   */
  const std::vector<Eigen::Vector2d>& getSkeletonPoints() const
  {
    return skeleton_points_;
  }

  /**
   * @brief 设置采样参数
   */
  void setSampleDistance(double dist) { sample_distance_ = dist; }
  void setMaxSamples(int max_samples) { max_samples_ = max_samples; }

  /**
   * @brief 设置是否使用骨架提取
   */
  void setUseSkeleton(bool use) { use_skeleton_ = use; }

private:
  TopoGraph::Ptr graph_;                  ///< 拓扑图
  VisibilityChecker visibility_checker_;  ///< 可见性检查器
  SkeletonExtractor skeleton_extractor_;  ///< 骨架提取器

  std::vector<Eigen::Vector2d> skeleton_points_;  ///< 骨架点集

  // 采样参数
  double sample_distance_;    ///< 采样点间距
  int max_samples_;           ///< 最大采样数
  bool use_skeleton_;         ///< 是否使用骨架提取

  std::mt19937 rng_;          ///< 随机数生成器

  /**
   * @brief 在骨架点上采样
   */
  std::vector<Eigen::Vector2d> sampleFromSkeleton(
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& goal);

  /**
   * @brief 在自由空间随机采样
   */
  std::vector<Eigen::Vector2d> sampleFromFreeSpace(
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& goal);

  /**
   * @brief 添加采样点到拓扑图
   */
  void addSampleToGraph(const Eigen::Vector2d& sample_point);

  /**
   * @brief 检查采样点与所有守卫点的可见性
   * @return 可见的守卫点ID列表
   */
  std::vector<int> checkVisibilityToGuards(const Eigen::Vector2d& point);

  /**
   * @brief 处理采样点（核心算法逻辑）
   * @return 添加的节点ID，-1表示未添加
   */
  int processSamplePoint(const Eigen::Vector2d& point);
};

/**
 * @class TopoPathFinder
 * @brief 拓扑路径查找器 - 使用构建好的拓扑图查找路径
 */
class TopoPathFinder
{
public:
  /**
   * @brief 构造函数
   */
  TopoPathFinder();

  ~TopoPathFinder() = default;

  /**
   * @brief 设置拓扑图
   */
  void setGraph(TopoGraph::Ptr graph);

  /**
   * @brief 设置代价地图（用于计算路径代价）
   */
  void setCostmap(nav2_costmap_2d::Costmap2D* costmap);

  /**
   * @brief 查找从起点到终点的所有路径
   * @param start 起始位置
   * @param goal 目标位置
   * @param max_paths 最大返回路径数
   * @return 规划结果
   */
  TopoPlanResult findPaths(
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& goal,
    size_t max_paths = 5);

  /**
   * @brief 只查找最优路径
   */
  TopoPath findBestPath(
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& goal);

  /**
   * @brief 设置路径优化参数
   */
  void setSmoothing(bool enable) { enable_smoothing_ = enable; }
  void setSmoothingIterations(int iter) { smoothing_iterations_ = iter; }

private:
  TopoGraph::Ptr graph_;              ///< 拓扑图
  nav2_costmap_2d::Costmap2D* costmap_;  ///< 代价地图
  VisibilityChecker cost_checker_;    ///< 路径代价检查器

  bool enable_smoothing_;             ///< 是否启用路径平滑
  int smoothing_iterations_;          ///< 平滑迭代次数

  /**
   * @brief 计算路径代价
   */
  double computePathCost(const TopoPath& path);

  /**
   * @brief 平滑路径
   */
  void smoothPath(TopoPath& path);

  /**
   * @brief 路径插值生成密集点
   */
  std::vector<Eigen::Vector2d> interpolatePath(
    const std::vector<int>& node_ids);
};

/**
 * @class TopoPlanner
 * @brief 完整的拓扑规划器 - 整合构建和搜索
 */
class TopoPlanner
{
public:
  /**
   * @brief 构造函数
   */
  TopoPlanner();

  ~TopoPlanner() = default;

  /**
   * @brief 初始化规划器
   */
  bool initialize(nav2_costmap_2d::Costmap2D* costmap);

  /**
   * @brief 规划从起点到终点的路径
   * @param start 起点
   * @param goal 终点
   * @param max_paths 最大返回路径数
   * @return 规划结果
   */
  TopoPlanResult plan(
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& goal,
    size_t max_paths = 5);

  /**
   * @brief 获取当前拓扑图
   */
  TopoGraph::Ptr getGraph() { return builder_.getGraph(); }

  /**
   * @brief 获取骨架点
   */
  const std::vector<Eigen::Vector2d>& getSkeletonPoints() const
  {
    return builder_.getSkeletonPoints();
  }

  /**
   * @brief 配置参数
   */
  void setSampleDistance(double dist) { builder_.setSampleDistance(dist); }
  void setUseSkeleton(bool use) { builder_.setUseSkeleton(use); }
  void setMaxSamples(int max) { builder_.setMaxSamples(max); }
  void setSmoothing(bool enable) { finder_.setSmoothing(enable); }

private:
  TopoGraphBuilder builder_;   ///< 拓扑图构建器
  TopoPathFinder finder_;      ///< 路径查找器
  bool initialized_;           ///< 初始化标志
};

}  // namespace topo_path_planner

#endif  // TOPO_PATH_PLANNER_TOPO_GRAPH_HPP_
