/**
 * @file topo_node.hpp
 * @brief 拓扑图节点定义 - 守卫点和连接点
 */

#ifndef TOPO_PATH_PLANNER_TOPO_NODE_HPP_
#define TOPO_PATH_PLANNER_TOPO_NODE_HPP_

#include <Eigen/Core>
#include <vector>
#include <memory>
#include <map>

namespace topo_path_planner
{

/**
 * @enum NodeType
 * @brief 拓扑图节点类型
 */
enum class NodeType
{
  GUARD,      ///< 守卫点 - 负责探索新区域
  CONNECTION  ///< 连接点 - 连接两个守卫点
};

/**
 * @struct TopoNode
 * @brief 拓扑图节点基类
 */
struct TopoNode
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int id;                              ///< 节点唯一ID
  NodeType type;                       ///< 节点类型
  Eigen::Vector2d position;            ///< 2D位置 (x, y)
  double height;                       ///< 高度信息
  std::vector<int> neighbors;          ///< 相邻节点ID列表
  std::vector<int> unconnected;        ///< 相邻但无法连接的点(如台阶等单向边)

  /**
   * @brief 构造函数
   */
  TopoNode()
    : id(-1), type(NodeType::GUARD), position(0.0, 0.0), height(0.0) {}

  TopoNode(int id, NodeType t, const Eigen::Vector2d& pos, double h = 0.0)
    : id(id), type(t), position(pos), height(h) {}

  /**
   * @brief 添加邻居节点
   */
  void addNeighbor(int neighbor_id)
  {
    neighbors.push_back(neighbor_id);
  }

  /**
   * @brief 添加单向不可连接点
   */
  void addUnconnected(int node_id)
  {
    unconnected.push_back(node_id);
  }

  /**
   * @brief 计算与另一个节点的距离
   */
  double distanceTo(const TopoNode& other) const
  {
    return (position - other.position).norm();
  }
};

/**
 * @struct GuardNode
 * @brief 守卫点 - 可以连接多个连接点
 */
struct GuardNode : public TopoNode
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::vector<int> connection_points;  ///< 该守卫点连接的所有连接点ID

  GuardNode()
    : TopoNode()
  {
    type = NodeType::GUARD;
  }

  GuardNode(int id, const Eigen::Vector2d& pos, double h = 0.0)
    : TopoNode(id, NodeType::GUARD, pos, h) {}
};

/**
 * @struct ConnectionNode
 * @brief 连接点 - 只能连接两个守卫点
 */
struct ConnectionNode : public TopoNode
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int guard1_id;    ///< 第一个守卫点ID
  int guard2_id;    ///< 第二个守卫点ID
  double length;    ///< 连接长度

  ConnectionNode()
    : TopoNode(), guard1_id(-1), guard2_id(-1), length(0.0)
  {
    type = NodeType::CONNECTION;
  }

  ConnectionNode(int id, int g1, int g2, const Eigen::Vector2d& pos, double len = 0.0)
    : TopoNode(id, NodeType::CONNECTION, pos), guard1_id(g1), guard2_id(g2), length(len)
  {
    neighbors = {g1, g2};
  }

  /**
   * @brief 检查是否连接指定的守卫点对
   */
  bool connectsGuards(int g1, int g2) const
  {
    return (guard1_id == g1 && guard2_id == g2) ||
           (guard1_id == g2 && guard2_id == g1);
  }
};

/**
 * @class TopoGraph
 * @brief 拓扑图类 - 管理所有节点和边
 */
class TopoGraph
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<TopoGraph>;

  TopoGraph();
  ~TopoGraph() = default;

  /**
   * @brief 添加守卫点
   * @param pos 位置
   * @param height 高度
   * @return 新创建的守卫点ID
   */
  int addGuard(const Eigen::Vector2d& pos, double height = 0.0);

  /**
   * @brief 添加连接点
   * @param pos 位置
   * @param guard1_id 第一个守卫点ID
   * @param guard2_id 第二个守卫点ID
   * @return 新创建的连接点ID
   */
  int addConnection(const Eigen::Vector2d& pos, int guard1_id, int guard2_id);

  /**
   * @brief 获取节点
   */
  TopoNode* getNode(int id);
  const TopoNode* getNode(int id) const;

  /**
   * @brief 获取守卫点
   */
  GuardNode* getGuard(int id);
  const GuardNode* getGuard(int id) const;

  /**
   * @brief 获取连接点
   */
  ConnectionNode* getConnection(int id);
  const ConnectionNode* getConnection(int id) const;

  /**
   * @brief 获取所有守卫点
   */
  std::vector<GuardNode*> getAllGuards();

  /**
   * @brief 获取所有连接点
   */
  std::vector<ConnectionNode*> getAllConnections();

  /**
   * @brief 检查两个守卫点是否已通过连接点连接
   * @return 如果已连接，返回连接点ID；否则返回-1
   */
  int areGuardsConnected(int guard1_id, int guard2_id) const;

  /**
   * @brief 寻找两个守卫点之间的所有路径
   * @param start_id 起始守卫点ID
   * @param end_id 目标守卫点ID
   * @return 所有路径，每条路径是一系列节点ID
   */
  std::vector<std::vector<int>> findAllPaths(int start_id, int end_id) const;

  /**
   * @brief 使用Dijkstra寻找最短路径
   */
  std::vector<int> findShortestPath(int start_id, int end_id) const;

  /**
   * @brief 清空图
   */
  void clear();

  /**
   * @brief 获取节点数量
   */
  size_t nodeCount() const { return nodes_.size(); }

  /**
   * @brief 获取守卫点数量
   */
  size_t guardCount() const { return guards_.size(); }

  /**
   * @brief 获取连接点数量
   */
  size_t connectionCount() const { return connections_.size(); }

private:
  std::map<int, TopoNode*> nodes_;                          ///< 所有节点（裸指针，不拥有所有权）
  std::map<int, std::unique_ptr<GuardNode>> guards_;        ///< 守卫点（拥有所有权）
  std::map<int, std::unique_ptr<ConnectionNode>> connections_; ///< 连接点（拥有所有权）

  int next_id_;                                              ///< 下一个可用的节点ID

  /**
   * @brief DFS辅助函数，用于寻找所有路径
   */
  void dfsFindPaths(
    int current,
    int end,
    std::vector<int>& path,
    std::vector<bool>& visited,
    std::vector<std::vector<int>>& all_paths) const;
};

}  // namespace topo_path_planner

#endif  // TOPO_PATH_PLANNER_TOPO_NODE_HPP_
