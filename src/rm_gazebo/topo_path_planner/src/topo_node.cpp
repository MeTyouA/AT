/**
 * @file topo_node.cpp
 * @brief 拓扑图节点实现
 */

#include "topo_path_planner/topo_node.hpp"
#include <queue>
#include <limits>
#include <algorithm>

namespace topo_path_planner
{

TopoGraph::TopoGraph()
  : next_id_(0)
{
}

int TopoGraph::addGuard(const Eigen::Vector2d& pos, double height)
{
  int id = next_id_++;
  auto guard = std::make_unique<GuardNode>(id, pos, height);
  guards_[id] = std::move(guard);
  nodes_[id] = guards_[id].get();
  return id;
}

int TopoGraph::addConnection(const Eigen::Vector2d& pos, int guard1_id, int guard2_id)
{
  int id = next_id_++;
  auto conn = std::make_unique<ConnectionNode>(id, guard1_id, guard2_id, pos);

  // 计算连接长度
  auto* g1 = getGuard(guard1_id);
  auto* g2 = getGuard(guard2_id);
  if (g1 && g2) {
    conn->length = (g1->position - g2->position).norm();
  }

  connections_[id] = std::move(conn);
  nodes_[id] = connections_[id].get();

  // 更新守卫点的邻居关系
  if (g1) {
    g1->addNeighbor(id);
    g1->connection_points.push_back(id);
  }
  if (g2) {
    g2->addNeighbor(id);
    g2->connection_points.push_back(id);
  }

  return id;
}

TopoNode* TopoGraph::getNode(int id)
{
  auto it = nodes_.find(id);
  return (it != nodes_.end()) ? it->second : nullptr;
}

const TopoNode* TopoGraph::getNode(int id) const
{
  auto it = nodes_.find(id);
  return (it != nodes_.end()) ? it->second : nullptr;
}

GuardNode* TopoGraph::getGuard(int id)
{
  auto it = guards_.find(id);
  return (it != guards_.end()) ? it->second.get() : nullptr;
}

const GuardNode* TopoGraph::getGuard(int id) const
{
  auto it = guards_.find(id);
  return (it != guards_.end()) ? it->second.get() : nullptr;
}

ConnectionNode* TopoGraph::getConnection(int id)
{
  auto it = connections_.find(id);
  return (it != connections_.end()) ? it->second.get() : nullptr;
}

const ConnectionNode* TopoGraph::getConnection(int id) const
{
  auto it = connections_.find(id);
  return (it != connections_.end()) ? it->second.get() : nullptr;
}

std::vector<GuardNode*> TopoGraph::getAllGuards()
{
  std::vector<GuardNode*> result;
  result.reserve(guards_.size());
  for (auto& pair : guards_) {
    result.push_back(pair.second.get());
  }
  return result;
}

std::vector<ConnectionNode*> TopoGraph::getAllConnections()
{
  std::vector<ConnectionNode*> result;
  result.reserve(connections_.size());
  for (auto& pair : connections_) {
    result.push_back(pair.second.get());
  }
  return result;
}

int TopoGraph::areGuardsConnected(int guard1_id, int guard2_id) const
{
  for (const auto& pair : connections_) {
    if (pair.second->connectsGuards(guard1_id, guard2_id)) {
      return pair.first;
    }
  }
  return -1;
}

std::vector<std::vector<int>> TopoGraph::findAllPaths(int start_id, int end_id) const
{
  std::vector<std::vector<int>> all_paths;
  std::vector<int> path;
  std::vector<bool> visited(next_id_, false);

  dfsFindPaths(start_id, end_id, path, visited, all_paths);

  return all_paths;
}

void TopoGraph::dfsFindPaths(
  int current,
  int end,
  std::vector<int>& path,
  std::vector<bool>& visited,
  std::vector<std::vector<int>>& all_paths) const
{
  path.push_back(current);
  visited[current] = true;

  if (current == end) {
    all_paths.push_back(path);
  } else {
    const TopoNode* node = getNode(current);
    if (node) {
      for (int neighbor : node->neighbors) {
        if (!visited[neighbor]) {
          dfsFindPaths(neighbor, end, path, visited, all_paths);
        }
      }
    }
  }

  path.pop_back();
  visited[current] = false;
}

std::vector<int> TopoGraph::findShortestPath(int start_id, int end_id) const
{
  if (start_id == end_id) {
    return {start_id};
  }

  // Dijkstra算法
  const double INF = std::numeric_limits<double>::infinity();
  std::map<int, double> dist;
  std::map<int, int> prev;
  std::priority_queue<
    std::pair<double, int>,
    std::vector<std::pair<double, int>>,
    std::greater<>
  > pq;

  // 初始化
  for (const auto& pair : nodes_) {
    dist[pair.first] = INF;
  }
  dist[start_id] = 0.0;
  pq.push({0.0, start_id});

  while (!pq.empty()) {
    auto [d, u] = pq.top();
    pq.pop();

    if (d > dist[u]) continue;
    if (u == end_id) break;

    const TopoNode* node = getNode(u);
    if (!node) continue;

    for (int v : node->neighbors) {
      const TopoNode* neighbor = getNode(v);
      if (!neighbor) continue;

      double edge_weight = node->distanceTo(*neighbor);
      if (dist[u] + edge_weight < dist[v]) {
        dist[v] = dist[u] + edge_weight;
        prev[v] = u;
        pq.push({dist[v], v});
      }
    }
  }

  // 重建路径
  std::vector<int> path;
  if (dist[end_id] == INF) {
    return path;  // 没有路径
  }

  for (int v = end_id; v != start_id; v = prev[v]) {
    path.push_back(v);
  }
  path.push_back(start_id);
  std::reverse(path.begin(), path.end());

  return path;
}

void TopoGraph::clear()
{
  nodes_.clear();
  guards_.clear();
  connections_.clear();
  next_id_ = 0;
}

}  // namespace topo_path_planner
