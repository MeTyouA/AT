/**
 * @file test_topo.cpp
 * @brief 拓扑路径规划器测试程序
 */

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include "topo_path_planner/topo_graph.hpp"

using namespace topo_path_planner;

/**
 * @brief 创建一个简单的测试代价地图
 */
nav2_costmap_2d::Costmap2D* createTestCostmap()
{
  // 创建一个10m x 10m的地图，分辨率5cm
  auto* costmap = new nav2_costmap_2d::Costmap2D(
    200, 200,      // cells_x, cells_y
    0.05,          // resolution
    0.0, 0.0       // origin_x, origin_y
  );

  // 设置边界为障碍物
  for (unsigned int i = 0; i < 200; ++i) {
    costmap->setCost(0, i, 254);
    costmap->setCost(199, i, 254);
    costmap->setCost(i, 0, 254);
    costmap->setCost(i, 199, 254);
  }

  // 在中间设置一些障碍物形成迷宫
  for (int i = 50; i < 150; ++i) {
    costmap->setCost(100, i, 254);
  }

  // 留一个缺口
  for (int i = 90; i < 110; ++i) {
    costmap->setCost(100, i, 0);
  }

  return costmap;
}

/**
 * @brief 测试拓扑图构建
 */
void testTopoGraphBuilding()
{
  RCLCPP_INFO(rclcpp::get_logger("test"), "=== Testing TopoGraph Building ===");

  auto* costmap = createTestCostmap();

  TopoPlanner planner;
  planner.initialize(costmap);
  planner.setUseSkeleton(true);

  // 测试点：起点在左下角，终点在右上角
  Eigen::Vector2d start(1.0, 1.0);
  Eigen::Vector2d goal(9.0, 9.0);

  RCLCPP_INFO(rclcpp::get_logger("test"), "Planning from (%.2f, %.2f) to (%.2f, %.2f)",
    start.x(), start.y(), goal.x(), goal.y());

  auto result = planner.plan(start, goal, 3);

  if (result.success) {
    RCLCPP_INFO(rclcpp::get_logger("test"), "Planning SUCCESS!");
    RCLCPP_INFO(rclcpp::get_logger("test"), "  Found %zu paths", result.paths.size());
    RCLCPP_INFO(rclcpp::get_logger("test"), "  Best path length: %.2f", result.best_path.length);
    RCLCPP_INFO(rclcpp::get_logger("test"), "  Best path cost: %.2f", result.best_path.cost);
    RCLCPP_INFO(rclcpp::get_logger("test"), "  Graph has %zu guards, %zu connections",
      planner.getGraph()->guardCount(),
      planner.getGraph()->connectionCount());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("test"), "Planning FAILED: %s", result.message.c_str());
  }

  delete costmap;
}

/**
 * @brief 测试可见性判断
 */
void testVisibilityChecker()
{
  RCLCPP_INFO(rclcpp::get_logger("test"), "=== Testing VisibilityChecker ===");

  auto* costmap = createTestCostmap();
  VisibilityChecker checker(costmap);

  // 测试两点可见性
  Eigen::Vector2d p1(1.0, 1.0);
  Eigen::Vector2d p2(3.0, 3.0);

  bool visible = checker.isVisible(p1, p2);
  RCLCPP_INFO(rclcpp::get_logger("test"), "  (%.2f, %.2f) -> (%.2f, %.2f) visible: %s",
    p1.x(), p1.y(), p2.x(), p2.y(), visible ? "true" : "false");

  // 测试被障碍物阻挡的两点
  Eigen::Vector2d p3(1.0, 5.0);
  Eigen::Vector2d p4(9.0, 5.0);

  visible = checker.isVisible(p3, p4);
  RCLCPP_INFO(rclcpp::get_logger("test"), "  (%.2f, %.2f) -> (%.2f, %.2f) visible: %s",
    p3.x(), p3.y(), p4.x(), p4.y(), visible ? "true" : "false");

  delete costmap;
}

/**
 * @brief 测试拓扑图数据结构
 */
void testTopoGraph()
{
  RCLCPP_INFO(rclcpp::get_logger("test"), "=== Testing TopoGraph ===");

  TopoGraph graph;

  // 添加守卫点
  int g1 = graph.addGuard(Eigen::Vector2d(0.0, 0.0));
  int g2 = graph.addGuard(Eigen::Vector2d(5.0, 0.0));
  int g3 = graph.addGuard(Eigen::Vector2d(5.0, 5.0));
  int g4 = graph.addGuard(Eigen::Vector2d(0.0, 5.0));

  RCLCPP_INFO(rclcpp::get_logger("test"), "  Added %zu guards", graph.guardCount());

  // 添加连接点
  int c1 = graph.addConnection(Eigen::Vector2d(2.5, 0.0), g1, g2);
  int c2 = graph.addConnection(Eigen::Vector2d(5.0, 2.5), g2, g3);
  int c3 = graph.addConnection(Eigen::Vector2d(2.5, 5.0), g3, g4);
  int c4 = graph.addConnection(Eigen::Vector2d(0.0, 2.5), g4, g1);

  RCLCPP_INFO(rclcpp::get_logger("test"), "  Added %zu connections", graph.connectionCount());

  // 检查守卫点连接
  int conn_id = graph.areGuardsConnected(g1, g2);
  RCLCPP_INFO(rclcpp::get_logger("test"), "  Guards %d and %d connected: %s",
    g1, g2, conn_id >= 0 ? "true" : "false");

  // 查找路径
  auto path = graph.findShortestPath(g1, g3);
  RCLCPP_INFO(rclcpp::get_logger("test"), "  Shortest path from guard %d to %d has %zu nodes",
    g1, g3, path.size());
}

/**
 * @brief 主函数
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("test"), "Starting Topo Path Planner Tests...");

  testTopoGraph();
  testVisibilityChecker();
  testTopoGraphBuilding();

  RCLCPP_INFO(rclcpp::get_logger("test"), "All tests completed!");

  rclcpp::shutdown();
  return 0;
}
