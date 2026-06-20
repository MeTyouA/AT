# Topo Path Planner - 拓扑路径规划器

基于哈工大"哨兵"机器人技术文档实现的ROS2拓扑路径搜索规划器。

## 功能特点

- **拓扑路径搜索**: 基于守卫点和连接点的全局路径规划算法
- **骨架提取**: 从代价地图自动提取车道中心线用于采样
- **可见性判断**: 智能判断两点间是否有障碍物和高差
- **多路径规划**: 可找到多条可行路径并按代价排序
- **路径平滑**: 支持路径优化和平滑处理
- **可视化**: 完整的RViz可视化支持

## 算法原理

### 核心概念

1. **守卫点 (Guard)**: 负责探索新区域的节点
2. **连接点 (Connection)**: 连接两个守卫点的节点

### 算法流程

```
1. 骨架提取 → 获取车道中心线
2. 采样 → 在骨架点上进行智能采样
3. 可见性判断 → 检查采样点与守卫点的可见性
4. 拓扑图构建 → 根据可见性构建拓扑连接关系
5. 路径搜索 → 使用图搜索算法查找最优路径
```

## 依赖项

- ROS2 Humble
- nav2_costmap_2d
- OpenCV 4
- Eigen3
- visualization_msgs

## 编译

```bash
cd ~/HIT_ws/sentry_planning
colcon build --packages-select topo_path_planner
source install/setup.bash
```

## 使用

### 启动规划器

```bash
ros2 launch topo_path_planner topo_planner.launch.py
```

### 发送目标点

```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 5.0, y: 3.0, z: 0.0},
    orientation: {w: 1.0}
  }
}"
```

### 运行测试

```bash
ros2 run topo_path_planner test_topo
```

## 参数配置

编辑 `config/topo_planner_params.yaml`:

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `global_frame` | 全局坐标系 | "map" |
| `sample_distance` | 采样点间距 | 0.5m |
| `max_samples` | 最大采样数 | 500 |
| `use_skeleton` | 是否使用骨架提取 | true |
| `enable_smoothing` | 是否启用路径平滑 | true |
| `smoothing_iterations` | 平滑迭代次数 | 5 |

## 发布的话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/topo_plan_path` | nav_msgs/Path | 规划出的最优路径 |
| `/topo_graph_visualization` | MarkerArray | 拓扑图可视化 |
| `/topo_skeleton_image` | Image | 骨架图像 |

## 订阅的话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/goal_pose` | PoseStamped | 目标点 |

## 代码结构

```
topo_path_planner/
├── include/topo_path_planner/
│   ├── topo_node.hpp          # 拓扑图节点数据结构
│   ├── visibility_checker.hpp # 可见性判断
│   ├── skeleton_extractor.hpp # 骨架提取
│   ├── topo_graph.hpp         # 拓扑图和路径搜索
│   └── topo_planner_node.hpp  # ROS2节点
├── src/
│   ├── topo_node.cpp
│   ├── visibility_checker.cpp
│   ├── skeleton_extractor.cpp
│   ├── topo_graph.cpp
│   └── topo_planner_node.cpp
├── config/
│   └── topo_planner_params.yaml
├── launch/
│   └── topo_planner.launch.py
└── test/
    └── test_topo.cpp
```

## RViz可视化

添加以下显示项查看规划结果：

- **MarkerArray**: `/topo_graph_visualization`
  - 红色球体 = 守卫点
  - 灰色线段 = 连接
  - 蓝色线 = 最优路径
  - 绿色线 = 备选路径

- **Path**: `/topo_plan_path`

- **Image**: `/topo_skeleton_image` (骨架提取结果)

## 技术参考

基于以下论文/项目实现：
- fast-planner 拓扑路径搜索算法
- 哈尔滨工业大学哨兵机器人技术文档

## 许可证

MIT License
