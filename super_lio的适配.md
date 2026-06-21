# super_lio 的适配

本文记录本仓库中将定位前端切换到 `super_lio` 后，点云地图跟随小车自转的问题定位过程、最终数据流、错误原因和代码修改位置。

## 当前数据流

仿真启动：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py
```

导航相关点云/里程计数据流如下：

```text
/red_standard_robot1/velodyne_points
/red_standard_robot1/livox/imu
        |
        v
super_lio
        |
        |-- /red_standard_robot1/lio/odom
        |-- /red_standard_robot1/lio/cloud_world
        v
loam_interface
        |
        |-- /red_standard_robot1/lidar_odometry
        |-- /red_standard_robot1/registered_scan
        v
p_terrain_analysis_ext
        |
        v
/red_standard_robot1/terrain_map_ext
        |
        |-- Nav2 costmap obstacle source
        |-- Fast-Planner / RC-ESDF obstacle source
        |-- RViz 点云显示
```

`loam_interface` 当前订阅的是 `super_lio` 的输出：

```yaml
loam_interface:
  ros__parameters:
    state_estimation_topic: "lio/odom"
    registered_scan_topic: "lio/cloud_world"
    odom_frame: "odom"
    base_frame: "base_footprint"
    lidar_frame: "front_mid360"
```

## 问题现象

使用 `super_lio` 后，RViz 中的点云地图会跟着小车自转方向旋转，但旋转速度比小车慢。

表现主要出现在：

```text
/red_standard_robot1/lio/cloud_world
/red_standard_robot1/registered_scan
/red_standard_robot1/terrain_map_ext
```

这说明问题不是 Nav2 costmap 或 Fast-Planner 单独造成的，而是在 `super_lio` 输出 world 点云时已经出现。

## 最终原因

根因是仿真 `velodyne_points` 中的点内时间字段和实际点云发布频率不匹配。

实测 `velodyne_points` 字段：

```text
x, y, z, intensity, ring, time
```

其中 `time` 范围约为：

```text
min_time = 0.000000000
max_time = 0.099946670
```

但点云话题实际经常约 0.05s 发布一帧，也就是约 20Hz。结果是：

```text
点内 time 表示一帧有约 0.1s
实际话题周期经常只有约 0.05s
```

这会让 `super_lio` 认为一帧点云持续 0.1s，并按 0.1s 对每个点做运动去畸变。小车启动自转时，这个去畸变会使用错误的时间跨度，导致点云被过度或错误旋转补偿，最终表现为点云慢速跟着小车自转。

`point_lio` 之所以正常，是因为当前配置里：

```yaml
point_lio:
  ros__parameters:
    preprocess:
      timestamp_unit: 2
```

`Point-LIO` 在这个配置下实际几乎没有按 `0.1s` 的点内时间做强去畸变，所以没有触发同样的问题。不能直接照抄 `point_lio`，但可以借鉴它在当前仿真数据下的有效行为：不要让 `super_lio` 按这组不可靠的点内时间去畸变。

## 修改方案

给 `super_lio` 增加一个点内时间缩放参数：

```yaml
lio.sensor.point_time_scale
```

默认值为 `1.0`，保持原始行为；仿真中设置为 `0.0`，关闭点内时间去畸变。

仿真配置：

```yaml
super_lio:
  ros__parameters:
    lio.sensor.point_time_scale: 0.0
```

这样 `super_lio` 仍然使用 IMU 和 LiDAR 做 LIO，但不会再使用仿真里这组重叠的 `velodyne_points.time` 去做错误的点级 deskew。

## 修改位置

### 1. 新增全局参数声明

文件：

```text
super-lio/src/super_lio/include/lio/params.h
```

新增：

```cpp
extern double g_point_time_scale;
```

### 2. 新增全局参数定义

文件：

```text
super-lio/src/super_lio/src/lio/params.cpp
```

新增：

```cpp
double g_point_time_scale = 1.0;
```

### 3. ROS 参数读取

文件：

```text
super-lio/src/super_lio/src/ros/ROSWrapper.cpp
```

新增：

```cpp
node.declare_parameter<double>("lio.sensor.point_time_scale", 1.0);
node.get_parameter("lio.sensor.point_time_scale", g_point_time_scale);
```

### 4. 点内时间统一乘 scale

文件：

```text
super-lio/src/super_lio/src/ros/ROSWrapper.cpp
```

对 Livox、Hesai、Velodyne、Ouster 等点云解析分支中的点内时间统一乘：

```cpp
g_point_time_scale
```

Velodyne 分支核心逻辑：

```cpp
const double point_offset_time = pt.time * g_point_time_scale;
offset_time = std::max(offset_time, point_offset_time);
lidar_data.pc->emplace_back(
    pt.x, pt.y, pt.z, pt.intensity, point_offset_time);
lidar_data.end_time = lidar_data.start_time + offset_time;
```

同时将 `end_time` 改为使用最大点时间，而不是依赖最后一个点的 `offset_time`：

```cpp
offset_time = std::max(offset_time, point_offset_time);
lidar_data.end_time = lidar_data.start_time + offset_time;
```

这样更稳，因为点云中的点不一定严格按时间排序。

### 5. 仿真配置设置为 0

文件：

```text
pb2025_nav_bringup/config/simulation/nav2_params.yaml
```

新增：

```yaml
lio.sensor.point_time_scale: 0.0
```

注释说明：

```yaml
# 仿真 velodyne_points 的 time 为 0~0.1s，但话题约 20Hz；
# 置 0 关闭点内去畸变，避免 super_lio 按重叠扫描补偿导致点云慢速跟车旋转
```

## 已回退的无效方向

中途尝试过让 `loam_interface` 使用 `message_filters::ApproximateTime` 同步 `lio/odom` 和 `lio/cloud_world`，使 `lidar_odometry` 和 `registered_scan` 成对发布。

这个方向没有解决“点云跟着车自转”的根因，因为问题在 `super_lio` 发布 `/lio/cloud_world` 时就已经存在。因此这部分同步改动已回退，避免污染原代码。

## 验证命令

重新编译：

```bash
cd /home/wsc/Desktop/My_sentry2
colcon build --packages-select super_lio pb2025_nav_bringup --symlink-install
source install/setup.bash
```

重新启动：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py
```

确认参数生效：

```bash
ros2 param get /red_standard_robot1/super_lio lio.sensor.point_time_scale
```

期望输出：

```text
Double value is: 0.0
```

观察话题：

```bash
ros2 topic hz /red_standard_robot1/lio/odom
ros2 topic hz /red_standard_robot1/lio/cloud_world
ros2 topic hz /red_standard_robot1/registered_scan
ros2 topic hz /red_standard_robot1/terrain_map_ext
```

RViz 中重点观察：

```text
/red_standard_robot1/lio/cloud_world
/red_standard_robot1/terrain_map_ext
```

正常现象：小车自转时，墙体和环境点云不再按之前那种慢速绕车旋转。

## 后续注意

`lio.sensor.point_time_scale: 0.0` 是针对当前仿真 `velodyne_points.time` 异常/重叠的适配值。

如果之后换成真实雷达，且点内时间字段可靠，应恢复为：

```yaml
lio.sensor.point_time_scale: 1.0
```

否则真实高速运动时会失去点级去畸变能力。
