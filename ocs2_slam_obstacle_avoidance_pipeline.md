# OCS2 移动机械臂 SLAM + 动态避障完整 Pipeline

> **项目**: Tracer 移动底盘 + JAKA ZU5 机械臂 + OCS2 MPC 全身控制  
> **目标**: 从静态障碍物避障 → 基于 SLAM 建图/定位/动态障碍物的实时避障  
> **日期**: 2026-07-21

---

## 📊 当前系统架构分析

### 现有架构

```
┌──────────────┐    /odom (轮式里程计)     ┌──────────────────┐
│  tracer_base │ ──────────────────────────→│                  │
│  (CAN驱动)    │                           │ TracerJakaMrtNode │
└──────────────┘                           │                  │
                                           │ state = [x,y,yaw, │
┌──────────────┐  /joint_states             │  q1..q6]         │
│  JAKA arm    │ ──────────────────────────→│                  │
│  (ros2_ctrl) │                           └────────┬─────────┘
└──────────────┘                                    │
                                                    ▼
                                           OCS2 MPC (SLQ solver)
                                           + 静态障碍物 (task.info)
                                           + 自碰撞 (URDF geometry)
```

### 三个核心问题

| 问题 | 原因 |
|------|------|
| **定位漂移** | 只用 `tracer_base` 的轮式里程计，无外部校正 |
| **障碍物是静态的** | `task.info` 中一次加载，`EnvironmentCollisionConstraint` 构造时固定 |
| **无环境感知** | D435i 相机挂在 arm 上，但没有任何感知管线 |

### OCS2 已经支持动态障碍物！

阅读 OCS2 源码发现 `EnvironmentGeometryInterface` (`install/ocs2_mobile_manipulator/.../collision/EnvironmentGeometryInterface.h`) **已经内置了完整的动态障碍物 API**:

```cpp
// 这些 API 都已经被 OCS2 实现了，但还从未被调用！
envGeomInterface->addBox(name, halfExtents, position, orientation, minDist);
envGeomInterface->addSphere(name, radius, position, minDist);
envGeomInterface->addCylinder(name, radius, height, position, orientation, minDist);
envGeomInterface->removeObstacle(name);
envGeomInterface->updateObstaclePose(name, position, orientation);
envGeomInterface->clearAllObstacles();
```

且 `MobileManipulatorInterface::getEnvironmentGeometryInterface()` 已经暴露了 `shared_ptr<EnvironmentGeometryInterface>`，线程安全（内部有 `std::mutex`）。

**只需要写一个 ROS2 节点，把 SLAM 地图转换成障碍物并调用这些 API，整个动态避障就通了。**

---

## 🎯 推荐完整 Pipeline

### 传感器硬件选择

你的机器人是 **Tracer + JAKA ZU5 + D435i camera**（挂在 arm 上，通过 `tool0_and_camera_link` → `d435i_link` 连接）。根据不同硬件配置，推荐以下方案：

### 方案 A：只有 D435i RGB-D 相机 (最轻量，推荐起步)

```
D435i depth camera
    │
    ├── RGB image + Depth image
    │       │
    │       ▼
    │   ┌──────────────────┐
    │   │  RTAB-Map         │  ← 同时做 visual SLAM + 2D栅格地图
    │   │  (ros-humble-     │     唯一能同时输出 /map, /odom 校正
    │   │   rtabmap-ros)    │     和 2D occupancy grid 的方案
    │   └──────┬───────────┘
    │          │
    │   ┌──────┴───────────┐
    │   │  输出:            │
    │   │  /tf: map→odom   │  ← 校正里程计漂移
    │   │  /map (OccupGrid) │  ← 栅格地图
    │   │  /mapData (3D map)│  ← 可做 3D 障碍物
    │   └──────────────────┘
    │
    └── IMU (D435i 内置)
         → 改善视觉 SLAM 在快速运动时的鲁棒性
```

### 方案 B：有 LiDAR (如 Velodyne/Ouster) + IMU (最稳定)

```
LiDAR (mechanical) + IMU (9-axis)
    │
    ▼
┌─────────────────────────────┐
│  slam_toolbox                 │  ← 2D SLAM: 最稳定的 ROS2 2D 方案
│  (online_async 模式)          │     ATE=0.13m, 支持 lifelong mapping
│  输出: /map, map→odom tf     │
└──────────────┬──────────────┘
               │
    ┌──────────┴──────────┐
    │  可选: 与方案 A 融合  │
    │  用 robot_localization│
    │  做 EKF 融合多源里程计 │
    └─────────────────────┘
```

### 方案 C：LiDAR + IMU 做 3D SLAM (未来升级)

```
LiDAR + IMU
    │
    ▼
┌─────────────────────────────┐
│  LIO-SAM (ros2 branch)       │  ← 紧耦合 LiDAR-Inertial 因子图
│  + slam_toolbox (2D)         │     3D 点云地图 + 2D 栅格地图
└─────────────────────────────┘
```

### 推荐：方案 A (RTAB-Map) 作为起步

理由：

- 你已经有 D435i 深度相机（URDF 中确认为 `d435i_link`）
- RTAB-Map 是**唯一**一个开箱即用支持 RGB-D + 同时输出 occupancy grid + 校正里程计的 ROS2 方案
- Tracer 底盘噪声不大（履带式，非轮式），RTAB-Map 的视觉特征跟踪在你的场景足够
- 算力开销适中，不需要额外购买 LiDAR
- 后期加 LiDAR 可以用 `rtabmap_ros` 的 multi-sensor 模式直接融合

---

## 🔧 具体实施步骤

### 第 1 步: 安装 RTAB-Map

```bash
sudo apt install ros-humble-rtabmap-ros ros-humble-rtabmap
```

新建 launch 文件 `tracer_jaka_ocs2/launch/rtabmap_slam.launch.py`:

```python
# 关键配置:
# 1. rgb_topic := /camera/color/image_raw
# 2. depth_topic := /camera/depth/image_raw
# 3. camera_info_topic := /camera/depth/camera_info
# 4. odom_topic := /odom
# 5. output: /rtabmap/grid_map (nav_msgs/OccupancyGrid)
# 6. output: /tf map->odom (校正里程计漂移)
```

RTAB-Map 会:
- 从 `/odom` 获取初始位姿估计
- 用视觉特征+深度图做帧间匹配，发布 `map→odom` 的 TF 校正
- 增量发布 2D occupancy grid 到 `/rtabmap/grid_map`

### 第 2 步: 核心节点 — `ObstacleBridgeNode`

这是整个 Pipeline 的核心——把地图转成 OCS2 动态障碍物。

新建文件 `tracer_jaka_ocs2/src/ObstacleBridgeNode.cpp`:

```cpp
// 核心逻辑:

class ObstacleBridgeNode : public rclcpp::Node {
    // 持有 OCS2 的 envGeomInterface 共享指针
    std::shared_ptr<EnvironmentGeometryInterface> envGeomInterface_;

    // 订阅 OccupancyGrid
    // 定期（5-10 Hz）:
    //   1. 遍历 occupancy grid 的 occupied cells
    //   2. 做聚类 (连通域分析)
    //   3. 对每个 cluster, 用 BoundingBox 转成 OCS2 障碍物
    //   4. 调用 envGeomInterface_->clearAllObstacles()
    //   5. 调用 envGeomInterface_->addBox(...) 添加新障碍物
    //   6. 障碍物坐标从 /map 转到 /odom (用 TF)
};

// 核心循环:
void ObstacleBridgeNode::updateObstacles() {
    // 1. 获取最新的 occupancy grid
    auto grid = latestGrid_;

    // 2. 做连通域聚类
    auto clusters = connectedComponents(grid);

    // 3. 获取 map->odom 变换
    auto tf_map_to_odom = lookupTransform("odom", "map");

    // 4. 更新障碍物（线程安全）
    auto geom = envGeomInterface_;  // shared_ptr

    // 注意: clearAllObstacles 和 addBox 都在 OCS2 内部有 mutex 保护
    geom->clearAllObstacles();

    for (auto& cluster : clusters) {
        auto bbox = computeBoundingBox(cluster);
        // 转换到 odom 坐标系
        auto posInOdom = tf_map_to_odom * bbox.center;
        geom->addBox(
            "obs_" + std::to_string(cluster.id),
            bbox.halfExtents,
            posInOdom,
            Eigen::Quaterniond::Identity(),
            0.05  // minimum distance per obstacle
        );
    }
}
```

**关键设计考量:**

1. **频率**: 5-10 Hz 即可。障碍物更新不需要和 MPC 控制周期 (100Hz) 同步。
2. **坐标系**: SLAM 的地图在 `map` 帧，OCS2 状态在 `odom` 帧。需要通过 TF `map→odom` 做坐标变换。
3. **聚类**: 不能把每个 occupied cell 都作为一个障碍物（太多碰撞对会导致求解变慢），必须聚类。用连通域分析 (connected components) 是最简单有效的。
4. **降采样**: 2D occupancy grid 的高度信息是缺失的。可以假设障碍物高度为固定值（如 1.5m），或者用深度图的 3D 信息（需要额外处理）。

### 第 3 步: 修改 MRT Node 集成 ObstacleBridge

在 `TracerJakaMrtNode.cpp` 中，在 `initMrt()` 里创建 `ObstacleBridgeNode`:

```cpp
// 在 initMrt() 中:
void TracerJakaMrtBridge::initMrt() {
    // ... 现有代码 ...

    // 获取环境几何接口
    auto envGeomInterface = interface_->getEnvironmentGeometryInterface();
    if (envGeomInterface) {
        obstacleBridge_ = std::make_unique<ObstacleBridgeNode>(
            shared_from_this(), envGeomInterface);
        RCLCPP_INFO(get_logger(),
            "ObstacleBridge initialized with SLAM integration");
    }
}
```

### 第 4 步: 修改 launch 文件

在 `ocs2_real.launch.py` 中加入:

```python
# RTAB-Map SLAM
rtabmap_node = Node(
    package='rtabmap_ros', executable='rtabmap',
    parameters=[{
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_odom_info': True,
        'frame_id': 'base_footprint',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',
        'Grid/FromDepth': True,  # 从深度图生成 2D occupancy grid
        'grid_map_topic': '/rtabmap/grid_map',
    }],
    remappings=[
        ('rgb/image', '/camera/color/image_raw'),
        ('depth/image', '/camera/depth/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('odom', '/odom'),
    ],
)

# Obstacle Bridge
obstacle_bridge = Node(
    package='tracer_jaka_ocs2',
    executable='obstacle_bridge_node',
    parameters=[{
        'grid_map_topic': '/rtabmap/grid_map',
        'update_rate': 5.0,          # Hz
        'min_cluster_size': 3,        # 最小聚类 cell 数
        'obstacle_height': 1.5,       # 假设障碍物高度 [m]
    }],
)
```

---

## 📐 完整架构图

```
                     ┌──────────────────────────────────┐
                     │         RTAB-Map (SLAM)           │
                     │  RGB-D + Odometry → /map, tf      │
                     └──────────┬───────────────────────┘
                                │
          ┌─────────────────────┼─────────────────────┐
          │                     │                     │
    /rtabmap/grid_map    /tf: map→odom         /rtabmap/odom
   (OccupancyGrid)      (校正里程计漂移)       (校正后的里程计)
          │                     │                     │
          ▼                     ▼                     ▼
  ┌───────────────┐   ┌───────────────┐    ┌──────────────────┐
  │ ObstacleBridge │   │ TF 校正 MRT    │    │ MRT Node 改用     │
  │ Node (新)      │   │ 的 state 计算  │    │ 校正后 odom      │
  │               │   │              │    │                  │
  │ 栅格地图→聚类  │   │ base_x,base_y │    │ 替代原始 /odom    │
  │ →envGeom API  │   │ base_yaw 用   │    │ 减少累积漂移      │
  │               │   │ map→odom 校正 │    │                  │
  └───────┬───────┘   └───────────────┘    └──────────────────┘
          │
          ▼
  ┌──────────────────────────────────────────┐
  │  EnvironmentGeometryInterface              │
  │  (OCS2 内置, 已支持动态障碍物, 线程安全)   │
  │  addBox / addSphere / clearAllObstacles   │
  └──────────────┬───────────────────────────┘
                 │
                 ▼
  ┌──────────────────────────────────────────┐
  │  MobileManipulatorEnvironmentCollision    │
  │  Constraint                              │
  │  ← MPC 求解时自动计算距离 + Jacobian      │
  └──────────────┬───────────────────────────┘
                 │
                 ▼
  ┌──────────────────────────────────────────┐
  │  OCS2 SLQ MPC Solver                     │
  │  同时优化:                                │
  │  - 全身轨迹跟踪                           │
  │  - 自碰撞 (28 link pairs)                 │
  │  - 环境碰撞 ← 现在是动态障碍物了!          │
  │  - 关节限位/速度限位                       │
  └──────────────────────────────────────────┘
```

---

## 🚀 实施优先级

| 优先级 | 任务 | 工作量 | 依赖 |
|--------|------|--------|------|
| **P0** | 安装 RTAB-Map, 调通 D435i → SLAM → `/map` | 2-3 天 | D435i 驱动 (已有 realsense-ros) |
| **P0** | 实现 `ObstacleBridgeNode` (栅格→聚类→OCS2 API) | 3-5 天 | 可用仿真验证 |
| **P1** | MRT Node 使用 `map→odom` TF 校正定位 | 1-2 天 | RTAB-Map 跑通 |
| **P1** | Task.info 配置调优（activationDistance, obstacle 密度） | 2-3 天 | ObstacleBridge 跑通 |
| **P2** | 3D 障碍物信息利用深度图做高度估计 | 3-5 天 | P1 完成 |
| **P2** | 增加 LiDAR 实现 LIO-SAM 融合, 提升鲁棒性 | 1-2 周 | 硬件采购 |

---

## ⚠️ 关键注意事项

### 1. D435i 挂在 arm 上的问题

SLAM 时 arm 运动会导致相机视角剧烈变化。建议:
- **建图阶段** arm 保持固定（如 home 位姿）
- **定位阶段** 如果 arm 必须运动，用 joint states + FK 补偿相机运动到 `base_link` 坐标系

### 2. 障碍物数量控制

太多障碍物会让 OCS2 碰撞检测的计算量暴增（每个障碍物 × 每个 collision link = O(n×m) 对）。建议:
- 限制最多 ~50-100 个聚类块
- 只保留机器人前方/周边的障碍物（距机器人 3-5m 内）

### 3. task.info 中 `collisionLinks` 的选择

当前只选了 7 个 link (base_link, Link_3~Link_6, tool0_and_camera_link, d435i_link)。ARM 部分 link 会动，确保环境碰撞检查的 link 选择合理，避免漏检。

### 4. 坐标系一致性

OCS2 的状态在 `odom` 帧，SLAM 地图在 `map` 帧。所有障碍物坐标必须转换到 `odom` 帧（或统一用 `map` 帧并改 `world_frame` 参数）。

### 5. 可选 SLAM 库对比

| 库 | 类型 | ROS2 支持 | 地图输出 | 定位 | 推荐度 |
|---|------|-----------|----------|------|--------|
| **RTAB-Map** | Visual + LiDAR | ✅ Humble | 2D grid + 3D map | ✅ | ⭐⭐⭐⭐⭐ (首推) |
| **slam_toolbox** | 2D LiDAR | ✅ Humble | 2D grid | ✅ | ⭐⭐⭐⭐ (有LiDAR时最佳) |
| **Cartographer** | 2D/3D LiDAR | ✅ Humble | 2D grid | ✅ | ⭐⭐⭐ (参数调优复杂) |
| **LIO-SAM** | LiDAR-Inertial | ✅ Humble (ros2 branch) | 3D 点云 | ✅ | ⭐⭐⭐⭐ (3D SLAM) |
| **FAST-LIO2** | LiDAR-Inertial | ⚠️ 部分 | 3D 点云 | ✅ | ⭐⭐⭐ (速度快,需自行封装) |
| **ORB-SLAM3** | Visual | ⚠️ 社区版 | 3D 稀疏 | ✅ | ⭐⭐ (无原生ROS2) |

---

## 📁 相关文件索引

### 项目核心文件
- `TracerJakaMrtNode.cpp` — MRT 控制桥，里程计+关节状态→OCS2 观测
- `TracerJakaMpcNode.cpp` — MPC 求解器节点
- `TracerJakaVisualization.cpp` — 轨迹可视化 + 障碍物 marker
- `task_real.info` — 实机 OCS2 配置（含静态障碍物定义）
- `ocs2_real.launch.py` — 实机 launch 文件

### OCS2 碰撞系统
- `EnvironmentGeometryInterface.h` — 环境障碍物管理 API（已支持动态增删）
- `EnvironmentCollisionConstraint.h` — 环境碰撞约束
- `MobileManipulatorInterface.h` — 机器人接口，暴露 `getEnvironmentGeometryInterface()`
- `MobileManipulatorSelfCollisionConstraint.h` — 自碰撞约束

---

## 🔗 参考资源

- [RTAB-Map ROS2 Wiki](https://github.com/introlab/rtabmap_ros)
- [slam_toolbox GitHub](https://github.com/SteveMacenski/slam_toolbox)
- [LIO-SAM ROS2 Branch](https://github.com/TixiaoShan/LIO-SAM/tree/ros2)
- [OCS2 官方文档](https://leggedrobotics.github.io/ocs2/)
- [realsense-ros (D435i 驱动)](https://github.com/IntelRealSense/realsense-ros)
