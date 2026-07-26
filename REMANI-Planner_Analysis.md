# REMANI-Planner 全局规划器 Pipeline、RViz2 可视化分析与 ROS2 迁移方案

> 本文档详细分析了 REMANI-Planner 的全局规划器 Pipeline 架构，并给出了 ROS2 迁移方案及与 OCS2 + MPC 集成的完整路线图。

---

## 目录

- [一、Pipeline 架构总览](#一pipeline-架构总览)
  - [1.1 整体架构](#11-整体架构)
  - [1.2 Pipeline 核心流程（4步法）](#12-pipeline-核心流程4步法)
  - [1.3 轨迹表示：MINCO](#13-轨迹表示minco)
  - [1.4 碰撞检测体系（4层）](#14-碰撞检测体系4层)
  - [1.5 FSM 状态机](#15-fsm-状态机)
- [二、核心文件清单](#二核心文件清单)
  - [2.1 按优先级排列](#21-按优先级排列)
  - [2.2 各文件职责](#22-各文件职责)
- [三、ROS2 迁移方案](#三ros2-迁移方案)
  - [3.1 总体策略](#31-总体策略)
  - [3.2 Phase 1：基础设施迁移](#32-phase-1基础设施迁移)
  - [3.3 Phase 2：核心依赖适配](#33-phase-2核心依赖适配)
  - [3.4 Phase 3：架构改进](#34-phase-3架构改进)
  - [3.5 推荐包结构](#35-推荐包结构)
- [四、OCS2 + MPC 轨迹跟踪与动态避障集成](#四ocs2--mpc-轨迹跟踪与动态避障集成)
  - [4.1 当前能力盘点](#41-当前能力盘点)
  - [4.2 推荐架构](#42-推荐架构)
  - [4.3 动态避障集成方案](#43-动态避障集成方案)
  - [4.4 推荐实施顺序](#44-推荐实施顺序)
  - [4.5 关键注意事项](#45-关键注意事项)
- [五、总结](#五总结)
- [六、RViz2 可视化代码与数据流](#六rviz2-可视化代码与数据流)

---

## 一、Pipeline 架构总览

### 1.1 整体架构

REMANI-Planner 采用 **分层时间-空间优化** 架构，针对移动机械臂（Mobile Manipulator, MM）做实时全身运动规划。

**核心思路**：

> **前端**：环境自适应搜索（Hybrid A\* + 全身 RRT）
>
> **后端**：时空联合轨迹优化（MINCO 轨迹 + L-BFGS 优化）

#### 系统架构图

```
┌──────────────────────────────────────────────────────┐
│                  REMANI-Planner                       │
│                                                      │
│  ┌─────────────┐   ┌──────────────┐   ┌───────────┐  │
│  │  FSM 状态机  │ → │ PlannerManager│ → │ 轨迹发布   │  │
│  │  (调度核心)  │   │  (前后端串联) │   │ (ROS msg) │  │
│  └─────────────┘   └──────┬───────┘   └───────────┘  │
│                           │                          │
│         ┌─────────────────┼─────────────────┐        │
│         ▼                 ▼                 ▼        │
│  ┌────────────┐   ┌──────────────┐   ┌───────────┐   │
│  │ 前端搜索    │   │ 后端优化      │   │ 碰撞检测   │   │
│  │ Kino A*    │   │ MINCO+L-BFGS │   │ GridMap   │   │
│  │ + RRT      │   │              │   │ + MMConfig│   │
│  └────────────┘   └──────────────┘   └───────────┘   │
│                                                      │
└──────────────────────────────────────────────────────┘
         ▲                                    │
         │ 感知 (深度相机/点云)                 ▼
         │                            ┌──────────────┐
         └────────────────────────────│  mm_controller│
                                      │  轨迹跟踪控制器 │
                                      └──────────────┘
```

### 1.2 Pipeline 核心流程（4步法）

```
STEP 1: Global Plan (一次性)
┌─────────────────────────────────────────────┐
│ planGlobalTrajWaypoints()                    │
│ 输入: start_pos, end_pt, waypoints          │
│ 通过多段 MINCO 连接 waypoints               │
│ 输出: global_traj → traj_container_         │
│ 时间分配: 基于 max_vel/max_mani_vel 计算    │
└─────────────────────┬───────────────────────┘
                      │
                      ▼
STEP 2: Local Target 选取
┌─────────────────────────────────────────────┐
│ getLocalTarget()                             │
│ 沿 global_traj 采样                          │
│ 取 planning_horizon 距离处的点               │
│ 做碰撞检测 (确保 local_target 自身安全)      │
│ 输出: local_target_pt, vel, acc, yaw        │
└─────────────────────┬───────────────────────┘
                      │
                      ▼
STEP 3: Front-End 前端路径搜索
┌─────────────────────────────────────────────┐
│ computeInitReferenceState()                  │
│ ┌─────────────────────────────────────────┐  │
│ │ a) Hybrid A* (KinoAstar)                 │  │
│ │    - 搜索基底 (x, y, yaw) 的可行路径     │  │
│ │    - 使用 Reeds-Shepp/Dubins 曲线连接    │  │
│ │    - 考虑前进/后退 (singul) 切换         │  │
│ │    - 惩罚项: 前进/后退/转向/换挡         │  │
│ ├─────────────────────────────────────────┤  │
│ │ b) 全身 RRT (SampleManiRRT)              │  │
│ │    - 在 A* 路径点上采样机械臂构型         │  │
│ │    - 双向RRT + 碰撞检测                  │  │
│ │    - 作为 A* 连续失败后的 fallback       │  │
│ └─────────────────────────────────────────┘  │
│ 输出: MINCO 初值 (initMJO_container)        │
│       分段轨迹: 每次换向(singul切换)=一段   │
└─────────────────────┬───────────────────────┘
                      │
                      ▼
STEP 4: Back-End 后端轨迹优化
┌─────────────────────────────────────────────┐
│ OptimizeTrajectory_lbfgs()                   │
│                                              │
│ 代价函数:                                     │
│ J = w₁ · J_obs_car        (底盘障碍物)       │
│   + w₂ · J_obs_mani       (机械臂障碍物)     │
│   + w₃ · J_self_collision (自碰撞)           │
│   + w₄ · J_feas_car       (底盘可行性)       │
│   + w₅ · J_feas_joint     (关节可行性)       │
│   + w₆ · J_time           (时间代价)         │
│   + J_snap               (Jerk/Snap平滑)     │
│                                              │
│ 约束: 动力学约束 (vel/acc/jerk limit)         │
│ 优化变量: 中间点位置 + 段时间分配             │
│ 求解器: L-BFGS (无约束化技巧)                │
│                                              │
│ 输出: 最优 MINCO 轨迹 → SingulTrajData       │
└─────────────────────────────────────────────┘
```

#### 关键数据流

```cpp
// 伪代码表示
REMANIReplanFSM::execFSMCallback() {
    switch (exec_state_) {
        case GEN_NEW_TRAJ:
            planFromGlobalTraj(10);  // 最多尝试10次
            → callReboundReplan(flag_use_poly_init, flag_randomPolyTraj)
            → planner_manager_->reboundReplan(...)
            → sendPolyTrajROSMsg()
            break;

        case EXEC_TRAJ:
            // 检测是否需要重规划
            if (需要重规划)
                changeFSMExecState(REPLAN_TRAJ);
            break;

        case REPLAN_TRAJ:
            planFromLocalTraj(flag_relan_astar_);  // 从当前轨迹采样
            → callReboundReplan(...)
            break;

        case EMERGENCY_STOP:
            callEmergencyStop(mm_state_pos_, mm_car_yaw_);
            break;
    }
}
```

### 1.3 轨迹表示：MINCO

- **来源**：[GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER) (ZJU FAST Lab)
- **多项式阶数**：7 阶 (8 次多项式，degree=7)
- **状态维度**：$2 + N_{joints}$（移动基底 x, y + 机械臂各关节角）
  - 默认配置：mobile_base_dim=2, manipulator_dim=6 → 总维度 8
- **核心数据结构**：`poly_traj::MinSnapOpt<8>` → `poly_traj::Trajectory<7>`

#### MINCO 的数学本质

MINCO 将轨迹参数化为：**中间点位置 + 每段时间**。给定这些参数后，通过求解带状线性系统（Banded LU）唯一确定多项式系数。

```
参数化: (innerPts, time_vec) → 多项式系数 coeffMat
优点:
  - 参数空间紧凑 (只有中间点和时间)
  - 空间-时间联合优化
  - 平滑性约束自动满足 (C³ 连续)
  - 梯度可以高效计算 (解析梯度通过伴随方法)
```

#### SingulTraj 机制

移动底盘不能瞬时切换前进/后退，因此：
- 每段**连续同向运动**为一条 `LocalTrajData`
- 换向时另起一段，标记 `singul = 1` (前进) / `-1` (后退)
- `SingulTrajData` 是多段 `LocalTrajData` 的容器

```cpp
// plan_container.hpp
struct SingulTrajData {
    SingulTraj singul_traj;  // vector<LocalTrajData>
    int traj_id;
    double duration;
    double start_time;

    // 通过时间查询状态
    Eigen::VectorXd getPos(double t) const;
    Eigen::VectorXd getVel(double t) const;
    int getSingul(double t) const;  // 获取当前位置的前进/后退状态
};
```

### 1.4 碰撞检测体系（4层）

| 类型 | 函数 | 说明 |
|:---|:---|:---|
| **Car-Obs** | `MMConfig::checkCarObsCollision()` | 底盘 vs 障碍物：多条球体/柱体与 ESDF 查询 |
| **Mani-Obs** | `MMConfig::checkManiObsCollision()` | 机械臂 vs 障碍物：连杆球体模型 |
| **Car-Mani** | `MMConfig::checkCarManiCollision()` | 底盘 vs 机械臂（自碰撞） |
| **Mani-Mani** | `MMConfig::checkManiManiCollision()` | 机械臂自碰撞：关节间距离检测 |

**碰撞模型**：使用球体包裹（sphere-based）— 底盘和机械臂连杆上放置多个碰撞球，通过 `setLinkPoint()` 配置。在轨迹优化的梯度计算中，这些球体位置相对于优化变量求解析梯度。

**ESDF 查询**：
```cpp
// GridMap 提供带梯度的距离查询
grid_map_->evaluateEDTWithGrad(pos, dist, grad);
// dist  < safe_margin_ → 碰撞
// dist  < 0 → 在障碍物内部
```

### 1.5 FSM 状态机

```
                    ┌──────────┐
                    │   INIT   │ ← 等待里程计
                    └────┬─────┘
                         │ have_odom_
                         ▼
                    ┌──────────────┐
              ┌─────│ WAIT_TARGET  │ ← 等待用户发送目标
              │     └──────┬───────┘
              │            │ have_target_
              │            ▼
              │     ┌──────────────┐
              │     │ GEN_NEW_TRAJ │ ← 从头规划全局轨迹
              │     └──────┬───────┘
              │            │ success
              │            ▼
              │     ┌──────────────┐
              │  ┌─→│  EXEC_TRAJ   │ ← 执行轨迹, 监控碰撞
              │  │  └──┬───┬───────┘
              │  │     │   │ 需要重规划 / 检测到碰撞
              │  │     │   ▼
              │  │     │  ┌──────────────┐
              │  │     └─→│ REPLAN_TRAJ   │ ← 局部重规划
              │  │        └──────┬─────────┘
              │  │               │ success → EXEC_TRAJ
              │  │               │ fail×20 → WAIT_TARGET
              │  │
              │  │  碰撞即将发生 & 重规划失败
              │  │        │
              │  │        ▼
              │  │  ┌────────────────┐
              │  └──│ EMERGENCY_STOP │ ← 紧急停止
              │     └────────┬───────┘
              │              │ 静止后
              │              ▼
              └──────── GEN_NEW_TRAJ (重新规划)
```

---

## 二、核心文件清单

### 2.1 按优先级排列

| 优先级 | 文件路径 | 行数(估) | 角色 |
|:---:|:---|:---:|:---|
| ⭐⭐⭐ | `remani_planner/plan_manage/src/remani_replan_fsm.cpp` | ~700 | **FSM 状态机**，整个规划流程的调度核心 |
| ⭐⭐⭐ | `remani_planner/plan_manage/include/plan_manage/remani_replan_fsm.h` | ~166 | FSM 头文件，状态/参数定义 |
| ⭐⭐⭐ | `remani_planner/plan_manage/src/planner_manager.cpp` | ~581 | **规划管理器**，`reboundReplan()` 串联前后端 |
| ⭐⭐⭐ | `remani_planner/plan_manage/include/plan_manage/planner_manager.h` | ~86 | 管理器接口定义 |
| ⭐⭐⭐ | `remani_planner/traj_opt/include/optimizer/poly_traj_optimizer.hpp` | ~234 | **后端优化器**接口，L-BFGS 代价函数声明 |
| ⭐⭐⭐ | `remani_planner/traj_opt/src/poly_traj_optimizer.cpp` | ~600+ | 后端优化器实现，包含障碍物/可行性/时间代价及梯度计算 |
| ⭐⭐⭐ | `remani_planner/path_searching/include/path_searching/kino_astar.h` | ~213 | **前端 Hybrid A\*** 接口定义 |
| ⭐⭐⭐ | `remani_planner/path_searching/src/kino_astar.cpp` | ~800+ | Hybrid A\* 搜索实现，Reeds-Shepp/Dubins 曲线 |
| ⭐⭐ | `remani_planner/path_searching/include/path_searching/sample_mani_RRT.h` | ~80 | **全身 RRT** 接口 |
| ⭐⭐ | `remani_planner/path_searching/src/sample_mani_RRT.cpp` | ~300+ | 全身 RRT 实现，机械臂构型采样 |
| ⭐⭐ | `remani_planner/traj_utils/include/traj_utils/poly_traj_utils.hpp` | ~1650 | **MINCO 轨迹**数学核心，多项式 Piece/Trajectory/MinSnapOpt |
| ⭐⭐ | `remani_planner/traj_utils/include/traj_utils/plan_container.hpp` | ~204 | **轨迹容器**数据结构 |
| ⭐⭐ | `remani_planner/plan_env/include/plan_env/grid_map.h` | ~631 | **ESDF 占据地图**接口 |
| ⭐⭐ | `remani_planner/plan_env/src/grid_map.cpp` | ~500+ | 地图实现，raycasting, ESDF 更新 |
| ⭐⭐ | `remani_planner/mm_config/include/mm_config/mm_config.hpp` | ~176 | **机器人运动学/碰撞模型**接口 |
| ⭐⭐ | `remani_planner/mm_config/src/mm_config.cpp` | ~400+ | 运动学实现，碰撞检测，可视化 |
| ⭐ | `remani_planner/plan_manage/config/mm_param.yaml` | ~40 | 机器人物理参数（DH表、尺寸、关节限位） |
| ⭐ | `remani_planner/plan_manage/config/remani_planner_param.yaml` | ~59 | 算法参数（搜索、优化、FSM 阈值） |
| ⭐ | `remani_planner/plan_manage/src/planning_visualization.cpp` | ~300+ | RViz 可视化辅助 |
| ⭐ | `mm_controller/src/mm_controller_fsm.cpp` | ~300+ | 轨迹跟踪控制器（另一套 FSM） |
| ⭐ | `remani_planner/plan_manage/src/remani_planner_node.cpp` | ~30 | ROS 主节点入口 |

### 2.2 各文件职责

```
remani_planner/
├── plan_manage/            ← 顶层调度
│   ├── FSM (状态机)        ← execFSMCallback() 每10ms触发
│   ├── PlannerManager      ← 核心 planning API
│   └── Visualization       ← RViz 可视化
│
├── path_searching/         ← 前端 (Front-End)
│   ├── KinoAstar           ← 基底 Hybrid A*
│   │   策略: SE(2) 空间搜索 (x, y, yaw)
│   │   运动基元: Reeds-Shepp / Dubins 曲线
│   │   启发式: 混合 A* 惩罚项 (前进/后退/转向/换挡)
│   ├── SampleManiRRT       ← 机械臂 RRT
│   │   策略: 在 A* 路径点之间采样关节构型
│   └── RRT (基础)          ← 通用 RRT 实现
│
├── traj_opt/               ← 后端 (Back-End)
│   ├── PolyTrajOptimizer   ← L-BFGS 优化器
│   │   代价: obstacle + feasibility + time + snap
│   │   梯度: 解析梯度链式法则 (MINCO 伴随方法)
│   │   约束: 动力学 (vel/acc/jerk), 碰撞 (ESDF)
│   └── LBFGS               ← 无约束优化求解器 (自带)
│
├── plan_env/               ← 环境感知
│   └── GridMap             ← ESDF 占据地图
│       输入: 深度相机 / 点云
│       输出: occupancy + ESDF distance field
│       支持: trilinear interpolation (距离+梯度)
│
├── traj_utils/             ← 基础设施
│   ├── poly_traj_utils     ← MINCO 多项式轨迹
│   │   Piece<T>: 单段多项式
│   │   Trajectory<T>: 多段连接
│   │   MinSnapOpt: MINCO 生成+梯度
│   └── plan_container      ← 数据容器
│        GlobalTrajData, SingulTrajData, etc.
│
└── mm_config/              ← 机器人模型
    └── MMConfig            ← 运动学+碰撞几何
         Forward Kinematics (DH参数)
         碰撞球体配置
         可视化mesh生成
```

---

## 三、ROS2 迁移方案

### 3.1 总体策略

**分层迁移，先跑通再优化。** 不要一次性推倒重来。

### 3.2 Phase 1：基础设施迁移（预计 1-2 周）

#### ROS1 → ROS2 API 映射表

| ROS1 | ROS2 |
|:---|:---|
| `ros::NodeHandle` | `rclcpp::Node` (继承) |
| `nh.param<T>(...)` | `node->declare_parameter<T>(...)` / `get_parameter(...)` |
| `ros::Timer` | `rclcpp::TimerBase::SharedPtr` |
| `ros::Publisher` | `rclcpp::Publisher<T>::SharedPtr` |
| `ros::Subscriber` | `rclcpp::Subscription<T>::SharedPtr` |
| `ros::spinOnce()` | `rclcpp::spin_some(node)` 或 `SingleThreadedExecutor` |
| `ros::Duration(...)` | `rclcpp::Duration(...)` |
| `ros::Time::now()` | `node->now()` |
| `ros::package::getPath(...)` | `ament_index_cpp::get_package_share_directory(...)` |
| `.launch` (XML) | `.launch.py` (Python) |
| `catkin_make` | `colcon build` |
| `package.xml` format v2 | format v3 |
| `CMakeLists.txt` + catkin | `CMakeLists.txt` + ament_cmake |

#### CMakeLists.txt 迁移示例

**ROS1 (原始)**：
```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs geometry_msgs nav_msgs sensor_msgs
  visualization_msgs tf tf2 cv_bridge pcl_ros
  image_transport dynamic_reconfigure control_msgs trajectory_msgs
  message_generation eigen_conversions
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES remani_planner
  CATKIN_DEPENDS ...
)
```

**ROS2 (迁移后)**：
```cmake
cmake_minimum_required(VERSION 3.8)
project(remani_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(ompl REQUIRED)

add_library(remani_planner_core SHARED
  src/planner_manager.cpp
  src/remani_replan_fsm.cpp
  src/planning_visualization.cpp
  # ...
)
target_include_directories(remani_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(remani_planner_core
  rclcpp geometry_msgs nav_msgs sensor_msgs
  visualization_msgs tf2 tf2_eigen cv_bridge
)
target_link_libraries(remani_planner_core
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

add_executable(remani_planner_node src/remani_planner_node.cpp)
target_link_libraries(remani_planner_node remani_planner_core)
ament_target_dependencies(remani_planner_node rclcpp)

install(TARGETS remani_planner_core remani_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
```

#### package.xml 迁移示例

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
            schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>remani_planner</name>
  <version>1.0.0</version>
  <description>Real-time Whole-body Motion Planning for Mobile Manipulators</description>
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>GPLv3</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>visualization_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_eigen</depend>
  <depend>cv_bridge</depend>
  <depend>eigen</depend>
  <depend>libpcl-all-dev</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

#### ROS1 → ROS2 代码改写示例

**ROS1 (原始)**：
```cpp
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

class REMANIReplanFSM {
private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher poly_traj_pub_;
    ros::Timer exec_timer_;

    void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
        mm_state_pos_(0) = msg->pose.pose.position.x;
        // ...
    }

public:
    void init(ros::NodeHandle& nh) {
        nh_ = nh;
        double val;
        nh.param("fsm/planning_horizon", val, 5.0);
        odom_sub_ = nh.subscribe("odom_world", 1, &REMANIReplanFSM::odomCallback, this);
        poly_traj_pub_ = nh.advertise<PolynomialTraj>("planning/trajectory", 10);
        exec_timer_ = nh.createTimer(ros::Duration(0.01), &REMANIReplanFSM::execFSMCallback, this);
    }
};
```

**ROS2 (迁移后)**：
```cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class REMANIReplanFSM : public rclcpp::Node {
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<PolynomialTraj>::SharedPtr poly_traj_pub_;
    rclcpp::TimerBase::SharedPtr exec_timer_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        mm_state_pos_(0) = msg->pose.pose.position.x;
        // ...
    }

public:
    REMANIReplanFSM() : Node("remani_planner_node") {
        // 参数声明
        this->declare_parameter("fsm.planning_horizon", 5.0);
        double val = this->get_parameter("fsm.planning_horizon").as_double();

        // 订阅者
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom_world", 1,
            std::bind(&REMANIReplanFSM::odomCallback, this, std::placeholders::_1));

        // 发布者
        poly_traj_pub_ = this->create_publisher<PolynomialTraj>("planning/trajectory", 10);

        // 定时器
        exec_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&REMANIReplanFSM::execFSMCallback, this));
    }
};
```

### 3.3 Phase 2：核心依赖适配

| 依赖 | ROS1 情况 | ROS2 方案 | 改动量 |
|:---|:---|:---|:---:|
| **OMPL** | `libompl-dev` (ROS1 apt) | 需自行编译或使用 OMPL 官方仓库 | 低 |
| **Eigen3** | 系统安装 | 系统安装 (无需改动) | 无 |
| **L-BFGS** | `traj_opt/include/optimizer/lbfgs.hpp` 自带 | 无需改动 | 无 |
| **sensor_msgs/JointState** | 直接可用 | `<sensor_msgs/msg/joint_state.hpp>` | 低 |
| **nav_msgs/Odometry** | 直接可用 | `<nav_msgs/msg/odometry.hpp>` | 低 |
| **tf/tf2** | tf 和 tf2 混用 | 统一用 `tf2` (`tf2_ros`, `tf2_eigen`) | 中 |
| **message_filters** | roscpp | ROS2 有移植版 `message_filters` | 低 |
| **cv_bridge** | 直接可用 | `<cv_bridge/cv_bridge.hpp>` | 低 |
| **PCL** | ROS1 版本 | ROS2 版本 (头文件路径可能变化) | 低 |
| **rviz_plugins** | Qt5 + librviz | 需用 Qt + rviz2 重写 | 高 |

### 3.4 Phase 3：架构改进

在迁移过程中，可以做以下结构性改进：

#### 改进 1：FSM 从 Timer 驱动改为 Action Server

```
ROS1: Timer(10ms) + switch-case 状态机轮询
  → 状态切换不可取消，无反馈机制

ROS2: rclcpp_action::Server<NavigateToPose>
  → 支持 cancel / feedback / result
  → 客户端可以知道规划进度
```

```cpp
// ROS2 Action 方案示例
class REMANIReplanFSM : public rclcpp::Node {
    // Action Server 替代 FSM timer
    rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const NavigateToPose::Goal> goal)
    {
        end_pt_ = goal->pose;
        trigger_planning_ = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
    {
        changeFSMExecState(EMERGENCY_STOP, "CANCEL");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void execute_planning(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
        // 反馈机制
        auto feedback = std::make_shared<NavigateToPose::Feedback>();
        feedback->state = "GEN_NEW_TRAJ";
        goal_handle->publish_feedback(feedback);

        // 规划逻辑
        bool success = planFromGlobalTraj(10);
        // ...

        auto result = std::make_shared<NavigateToPose::Result>();
        result->success = success;
        goal_handle->succeed(result);
    }
};
```

#### 改进 2：使用 Lifecycle Node 管理规划器生命周期

```cpp
// 使用生命周期管理，方便调试和资源管理
class REMANIReplanFSM : public rclcpp_lifecycle::LifecycleNode {
public:
    // configure: 加载参数、初始化模块
    CallbackReturn on_configure(const rclcpp_lifecycle::State&) {
        mm_config_->setParam(...);
        grid_map_->initMap(...);
        return CallbackReturn::SUCCESS;
    }

    // activate: 开始运行
    CallbackReturn on_activate(const rclcpp_lifecycle::State&) {
        exec_timer_->reset();
        return CallbackReturn::SUCCESS;
    }

    // deactivate: 暂停
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) {
        exec_timer_->cancel();
        return CallbackReturn::SUCCESS;
    }
};
```

#### 改进 3：MMConfig 插件化

使得不同机器人的碰撞模型可插拔：

```cpp
// 定义机器人配置的基类接口
class MMConfigBase {
public:
    virtual void getCarPts(const Eigen::Vector3d& state,
                           std::vector<Eigen::Vector3d>& pts) = 0;
    virtual bool checkCollision(const Eigen::Vector3d& car_state,
                                const Eigen::VectorXd& mani_state,
                                bool safe) = 0;
    virtual int getManiDof() const = 0;
    // ...
};

// 你的机器人只需实现这个接口
class MyRobotConfig : public MMConfigBase {
    // 实现你的运动学和碰撞检测
};
```

### 3.5 推荐包结构

```
remani_planner_ros2/
├── remani_msgs/              # 自定义 ROS2 消息
│   ├── msg/
│   │   ├── PolynomialTraj.msg
│   │   └── DataDisp.msg
│   ├── CMakeLists.txt
│   └── package.xml
│
├── remani_map/               # GridMap 地图模块 (独立 Node)
│   ├── include/remani_map/
│   │   └── grid_map.hpp
│   ├── src/
│   │   ├── grid_map.cpp
│   │   └── remani_map_node.cpp
│   ├── CMakeLists.txt
│   └── package.xml
│
├── remani_planner/           # 核心规划器
│   ├── include/remani_planner/
│   │   ├── remani_replan_fsm.hpp
│   │   ├── planner_manager.hpp
│   │   ├── optimizer/
│   │   │   └── poly_traj_optimizer.hpp
│   │   ├── path_searching/
│   │   │   ├── kino_astar.hpp
│   │   │   └── sample_mani_RRT.hpp
│   │   ├── mm_config_base.hpp      # 新：机器人配置基类
│   │   └── planning_visualization.hpp
│   ├── src/
│   │   ├── remani_planner_node.cpp  # 主节点 (Lifecycle + Action)
│   │   ├── remani_replan_fsm.cpp
│   │   ├── planner_manager.cpp
│   │   ├── poly_traj_optimizer.cpp
│   │   ├── kino_astar.cpp
│   │   ├── sample_mani_RRT.cpp
│   │   └── mm_config_default.cpp
│   ├── config/
│   │   ├── planner_param.yaml
│   │   └── mm_param.yaml
│   ├── launch/
│   │   └── planner.launch.py
│   ├── CMakeLists.txt
│   └── package.xml
│
├── my_robot_config/          # 你的机器人专属配置包
│   ├── include/
│   │   └── my_robot_config.hpp   # 继承 MMConfigBase
│   ├── src/
│   │   └── my_robot_config.cpp
│   ├── config/
│   │   └── my_robot_param.yaml
│   └── CMakeLists.txt
│
├── remani_controller/        # 轨迹跟踪控制器 (可被 OCS2 替代)
│   ├── src/
│   │   └── mm_controller_node.cpp
│   └── CMakeLists.txt
│
└── remani_bringup/           # 总启动入口
    ├── launch/
    │   ├── simulation.launch.py
    │   └── real_robot.launch.py
    └── CMakeLists.txt
```

---

## 四、OCS2 + MPC 轨迹跟踪与动态避障集成

### 4.1 当前能力盘点

你已经拥有的：

```
REMANI-Planner                         OCS2 (你已跑通)
───────────────────────                 ──────────────────
✅ 生成全局参考轨迹 (MINCO)             ✅ MPC 轨迹跟踪
✅ 多项式轨迹, 任意时刻查询 pos/vel/acc  ✅ 全身轨迹代价 (已加入)
✅ 碰撞检测完备 (ESDF)                  ✅ 动力学约束优化
✅ 前端搜索 (Hybrid A* + RRT)           ✅ SQP/iLQR 求解器
```

### 4.2 推荐架构

**核心思想**：用 REMANI 做全局规划（低频 ~1Hz），用 OCS2 MPC 做局部跟踪 + 动态避障（高频 ~50Hz）。

```
┌─────────────────────────────────────────────────────────────────┐
│                        完整系统架构                              │
│                                                                 │
│  感知 (RGB-D / LiDAR)                                            │
│       │                                                         │
│       ├──────────────────────────────────────┐                   │
│       ▼                                      ▼                   │
│  ┌──────────────┐                    ┌──────────────┐            │
│  │  ESDF Map    │                    │  障碍物检测   │            │
│  │ (GridMap)    │                    │ (动态障碍物)  │            │
│  └──────┬───────┘                    └──────┬───────┘            │
│         │                                   │                    │
│         ▼                                   │                    │
│  ┌──────────────────────┐                   │                    │
│  │  REMANI-Planner      │                   │                    │
│  │  (全局规划, ~1Hz)    │                   │                    │
│  │                      │                   │                    │
│  │  Hybrid A* + RRT     │                   │                    │
│  │  → MINCO 全状态轨迹  │                   │                    │
│  └──────────┬───────────┘                   │                    │
│             │                               │                    │
│             │ 参考轨迹                       │                    │
│             ▼                               │                    │
│  ┌──────────────────────┐                   │                    │
│  │  Trajectory Sampler  │                   │                    │
│  │                      │                   │                    │
│  │  MINCO → OCS2 格式   │                   │                    │
│  │  均匀 Δt 采样        │                   │                    │
│  │  全状态序列           │                   │                    │
│  └──────────┬───────────┘                   │                    │
│             │                               │                    │
│             │ TargetTrajectories            │                    │
│             ▼                               ▼                    │
│  ┌──────────────────────────────────────────────────┐            │
│  │              OCS2 MPC (局部跟踪+避障, ~50Hz)      │            │
│  │                                                  │            │
│  │  代价函数:                                        │            │
│  │  ┌──────────────────────────────────────────┐    │            │
│  │  │ J = (x - x_ref)ᵀQ(x - x_ref)  跟踪误差    │    │            │
│  │  │   + (u - u_ref)ᵀR(u - u_ref)  输入代价    │    │            │
│  │  │   + J_whole_body              全身代价(已有)│    │            │
│  │  │   + J_dynamic_obs  ← NEW      动态避障代价  │    │            │
│  │  │   + J_joint_limits            关节极限代价  │    │            │
│  │  │   + J_smoothness              输入平滑代价  │    │            │
│  │  └──────────────────────────────────────────┘    │            │
│  │                                                  │            │
│  │  约束:                                            │            │
│  │  ┌──────────────────────────────────────────┐    │            │
│  │  │ · 动力学约束 (系统方程)                    │    │            │
│  │  │ · 碰撞约束 (ESDF 距离 ≥ safe_margin)      │    │            │
│  │  │ · 关节位置/速度/加速度约束                  │    │            │
│  │  └──────────────────────────────────────────┘    │            │
│  │                                                  │            │
│  │  输出: 最优控制序列 u₀, u₁, ..., u_{N-1}         │            │
│  └──────────────────────┬───────────────────────────┘            │
│                         │                                         │
│                         │ 关节速度/力矩指令                         │
│                         ▼                                         │
│                  ┌──────────────┐                                  │
│                  │ 机器人底层控制 │                                 │
│                  └──────────────┘                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 4.3 动态避障集成方案

#### 方案 A：代价函数法（推荐先做）

> 在 OCS2 的代价函数中加入基于 ESDF 的避障惩罚项。优势：实现简单，不需要处理约束可行性问题。

```cpp
// 在 OCS2 的自定义代价函数中添加
// 位置: OCS2 的 IntermediateCost 或自定义 CostTerm

#include <ocs2_core/cost/StateInputCost.h>
#include "remani_map/grid_map.hpp"  // 使用迁移后的 ESDF 地图

class DynamicObstacleCost final : public ocs2::StateInputCost {
public:
    DynamicObstacleCost(std::shared_ptr<GridMap> grid_map,
                        std::shared_ptr<MMConfigBase> mm_config)
        : grid_map_(grid_map), mm_config_(mm_config) {}

    // 核心：计算当前状态下的避障代价
    ocs2::scalar_t getValue(
        ocs2::scalar_t time,
        const ocs2::vector_t& state,
        const ocs2::vector_t& input,
        const ocs2::TargetTrajectories& target) const override
    {
        // 1. 从 state 中提取基底位置和关节角
        Eigen::Vector2d base_pos = state.head<2>();
        double yaw = state(2);
        Eigen::VectorXd joint_pos = state.segment(3, N_JOINTS);

        double total_cost = 0.0;

        // 2. 底盘避障代价
        //    (与 REMANI 的 obstacleGradCostforMM 思路相同)
        std::vector<Eigen::Vector3d> car_pts;
        mm_config_->getCarPts(
            Eigen::Vector3d(base_pos.x(), base_pos.y(), yaw),
            car_pts);

        for (const auto& pt : car_pts) {
            double dist = grid_map_->getPreciseDistance(pt);
            if (dist < safe_margin_) {
                // 二次惩罚: cost = w * (safe_margin - dist)²
                double penetration = safe_margin_ - dist;
                total_cost += wei_obs_car_ * penetration * penetration;
            }
        }

        // 3. 机械臂避障代价
        total_cost += computeManiObsCost(base_pos, yaw, joint_pos);

        // 4. 自碰撞代价
        total_cost += computeSelfCollisionCost(joint_pos);

        return total_cost;
    }

    // 梯度（供 OCS2 的 SQP/iLQR 使用）
    ocs2::VectorFunctionLinearApproximation getQuadraticApproximation(
        ocs2::scalar_t time,
        const ocs2::vector_t& state,
        const ocs2::vector_t& input,
        const ocs2::TargetTrajectories& target) const override
    {
        // 使用有限差分或解析梯度
        // 如果使用 ESDF，可以通过 trilinear interpolation 获取梯度
        // grid_map_->evaluateEDTWithGrad(pos, dist, grad);
        //
        // 如果不提供梯度，OCS2 会自动进行数值差分
    }

private:
    std::shared_ptr<GridMap> grid_map_;
    std::shared_ptr<MMConfigBase> mm_config_;

    double safe_margin_ = 0.10;
    double wei_obs_car_ = 1000.0;
    double wei_obs_mani_ = 1000.0;
    double wei_self_collision_ = 500.0;
};
```

**在 OCS2 中注册自定义代价**：

```cpp
// 在你的 OCS2 初始化和设置代码中
auto grid_map = std::make_shared<GridMap>();
grid_map->initMap(node);

auto mm_config = std::make_shared<MyRobotConfig>();
mm_config->setParam(node);

// 创建自定义避障代价
auto obstacle_cost = std::make_shared<DynamicObstacleCost>(
    grid_map, mm_config);

// 添加到 OCS2 的 cost collection
problem.costCollection.add(
    "dynamic_obstacle_cost", obstacle_cost);
```

#### 方案 B：硬约束法（更安全但更复杂）

```cpp
// 将碰撞作为不等式约束 g(x) >= 0 加入 OCS2
// 其中 g(x) = min_distance(x) - safe_margin

#include <ocs2_core/constraint/StateInputConstraint.h>

class CollisionConstraint final : public ocs2::StateInputConstraint {
public:
    // 约束数量 (每个碰撞球一个约束, 或取 min)
    size_t getNumConstraints(ocs2::scalar_t time) const override {
        return 1;  // 取所有碰撞球的最小距离
    }

    // 约束值: g(x, u) >= 0
    // g = min_dist - safe_margin
    ocs2::vector_t getValue(
        ocs2::scalar_t time,
        const ocs2::vector_t& state,
        const ocs2::vector_t& input,
        const ocs2::TargetTrajectories& target) const override
    {
        double min_dist = computeMinDistanceToAllObstacles(state);
        Eigen::Matrix<ocs2::scalar_t, 1, 1> g;
        g(0) = min_dist - safe_margin_;
        return g;
    }

    // 约束的线性近似 (供 SQP)
    ocs2::VectorFunctionLinearApproximation getLinearApproximation(
        ocs2::scalar_t time,
        const ocs2::vector_t& state,
        const ocs2::vector_t& input,
        const ocs2::TargetTrajectories& target) const override
    {
        // 用 ESDF 的梯度信息
        // ...
    }
};
```

### 4.4 推荐实施顺序

```
Week 1-2:  REMANI-Planner 的 ROS2 基础迁移
          最低目标: GridMap + MMConfig 能跑通 (地图 + 运动学)

Week 3:   实现 Trajectory Sampler
          MINCO 轨迹 → OCS2 TargetTrajectories 格式
          验证: mpc 能跟踪 REMANI 生成的参考轨迹

Week 4:   实现方案A (代价函数法动态避障)
          - 从 GridMap 获取 ESDF 距离/梯度
          - 在 OCS2 的 IntermediateCost 添加避障项
          - 仿真验证: 底盘在静态障碍物中实时避障

Week 5:   加入机械臂避障
          - 球体碰撞检测移植 (MMConfig 的 4 层碰撞逻辑)
          - 在 OCS2 代价中完整添加 mani_obs + self_collision

Week 6:   调参 & 测试
          - 权重调整: 跟踪精度 vs 避障
          - MPC 求解失败的回退策略
          - 实机 / 高保真仿真测试
```

### 4.5 关键注意事项

#### 1. 时间同步

REMANI 的 MINCO 轨迹自带时间信息，需要准确同步到 OCS2 的 MRT：

```cpp
// MINCO → OCS2 TargetTrajectories
Trajectory<7> minco_traj = singul_traj_data.singul_traj[i].traj;
double start_time = singul_traj_data.start_time;
double dt = 0.02;  // MPC 控制周期

std::vector<ocs2::scalar_t> times;
std::vector<ocs2::vector_t> states, inputs;

for (double t = 0; t < minco_traj.getTotalDuration(); t += dt) {
    times.push_back(start_time + t);

    ocs2::vector_t state(N_STATE);
    state << minco_traj.getPos(t),      // x,y + joint angles
             minco_traj.getVel(t);      // vx,vy + joint velocities
    states.push_back(state);

    // 从轨迹中可以推导 feedforward input
    ocs2::vector_t input(N_INPUT);
    // ... 根据动力学模型计算
    inputs.push_back(input);
}

ocs2::TargetTrajectories target(times, states, inputs);
```

#### 2. Singul 切换处理

基底前进/后退切换点是轨迹中的一个"奇点"（速度≈0），需要特别注意：

- **方案1**：在切换点分裂轨迹 → 每段单独传给 OCS2
- **方案2**：在切换点附近降低 MPC 的跟踪权重
- **方案3**：直接在 OCS2 的系统模型中支持前后运动（使用速率的绝对值作为优化变量）

#### 3. ESDF 梯度用于 MPC

```cpp
// GridMap 提供解析梯度，不要用数值差分
// ESDF 的三线性插值天然支持梯度计算

// REMANI 中已有的高效实现:
pair<double, Vector3d> evaluateEDTWithGrad(const Vector3d& pos,
                                            double& dist,
                                            Vector3d& grad) {
    // 1. 找到周围 8 个网格点
    // 2. 对它们的 ESDF 值做三线性插值
    // 3. 同时对插值系数求导得到梯度
    // 时间复杂度: O(1)
}
```

#### 4. 频率规划

| 模块 | 频率 | 说明 |
|:---|:---:|:---|
| REMANI 全局规划 | ~1 Hz | 收到新目标或检测到当前路径不可行时 |
| REMANI 局部重规划 | ~5 Hz | 在 EXEC_TRAJ 中定期检查 |
| OCS2 MPC | ~50 Hz | 实时轨迹跟踪 + 局部避障 |
| 碰撞检测 | ~100 Hz | 单独线程回调 |
| ESDF 更新 | ~10 Hz | GridMap 定时更新 |

#### 5. 失败回退策略

```
MPC求解成功?
  ├── 是 → 执行控制量
  └── 否 → 检测原因
       ├── 约束不可行 (碰撞无法避免)
       │   └── 触发 REMANI 全局重规划
       │       同时执行 MPC 的零速度 fallback
       │
       └── 求解器超时/不收敛
           └── 使用上一次的最优控制序列
              (warm-start from previous solution)
```

---

## 五、总结

### 组件角色映射

| REMANI-Planner 组件 | 在 ROS2+OCS2 架构中的角色 | 状态 |
|:---|:---|:---:|
| **KinoAstar + RRT** | 保留 → 全局前端搜索 | 迁移 |
| **MINCO 轨迹优化** | 保留 → 全局参考轨迹生成 | 迁移 |
| **GridMap (ESDF)** | 保留 → MPC 的碰撞查询模块 | 迁移 |
| **MMConfig (碰撞检测)** | 保留 → 移植到 OCS2 代价函数/约束中 | 迁移+适配 |
| **FSM** | 重构 → ROS2 Action Server + Lifecycle Node | 重写 |
| **mm_controller** | 替换 → OCS2 MPC | 替换 |

### 核心架构

```
REMANI (全局规划, ~1Hz)
    │
    │ MINCO 参考轨迹
    ▼
Trajectory Sampler
    │
    │ OCS2 TargetTrajectories
    ▼
OCS2 MPC (局部跟踪+避障, ~50Hz)
    │
    │ 关节控制量
    ▼
机器人底层控制器
```

### 关键里程碑

- ✅ **M1**：ROS2 基础迁移 (GridMap + MMConfig 跑通)
- ✅ **M2**：MINCO → OCS2 轨迹桥接
- ✅ **M3**：MPC + ESDF 动态避障 (方案A)
- ✅ **M4**：全身避障 (底盘+机械臂) + 实机验证

---

*文档生成日期：2026-07-22*

---

## 六、RViz2 可视化代码与数据流

本节专门对应 `plan_manage/launch/exp0.rviz` 和 `exp1.rviz` 中启用的
Display。两份配置的可视化 topic 基本相同，固定坐标系都是 `world`。

### 6.1 RViz2 topic 与代码映射

| RViz Display | Topic | 发布位置 | 数据含义 |
|---|---|---|---|
| 当前机器人模型 | `/model_vis/vis_mm` | `REMANIReplanFSM::publishRobotModel()` | 当前里程计和关节状态经过 MMConfig 正运动学得到的底盘/机械臂 mesh |
| 局部起点/终点 | `/kinoastar/local_start_goal` | `KinoAstar` + `MMConfig::visMM()` | Hybrid A* 本次搜索的 start/goal mesh |
| 前端全身 mesh | `/front_end_mm_mesh_vis` | `PolyTrajOptimizer::displayFrontEndMesh()` | A* + 机械臂采样得到的离散可行路径 |
| 后端全身 mesh | `/back_end_mm_mesh_vis` | `PolyTrajOptimizer::displayBackEndMesh()` | MINCO/L-BFGS 优化后轨迹的采样状态 |
| 全局/局部轨迹 | `/global_traj` | `PlanningVisualization::displayGlobalTraj()` | 蓝色 `LINE_STRIP`，新目标时显示全局轨迹，局部重规划成功后复用该 topic |
| Kinodynamic A* 路径 | `/kinoastar/path` | `KinoAstar::visPath()` | 前端路径点或路径折线；`SPHERE_LIST`/`LINE_STRIP` 由参数决定 |
| 优化控制点 | `/optimal_ctrl_pts` | `PlanningVisualization::displayOptimalCtrlPts()` | 后端优化得到的控制点，红色半透明点集 |
| 仿真地图 | `/map_generator/global_cloud` | 外部 map generator | RViz 直接显示的环境点云，不是 `GridMap::publishMap()` 的内部占据云 |
| 栅格占据云 | `grid_map/occupancy` | `GridMap::publishMap()` | 当前局部 occupancy buffer 的 `PointCloud2` |
| 膨胀占据云 | `grid_map/occupancy_inflate` | `GridMap::publishMapInflate()` | 碰撞检测使用的膨胀体素 |
| ESDF 可视化 | `grid_map/esdf` | `GridMap::publishESDF()` | `PointCloud2`，距离值编码在 intensity 字段 |

`exp*.rviz` 当前默认显示了仿真全局点云；后三个 `grid_map/*` topic 已由代码
发布，但若要查看内部 occupancy/ESDF，需要在 RViz 中另加 PointCloud2 Display。

### 6.2 端到端可视化 pipeline

```text
传感器/仿真点云 + 里程计 + JointState
             │
             ├── GridMap::cloudCallback()/cloudOdomCallback()
             │       └── occupancy buffer → ESDF → PointCloud2
             │
             └── REMANIReplanFSM::mmCarOdomCallback()/mmManiOdomCallback()
                     └── 当前状态 → publishRobotModel()
                                      └── MMConfig::getMMMarkerArray()
                                          → /model_vis/vis_mm

RViz 2D Nav Goal (/move_base_simple/goal)
             │
             └── waypointCallback() → planNextWaypoint()
                     → MMPlannerManager::reboundReplan()
                         ├── KinoAstar::visPath()
                         │    └── /kinoastar/path
                         ├── displayAStarList()/displayInitWaypoints()
                         │    └── /a_star_list、/init_waypoints（调试 topic）
                         ├── displayFrontEndMesh()
                         │    └── /front_end_mm_mesh_vis
                         ├── OptimizeTrajectory_lbfgs()
                         └── displayBackEndMesh()
                              └── /back_end_mm_mesh_vis
```

### 6.3 重要类和方法

#### `REMANIReplanFSM`

- `init()`：创建可视化 publisher 和 50 ms 的模型发布定时器；模型使用
  `reliable + transient_local`，因此 RViz 后启动也能收到最近一帧。
- `waypointCallback()`：接收 RViz 的 `PoseStamped`，提取 x、y 和四元数 yaw，
  触发全局/局部规划。
- `publishRobotModel()`：读取最新底盘/关节状态，调用 MMConfig 生成 mesh
  `MarkerArray`。它显示的是“当前实测状态”，不是规划预测状态。
- `callReboundReplan()`：规划成功后采样 `SingulTrajData`，调用
  `displayGlobalTraj()` 和 `displayBackEndMesh()`。

#### `MMPlannerManager`

- `initPlanModules()`：创建 `GridMap`、`MMConfig` 和 `PolyTrajOptimizer`，形成
  可视化所需的地图、机器人模型和优化器依赖。
- `computeInitReferenceState()`：前端搜索成功后提取 path、waypoint、control
  point，并分别交给 `PlanningVisualization` 和 `displayFrontEndMesh()`。
- `reboundReplan()`：串起前端初值、后端优化、轨迹容器写入和发布前的可视化。

#### `PlanningVisualization`

- `displayMarkerList()`：把 `Eigen::Vector3d` 列表编码为 `SPHERE_LIST` 或
  `LINE_STRIP`；颜色和尺寸由调用方传入，统一使用 `world` frame。
- `displayGlobalTraj()`：把二维轨迹补成 z=0 的三维点并发布蓝色折线。
- `displayOptimalCtrlPts()` / `displayOptWaypoints()`：发布优化控制点和
  waypoint；当 `id==0` 时先发送 `DELETEALL` 清除上一轮结果。
- `displayArrowList()` / `displayIntermediateGrad()`：使用 `MarkerArray` 的
  ARROW 表示优化梯度；箭头输入按 start/end 成对排列。

#### `KinoAstar`

- `visPath(path, pt)`：前端路径的唯一可视化入口；`pt=true` 显示离散点，
  `pt=false` 显示折线。
- `displayLocalStartGoal()`：发布起点/终点的 mesh marker；真正的 mesh 组装由
  `MMConfig` 完成。
- `KinoAstarSearchAndGetSimplePath()`：搜索成功后采样基底路径并调用
  `visPath()`，之后再进入机械臂构型采样。

#### `PolyTrajOptimizer`

- `displayFrontEndMesh()`：把前端路径的 `[x,y,joint...]` 与 yaw 逐点转成
  `MarkerArray`，使用 `vis_mm_front_end` namespace。
- `displayBackEndMesh()`：按优化轨迹时间采样状态，再将每个采样状态的 mesh
  累积发布到 `/back_end_mm_mesh_vis`。

#### `GridMap`

- `visCallback()`：每 50 ms 调用三个发布函数；发布函数首先检查订阅者数量，
  没有 RViz Display 时避免遍历体素。
- `publishMap()` / `publishMapInflate()`：遍历局部体素窗口，分别读取原始和
  膨胀 occupancy，转换为 PCL 点云后发布 `PointCloud2`。
- `publishESDF()`：在固定 z 切片查询 ESDF，将距离裁剪到 [-3,3] m 后归一化到
  intensity，RViz 用颜色变换器显示距离。

### 6.4 Marker 的 ID、namespace 和刷新规则

- `MMConfig` 使用 `idx * vis_idx_size_ + offset`，其中 `vis_idx_size_=100`；
  offset 0 预留给底盘，11–20 预留给机械臂连杆/夹爪。
- `PlanningVisualization::displayMarkerList()` 为点和线使用 `id` 与 `id+1000`，
  防止同一 topic 内互相覆盖。
- `KinoAstar` 的显示函数会先发布 `DELETEALL`，因此适合显示“当前一次搜索”。
- mesh topic 使用 MarkerArray；前端/后端函数通过固定 namespace 和 ID 覆盖旧
  模型。若修改采样数量，应同步考虑旧 ID 是否需要显式删除。

### 6.5 阅读和调试建议

1. 先确认 `world` TF 和 `/model_vis/vis_mm`，验证机器人模型坐标和 mesh URI。
2. 再打开 `/map_generator/global_cloud` 或 `grid_map/occupancy`，确认地图 frame
   与 RViz Fixed Frame 一致。
3. 发送一个 2D Nav Goal，按 `/kinoastar/local_start_goal` → `/kinoastar/path`
   → `/front_end_mm_mesh_vis` → `/back_end_mm_mesh_vis` 的顺序检查 pipeline。
4. 如果轨迹不显示，优先检查 `get_subscription_count()`：很多显示函数在没有
   RViz 订阅者时会直接返回；其次检查 topic 名称、Marker frame_id 和 ID 是否冲突。
5. 如果模型只在启动时出现，检查 QoS：当前机器人模型是 transient-local，而
   大多数调试 Marker 是 volatile，后者需要在 RViz 启动后重新触发一次规划。
