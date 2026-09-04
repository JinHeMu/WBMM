# OCS2 + REMANI 移动机械臂 MPC 工程全面指南

> Tracer 差速底盘 + JAKA 6-DOF 机械臂的全方位轨迹规划与模型预测控制系统
>
> 📖 快速命令速查见 [QUICKSTART.md](QUICKSTART.md)。

---

## 目录

1. [工程概述](#1-工程概述)
2. [整体 Pipeline](#2-整体-pipeline)
3. [状态与控制量映射](#3-状态与控制量映射)
4. [源码包详解](#4-源码包详解)
   - [4.1 ocs2_ros2 — OCS2 MPC 框架](#41-ocs2_ros2--ocs2-mpc-框架)
   - [4.2 tracer_jaka — 机器人端实现](#42-tracer_jaka--机器人端实现)
   - [4.3 remani_planner — 全局规划器](#43-remani_planner--全局规划器)
   - [4.4 传感器与驱动包](#44-传感器与驱动包)
5. [启动文件全览](#5-启动文件全览)
6. [配置文件参考](#6-配置文件参考)
7. [关键设计细节](#7-关键设计细节)
8. [运行说明](#8-运行说明)
9. [仿真环境](#9-仿真环境)
10. [nvblox 3D ESDF 建图 (Isaac ROS)](#10-nvblox-3d-esdf-建图-isaac-ros)
   - [10.1 概述](#101-概述)
   - [10.2 包结构](#102-包结构)
   - [10.3 核心组件详解](#103-核心组件详解)
   - [10.4 启动文件详解](#104-启动文件详解)
   - [10.5 关键消息与服务](#105-关键消息与服务)
   - [10.6 与 OCS2 系统的集成计划](#106-与-ocs2-系统的集成计划)
   - [10.7 nvblox 关键话题速查](#107-nvblox-关键话题速查)
   - [10.8 Nav2 Costmap 集成](#108-nav2-costmap-集成)
   - [10.9 性能考量](#109-性能考量)
   - [10.10 上游 nvblox 生态](#1010-上游-nvblox-生态)

---

## 1. 工程概述

本工程是一个基于 ROS 2 Humble 的**移动机械臂全身模型预测控制**系统，包含以下核心能力：

- **全局轨迹规划**（REMANI）：使用 Kino-A\* 搜索 + MINCO 多项式优化生成无碰撞全身轨迹
- **局部 MPC 跟踪**（OCS2）：使用 SLQ/DDP 算法对规划轨迹进行带约束的实时跟踪
- **桥接层**：将 REMANI 的平坦输出多项式轨迹转换为 OCS2 全状态目标轨迹
- **仿真环境**：MuJoCo 物理引擎 + ros2_control，支持 LiDAR/IMU/深度相机等传感器仿真
- **实机支持**：Tracer 底盘 CAN 驱动、JAKA 机械臂驱动、IMU、Lakibeam 激光雷达

### 机器人模型

| 部件 | 型号 | 关节数 | 驱动 |
|------|------|--------|------|
| 底盘 | Tracer (差速) | 3 (x, y, yaw 平面运动) | CAN 总线 |
| 机械臂 | JAKA Zu 5 | 6 (joint_1~joint_6) | Ethernet |
| 激光雷达 | Lakibeam1 | — | Ethernet |
| IMU | Hipnuc | — | USB/串口 |

---

## 2. 整体 Pipeline

```
┌──────────────────────────────────────────────────────────────────────┐
│                         PLAN-THEN-TRACK 架构                         │
└──────────────────────────────────────────────────────────────────────┘

  REMANI Planner (全局, 低频 ~1-5 Hz)
  │   输入: ESDF 地图 + 目标位姿 + 当前全身状态
  │   输出: PolynomialTraj [x, y, q1..q6] (8维分段多项式)
  │
  ▼
  RemaniToOcs2ReferenceBridge
  │   - 多项式解析: 位置/速度/加速度
  │   - yaw 重建: atan2(singul*vy, singul*vx)
  │   - v/ω 计算: v=s*|vel|, ω=(vx·ay-vy·ax)/(vx²+vy²)
  │   - 时间同步: REMANI ROS 时间 → OCS2 内部时间
  │   - 零速退化处理 + yaw 解包
  │
  ▼
  TargetTrajectories: 9维 state [x, y, yaw, q1..q6] + 8维 input [v, ω, q̇1..q̇6]
  │   Topic: /mobile_manipulator_mpc_target
  │
  ▼
  RosReferenceManager
  │
  ├── WholeBodyTrajectoryCost  ← 状态跟踪代价 (Q矩阵)
  └── QuadraticInputCost       ← 输入代价 (R矩阵)
  │
  ▼
  OCS2 MPC (SLQ/DDP 求解器)
  │   约束: 自碰撞、环境避障、关节位置/速度限制
  │   输出: /mobile_manipulator_mpc_policy
  │
  ▼
  MRT (Mode-Reference Trajectory) 控制器桥
  │   输入: odom + /joint_states → 状态估计
  │   输出: Twist (底盘) + Float64MultiArray (机械臂)
  │
  ▼
  Tracer 底盘 + JAKA 机械臂 (MuJoCo 仿真 或 真实硬件)
```

### 数据流细节

```
REMANI PolynomialTraj (分段, 按 gear 方向)
  │
  ├── trajectory_id=1: 新轨迹开始, 清空旧缓存
  ├── trajectory_id=2,3,...: 追加分段
  │
  ▼
Bridge 内部 debounce timer (40ms 超时 = 接收完成)
  │
  ▼
采样 (dt=0.04s, horizon=3.0s, hold_at_end=2.0s)
  │
  ▼
OCS2 MPC (100Hz 求解, horizon=2.0s)
  │
  ▼
MRT (125Hz sim / 100Hz real)
  │
  ▼
底盘: /base_controller/cmd_vel (sim) 或 /cmd_vel (real)
机械臂: /arm_controller/commands (sim) 或 /arm_controller/commands (real)
```

---

## 3. 状态与控制量映射

| 内容 | REMANI | OCS2 |
|------|--------|------|
| 轨迹状态 | `[x, y, q1..q6]` (8维) | `[x, y, yaw, q1..q6]` (9维) |
| 控制/导数 | `[vx, vy, q̇1..q̇6]` | `[v, ω, q̇1..q̇6]` (8维) |
| 行驶方向 | `singul = +1/-1` | `v` 的正负号 |
| 航向 | 由速度和 `singul` 恢复 | 显式 yaw 状态 |

### 运动学模型 (差速底盘)

```
ẋ  = v · cos(yaw)
ẏ  = v · sin(yaw)
ẏaw = ω
q̇  = u_arm
```

### 桥接节点中的 yaw/v/ω 重建公式

```
yaw = atan2(s · vy, s · vx)    其中 s = singul
v   = s · √(vx² + vy²)
ω   = (vx · ay - vy · ax) / (vx² + vy²)
```

---

## 4. 源码包详解

### 4.1 ocs2_ros2 — OCS2 MPC 框架

上游 OCS2 库的 ROS 2 移植版本，包含 git submodules，是整个系统的核心求解器框架。**极少直接修改此部分代码**。

```
src/vendor/ocs2_ros2/
├── core/
│   ├── ocs2_core/          # 核心数据结构: 状态/输入向量、代价函数、约束、数值积分
│   ├── ocs2_oc/            # 最优控制: Lagrangian、Hamiltonian、滚动时域
│   └── ocs2_thirdparty/    # 第三方依赖
├── mpc/
│   ├── ocs2_ddp/           # DDP/SLQ 求解器 (GaussNewtonDDP_MPC)
│   ├── ocs2_mpc/           # MPC 基类 (MPC_BASE)
│   ├── ocs2_ipm/           # 内点法求解器
│   ├── ocs2_slp/           # SLP (Sequential Linear Programming)
│   └── ocs2_qp_solver/     # QP 子问题求解器
├── robotics/
│   ├── ocs2_ros_interfaces/  # ROS 2 接口层:
│   │   ├── MPC_ROS_Interface    — MPC 节点 (发布 policy, 订阅 observation)
│   │   ├── RosReferenceManager  — 订阅 /..._mpc_target, 管理参考轨迹
│   │   ├── TargetTrajectoriesRosPublisher — 发布参考轨迹
│   │   └── MRT_ROS_Interface    — MRT 控制器桥 ROS 接口
│   ├── ocs2_msgs/           # 自定义消息: MpcObservation, MpcTargetTrajectories 等
│   ├── ocs2_robotic_tools/  # 运动学/动力学工具
│   └── ocs2_python_interface/  # Python 绑定
├── basic examples/
│   └── ocs2_mobile_manipulator/  # 移动机械臂专用模块:
│       ├── MobileManipulatorInterface       — 加载 task.info, 创建所有组件
│       ├── WheelBasedMobileManipulatorDynamics — 差速底盘动力学 (ẋ=v·cos(yaw)...)
│       ├── WholeBodyTrajectoryCost           — 全身轨迹跟踪代价 (状态线性插值)
│       ├── QuadraticInputCost                — 输入二次代价
│       ├── EndEffectorCost                   — 末端执行器代价
│       ├── SelfCollisionConstraint           — 自碰撞软约束
│       ├── EnvironmentCollisionConstraint    — 环境避障软约束
│       ├── JointPositionLimits               — 关节位置限制
│       └── JointVelocityLimits               — 关节速度限制
└── submodules/               # 子模块 (plane_segmentation_ros2 等)
```

**关键类说明：**

| 类 | 功能 |
|----|------|
| `MobileManipulatorInterface` | 加载 task.info + URDF, 构造 DDP settings、rollout、OC problem、cost/constraint |
| `GaussNewtonDDP_MPC` | SLQ (Sequential Linear Quadratic) DDP 求解器 |
| `MPC_ROS_Interface` | 将 MPC 求解器发布为 ROS topic，接收 observation |
| `RosReferenceManager` | 管理参考轨迹，订阅 `/mobile_manipulator_mpc_target` |
| `WholeBodyTrajectoryCost` | 全状态跟踪代价，内部对参考进行线性插值 (含 yaw 最短角) |
| `MRT_ROS_Interface` | MRT 控制器：将 MPC 策略转换为硬件命令 |

---

### 4.2 tracer_jaka — 机器人端实现

这是本工程的核心代码区，包含所有 ROS 2 可执行节点和配置文件。

```
src/
├── algorithms/control/tracer_jaka_ocs2/   # ★ 主包: MPC/MRT节点 + 桥接 + 手柄控制 + 配置
├── simulation/tracer_jaka_mujoco/         # MuJoCo 仿真: 桥接节点 + 场景模型 + SLAM + EKF
├── robot/tracer_jaka_moveit_config/       # MoveIt2 配置 (单独使用, 不与 MPC 同时运行)
├── perception/grid_map/                   # GridMap / ESDF 工具包:
│   ├── mjcf_to_esdf          — MuJoCo 场景 → ESDF NPZ 转换工具
│   └── maps/                 — 预生成的 ESDF 地图文件
├── perception/esdf_simple_nav/            # 基于 ESDF 的简单导航 (实验性)
├── drivers/arm/jaka_driver_tools/         # JAKA 机械臂 ROS 2 驱动/工具
├── robot/tracer_jaka_description/         # JAKA/Tracer-JAKA URDF 描述（统一入口）
├── drivers/arm/jaka_hardware_interface/   # JAKA ros2_control 硬件接口
├── drivers/base/tracer_base/              # Tracer 底盘驱动
└── drivers/arm/dh_ag_ros2/                # DH AG95 夹爪驱动
```

#### 4.2.1 tracer_jaka_ocs2 — 可执行节点一览

| 可执行文件 | 源文件 | 行数 | 功能 |
|-----------|--------|------|------|
| `tracer_jaka_mpc_node` | `TracerJakaMpcNode.cpp` | ~100 | MPC 求解器: 加载 task.info+URDF, 创建 `MobileManipulatorInterface`, 启动 SLQ/DDP MPC |
| `tracer_jaka_mrt_node` | `TracerJakaMrtNode.cpp` | ~1460 | MRT 控制器桥: 接收 `odom`+`/joint_states` 估计状态, 将 MPC policy 转为 `Twist`+`Float64MultiArray` |
| `tracer_jaka_target_node` | `TracerJakaTargetNode.cpp` | ~200 | RViz 交互式目标: 创建 Interactive Marker, 右键 "Send target" |
| `tracer_jaka_joy_target_node` | `tracer_jaka_joy_target_node.cpp` | ~550 | 手柄遥操作: 发送静态目标位姿 (Set-Point) |
| `tracer_jaka_joy_whole_body_node` | `tracer_jaka_joy_whole_body_node.cpp` | ~450 | 手柄 "胡萝卜" 模式: 发布短时全身轨迹 (velocity-based waypoints) |
| `remani_to_ocs2_reference_bridge` | `remani_to_ocs2_reference_bridge.cpp` | ~924 | **★ REMANI→OCS2 桥接**: 多项式解析+yaw重建+分段拼接+时间同步+参考发布 |
| `tracer_jaka_whole_body_trajectory_node` | `tracer_jaka_whole_body_trajectory_node.cpp` | ~750 | CSV 轨迹回放: 读取离线全身轨迹 CSV 并发布为 OCS2 参考 |
| `csv_path_visualizer_node` | `csv_path_visualizer_node.cpp` | ~220 | CSV 路径可视化: 在 RViz 中显示 CSV 文件路径 |
| — | `TracerJakaVisualization.cpp` | ~500 | MPC 状态可视化: 发布预测轨迹、约束状态等 RViz marker |

#### 4.2.2 tracer_jaka_mujoco — 仿真环境

| 组件 | 文件 | 功能 |
|------|------|------|
| MuJoCo 桥接 | `mujoco_bridge` 可执行 | 连接 MuJoCo 物理引擎与 ROS 2: 读取 MJCF 模型, 发布 /joint_states, /wheel/odometry, /imu/data, /scan, 订阅 /cmd_vel 和 /arm_controller/commands |
| 场景模型 | `models/scene.xml` | MuJoCo 场景: 墙壁、低桌障碍物、机器人模型 include |
| 机器人模型 | `models/tracer_jaka_zu5.xml` | 机器人 MJCF: 底盘平面关节(base_x/y/yaw)+6个机械臂关节+轮子+传感器 |
| 机器人子模型 | `models/tracer_jaka_zu5_robot.xml` | JAKA 机械臂的 MuJoCo 运动学定义 |
| 权威 URDF | `tracer_jaka_description/urdf/tracer_jaka_zu5.urdf` | 仿真与实机共用的几何、TF、限位和碰撞模型 |
| ros2_control xacro | `tracer_jaka_description/urdf/tracer_jaka_zu5.controlled.urdf.xacro` | 通过 `control_backend:=mujoco/real/mock` 切换后端 |
| EKF 配置 | `config/ekf.yaml` | robot_localization EKF: 融合 /wheel/odometry + /imu/data → /odometry/filtered |
| SLAM 配置 | `config/slam_toolbox.yaml` | slam_toolbox 异步 SLAM: /scan → /map |
| 传感器配置 | `config/sensors.yaml` | MuJoCo 传感器参数: LiDAR(30Hz, 270°), IMU(100Hz), D455 相机(30Hz) |
| 控制器 | `tracer_jaka_description/config/ros2_controllers.yaml` | 统一定义 `base_controller`、`arm_controller`、`arm_trajectory_controller` 与 `fts_broadcaster` |

#### 4.2.3 grid_map — ESDF 工具 (Python 包)

| 工具 | 文件 | 功能 |
|------|------|------|
| `mjcf_to_esdf` | `grid_map/mjcf_to_esdf.py` | 将 MuJoCo XML 场景中的静态几何体 (box/sphere/cylinder/capsule) 转换为 ESDF NPZ 文件 (使用 scipy distance_transform_edt) |
| `esdf_rviz_node` | `grid_map/esdf_rviz_node.py` | 将 ESDF OccupancyGrid 转换为 PointCloud2 供 RViz2 可视化 |
| `maps/` | `maps/` | 预生成的 ESDF 文件，如 `tracer_jaka_zu5_scene_esdf.npz` |

**ESDF 再生命令：**
```bash
ros2 run grid_map mjcf_to_esdf \
  --xml /absolute/path/to/scene.xml \
  --output /absolute/path/to/scene_esdf.npz
```
转换器自动排除带关节的 body (机器人模型)，仅包含静态几何体。坐标保持 MuJoCo world 系，需在启动时通过 `static_esdf_offset_*` 参数对齐到 odom 系。

#### 4.2.4 jaka_ros2 子模块 — 硬件驱动

| 包 | 功能 |
|----|------|
| `tracer_base` | Tracer 底盘 CAN 驱动: 订阅 `/cmd_vel`, 发布 `/odom` + `odom→base_footprint` TF |
| `jaka_driver` | JAKA 机械臂驱动: 提供 `arm_controller` (速度控制) 和 MoveIt 接口 |
| `jaka_hardware_interface` | JAKA ros2_control HardwareInterface 插件 |
| `tracer_jaka_description` | JAKA/Tracer-JAKA 机械臂 URDF (含 DH 参数) |
| `dh_ag95_gripper` | DH AG95 夹爪 ROS 2 驱动 |

---

### 4.3 remani_planner — 全局规划器

REMANI (REactive Mobile mAnipulator Navigation Intelligence) 是一个基于分层优化的全身轨迹规划器，由以下子包组成：

```
src/vendor/remani_planner/
├── plan_env/         # 环境表示
├── mm_config/        # 移动机械臂运动学/动力学配置
├── path_searching/   # 前端路径搜索
├── traj_opt/         # 后端轨迹优化
├── traj_utils/       # 轨迹工具/数据类型
├── quadrotor_msgs/   # 自定义消息定义
└── plan_manage/      # ★ 顶层规划管理: 节点 + 重规划 FSM + 可视化
```

#### 4.3.1 各子包功能

| 子包 | 源文件 | 功能 |
|------|--------|------|
| `plan_env` | `grid_map.cpp` | ESDF 栅格地图: 从 NPZ 静态加载或从点云动态构建 |
| | `raycast.cpp` | 光线投射: 用于碰撞检测 |
| `mm_config` | `mm_config.cpp` | 从 URDF 加载移动机械臂运动学配置 (关节轴、base_footprint→Link_0 变换、碰撞球体) |
| `path_searching` | `kino_astar.cpp` | Kino-A\* (运动学 A\*): 在 SE(2)×R⁶ 空间搜索初始路径 |
| | `rrt.cpp` | RRT 规划器 (备用) |
| | `sample_mani_RRT.cpp` | 机械臂 RRT 采样规划器 (备用) |
| `traj_opt` | `poly_traj_optimizer.cpp` | MINCO 多项式轨迹优化 (1727行): 后端优化平滑性 + 避碰 + 动力学约束, L-BFGS 求解器 |
| `traj_utils` | `poly_traj_utils.hpp`, `plan_container.hpp`, `root_finder.hpp` | 多项式轨迹数据结构 (7阶分段 MINCO), 速度/加速度极值计算 (Sturm 序列求根) |
| `plan_manage` | `remani_replan_fsm.cpp` (~1248行) | **重规划有限状态机**: WAIT_TARGET → GEN_NEW_TRAJ → EXEC_TRAJ, 跟踪误差监控, 目标到达判定 |
| | `planner_manager.cpp` (~743行) | **规划管理器**: 协调 Kino-A\* 搜索 + MINCO 优化 + 轨迹发布 |
| | `planning_visualization.cpp` (~480行) | RViz 可视化: 前端路径, 后端轨迹, 碰撞球体, A\* 搜索树, 优化梯度调试 |
| | `remani_planner_node.cpp` (~17行) | 主节点入口 |

#### 4.3.2 自定义消息

**quadrotor_msgs — 核心通信消息:**

| 消息 | 字段 | 说明 |
|------|------|------|
| `PolynomialTraj` | `header`, `trajectory_id`, `action`, `singul`, `trajectory[]` | REMANI→OCS2 桥接的主消息: 多段 8 维多项式轨迹 |
| `PolynomialMatrix` | `num_order`, `num_dim`, `data[]`, `duration` | 单段多项式的系数矩阵 (Eigen 列主序) 和时长 |
| `PolynomialTrajectory` | `trajectory_id`, `action`, `num_order`, `num_segment`, `coef_x/y/z[]`, `time[]` | 原始 3D 多项式轨迹 (用于传统四旋翼) |
| `PositionCommand` | position/velocity/acceleration/jerk, yaw, yaw_dot, kx/kv gains, trajectory_id | 位置控制命令 |
| `SO3Command` | force, orientation, kr/kom gains, AuxCommand | SO3 姿态控制器命令 |
| `TRPYCommand` | thrust, roll, pitch, yaw, AuxCommand | 推力/姿态角控制命令 |
| `OutputData` | IMU data, pressure, magnetometer, loop_rate, voltage | 飞控输出数据 (含 IMU) |
| `StatusData` | loop_rate, voltage, seq | 飞控状态数据 |
| `Serial` | channel, type, data[] | 通用串行通信消息 |
| `Odometry` | curodom, kfodom, kfid, status | 带 KF 状态的里程计 |

**Action 命令定义:**
- `ACTION_ADD` (1): 添加/追加轨迹段
- `ACTION_ABORT` (2): 紧急中止，清空轨迹
- `ACTION_WARN_START` (3) / `ACTION_WARN_FINAL` (4) / `ACTION_WARN_IMPOSSIBLE` (5): 警告信号

**traj_utils — 辅助消息:**

| 消息 | 字段 | 说明 |
|------|------|------|
| `PolyTraj` | drone_id, traj_id, start_time, order, coef_x/y/z[], duration[] | 单架飞行器多项式轨迹 (用于多机调度) |
| `DataDisp` | header, a/b/c/d/e | 通用 5 字段遥测/调试消息 |
| `Assignment` | uint32[10] assignment | 多机任务分配数组 |

#### 4.3.3 独立仿真器

`remani_simulator.py` (282行): 替代原 ROS 1 的 fake_mm/controller/local_sensing 链的 ROS 2 Python 节点。发布静态点云全局地图，直接解析 `PolynomialTraj` 消息生成理想里程计和关节状态反馈。支持两种地图类型:
- **森林模式 (exp0)**: 随机生成长方体障碍物
- **桥模式 (exp1)**: 高架桥面 + 支撑结构

#### 4.3.4 重规划 FSM 状态机

```
WAIT_TARGET
    │ 收到 /goal_pose 或 /move_base_simple/goal
    ▼
GEN_NEW_TRAJ
    │ Kino-A* 搜索 + MINCO 优化成功
    ▼
EXEC_TRAJ
    │
    ├── 跟踪误差超阈值 (>persistence) → GEN_NEW_TRAJ (以当前状态为起点)
    ├── 轨迹时间到达终点 + 目标容差满足 → WAIT_TARGET (成功)
    ├── 轨迹时间到达终点 + 目标容差不满足 → GEN_NEW_TRAJ
    └── 收到 ACTION_ABORT → WAIT_TARGET
```

**跟踪误差重规划参数 (默认值):**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `tracking_error_position_threshold` | 0.30 m | 底盘位置误差阈值 |
| `tracking_error_yaw_threshold` | 0.45 rad | 航向误差阈值 |
| `tracking_error_joint_threshold` | 0.30 rad | 关节误差阈值 |
| `tracking_error_persistence` | 0.30 s | 误差持续超过阈值才触发 |
| `tracking_error_grace_period` | 0.60 s | 新轨迹开始后的容忍窗口 |
| `tracking_error_min_interval` | 2.0 s | 两次重规划最小间隔 |
| `tracking_goal_position_tolerance` | 0.12 m | 目标位置容差 |
| `tracking_goal_yaw_tolerance` | 0.20 rad | 目标航向容差 |
| `tracking_goal_joint_tolerance` | 0.15 rad | 目标关节容差 |

---

### 4.4 传感器与驱动包

```
src/drivers/sensors/
├── hipnuc_imu/          # Hipnuc IMU 驱动 (串口, USB)
├── hipnuc_imu_can/      # Hipnuc IMU 驱动 (CAN)
└── hipnuc_lib_package/  # Hipnuc 共享 C 库: 二进制协议解码 + NMEA 解析 + CANopen/J1939 解析

src/drivers/sensors/lakibeam1/
└── lakibeam1/           # Lakibeam1 单线激光雷达驱动 (Ethernet/UDP)
```

#### 4.4.1 Hipnuc IMU 驱动

**`hipnuc_lib_package` — 共享解码库 (C)**

| 文件 | 功能 |
|------|------|
| `hipnuc_dec.c` (490行) | HiPNUC 二进制协议解码: 帧同步 (0x5A 0xA5), CRC16 校验, HI91/HI81/HI83 三种数据包 |
| `nmea_dec.c` (390行) | NMEA 语句解析: GGA (位置), RMC (最小导航), SXT (专用 INS) |
| `canopen_parser.c` (79行) | CANopen TPDO 帧解析 (加速度/陀螺/欧拉/四元数/气压/倾角) |
| `hipnuc_j1939_parser.c` (161行) | J1939 PGN 解析 (13 种 PGN: 经纬度/速度/时间/IMU/姿态等) |

**协议支持:**
| 数据包 | 标记 | 内容 |
|--------|------|------|
| HI91 | 0x91 | IMU 浮点数据 (四元数+角速度+线加速度+欧拉角) |
| HI81 | 0x81 | INS 组合导航数据 |
| HI83 | 0x83 | 位图式变长数据包 (可选字段: 加速度/陀螺/磁力计/RPY/四元数/UTC/气压/温度/GNSS/速度) |

**`hipnuc_imu` (串口驱动)**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `serial_port` | `/dev/ttyUSB0` | 串口设备 |
| `baud_rate` | `115200` | 波特率 (支持 termios2 自定义) |
| `frame_id` | `imu_link` | TF 坐标系 |

**`hipnuc_imu_can` (CAN 驱动)**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `can_port` | `can0` | CAN 接口 |
| `baud_rate` | `500000` | CAN 波特率 |
| `node_id` | `8` | CAN 节点 ID |

**发布话题 (两种驱动均):**
- `/IMU_data` — `sensor_msgs/Imu`
- `/IMU_mag` — `sensor_msgs/MagneticField`
- `/IMU_euler` — `geometry_msgs/Vector3Stamped`
- `/IMU_temp` — `sensor_msgs/Temperature` (仅串口)
- `/IMU_pressure` — `sensor_msgs/FluidPressure` (仅串口)

#### 4.4.2 Lakibeam1 激光雷达驱动

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `hostip` | `192.168.198.1` | 主机 IP |
| `sensorip` | `192.168.198.2` | 雷达 IP |
| `frame_id` | `laser_link` | 发布坐标系 |
| `scan_freq` | `30` | 扫描频率 (Hz) |
| `filter` | `3` | 滤波器等级 |
| `laser_enable` | `true` | 激光使能 |
| `scan_start` | `45.0` | 扫描起始角 (°) |
| `scan_end` | `315.0` | 扫描结束角 (°) |
| `inverted` | `false` | 倒置安装 |
| `angle_offset` | `0.0` | 角度偏移 |
| `configure_sensor` | `true` | 启动时通过 HTTP PUT 配置雷达参数 |

**通信协议:**
- **数据流**: UDP 端口 2368 (MSOP 协议: 每包 12 个 block, 每 block 16 个测距点)
- **配置**: HTTP REST API (`/api/v1/sensor/parameters/`)
- **遥测**: HTTP GET (`/api/v1/system/firmware`, `/api/v1/system/monitor`, `/api/v1/sensor/overview`)
- **USB 备用**: RNDIS `192.168.8.2`

---

## 5. 启动文件全览

### 5.1 仿真启动

#### `ocs2_sim.launch.py` — 主仿真启动 (tracer_jaka_ocs2)

**这是最完整的仿真入口**。启动顺序：

| 时序 | 组件 | 说明 |
|------|------|------|
| t=0 | `_ensure_urdf` (OpaqueFunction) | xacro→urdf 编译 |
| t=0 | `bridge.launch.py` (IncludeLaunch) | MuJoCo 物理引擎 + robot_state_publisher + ros2_control |
| t=0 | EKF + SLAM | robot_localization 融合里程计+IMU, slam_toolbox |
| t=8s | MPC + MRT | SLQ/DDP 求解器 + MRT 控制器桥 |
| t=12s | REMANI + Bridge | 全局规划器 + 多项式→OCS2 桥接 |
| t=4s | RViz | 可视化 |

**所有参数：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_sim_time` | `true` | 使用 /clock 仿真时间 |
| `viewer` | `true` | MuJoCo 原生可视化窗口 |
| `use_rviz` | `true` | 启动 RViz2 |
| `use_joy` | `true` | 启用手柄驱动 |
| `use_csv_target` | `false` | 使用 CSV 文件作为参考 (与 REMANI 互斥) |
| `start_slam` | `true` | 启动 slam_toolbox |
| `start_remani` | `true` | 启动 REMANI 规划器 + 桥接 |
| `mrt_odom_topic` | `/odometry/filtered` | MRT 使用的里程计话题 |
| `remani_static_esdf_file` | `grid_map/maps/tracer_jaka_zu5_scene_esdf.npz` | 静态 ESDF 文件 |
| `remani_static_esdf_offset_x` | `0.0` | ESDF X 偏移（MuJoCo 与 odom 同原点） |
| `remani_static_esdf_offset_y` | `0.0` | ESDF Y 偏移 |
| `remani_static_esdf_offset_z` | `0.0` | ESDF Z 偏移 |
| `task_file` | `config/task.info` | OCS2 任务配置文件 |
| `xacro_file` | `tracer_jaka_description/urdf/tracer_jaka_zu5.urdf` | 权威机器人 URDF |
| `urdf_file` | `/tmp/ocs2_tracer_jaka/tracer_jaka.urdf` | 编译后 URDF 路径 |
| `lib_folder` | `/tmp/ocs2_tracer_jaka/auto_generated` | 自动生成库路径 |

#### `bridge.launch.py` — MuJoCo 桥接 (tracer_jaka_mujoco)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `model` | `models/scene.xml` | MuJoCo 场景 XML |
| `viewer` | `true` | 可视化窗口 |
| `camera` | `false` | 是否发布 D455 RGB-D 数据 |
| `camera_rate` | `30.0` | 相机发布频率 (Hz) |

**发布的话题：**
- `/joint_states` — 所有关节状态 (含 base_x/y/yaw)
- `/wheel/odometry` — 轮式里程计
- `/imu/data` — IMU 数据
- `/scan` — 2D 激光扫描 (270°, 30Hz)
- `/clock` — 仿真时钟 (250Hz)
- `/lidar/points` — 可选点云调试

---

### 5.2 REMANI 规划器启动

#### `remani_mpc_tracking.launch.py` — REMANI + 桥接联合启动

**这是标准的 REMANI-MPC 集成启动**。同时启动规划器和桥接器。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_sim_time` | `false` | 仿真时间 |
| `start_planner` | `true` | 启动 REMANI 规划器 |
| `start_bridge` | `true` | 启动 REMANI→OCS2 桥接 |
| `urdf_file` | `tracer_jaka_description/urdf/tracer_jaka_zu5.urdf` | OCS2 使用的权威机器人 URDF |
| `static_esdf_file` | `tracer_jaka_zu5_scene_esdf.npz` | 静态 ESDF |
| `static_esdf_offset_x/y/z` | `0.0/0.0/0.0` | ESDF 偏移 |
| `odom_topic` | `/base_controller/odom` | 里程计话题 |
| `joint_state_topic` | `/joint_states` | 关节状态话题 |
| `planner_to_ocs2_x/y/yaw` | `0.0/0.0/0.0` | 规划器到 OCS2 坐标变换 |
| `tracking_error_replan_enabled` | `true` | 跟踪误差重规划开关 |
| `tracking_error_position_threshold` | `0.30` | 位置误差阈值 (m) |
| `tracking_error_yaw_threshold` | `0.45` | 航向误差阈值 (rad) |
| `tracking_error_joint_threshold` | `0.30` | 关节误差阈值 (rad) |
| `tracking_error_persistence` | `0.30` | 误差持续时间 (s) |
| `tracking_error_min_interval` | `2.0` | 最小重规划间隔 (s) |
| `tracking_error_grace_period` | `0.60` | 新轨迹容忍窗口 (s) |
| `tracking_goal_position_tolerance` | `0.12` | 目标位置容差 (m) |
| `tracking_goal_yaw_tolerance` | `0.20` | 目标航向容差 (rad) |
| `tracking_goal_joint_tolerance` | `0.15` | 目标关节容差 (rad) |

**桥接器参数 (在 remani_mpc_tracking.launch.py 中设置)：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `sample_dt` | `0.04` | 参考采样周期 (s) |
| `reference_horizon` | `3.0` | 滚动参考窗口长度 (s) |
| `start_lead` | `0.05` | 起始提前量 (s) |
| `publish_rate` | `20.0` | 参考发布频率 (Hz) |
| `assembly_timeout` | `0.04` | 分段组装去抖动超时 (s) |
| `zero_velocity_threshold` | `1.0e-4` | 零速度检测阈值 |
| `hold_at_end` | `2.0` | 轨迹终点保持时间 (s) |
| `state_dim` | `9` | OCS2 状态维度 |
| `input_dim` | `8` | OCS2 输入维度 |
| `arm_dim` | `6` | 机械臂关节数 |

**桥接器内部状态机：**

```
WAIT_OBSERVATION → WAIT_TRAJECTORY → ASSEMBLING → TRACKING
                                                  ├── 新 ID=1: 保留旧轨迹, 组装新轨迹
                                                  ├── ACTION_ABORT: 发布当前状态保持
                                                  └── 轨迹结束: 发布终点保持 (hold_at_end)
```

#### `exp0.launch.py` — REMANI 独立启动

通过 `sim_example.py` 辅助脚本加载 exp0 参数，启动 REMANI 规划器 (不含桥接)。

---

### 5.3 实机启动

#### `ocs2_real.launch.py` — 实机启动 (tracer_jaka_ocs2)

**时序：**

| 时序 | 组件 | 说明 |
|------|------|------|
| t=0 | robot_state_publisher | 发布 URDF 中的 TF |
| t=0 | tracer_base_node (CAN) | 底盘驱动 |
| t=0 | controller_manager (ros2_control) | JAKA 硬件接口 |
| t=2s | joint_state_broadcaster | 关节状态发布 |
| jsb退出后 | arm_controller | 速度前向控制器 |
| t=10s | MPC + MRT + Joy | OCS2 三件套 |

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_rviz` | `true` | 启动 RViz2 |
| `can_port` | `can0` | CAN 接口 |
| `publish_odom_tf` | `true` | tracer_base 发布 odom→base_footprint TF |
| `robot_ip` | `192.168.0.100` | JAKA 机械臂 IP |
| `local_ip` | `192.168.0.10` | 本地 IP |
| `task_file` | `config/task_real.info` | 实机 task 配置 |
| `urdf_file` | `tracer_jaka_description/urdf/tracer_jaka_zu5.urdf` | 实机规划与运动学共用 URDF |
| `use_joy` | `true` | 手柄驱动 |

**实机 MRT 关键配置：**

| 参数 | 值 | 说明 |
|------|------|------|
| `use_sim_time` | `false` | 系统时间 |
| `mrt_loop_rate` | `100.0` | MRT 循环频率 (Hz) |
| `traj_horizon` | `0.10` | 轨迹预测窗口 (s) |
| `use_stamped_cmd` | `false` | 发布 Twist (非 TwistStamped) |
| `base_cmd_topic` | `/cmd_vel` | 底盘命令话题 |
| `odom_topic` | `/odom` | 里程计话题 |
| `arm_cmd_topic` | `/arm_controller/commands` | 机械臂命令话题 |
| `base_frame` | `base_footprint` | 底盘坐标系 |
| `world_frame` | `odom` | 世界坐标系 |
| `ee_frame` | `tool0` | 末端坐标系 |

---

### 5.4 其他启动文件

#### SLAM 相关 (tracer_jaka_mujoco)

| 文件 | 功能 |
|------|------|
| `real_slam.launch.py` | 实机 SLAM: 启动 tracer_base + IMU + Lakibeam + robot_localization + slam_toolbox + RViz |
| `slam_sim.launch.py` | 仿真 SLAM (基于 MuJoCo 传感器) |
| `localization.launch.py` | 纯定位模式 (已有地图) |

#### 传感器/硬件

| 文件 | 包 | 功能 |
|------|------|------|
| `imu_spec_msg.launch.py` | hipnuc_imu | Hipnuc IMU 驱动 |
| `imu_spec_msg.launch.py` | hipnuc_imu_can | Hipnuc IMU CAN 驱动 |
| `lakibeam1_scan.launch.py` | lakibeam1 | Lakibeam 激光雷达 |
| `lakibeam1_scan_view.launch.py` | lakibeam1 | Lakibeam + RViz |
| `tracer_base.launch.py` | tracer_base | Tracer 底盘 CAN 驱动 |
| `tracer_mini_base.launch.py` | tracer_base | Tracer Mini 底盘驱动 |

#### 传感器仿真 (tracer_jaka_mujoco)

| 文件 | 功能 |
|------|------|
| `d435_sensor.launch.py` | 仿真 D435 深度相机 |
| `d455_sensor.launch.py` | 仿真 D455 深度相机 |
| `esdf_sensor_sim.launch.py` | 仿真 ESDF 传感器 |
| `nvblox.launch.py` | nvblox 3D 建图 |

#### MoveIt (单独运行)

| 文件 | 功能 |
|------|------|
| `moveit_demo.launch.py` | MuJoCo + MoveIt2 联合仿真 |

---

## 6. 配置文件参考

### 6.1 OCS2 任务配置 (task.info / task_real.info)

文件位于 `tracer_jaka_ocs2/config/`。

#### `model_information` — 机器人模型元信息

| 字段 | 值 | 说明 |
|------|------|------|
| `manipulatorModelType` | `1` | 差速移动机械臂 |
| `baseFrame` | `base_footprint` | 底盘基座 |
| `eeFrame` | `tool0` | 末端执行器 |
| `removeJoints` | `left_wheel`, `right_wheel` | 排除轮子关节 |

#### `ddp` — SLQ/DDP 求解器设置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `algorithm` | `SLQ` | Sequential Linear Quadratic |
| `nThreads` | `3` | 并行线程数 |
| `maxNumIterations` | `1` | 最大迭代次数 (实时 MPC 每步只迭代一次) |
| `minRelCost` | `0.1` | 最小相对代价下降 |
| `constraintTolerance` | `1e-3` | 约束容差 |
| `timeStep` | `1e-3` | DDP 积分步长 |
| `backwardPassIntegratorType` | `ODE45` | 反向传播积分器 |
| `strategy` | `LINE_SEARCH` | 线搜索步长策略 |
| `useFeedbackPolicy` | `true` | 使用反馈策略 |
| `constraintPenaltyInitialValue` | `20.0` | 约束惩罚初值 |
| `constraintPenaltyIncreaseRate` | `2.0` | 约束惩罚增长率 |

#### `mpc` — MPC 设置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `timeHorizon` | `2.0` s | MPC 预测时域 |
| `solutionTimeWindow` | `0.2` s | 解时间窗口 |
| `coldStart` | `false` | 热启动 |
| `mpcDesiredFrequency` | `100` Hz | MPC 求解目标频率 |
| `mrtDesiredFrequency` | `400` Hz | MRT 目标频率 |

#### `wholeBodyTracking` — 全身轨迹跟踪代价

| 状态 | 权重 | 说明 |
|------|------|------|
| x | 5.0 | 底盘 X 位置 |
| y | 5.0 | 底盘 Y 位置 |
| yaw | 2.0 | 底盘航向 |
| joint_1~6 | 2.0 | 各机械臂关节 |
| `finalWeightScale` | 1.0 | 终端权重缩放 |

#### `inputCost` — 输入代价 (R 矩阵)

| 输入 | 权重 | 说明 |
|------|------|------|
| v (线速度) | 2.5 | 底盘前进速度 × 缩放 0.1 |
| ω (角速度) | 10.0 | 底盘转向速度 × 缩放 0.1 |
| q̇1~q̇6 | 1.0 | 各关节速度 × 缩放 0.01 |

#### `selfCollision` — 自碰撞

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `activate` | `true` | 启用自碰撞 |
| `minimumDistance` | `0.02` m | 最小安全距离 |
| `mu` | `1e-2` | 惩罚权重 |
| 检查对数 | 28 对 | 底盘-机械臂, 非相邻连杆, 工具-连杆 |

#### `environmentCollision` — 环境避障

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `activate` | `true` | 启用环境避障 |
| `minimumDistance` | `0.05` m | 期望最小间隙 |
| `activationDistance` | `0.20` m | 激活距离 (ThresholdRelaxedBarrierPenalty) |
| 障碍物 | 低桌 (1桌面+4桌腿) + 4面墙 | 低桌穿越场景 |

#### `jointPositionLimits` / `jointVelocityLimits`

| 限制 | 底盘 | 机械臂 |
|------|------|--------|
| 位置下限 | — | -2.0 rad (所有关节) |
| 位置上限 | — | 2.0 rad (所有关节) |
| 速度下限 (v, ω) | -0.5, -1.0 | -2.0 rad/s |
| 速度上限 (v, ω) | 0.5, 1.0 | 2.0 rad/s |

### 6.2 传感器配置 (sensors.yaml)

| 传感器 | 频率 | 话题 | 参数 |
|--------|------|------|------|
| 底盘 | 100 Hz | `/joint_states`, `/wheel/odometry` | 差速运动学: wheel_separation=0.34m, wheel_radius=0.065m |
| LiDAR | 30 Hz | `/scan` | 270°, 1080 rays, range=100m |
| IMU | 100 Hz | `/imu/data` | 噪声: orientation=0.01, gyro=0.005, accel=0.05 |
| 时钟 | 250 Hz | `/clock` | MuJoCo 步长 0.002s (500Hz 物理) |
| D455 | 30 Hz | `/camera/d455/*` | 可选, 640×480, fovy=58° |

### 6.3 EKF 配置 (ekf.yaml)

- 融合: `/wheel/odometry` (里程计) + `/imu/data` (IMU)
- 输出: `/odometry/filtered` (带协方差)
- TF: `odom → base_footprint`
- 频率: 与输入消息同步
- 状态: 2D 位姿 (x, y, yaw) + 线速度 + 角速度

### 6.4 统一控制器配置 (tracer_jaka_description/config/ros2_controllers.yaml)

| 控制器 | 类型 | 关节 |
|--------|------|------|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | 所有关节 |
| `arm_controller` | `forward_command_controller/ForwardCommandController` | joint_1~joint_6 |
| `arm_trajectory_controller` | `joint_trajectory_controller/JointTrajectoryController` | joint_1~joint_6 |
| `base_controller` | `diff_drive_controller/DiffDriveController` | 左右轮 |
| `fts_broadcaster` | `force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster` | 末端六维力传感器 |

---

## 7. 关键设计细节

### 7.1 REMANI→OCS2 桥接器核心算法

#### 多项式解析

REMANI 使用多项式矩阵表示轨迹。对于维度 `dim`、阶数 `num_order` 的多项式：

```
position(t) = Σ coefficient(column, dim) · t^(num_order - column)
              column=0..num_order
```

其中 `t ∈ [0, duration]`。

#### yaw 重建与退化处理

1. **正常情况**: `yaw = atan2(s·vy, s·vx)`, `v = s·√(vx²+vy²)`, `ω = (vx·ay-vy·ax)/(vx²+vy²)`
2. **零速度点** (起/终点、换向点): 速度低于 `zero_velocity_threshold` 时，保持上一个有效 yaw，v=0, ω=0
3. **yaw 解包**: 逐点 `unwrapNear()` 防止 ±π 跳变

#### 时间同步

```
t_ocs2_start = t_obs + (t_remani_start - t_ros_now)
```

使用最新 MPC observation 建立 ROS 时间到 OCS2 内部时间的映射。过期的轨迹部分通过 `elapsed = max(0, ros_now - remani_start)` 跳过。

### 7.2 分段轨迹拼接

- REMANI 的 `trajectory_id` 从 1 开始递增
- `id==1` 表示新轨迹开始，清空旧缓存
- 后续 ID 追加到当前缓存
- 每次收到新分段后，重置 40ms debounce timer
- timer 到期后认为接收完成，发布完整参考
- 各段的起始时间 = 前面所有段的 duration 之和

### 7.3 坐标系

| 坐标系 | 说明 |
|--------|------|
| `map` | 全局地图原点 (SLAM 提供 map→odom) |
| `odom` | 里程计原点 (EKF 提供 odom→base_footprint) |
| `base_footprint` | 机器人底盘投影 |
| `base_link` | 底盘本体 |
| `Link_0` ~ `Link_6` | JAKA 机械臂各连杆 |
| `tool0` | 末端执行器 |
| `laser_link` | 激光雷达 |
| `imu_link` | IMU |

MuJoCo 场景、平面关节和 odom 现在使用相同原点；初始
`base_footprint` 位于 `world x=0`，因此 `x_odom = x_mujoco`。

### 7.4 重规划触发机制

REMANI FSM 在 EXEC_TRAJ 状态下持续监控跟踪误差：

1. 将当前 odom+joint_states 与 REMANI 多项式在当前轨迹时间处的参考值比较
2. 如果任一误差 (位置/yaw/关节) 连续超过阈值 ≥ `persistence` 秒
3. 且不在 `grace_period` 内，且距上次重规划 ≥ `min_interval`
4. 则以**当前实测状态为起点**、**原目标为目标**重新规划
5. 新轨迹以 `trajectory_id=1` 发布，桥接器自动清空旧缓存并替换

### 7.5 参考轨迹的滚动发布

桥接器以 20Hz 频率发布滚动参考窗口：

- 窗口长度: `reference_horizon` (3.0s)
- 采样间隔: `sample_dt` (0.04s) → 75 个采样点
- 终点保持: `hold_at_end` (2.0s), u_ref 全零
- `WholeBodyTrajectoryCost` 内部做线性插值 (yaw 用最短角插值)

因此无需按 100Hz MPC 频率生成参考点。

### 7.6 MPC 约束分层

| 层级 | 约束 | 类型 |
|------|------|------|
| 状态跟踪 | `WholeBodyTrajectoryCost` | 二次代价 (软) |
| 输入平滑 | `QuadraticInputCost` | 二次代价 (软) |
| 自碰撞 | `SelfCollisionConstraint` | ThresholdRelaxedBarrierPenalty (软) |
| 环境避障 | `EnvironmentCollisionConstraint` | ThresholdRelaxedBarrierPenalty (软) |
| 关节位置 | `JointPositionLimits` | 松弛障碍惩罚 (软) |
| 关节速度 | `JointVelocityLimits` | 松弛障碍惩罚 (软) |

### 7.7 职责划分

| 模块 | 职责 |
|------|------|
| **REMANI** | 全局/局部运动学搜索, 动态地图, 完整机器人碰撞检查, 路径拓扑变更 |
| **OCS2 MPC** | 轨迹跟踪, 输入/关节限制, 自碰撞预防, 短时局部避障修正 |
| **Safety Monitor** | 跟踪误差超限 → REMANI 重规划; 紧急情况 → 急停 |
| **MRT** | 策略→硬件命令转换, 状态估计 |

---

## 8. 运行说明

### 8.1 构建

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash

# 全量构建
colcon build \
  --packages-up-to tracer_jaka_ocs2 tracer_jaka_mujoco tracer_base remani_planner \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# 单包快速构建
colcon build --packages-select remani_planner --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/local_setup.bash
```

### 8.2 仿真运行

```bash
# 终端 1: 启动 MuJoCo 仿真 + OCS2 MPC/MRT + REMANI + 桥接 (一键全包)
ros2 launch tracer_jaka_bringup ocs2_sim.launch.py

# 或分步启动:
# 终端 1: 仅仿真 + MPC/MRT
ros2 launch tracer_jaka_bringup ocs2_sim.launch.py start_remani:=false start_slam:=false
# 终端 2: REMANI + 桥接
ros2 launch remani_planner remani_mpc_tracking.launch.py use_sim_time:=true
```

**在 RViz 中**：使用 "2D Goal Pose" 工具点击地图设置目标，右键选择 "Send target"。

### 8.3 实机运行

```bash
# 准备工作
sudo ip link set can0 up type can bitrate 500000
sudo chmod 777 /dev/ttyUSB0

# 各硬件驱动 (如未集成启动)
ros2 launch hipnuc_imu imu_spec_msg.launch.py
ros2 launch lakibeam1 lakibeam1_scan.launch.py

# 主程序
ros2 launch tracer_jaka_bringup ocs2_real.launch.py
```

### 8.4 实机 SLAM 建图

```bash
ros2 launch tracer_jaka_bringup real_slam.launch.py
```

**关键验证命令：**
```bash
ros2 topic hz /odom
ros2 topic hz /IMU_data
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo base_footprint laser_link
ros2 run tf2_ros tf2_echo base_footprint imu_link
```

### 8.5 生成 ESDF 地图

当 MuJoCo 静态场景发生变化时：

```bash
ros2 run grid_map mjcf_to_esdf \
  --xml /absolute/path/to/scene.xml \
  --output /absolute/path/to/scene_esdf.npz
```

### 8.6 手柄控制

支持 PS4/PS5 手柄，三种模式：

| 节点 | 模式 | 操作 |
|------|------|------|
| `tracer_jaka_joy_target_node` | Set-Point 目标 | 发送固定目标位姿 |
| `tracer_jaka_joy_whole_body_node` | "胡萝卜" 模式 | 左摇杆 Y 前进/后退, 右摇杆 X 转向, LB 安全开关, A 手臂归位 |

---

## 9. 仿真环境

### 9.1 MuJoCo 场景

`models/scene.xml` 包含：

- **4 面墙**: 限定 5m×5m 的运动空间 (odom 坐标: x∈[-0.5,4.5], y∈[-2.5,2.5])
- **低桌**: 桌面高 1.15m, 桌下净高 1.10m, 4 条桌腿
- **机器人模型**: Tracer 底盘 + JAKA Zu 5 机械臂 (include `tracer_jaka_zu5.xml`)

### 9.2 传感器仿真

| 传感器 | 类型 | 频率 | 用途 |
|--------|------|------|------|
| 轮式里程计 | 运动学计算 | 100 Hz | 底盘速度积分 |
| 2D LiDAR | 光线追踪 | 30 Hz | 模拟 Lakibeam, 270° 扫描 |
| IMU | 加速度计+陀螺仪 | 100 Hz | 姿态估计 |
| D455 深度相机 | 离屏渲染 | 30 Hz (可选) | RGB-D 感知 |

### 9.3 碰撞模型

- **机器人碰撞体**: 从 URDF 加载，每个 link 有独立碰撞球体 (半径 0.035~0.085m)
- **环境组**: group 0 (环境物体，LiDAR 可见)
- **机器人组**: group 1 (视觉), group 3 (碰撞, LiDAR 不可见)

---

## 10. nvblox 3D ESDF 建图 (Isaac ROS)

> 代码路径: `/home/a/workspaces/isaac_ros-dev` — NVIDIA Isaac ROS 的 GPU 加速 3D 重建框架，为 REMANI/OCS2 提供实时动态 ESDF 地图。

### 10.1 概述

nvblox 是 NVIDIA 的 GPU 加速 3D 建图库，通过 RGB-D 相机（及可选 LiDAR）实时构建截断符号距离场 (TSDF) 和欧几里得符号距离场 (ESDF)。在本工程中，nvblox 为 REMANI 规划器和 OCS2 MPC 提供**实时 3D ESDF**，替代 `task.info` 中预设的静态障碍物，使系统能够感知和响应环境变化。

```
Isaac ROS nvblox 3D ESDF Pipeline:

MuJoCo D455 相机 / 实机 D455/D435
   │  depth image + camera_info + color image
   ▼
nvblox_node (GPU 加速, component_container_mt)
   │  TSDF 积分 → ESDF 计算 → Mesh 重建
   │  发布: /nvblox_node/static_esdf_pointcloud (PointCloud2, intensity=距离)
   │  服务: /nvblox_node/get_esdf_and_gradient (查询任意 AABB 内的 3D ESDF)
   ▼
┌──────────────────────┬──────────────────────────┐
│ esdf_viz_node        │ esdf_visualizer           │
│ (nvblox_esdf_viz)    │ (my_nvblox_bringup)        │
│ 全量点云重上色       │ 服务调用 + robot-centric   │
│ /nvblox/esdf_viz      │ /nvblox_node/esdf_3d_     │
│ (PointCloud2, rgb)    │   pointcloud (PointCloud2) │
└──────────────────────┴──────────────────────────┘
                                │
                                ▼
                    REMANI GridMap (待对接)
                    OCS2 动态避障 (待对接)
```

### 10.2 包结构

```
src/
├── isaac_ros_nvblox/       # NVIDIA 官方 nvblox ROS 2 包 (Git submodule)
│   ├── nvblox_ros/              # 核心 ROS 节点: NvbloxNode (C++, GPU)
│   ├── nvblox_ros_common/       # 共享工具库 (图像预处理等)
│   ├── nvblox_msgs/             # 自定义消息/服务
│   │   ├── msg/DistanceMapSlice.msg, Mesh.msg, VoxelBlock.msg ...
│   │   └── srv/EsdfAndGradients.srv, FilePath.srv
│   ├── nvblox_nav2/             # Nav2 Costmap 2D 切片插件
│   ├── nvblox_rviz_plugin/      # RViz 可视化插件 (3D voxel/mesh)
│   ├── nvblox_examples_bringup/ # Isaac Sim 示例 launch
│   └── nvblox_foxglove/         # Foxglove 可视化集成
│
├── my_nvblox_bringup/        # ★ 本工程的自定义 nvblox 启动包
│   ├── launch/
│   │   ├── nvblox_core.launch.py     — 核心 mapping launch (sensor-agnostic)
│   │   ├── nvblox_mapping.launch.py  — 向后兼容别名
│   │   ├── mujoco_esdf.launch.py     — MuJoCo 仿真 D455 → nvblox
│   │   ├── d455_esdf.launch.py       — 实机 D455 → nvblox
│   │   ├── d435_esdf.launch.py       — 实机 D435 → nvblox
│   │   └── d455_bag_esdf.launch.py   — ROS bag 回放离线建图
│   ├── config/
│   │   ├── nvblox_3d.yaml            — 3D ESDF 模式配置 (5cm 分辨率, 2m 截断)
│   │   └── d455_esdf_bag_qos.yaml    — Bag 回放 QoS 覆盖
│   ├── rviz/esdf_3d.rviz             — ESDF 3D RViz 配置
│   └── my_nvblox_bringup/
│       └── esdf_visualizer.py        — 机器人中心 ESDF 查询+发布节点
│
├── nvblox_esdf_viz/          # ★ 3D ESDF 全量点云可视化
│   ├── launch/esdf_viz.launch.py
│   ├── rviz/esdf_3d.rviz
│   └── nvblox_esdf_viz/
│       └── esdf_viz_node.py          — 全量点云重上色 (红=近障碍, 蓝=远)
│
└── isaac_ros_common/         # Isaac ROS 公共库 (Docker, 测试工具等)
```

### 10.3 核心组件详解

#### 10.3.1 `NvbloxNode` — GPU 加速建图核心

`nvblox_ros::NvbloxNode` 是一个 `rclcpp_components` 组件节点，在独立的 `component_container_mt` 多线程容器中运行。

**核心能力：**
- **TSDF 积分**: 从 RGB-D 图像逐帧投影积分 (projective TSDF integrator)
- **ESDF 计算**: GPU 并行计算带符号距离场 (`esdf_mode: "3d"` 输出 3D 体素)
- **Mesh 重建**: Marching Cubes 算法生成 3D Mesh
- **动态清除**: 半径清除（机器人周围）+ AABB/球体清除（通过 service）
- **分层发布**: 每层（TSDF/ESDF/Mesh）独立控制发布频率

**关键参数 (nvblox_3d.yaml):**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `voxel_size` | `0.05` | TSDF/ESDF 体素边长 (m) |
| `esdf_mode` | `"3d"` | 3D ESDF 模式 (非 "2d" 切片) |
| `mapping_type` | `"static_tsdf"` | 静态 TSDF 模式 |
| `integrate_depth_rate_hz` | `30.0` | 深度积分频率 (Hz) |
| `update_esdf_rate_hz` | `10.0` | ESDF 更新频率 (Hz) |
| `esdf_integrator_max_distance_m` | `2.0` | ESDF 最大距离截断 (m) |
| `max_back_projection_distance` | `5.0` | 深度最大反向投影距离 (m) |
| `projective_integrator_truncation_distance_vox` | `4.0` | TSDF 截断距离 (体素) |
| `projective_integrator_max_integration_distance_m` | `5.0` | 最大积分距离 (m) |
| `map_clearing_radius_m` | `7.0` | 机器人周围地图清除半径 (<0 则保留所有) |
| `clear_map_outside_radius_rate_hz` | `1.0` | 清除检查频率 (Hz) |
| `publish_esdf_distance_slice` | `false` | 不发布 2D 切片 (使用 3D 模式) |
| `do_depth_preprocessing` | `false` | 不预处理深度图 |

#### 10.3.2 `esdf_visualizer` — 机器人中心 ESDF 查询节点

`my_nvblox_bringup/esdf_visualizer.py` (215行) 通过 **service 调用** `/nvblox_node/get_esdf_and_gradient` 获取机器人周围 AABB 内的 3D ESDF 体素网格，转换为带 `intensity` (距离值) 的 `PointCloud2` 发布。

**工作流程：**
1. 通过 TF 获取 `base_footprint` 在 `odom` 系中的位置
2. 构造 `EsdfAndGradients` 服务请求 (AABB 包围盒)
3. 异步调用 nvblox 服务，获取 3D ESDF 体素数据
4. 降采样 (voxel_subsampling) + 距离过滤 (max_visualized_distance)
5. 发布 `/nvblox_node/esdf_3d_pointcloud`

**参数:**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `query_size_x_m` | `4.0` | 查询 AABB X 宽度 (m) |
| `query_size_y_m` | `4.0` | 查询 AABB Y 宽度 (m) |
| `query_min_z_m` | `-0.2` | 查询 AABB Z 最低点 (m) |
| `query_size_z_m` | `3.0` | 查询 AABB Z 高度 (m) |
| `publish_rate_hz` | `1.0` | 发布频率 (Hz) |
| `max_visualized_distance_m` | `1.5` | 仅显示距障碍 ≤ 此距离的体素 (m) |
| `voxel_subsampling` | `2` | 降采样因子 (每 N 个体素取 1) |
| `follow_tracking_frame` | `true` | 窗口跟随机器人; false=固定 |
| `unobserved_value` | `-1000.0` | 未观测区域的标记值 |

#### 10.3.3 `esdf_viz_node` — 全量 ESDF 点云可视化

`nvblox_esdf_viz/esdf_viz_node.py` (143行) 订阅 nvblox 直接发布的 `/nvblox_node/static_esdf_pointcloud` 全量 3D ESDF 点云，用**发散色带**重新上色后发布。

**色带映射:** 近障碍 (0m) = 红, 中间 = 黄→绿→青, 远 (>max_view) = 蓝。支持**切片模式**: 只保留某高度 z 附近的一薄层，复现 2D 地图观感。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `input_topic` | `/nvblox_node/static_esdf_pointcloud` | 输入全量 ESDF 点云 |
| `output_topic` | `/nvblox/esdf_viz` | 彩色点云输出 |
| `max_view_distance` | `2.0` | 仅保留距离 ≤ 此值的体素 (m) |
| `slice_enabled` | `false` | 启用 2D 切片模式 |
| `slice_z` | `0.30` | 切片中心高度 (m) |
| `slice_thickness` | `0.10` | 切片厚度 (m) |

### 10.4 启动文件详解

#### `nvblox_core.launch.py` — 核心通用启动

sensor-agnostic 的基础 mapping launch，所有传感器特定 launch 均包含此文件。启动：
1. `nvblox_node` (ComposableNode 容器)
2. `esdf_visualizer` (ESDF 服务查询 → PointCloud2)
3. `rviz2` (可选)

**所有参数 (24个):** 参见上方 nvblox_3d.yaml 参数表。

#### `mujoco_esdf.launch.py` — MuJoCo 仿真集成

搭配 `tracer_jaka_ocs2/ocs2_sim.launch.py` 的 nvblox 建图启动：

```bash
# 终端 1: MuJoCo 仿真 + 传感器 (含 D455 相机)
ros2 launch tracer_jaka_mujoco bridge.launch.py camera:=true

# 终端 2: nvblox 3D ESDF 建图
ros2 launch my_nvblox_bringup mujoco_esdf.launch.py global_frame:=odom
ros2 launch my_nvblox_bringup mujoco_esdf.launch.py global_frame:=odom

# 终端 3: OCS2 MPC + REMANI (如需要)
ros2 launch tracer_jaka_bringup ocs2_sim.launch.py start_slam:=false start_remani:=false
```

**已配置的 topic 连接：** D455 RGB-D 话题 (`/camera/d455/{depth,color}/image_raw` + `camera_info`)。

#### `d455_esdf.launch.py` / `d435_esdf.launch.py` — 实机集成

适用于实机 D455/D435 RGB-D 相机：

```bash
# 实机 nvblox 3D ESDF
ros2 launch my_nvblox_bringup d455_esdf.launch.py \
  global_frame:=odom \
  use_sim_time:=false \
  voxel_size:=0.05 \
  map_clearing_radius_m:=-1.0 \
  esdf_viz_follow_robot:=false \
  esdf_viz_size_x:=12.0 \
  esdf_viz_size_y:=12.0
```

**关键差异 (d455_esdf vs mujoco_esdf):**

| 参数 | MuJoCo | 实机 D455 |
|------|--------|-----------|
| `depth_image_topic` | `/camera/d455/depth/image_raw` | `/camera/d455/depth/image_rect_raw` |
| `map_clearing_radius_m` | `7.0` (移动清除) | `-1.0` (持久地图) |
| `esdf_viz_follow_robot` | `true` (跟随) | `false` (固定全局视角) |
| `esdf_viz_size_x/y` | `4.0` / `4.0` | `12.0` / `12.0` |
| `esdf_viz_rate` | `1.0` Hz | `0.2` Hz |
| `esdf_viz_subsampling` | `2` | `3` |
| `esdf_viz_max_distance` | `1.5` m | `0.5` m |

#### `d455_bag_esdf.launch.py` — ROS Bag 离线建图

从 NUC 录制的 ROS bag (`sqlite3` 格式) 回放深度图 + TF，构建持久持久化 ESDF 地图：

```bash
ros2 launch my_nvblox_bringup d455_bag_esdf.launch.py \
  bag:=/path/to/bag_directory \
  rate:=0.5 \
  voxel_size:=0.10 \
  esdf_viz_size_x:=20.0 \
  esdf_viz_size_y:=20.0
```

**独立域: ROS_DOMAIN_ID=21**，避免干扰实机运行。时序: nvblox 先启动 5s, 然后 `ros2 bag play --clock 100.0` 回放。

### 10.5 关键消息与服务

#### `EsdfAndGradients.srv` — ESDF 查询服务

```yaml
# 请求
bool update_esdf                  # 先更新 ESDF
bool visualize_esdf               # 同时发到可视化话题
bool use_aabb                     # 启用 AABB 包围盒
string frame_id                   # 坐标系
geometry_msgs/Point aabb_min_m    # AABB 最小角 (m)
geometry_msgs/Vector3 aabb_size_m # AABB 尺寸 (m)
geometry_msgs/Point[] aabbs_to_clear_min_m    # 要清除的 AABB
geometry_msgs/Vector3[] aabbs_to_clear_size_m
geometry_msgs/Point[] spheres_to_clear_center_m # 要清除的球体
float32[] spheres_to_clear_radius_m
---
# 响应
geometry_msgs/Point origin_m            # 网格原点 (m)
float32 voxel_size_m                    # 体素大小 (m)
std_msgs/Float32MultiArray esdf_and_gradients # 扁平化 ESDF 体素数据
bool success                            # 是否成功
```

**使用示例 (命令行):**
```bash
ros2 service call /nvblox_node/get_esdf_and_gradient \
  nvblox_msgs/srv/EsdfAndGradients \
  "{update_esdf: true,
    visualize_esdf: true,
    use_aabb: true,
    frame_id: 'odom',
    aabb_min_m: {x: -2.0, y: -2.0, z: 0.0},
    aabb_size_m: {x: 4.0, y: 4.0, z: 1.5},
    aabbs_to_clear_min_m: [],
    aabbs_to_clear_size_m: [],
    spheres_to_clear_center_m: [],
    spheres_to_clear_radius_m: []}"
```

#### `DistanceMapSlice.msg` — 2D 距离图切片

nvblox 的 2D ESDF 切片消息 (仅在 `esdf_mode: "2d"` 时发布)：

```yaml
std_msgs/Header header
float32 resolution          # 分辨率 (m/pixel)
uint32 width                # 宽度 (pixels, X 方向)
uint32 height               # 高度 (pixels, Y 方向)
geometry_msgs/Point origin  # 左上角原点 (m)
float32 unknown_value       # 未观测区域的标记值
float32[] data              # 行主序 ESDF 数据 (m)
```

### 10.6 与 OCS2 系统的集成计划

当前状态: nvblox 3D ESDF 建图已可独立运行，与 REMANI/OCS2 的对接尚在规划中。

| 阶段 | 描述 | 状态 |
|------|------|------|
| **阶段 1** | nvblox + MuJoCo D455 仿真建图 → 发布 3D ESDF 点云 | ✅ 已完成 |
| **阶段 2** | nvblox + 实机 D455/D435 建图 | ✅ 已完成 |
| **阶段 3** | Bag 离线回放生成持久 ESDF NPZ → 导入 REMANI GridMap | ⬜ 待实现 |
| **阶段 4** | nvblox 实时 ESDF → REMANI GridMap 动态更新 | ⬜ 待实现 |
| **阶段 5** | nvblox 实时 ESDF → OCS2 EnvironmentCollisionConstraint 动态障碍物 | ⬜ 待实现 |
| **阶段 6** | GPU 加速的动态 ESDF + 时空规划 (参见 总体_Pipeline.md) | ⬜ 待实现 |

**关键对接点：**

1. **ESDF → REMANI GridMap**: 通过 `EsdfAndGradients` 服务定期查询机器人周围的 3D ESDF，以体素形式导入 REMANI 的 `GridMap`，替代/补充静态 `.npz` 文件

2. **ESDF → OCS2 动态避障**: 将 nvblox 的实时 ESDF 转换为 OCS2 `EnvironmentCollisionConstraint` 中的动态障碍物列表，使 MPC 能临时绕开新出现的障碍物

3. **持久化存储**: 将 nvblox 累积的 TSDF/ESDF 导出为 REMANI 兼容的 `.npz` 文件，实现"探索-建图-规划"的闭环

### 10.8 Nav2 Costmap 集成 (nvblox_nav2)

`nvblox_nav2` 包提供了 `NvbloxCostmapLayer` — 一个 Nav2 costmap 插件，将 nvblox 的 `DistanceMapSlice` 2D ESDF 切片转换为 Nav2 costmap 代价值：

- 订阅: `/nvblox_node/static_map_slice` (2D 模式)
- ESDF 距离 → costmap 代价映射:
  - `distance ≤ 0` → `LETHAL_OBSTACLE` (254)
  - `distance < inflation` → `INSCRIBED_INFLATED_OBSTACLE` (253)
  - `distance > max_obstacle_distance` → `FREE_SPACE` (0)
  - 其余 → 线性插值
- 支持 TF 坐标系对齐 + 二值 costmap 模式

**关键参数:**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `nvblox_map_slice_topic` | `/nvblox_node/static_map_slice` | 2D ESDF 切片话题 |
| `max_obstacle_distance` | `1.0` | 最大障碍物距离 (m) |
| `inflation_distance` | `0.5` | 膨胀半径 (m) |
| `convert_to_binary_costmap` | `false` | 二值化模式 |

### 10.9 性能考量

**GPU→CPU 传输延迟**: `EsdfAndGradients` 服务需要将 GPU 上的 3D ESDF 体素网格复制回 CPU 作为 `Float32MultiArray`。对于 4m×4m×2m 范围、5cm 体素的查询 (约 80³ = 512,000 个浮点数)，传输约需 2MB 带宽。建议:
- 使用 `voxel_subsampling` 降采样降低数据量
- 仅在需要时调用服务（而非每个 MPC 周期）
- 或订阅 `static_esdf_pointcloud` 以流式获取体素

**实时性建议**: 对于 100Hz MPC 求解器:
- nvblox 的 10Hz ESDF 更新频率足够提供局部环境信息
- 在 ESDF 更新之间，MPC 使用上一帧的静态障碍物
- 可考虑使用 `OccupancyGrid` (2D) 作为底盘的快速碰撞查询

---

### 10.10 参考: 上游 nvblox 生态

上游 `isaac_ros_nvblox` 仓库还提供了丰富的参考实现:

| 包 | 用途 |
|----|------|
| `nvblox_examples_bringup` | RealSense/ZED/IsaacSim 多相机示例 launch + 参数配置 |
| `nvblox_rviz_plugin` | RViz 面板插件 (3D voxel/mesh 渲染) |
| `nvblox_foxglove` | Foxglove Studio 扩展 |
| `realsense_splitter` | RealSense 红外/深度流分离节点 |
| `nvblox_image_padding` | 图像边缘填充 (支持语义分割 mask 同步) |
| `isaac_ros_launch_utils` | 启动文件工具库 (ArgumentContainer, component_container 等) |

### 10.7 nvblox 关键话题速查

| Topic | 类型 | 方向 | 说明 |
|-------|------|------|------|
| `/nvblox_node/static_esdf_pointcloud` | `sensor_msgs/PointCloud2` | nvblox→ROS | 全量 3D ESDF 点云 (x,y,z,intensity=distance) |
| `/nvblox_node/esdf_3d_pointcloud` | `sensor_msgs/PointCloud2` | esdf_visualizer→ROS | 机器人中心 AABB 查询的 ESDF 子集 |
| `/nvblox/esdf_viz` | `sensor_msgs/PointCloud2` | esdf_viz_node→ROS | 彩色 ESDF 可视化点云 (x,y,z,rgb) |
| `/nvblox_node/mesh` | `nvblox_msgs/Mesh` | nvblox→ROS | 重建的 3D Mesh |
| `/nvblox_node/get_esdf_and_gradient` | `nvblox_msgs/EsdfAndGradients` (srv) | →nvblox | 按 AABB 查询 ESDF 体素网格 |

---

## 附录 A: 重要 Topic 速查

| Topic | 类型 | 方向 | 说明 |
|-------|------|------|------|
| `/mobile_manipulator_mpc_target` | `ocs2_msgs/MpcTargetTrajectories` | → MPC | OCS2 参考轨迹 (桥接器发布) |
| `/mobile_manipulator_mpc_observation` | `ocs2_msgs/MpcObservation` | MRT→MPC | 当前状态观测 |
| `/mobile_manipulator_mpc_policy` | `ocs2_msgs/MpcPolicy` | MPC→MRT | MPC 求解的策略 |
| `/planning/trajectory` | `quadrotor_msgs/PolynomialTraj` | REMANI→Bridge | REMANI 多项式轨迹 |
| `/joint_states` | `sensor_msgs/JointState` | 仿真/硬件→ROS | 所有关节状态 |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF→MRT | 融合里程计 (仿真) |
| `/odom` | `nav_msgs/Odometry` | tracer_base→ROS | 轮式里程计 (实机) |
| `/wheel/odometry` | `nav_msgs/Odometry` | MuJoCo→ROS | 原始轮式里程计 (仿真) |
| `/imu/data` | `sensor_msgs/Imu` | MuJoCo→ROS | IMU 数据 |
| `/scan` | `sensor_msgs/LaserScan` | MuJoCo/Lakibeam→ROS | 2D 激光扫描 |
| `/cmd_vel` | `geometry_msgs/Twist` | MRT→底盘 | 底盘速度命令 (实机) |
| `/base_controller/cmd_vel` | `geometry_msgs/Twist` | MRT→底盘 | 底盘速度命令 (仿真) |
| `/arm_controller/commands` | `std_msgs/Float64MultiArray` | MRT→机械臂 | 机械臂速度命令 (仿真) |
| `/arm_controller/commands` | `std_msgs/Float64MultiArray` | MRT→机械臂 | 机械臂速度命令 (实机) |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz→REMANI | 规划目标 (2D Goal Pose) |
| `/map` | `nav_msgs/OccupancyGrid` | slam_toolbox→ROS | 2D 占据栅格地图 |
| `/clock` | `rosgraph_msgs/Clock` | MuJoCo→ROS | 仿真时钟 |
| `/nvblox_node/static_esdf_pointcloud` | `sensor_msgs/PointCloud2` | nvblox→ROS | 3D ESDF 全量点云 (intensity=距离) |
| `/nvblox_node/esdf_3d_pointcloud` | `sensor_msgs/PointCloud2` | esdf_visualizer→ROS | 机器人中心 ESDF 子集 |
| `/nvblox/esdf_viz` | `sensor_msgs/PointCloud2` | esdf_viz_node→ROS | 彩色 ESDF 可视化点云 |
| `/nvblox_node/mesh` | `nvblox_msgs/Mesh` | nvblox→ROS | 3D 重建 Mesh |

## 附录 B: 重要 TF 树

```
map ──(SLAM/静态)──→ odom ──(EKF/tracer_base)──→ base_footprint
                                                        │
                                            (robot_state_publisher)
                                                        │
                ┌───────────────────────────────────────┤
                ▼                                       ▼
            base_link                              laser_link
                │                                   imu_link
                ▼                                   d455_link
              Link_0
                │
                ▼
              Link_1 → Link_2 → Link_3 → Link_4 → Link_5 → Link_6 → tool0
```

## 附录 C: 自定义手柄控制 ("胡萝卜" 模式) 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `linear_speed_max` | 0.4 m/s | 最大线速度 |
| `angular_speed_max` | 1.0 rad/s | 最大角速度 |
| `deadzone` | 0.10 | 摇杆死区 |
| `lookahead_time` | 1.5 s | 前视时间 (=目标距离 = 当前速度 × lookahead_time) |
| `trajectory_horizon` | 2.0 s | 轨迹总时长 |
| `num_waypoints` | 5 | 航点数 |
| `arm_home` | [-0.515, 1.5707, -1.5707, 1.5707, 1.5707, 0.254] (sim) | 机械臂 home 构型 |
| | [0.0, 1.5707, -1.57, 1.5707, 1.57, 0.785398] (real) | |

---

> **文档版本**: 2026-08-01
> **工程路径**: `/home/a/WBMM` (OCS2+REMANI), `/home/a/workspaces/isaac_ros-dev` (nvblox/Isaac ROS)
> **ROS 版本**: Humble
> **C++ 标准**: C++17
