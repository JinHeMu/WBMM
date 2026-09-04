# Architecture

> 技术合同：所有新功能、重构、Bug 修复都必须先对照本文档，再修改代码。
> 状态：持续维护。若实现与本文档不一致，以本文档为基准更新代码或文档。

## System Goal

从一份统一 YAML 任务文件出发，自动完成：

```text
统一 YAML 任务
  -> TA-WBMP 任务轨迹生成 + 全身可达性/碰撞/运动学规划
  -> REMANI 碰撞感知导航到 9D 预接触状态 q_pre
  -> 显式参考所有权交接
  -> OCS2 MPC 跟踪预接触接近与接触任务轨迹
  -> 可选 whole_body_force_control（force_control_enabled=false/true）
  -> MuJoCo / 实机闭环
```

`wipe_planner` 已退出主链，但暂不物理删除：它仍保留部分尚未完全等价迁移的
接触安全监督与回归基线。

## Data Flow

```text
task.yaml
  |
  v
TaskTrajectoryGenerator        # surface/pattern/execution 解析
  |
  v
TaskAwarePlanner (TA-WBMP)     # 生成 Plan: waypoints + q_pre + task_targets/normals
  |
  v
ExecutionCoordinator
  |-- WholeBodyGoal(q_pre) ---------------> REMANI
  |-- REMANI trajectory -----> bridge ----> OCS2 MPC  (NAVIGATING 阶段)
  |-- set_task_execution(true) -----------> REMANI TASK_EXEC
  |-- MpcTargetTrajectories ---------------> OCS2 MPC/MRT
  |-- optional wrench ---------------------> whole_body_force_control
  |       \-> corrected 9D reference -----> OCS2 MPC
  v
MuJoCo / real robot
```

执行过程中**同一时刻只有一个 MPC 参考所有者**：

- `NAVIGATING`：REMANI bridge 持有 MPC reference。
- `TASK_EXEC`：TA-WBMP Coordinator 持有 MPC reference。

交权使用 `/remani_bridge/set_reference_enabled` 和
`/remani_planner/set_task_execution` 两次确认。bridge 释放 publisher 后运行时数量
必须先变为 0；Coordinator 建立 publisher 后必须变为 1。任一步骤超时或发现
多于一个发布者都进入 `FAILED`，不得继续发送任务参考。

## State Definition

全身状态固定为 9 维，顺序不可变：

```text
x = [x_b, y_b, yaw_b, q1, q2, q3, q4, q5, q6]^T
```

| 符号 | 含义 |
|---|---|
| `x_b, y_b` | 底盘在规划坐标系（`odom`/`map`）中的位置 |
| `yaw_b` | 底盘航向角 |
| `q1..q6` | JAKA 机械臂关节角 |

该状态同时用于：

- `trajectory_msgs/JointTrajectory` 可视化契约
- `ocs2_msgs/MpcObservation` 实测状态
- `sensor_msgs/JointState` live state
- TA-WBMP / wipe_planner 内部 `Eigen::VectorXd`

## Control Definition

MPC/执行输入固定为 8 维：

```text
u = [v_b, omega_b, qdot1, qdot2, qdot3, qdot4, qdot5, qdot6]^T
```

| 符号 | 含义 |
|---|---|
| `v_b` | 底盘纵向速度 |
| `omega_b` | 底盘航向角速度 |
| `qdot1..qdot6` | 机械臂关节速度 |

## Task

任务层期望通常表达为末端执行器（EE）目标：

```text
p_ee_des(t), R_ee_des(t), V_ee_des(t)
```

全身运动学关系（概念模型）：

```text
V_ee = J_wb(x) u
```

其中 `J_wb(x)` 是全身 Jacobian，把底盘速度与机械臂关节速度映射到 EE 空间速度。
实际代码中 TA-WBMP 使用 URDF + Pinocchio 前向运动学/IK 生成离散 9D waypoint，
OCS2 使用全身模型做滚动优化跟踪。

## Base Constraint

差速底盘非完整约束：

```text
x_b_dot = v_b * cos(yaw_b)
y_b_dot = v_b * sin(yaw_b)
yaw_b_dot = omega_b
```

所有底盘路径/参考都必须满足该约束，不允许产生横向速度。

## Coordinate Frames

详细帧树见 `docs/frames.md`。核心约定：

```text
map
  -> odom
      -> base_footprint
          -> base_link
              -> jaka_base_link
                  -> Link_1 .. Link_6
                      -> tool0_and_camera_link
                          -> tool0
```

- 规划/任务坐标系：`odom`（MuJoCo 默认），实机定位时可使用 `map`。
- 所有 9D 状态中的 `x_b, y_b, yaw_b` 都是相对规划坐标系的底盘位姿。
- EE 位置/法向量在规划坐标系中表达。
- 力传感器读数在传感器自身坐标系，使用时必须明确 `F^sensor`，并由上层决定是否转换到 EE/世界系。

## Modules

| 模块 | 职责 |
|---|---|
| `ta_wbmp` | 统一 YAML 任务解析、任务轨迹生成、TA-WBMP 全身规划、执行协调器、可选力控 |
| `remani_planner` | vendor，碰撞感知导航到 q_pre |
| `tracer_jaka_ocs2` | OCS2 MPC + MRT，跟踪 9D 全身参考 |
| `tracer_jaka_mujoco` | MuJoCo 仿真、URDF、场景 |
| `whole_body_force_control` | 通用恒力/导纳/力跟随控制库 |
| `wbmm_visualization` | 统一整机 mesh/轨迹/播放可视化 |
| `wipe_planner` | 旧主链实现，保留为接触安全回归基线，不再作为主链依赖 |

## Interfaces

主要 C++ 接口：

- `TaskTrajectoryProvider::generate() -> TaskTrajectory`
- `TaskAwarePlanner::plan() -> Plan`
- `WholeBodyStateValidityChecker::check(state)`
- `CandidateCostEvaluator::evaluate(metrics)`
- `NavigationCostEstimator::estimate(start, goal)`
- `whole_body_force_control::AdmittanceController`
- `whole_body_force_control::ForceFollower`
- `whole_body_force_control::WholeBodyKinematics`

主要 ROS 2 接口：

- 服务：`/ta_wbmp/execution/start`、`/ta_wbmp/execution/enable_force_control`、`/remani_planner/set_task_execution`
- 话题：见下文 ROS Topics。

## Costs

TA-WBMP 候选评分权重（`CandidateCostWeights`）：

```text
score =
  w_pos    * position_error
+ w_axis   * axis_error
+ w_arm    * arm_path_length
+ w_base   * base_path_length
+ w_margin * inverse_joint_margin
+ w_manip  * inverse_manipulability
+ w_sigma  * inverse_min_sigma
+ w_standoff_dev * standoff_deviation
+ w_offset * longitudinal_offset
+ w_nav    * navigation_cost_estimate
+ w_preferred * preferred_standoff
```

OCS2 MPC 侧仍有独立的全身跟踪 Q 和输入 R 权重，定义在 OCS2 `task.info` / 模型配置中。

## Constraints

硬约束 / 强约束：

- 9D 状态维度与关节名顺序固定。
- 差速底盘非完整约束。
- 任务 EE 位置误差 ≤ `max_position_error`，姿态误差 ≤ `max_axis_error`。
- 关节限位与最小关节裕度。
- 可操作度 / 最小奇异值阈值。
- 自碰撞与 ESDF 环境碰撞（REMANI）。
- 底盘 standoff / longitudinal offset 几何语义：standoff 沿底盘—任务面方向，longitudinal offset 沿水平正交方向。
- 力控安全：传感器超时保持、力误差节流/暂停、硬限位锁存、尖峰拒绝。

阶段约束：

```text
NAVIGATE -> PRECONTACT_ALIGN -> PRECONTACT_APPROACH -> TASK_CONSTRAINED
```

## Optimization Variables

- 任务候选：`standoff`、`longitudinal_offset`、`yaw_offset`
- 底盘路径：SE2 离散点 `(x, y, yaw)`
- 机械臂 IK：`q = [q1..q6]`
- 全身 waypoint：`x = [x_b, y_b, yaw_b, q1..q6]`
- OCS2 滚动优化：状态轨迹 `x(.)`、输入轨迹 `u(.)`

## Solver

| 层级 | 方法 |
|---|---|
| TA-WBMP 底盘导航 | SE2 grid A* + polyline simplify |
| TA-WBMP 任务候选 | 候选枚举 + URDF/Pinocchio IK + 碰撞/运动学检查 |
| REMANI | vendor 优化式轨迹规划（碰撞感知） |
| OCS2 MPC | SLQ / MRT 滚动时域控制 |
| 力控 | 二阶导纳或准静态力跟随 + 全身运动学修正 |

## ROS Topics

主要话题（以当前实现为准）：

| Topic | Type | 方向 | QoS/备注 |
|---|---|---|---|
| `/ta_wbmp/execution/status` | `std_msgs/String` | 输出 | transient_local，状态机 |
| `/ta_wbmp/execution/force_state` | `std_msgs/String` | 输出 | 力控状态 |
| `/ta_wbmp/execution/start` | `std_srvs/Trigger` | 服务 | 启动执行 |
| `/ta_wbmp/execution/enable_force_control` | `std_srvs/SetBool` | 服务 | 运行时开关力控 |
| `/remani_planner/whole_body_goal` | `traj_utils/WholeBodyGoal` | 输出 | 发给 REMANI 的 q_pre |
| `/remani_planner/set_task_execution` | `std_srvs/SetBool` | 服务 | 显式交权 |
| `/remani_planner/fsm_state` | `std_msgs/String` | 输入 | REMANI 状态 |
| `/planning/trajectory` | `quadrotor_msgs/PolynomialTraj` | 输入 | REMANI 导航轨迹 |
| `/mobile_manipulator_mpc_target` | `ocs2_msgs/MpcTargetTrajectories` | 输出 | 9D MPC 参考 |
| `/mobile_manipulator_mpc_observation` | `ocs2_msgs/MpcObservation` | 输入 | 实测 9D 状态 |
| `/fts_broadcaster/wrench` | `geometry_msgs/WrenchStamped` | 输入 | 力传感器 |
| `/base_controller/cmd_vel` | `geometry_msgs/Twist` | 输出 | MRT -> 底盘 |
| `/arm_controller/commands` | `std_msgs/Float64MultiArray` | 输出 | MRT -> 机械臂 |
| `/wbmm/whole_body_trajectory` | `trajectory_msgs/JointTrajectory` | 可视化 | transient_local |
| `/wbmm/phase_schedule` | `std_msgs/String` | 可视化 | transient_local |
| `/wbmm/robot_mesh` | `visualization_msgs/MarkerArray` | 可视化 | transient_local |

## Update Frequency

| 环节 | 频率 |
|---|---|
| TA-WBMP 离线规划 | 一次性 |
| ExecutionCoordinator MPC reference | 默认 20 Hz（`reference_rate`） |
| OCS2 MPC / MRT 控制环 | 约 125 Hz |
| MuJoCo / 实机底层控制 | 125 Hz 级别 |
| 力传感器 | 由驱动决定（通常 50–500 Hz） |
| wbmm robot_mesh | 10–30 Hz |
| REMANI 导航轨迹 | 事件驱动 / 重规划 |
