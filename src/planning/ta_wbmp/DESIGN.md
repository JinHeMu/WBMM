# TA-WBMP 通用任务全身规划架构

## 1. 目标与边界

TA-WBMP 将接触作业、绘制、抓取等任务统一表示为带姿态、切向、表面法向和接触标志的末端约束轨迹：

```text
Task 任务轨迹生成
        ↓
基于任务的全身轨迹规划
        ↓
REMANI 导航到 q_pre
        ↓
OCS2 MPC 跟踪任务全身轨迹
```

模块输出的是名义 9D 全身轨迹：

```text
[base_x, base_y, base_yaw, joint_1, ..., joint_6]
```

真实执行仍需统一地图/ESDF、REMANI、OCS2、控制器和硬件安全链。预览节点只把执行结果发布到 `/ta_wbmp/execution/*` 私有命名空间。参考所有权按阶段唯一：NAVIGATING 由 REMANI bridge 持有，TASK_EXEC 由 Coordinator 持有；Coordinator 负责显式交权，并调用独立的 `whole_body_force_control` 库对接触段做可选力修正。实机启动时 `execution_enabled` 仍必须保持为 `false`，直到后端、TF、Topic 唯一所有权和急停全部核验完毕。

## 2. 第一层：Task 任务轨迹生成

核心接口：

```cpp
class TaskTrajectoryProvider
{
public:
  virtual TaskTrajectory generate() const = 0;
};
```

默认实现 `TaskTrajectoryGenerator` 从 YAML 生成：

- `raster`：平面/曲面蛇形覆盖；
- `ras`：连续单笔 RAS 绘制；
- `waypoint_sequence`：通用末端约束序列，可表示抓取的 approach/grasp/lift/transfer/place；
- 用户自定义 `TaskTrajectoryProvider`：相机、CAD、点云、示教轨迹或在线任务生成器。

统一任务点：

```cpp
struct TaskWaypoint
{
  double progress;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d tangent;
  Eigen::Vector3d surface_normal;
  double nominal_speed;
  bool contact;
  std::string label;
};
```

任务生成层不知道 IK、REMANI、OCS2，也不选择底盘姿态。

## 3. 第二层：基于任务的全身规划

当前兼容入口是 `TaskAwarePlanner`，其职责是：

1. 读取 `TaskTrajectory`；
2. 采样底盘 standoff、纵向偏移和 IK 分支；
3. 对完整末端任务逐点求解机械臂和底盘状态；
4. 计算完整任务的瓶颈指标；
5. 根据可注入代价选择候选；
6. 生成预接触、任务切入和完整 9D 名义轨迹。

候选指标包括：

- 最大末端位置/姿态误差；
- 最小关节裕量；
- 最小 Yoshikawa manipulability；
- 最小 Jacobian 奇异值 `min_sigma`；
- 底盘/机械臂路径长度；
- 近似导航代价；
- 可行性和失败原因。

### 3.1 创新代价接口

```cpp
class CandidateCostEvaluator
{
public:
  virtual double evaluate(const CandidateMetrics& metrics) const = 0;
};
```

默认使用 YAML 权重的 `WeightedCandidateCost`。研究代码可注入新的代价，例如：

- future-task bottleneck margin；
- task-direction manipulability；
- 任务全程 ESDF clearance；
- 能耗、底盘转向、倒车次数；
- 导航和任务质量的 Pareto/学习排序。

### 3.2 碰撞与状态有效性接口

```cpp
class WholeBodyStateValidityChecker
{
public:
  virtual StateValidityResult check(const Eigen::VectorXd& state) const = 0;
};
```

默认使用 `UrdfSelfCollisionStateValidityChecker`，按 REMANI/WipePlanner 相同的
URDF 非相邻 link 配对规则检查自碰撞，并对预接近段、完整任务段及段间插值
全部检查。`AcceptAllStateValidityChecker` 只允许由离线测试显式注入，生产执行
路径不得使用。环境碰撞仍必须由与 REMANI 共用 ESDF、安全距离和 unknown-space
策略的 Adapter 注入；在该 Adapter 完成 L2 验证前，不能把 TA-WBMP 称为环境
碰撞闭环已完成。

`WholeBodyStateValidityChecker::checksEnvironment()` 是生产放行能力标志。当前
URDF checker 返回 `false`，因此 Coordinator 即使设置了
`execution_enabled:=true` 也会 fail-closed 拒绝发送 REMANI goal。只有共享
REMANI/ESDF adapter 覆盖该能力并完成环境检查后，生产执行才可放行；不能用
参数绕过。

### 3.3 导航代价接口

```cpp
class NavigationCostEstimator
{
public:
  virtual double estimate(const Eigen::VectorXd& start,
                          const Eigen::VectorXd& goal) const = 0;
};
```

当前默认使用 `Se2NavigationCostEstimator`。后续可以替换为：

- 2D occupancy-grid A*；
- REMANI coarse estimate；
- Top-K 完整 REMANI rollout；
- 含底盘/机械臂共同状态的任务入口代价。

## 4. 第三层：机器人执行契约

每次成功规划都显式输出：

```text
q_pre    : REMANI 需要到达的预任务 9D 全身状态
q_entry  : 任务接触/切入时的 9D 全身状态
tau_task : 交给 OCS2 MPC 跟踪的任务全身轨迹
```

ROS2 调试接口：

```text
/ta_wbmp/execution/remani_goal
  traj_utils/msg/WholeBodyGoal

/ta_wbmp/execution/ocs2_task_reference
  ocs2_msgs/msg/MpcTargetTrajectories
```

它们位于 TA-WBMP 私有命名空间，默认不会抢占现有 `/remani_planner/whole_body_goal` 或 `/mobile_manipulator_mpc_target`。正式集成时由唯一 Coordinator 在完成状态机、TF和所有权检查后进行 remap/转发：

```text
PLAN
  → publish q_pre to REMANI
  → wait navigation completion and measured 9D tolerance
  → request TASK_EXEC ownership
  → execute precontact transition
  → stream tau_task to OCS2
  → optional force correction on contact waypoints
  → monitor tracking/contact/safety
```

已实现的 `ta_wbmp_execution_coordinator_node` 状态机为：

```text
READY
  -- /ta_wbmp/execution/start --> NAVIGATING
  -- measured 9D arrival -----> REQUESTING_TASK
  -- REMANI TASK_EXEC ack ----> TASK_EXEC
  -- reference completed -----> COMPLETE
```

它将任务坐标系的底盘状态通过 TF 转为 OCS2 `control_frame`，保持六个关节量不变；滚动 MPC 窗口使用 OCS2 observation 时钟。若全身跟踪误差增大，虚拟任务进度会减速或暂停，而不是按墙上时间跳过轨迹点。

安全使用约束：

- `/mobile_manipulator_mpc_target` 按阶段只能有一个发布者：NAVIGATING 为 REMANI bridge，TASK_EXEC 为 Coordinator。交权必须先由 `/remani_bridge/set_reference_enabled` 确认 bridge 释放并观测 publisher 数为 0，再由 REMANI 确认 TASK_EXEC，最后由 Coordinator 建立 publisher 并观测数量为 1；
- Coordinator 负责参考所有权和 YAML 配置的力误差节流/硬力限锁存，但不替代硬件急停或独立碰撞停机；
- `world/map -> odom` TF 缺失时不发布 MPC 任务参考；
- 默认 `execution_enabled:=false`，调用 start 服务也会被拒绝，不会写实际 REMANI/OCS2 目标；
- `approach` 段仍是 TA-WBMP 生成的慢速名义接近；力修正只在 `contact=true` 的 `TASK_CONSTRAINED` 点激活，导航和预接触阶段不会预加载导纳。

仅启动 Coordinator 的安全准备模式（不会启动 REMANI、OCS2 或机器人）：

```bash
ros2 launch ta_wbmp execution_pipeline.launch.py \
  scenario:=blackboard execution_enabled:=false
```

集成栈检查完成后才可显式放行；建议仍保留 `auto_start:=false`，人工调用服务：

```bash
ros2 launch ta_wbmp execution_pipeline.launch.py \
  scenario:=blackboard control_frame:=odom execution_enabled:=true \
  force_control_enabled:=false
ros2 service call /ta_wbmp/execution/start std_srvs/srv/Trigger '{}'
```

## 5. 三个基准场景

| 场景 | 配置 | 轨迹生成器 | 底盘策略 |
|---|---|---|---|
| 擦桌子 | `config/table_wipe.yaml` | 平面 raster | 固定底盘，机械臂覆盖小桌面区域 |
| 擦黑板 | `config/blackboard_wipe.yaml` | 平面 raster | 沿任务方向移动底盘 |
| 画 RAS | `config/ras_drawing.yaml` | 0.90 m × 0.60 m 连续单笔 RAS | 底盘/机械臂联合随任务移动 |

运行：

```bash
ros2 launch ta_wbmp task_pipeline.launch.py scenario:=table
ros2 launch ta_wbmp task_pipeline.launch.py scenario:=blackboard
ros2 launch ta_wbmp task_pipeline.launch.py scenario:=ras
```

## 6. 实验数据接口

`ta_wbmp_scenario_runner` 为每个场景输出：

```text
experiment/
├── task_trajectory.csv
├── candidates.csv
├── whole_body_trajectory.csv
└── summary.yaml
```

- `task_trajectory.csv`：生成层输出；
- `candidates.csv`：所有候选、指标、代价和失败原因；
- `whole_body_trajectory.csv`：阶段化9D规划结果；
- `summary.yaml`：选中候选、质量指标、q_pre/q_entry 和执行后端契约。

这四个文件分别支持任务轨迹实验、候选代价消融、全身规划指标分析和执行复现。

## 7. 新任务接入方式

### 7.1 只增加 YAML

如果任务可以表示成末端 waypoint 序列，使用：

```yaml
pattern:
  type: waypoint_sequence
  sample_spacing: 0.03
  tangential_speed: 0.05
  waypoints:
    - {label: approach, position: [...], contact: false}
    - {label: grasp, position: [...], contact: true}
    - {label: lift, position: [...], contact: false}
```

### 7.2 自定义生成器

继承 `TaskTrajectoryProvider`，输出统一 `TaskTrajectory`。全身规划、候选代价、实验输出和执行接口不需要修改。

### 7.3 自定义研究模块

- 新入口代价：实现 `CandidateCostEvaluator`；
- 新碰撞/ESDF：实现 `WholeBodyStateValidityChecker`；
- 新导航估价：实现 `NavigationCostEstimator`；
- 新轨迹生成：实现 `TaskTrajectoryProvider`。

## 8. 当前验证边界

已验证的是三类任务的离线轨迹生成、IK全身规划、约束报告、候选选择、执行消息构造，以及 Coordinator 在 `execution_enabled=false` 下的安全准备行为。

尚未由本次重构证明：

- 三场景的真实 REMANI 导航成功；
- 统一 ESDF 环境碰撞距离；
- Coordinator 驱动 OCS2/MuJoCo 闭环完成整条任务；
- 真实桌面/黑板接触力和抓取稳定性；
- 实机安全与跟踪误差。

这些必须在唯一执行 Coordinator、统一地图/ESDF和硬件安全检查完成后继续验证。
