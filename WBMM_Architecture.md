# WBMM 任务全身规划 + OCS2 跟踪执行：最终代码架构

> 状态：目标架构，不是一次性生成的 TODO 列表。  
> 原则：先数学合同，再代码；先跑通最小闭环，再增加抽象。

---

## 0. 最终结论

保留三条不可动摇的边界：

```text
Search        -> Topology + Initial Guess
Optimization  -> Nominal WholeBodyTrajectory
OCS2          -> Tracking / Execution
```

最终数据主干：

```text
TaskTrajectory
      |
      v
SearchResult
      |
      v
WholeBodyTrajectory      <-- 规划完成后唯一机器人名义运动轨迹
      |
      v
OCS2 Reference
      |
      v
WholeBodyCommand
```

注意精确表述：

> 不是整个项目只能存在一种 trajectory；  
> 而是“规划完成以后，机器人运动只认一种 canonical `WholeBodyTrajectory`”。

- `TaskTrajectory`：任务要求机器人末端做什么。
- `SearchResult`：哪条路、哪个解族值得继续优化。
- `WholeBodyTrajectory`：整个机器人具体怎么运动。
- `OCS2 Reference`：OCS2 可跟踪的滚动参考。
- `WholeBodyCommand`：真正发给机器人/仿真器的控制命令。

---

## 1. 三条硬边界

### 1.1 Search

只负责：

- 差速底盘 `(x, y, yaw)` 的可行通道；
- 障碍物；
- Task Entry Region；
- 粗略机械臂可达性；
- 初始臂构型。

输出：

```text
Topology + Initial Guess
```

不负责最终轨迹质量。

### 1.2 Optimization

只负责：

- 把粗解变成连续、光滑、碰撞安全、任务可行的全身名义轨迹；
- 正确建立差速底盘状态转移；
- 激活相位相关约束；
- 输出 `WholeBodyTrajectory`。

不负责 ROS、可视化、OCS2 通信。

### 1.3 OCS2

只负责：

- 跟踪名义轨迹；
- 动力学与输入限制；
- 扰动抑制；
- 滚动时域局部修正。

如果 MPC 发现原轨迹拓扑上不可行，应该触发重规划，而不是让 MPC 自己承担全局重规划。

---

## 2. 数学合同必须先写死

写代码前，先固定：

```text
x = [base_x, base_y, base_yaw, q1, q2, q3, q4, q5, q6]   // 9D
u = [v, omega, qdot1, qdot2, qdot3, qdot4, qdot5, qdot6]  // 8D
```

末端位姿：

```text
T_ee = FK(x)
```

差速底盘连续模型：

```text
x_dot     = v * cos(yaw)
y_dot     = v * sin(yaw)
yaw_dot   = omega
q_dot     = u_arm
```

离散状态转移：

```text
x_{k+1} = f_diff(x_k, u_k, dt)
```

如果优化变量同时包含 `x_k` 和 `u_k`，必须加 defect constraint：

```text
x_{k+1} - f_diff(x_k, u_k, dt) = 0
```

如果第一版使用状态参数化，则必须至少显式约束横向滑移：

```text
Delta_x   = x_{k+1} - x_k
Delta_y   = y_{k+1} - y_k
Delta_yaw = wrap(yaw_{k+1} - yaw_k)

r_lateral = -sin(yaw_k) * Delta_x + cos(yaw_k) * Delta_y
```

要求：

```text
r_lateral = 0
```

否则优化器仍然可能生成“横移”的伪轨迹。

---

## 3. 最终类型边界

### 3.1 TaskTrajectory

语义：任务层要求末端做什么。

```cpp
struct TaskTrajectoryPoint
{
  double time_from_start;
  Pose pose;
  Vector3 tangent;
  Vector3 surface_normal;
  bool contact;
};
```

可用 phase：

```text
IDLE
NAVIGATE
PRE_EXECUTION
EXECUTION
TRACKING
FINISH
FAULT
```

第一版任务轨迹只给位置和姿态，不给末端速度；力控也不进入任务轨迹。  
`TaskTrajectory` 只描述名义任务，力控在执行层修正 `WholeBodyTrajectory`。

### 3.2 SearchResult

语义：拓扑 + 初值，不是最终轨迹。

```cpp
struct SearchResult
{
  std::vector<BaseState> base_path;
  std::vector<JointState> arm_seed;
  std::vector<ExecutionPhase> phases;

  double path_length{0.0};
  double solve_time{0.0};
  bool success{false};
};
```

要求：

```text
base_path.size() == arm_seed.size() == phases.size()
```

`base_path[i]` 和 `arm_seed[i]` 对应同一个全身节点。

### 3.3 WholeBodyTrajectory

语义：最终机器人名义运动轨迹。

```cpp
struct WholeBodyTrajectoryPoint
{
  double time_from_start{0.0};
  WholeBodyState state;
  std::optional<WholeBodyInput> feedforward_input;
  ExecutionPhase phase{ExecutionPhase::kIdle};

  // 可选：只关联任务点，不复制整条 TaskTrajectory
  std::optional<TaskTrajectoryPoint> task_reference;
};

struct WholeBodyTrajectory
{
  std::string trajectory_id;
  std::uint64_t environment_revision{0};
  std::uint64_t collision_model_revision{0};
  std::vector<WholeBodyTrajectoryPoint> points;
};
```

### 3.4 OCS2 Reference

通过 `ReferenceAdapter` 转换：

```text
WholeBodyTrajectory
    -> ocs2_msgs/MpcTargetTrajectories
```

不要把 `TaskTrajectory` 直接当成 OCS2 参考。

---

## 4. 目标仓库结构

### 4.1 长期目标

```text
src/
├── core/wbmm_core/                    # 纯 C++ 稳定合同
├── wbmm/                              # 唯一主工程包
│   ├── include/wbmm/
│   │   ├── task/
│   │   ├── search/
│   │   ├── optimization/
│   │   ├── reference/
│   │   ├── control/
│   │   ├── safety/
│   │   ├── logging/
│   │   └── runtime/
│   ├── src/
│   ├── config/
│   ├── launch/
│   └── test/
├── visual/wbmm_visualization/         # 已有，保留
├── control/tracer_jaka_ocs2/          # 已有，OCS2 Adapter
├── planning/ta_wbmp/                  # 原型，逐步迁移
├── applications/wiping/wipe_planner/  # 具体任务生成
├── bringup/                           # launch/config 总装
└── vendor/
    ├── remani_planner/
    └── ocs2_ros2/
```

### 4.2 第一版实际只创建这些文件

不要一开始创建全部类。

```text
src/
├── core/wbmm_core/
│   └── include/wbmm_core/
│       ├── types.hpp
│       ├── trajectory.hpp
│       └── robot_model.hpp
│
└── wbmm/
    ├── include/wbmm/
    │   ├── task/
    │   │   └── task_entry.hpp
    │   ├── search/
    │   │   └── search_planner.hpp
    │   ├── optimization/
    │   │   ├── trajectory_optimizer.hpp
    │   │   └── whole_body_problem.hpp
    │   └── reference/
    │       └── ocs2_reference_adapter.hpp
    │
    └── src/
        ├── task/task_entry.cpp
        ├── search/kino_astar.cpp
        ├── optimization/whole_body_problem.cpp
        ├── optimization/lbfgs_optimizer.cpp
        ├── reference/ocs2_reference_adapter.cpp
        └── logging/experiment_logger.cpp
```

原则：

> 先出现第二个实现，再抽象接口。  
> 不要为了未来可能存在第二个实现，把所有东西提前接口化。

---

## 5. 真正需要抽象的接口

只保留这几类真正有替换需求的边界：

```text
SearchPlanner
TrajectoryOptimizer
CollisionValidator
DistanceField
TrajectoryTracker
RobotAdapter
```

### 5.1 SearchPlanner

```cpp
class SearchPlanner
{
public:
  virtual ~SearchPlanner() = default;

  virtual Result<SearchResult> search(
    const SearchProblem& problem,
    const OperationContext& context) = 0;
};
```

可替换：

- Kino A*
- Hybrid A*
- RRT-Connect
- REMANI Adapter

### 5.2 TrajectoryOptimizer

```cpp
class TrajectoryOptimizer
{
public:
  virtual ~TrajectoryOptimizer() = default;

  virtual Result<OptimizationResult> optimize(
    const OptimizationProblem& problem,
    const OperationContext& context) = 0;
};
```

可替换：

- L-BFGS
- SQP
- IPM

### 5.3 CollisionValidator

用于 Search / 离散检查。

```cpp
class CollisionValidator
{
public:
  virtual ~CollisionValidator() = default;

  virtual Result<CollisionCheckResult> checkState(
    const WholeBodyState& state,
    const EnvironmentSnapshot& environment) = 0;

  virtual Result<CollisionCheckResult> checkSegment(
    const WholeBodyState& from,
    const WholeBodyState& to,
    const EnvironmentSnapshot& environment) = 0;
};
```

### 5.4 DistanceField

用于 Optimization，必须提供距离梯度。

```cpp
class DistanceField
{
public:
  virtual ~DistanceField() = default;

  virtual Result<DistanceQueryResult> query(
    const Vector3& point,
    const EnvironmentSnapshot& environment) const = 0;
};

struct DistanceQueryResult
{
  double signed_distance_m;
  Vector3 gradient;
  std::uint64_t environment_revision;
};
```

不要把 Search 用的碰撞检查器和后端梯度距离场做成同一种接口。

### 5.5 TrajectoryTracker

```cpp
class TrajectoryTracker
{
public:
  virtual ~TrajectoryTracker() = default;

  virtual Result<WholeBodyInput> update(
    const TrackingRequest& request,
    const OperationContext& context) = 0;
};
```

可替换：

- `Ocs2TrackerAdapter`
- `WholeBodyQpTracker`
- `SimplePdTracker`

### 5.6 RobotAdapter

```cpp
class RobotAdapter
{
public:
  virtual ~RobotAdapter() = default;

  virtual Result<WholeBodyState> readState() = 0;
  virtual Status sendCommand(const WholeBodyInput& command) = 0;
};
```

可替换：

- `MujocoRobotAdapter`
- `TracerJakaRobotAdapter`

---

## 6. 哪些东西不要接口化

不要为每一个 cost 都建立抽象基类。

第一版内建：

```cpp
class WholeBodyObjective
{
public:
  CostBreakdown evaluate(const TrajectoryVariables& x) const;

private:
  double taskCost(const TrajectoryVariables& x) const;
  double smoothnessCost(const TrajectoryVariables& x) const;
  double collisionCost(const TrajectoryVariables& x) const;
  double manipulabilityCost(const TrajectoryVariables& x) const;
  double postureCost(const TrajectoryVariables& x) const;
};
```

等真正需要第二个求解器或第二种代价实现，再抽接口。

Constraint 也不要只返回一个标量 violation。

推荐：

```cpp
struct ConstraintEvaluation
{
  Eigen::VectorXd value;
  Eigen::MatrixXd jacobian;
};

class Constraint
{
public:
  virtual ~Constraint() = default;

  virtual ConstraintEvaluation evaluate(
    const TrajectoryVariables& x) const = 0;
};
```

因为 SQP/IPM 需要的是：

```text
c(x) in R^m
```

而不是一个标量。

---

## 7. Search 设计

### 7.1 搜索空间

搜索：

```text
(x, y, yaw)
```

不要直接在 9D 空间 A*。

节点代价中加入：

- task reachability；
- arm IK feasibility；
- manipulability；
- whole-body collision；
- task entry cost。

### 7.2 Task Entry Region

导航终点不是一个固定点，而是一个区域：

```text
G_task = { base_state | task reachable and safe }
```

判断条件：

```text
IK(T_task,0, x_b, y_b, yaw_b) 存在
m(q) > m_min
d(q) > d_safe
```

### 7.3 分层判定

不要每扩展一个 A* 节点就做完整 6DOF IK + 全身碰撞。

推荐：

```text
cheap reachability
      |
      v
IK candidate
      |
      v
full validation
```

顺序：

1. 预计算 workspace / reachability map；
2. 靠近 Task Entry Region 时才做 IK；
3. 最后才做全身碰撞和 manipulability 验证。

---

## 8. Optimization 设计

### 8.1 第一版固定 `dt`

第一版：

```text
dt = fixed
```

只优化：

```text
x_k
u_k
```

或者：

```text
u_k
```

跑通后再加入：

```text
T_k
```

原因：

- 速度/加速度约束会变复杂；
- smoothness 对时间敏感；
- gradient 更难调；
- L-BFGS 容易产生极小 segment time。

### 8.2 差速非完整约束

方案 A：

```text
x_{k+1} - f_diff(x_k, u_k, dt) = 0
```

方案 B：

```text
r_lateral = -sin(yaw_k) * Delta_x + cos(yaw_k) * Delta_y = 0
```

第一版推荐方案 B，公式简单、容易实现、数值稳定。

### 8.3 第一版代价

```text
J = w_task    * J_task
  + w_smooth  * J_smooth
  + w_posture * J_posture
```

先跑通：

```text
Search -> Optimize -> RViz
```

再逐个加入：

```text
J_collision
J_manip
J_base
J_arm
velocity/nonholonomic
phase
```

### 8.4 末端任务约束

第一版：

```text
r_ee(t) = log(T_ee_des(t)^(-1) * T_ee(q(t)))
J_task  = r_ee(t)^T Q r_ee(t)
```

第一版先软约束：

```text
w_task high
```

后续再加：

- 增广拉格朗日；
- 零空间投影；
- SQP / IPM 硬约束。

### 8.5 相位是约束激活计划

不要只把 phase 理解成权重调度。

例如：

```text
NAVIGATE:
  底盘导航到任务入口，末端约束弱或关闭

PRE_EXECUTION:
  末端对齐和接近准备，逐渐激活任务约束

EXECUTION:
  执行任务，末端约束最强

TRACKING:
  纯轨迹跟踪，维持全身轨迹连续性和安全性

FINISH:
  撤退、停止和安全收尾
```

真正有价值的是：

```text
任务约束的结构随 phase 改变
```

而不仅仅是：

```text
w_task: 1 -> 1000
```

---

## 9. Reference 与 OCS2 集成

### 9.1 单一 ReferenceManager

不要通过 ROS publisher 数量判断所有权。

始终只有一个：

```text
ReferencePublisher
```

其他模块只提交：

```cpp
struct ReferenceRequest
{
  ReferenceOwner owner;
  std::uint64_t generation;
  WholeBodyTrajectory trajectory;
  ReferenceMode mode;
};
```

`ReferenceManager` 根据：

```text
NORMAL
CONTACT_CORRECTION
REPLAN
SAFE_HOLD
```

决定当前参考。

这样从架构上就不可能出现两个模块抢 topic。

### 9.2 Phase schedule

内部 C++ 使用：

```cpp
struct PhaseSegment
{
  double start_time;
  double end_time;
  ExecutionPhase phase;
  std::string task_id;
  bool contact;
};

using PhaseSchedule = std::vector<PhaseSegment>;
```

ROS 传输第一版可以继续兼容：

```text
std_msgs/String
"<t0> <PHASE0>;<t1> <PHASE1>;..."
```

但长期应改为 typed message：

```text
PhaseSchedule
  PhaseSegment[] segments
```

### 9.3 OCS2 边界

Planner 负责：

- 绕哪个障碍；
- 从哪个任务入口进入；
- base 大尺度走哪里；
- 生成 task-feasible reference。

OCS2 负责：

- tracking；
- disturbance；
- 小范围 base/arm redistribution；
- 输入/状态限制。

如果 MPC 发现原轨迹根本不可行：

```text
replan
```

而不是让 MPC 自己承担几十米导航的重新优化。

---

## 10. 可视化与日志：第一天就做

算法不直接发 Marker。

算法返回：

```text
SearchResult
InitialGuess
WholeBodyTrajectory
OptimizationTrace
```

可视化由 `wbmm_visualization` 统一完成。

固定主题：

```text
/wbmm/task_trajectory
/wbmm/search/raw_base_path
/wbmm/search/expanded_nodes
/wbmm/search/task_entry_region
/wbmm/planner/initial_guess
/wbmm/planner/optimized_trajectory
/wbmm/planner/ee_trajectory
/wbmm/controller/reference
/wbmm/controller/prediction
/wbmm/controller/executed
/wbmm/debug/collision_distance
/wbmm/debug/manipulability
/wbmm/debug/task_error
```

颜色：

| 颜色 | 含义 |
|---|---|
| 灰色 | Search result |
| 黄色 | Initial guess |
| 绿色 | Optimized trajectory |
| 蓝色 | MPC prediction |
| 红色 | Executed trajectory |

实验目录：

```text
runs/
└── 2026-09-10_001/
    ├── config.yaml
    ├── metadata.json
    ├── task_trajectory.csv
    ├── search_result.csv
    ├── initial_trajectory.csv
    ├── optimized_trajectory.csv
    ├── optimizer_trace.csv
    ├── mpc_reference.csv
    ├── executed_trajectory.csv
    ├── metrics.json
    ├── rosbag2/
    └── plots/
```

`optimizer_trace.csv`：

```text
iteration,total_cost,task_cost,collision_cost,smooth_cost,solve_time
```

`metrics.json`：

```json
{
  "search_time_ms": 23.5,
  "optimization_time_ms": 81.2,
  "max_task_error": 0.021,
  "min_collision_distance": 0.126,
  "min_manipulability": 0.31,
  "mpc_mean_tracking_error": 0.012
}
```

必须单独记录：

```text
J_task
J_collision
J_smooth
J_manip
```

便于后续 ablation。

---

## 11. 仿真与实机

更准确的表述是：

```text
Planning/Core 不区分 MuJoCo / Real
Runtime + Safety 必然存在平台特异实现
```

平台差异：

| 层 | 仿真 | 实机 |
|---|---|---|
| State | MujocoRobotAdapter | TracerJakaRobotAdapter |
| Command | MujocoCommandSink | TracerJakaCommandSink |
| Safety | 仿真安全门 | 急停、watchdog、力零点、故障恢复 |
| Runtime | MuJoCo launch | 真机 launch |

实机额外需要：

- actuator delay；
- communication timeout；
- hardware state；
- emergency stop；
- command watchdog；
- force zeroing；
- fault recovery；
- mode switch。

这些属于 Runtime + Safety，不属于 Planning Core。

---

## 12. 开发顺序

### Step 0：Math Contract

先写清：

```text
x = [x_b, y_b, yaw_b, q]
u = [v, omega, qdot]
T_ee = FK(x)
x_{k+1} = f_diff(x_k, u_k, dt)
```

### Step 1：Core Data Model

- `WholeBodyState`
- `WholeBodyInput`
- `TaskTrajectory`
- `SearchResult`
- `WholeBodyTrajectory`
- `PhaseSegment`

### Step 2：Base Search

- Kino A*；
- 差速模型；
- 避障；
- 输出 base seed；
- RViz 可视化。

### Step 3：Task Entry Region

- 从固定 base goal 升级为任务入口区域；
- cheap reachability；
- IK candidate；
- full validation。

### Step 4：Whole-body Seed

- 给 base path 配 IK / arm posture；
- 产生 9D 初始轨迹。

### Step 5：Minimal Whole-body Optimizer

只放：

```text
J_task + J_smooth + J_posture
```

先跑通：

```text
Search -> Optimize -> RViz
```

### Step 6：逐项加代价

顺序：

```text
J_collision
J_manip
J_base
velocity/nonholonomic
phase
```

### Step 7：Phase-dependent Constraints

实现：

```text
IDLE
NAVIGATE
PRE_EXECUTION
EXECUTION
TRACKING
FINISH
FAULT
```

### Step 8：OCS2 Tracker

- `WholeBodyTrajectory` -> OCS2 reference；
- 复用 `remani_to_ocs2_reference_bridge`。

### Step 9：MuJoCo -> Real

- 替换 Adapter；
- 接入安全监控；
- 实机默认 `execution_enabled=false`。

---

## 13. 第一版验收标准

第一版只要求以下闭环成立：

```text
Core Data
  -> SearchPlanner
  -> Kino A*
  -> Task Entry Region
  -> SearchResult
  -> RViz
  -> CSV Logger
```

验收条件：

- 给定任务和 start state，可以输出一条 base seed；
- 可以判断 Task Entry Region；
- 可以在 RViz 中看到 search path 和 task entry region；
- 可以把 `SearchResult` 写入 CSV；
- 不依赖 OCS2；
- 不依赖优化器；
- 不依赖真实机器人。

然后再做：

```text
SearchResult
  -> WholeBodySeed
  -> Minimal Optimizer
  -> WholeBodyTrajectory
  -> OCS2
```

---

## 14. Codex / 工程任务粒度

不要给 Codex：

```text
按照这份架构全部实现。
```

建议拆成：

```text
任务 1：
实现 SearchPlanner 接口下的 KinoAStarSearch。
状态只包含 (x,y,yaw)。
输入输出遵循 core/types.hpp。
不能依赖 ROS2，不能修改 optimizer。

任务 2：
实现 TaskEntryRegion。
只负责判断状态是否可进入任务。
不能调用 solver。

任务 3：
实现 LbfgsTrajectoryOptimizer。
只组合已有 cost，不能重新实现 FK、collision 或 manipulability。
```

关键原则：

> 文档是目标架构，不是一次性生成的 TODO 列表。  
> 先自己掌握前 20%：Core -> SearchPlanner -> KinoA* -> TaskEntryRegion -> SearchResult。

---

## 15. 关键修改清单

相比初版方案，最终版明确修改：

1. `TaskTrajectory` 和 `WholeBodyTrajectory` 语义分离；
2. `WholeBodyTrajectory` 是规划后的唯一机器人名义轨迹；
3. 后端必须显式建立差速状态转移/横向滑移约束；
4. 第一版固定 `dt`，不优化时间；
5. `Constraint` 返回向量残差 + Jacobian，不返回标量 violation；
6. Collision 拆成 `CollisionValidator` 和 `DistanceField`；
7. 只为 Search、Optimizer、Collision、Tracker、Robot 做接口；
8. Cost 不全部虚函数化；
9. Reference 使用单一 `ReferenceManager`，不依赖 publisher 数量；
10. Phase schedule 内部 typed，ROS 第一版可临时用字符串；
11. Phase 是约束激活计划，不只是权重调度；
12. OCS2 只做局部动态修正，不可行就重规划；
13. Planning/Core 不分仿真/实机，Runtime/Safety 才分；
14. 第一版只做最小闭环，后续按需抽象。

---

## 16. 一句话总结

> 保留 `Task -> Search -> WholeBodyOptimization -> WholeBodyTrajectory -> OCS2` 的骨架；  
> 真正需要抽象的只有 Search、Optimizer、Collision、Tracker、Robot Adapter；  
> 后端先解决差速状态转移，再谈时间优化和硬约束；  
> 先跑通 `Search -> MinimalOptimizer -> RViz -> CSV`，再逐步加入碰撞、操作度、相位和 OCS2。
