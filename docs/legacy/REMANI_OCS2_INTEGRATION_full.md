# REMANI 规划 → OCS2 MPC 跟踪 原理

本文解释本工程中：REMANI 规划出的轨迹，是如何被 OCS2 MPC（GaussNewtonDDP / SLQ）跟踪执行的。
对应代码：`tracer_jaka_ocs2/src/remani_to_ocs2_reference_bridge.cpp`、`TracerJakaMpcNode.cpp`、`TracerJakaMrtNode.cpp`，
以及 `basic examples/ocs2_mobile_manipulator/` 下的 cost / interface / dynamics。

## 1. 两条本质不同的轨迹

REMANI 和 MPC 工作在完全不同的状态空间里，中间必须有一层"翻译"。

| | REMANI（规划器） | OCS2 MPC（跟踪器） |
|---|---|---|
| 状态 | 平坦输出 `p = [x, y, q1..q6]`（8 维） | 全状态 `x = [x, y, yaw, q1..q6]`（9 维） |
| 控制 | 隐含在多项式系数里 | `u = [v, omega, qdot1..qdot6]`（8 维） |
| 时间表示 | 分段多项式（piecewise polynomial），按 `trajectory_id` 分片 | 离散航点 `(t_k, x_k, u_k)` 的时间序列 |
| 非完整约束 | 用 `singul = +1/-1` 表示前进/倒车齿轮 | 内建于动力学模型（bicycle model） |

**所以"MPC 跟踪轨迹"不是简单播放 REMANI 的速度，而是：**

```
REMANI 多项式轨迹
   │  /planning/trajectory (quadrotor_msgs/PolynomialTraj)
   ▼
remani_to_ocs2_reference_bridge  ← 采样多项式 + 重建 yaw/v/omega + 坐标变换
   │  mobile_manipulator_mpc_target (ocs2_msgs/MpcTargetTrajectories)
   ▼
RosReferenceManager → ReferenceManager（MPC 的"参考轨迹"缓存）
   │
   ▼
GaussNewtonDDP_MPC 求解：min Σ [状态跟踪代价 + 输入代价]，受动力学与软约束限制
   │  mobile_manipulator_mpc_policy
   ▼
MRT 节点按实测状态每周期求值 policy → /base_controller/cmd_vel + /arm_controller/commands
   ▼
真实/仿真机器人
```

## 2. 桥接层：把多项式变成参考轨迹

`remani_to_ocs2_reference_bridge` 分三步做"翻译"：

1. **按 `trajectory_id` 拼接分段**。REMANI 无 count 字段，用 `id==1` 表示批开始、
   每次收到消息重置 debounce timer（默认 0.04 s），timer 到期即 `finishAssembly()` 组装成一条完整轨迹。
   新轨迹先进 `pending_`，直到其 `startStamp`（消息 header.stamp 对应时刻）到了才切到 `active_`——
   避免新轨迹过早覆盖旧轨迹造成跳变。

2. **采样多项式**。每段是若干 `PolynomialMatrix` piece，系数列主序存储（第 0 列为最高次、末列为常数项）。
   对时刻 t 求位置/速度/加速度：
   ```
   pos(t) = Σ c_k · t^(deg-k)
   vel(t) = Σ (deg-k) · c_k · t^(deg-k-1)
   acc(t) = Σ (deg-k)(deg-k-1) · c_k · t^(deg-k-2)
   ```

3. **重建 yaw、前向速度、角速度**（REMANI 平坦空间 → 非完整全状态的关键一步）。
   设采样得 (px, py, vx, vy, ax, ay)，先做可配置的 2D 刚体变换到 odom 系，然后：
   ```
   speed² = vx² + vy²

   yaw = unwrapNear( atan2(singul·vy, singul·vx), yaw_prev )   // singul∈{+1,-1}
   v   = singul · sqrt(speed²)                                  // 带符号线速度
   ω   = (vx·ay − vy·ax) / speed²                               // 路径的"自然"偏航率
   ```
   - `atan2(singul·vy, singul·vx)`：倒车（singul=−1）时车头仍朝前，v 为负。
   - `ω` 是让车头始终贴着速度方向所需的偏航率（heading = atan2(vy,vx) 的导数）。
   - 当 `speed²` 低于阈值（起/终点、换向点）：`yaw` 沿用上一帧、`v=ω=0`，
     避免 atan2 数值不稳定导致 yaw 振荡。
   - 每条轨迹结束时把输入强制置零（保持终端位姿），直到新轨迹或 abort 到来。

4. **发布滚动参考窗**。每 20 Hz 一次：
   - 第 0 帧 = 最新 MPC 观测状态（锚定帧，消除参考跳变）；
   - 后续按 `sample_dt=0.04 s` 采样约 3 s 前瞻窗口（约 76 帧），
     时间轴对齐 OCS2 观测时钟（`observationTime + (now − observationRosStamp)`）。
   - 经 `TargetTrajectoriesRosPublisher` 发布到 `mobile_manipulator_mpc_target`。

## 3. MPC 层：滚动时域的跟踪优化

`TracerJakaMpcNode` 构造 `GaussNewtonDDP_MPC`（`task.info` 里 `algorithm SLQ`）。
`RosReferenceManager` 订阅 `mobile_manipulator_mpc_target`，把桥接层发的参考写入
`ReferenceManager`；MPC 求解时在每个预测时刻 `t` 查询参考（线性插值，yaw 走最短角路径）。

**跟踪代价**（`WholeBodyTrajectoryCost`，见 `MobileManipulatorInterface.cpp`）：

```
J_state(x,t) = ½ (x − x_d(t))ᵀ Q (x − x_d(t))
```

- `x_d(t)` 由 ReferenceManager 按时间插值得到（越界钳位到首/末航点）。
- `Q` 来自 `wholeBodyTracking.Q`：base = diag(5, 5, 2)（x, y, yaw），arm = diag(2,…,2)。
- 终端代价同结构，`finalWeightScale = 1.0`。
- yaw 误差先 wrap 到 [-π, π] 再做二次型。

**输入代价**（`QuadraticInputCost`）：

```
J_input(u) = ½ uᵀ R u
```

- `R` 带 scaling：base `0.1 × diag(2.5, 10.0) = diag(0.25, 1.0)`（转向 ω 阻尼更强，
  抑制末端左右抖动）；arm `1e-2 × diag(1,…,1) = diag(0.01,…,0.01)`。

**动力学**（`WheelBasedMobileManipulatorDynamics`，bicycle model，速度级输入）：

```
ẋ = [ v·cos(yaw), v·sin(yaw), ω, q̇1..q̇6 ]
```

这是 MPC 内部用来前向积分（rollout）的模型。正是它保证了跟踪结果是**动力学可行**的——
桥接层给的前馈输入只是参考，MPC 会在满足该模型的前提下重算自己的输入。

**软约束**（把"偏离参考"的答案拉回来）：
- 关节位置限位（取自 URDF）+ 关节速度限位（`jointVelocityLimits`），`RelaxedBarrierPenalty`；
- 自碰撞（`selfCollision`，`ThresholdRelaxedBarrierPenalty`，minDist 0.02 m）；
- 环境避障（`environmentCollision`，对静态障碍盒 low table / 墙做距离约束，
  `activationDistance 0.20 m`，minDist 0.05 m）。

**求解配置**（`mpc` 节）：时域 2.0 s，`maxNumIterations=1`（每周期只做一次 SLQ 迭代），
`mpcDesiredFrequency=100 Hz`，`coldStart=false`（用上一帧解热启动），`useFeedbackPolicy=true`
→ 输出是一条**仿射反馈律** `u*(t) + K(t)·(x − x*(t))`，不只是开环前馈。

所以"跟踪"的本质是：**在每个 MPC 周期，从当前实测状态出发，重新解一个使
`J_state + J_input` 最小的 2 秒最优控制问题，其参考轨迹就是 REMANI 轨迹**
——参考只决定"想去哪"，MPC 决定"怎么去、去不去的成"（受动力学/限位/碰撞约束）。

## 4. MRT 层：把策略落到控制器

`TracerJakaMrtNode`（MRT = MPC Real-Time）以 125 Hz 闭环：

1. 从 EKF 里程计取底盘 (x, y, yaw)，从 `/joint_states` 取 6 关节角，组成观测 x。
2. `mrt_->updatePolicy()` 收取最新 policy；`evaluatePolicy(obs.time, obs.state)` 得到
   当前最优输入 `u*(t)` 与预测状态 `x*(t)`。
3. **底盘**：直接发 `u*(t)[0:2] = (v, ω)` → `/base_controller/cmd_vel`。
4. **机械臂**：求 `t + traj_horizon`（launch 默认 1.0 s）处的预测关节位置，发
   `Float64MultiArray` → `/arm_controller/commands`（ForwardCommandController）。
   用超前一点的目标点是为了给底层关节控制器一个"前视"，避免每拍只追当前位置。
5. 安全兜底：policy 时间越界 / 含 NaN / 关节一步跳变超过 `arm_max_delta_per_step(0.5 rad)`
   时，底盘停、机械臂保持当前位姿。

这样形成了两条反馈回路：
- **内环**：MRT 每 8 ms 用实测状态求值反馈律 → 抑制扰动与跟踪误差；
- **外环**：MPC 每 10 ms 从最新观测重解 → 滚动时域跟踪；
- **更外层**：REMANI FSM 的 `tracking_error_replan_enabled` 在实测偏离规划超过阈值时
  重新规划，发布新轨迹 → 桥接层 → MPC，形成"规划-跟踪-再规划"闭环。

## 5. 关键参数速查

| 量 | 值 | 位置 |
|---|---|---|
| MPC 时域 | 2.0 s | `task.info` mpc.timeHorizon |
| MPC 频率 | 100 Hz | `task.info` mpc.mpcDesiredFrequency |
| 参考发布频率 / 采样 | 20 Hz / dt=0.04 s，约 3 s 窗 | bridge 参数 `publish_rate`/`sample_dt`/`reference_horizon` |
| Q base (x,y,yaw) | 5, 5, 2 | `task.info` wholeBodyTracking.Q.base |
| Q arm | 2 × 6 | `task.info` wholeBodyTracking.Q.arm |
| R base (v,ω) 有效 | 0.25, 1.0 | `inputCost.R.base` scaling 0.1 × (2.5, 10) |
| R arm 有效 | 0.01 × I₆ | `inputCost.R.arm` scaling 1e-2 |
| 关节速度限位 | ±2.0 rad/s | `jointVelocityLimits` |
| 环境避障 | activation 0.20 m / minDist 0.05 m | `environmentCollision` |

## 6. 一句话总结

REMANI 只负责在平坦空间里"指路"（多项式 → 桥接层 → 参考轨迹 `x_d(t), u_d(t)`）；
OCS2 MPC 在每个控制周期把参考当作二次型代价的期望值，在一个**带动力学模型和软约束的
2 秒滚动最优控制问题**里重新求解输入，再由 MRT 用实测状态每 8 ms 求值反馈律下发底盘与关节命令。
参考提供"目标"，MPC 提供"可行、平滑、避障"的实现，MRT 负责"闭合真实反馈环"。

## 7. MPC 如何"知道"自己在跟踪哪条轨迹？时间误差怎么处理？

### 7.1 MPC 其实不知道"轨迹身份"

- MPC 侧唯一的信息是 ReferenceManager 里一份 `(t_k, x_k, u_k)` 时间序列快照，由桥接层每 20 Hz 整体刷新。
  每个 MPC 预测时刻 τ 只做一件事：`x_d(τ) = 对快照按时间插值`，然后最小化 `½(x − x_d(τ))ᵀ Q (x − x_d(τ))`。
- "正在跟踪哪条轨迹"这个身份**不在 MPC 里**，而在：
  - 桥接层：`active_` / `pending_` 按 `startStamp`（轨迹起始时刻）决定当前用哪条；
  - FSM：记录目标终点 `end_pt_` / `end_yaw_`，并判断 `measured_goal_reached`（到没到目标）。
- 桥接层每周期把参考第 0 帧锚定到当前实测状态，因此参考窗口永远"从你现在的位置开始"，
  MPC 只是追一个滚动参考，它不需要、也没有"我是不是在跟第 N 条轨迹"这种概念。
  如果桥接层收到 abort 发布的是保持位姿参考，MPC 就去"跟踪"保持位姿——它同样毫无察觉。

### 7.2 时间误差的产生（参考是"墙钟刚性"的）

桥接层 `sampleAt` 采样用的是绝对墙钟：`relativeTime = (rosStamp − startStamp)`。
也就是参考是一条**按墙钟排定行程**的曲线：

```
x_d(τ) = trajectory( wall_time(τ) − startStamp )
```

如果机器人晚到 Δt：在 MPC 时刻 τ，参考是 trajectory 上超前 Δt 行程的那个点。
MPC 没有"时间状态"，它只有"此刻位置 vs 参考此刻位置"的空间误差。
**时间误差被折算成了空间误差，靠 Q 代价去追。** 代价函数从不"记得 A 本该在 t=10s 到达"，
它只问"现在这一刻，你的位置离参考该在的位置差多少"。

### 7.3 本工程处理时间误差的三个机制

1. **MPC 内在的"追"**：`½(x − x_d(τ))ᵀ Q (x − x_d(τ))` + 输入限位。
   小滞后（比如几毫秒）→ 代价自动让机器人多走一点追平；
   但滞后一旦超过输入限位能补的范围（v ≤ 0.5 m/s，ω ≤ 1.0 rad/s），
   机器人会**永远落后**——因为参考按墙钟继续前进，MPC 每个周期都在追一个走掉的目标。

2. **桥接层锚定第 0 帧**（`publishReference` 把首帧设为实测状态）：
   保证参考窗口从当前实测开始，MPC 不会被要求追一个遥不可及的跳变点。
   但这只防"参考起点跳变"，**不消除时钟偏移**——后续帧仍是墙钟采样。

3. **REMANI tracking-error replan（真正的"校钟"）**：`remani_replan_fsm.cpp` 的 `EXEC_TRAJ`
   每个控制周期用**同一墙钟时刻**比较实测全身状态与参考状态，从而同时检测空间偏离和
   "控制器落后于时间参数化参考"两种情况。阈值：位置 > 0.30 m / yaw > 0.45 rad / 关节 > 0.30 rad，
   且误差持续 ≥ `tracking_error_persistence_`(0.3 s)、距上次重规划 ≥ 2 s →
   调用 `planNextWaypoint(end_pt_, end_yaw_)` **从当前实测状态重新规划**。
   新轨迹 `startStamp = now`（`msg.header.stamp` 由新的 `start_time` 生成），
   参考时钟从此刻清零，旧的行程安排作废。

4. **MRT 前视（traj_horizon）**：机械臂命令取 `t + 1.0 s` 的预测状态而不是当前状态，
   补偿底层关节控制器 / 执行器的响应延迟——这是对"执行晚了一点"最底层的补偿。

### 7.4 一般理论上的做法（本工程没做）

- **轨迹重参数化 / 时间缩放**：把参考时间 s 从墙钟解耦，改用"路径进度"参数化（例如弧长 s）。
  落后时把参考时钟调慢（ds/dt 减小），让"到 A 的时刻"变成可以伸缩的量而不是死值。
  经典解法，但需要给状态加进度变量或改代价结构。
- **Path following + 独立速度剖面**：只跟踪几何路径 π(λ)，把时间放进标量进度 s(t)，
  上层只控制 s(t) 快慢——空间跟踪与时间安排彻底解耦。
- **从源头保证可执行**：让 REMANI 用与 MPC 一致的动力学与速度/加速度限位来规划，
  减少"计划本身追不上"的情况。

### 7.5 如果想让跟踪更紧 / 更能吸收时间误差

- 提高 `Q.base`（尤其 x,y）→ 追得更紧，但更激进、更耗输入；
- 提高桥接层参考中前馈输入 `u_d` 的可信度（让代价更"听"前馈）；
- 把参考改为"当前状态到路径的投影 + 有限前瞻"（按路径距离而非墙钟推进），
  即 7.4 的第一种做法——这是让 MPC 自己吸收时间误差的正路。
