# Force-MPC 机械臂奇异性与操作度裕度

2026-10-08：已实现可配置的过程／终端代价、预测与实测指标、参考减速和执行拦截，并完成 MuJoCo 键盘施力对比。实机专用配置默认关闭新增代价和奇异性执行拦截；没有进行实机运动验证。

## 代码职责

- `src/control/wbmm_ocs2/include/wbmm_ocs2/cost/ArmManipulabilityCost.h` 定义并解释雅可比选择、量纲缩放、奇异值、Yoshikawa 操作度及各代价公式。
- 对应 `.cpp` 实现归一化残差、值、中心差分梯度、PSD Gauss–Newton 近似和参数检查；健康区间跳过不必要的十二次差分评价。每个代价／诊断实例独立复用 Pinocchio 工作区，避免每次评价复制整个 Data；计算和 clone 受互斥锁保护，并有并发一致性测试。
- `WbmmInterface.cpp` 读取 `task.info` 并独立注册 `armManipulability` 过程项及 `finalArmManipulability` 终端项。控制／传感器节点不注册 OCP 代价。
- `wbmm_robot_metrics` 继续提供通用 SVD 评价，OCS2 继续持有原生 Pinocchio 模型，避免复制运动学或创建第二套指标公式。
- `wbmm_ocs2_ros` 负责诊断和执行检查；`whole_body_force_control` 接收减速反馈，在导纳积分中缩放增量，并在拦截后冻结参考。

## 指标与代价

状态为 `[base_x, base_y, base_yaw, q1..q6]`，输入为 `[v, omega, qdot1..qdot6]`。雅可比行顺序是 `[linear; angular]`，参考点为 `tool0` 原点，表达坐标系为 `odom`。机械臂评价只取全身 `6x8` 雅可比最后六列，不允许底盘冗余掩盖机械臂奇异。

位姿任务使用 `Jbar = [Jv; ell * Jw]`，本轮固定 `ell=0.30 m`。定义：

\[
s=\sigma_{\min}(\bar J_a),\qquad
w=\prod_i\sigma_i=\sqrt{\det(\bar J_a\bar J_a^T)}.
\]

最小奇异值表示最弱方向的速度能力；操作度是速度椭球体积的尺度。不同任务维数、TCP 点或特征长度对应的阈值不能直接复用。两者都不能等价为实际关节速度盒约束下的可达速度，也不能单独证明硬件安全。

`normalizeMargins=true` 时，过程代价为：

\[
L_a=\frac{\alpha}{2}\left[
\lambda_s\max(0,1-s/s_r)^2+
\lambda_w\max(0,1-w/w_r)^2\right].
\]

其中 `alpha=weightScale`。只加入启用的指标项，达到裕度后相应惩罚为零。`normalizeMargins=false` 保留原有 `0.5 * weight * max(0, reference-metric)^2` 语义。权重／参考／缩放必须有效；归一化启用项的参考必须为正，且归一化权重不能溢出。

终端项使用同一实现和指标契约，但独立启用、独立调整权重和参考。终端参数默认继承过程参数。它鼓励预测结束时保留裕度，是软偏好；本轮实测最低奇异值仍低于 `0.12` 的期望裕度，不能把该目标当作硬下界。

代码还保留条件数、正则逆操作度和固定方向指标。逆操作度是 `trace((Jbar*Jbar^T + regularization*I)^-1)`，并非 `1/w`；方向指标是投影幅值，不能代替最小奇异值。本轮不默认启用这些项。完全奇异构型和重复奇异值附近的局部梯度可能不足以给出脱离方向，因此仿真使用已有 `low` 起始姿态。

## 启动和调参

用户原来的命令直接加载新的仿真配置：

```bash
ros2 launch tracer_jaka_bringup force_mpc.launch.py \
  backend:=sim fake_wrench:=true keyboard_wrench:=true
```

默认任务配置：

- 仿真：`src/bringup/config/sim/task_force_mpc.info`，启用奇异值过程／终端代价和执行拦截。
- 实机：`src/bringup/config/real/task_force_mpc.info`，新增代价和拦截默认关闭，硬件写入仍由原有门控控制。
- 其他导航／轨迹跟踪入口继续使用自己的 `task.info`。

本轮仿真关键参数：

```text
armManipulability
{
  activate true                  ; 只控制过程项
  normalizeMargins true
  weightScale 1.0
  metrics
  {
    scope "arm"
    task "pose"
    scaling "characteristic_length"
    characteristicLength 0.30
  }
  useMinSingularValue true
  minSingularRef 0.12
  minSingularWeight 0.5
  useYoshikawa false              ; 设为 true 叠加操作度
  yoshikawaRef 0.0015
  yoshikawaWeight 0.005
  terminal
  {
    activate true                ; 独立于过程 activate
    weightScale 1.0
    minSingularRef 0.12           ; 未列出的参数继承过程项
  }
  diagnostics { activate true }
}
```

终端可覆盖 `normalizeMargins`、三个下界指标的参考／权重，以及奇异值、Yoshikawa、逆操作度和条件数的开关／参数。过程的 frame／scope／task／scaling 是共同契约。可通过 `task_file:=/absolute/path/candidate.info` 指定完整配置。

专用仿真任务将 MPC 的关节速度软限值与实际 force-MPC 指令上限对齐：底盘 `v=±0.1 m/s`、`omega=±0.4 rad/s`，机械臂 `qdot=±0.2 rad/s`。三组对比都使用这些相同限值，没有提高执行速度来换取改善。原通用仿真 `task.info` 为底盘 `±0.5/±1.0`、机械臂 `±2.0`，与 force-MPC 实际指令上限存在差异。软限值仍可能被优化器违反，MRT 原有硬限速继续生效；修改 force YAML 速度时也必须同步调整专用任务配置。

## 诊断与执行拦截

两个 `Float64MultiArray` 诊断主题的 layout label 包含 frame、TCP 点、scope、task、scaling 和特征长度。无效指标保留 NaN／valid 标志，不伪造为零或健康值。

执行联锁首次故障原因单独锁存在 `/mobile_manipulator_force_execution_fault_reason`：`robot_state_timeout:...` 区分 odom／关节反馈超时，`controller_heartbeat_timeout` 表示控制器心跳超时，`controller:FAULT_...` 保留控制器上报的故障。日志同时记录三种反馈的墙钟年龄；后续健康心跳不会覆盖首次原因，成功显式复位后清空。原执行状态 `FAULT_INTERLOCK` 和停止行为保持一致，0.25 秒超时阈值未放宽。

| 主题 | data 顺序 |
|---|---|
| `/mobile_manipulator_arm_kinematic_metrics` | time, sigma_min, yoshikawa, condition_number, joint_margin, reference_scale, valid, guard_active |
| `/mobile_manipulator_arm_prediction_metrics` | time, horizon_s, min_sigma, terminal_sigma, min_yoshikawa, terminal_yoshikawa, time_to_min_s, valid, solve_ms |

预测诊断在 MPC `postSolverRun` 中抽样完整 2 秒预测窗口，约每 50 ms 预测时间取样，并强制包含终点；最多每 100 ms 墙钟时间发布一次。它不是连续时间最小值的认证。MRT 接收到的是约 `0.2 s` 的下发策略，不应拿它代替完整预测窗口。`solve_ms` 是同步回调测得的求解段墙钟耗时，不是所有线程或全部通信开销。

`armSingularityGuard` 在任务文件中配置：

```text
armSingularityGuard
{
  activate true
  stopSigma 0.02
  slowSigma 0.08
  resetSigma 0.09
  maxJointSampleStep 0.005
}
```

低于 `slowSigma` 后，参考增量按 `(s-stopSigma)/(slowSigma-stopSigma)` 缩放，最低为正常增量的 5%；达到 `stopSigma` 时锁存故障。每周期检查实测构型，并在发布底盘／机械臂指令前检查实测 q 到限速后的实际关节目标之间的路径。按最大关节步长分段，超过 128 段或任何指标无效时拒绝执行。该抽样检查不保证连续轨迹、伺服滞后或模型误差下的硬件安全。

`safety.singularity_feedback_enabled=true` 位于公共 force-MPC YAML；缺少、过期或无效反馈会阻止参考推进。自定义旧 task 文件需开启指标诊断，或明确关闭该反馈功能。发生 `FAULT_SINGULARITY` 后停止底盘并保持机械臂，导纳参考停止累积，松手不会自动恢复。

恢复要求先处于经确认的安全构型，有新鲜反馈且 `sigma>=resetSigma`；随后按顺序调用：

```bash
ros2 service call /mobile_manipulator/force_control/reset_arm_singularity std_srvs/srv/Trigger '{}'
ros2 service call /whole_body_force_control/reset std_srvs/srv/Trigger '{}'
# 等待 force controller ACTIVE 后：
ros2 service call /mobile_manipulator/force_control/reset_interlock std_srvs/srv/Trigger '{}'
```

复位不会自动规划脱离奇异点，MRT 会等待复位后生成的新策略。不要为了复位而降低阈值并自动重放旧目标。

## 实测对比

场景为 `force_follow_infinite`，包含完整机器人碰撞模型；MuJoCo 查看器及 C++ 键盘施力窗口开启，RViz 为减少图形开销关闭。探针只给 WBMM 键盘窗口发送按键，离开阶段和退出时释放全部按键，不向硬件发布命令。

每组全新启动，按仿真时间执行：零力 3 秒、`f` 施力 8 秒、释放 4 秒、`r` 反向 6 秒、释放 4 秒。`f/r` 对应 TCP `-Z/+Z` 的 10 N；在起始姿态下分别接近世界 `+Z/-Z`。以 `/clock` 定时，避免求解耗时不同造成实际施力时长不同。三组执行拦截均关闭，参考未减速，因此下面衡量的是代价本身的效果。

| 指标 | 新增代价关闭 | 奇异值过程＋终端 | 奇异值＋Yoshikawa 过程＋终端 |
|---|---:|---:|---:|
| 实测最小奇异值 | 0.01090 | 0.04300 | 0.04316 |
| 实测最小操作度 | 0.0001381 | 0.0004583 | 0.0004600 |
| TCP 位置误差 RMS | 35.53 mm | 32.24 mm | 32.24 mm |
| 求解耗时 P95 | 2.66 ms | 28.77 ms | 28.14 ms |
| 意外碰撞／故障 | 无 | 无 | 无 |

奇异值代价使最低值提高约 3.95 倍；叠加操作度后为约 3.96 倍，额外收益很小，因此默认只启用奇异值代价。工作区复用前的相同实验保存在 `before_workspace_reuse/`，奇异值项求解 P95 为 33.68 ms；复用后为 28.77 ms，约降低 14.6%。仍有数值导数开销；本场景策略未过期，但不能据此声称已经达到配置的 100 Hz MPC 更新频率。两轮闭环轨迹受通信与求解时序影响，不能要求逐帧完全相同。

![指标对比](diagnostics/force_mpc_arm_margin/comparison.png)

以下是相同施力历史的 14.8 秒处，从实测关节／底盘状态和 URDF 重建的关节原点几何；不是手绘理想姿态，也不是碰撞模型认证。此时 joint_3 从约 -9.1° 保持为约 -21.5°，最小奇异值从 0.0187 提升到约 0.0431。

![姿态对比](diagnostics/force_mpc_arm_margin/postures.png)

报告、逐帧实测状态和配置保存在 `docs/diagnostics/force_mpc_arm_margin/`。`baseline_stress14.json` 另记录了对齐速度限值之前的长时间基线：14 秒施力阶段尚未完成，约 10.5 秒时触发 `FAULT_TRACKING_ERROR`，最低奇异值约 `1.59e-5`；它是问题复现证据，不参与上述三组公平对比。

保护测试独立进行：`guard_enabled.json` 使用默认阈值并持续施力 14 秒，最低 sigma 为 0.02865、无碰撞及故障；`guard_trip.json` 人为提高 stop 阈值为 0.11、关闭软代价，用于验证执行路径拦截。后者在实测 sigma 仍约 0.1108 时拒绝危险指令，记录了底盘零速度、机械臂保持变化 0、参考位置变化 0，以及松手后故障继续锁存。真实 ROS 服务调用验证了初始安全构型允许复位、裕度不足时拒绝复位。这些是 MuJoCo 闭环证据。复位服务与执行检查共享的 Pinocchio 工作区由 cost 内的互斥锁保护。

`integration_guard.json` 是开启默认奇异性保护后运行已有 C++ 综合探针的结果：正／反向 TCP 位移约 +222.4／-222.6 mm，底盘和机械臂均参与运动，撤力漂移约 0.020 mm；主动关闭虚拟力源触发 `FAULT_WRENCH_TIMEOUT`，底盘指令归零、机械臂保持变化为零，恢复数据不会自行解除联锁。显式复位后的 TCP 位移约 0.152 mm，没有重放旧偏移。MRT 首次原因记录为 `controller:FAULT_WRENCH_TIMEOUT`，验证了新增诊断路径。

最终版本使用默认配置连续完成三次独立的 `w/s/a/d/r/f` 六方向回归，每方向施力 3 秒、释放 2 秒：最低 sigma 分别为 0.09932／0.09935／0.09941，位置误差 RMS 为 11.48／11.49／11.49 mm，三次均无碰撞及故障；安全构型复位服务也均通过。报告为 `six_axes.json`、`six_axes_repeat2.json`、`six_axes_repeat3.json`。优化前首次尝试在第一个方向开始时触发现有 `FAULT_INTERLOCK`，未发生奇异性故障或碰撞；旧联锁记录不足以区分机器人反馈过期、控制器心跳过期或短暂控制器故障，原因无法追溯确认。失败记录保留为 `six_axes_interlock_failure.*`；最终版本新增首次原因和反馈年龄诊断，没有放宽超时阈值，也没有自动重置联锁。三次通过不能作为任意时长／负载下都不触发联锁的认证。

## 复现

先加载 ROS 和工作空间环境，再使用系统 Python 的 `-s`，避免当前用户 NumPy 2.x 与 ROS Pinocchio 的 NumPy 1.x ABI 冲突：

```bash
/usr/bin/python3 -s src/bringup/test/run_force_mpc_arm_margin_ablation.py \
  --directory docs/diagnostics/force_mpc_arm_margin
/usr/bin/python3 -s src/bringup/test/plot_force_mpc_arm_margin.py \
  --directory docs/diagnostics/force_mpc_arm_margin

/usr/bin/python3 -s src/bringup/test/run_force_mpc_arm_margin_ablation.py \
  --directory docs/diagnostics/force_mpc_arm_margin --cases guard_trip \
  --sequence f:8,zero:2 --expect-singularity-fault --check-reset-services

/usr/bin/python3 -s src/bringup/test/run_force_mpc_arm_margin_ablation.py \
  --directory docs/diagnostics/force_mpc_arm_margin --cases integration_guard \
  --integration-probe

/usr/bin/python3 -s src/bringup/test/run_force_mpc_arm_margin_ablation.py \
  --directory docs/diagnostics/force_mpc_arm_margin \
  --cases six_axes,six_axes_repeat2,six_axes_repeat3 \
  --sequence w:3,zero:2,s:3,zero:2,a:3,zero:2,d:3,zero:2,r:3,zero:2,f:3,zero:2 \
  --check-reset-services
```

运行器使用私有 ROS domain、每组新启动的仿真和同一仿真时间序列，并在每组结束时清理它启动的进程。探针从实测状态独立计算 FK／SVD，报告只有在所需施力与 TCP 运动被观察到且所有阶段完成时通过；对比绘图器从原始轨迹再次验证施力、运动以及三组保护关闭且参考没有减速。故障注入模式则明确验收停止、冻结和锁存。

构建通过；本轮运行的 62 项 C++ 测试和 23 项启动／配置检查通过。C++ 覆盖归一化公式及梯度、PSD 近似、无效参数、独立终端注册与继承、机械臂指标不受底盘位置／航向掩盖、两端安全但内部奇异的指令路径、减速及反向积分、工作区复用／并发 clone 一致性和首次联锁原因锁存。此次没有做实机运动、真实接触环境或所有起始构型的认证。

## 使用注意

- 原 force-MPC 启动命令会加载专用 `task_force_mpc.info`；仿真默认开启奇异值过程／终端代价和保护，Yoshikawa 项可按需开启。
- `minSingularRef=0.12` 是期望软裕度，不能理解为运行时下界。执行保护单独采用 `stopSigma=0.02`、`slowSigma=0.08`、`resetSigma=0.09`；更换 TCP 点、任务维数或特征长度后，需要重新标定阈值。
- 发生故障后先解除原因并回到安全构型，再按上述服务顺序复位。低裕度构型的复位会被拒绝；系统不会自动规划脱离完全奇异构型。
- 数值差分仍有开销，本轮启用代价时求解 P95 约 29 ms；125 Hz 的 MRT 执行频率不等于 100 Hz 的 MPC 求解频率。加长预测窗口或叠加其他代价后，应继续观察策略年龄和求解耗时。
- 首次旧联锁原因不能从旧记录中还原；今后如再次触发，查看首次原因主题和日志中的 odom／关节／控制器年龄，保留日志再排查，避免反复复位掩盖问题。
- 实机新增代价和保护默认关闭，本轮只有仿真证据。实机启用前需按实际模型、速度限制和起始构型验证；其他导航入口继续使用自己的任务配置。
