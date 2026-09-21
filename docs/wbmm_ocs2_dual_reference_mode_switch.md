# WBMM OCS2 双参考与 TaskPhase 模式切换

> Status: DRAFT
>
> Author: Agent
>
> Reviewer: TBD
>
> Reviewed at: TBD
>
> Review Level: L1
>
> 代码基线：2026-09-21 工作区 `wbmm_ocs2` / `wbmm_ocs2_ros` 双参考模式接口实现
>
> Warning: 本文档尚未经过人工完整审阅，不能单独作为实机执行或安全放行依据。

## 1. 文档目的

本文档记录 WBMM 当前的双参考 / TaskPhase 模式切换实现，覆盖：

- OCS2 问题层如何同时保存 9D 全身参考和 7D 末端参考；
- 外部如何通过 ROS service 切换 TaskPhase；
- 每个 phase 下两路参考的代价权重；
- MPC、MRT、REMANI bridge、目标 marker、力控之间的接口；
- reset、policy mode 检查和切换安全行为；
- 当前已验证内容与仍未验证的边界。

本文档取代旧文档中“单一 `_mpc_target` 同时承载 7D / 9D”以及“用 `task_phase` 话题直接切换”的旧设计描述。旧设计的过程与算法背景仍可参考
[wbmm_ocs2_mode_manipulability_esdf_design.md](wbmm_ocs2_mode_manipulability_esdf_design.md)，但凡与该文档冲突之处，以本文档记录的 CURRENT 为准。

---

## 2. CURRENT：状态、参考和 phase 定义

### 2.1 状态与输入

沿用 [math_contract.md](math_contract.md)：

<!-- prettier-ignore -->
$$
x = [x_b, y_b, \psi_b, q_1, q_2, q_3, q_4, q_5, q_6]^T \in \mathbb{R}^9
$$

$$
u = [v, \omega, \dot{q}_1, \dot{q}_2, \dot{q}_3, \dot{q}_4, \dot{q}_5, \dot{q}_6]^T \in \mathbb{R}^8
$$

- `whole_body` 参考：9D，直接对应上式的完整状态；
- `end_effector` 参考：7D，对应 `[x, y, z, qx, qy, qz, qw]`；
- 两路参考在同一个 `WbmmReferenceManager` 中同时维护，互不覆盖。

### 2.2 TaskPhase

`wbmm_ocs2::TaskPhase` 是 MPC 代价权重的阶段标识：

| 数值 | 名称 | 任务语义 | 默认主导参考 |
|---:|---|---|---|
| 0 | `kNavigation` | 导航 / 全身轨迹跟踪 | 9D whole-body |
| 1 | `kTransition` | 执行前过渡 | whole-body + EE 同时加权 |
| 2 | `kExecution` | 末端执行 / 接触任务 | 7D end-effector |
| 3 | `kRetract` | 回撤 / 切回全身 | 9D whole-body |

TaskPhase 同时作为 OCS2 `ModeSchedule` 的 mode 载体，使 MRT 可以通过
`MRT_ROS_Interface::evaluatePolicy()` 返回值检查“当前正在执行的 policy 是否属于请求的 phase”。

> 注意：WBMM 当前动力学与约束不依赖 mode，TaskPhase 只用于代价权重和运行安全校验，不是混合动力学切换。

### 2.3 两路参考维度的含义

| 参考 | 维度 | 字段语义 | 典型发布者 |
|---|---:|---|---|
| whole-body target | 9D | `[x, y, yaw, q1..q6]` | REMANI bridge / 力控 legacy whole-body 输出 |
| end-effector target | 7D | `[x, y, z, qx, qy, qz, qw]` | interactive marker / 力控 EE 输出 / 任务或视觉节点 |

---

## 3. CURRENT：OCS2 核心层实现

### 3.1 `WbmmReferenceManager`

文件：

- `src/control/wbmm_ocs2/include/wbmm_ocs2/WbmmReferenceManager.h`
- `src/control/wbmm_ocs2/src/WbmmReferenceManager.cpp`

职责：

1. 同时维护两个 `ocs2::TargetTrajectories` buffer；
2. 通过 `BufferedValue` 在 `preSolverRun()` 时统一锁存；
3. 同时维护 active phase 和 requested phase；
4. 对外部单目标接口按状态维度路由：

```text
9D target  -> wholeBodyTarget_
7D target  -> endEffectorTarget_
无法判断   -> requested phase 路由
```

必须同时重写 const 和 rvalue 两个虚函数：

```cpp
void setTargetTrajectories(const ocs2::TargetTrajectories&) override;
void setTargetTrajectories(ocs2::TargetTrajectories&&) override;
```

原因：`MPC_ROS_Interface::resetMpcNode()` 和旧 `RosReferenceManager` 使用 `std::move()` 调用
rvalue 版本；如果漏掉该重载，target 会落到基类的单 buffer，双参考实际读到空 reference。

### 3.2 `WbmmInterface`

文件：

- `src/control/wbmm_ocs2/include/wbmm_ocs2/WbmmInterface.h`
- `src/control/wbmm_ocs2/src/WbmmInterface.cpp`

当 `task.info` 中 `modeSwitch.activate=true` 时：

- 同时注册 `wholeBodyTracking` 和 `endEffectorTracking` 的 stage cost；
- 同时注册二者的 final cost；
- 不再注册旧的 `EndEffectorConstraint` 软约束；
- 两个 cost 均由 `PhaseWeightedStateCost` 按 phase 加权。

生产配置默认：

```ini
modeSwitch
{
  activate     true
  initialPhase 0
}
```

因此默认启动行为与原来的 Navigation / whole-body 跟踪一致，同时运行中可通过 service 切换。

### 3.3 `PhaseWeightedStateCost`

文件：

- `src/control/wbmm_ocs2/include/wbmm_ocs2/cost/PhaseWeightedStateCost.h`
- `src/control/wbmm_ocs2/src/PhaseWeightedStateCost.cpp`

形式：

<!-- prettier-ignore -->
$$
L(x, t, s) = w_{wb}(s) L_{wb}(x, t) + w_{ee}(s) L_{ee}(x, t) + L_{reg}(x, u)
$$

其中：

- `s` 为锁存后的 active TaskPhase；
- `w=0` 的项通过 `isActive()` 直接退出本项优化；
- 不同 phase 只改变权重，不动态增删 OCS2 problem 中的 term，避免在线修改 OCP 结构。

当前生产 `task.info` 配置：

| phase | `w_wb` | `w_ee` | 说明 |
|---|---:|---:|---|
| Navigation | 1.0 | 0.0 | whole-body 跟踪，行为接近原有模式 |
| Transition | 0.5 | 0.5 | 两路参考同时生效 |
| Execution | 0.0 | 1.0 | EE 主导；whole-body 不把机器人拉回旧参考 |
| Retract | 0.5 | 0.0 | 回撤到 whole-body 主导 |

### 3.4 Execution 阶段为什么 whole-body 权重为 0.0

当前 `WholeBodyTrajectoryCost` 跟踪完整 9D 状态，还没有拆成
`BaseTrackingCost + ArmPostureCost`。如果 Execution 阶段保留非零 whole-body 权重：

- Remani 导航参考结束时，最后一个 whole-body waypoint 会形成“回拉”；
- 机器人到达执行位姿后，底盘和机械臂会被拉回参考末端状态；
- EE 任务与 whole-body 正则可能直接冲突。

因此当前版本在 Execution 阶段使用 `w_wb = 0.0`。后续若实现专门的
底盘位置/航向保持 + 臂构型正则，再单独引入非零执行阶段全身正则。

---

## 4. CURRENT：ROS 接口契约

### 4.1 接口总表

| 接口 | 类型 | 方向 | 维度 / 值 | QoS | 说明 |
|---|---|---|---:|---|---|
| `/mobile_manipulator_whole_body_target` | `ocs2_msgs/msg/MpcTargetTrajectories` | 发布者 → MPC | 9D | reliable, keep last 1 | 全身参考 |
| `/mobile_manipulator_ee_target` | `ocs2_msgs/msg/MpcTargetTrajectories` | 发布者 → MPC | 7D | reliable, keep last 1 | 末端参考 |
| `/mobile_manipulator_set_task_phase` | `wbmm_ocs2_ros/srv/SetTaskPhase` | 外部 → MPC | request `int32 phase` | 标准 service | 切换 TaskPhase |
| `/mobile_manipulator_task_phase_state` | `wbmm_ocs2_ros/msg/TaskPhaseState` | MPC → MRT/外部 | requested/active | reliable + transient_local | latched phase 状态 |

不再保留：

```text
/mobile_manipulator_mpc_target
```

该话题不再由新接口订阅或发布。所有参考统一走两路 target topic。

### 4.2 `SetTaskPhase.srv`

```text
int32 phase
---
bool success
string message
int32 requested_phase
int32 active_phase
```

- `phase` 只允许 `0..3`；
- `success=true` 表示请求已被 ReferenceManager 接收；
- `active_phase` 在下一次 MPC solve 的 `preSolverRun()` 后才更新，因此成功响应中仍可能是旧值；
- `requested_phase` 是立即生效的期望值，MRT 使用它做 policy mode 一致性检查。

### 4.3 `TaskPhaseState.msg`

```text
std_msgs/Header header
int32 requested_phase
int32 active_phase
bool mode_switch_enabled
```

- `requested_phase`：最近一次 service 请求的 phase，也是 MRT 的安全目标 phase；
- `active_phase`：最近一次 `preSolverRun()` 锁存后、供 `PhaseWeightedStateCost` 使用的 phase；
- `mode_switch_enabled`：对应 `task.info` 的 `modeSwitch.activate`；
- 该状态话题由 MPC 节点周期发布，并使用 `transient_local`，保证后启动的 MRT 也能收到最新状态。

### 4.4 Topic / service 名称参数

`src/bringup/config/common/ocs2.yaml` 中可覆盖：

```yaml
wbmm_mpc_node:
  ros__parameters:
    robot_name: mobile_manipulator
    whole_body_target_topic: mobile_manipulator_whole_body_target
    ee_target_topic: mobile_manipulator_ee_target
    task_phase_state_topic: mobile_manipulator_task_phase_state
    set_task_phase_service: mobile_manipulator_set_task_phase
    initial_task_phase: -1

wbmm_mrt_node:
  ros__parameters:
    robot_name: mobile_manipulator
    task_phase_state_topic: mobile_manipulator_task_phase_state
    initial_task_phase: -1
```

`initial_task_phase=-1` 表示使用 `task.info` 的 `modeSwitch.initialPhase`。

---

## 5. CURRENT：数据流

```text
REMANI / whole-body planner
        |
        | 9D
        v
/mobile_manipulator_whole_body_target ----+
                                          |
fanuc marker / vision / force control     |
        |                                 |
        | 7D                              |
        v                                 v
/mobile_manipulator_ee_target -------> WbmmReferenceManager
                                          |
external task state machine               |  phase service
        |                                 v
        +-----------------------> /mobile_manipulator_set_task_phase
                                          |
                                          v
                                 active / requested phase
                                          |
                                          v
                              PhaseWeightedStateCost
                                          |
                  +-----------------------+-----------------------+
                  |                                               |
         w_wb(phase) * WholeBodyTracking         w_ee(phase) * EndEffectorTracking
                  |                                               |
                  +-----------------------+-----------------------+
                                          |
                                   OCS2 OptimalControlProblem
                                          |
                                          v
                             MPC policy (mode = TaskPhase)
                                          |
                                          v
                        WbmmMrtNode phase / policy mode check
```

always-on terms 仍包括：

- input cost；
- joint position / velocity limits；
- self collision；
- body-relative / environment collision（按 task 配置）；
- 其他已注册的约束和代价。

---

## 6. CURRENT：ROS 节点行为

### 6.1 `WbmmMpcNode`

文件：`src/control/wbmm_ocs2_ros/src/WbmmMpcNode.cpp`

- 不再使用 `RosReferenceManager`；
- solver 直接使用 `WbmmInterface` 内部的 `WbmmReferenceManager`；
- 订阅 9D / 7D 两个 target topic；
- 校验 target：
  - state 维度必须一致；
  - 所有值有限；
  - time 与 state 数量一致；
  - 多点时 time 严格递增；
  - 不符合要求时 drop 并 warning，不杀节点；
- `modeSwitch.activate=true` 时创建：
  - `SetTaskPhase` service；
  - `TaskPhaseState` latched publisher；
  - service 请求合法时调用 `WbmmInterface::setTaskPhase()`；
- `modeSwitch.activate=false` 时：
  - 不创建 phase service / state topic；
  - 仍订阅两路 target；内部 OCP 走旧的单 cost / constraint 分支。

### 6.2 `WbmmMrtNode`

文件：`src/control/wbmm_ocs2_ros/src/WbmmMrtNode.cpp`

- `modeSwitch.activate=true` 时订阅 `TaskPhaseState`；
- `currentPhase_` 使用 requested phase；
- `resetMpc()`：
  - phase 为 `Execution` → TF 查询当前 EE pose，发送 7D reset target；
  - 其余 phase → 使用当前 observation state，发送 9D reset target；
- 每个控制周期读取 policy：
  - `evaluatePolicy()` 返回 policy mode；
  - 如果 policy mode 与 requested phase 不一致：
    - 停止底盘输出；
    - 保持机械臂；
    - 直到新 policy 到达并且 mode 一致；
- `modeSwitch.activate=false` 时保留旧 `use_whole_body_target` 逻辑。

### 6.3 `WbmmTargetNode`

文件：`src/control/wbmm_ocs2_ros/src/WbmmTargetNode.cpp`

- 右键菜单发送 7D target；
- 发布到 `mobile_manipulator_ee_target`；
- 不再发布 `mobile_manipulator_mpc_target`；
- `input_dim=8` 只用于 `TargetTrajectories.inputTrajectory` 的零输入占位；
- 交互 marker 只负责给 EE 参考，不负责切换 phase，也不直接发布 phase state。

### 6.4 `remani_to_ocs2_reference_bridge`

文件：`src/control/wbmm_ocs2_ros/src/remani_to_ocs2_reference_bridge.cpp`

- 已从旧 `TargetTrajectoriesRosPublisher` 切换为显式 topic publisher；
- 发布 9D rolling reference 到 `mobile_manipulator_whole_body_target`；
- `reference_owner_service` 仍控制它是否持有 whole-body 参考发布权；
- `robot_name` 默认仍为 `mobile_manipulator`，默认 topic 为：

```text
robot_name + "_whole_body_target"
```

### 6.5 `whole_body_force_control`

文件：

- `src/control/whole_body_force_control/src/node_config.cpp`
- `src/bringup/config/common/force_control.yaml`

- EE 输出模式继续发布 7D 到 `mobile_manipulator_ee_target`；
- legacy whole-body 输出从 `_mpc_target` 改为 `mobile_manipulator_whole_body_target`；
- 力控 profile launch 可直接使用 `initial_task_phase: 2` 进入 Execution；
- 通用实机入口默认 `initial_task_phase: -1`，需要通过 service 或显式 launch 参数切到 Execution。

---

## 7. CURRENT：phase 切换时序

```text
1. 外部任务状态机调用
   /mobile_manipulator_set_task_phase { phase: 1 }

2. WbmmMpcNode service 回调:
   - WbmmReferenceManager::setTaskPhase(phase)
   - requested_phase 立即更新
   - active_phase 仍在旧值
   - 通过 TaskPhaseState requested_phase 发布新值

3. WbmmMrtNode 收到 TaskPhaseState:
   - currentPhase_ 更新为新 requested phase
   - 下一次控制周期检查 policy mode
   - 若 policy mode 仍为旧 phase，则 stopAndHold()

4. 下一次 MPC solve:
   - WbmmReferenceManager::preSolverRun()
   - active_phase、whole-body target、EE target 一起锁存
   - PhaseWeightedStateCost 使用新权重
   - policy 的 ModeSchedule = 新 phase

5. WbmmMrtNode 收到新 policy:
   - policy mode 与 currentPhase_ 一致
   - 恢复正常底盘和机械臂输出
```

建议上层切换顺序：

```text
Navigation
   -> 确保 whole-body target 持续有效
   -> 确保 EE hold target 已发布
Transition
   -> 等待至少一个 MPC horizon 完成权重过渡
Execution
   -> 持续更新 EE target
   -> 根据需要由力控/视觉接管 EE correction
Retract
   -> 释放 EE 主导，whole-body 参考重新有效
```

---

## 8. CURRENT：配置项

### 8.1 `task.info` 中的 mode switch

```ini
modeSwitch
{
  activate       true
  initialPhase   0
}
```

### 8.2 `wholeBodyTracking` 权重

```ini
wholeBodyTracking
{
  activate true

  phaseWeights
  {
    (0,0)  1.0
    (1,0)  0.5
    (2,0)  0.0
    (3,0)  0.5
  }

  finalWeightScale 1.0

  Q { ... }
}
```

### 8.3 `endEffectorTracking` 权重

```ini
endEffectorTracking
{
  activate true

  phaseWeights
  {
    (0,0)  0.0
    (1,0)  0.5
    (2,0)  1.0
    (3,0)  0.0
  }

  Q
  {
    position
    {
      (0,0) 100.0
      (1,1) 100.0
      (2,2) 100.0
    }
    orientation
    {
      (0,0) 25.0
      (1,1) 25.0
      (2,2) 25.0
    }
  }
}
```

### 8.4 launch 参数

`ocs2.launch.py` 新增：

```text
initial_task_phase
  -1 = 使用 task.info 的 modeSwitch.initialPhase
   0 = Navigation
   1 = Transition
   2 = Execution
   3 = Retract
```

该参数会同时传给 MPC 和 MRT，保证启动时两者 phase 一致。

---

## 9. CURRENT：使用方式

### 9.1 编译

```bash
colcon build --packages-select \
  wbmm_ocs2 wbmm_ocs2_ros whole_body_force_control tracer_jaka_bringup
```

### 9.2 查看接口

```bash
ros2 interface show wbmm_ocs2_ros/msg/TaskPhaseState
ros2 interface show wbmm_ocs2_ros/srv/SetTaskPhase
```

### 9.3 切换 phase

```bash
ros2 service call /mobile_manipulator_set_task_phase \
  wbmm_ocs2_ros/srv/SetTaskPhase "{phase: 2}"
```

辅助脚本：

```bash
ros2 run wbmm_ocs2_ros set_task_phase.py 2
```

### 9.4 查看 phase 状态

```bash
ros2 topic echo /mobile_manipulator_task_phase_state --once
```

### 9.5 实机力控进入 Execution

方式一：显式启动 phase：

```bash
ros2 launch tracer_jaka_bringup whole_body_force_control.launch.py \
  initial_task_phase:=2
```

方式二：启动后由外部任务状态机调用 phase service：

```bash
ros2 run wbmm_ocs2_ros set_task_phase.py 2
```

### 9.6 仿真力控 profile

`whole_body_force_control_profiles.launch.py` 当前对 MPC/MRT 直接设置：

```text
initial_task_phase = 2
```

因此仿真力控 profile 启动后即处于 Execution，EE target 权重为 1.0。

---

## 10. CURRENT：验证记录

本文档对应的代码已完成以下验证：

| 验证 | 结果 | 说明 |
|---|---|---|
| 单包编译 | 通过 | `wbmm_ocs2`、`wbmm_ocs2_ros`、`whole_body_force_control`、`tracer_jaka_bringup` |
| `wbmm_ocs2` gtest | 15/15 通过 | 包含 `WbmmReferenceManager.RvalueSetRoutesByStateDimension` |
| `whole_body_force_control` gtest | 24/24 通过 | 力控核心与 kinematics 测试 |
| MPC 节点启动 | 通过 | real / sim `task.info` 均可进入 dual-reference 分支 |
| service + state topic | 通过 | service 切换 phase，latched state 可读 |
| MPC + MRT 假 odom/joint_states | 通过 | reset、first policy、100 Hz MRT loop |
| 运行中 0 → 1 phase 切换 | 通过 | MRT 先 hold，新 policy mode 更新后恢复 |

未验证：

- 真实机器人、真实 FTS、真实执行器上的 phase 切换；
- Execution 阶段真实 EE 接触任务与力控 correction 的闭环；
- 高速切换、掉线、超时、多发布者冲突；
- 急停 / 看门狗 / SAFE_HOLD / FAULT 与 phase 的联合行为。

---

## 11. CURRENT：文件索引

| 文件 | 作用 |
|---|---|
| `src/control/wbmm_ocs2/include/wbmm_ocs2/WbmmReferenceManager.h` | 双参考 + phase 状态 |
| `src/control/wbmm_ocs2/src/WbmmReferenceManager.cpp` | 维度路由、rvalue 重写、锁存 |
| `src/control/wbmm_ocs2/include/wbmm_ocs2/cost/PhaseWeightedStateCost.h` | 阶段权重 wrapper |
| `src/control/wbmm_ocs2/src/WbmmInterface.cpp` | modeSwitch 分支注册双 cost |
| `src/control/wbmm_ocs2_ros/msg/TaskPhaseState.msg` | phase 状态消息 |
| `src/control/wbmm_ocs2_ros/srv/SetTaskPhase.srv` | phase 切换服务 |
| `src/control/wbmm_ocs2_ros/src/WbmmMpcNode.cpp` | 双 target 订阅、service、state |
| `src/control/wbmm_ocs2_ros/src/WbmmMrtNode.cpp` | phase 订阅、reset、policy mode hold |
| `src/control/wbmm_ocs2_ros/src/WbmmTargetNode.cpp` | 7D EE target 发布 |
| `src/control/wbmm_ocs2_ros/src/remani_to_ocs2_reference_bridge.cpp` | 9D whole-body target 发布 |
| `src/control/whole_body_force_control/src/node_config.cpp` | 力控 target topic 配置 |
| `src/bringup/config/common/ocs2.yaml` | ROS topic / service / initial phase 参数 |
| `src/bringup/config/{real,sim}/task.info` | modeSwitch + phase weights |
| `src/bringup/launch/ocs2.launch.py` | `initial_task_phase` 参数传递 |

---

## 12. TBD / 待确认

- `TBD`：是否保留 `modeSwitch.activate=false` 兼容路径的正式支持周期；
- `TBD`：Execution 阶段是否需要专门的底盘/姿态正则，而不是当前 `w_wb=0.0`；
- `TBD`：是否将 `TaskPhase` 与力控内部的 `ExecutionPhase` 统一为单一枚举；
- `TBD`：是否增加 phase 切换频率限制，防止上层状态机抖动导致 MRT 反复 hold；
- `TBD`：真实硬件上 TF 不可用、EE target 超时、phase service 重复调用时的降级策略；
- `TBD`：是否需要在 MPC policy 中显式发布“当前 active phase”，而不是由 state topic 周期发布。

以上 TBD 项均需要人工确认后，才能作为实机安全或执行放行依据。
