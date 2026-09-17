# whole_body_force_control

通用移动机械臂力控制包，只负责力控算法、ROS 节点和配置。
机器人模型、全身运动学与消息转换已抽到
`src/robotics/wbmm_pinocchio` 与 `src/robotics/wbmm_ros_interfaces`。MuJoCo 闭环验证入口为
`tracer_jaka_bringup/force_control_mujoco_test.launch.py`。

## 节点代码结构

- `src/node.hpp`：`WholeBodyForceControlNode` 类声明、配置和运行状态。
- `src/node.cpp`：模型/控制器装配、定时器、`update()` 控制周期、故障检查与锁存、`main()`。
- `src/node_config.cpp`：参数默认值、legacy / 六轴配置解析、轴掩码和校验。
- `src/node_ros_io.cpp`：订阅回调、观测/wrench 转换与校验、TF、参考轨迹和状态发布。
- `src/force_processor.cpp`：力预处理流水线：自动 tare、TF 坐标变换、滤波、有限性和安全限幅。
- `src/controllers.cpp`：不依赖 ROS 的力控算法；控制器不再做内部滤波。

阅读建议：看控制流程主要看 `node.cpp`；调参数看 `node_config.cpp`；查消息、坐标变换和发布看 `node_ros_io.cpp`。

## 功能边界

- `AdmittanceController`：实现单轴
  `M*x_ddot + D*x_dot + K*x = F_measured - F_desired`。
  - `F_desired=0`：普通外力导纳；
  - `F_desired>0`：恒力误差导纳。
- `ForceFollower`：无阻尼项的准静态跟随
  `x_target=(F_measured-F_desired)/K`，输入假设已由 `ForceProcessor` 滤波。
- `CartesianComplianceController`：六个相互独立的导纳通道，轴顺序固定为
  `[Fx,Fy,Fz,Tx,Ty,Tz] -> [dx,dy,dz,rx,ry,rz]`。
- `wbmm::pinocchio::WholeBodyKinematics`：用完整 6D IK 实现末端平移和转动修正；
  底盘只分担平移在当前航向上的分量，转动修正由机械臂实现。
- `whole_body_force_control_node`：接收力传感器和 OCS2 观测，发布完整9D状态、
  8D输入参考。

## 力处理流水线

所有原始 FTS 先经过 `ForceProcessor`：

```text
raw FTS
  ↓
fail-closed: |F_raw| > hard_force_norm_limit 立即故障保持
  ↓
自动 tare（启动后采样 N 帧）
  ↓
TF: message.frame_id -> sensor_frame -> tcp_frame
  ↓
scale
  ↓
低通滤波
  ↓
finite / per-axis hard limit / max rate 安全检查
  ↓
CartesianComplianceController（名义 TCP 系）
  ↓
whole-body IK + 可达性 anti-windup + 9D reference 限速
  ↓
MPC target
```

控制器内部不再做滤波；它们只接收已经处理好的 wrench。新增的原始力范数检查位于
tare 和滤波之前，因此大于约 20 N 的阶跃不会被滤波平滑掉或 tare 掉。

### 防积分饱和（anti-windup）

导纳控制器在**捕获的名义 TCP 系**中输出修正量，再用名义 TCP 姿态旋转到
`state_frame`；不会使用每周期测量到的实际 TCP 姿态，因此跟踪误差不会反过来
旋转外力方向、形成正反馈或造成参考跳变。

每个周期还会把 whole-body IK 实际能实现的 TCP 修正反馈给导纳控制器：

- 正常可达时，修正量完全由 `M-D-K` 动力学决定；
- 当 IK 受关节限位/工作空间约束无法继续跟随，积分器被收缩到可达值并清零该轴
  速度，避免“隐藏的偏移”持续累加；
- 释放外力后不会因为积分数米偏移而产生后续跳变。

参考输出还会对底盘 x/y 和所有关节做逐周期限速（`max_base_velocity` /
`max_joint_velocity`），即使 IK 在奇异位形切换分支，MPC 收到的目标也是连续
状态。

## 控制模式

| `control_mode` | 目标力 | 动力学 |
|---|---:|---|
| `admittance` | 强制按 0 N | 二阶 `M-D-K` 外力导纳 |
| `constant_force` | `desired_force` | 二阶恒力误差导纳 |
| `force_follow` | 默认 0 N | 无D准静态力—位移跟随 |

恒力模式中输出符号由 `F_measured-F_desired` 决定；通过
`response_body_x/y/z` 配置正位移方向。实际接触任务仍必须配置碰撞、接触方向、
最大位移和传感器超时保护。

## wbmm_core 接入

自 PR1 起，本包的 ROS 边界统一经过 `wbmm_core`：

- `MpcObservation` -> `wbmm::core::WholeBodyState`；
- `WrenchStamped` -> `wbmm::core::Wrench`；
- 发布前先构造并校验 `wbmm::core::WholeBodyTrajectory`，
  再转换为 `MpcTargetTrajectories`；
- 转换后先调用 `wbmm::core::validate(...)`，再调用
  `wbmm::pinocchio::PinocchioRobotModel::validate(...)`（关节名称、顺序、限位）；校验失败按
  fail-closed 丢弃该帧，并打印带 `RobotModel` 原因的节流日志。

新增参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `state_frame` | `odom` | OCS2 9D state 的参考坐标系；只用于 core 轨迹的 frame 合同与日志，不改变数值语义。 |

`WholeBodyKinematics` 不再直接解析 URDF，而是通过
`wbmm::core::RobotModel`（当前实现 `PinocchioRobotModel`）获取 FK、Jacobian
和关节限位；旧的 `WholeBodyKinematics(urdf_file, ee_frame)` 构造函数仍保留，
供既有调用方平滑迁移。

## 六轴选择

显式设置 `admittance_axes` 后进入 6D 模式。可选轴名为
`fx, fy, fz, tx, ty, tz`：

```yaml
admittance_axes: [fx, fy, fz, tx, ty, tz]
constant_force_axes: [fz]
desired_wrench: [0.0, 0.0, 12.0, 0.0, 0.0, 0.0]
absolute_wrench_axes: [fz]
```

- `admittance_axes`：允许产生柔顺位移/转角的轴；未选择的轴输出严格为零。
- `constant_force_axes`：启用恒力目标的轴，必须是 `admittance_axes` 的子集。
- 未列入 `constant_force_axes` 的导纳轴使用零目标，即普通被动导纳。
- 力矩目标 `Tx/Ty/Tz` 默认均为 `0 N·m`，而且力矩恒力轴默认不开启。
- `desired_wrench`、所有 `*_6d` 参数都按
  `[Fx,Fy,Fz,Tx,Ty,Tz]` 顺序排列，且必须恰好包含六项。

完整示例见 [config/six_axis_example.yaml](config/six_axis_example.yaml)。例如：

```bash
ros2 launch tracer_jaka_bringup force_control_mujoco_test.launch.py \
  run_test:=false control_mode:=constant_force \
  controller_params_file:=$(ros2 pkg prefix whole_body_force_control)/share/whole_body_force_control/config/six_axis_example.yaml
```

6D 模式把 `WrenchStamped` 从消息的 `frame_id` 变换到 `ee_frame`，同时包含
力矩的力臂项；随后在名义末端局部系产生 `[dx,dy,dz,rx,ry,rz]`。
`require_wrench_frame` 默认在 6D 模式开启，没有 `frame_id` 或 TF 不可用时会拒绝
该帧数据。还没有收到有效 wrench 时节点保持观测位置不动；已经收到过数据后，
力传感器或 OCS2 观测超时会锁存 `FAULT_WRENCH_TIMEOUT` / `FAULT_OBSERVATION_TIMEOUT`，
关闭导纳控制并发布“保持当前观测”的参考；原始力范数超过
`hard_force_norm_limit` 时立即锁存 `FAULT_WRENCH_LIMIT` 并保持。故障排除后
需重启节点或上层重新启动；当前版本不提供动态 enable 服务。

## MuJoCo自动测试

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tracer_jaka_bringup force_control_mujoco_test.launch.py viewer:=true
```

默认验证 `force_follow`，依次注入 `0 -> +5 -> +12 -> 0 -> -8 -> 0 N`，并检查
底盘和机械臂的双向实际运动、卸载回零、底盘侧滑、控制故障和 MuJoCo 意外碰撞。报告写入
`/tmp/whole_body_force_control_test_report.json`。

## 20 秒持续力跟随

`force_follow_20s_sim.yaml` 是原先的 5 m 开阔直线有限长行程配置；持续施力时
会在约 5.2 m 位移处停止，因此适合“有限长持续运动”回归。

```bash
ros2 launch tracer_jaka_bringup force_control_20s_follow_test.launch.py
```

## 无限力跟随仿真（默认）

`whole_body_force_control_sim.launch.py` 现已默认使用“无限力跟随”配置：

- 配置：`config/force_follow_infinite_sim.yaml`
- 场景：`models/scene_force_follow_infinite.xml`

该入口直接并列启动 MuJoCo bridge、robot_state_publisher、OCS2 MPC/MRT、
whole-body force control 和可选 RViz，不再包含通用的 `ocs2_sim.launch.py`，
因此也没有 SLAM、REMANI 或 REMANI bridge 开关。对外仅保留三个参数：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `viewer` | `true` | 是否打开 MuJoCo 原生窗口 |
| `use_rviz` | `true` | 是否打开力控专用 RViz |
| `profile` | `infinite` | 实验档位：`infinite` 或 `20s` |

```bash
ros2 launch tracer_jaka_bringup whole_body_force_control_sim.launch.py \
  viewer:=true use_rviz:=true profile:=infinite
```

仿真/实机不做成隐藏的布尔参数：本入口始终只启动仿真，实机继续使用独立的
`whole_body_force_control_real.launch.py`。

原理：

- 与有限弹性位移不同，不再在 `force / stiffness` 或小 `max_offset` 处停住；
- 只要持续施加有符号的 `tool0/Fx`，参考位移就按 `max_velocity` 持续爬升；
- 因此机器人会一直沿着力的方向移动，直到撤力或人为停止。

自动验证：

```bash
ros2 launch tracer_jaka_bringup force_control_infinite_follow_test.launch.py
```

该入口在超大开阔 MuJoCo 地面场景中连续施加 30 秒 `tool0/Fx=7 N`，并把 30 秒
分成多个窗口检查底盘和机械臂是否持续前进。报告写入
`/tmp/whole_body_force_control_infinite_report.json`。

> 该配置为仿真验证使用：把 `max_offset` / `max_base_delta` 设置为很大值以表达
> “无有限位移终点”，不代表实机可以取消安全限位。实机仍需使用独立的安全配置。

## 手动跟随

```bash
ros2 launch tracer_jaka_bringup force_control_mujoco_test.launch.py \
  viewer:=true run_test:=false
```

在另一个终端持续发布后，等待日志出现
`Captured nominal state; accepting legacy scalar wrench`：

```bash
ros2 topic pub -r 50 /whole_body_force_control/fake_wrench \
  geometry_msgs/msg/WrenchStamped \
  "{header: {frame_id: tool0}, wrench: {force: {x: 7.0}}}"
```

将 `x: 7.0` 改为负数即可验证反向拉动。该首轮配置是有符号的单轴
`force_follow`：工具坐标系 `Fx` 的符号决定沿底盘当前朝向的正/反运动，底盘与
机械臂按 `base_share` 分担位移；它不是接触恒力控制，也不是二阶导纳。

## 实机无限力跟随

> 实机验证前请先完成 `docs/whole_body_force_control_real_deployment.md` 的阶段 A/B/C，
> 不要跳过只读接线、方向标定和影子计算。

若要在开阔实机环境中验证“持续受力持续跟随”，可显式使用实机无限力跟随配置：

```bash
ros2 launch tracer_jaka_bringup whole_body_force_control_real.launch.py   hardware_write:=true   force_reference_output_enabled:=true   force_control_armed:=false   force_params_file:=$(ros2 pkg prefix whole_body_force_control)/share/whole_body_force_control/config/force_follow_infinite_real.yaml
```

当前版本不提供 `/whole_body_force_control/enable` 服务，`armed` 由启动参数控制；
需要在故障后重新 armed 时，重启该节点或由上层重新启动。

该配置启用了 `force_velocity_mode: true`：推/拉力超过 `force_deadband` 时，
机器人以 `max_velocity` 持续沿力的方向移动；撤力后停止在当前位姿，不回弹。
默认实机无限配置的移动速度很低（`0.02 m/s`），但仍必须保证场地足够空旷并随时可急停。

## 实机部署

实机入口只启动力控相关节点：

```bash
ros2 launch tracer_jaka_bringup whole_body_force_control_real.launch.py
```

默认是只读、无底盘命令、无 OCS2 目标输出、未 armed 的观察模式。完整的分阶段
检查、运动放行命令、停止方法和验收标准见
[`docs/whole_body_force_control_real_deployment.md`](../../../docs/whole_body_force_control_real_deployment.md)。

运行时状态：

- `/whole_body_force_control/control_state`：`DISABLED`、`SETTLING`、`ACTIVE`
  或 `FAULT_*`；
- `/whole_body_force_control/status`：控制量与底盘/末端实际跟踪量；
- `/mobile_manipulator_mpc_target`：唯一的 OCS2 参考接口。

## 恒力模式

```bash
ros2 launch tracer_jaka_bringup force_control_mujoco_test.launch.py \
  viewer:=true run_test:=false control_mode:=constant_force desired_force:=12.0
```

这个无障碍测试场景没有接触面，只用于检查接口和参考方向；恒力闭环的物理验证
必须换成带接触体的 MuJoCo 场景，并把导纳轴、恒力轴、坐标系和限幅按该场景配置。

## 主要参数

- `topics.wrench`, `topics.status`, `topics.control_state`
- `force_sensor.sensor_frame`, `force_sensor.tcp_frame`,
  `force_sensor.require_wrench_frame`, `force_sensor.tf_lookup_timeout`,
  `force_sensor.tf_fallback_to_latest`
- `force_sensor.tare_samples`, `force_sensor.filter_alpha`,
  `force_sensor.wrench_scale`, `force_sensor.force_timeout`
- `force_sensor.hard_force_norm_limit`：原始 `Fx/Fy/Fz` 范数硬限幅，默认 20 N；
  设为 0 可关闭该单独检查
- `force_sensor.hard_wrench_limit`、`force_sensor.max_wrench_rate`
- `admittance.enable`, `admittance.output`, `admittance.selected_axes`,
  `admittance.mass`, `admittance.damping`, `admittance.stiffness`,
  `admittance.max_offset`, `admittance.max_velocity`
- `whole_body.base_share`, `whole_body.max_base_delta`,
  `whole_body.max_base_velocity`, `whole_body.max_joint_delta`,
  `whole_body.max_joint_velocity`
- `safety.observation_timeout`, `safety.capture_settle_time`,
  `safety.enforce_single_target_owner`, `safety.loop_rate`
- `output.reference_horizon`, `output.reference_dt`, `output.input_dimension`

`/whole_body_force_control/status` 的前 9 项保持旧接口，其后依次为 TCP 系 `filtered_wrench[6]`、名义 TCP 系 `offset[6]`、
`velocity[6]`、导纳轴掩码
`[6]`，末尾为 `admittance_enable`、`reference_output_enabled`、
`fault_latched` 三项。
