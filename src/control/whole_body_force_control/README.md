# whole_body_force_control

通用移动机械臂力控包，负责力传感器预处理、TCP 系导纳修正和任务空间/全身 reference 生成。默认输出 7D 末端位姿给 OCS2，由 OCS2 决定底盘与机械臂分工；旧 9D base_share + IK 路线保留为实验基线。

机器人模型、全身运动学与 ROS 消息转换由：

- `src/robotics/wbmm_pinocchio`
- `src/robotics/wbmm_ros_interfaces`

提供。

## 节点结构

- `src/node.hpp`：节点、参数和运行状态声明。
- `src/node.cpp`：模型/控制器装配、控制周期、故障保持、主流程。
- `src/node_config.cpp`：参数声明、校验和默认值。
- `src/node_ros_io.cpp`：订阅回调、TF 变换、reference/修正量发布。
- `src/force_processor.cpp`：tare、滤波、硬限幅和有限性检查。
- `src/controllers.cpp`：单轴导纳和六轴笛卡尔导纳。

## 控制链路

```text
raw FTS in sensor_frame
  -> finite / raw hard force norm / raw per-axis hard limit
  -> auto tare
  -> wrench_scale in sensor_frame
  -> TF: tcp_frame <- sensor_frame
     （完整 wrench 变换，包含力臂力矩）
  -> 一阶低通滤波，tcp_frame
  -> finite / per-axis hard limit
  -> CartesianComplianceController
     （名义 TCP 系，6 轴独立导纳）
  -> whole-body IK 可达性 anti-windup
  -> state_frame 参考
  -> 底盘/关节 reference 速度平滑
  -> MPC target
```

导纳方程：

```text
M * x_ddot + D * x_dot + K * x = F_measured
```

- `K > 0`：弹性导纳，撤力后回到名义位置。
- `K = 0`：纯力跟随，持续力产生持续位移，撤力后位移保持。

## 坐标系

- `force_sensor.sensor_frame`：原始 FTS frame，例如 `jk_se_vi_200_link`。
- `force_sensor.tcp_frame`：导纳控制使用的工具 TCP frame，例如 `tool0`。
- `state_frame`：OCS2/MPC 状态 frame，例如 `odom`。

力先从 sensor frame 完整变换到 TCP frame，再在 TCP 系做导纳。  
修正量最后转换到 `state_frame`，供 MPC 使用。

## 六个柔顺轴

```yaml
admittance:
  selected_axes: [true, true, true, false, false, false]
```

顺序固定：

```text
[Fx, Fy, Fz, Tx, Ty, Tz]
```

未选中轴输出严格为零。

## 主要参数

### topics

```yaml
topics:
  wrench: /fts_broadcaster/wrench
  correction: /whole_body_force_control/correction
  states: /whole_body_force_control/states
```

- `wrench`：力传感器输入。
- `correction`：导纳修正量、wrench、速度等数组。
- `states`：控制状态字符串，例如 `TARING`、`SETTLING`、`ACTIVE`、`FAULT_*`。

### force_sensor

```yaml
force_sensor:
  sensor_frame: jk_se_vi_200_link
  tcp_frame: tool0
  tf_lookup_timeout: 0.05
  tf_fallback_to_latest: true
  tare_samples: 50
  filter_alpha: 0.25
  wrench_scale: [1, 1, 1, 1, 1, 1]
  hard_force_norm_limit: 20.0
  hard_wrench_limit: [20, 20, 20, 4, 4, 4]
  force_timeout: 0.50
```

- `sensor_frame`：原始 FTS frame。
- `tcp_frame`：导纳 TCP frame。
- `tf_lookup_timeout`：按 wrench stamp 查询 TF 的超时。
- `tf_fallback_to_latest`：stamp 查询失败时是否回退 latest TF。
- `tare_samples`：启动后自动 tare 采样帧数。
- `filter_alpha`：一阶低通系数，范围 0~1。
- `wrench_scale`：sensor frame 下的六轴缩放。
- `hard_force_norm_limit`：原始 `Fx/Fy/Fz` 范数硬限幅；0 表示关闭该检查。
- `hard_wrench_limit`：六轴分量硬限幅。
- `force_timeout`：力数据超时。

原始硬限幅在 tare 前执行，避免大阶跃被 tare 或滤波掩盖。

### admittance

```yaml
admittance:
  enable: false
  output: false
  selected_axes: [true, true, true, false, false, false]
  mass: [3, 3, 3, 0.3, 0.3, 0.3]
  damping: [45, 45, 45, 4.5, 4.5, 4.5]
  stiffness: [0, 0, 0, 0, 0, 0]
  max_velocity: [0.25, 0.25, 0.10, 0.15, 0.15, 0.15]
```

- `enable`：是否进入导纳控制。
- `output`：是否向 MPC 发布 reference。
- `selected_axes`：启用的六轴。
- `mass` / `damping` / `stiffness`：六轴导纳参数。
- 也可用 `damping_ratio`，此时：

```text
D = 2 * zeta * sqrt(M * K)
```

- `max_velocity`：六轴修正速度上限。

不设置 `admittance.max_offset`。  
修正量不再有固定最大位移，只有：

- 机械臂关节硬限位；
- IK 可达性 anti-windup；
- MPC/上层碰撞约束。

### whole_body

```yaml
whole_body:
  # Legacy baseline only. The preferred ee_pose output path does not use
  # base_share; OCS2 decides how base and arm share the motion.
  base_share: 0.4
  max_base_velocity: 0.5
  max_joint_velocity: 0.5
  max_ee_linear_velocity: 0.05
  max_ee_angular_velocity: 0.20
  max_ee_translation_offset: 0.10
  max_ee_rotation_offset: 0.30
```

- `base_share`：仅 legacy `whole_body_state` 模式使用；ee_pose 模式不使用。
- `max_base_velocity` / `max_joint_velocity`：legacy 9D reference 的速度上限。
- `max_ee_linear_velocity` / `max_ee_angular_velocity`：ee_pose 模式任务空间目标的速度上限。
- `max_ee_translation_offset` / `max_ee_rotation_offset`：ee_pose 模式任务空间修正的 anti-windup 边界。

`whole_body.max_base_delta` 和 `whole_body.max_joint_delta` 不用于 ee_pose 模式。  
ee_pose 模式下底盘/机械臂分工由 OCS2 决定；legacy 模式仍保留原有 IK 可达性限制。

### safety

```yaml
safety:
  observation_timeout: 0.25
  capture_settle_time: 1.0
  enforce_single_target_owner: true
  loop_rate: 50.0
```

### output

```yaml
output:
  # ee_pose: publish a 7D EndEffectorPose target to the OCS2 EE reference.
  # whole_body_state: legacy 9D base_share + IK baseline.
  mode: ee_pose
  reference_horizon: 1.0
  reference_dt: 0.1
  input_dimension: 8
```

## 故障保持

以下故障会锁存并停止导纳控制：

- `FAULT_WRENCH_TIMEOUT`
- `FAULT_OBSERVATION_TIMEOUT`
- `FAULT_WRENCH_LIMIT`
- `FAULT_WRENCH_FRAME`
- `FAULT_WRENCH_TF`
- `FAULT_WRENCH_INVALID`
- `FAULT_TARGET_OWNER`

故障时节点只读取一次当前观测，然后持续发布同一个固定 hold reference，不再逐帧追随实测位置。故障需要重启部署恢复。

## 仿真

MuJoCo 入口：

```bash
ros2 launch tracer_jaka_bringup mujoco_hardware_interface.launch.py viewer:=true
ros2 launch tracer_jaka_bringup whole_body_force_control_profiles.launch.py \
  use_rviz:=false profile:=three_axis_follow
```

主要 profile：

- `sensor_z`
- `three_axis_admittance`
- `three_axis_follow`
- `infinite`
- `20s`

虚拟力节点：

```bash
ros2 run whole_body_force_control fake_wrench_sequence.py
```

手动修改力：

```bash
ros2 param set /virtual_force_publisher force "[5.0, 0.0, 0.0]"
ros2 param set /virtual_force_publisher torque "[0.0, 0.0, 1.0]"
```

自动时序：

```bash
ros2 run whole_body_force_control fake_wrench_sequence.py \
  --ros-args -p sequence_enabled:=true
```

单轴顺序测试：

```bash
ros2 run whole_body_force_control axis_wrench_sequence.py
```

默认顺序为：

```text
2 s  zero
2 s  +X 5 N
2 s  zero
2 s  +Y 5 N
2 s  zero
2 s  +Z 5 N
2 s  zero
```

可用参数：

```bash
ros2 run whole_body_force_control axis_wrench_sequence.py \
  --ros-args \
  -p force_magnitude:=5.0 \
  -p zero_duration:=2.0 \
  -p hold_duration:=2.0 \
  -p final_zero_duration:=2.0 \
  -p loop:=false
```

## 碰撞说明

力控节点只保证：

- 关节硬限位；
- IK 可达性 anti-windup；
- reference 速度平滑。

力控节点当前没有独立碰撞检测接口。  
“满足碰撞约束”必须由 OCS2/MPC 的 collision model 或额外碰撞检测模块保证。
