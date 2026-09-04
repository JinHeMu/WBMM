# whole_body_force_control

通用移动机械臂力控制包，不依赖 WipePlanner 的任务几何、覆盖路径或状态机。
本包只拥有力控算法和配置；MuJoCo 闭环验证入口为
`tracer_jaka_bringup/force_control_mujoco_test.launch.py`。

## 功能边界

- `AdmittanceController`：实现单轴
  `M*x_ddot + D*x_dot + K*x = F_measured - F_desired`。
  - `F_desired=0`：普通外力导纳；
  - `F_desired>0`：恒力误差导纳。
- `ForceFollower`：无阻尼项的准静态跟随
  `x_target=(F_measured-F_desired)/K`，带低通、限速和位移限幅。
- `CartesianComplianceController`：六个相互独立的导纳通道，轴顺序固定为
  `[Fx,Fy,Fz,Tx,Ty,Tz] -> [dx,dy,dz,rx,ry,rz]`。
- `WholeBodyKinematics`：用完整 6D IK 实现末端平移和转动修正；底盘只分担
  平移在当前航向上的分量，转动修正由机械臂实现。
- `whole_body_force_control_node`：接收力传感器和 OCS2 观测，发布完整9D状态、
  8D输入参考。

## 控制模式

| `control_mode` | 目标力 | 动力学 |
|---|---:|---|
| `admittance` | 强制按 0 N | 二阶 `M-D-K` 外力导纳 |
| `constant_force` | `desired_force` | 二阶恒力误差导纳 |
| `force_follow` | 默认 0 N | 无D准静态力—位移跟随 |

恒力模式中输出符号由 `F_measured-F_desired` 决定；通过
`response_body_x/y/z` 配置正位移方向。实际接触任务仍必须配置碰撞、接触方向、
最大位移和传感器超时保护。

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
该帧数据。已经收到过数据后，力传感器或 OCS2 观测一旦超时会锁存故障、撤销
`armed`，并在参考输出门已打开时发送“保持当前观测”的参考；不会把失联伪装成
零力继续运动。故障排除后需调用 `/whole_body_force_control/enable` 重新使能。

为了兼容已有仿真和实机配置，默认 `admittance_axes: [legacy]`，继续使用原来的
`force_axis`、`absolute_force` 和 `response_body_x/y/z` 标量路径。

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
ros2 launch tracer_jaka_bringup whole_body_force_control_real.launch.py   jaka_read_only:=false   command_output_enabled:=true   safety_release:=true   force_reference_output_enabled:=true   force_control_armed:=false   force_params_file:=$(ros2 pkg prefix whole_body_force_control)/share/whole_body_force_control/config/force_follow_infinite_real.yaml
```

启动稳定后，再手动使能：

```bash
ros2 service call /whole_body_force_control/enable   std_srvs/srv/SetBool "{data: true}"
```

该配置启用了 `force_velocity_mode: true`：推/拉力超过 `force_deadband` 时，
机器人以 `max_velocity` 持续沿力的方向移动；撤力后停止在当前位姿，不回弹。
默认实机无限配置的移动速度很低（`0.02 m/s`），但仍必须保证场地足够空旷并随时可急停。

## 实机部署

实机入口不启动 WipePlanner 或 REMANI：

```bash
ros2 launch tracer_jaka_bringup whole_body_force_control_real.launch.py
```

默认是只读、无底盘命令、无 OCS2 目标输出、未 armed 的观察模式。完整的分阶段
检查、运动放行命令、停止方法和验收标准见
[`docs/whole_body_force_control_real_deployment.md`](../../../../docs/whole_body_force_control_real_deployment.md)。

运行时状态：

- `/whole_body_force_control/control_state`：`DISABLED`、`SETTLING`、`ACTIVE`
  或 `FAULT_*`；
- `/whole_body_force_control/enable`：`std_srvs/srv/SetBool`，仅在 OCS2 观测和
  wrench 都新鲜且没有其他 OCS2 目标发布者时允许 armed；
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

- `wrench_topic`, `status_topic`, `force_axis`, `absolute_force`
- `desired_force`, `mass`, `damping`, `stiffness`
- `max_offset`, `max_velocity`, `force_timeout`
- `observation_timeout`, `armed`, `reference_output_enabled`
- `enforce_single_target_owner`, `force_scale`, `wrench_scale_6d`
- `hard_wrench_limit`, `control_state_topic`, `enable_service`
- `response_body_x/y/z`, `base_share`, `max_base_delta`, `max_joint_delta`
- `admittance_axes`, `constant_force_axes`, `absolute_wrench_axes`
- `desired_wrench`, `mass_6d`, `damping_6d`, `stiffness_6d`
- `max_offset_6d`, `max_velocity_6d`, `filter_alpha_6d`
- `require_wrench_frame`

`/whole_body_force_control/status` 的前 9 项保持旧接口；6D 模式在其后追加
`filtered_wrench[6]`、`offset[6]`、`velocity[6]`、导纳轴掩码 `[6]` 和恒力轴掩码
`[6]`，末尾再追加 `armed`、`reference_output_enabled`、`fault_latched` 三项。
