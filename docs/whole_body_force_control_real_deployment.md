# Whole Body Force Follow 实机部署方案

## 1. 首轮目标和边界

首轮只验证“有符号外力跟随”：操作者沿工具坐标系 `Fx` 推/拉末端，
`whole_body_force_control` 生成 9D 全身参考
`[base_x, base_y, base_yaw, q1..q6]`，OCS2/MRT 同时驱动差速底盘和 JAKA 机械臂。

本入口不启动 WipePlanner、REMANI、MoveIt Servo 或其他目标发布器。首轮也不启用
恒力和二阶导纳，避免把传感器符号、坐标系、重力补偿和全身跟踪问题混在一起。

```text
/fts_broadcaster/wrench
          |
          v
whole_body_force_control -- /mobile_manipulator_mpc_target --> OCS2/MRT
                                                              |       |
                                                              v       v
                                                        JAKA arm cmd   /cmd_vel
```

差速底盘不能横移，因此首轮将工具 `Fx` 的有符号测量映射为底盘当前朝向的正/反
位移；`base_share: 0.35` 表示目标优先由底盘分担约 35%，其余由机械臂 IK 分担。

## 2. 已固化的接口和安全默认值

| 项目 | 值 |
|---|---|
| 实机入口 | `whole_body_force_control_real.launch.py` |
| wrench | `/fts_broadcaster/wrench`，`frame_id=tool0` |
| OCS2 观测 | `/mobile_manipulator_mpc_observation` |
| 唯一目标 | `/mobile_manipulator_mpc_target` |
| 状态 | `/whole_body_force_control/control_state` |
| 诊断 | `/whole_body_force_control/status` |
| 使能/停止 | `/whole_body_force_control/enable` (`std_srvs/srv/SetBool`) |

实机 launch 默认四道门全部关闭：

- `jaka_read_only:=true`：机械臂硬件只读；
- `command_output_enabled:=false`：不向底盘发 `/cmd_vel`；
- `force_reference_output_enabled:=false`：不向 OCS2 发目标；
- `force_control_armed:=false`：不计算活动力跟随参考。

打开参考输出时，launch 强制要求前三项形成一致的显式组合：JAKA 可写、底盘输出
开启且 `safety_release:=true`。运行时还会检查观测/wrench 新鲜度、15 N/2 N·m
硬阈值以及 `/mobile_manipulator_mpc_target` 是否存在其他发布者。故障会锁存并撤销
armed，不能自动恢复运动。

这些阈值是保守的调试起点，不是经过安全认证的限值。现场仍需实体急停、底盘
架空/限速区域和第二名观察员。

## 3. 阶段 A：只读接线和方向标定（L5，不运动）

先保证工具无接触、机械臂静止、负载参数正确，再启动默认入口：

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tracer_jaka_bringup whole_body_force_control_real.launch.py
```

检查控制器、数据频率、坐标系和目标所有权：

```bash
ros2 control list_controllers
ros2 topic hz /fts_broadcaster/wrench
ros2 topic echo /fts_broadcaster/wrench --once
ros2 topic echo /whole_body_force_control/control_state
ros2 topic info /mobile_manipulator_mpc_target -v
```

验收条件：

1. `joint_state_broadcaster`、`fts_broadcaster` 等预期控制器为 active；
2. wrench 连续、有限、无明显漂移，消息 `frame_id` 为 `tool0`；
3. 空载静止值接近零，轻推与轻拉时 `force.x` 符号相反；
4. 默认状态为 `DISABLED`，底盘和机械臂不运动；
5. 除本节点外没有任何 `/mobile_manipulator_mpc_target` 发布者。

JAKA 硬件初始化会采集零偏置样本；这个阶段工具必须卸载且静止。负载质量、质心、
传感器安装方向或 `force_scale` 未确认前，不得进入运动阶段。若“向前推”得到的符号
与期望相反，在
`whole_body_force_control/config/force_follow_real.yaml` 中只改
`force_scale: -1.0`，然后重新执行本阶段；不要同时交换多个坐标轴来碰运气。

当前 `jaka_hardware_interface` 会先用项目配置的安装旋转矩阵和力臂把原始 wrench
变换到 `tool0`，然后 broadcaster 才以 `frame_id=tool0` 发布。安装旋转矩阵目前仍在
驱动代码中，力臂来自 ros2_control 参数；MuJoCo 无法验证二者与现场安装误差。因此
“空载静态接近零、六个轴逐轴施力方向正确、改变机械臂姿态后无不可接受的重力漂移”
是进入阶段 C 的硬门槛，而不是可跳过的调参建议。未满足时只能停留在 L5。

## 4. 阶段 B：影子计算（仍不运动）

保持默认 launch 运行，确认 wrench 与 OCS2 观测都新鲜后使能：

```bash
ros2 service call /whole_body_force_control/enable \
  std_srvs/srv/SetBool "{data: true}"
```

此时 `reference_output_enabled=false`，节点只计算和发布诊断，不发 OCS2 目标。
等待 `SETTLING -> ACTIVE` 后，以不超过约 5 N 的小力推、拉，记录：

```bash
ros2 bag record /fts_broadcaster/wrench \
  /whole_body_force_control/status \
  /whole_body_force_control/control_state \
  /mobile_manipulator_mpc_observation
```

验收条件：推/拉的 `filtered_force` 和 `control_offset` 均能双向变号；撤力后偏置
接近零；没有 `FAULT_*`。任何方向不一致、静态偏置或突跳都应在此阶段解决。

停止/清故障统一使用：

```bash
ros2 service call /whole_body_force_control/enable \
  std_srvs/srv/SetBool "{data: false}"
```

## 5. 阶段 C：受控实机运动（L6，需现场授权）

将底盘架空或置于空旷限速区，机械臂放在远离奇异位形和关节限位的中间姿态；确认
JAKA 自带力控/导纳功能关闭，避免两个外环互相竞争。然后使用完整运动门启动，但仍
保持 `force_control_armed:=false`：

```bash
ros2 launch tracer_jaka_bringup whole_body_force_control_real.launch.py \
  jaka_read_only:=false \
  command_output_enabled:=true \
  safety_release:=true \
  force_reference_output_enabled:=true \
  force_control_armed:=false
```

确认所有状态稳定后，再通过服务人工使能。首轮只施加约 2--5 N，单次 1--2 秒，
观察底盘和机械臂是否同向分担；释放后立即检查回零。当前实机配置限制为：

- 力跟随偏置 `max_offset=0.020 m`；
- 偏置变化率 `max_velocity=0.010 m/s`；
- 底盘相对名义位置 `max_base_delta=0.010 m`；
- 单关节相对名义位置 `max_joint_delta=0.080 rad`；
- MRT 侧机械臂单步/速度还有限幅 `0.03 rad`、`0.08 rad/s`。

先完成正向推、撤力、反向拉、撤力四个独立动作。发生以下任一情况立即急停并以
`data: false` 退出：方向相反、底盘与机械臂互相对抗、明显振荡、wrench 跳变、
TF/观测中断、接近关节/碰撞限制或出现任何 `FAULT_*`。

## 6. MuJoCo fake wrench 验证

自动回归：

```bash
ros2 launch tracer_jaka_bringup force_control_mujoco_test.launch.py viewer:=true
```

它向 `/whole_body_force_control/fake_wrench` 依次发送
`0, +5, +12, 0, -8, 0 N`，验证底盘和机械臂双向参与、释放回零、无侧滑、无控制
故障和无意外碰撞。2026-09-02 的隔离 ROS domain 实测结果保存在
[`docs/validation/whole_body_force_control_mujoco_fake_wrench.json`](validation/whole_body_force_control_mujoco_fake_wrench.json)。

这个测试属于 fake wrench 的 MuJoCo 无接触跟踪验证：证明了消息接口、正负方向、
全身目标和仿真执行链路；它不证明真实传感器重力补偿、接触稳定性、网络时延或实体
急停有效性。完成实机跟踪验收后，再单独设计带接触体的二阶导纳测试。
