# Force-MPC 参数来源与加载顺序

本说明适用于 `force_mpc.launch.py` 的仿真和实机。2026-10-09 根据用户要求恢复原版 task.info 的速度约束格式，撤回上一轮由 YAML 覆盖 MPC 速度上下界的改动。

## 规划、执行和 hardware interface

| 环节 | 配置来源 | 实际行为 |
|---|---|---|
| MPC 关节位置软约束 | task_force_mpc.info → jointPositionLimits | activate 控制是否添加机械臂位置惩罚；位置上下界来自 URDF；mu/delta 控制松弛障碍惩罚 |
| MPC 底盘和关节速度软约束 | task_force_mpc.info → jointVelocityLimits | lowerBound/upperBound 的 base.wheelBasedMobileManipulator 为 [v, omega]；arm 为六个关节速度；mu/delta 控制惩罚 |
| MRT 指令限幅 | force_mpc.yaml → wbmm_mrt_node | arm_max_command_velocity 限制关节位置指令变化率；base_max_linear_velocity/base_max_angular_velocity 限制底盘输出速度 |
| JAKA hardware interface 反馈保护 | force_mpc.launch.py → wbmm_hardware_interface.launch.py → URDF hardware 参数 | 监测实测关节速度，超限停止；与规划约束和指令限幅的对象不同 |
| 末端导纳参考速度 | force_mpc.yaml → end_effector | 平移/旋转向量范数限制，单位分别为 m/s、rad/s |

速度配置按职责分别保留，不自动互相覆盖。当前仿真 task 的规划值为底盘 ±0.1 m/s、±0.4 rad/s，机械臂 ±0.2 rad/s；当前实机 task 保留原来的底盘 ±0.1 m/s、±0.4 rad/s、机械臂 ±0.5 rad/s。执行 YAML 保持底盘 0.1 m/s、0.4 rad/s、机械臂 0.2 rad/s。需要改变规划速度时修改 task 的上下界；需要改变实际指令速度时修改 YAML。

`jointPositionLimits` 并非无效。它将 URDF 中每个机械臂关节的 lower/upper 加入 StateInputSoftBoxConstraint；关闭 activate 只去掉位置项，速度项仍从 task.info 加载。底盘 x/y/yaw 不属于这一机械臂关节位置限制。

JAKA 已有反馈保护的传参关系保持原样：

```text
safety_max_joint_velocity = 2 × arm_max_command_velocity
safety_max_tracking_error = arm_max_delta_per_step
                          + 2 × arm_max_command_velocity / mrt_loop_rate
```

当前 arm_max_command_velocity=0.2 时，hardware interface 的实测速度阈值为 0.4 rad/s。该机制检测反馈速度并停止，不是 MPC 规划速度约束，也不是对位置指令做限速。

底盘驱动 tracer_messenger 直接把 Twist 的 linear.x/angular.z 转给 SetMotionCommand，当前软件中没有同类可配置反馈速度保护；MRT 已有的底盘指令限幅继续保留。MuJoCo bridge 另有自己的底盘速度限幅。本次没有新增底层保护机制或包依赖。

## YAML 覆盖顺序

每个节点只读取自己节点名称下的 ros__parameters。后加载的同名参数覆盖前面的值。

力处理和导纳节点按顺序加载：

1. config/common/force_control.yaml：共享默认配置、话题、坐标系。
2. config/{sim,real}/force_control.yaml：后端配置。
3. config/common/force_mpc.yaml：force-MPC 功能参数。
4. force_params_file：用户局部覆盖，可只填写需要修改的字段。
5. calibration_file：仅覆盖力处理节点；实机默认加载 real/force_mpc_calibration.yaml。
6. launch 组合结果：时钟、接口连接、输出模式等。

MPC/MRT 先加载 common/ocs2.yaml，仿真再加载 sim/ocs2.yaml；MRT 随后加载第 3/4 层的执行参数。MPC 不接收 YAML 的机械臂/底盘速度限值。

`task_file` 选择一个完整 INFO 文件，不做 INFO 文件分层合并。默认选择对应后端的 task_force_mpc.info，其位置和速度软约束都保留有效。

## 其他调参入口

| 调整内容 | 配置入口 |
|---|---|
| 导纳 M/D/K、启用轴、末端速度 | force_mpc.yaml |
| 各轴力/力矩上限、滤波、死区、tare | force_mpc.yaml → force_sensor_processor |
| MPC 预测窗口、求解频率、代价和约束惩罚 | task_force_mpc.info |
| 奇异值/操作度过程与终端代价 | task_force_mpc.info → armManipulability |
| MRT 循环频率与位置前视时间 | common/ocs2.yaml，可由用户覆盖文件调整 |
| 重力补偿与标定变换 | real/force_mpc_calibration.yaml |
| 仿真物理伺服 kp/kv | tracer_jaka_mujoco/config/arm_servo.yaml |

保留上一轮与速度无关的配置整理：TCP 从 force_sensor_processor.force_sensor.tcp_frame 传给导纳/MPC/MRT/力源；处理后力话题从生产者传给导纳；admittance.enable:=auto 读取合并后的 YAML，显式 true/false 才覆盖它。专用 INFO 已移除当前 MRT 不使用的 mrtDesiredFrequency；实际执行频率由 mrt_loop_rate 决定。

raw_timeout、force_timeout、force_gate.timeout、robot_state_timeout 监测不同的数据，分别保留。不同环节的采样/控制频率也分别保留。filter_cutoff_hz > 0 时优先于 filter_alpha。

配置在启动时读取，修改后重启 launch。仅改变运行中的 ROS 参数不保证已构造的 MPC 问题或控制器随之更新。

本次验证记录：docs/diagnostics/force_mpc_task_limits/verification.json。上一轮 force_mpc_configuration 目录保留为历史快照，其 YAML 覆盖 MPC 速度的结论已被本次改动撤回。

本次三个包编译通过，35 项 C++ 测试与 46 项配置/启动测试通过；六方向拖动/松键共 13 阶段通过，静止/松键最大单轴 TCP 峰峰值 0.367 mm，断力停止与恢复通过。未执行实机运动验证。
