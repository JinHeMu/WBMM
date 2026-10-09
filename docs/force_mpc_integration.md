# 力跟随与 OCS2 联合控制

入口：`ros2 launch tracer_jaka_bringup force_mpc.launch.py`。

机械臂奇异性裕度已接入专用 `config/{sim,real}/task_force_mpc.info`。仿真默认启用奇异值过程／终端代价，操作度可在任务配置中叠加；实机新增代价默认关闭。公式、参数、键盘施力对比和报告见 [机械臂裕度说明](force_mpc_arm_margin.md)。
处理节点、导纳算法、MRT 联锁和虚拟力源均为 C++；launch 使用 Python，
导纳与执行参数集中在 bringup 的 YAML 中，MPC 代价与求解器参数位于专用 task.info。沿用 WBMM 的处理节点与导纳节点，
参考 jaka_keyboard 已测试的质量、阻尼、速度限制和重力补偿设计。

## 数据链路

```text
EDG 原始六维力 / MuJoCo 原始力 / C++ 虚拟力源
  -> force_sensor_processor
     重力及偏置补偿 -> 可选残余 tare -> sensor 到 TCP 完整 wrench 变换
     -> 未滤波硬限制 -> 按采样间隔低通 -> 各轴死区
  -> whole_body_force_control
     当前 TCP 分量转到启用时固定的 TCP 轴 -> 六轴导纳
     -> 平移/旋转范数速度限制 -> odom 中的 7D 末端目标
  -> OCS2 Execution 阶段 -> MRT -> 底盘速度与机械臂位置指令
```

完整 wrench 变换包含 `tau_tcp = R * tau_sensor + p.cross(R * f_sensor)`。
导纳前只是将同一当前 TCP 原点的力旋转到固定轴，不能再加上到启动 TCP
位置的力臂。固定轴避免工具旋转后力的方向被重复旋转。

OCS2 保持既有约定：9D 状态 `[x,y,yaw,q1..q6]`、8D 输入
`[v,omega,qdot1..qdot6]`、7D 末端目标 `[x,y,z,qx,qy,qz,qw]`。
底盘是差速模型，没有独立横向速度。底盘/机械臂分配继续由现有 OCS2
代价和约束决定，不保证每个方向的拖动都有同样的底盘参与量。

## 参数

修改 `src/bringup/config/common/force_mpc.yaml`，或复制后通过
`force_params_file:=/absolute/path/my_force_mpc.yaml` 加载。
按顺序加载 `common/force_control.yaml`、对应 backend 的 `force_control.yaml`、
默认 `common/force_mpc.yaml`，再加载用户覆盖文件。用户文件可只包含要修改的字段。力/力矩只检查各轴分量，导纳速度只限制末端平移和旋转
向量的范数；不再配置力范数或导纳逐轴速度上限。

六轴顺序为 `[x,y,z,rx,ry,rz]`，单位为 m、rad、N、Nm、s：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `admittance.selected_axes` | `[true,true,true,false,false,false]` | 三轴平移跟随；可独立开启旋转 |
| `admittance.mass` | `[20,20,20,2,2,2]` | 各轴虚拟质量/转动惯量 |
| `admittance.damping` | `[200,200,200,15,15,15]` | 各轴阻尼 |
| `admittance.stiffness` | `[0,0,0,0,0,0]` | K=0 持续拖动，撤力后停止在新位置；K>0 返回启用时位置 |
| `end_effector.max_linear_velocity` | `.2` | 末端导纳平移速度范数上限（m/s） |
| `end_effector.max_angular_velocity` | `.4` | 末端导纳旋转速度范数上限（rad/s） |
| `force_sensor.hard_wrench_limit` | `[30,30,30,10,10,10]` | 各轴力/力矩绝对值上限（N/Nm） |
| `force_sensor.filter_cutoff_hz` | `4.44` | 约等于 125 Hz 时 alpha=.2，随实际采样间隔计算系数 |
| `force_sensor.force_deadband_n` | `1` | 各轴力死区 |
| `force_sensor.torque_deadband_nm` | `.5` | 各轴力矩死区 |
| `force_sensor.tare_samples` | `50` | 去皮样本数 |
| `force_sensor.tare_after_compensation` | `true` | 实机先去重力，再采残余偏置；避免重力重复扣除 |
| `force_sensor.tare_on_start` | `true` | 无重力模型时启动 tare |
| `safety.max_tracking_error_m/rad` | `.10 / .35` | 目标与实测差距过大时锁存停止；0 关闭对应检查 |

MRT 的心跳/状态超时、底盘速度和机械臂指令速度限制也在同一 YAML 的
`wbmm_mrt_node` 中。速度限制反馈到导纳修正，避免限速后的积分持续堆积；
跟踪误差过大则停止，不能把该检查视为碰撞或工作空间检查。

`tare:=auto` 使用 YAML，`tare:=false/true` 同时覆盖两项 tare 开关。
去皮时保持自由空间、不接触、不施力；小于硬限制的持续外力仍会被当作零偏。

## 重力补偿及坐标系

实机默认加载 `src/bringup/config/real/force_mpc_calibration.yaml`，
它只提取修复后的
`/home/a/replace_disk_robot/logs/force_direction_audit/candidate_tool0_repaired_bounded4.json`
中的补偿字段，附源文件 SHA256，不运行 replace_disk_robot 的 Python 代码。
参考仓库当前 `tool/ft_gravity_samples_identified.json` 与该修复文件 SHA256 一致。
可通过 `calibration_file:=/absolute/path/calibration.yaml` 替换。

原始 EDG 轴沿用 broadcaster 的 `jk_se_vi_200_link` 标签；重力向量位于
`jaka_base_link`；控制 TCP 为 `tool0`；目标在 `odom`。
实机传入 `torque_sensor_mode:=1`，与参考程序的未补偿原始传感器模式一致。
驱动读取失败后不再用旧值伪装新样本，而是使状态失效并报告 ros2_control 错误。

修复标定包含物理传感器到 tool0 的外参，使用
`force_sensor.use_calibrated_transform: true` 和标定的 R、p。
该外参与 CAD 传感器 mesh 原点不同，因此不再额外叠加 CAD TF。
重力姿态仍通过 `tool0 <- jaka_base_link` 的 TF 获取。
仿真使用其自身 URDF 外参，不加载实机标定。
换工具、传感器安装或原始轴约定后需更新对应 YAML；本次未验证 WBMM 实机下的标定残差。

## 编译和仿真

在 WBMM 根目录：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  whole_body_force_control wbmm_ocs2_ros jaka_hardware_interface \
  tracer_jaka_description tracer_jaka_bringup
source install/setup.bash
ros2 launch tracer_jaka_bringup force_mpc.launch.py \
  backend:=sim fake_wrench:=true viewer:=true use_rviz:=true
```

虚拟源初始力为零，启动后等待 `/whole_body_force_control/states` 和
`/mobile_manipulator_force_execution_state` 均为 `ACTIVE` 后再施力。

可以直接通过按键窗口测试三轴平移力控：

```bash
ros2 launch tracer_jaka_bringup force_mpc.launch.py \
  backend:=sim fake_wrench:=true keyboard_wrench:=true
```

点击 `WBMM Virtual Force` 窗口使其获得焦点，等待力控 ACTIVE 后按键：

| 正向 / 负向 | 作用轴 |
| --- | --- |
| W / S | tool0 的 +X / -X |
| A / D | tool0 的 +Y / -Y |
| R / F | tool0 的 +Z / -Z |

按住持续施力，松开对应轴归零；可以同时按不同轴，同轴正负同时按下则抵消。
默认各轴 5 N，力矩为零，按键和幅值在 `force_mpc.yaml` 的
`keyboard_force_publisher.keys/force_n` 中配置。空格清零，Esc 或关窗退出。
窗口失去焦点时清零；按键节点退出后，虚拟传感器在 0.15 s 内清零并继续发布零力。

如果已用 `fake_wrench:=true` 启动仿真，可单独打开按键节点：

```bash
ros2 run whole_body_force_control keyboard_wrench_node
```

按键节点只发布 `/whole_body_force_control/virtual_wrench_command` 命令。
现有 `virtual_force_publisher` 是唯一原始力发布者，将 tool0 的力/力矩完整逆变换
到仿真传感器轴，再经过原有力处理和导纳链路；不会绕过 processed wrench。
该逆变换抵消力臂力矩，因此按键产生的 TCP 力矩确实为零。
按键窗口使用 Linux X11/XWayland，需在图形桌面终端中运行。

也可以继续用参数手动输入原始**传感器轴**力：
使用参数或自动闭环 probe 时关闭按键节点，避免它的连续命令覆盖参数输入。

```bash
source /opt/ros/humble/setup.bash
source /home/a/WBMM/install/setup.bash
ros2 param set /virtual_force_publisher force '[5.0, 0.0, 0.0]'
ros2 param set /virtual_force_publisher force '[0.0, 0.0, 0.0]'
ros2 param set /virtual_force_publisher force '[-5.0, 0.0, 0.0]'
```

这些值在**传感器轴**，不是 odom 轴。`torque` 同样接受三个 double。
`publish_enabled:=false` 可测试断流。
省略 `fake_wrench:=true` 时使用 MuJoCo 的 `/fts_broadcaster/wrench_raw`，
由 C++ 处理去皮/滤波；虚拟源不会启动。两种力源只选一种。

若 backend 已由其他入口启动，可传 `start_backend:=false`；需自行提供相同
odom、joint_states、TF 和执行接口，且不得同时启动其他末端目标发布器。
MPC 代价与求解器调参使用 backend 的 `task_force_mpc.info`，可用 `task_file:=...` 替换。
规划速度在 task 的 jointVelocityLimits 中修改；MRT 指令限幅和硬件保护传参使用
force_mpc.yaml 中的执行速度，YAML 不覆盖 MPC 规划约束。
参数加载顺序、共享接口和各环节参数职责见 [参数配置说明](force_mpc_configuration.md)。

## MuJoCo 关节执行与晃动修复

仿真默认加载 `tracer_jaka_mujoco/config/arm_servo.yaml`：恢复原 XML 的六轴
`kp`/`kv` 数值，保留重力/Coriolis 补偿和 `implicitfast` 积分。
上一轮提高增益的设置虽通过独立关节阶跃测试，但加重了完整 MPC 闭环的晃动，
因此已撤回。MuJoCo 位置执行器的积分建议见
[官方文档](https://mujoco.readthedocs.io/en/3.8.0/computation.html)。

当前执行链为：

```text
导纳 EE 参考 → OCS2 MPC → 预测关节位置 → C++ 位置指令限速 → MuJoCo 物理伺服
```

MRT 统一跟踪 MPC 的预测关节位置，已移除可选的关节速度积分模式及其参数。
导纳 M/D/K、按键力和末端速度上限由对应配置决定。
位置指令逐轴同时受 `arm_max_command_velocity × 仿真 dt` 和
`arm_max_delta_per_step`（相对实际反馈的最大超前量）限制。后者不是 rad/s。
故障保持/策略过期后清空指令状态，恢复时从反馈重新起步。

底盘使用当前时刻 MPC 输入，机械臂使用未来预测关节位置。
本次没有进行实机运动验证。

2026-10-09 参数精简验证：受影响的三个包编译成功，37 项 C++ 测试及
42 项配置/启动测试通过。完整 MuJoCo 闭环的六方向拖动/松键共 13 阶段通过，
静止/松键阶段最大单轴 TCP 峰峰值为 0.394 mm；力源断开、保持和复位恢复通过。
首轮零力启动出现过关节反馈突跳，连续位置指令未发生同等突跳，随后触发原有跟踪误差停止。
原因尚未确认，后续两次全新启动未重现；失败记录与通过记录均保留于
[参数精简验证记录](diagnostics/force_mpc_parameter_cleanup/verification.json)。

导纳在 `use_sim_time=true` 时按 `/clock` 积分，MPC/MRT 使用同一时间基准；
暂停时导纳目标不继续积累。输入超时与故障心跳仍使用墙钟，时钟回退则锁定
`FAULT_CLOCK_RESET`。MRT 的实际控制频率改用墙钟统计，同时报告 `sim/wall`；
之前慢仿真时出现的 335 Hz 等数字不再冒充实际控制频率。

关节参数在启动时读取，`kp`/`kv` 的单位为 Nm/rad、Nm/(rad/s)，按 `joint_1` 至
`joint_6` 排列。它们是仿真执行层的参数，与导纳阻尼分开。执行器力矩限制、关节限制、
碰撞与 F/T 仍由 MuJoCo 计算，没有覆盖机械臂 `qpos`。

启动方式不变：

```bash
source /home/a/WBMM/install/setup.bash
ros2 launch tracer_jaka_bringup force_mpc.launch.py \
  backend:=sim fake_wrench:=true keyboard_wrench:=true
```

完整闭环报告见 `docs/force_mpc_oscillation_validation.json`。使用上述命令，查看器、
RViz 和 C++ 按键窗口都开启；按下/松开 W/S、A/D、R/F 测试三轴正负各 10 N。
修复前零力/撤力段末端单轴最大峰峰值约 20–21 mm；修复后三轴撤力稳定段最大
约 0.61 mm。关节速度的快速往返波动也显著下降，13 个阶段全部通过。
MPC 仍可能缓慢调整冗余关节姿态，峰峰值统计需结合速度波动和末端运动区分。
这些结果是当前模型与当前参数下的仿真证据，不代表任意增益下的稳定性或实机精度。

可重复测试（先启动上述仿真）：

```bash
python3 /home/a/WBMM/src/bringup/test/force_mpc_oscillation_check.py \
  --all-axes --output /tmp/force_mpc_oscillation.json
```

测试脚本只向 WBMM 按键力窗口发送事件，结束时释放所有按键；只观察 ROS 输出，
不自行发布虚拟力。还单独验证了暂停/慢速仿真中的导纳时间，以及断流、停止、保持与恢复。
独立物理执行器验证见 `docs/mujoco_arm_servo_validation.json`，先前高增益版本的
`docs/mujoco_arm_servo_closed_loop_validation.json` 已标记为历史记录，不能作为当前配置的验收。

## 实机与恢复

实机默认只读反馈、导纳关闭：

```bash
ros2 launch tracer_jaka_bringup force_mpc.launch.py backend:=real
```

允许输出的入口为：

```bash
ros2 launch tracer_jaka_bringup force_mpc.launch.py \
  backend:=real hardware_write:=true tare:=true
ros2 service call /whole_body_force_control/enable std_srvs/srv/SetBool '{data: true}'
```

enable 服务仅在状态、力数据新鲜且传感器为 ACTIVE 时成功；启用时重新捕获
名义位姿，再等新的 MPC 策略后执行。`hardware_write` 属于启动时选项；
只读入口不能通过 enable 服务变为硬件执行入口。
停止拖动可调用同一服务 `{data: false}`，底盘停止、机械臂保持。

故障、力控心跳断流、odom/joint_states 超时均使 MRT 停底盘并固定捕获的
机械臂保持目标。数据恢复不会自动清除故障。恢复顺序：

```bash
ros2 service call /whole_body_force_control/force_sensor/reset std_srvs/srv/Trigger '{}'
ros2 service call /whole_body_force_control/reset std_srvs/srv/Trigger '{}'
# 实机默认 reset 后关闭导纳；等待传感器 ACTIVE 后重新启用
ros2 service call /whole_body_force_control/enable std_srvs/srv/SetBool '{data: true}'
# 等待力控 ACTIVE 后复位 MRT；未 ACTIVE 时该服务会拒绝
ros2 service call /mobile_manipulator/force_control/reset_interlock std_srvs/srv/Trigger '{}'
```

复位清除旧导纳位移，捕获当前位姿，并等待复位后的新策略，防止重放旧目标。
控制链进程退出后，组合启动会在 0.5 s 内关闭后端；仍运行的 MRT 有时间
通过 0.25 s 心跳/策略超时发布停止。

## 验证范围

C++ 单元测试覆盖重力与残余 tare、采样率变化、未滤波力矩尖峰、
旋转坐标系和速度范数限制；MRT 单元测试覆盖心跳联锁及显式恢复。
bringup 配置测试检查仿真/实机连接、标定层、tare 开关与执行门。

`BUILD_TESTING=ON` 时还提供 C++ 手动闭环检查：

```bash
# 仅在已启动 fake_wrench:=true 的隔离仿真中运行，双方 ROS_DOMAIN_ID 必须一致
ros2 run whole_body_force_control force_mpc_integration_probe
```

它通过虚拟力源参数施加正/反向力，并检查实测 TCP、底盘/机械臂参与量、
撤力保持、断流停止、恢复不自动运动及显式复位；结果写入
`/tmp/wbmm_force_mpc_report.json`。这是 MuJoCo 闭环证据，不能替代 WBMM 的实机测试。

本次最终配置的记录见 `docs/force_mpc_validation.json`：5 N 施力 10 s 后实测
TCP 前进 0.1662 m，底盘移动 0.1978 m，机械臂关节变化范数 0.0926 rad；
撤力稳定后的 3 s 漂移 7.52 mm，反向施力位移 -0.1635 m。
断流时底盘指令为零、机械臂保持指令变化为零，数据恢复未自行解锁；
显式复位后位移 2.41 mm，未观测到非地面碰撞。

按键节点单独验证了六个方向的处理后力（各轴 ±5 N、TCP 力矩为零），
松键、多轴组合、正负抵消、失焦、空格、关窗及按键进程异常退出归零。
记录见 `docs/keyboard_force_validation.json`。测试采用发往按键窗口的 X11
按下/松开事件，并检查运行中的 MuJoCo 力处理输出和 ACTIVE 状态。
