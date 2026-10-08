# 实机 force_mpc 震荡只读排查（2026-10-08）

后续 `tare=true` 的 ROS 录包已给出更直接证据：零导纳、固定目标期间仍存在增长震荡。
最新结论见 [ROS 录包 03 分析](force_mpc_rosbag03_analysis.md)；
下文保留第一阶段只读审查结果，SDK 通道的未确认事项不再作为本次主因判断依据。

## 结论与边界

首要嫌疑是 **MPC 速度积分执行模式与实机位置伺服滤波/延迟不匹配**。
最近 MuJoCo 的修复只在仿真覆盖为预测位置执行；实机 force_mpc 仍走速度积分。
另有静态补偿残差、力数据通道语义未确认的问题，可能触发运动并加重闭环耦合。
不能在缺少震荡时同步曲线的情况下认定唯一根因，也不能声称参数修改已在实机验证。

本次没有开启力控、发布机械臂/底盘命令、初始化 EDG、设置传感器、修改伺服滤波或执行 hardware write。
仅查询 ROS 节点、读取历史日志，建立 SDK 查询连接并调用 getter，完成后断开连接。
没有修改运行代码或现有控制配置，保留用户的未提交 force_mpc.yaml 修改。

## Git 改动与有效执行模式

审查基线：`b6301b0`；未提交改动：平移质量 10→20 kg、机械臂命令限速 .3→.2 rad/s、底盘限速 .2→.1 m/s。
安装目录的 force_mpc.yaml 与当前源文件对应；历史运行到底加载了哪一版质量/限速，普通节点日志没有完整参数快照，不能追溯确认。

参数加载顺序见 `src/bringup/launch/force_mpc.launch.py`：

| 项目 | real | sim |
| --- | --- | --- |
| common/ocs2.yaml | velocity integrator=false | 同左 |
| common/force_mpc.yaml 覆盖 | velocity integrator=true | 同左 |
| sim/force_mpc.yaml 最后覆盖 | 不加载 | velocity integrator=false |
| 执行方式 | 积分当前 MPC qdot，再发送绝对 q | 取未来预测 q，再限速发送 |

最新提交把速度积分的输入从“未来策略输入”修正为“当前策略输入”，但保留了实机这层积分。
MuJoCo 另外恢复了物理伺服增益、使用 implicitfast 和模型动力学补偿；这些不会改变 JAKA 伺服。
历史文档也明确该提交未验证实机闭环。

## 首要嫌疑：滤波位置伺服前叠加速度积分

`deploy/start.sh` 运行 `jaka_driver/jaka_login`。其代码调用
`servo_move_use_joint_LPF(2)`，为 servo_j/servo_p 配置一阶 2 Hz 低通。
按一阶滤波模型，时间常数为 `1/(2*pi*2)=79.6 ms`，并非固定纯延迟。
当天 start 日志确认初始化成功；没有从 SDK getter 读取到关节伺服 LPF 参数，因此
2 Hz 是启动代码和运行日志支持的配置，不是独立实测的当前伺服传递函数。

MPC 模型 `WbmmDynamics::systemFlowMap` 仅使用 `q_dot=u_arm`，没有关节伺服滤波状态。
MRT 的 `integrateArmCommand` 使用：

```text
q_cmd[k+1] = clamp(q_cmd[k] + dt * clamp(u_arm), q_measured +/- .05 rad)
```

`q_cmd` 带跨周期记忆。实机实际追随的是经过滤波的位置指令。
MPC 基于反馈反向修正速度时，先消除的是积累在 q_cmd 内的偏移；实际关节可能仍向旧方向运动。
这种相位滞后在高增益、策略陈旧情况下可造成往返过冲，限速只能限制幅度/斜率，不能保证稳定。

最新运行日志 `wbmm_mrt_node_15926_1791440744777.log`：控制 124.9–125.5 Hz，策略约 64–72 Hz，
平均策略年龄 25–28 ms、最大 64 ms，没有策略过期/状态超时记录。
`real/task.info` 的 useFeedbackPolicy=false：MRT 使用前馈轨迹；反馈主要依靠 MPC 重规划。
策略年龄不是完整端到端反馈延迟，这里不能把它直接当成实测延迟。

一个纯离线**简化**单关节模型，采用 2 Hz 一阶位置追随、24 ms 假设反馈延迟、
`u=-60*q_measured`、125 Hz 更新、.2 rad/s 限速与 .05 rad 超前限制，
从 1 mrad 初始误差发展为约 .011 rad 的稳定往返振幅（峰峰值）。零延迟时衰减。
它演示机制，不是实际 OCS2/JAKA 重放，也不构成实机稳定性证明。
当前姿态固定底座、忽略约束的局部 LQR 估算增益约 17–115 /s；这也只是线性近似。
可复现数值与假设见 `diagnostics/force_mpc_real_20261008/analyze.py`。

## 次要嫌疑：悬空补偿残差与力通道约定

用户确认：当前与震荡时都无接触，末端带负载。
SDK 状态采样 60 次，持续约 7.14 秒（首尾），各关节读数完全不变。
传感器模式 getter 返回 1；传感器状态正常，无错误。
采样数据见 `diagnostics/force_mpc_real_20261008/status.csv`。

按当前 WBMM URDF、标定旋转、重力与 bias 离线重算：

| 数据 | Fx | Fy | Fz | 说明 |
| --- | --- | --- | --- | --- |
| 标定预测传感器数据，N | -8.832 | -9.288 | 4.047 | bias + gravity |
| SDK actTorque 平均值，N | -8.194 | -7.986 | 6.236 | SDK 监控通道 |
| actTorque 经过当前补偿后，tool0，N | .469 | -2.038 | 1.587 | 向量范数 2.626 N |
| SDK torque 平均值，N | -10.200 | 4.222 | -2.100 | 另一原始数据通道 |

SDK 各 getter 的单次交叉读取：type=1 为 actTorque；type=2 为 original；type=3 为 real without gravity/bias。
实读 type1/type3 接近一致，约 `[-8.194,-7.975,6.335] N`，type2 约 `[-10.2,4.2,-2.0] N`。
SDK `get_torque_sensor_filter` 返回 5 Hz（区别于关节伺服 2 Hz 和 ROS 后处理 4.44 Hz）。

**尚未读取同一时刻的 EDG.torqSensor，不能认定以上任一 SDK 通道必然等于 /fts_broadcaster/wrench。**
本次没有初始化 EDG，以免修改实机流配置。尤其不能拿 type2 直接替换当前输入，或宣称已确认双重重力补偿。

代码注释把 torque_sensor_mode=1 当作“未补偿原始模式”，但随仓库 SDK 头文件描述它只是 on/off。
`TorqSensorMonitorData.actTorque` 的注释还说明其语义会受 Initialize 选项影响。
因此该设置**不能保证** EDG 输入与历史标定采用相同的原始力、坐标与补偿约定。
这需要 ROS 原始力录包与上述通道对照后确认。

如果 ROS 输入等于本次 actTorque 通道，当前 1 N 死区会保留 Y/Z 残差。
`tare:=false` 覆盖并关闭两项 tare；导纳启用时 reset 不会减去启用瞬间的力。
K=0 时，恒定力的稳态速度为 F/D：约 `[0,-.0102,.00794] m/s`（启用时固定轴，初始近似）。
这足以引发无接触运动，**恒定残差本身主要产生漂移，并不足以单独解释增长震荡**。
姿态改变后的补偿误差、载荷惯性及力滤波可进一步参与动态反馈；静态重力模型不补偿惯性。
残余 tare 可减少静态触发，但不能修复执行层相位问题，也不能修复错误的输入通道。

## 目前不支持的原因

- deploy/start 是一次性的 CAN/上电/使能初始化，没有留下第二套 MPC/目标发布器。
  最近 launch/节点日志没有支持同时重复启动驱动的证据；“logger 已注册”也可能来自内部可视化节点，不能据此认定有两个硬件控制器。
- 最近运行没有故障或策略过期记录，不支持持续超时/反复联锁复位是主要机制。
  更早两次日志有 WRENCH_LIMIT，不能据此推出最新一次震荡来自传感器超限。
- 当前 M=20、D=200、125 Hz 时，导纳离散速度系数 `1-D*dt/M=.92`；之前 M=10 时为 .84。
  就 K=0 导纳自身而言都稳定，不能把质量或阻尼参数直接认定为数值积分发散。
- 当前静止姿态的 URDF 139 个配置碰撞对象对，最小距离约 .0503 m，大于 MPC minimumDistance=.02 m；
  当前关节均在模型限位内。局部 6x6 Jacobian 最小奇异值约 .213（混合 m/rad 单位，仅供对比）。
  不支持当前姿态已严重奇异/碰撞，但这不是震荡全过程的碰撞或实机几何认证。
- URDF 的工具旋转与标定修复旋转相匹配；补偿代码旋转顺序没有发现明显重复旋转或统一负号。
  软件一致性不等于安装外参已经物理验收。

## 下一步：先录静止数据，不重新开启力控

若当前没有 telemetry 节点，由用户在终端启动下面的只读入口即可。
它会初始化正常硬件反馈流，但 hardware_write=false，不开启机械臂伺服输出，导纳保持关闭。
无需重新运行 deploy/start，也不要调用 enable/reset_interlock。

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tracer_jaka_bringup force_mpc.launch.py \
  backend:=real hardware_write:=false admittance.enable:=false \
  tare:=false use_rviz:=false
```

另一个终端运行：

```bash
cd /home/a/WBMM
bash deploy/record_force_mpc.sh /tmp/wbmm_force_static_20261008
```

悬空、无人接触，静止采样 15–20 秒，Ctrl+C 结束**录包终端**。
它只订阅，包含原始力、补偿力、目标、指令、joint_states、MPC observation/policy、TF、状态与日志。
开始录包前还会用只读 parameter getter 保存三个控制节点的有效参数到 bag 目录旁的 `.params` 目录。
只读运行中 /arm_controller/commands 没有消息是预期行为。
退出只读 launch 后仅把 bag 目录路径提供给 Codex；若路径已存在，用新的目录名。
不要执行 ros2 bag play：它会发布录制的命令话题。

之后离线比较：raw 是否符合标定预期、processed 是否越过死区、TF 重力姿态是否一致、
EDG 通道与 SDK 哪一路相同，以及静止时参考是否存在漂移。
若已有震荡期间的原始记录，按同步时间观察：
1. 目标基本不动、q_cmd/q_measured 来回且滞后增长 → 执行链优先。
2. processed force/导纳参考先周期性改变，再引起指令变化 → 力反馈/补偿耦合优先。
3. 多个发布者或状态反复切换 → 再查控制权/联锁。

现阶段不要求用户再次复现实机震荡。之后如准备修复，优先为 real 单独配置预测位置执行，
将实际 LPF/延迟纳入模型或降低外环带宽，并确认原始力通道；先在离线/仿真中检验，
再由用户决定实机验证。不能仅依靠加质量、降速度或 tare 后直接声称稳定。

## 用户自行记录已有实机试验

用户随后表示愿意自行操作实机并记录震荡过程，要求 Codex 只读，后续集中分析 ROS 数据，停止继续研究 SDK。
录包仍使用 `deploy/record_force_mpc.sh`；它不会启动驱动、使能导纳、发布关节/底盘命令。
Codex 的沙箱内试启动录包因 ROS UDP socket 权限受限，无法订阅实机数据，已经停止；
该试启动目录 `/tmp/wbmm_force_oscillation_20261008_01` 不能作为有效试验记录。

用户在本机终端按原命令启动 launch 后，先暂缓 enable，在另一终端运行：

```bash
cd /home/a/WBMM
bash deploy/record_force_mpc.sh /tmp/wbmm_force_oscillation_02
```

确认 recorder 已订阅 `/joint_states`、`/fts_broadcaster/wrench` 和
`/whole_body_force_control/processed_wrench`，先留一小段未启用的基线；
后续实机启停由用户自行操作。若开始增长震荡立即结束试验，不为凑足时长继续运行。
结束实机试验后，在录包终端 Ctrl+C，提供 bag 目录路径，连同旁边的 `.params` 目录保留。
无需 PlotJuggler 导出，也不需要录视频。后续按 ROS 同步数据区分参考、MPC、指令与反馈哪个环节先出现周期变化。
