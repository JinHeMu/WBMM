# Whole-Body Force Control 实机测试部署说明

> Status: DRAFT  
> Scope: 首轮实机测试  
> Warning: 本文档是测试阶段流程，不是已验收的安全部署规范。实机运动必须有人现场监督，并保证物理急停随时可达。
>
> IMPORTANT ARCHITECTURE UPDATE:
> `JakaHardwareInterface` 现在只透传原始 FTS 数据，不再做清零、坐标变换、滤波、死区和 stale 检测。
> `whole_body_force_control` 校验传感器 frame，内部 `ForceProcessor` 完成
> 启动时自动 tare、低通滤波、有限性检查和现有 wrench 限值检查；当前节点不做 TCP 换系。
> 实机前仍需确认传感器 frame 与 URDF/TF 的轴定义和自动 tare 时工具确实无外力。

> CURRENT（配置收敛，2026-09-16）：`whole_body_force_control/config/` 只保留
> `force_follow_sim.yaml` 和 `force_follow_real.yaml`；实机入口默认读取后者。
> real 先测试传感器 Z 轴导纳柔顺，不启用无限行程，底盘参考分担保持为零。
>
> CURRENT（安全门收敛）：real launch 现在只使用 `hardware_write` 作为唯一
> 实机写入门，`false` 为只读/无命令输出，`true` 允许力控参考输出和 OCS2/MRT
> 命令输出。不再有 `command_output_enabled` / `safety_release`。
> 下文旧阶段 C/D 的描述来自历史接口，不能直接执行；单一开关下不存在
> “只打开参考输出但禁止 JAKA 写”的中间态。配置收敛没有修改或验证硬件安全链。

## CURRENT：传感器原点单轴导纳（2026-09-17）

本轮按用户指令先测导纳柔顺，随后再测跟随模式。原始/去偏置/滤波后的 wrench
均保留在 `jk_se_vi_200_link` 原点、按该 link 的轴表达，单位为 N、N·m。
柔顺参考点也改为传感器原点；不再把 wrench 变换到 TCP，不再使用 TCP 力臂项。
URDF 中传感器与工具的固定几何关系保留，OCS2 本身的末端配置未修改。

实机使用现有六轴控制器，但只开启 `fz` 导纳，期望外力为 0；其他五轴柔顺输出为零：

$$
3\ddot{\delta}_z + 45\dot{\delta}_z + 400\delta_z = F_z
$$

$F_z$ 为去偏置、滤波后的传感器 Z 向力，$\delta_z$ 为名义传感器系下的位移（m）。
平移方向使用名义传感器姿态旋转到 `odom`，并通过 6D IK 保持名义传感器姿态。
正 Fz 沿传感器 +Z 柔顺；撤力后回到名义位置。仍采用 20 mm 位移边界、10 mm/s
参考速度边界、0.05 rad 关节参考偏移边界，底盘分担为零，所有实机开关默认关闭。
配置文件仍叫 `force_follow_real.yaml`，本轮只改变其模式和传感器参考，不改文件名。

[CURRENT] 三个相关包定向构建通过，23 项核心单元测试、24 项配置/安全门/启动组合
测试通过。隔离 ROS 合成输入验证了 +2 N → +5 mm、−2 N → −5 mm、撤力回零，
且 TCP frame 输入被拒绝、参考输出始终关闭。MuJoCo 入口限时启动 25 s，模型及节点
加载成功，但 MRT 仍等待 odom/joint_states，本轮没有完成 MuJoCo 闭环运动验收。
[TBD] 实机响应、物理急停、硬件反馈冻结/失联停止和原始力峰值保护尚未验收；
本轮没有整改先前审阅指出的其余安全缺口。下文运动步骤是人工验收事项，不能仅因
本轮软件测试通过就认定已具备实机部署条件。

本轮修改范围（相对于仓库根目录）：

- 力控节点：`src/control/whole_body_force_control/src/node.hpp`、`node.cpp`、`node_config.cpp`、`node_ros_io.cpp`。
- 配置：`src/control/whole_body_force_control/config/force_follow_real.yaml`、`force_follow_sim.yaml`；`src/robotics/tracer_jaka_description/config/ros2_controllers.yaml`。
- 构建依赖：`src/control/whole_body_force_control/CMakeLists.txt`、`package.xml`。
- 力控入口：`src/bringup/launch/real/whole_body_force_control_real.launch.py`；`src/bringup/launch/sim/whole_body_force_control_sim.launch.py`、`force_control_mujoco_test.launch.py`、`force_control_20s_follow_test.launch.py`、`force_control_infinite_follow_test.launch.py`。
- 假力输入与回归：`src/control/whole_body_force_control/scripts/whole_body_force_control_test.py`、`test/test_force_control.cpp`；`src/bringup/test/test_force_control_configs.py`。
- 说明：`docs/whole_body_force_control_real_deployment.md`、`docs/frame_contract.md`。

## 0. CURRENT：唯一的两个力控配置

| 文件（位于 `src/control/whole_body_force_control/config/`） | 用途 |
|---|---|
| `force_follow_sim.yaml` | 开阔 MuJoCo 仿真，带符号 Fx 持续跟随，撤力后停止；不可用于实机 |
| `force_follow_real.yaml` | 首轮实机传感器 Fz 单轴导纳柔顺，底盘分担为零，armed 与参考输出默认关闭 |

仿真启动入口的 `profile=infinite` / `20s` 共用同一 YAML。
20 秒有限行程回归仅覆盖 `force_velocity_mode=false`、`force_deadband=0`、
`max_offset=5.20`、`max_base_delta=5.10`，不再复制整份参数文件。
其他试验轴或 6D 模式需人工审查后修改对应的唯一配置；不保留第三份示例配置。

历史验证边界（2026-09-16 配置清理）：当时检查了 YAML 有效参数与清理前默认仿真、20 秒仿真、
Z 向实机档位一致，并执行定向构建与不启动节点的配置/启动门测试。
没有运行 MuJoCo 动态回归，也没有连接或执行实机。
工具/传感器坐标、符号、清零、重力影响、安全限幅、急停、看门狗和故障恢复均须人工逐条确认。

删除的五份配置及清理前两个主配置已备份到
`/tmp/wbmm_force_config_backup.teb8ta/config/`，安装目录旧链接保存在同一备份目录的
`installed_links/` 中；临时目录不是长期归档。

---

## 1. 当前测试目标

本轮实机测试只验证下面这条最小链路：

```text
力传感器清零
    ↓
读取 jk_se_vi_200_link/Fz
    ↓
whole_body_force_control 在传感器原点生成局部 Z 轴导纳参考
    ↓
OCS2 MPC / MRT
    ↓
机械臂 + 差分底盘执行
```

本轮明确**不做**：

- 不实现 FTS 重力补偿；
- 不实现力传感器到其他坐标系的完整 wrench 变换；
- 不做接触力恒力控制；
- 不做动态环境避障；
- 不做高速、大行程实机验证。

本轮只做：

1. 确认 FTS 在无外力时接近零；
2. 确认 `jk_se_vi_200_link/Fz` 的符号方向；
3. 用很小的正/负 `Fz` 产生沿传感器 Z 轴的受限导纳参考；
4. 确认 MPC/MRT 能把参考转成机械臂位置命令；
5. 确认没有意外底盘平面运动；
6. 确认力信号超时、FTS stale、目标发布者冲突时能进入故障保持。

> 注意：传感器 Z 轴不一定是世界竖直轴，其方向随传感器姿态变化。本轮在捕获的名义传感器局部系定义柔顺偏移，并保持名义传感器姿态。差速底盘只能在平面内运动；本轮保留 `base_share=0` 和 `max_base_delta=0`，不分配底盘参考位移。它们不是底盘硬件隔离开关。

---

## 2. 当前已实现的安全门

实机入口是：

```bash
ros2 launch tracer_jaka_bringup whole_body_force_control_real.launch.py
```

默认参数：

| 参数 | 默认值 | 作用 |
|---|---|---|
| `hardware_write` | `false` | JAKA 只读，不使能 servo，不发关节命令 |
| `force_reference_output_enabled` | `false` | 力控节点不发布 OCS2 参考 |
| `force_control_armed` | `false` | 力控节点不进入 ACTIVE |

只有以下条件同时满足，力控参考输出才允许打开：

```text
force_reference_output_enabled=true
hardware_write=true
```

当前已经存在的故障检测：

- force/wrench 超时；
- OCS2 observation 超时；
- hard wrench limit；
- 目标发布者冲突；
- 故障锁存后撤销 `armed` 并发布 hold 参考；
- MRT policy 过期/无效时停止底盘并保持机械臂；
- `ForceProcessor` 有限性检查、硬 wrench 限幅和 wrench 变化率限制；
- JAKA `read()` 失败/FTS stale 检测已从硬件接口移除，当前仍由力控节点的 `force_timeout` 负责兜底。

---

## 3. 测试前准备

### 3.1 机械与场地

- 保证物理急停可达。
- 首轮测试必须让轮子离地，或者把底盘可靠固定，防止受力后底盘被反推。
- 机械臂运动范围内不得有人、线缆或易碎物。
- 工具端不要挂载未知负载。
- 如果需要安装工具，先在完全断电状态下完成，并记录工具重量。
- Z 方向测试前先确认机械臂不会碰到地面、桌面或自身限位。

### 3.2 软件构建

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  wbmm_core wbmm_pinocchio wbmm_ros_interfaces \
  whole_body_force_control wbmm_ocs2 wbmm_ocs2_ros \
  tracer_base jaka_hardware_interface tracer_jaka_description tracer_jaka_bringup
source install/setup.bash
```

### 3.3 网络与 CAN

- JAKA `robot_ip` 与 `local_ip` 必须与实际网络一致。
- `can_port` 必须与实际底盘 CAN 设备一致。
- 确认没有其他进程占用 CAN。
- 确认 `tracer_base_node` 能正常读取 `/odom`。
- 确认 JAKA EDG 数据能正常读取。
- 确认 `/fts_broadcaster/wrench` 有数据。

检查命令示例：

```bash
ros2 topic hz /odom
ros2 topic hz /joint_states
ros2 topic hz /fts_broadcaster/wrench
ros2 topic echo /fts_broadcaster/wrench --once
```

### 3.4 力传感器清零与处理

当前 `JakaHardwareInterface` 只透传原始 `edg_state_.torqSensor`。

`whole_body_force_control` 内部的 `ForceProcessor` 现在负责：

1. **启动自动 tare**
   - 节点启动后收集前 `tare_samples` 帧原始 FTS；
   - 计算每轴平均零偏；
   - 后续原始数据先扣除该零偏。

2. **传感器原点与坐标系**
   - `sensor_frame=jk_se_vi_200_link`，替代力控节点原来的 `ee_frame` 参数；
   - 输入 wrench 必须表达在该传感器 link，力矩也以传感器原点为参考；
   - 删除力控节点的 sensor → TCP/tool0 变换，不再平移力矩参考点；
   - frame 不匹配时拒绝数据，armed 时锁存 `WRENCH_FRAME`；
   - 传感器到状态系的位姿和 Jacobian 由现有 URDF/FK 获取，与该 link 的 TF 定义一致；
   - 通用 `transformWrench()` 和 `ForceProcessor` 的显式换系能力保留，供传感器与其他 link 的独立调用使用，当前节点调用单位旋转、零平移。

3. **滤波**
   - 按 `filter_alpha` / `filter_alpha_6d` 做一阶低通；
   - 控制器内部不再做二次滤波。

4. **安全检查**
   - 有限性检查；
   - `hard_force_norm_limit`：在 tare/滤波前检查原始 `Fx/Fy/Fz` 范数，超过阈值立即故障锁存并 hold；
   - `hard_wrench_limit` 逐轴硬限幅；
   - `max_wrench_rate` 变化率限制；
   - 超限时进入故障锁存并 hold；
   - whole-body IK 不可达时导纳积分器做 anti-windup，避免释放外力后出现隐藏参考跳变。

实机前检查：

```bash
ros2 topic echo /fts_broadcaster/wrench
```

- 自动 tare 期间工具端必须无外力；
- 必须确认 FTS frame 为 `jk_se_vi_200_link`，其 TF 原点和轴方向准确；
- 确认 `filter_alpha`、`hard_force_norm_limit`、`hard_wrench_limit`、`max_wrench_rate` 已按测试配置设置。

---

## 4. 阶段 A：只读影子验证

目标：验证话题、TF、观测和命令所有权，不产生任何运动。

```bash
ros2 launch tracer_jaka_bringup whole_body_force_control_real.launch.py
```

该启动默认：

```text
hardware_write=false
force_reference_output_enabled=false
force_control_armed=false
```

检查项：

1. `/cmd_vel` 和 `/arm_controller/commands` 不应有本系统创建的发布者；
2. `/mobile_manipulator_mpc_target` 不应有意外发布者；
3. `/fts_broadcaster/wrench` 稳定输出；
4. `/mobile_manipulator_mpc_observation` 稳定输出；
5. `/whole_body_force_control/control_state` 对应为 `DISABLED`；
6. TF `odom -> base_footprint -> jk_se_vi_200_link` 正常；
7. RViz 中机器人状态与实际一致。

建议同时运行：

```bash
ros2 run tracer_jaka_bringup readiness_check.py \
  --ros-args \
  -p expected_target_publishers:=0 \
  -p audit_duration:=5.0
```

如果这里发现重复 TF、目标发布者冲突或命令发布者异常，必须停机排查。

---

## 5. 阶段 B：Z 方向力信号与符号标定

目标：在不进入 ACTIVE 的情况下，确认 `jk_se_vi_200_link/Fz` 的方向和幅值。

使用唯一的实机配置（现已包含 Z 测试设置）：

```text
src/control/whole_body_force_control/config/force_follow_real.yaml
```

该配置的特点：

- `control_mode: admittance`
- `sensor_frame: jk_se_vi_200_link`
- `admittance_axes: [fz]`
- `mass: 3.0`、`damping: 45.0`、`stiffness: 400.0`
- `force_axis: z`
- `absolute_force: false`
- `response_body_x: 0`
- `response_body_y: 0`
- `response_body_z: 1`
- `base_share: 0`
- `max_base_delta: 0`
- `max_joint_delta: 0.05`
- `max_offset: 0.02`
- `max_velocity: 0.01`

仍然保持只读，不打开参考输出：

```bash
ros2 launch tracer_jaka_bringup whole_body_force_control_real.launch.py \
  force_params_file:=$(ros2 pkg prefix whole_body_force_control)/share/whole_body_force_control/config/force_follow_real.yaml
```

检查 `jk_se_vi_200_link/Fz`：

1. 不施加外力，记录基准值；
2. 沿工具 Z 正方向轻微推/拉，记录力的符号；
3. 沿工具 Z 负方向轻微推/拉，记录力的符号；
4. 确认正负号和幅值符合预期；
5. 确认松手后回零。

记录内容：

| 测试动作 | `/fts_broadcaster/wrench` Fz | 预期方向 |
|---|---:|---|
| 无外力 | 约 0 | 基准 |
| +Z 方向轻推 | 正或负 | 记录实际符号 |
| -Z 方向轻推 | 与上面相反 | 记录实际符号 |

如果符号与预期相反，只修改测试配置里的 `response_body_z` 或 FTS 标定，不要直接进入 ACTIVE。

---

## 6. 阶段 C：旧“只打开参考输出”阶段已废弃

旧阶段依赖 `command_output_enabled` / `safety_release` 来关闭 JAKA 写入但打开力控参考。
当前接口只有 `hardware_write`：
- `hardware_write=false`：JAKA 只读，力控参考输出被拒绝；
- `hardware_write=true`：JAKA 可写，OCS2/MRT 命令输出同时使能。

因此不存在“只打开参考输出、不打开 JAKA 写”的中间态。需要重新人工设计力控调试阶段；
不要直接执行历史阶段 C 命令。

## 7. 阶段 D：传感器 Z 轴导纳实机验收（待人工审查）

这是第一次允许 JAKA 写命令和 MRT 发命令。必须同时满足：

```text
hardware_write=true
force_reference_output_enabled=true
force_control_armed=true
```

命令示例：

```bash
ros2 launch tracer_jaka_bringup whole_body_force_control_real.launch.py \
  hardware_write:=true \
  force_reference_output_enabled:=true \
  force_control_armed:=true \
  force_params_file:=$(ros2 pkg prefix whole_body_force_control)/share/whole_body_force_control/config/force_follow_real.yaml
```

首次测试建议：

- 机械臂负载尽量小；
- 施加的 Fz 从小到大：例如 `0.5 N -> 1 N -> 2 N`；
- 每次只保持很短时间；
- 一旦出现异常立即按物理急停；
- 不要连续施加力不放。

验收标准：

| 项目 | 通过标准 |
|---|---|
| 无外力时 | 不产生持续运动 |
| 正 Fz | 沿名义传感器 +Z 产生受限导纳偏移 |
| 负 Fz | 方向相反 |
| 松手 | 参考回到名义位置附近，不应持续漂移 |
| 最大位移 | 不超过 `max_offset` 附近 |
| 底盘 | 不应出现 X/Y 平面运动，因为 Z 测试配置 `max_base_delta=0` |
| 关节 | 不应出现突变、抖动或撞限位 |
| 控制状态 | 正常时 `ACTIVE`，故障时 `FAULT_*` |
| 力超时 | FTS 数据停止后应进入故障并保持 |

停止方法：

1. 松开力；
2. 按物理急停；
3. 如果只是正常退出，先取消力控参考或停止 launch；
4. 不要仅依赖软件按钮。

---

## 8. 当前还没有实现的安全项

下面这些是 **未来实机部署前必须补做** 的内容，当前测试版不做：

### 8.1 独立急停 / 看门狗 / 命令超时

- MRT 进程异常退出时，自动发送零速命令或触发硬件急停；
- 底盘必须有明确的 `/cmd_vel` 超时保护；
- JAKA 必须实现通信丢失后的明确安全停止；
- 急停链路不能只依赖 ROS topic 或 launch 参数。

### 8.2 硬限幅

当前 OCS2 的关节位置/速度限制主要是软约束，最终命令层还需要补充：

- 底盘线速度硬限幅；
- 底盘角速度硬限幅；
- 关节位置硬限幅；
- 关节速度硬限幅；
- 单步角度 delta 与真实关节速度的换算和限速验证；
- JAKA 控制器内部 `edg_servo_j` 的速度参数语义确认。

### 8.3 重力补偿

- 本轮不做 FTS 重力补偿；
- 本轮已删除 sensor → TCP/tool0 的 wrench 变换；
- 本轮只用 `sensor_frame` 下的 `Fz` 做传感器 Z 轴导纳测试；
- 后续要做重力补偿时，建议放在 `ForceProcessor` 中：
  `raw sensor wrench -> tare -> gravity compensation -> filter -> safety -> controller`；
- 重力补偿需要工具质量、质心、传感器姿态和外力方向，不能只靠 FTS 自身。

### 8.4 其他

- 动态 enable/disable 服务；
- 运行时故障恢复流程；
- 与硬件急停/安全 PLC 的联动；
- 真实避障和动态环境；
- 多传感器一致性检查；
- 长时间老化测试。

---

## 9. 故障排查清单

| 现象 | 优先检查 |
|---|---|
| `/fts_broadcaster/wrench` 无数据 | FTS broadcaster 是否加载；JAKA hardware 是否 activate；EDG 是否通信 |
| FTS 一直为零 | 零偏采样是否在外力下执行；工具是否安装；deadband 是否过大 |
| FTS 一直不变 | JAKA `read()` 是否失败；FTS stale 检测是否置 NaN；`latchFault` 是否触发 |
| 控制状态一直 WAITING | observation 或 wrench 是否超时；MPC/MRT 是否已启动 |
| 控制状态 FAULT | 查看 `/whole_body_force_control/control_state` 和节点日志 |
| `/cmd_vel` 无输出 | `hardware_write` 是否为 `true`；OCS2 是否已启动 |
| JAKA 不动 | `hardware_write` 是否仍为 true；`arm_controller` 是否 spawn；`edg_servo_j` 是否报错 |
| 机器人意外运动 | 立即急停；检查目标发布者数量、FTS 零点、`response_body_z` 符号 |
| FTS 超时后仍运动 | 立即急停；确认力控节点是否收到 NaN/无效数据并进入故障 |

---

## 10. 本轮测试完成标准

只有以下条件全部满足，才能认为是“首轮 Z 方向测试通过”：

- [ ] 只读影子模式无异常；
- [ ] 无外力时 FTS 零点可接受；
- [ ] `jk_se_vi_200_link/Fz` 正负方向明确；
- [ ] Z 方向参考方向与施加力方向一致；
- [ ] 松手后参考回归；
- [ ] 底盘没有意外 X/Y 运动；
- [ ] 关节没有突变、撞限位或持续抖动；
- [ ] FTS 超时/stale 能触发故障；
- [ ] 目标发布者唯一；
- [ ] 物理急停经过实际验证。

---

## 11. 当前结论

当前仓库已经具备：

- 原始 FTS 读取/透传；
- 启动自动 tare；
- 传感器 frame 校验及传感器 link 的运动学参考；
- `ForceProcessor` 低通滤波；
- 有限性检查和硬 wrench 限幅；
- wrench 变化率限制；
- whole-body reference 发布；
- MPC/MRT 实机链路；
- 基础软件安全门和故障锁存。

当前仓库还**不具备**：

- 重力补偿；
- 接触力恒力控制；
- 完整硬件急停/看门狗联动；
- 输出层底盘/关节硬限幅；
- FTS stale 的硬件级检测。

但当前仍**不具备无人监督实机部署条件**。  
本轮只适合：

- 轮子离地或底盘可靠固定；
- 有人现场监督；
- 物理急停可达；
- 小力、低速、短时间；
- 先 Z 轴、先单方向、先验证再扩展。
