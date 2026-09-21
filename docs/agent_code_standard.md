# Agent 代码编写与项目可掌控性标准

> Status: ACTIVE
> Author: Agent
> Reviewer: TBD
> Reviewed at: TBD
> Review Level: L1
> Warning: 本文档尚未经过人工审查，不能作为实现依据。

本文件规定 Agent 修改代码时必须遵守的最小规则。核心目标：让人能够理解项目入口、模块职责、数据流、参数归属和 sim/real 边界。

# 1. 最高优先级

```
1. 人能理解
2. 模块职责清楚
3. 数据流清楚
4. 方便替换算法
5. 方便仿真和实机部署
6. 容易测试
7. 最后才是减少编码工作
```

禁止为了“架构优雅”引入无必要的 manager、factory、registry、plugin、DI、多层 wrapper。

# 2. 修改前必须先写边界

每次编码前必须说明：

```
任务：
预计修改：
不修改：
输入 / 输出：
测试方式：
```

不得顺手重构无关模块。

# 3. 文件与目录职责

- 每个文件能用一句话解释职责。
- 每个一级目录必须有明确职责。
- 新增、删除、移动重要目录或入口时，必须同步 `docs/project_map.md`。
- 优先保持简单结构；能用一个模块解决的问题，不拆成三层抽象。

# 4. 模块边界

必须区分：

```
Algorithm / Core
Task
Adapter
Utils
App / Launch
```

规则：

- Core 保持小，只放公共数据结构和最小契约。
- Algorithm 只解决算法问题。
- Adapter 只做接口转换，必须薄。
- Task 组合算法和 Adapter，不写底层通信细节。
- Utils 只放通用工具，不成为垃圾目录。
- 不为未来假想需求提前抽象；第二次真实出现后再抽接口。
- 不修改 `vendor/`，除非用户明确要求并单独审查。

# 5. 数据流与程序入口

每个应用必须能回答：

```
从哪里启动？
工作目录是什么？
输入来自哪里？
经过哪些模块？
输出发布到哪里？
Planner / Optimizer / Controller 在哪里？
仿真和实机边界在哪里？
```

核心数据流必须能从入口追踪到最终输出。

# 6. 配置与参数归属

每个参数必须只有一个所有者。

## 6.1 通用算法与接口参数

通用算法结构、topic、frame、joint 名和接口默认值统一放：

```text
src/bringup/config/common/
  interface.yaml
  ocs2.yaml
  force_control.yaml
  ekf.yaml
  slam_toolbox.yaml
  remani.yaml
  moveit_bringup.yaml
```

## 6.2 实机/仿真差异参数

实机差异放 `config/real/`，仿真差异放 `config/sim/`。目录名已经表达 backend，
因此文件名不再重复 `_real` / `_sim` 后缀：

```text
src/bringup/config/sim/
  ocs2.yaml
  force_control.yaml
  ekf.yaml
  slam_toolbox.yaml
  remani.yaml
  task.info

src/bringup/config/real/
  ocs2.yaml
  force_control.yaml
  ekf.yaml
  slam_toolbox.yaml
  remani.yaml
  task.info
  d455_esdf_record_qos.yaml
```

适用于：

- 自研 OCS2 / 力控的实机部署 profile
- `robot_localization`
- `slam_toolbox`
- MoveIt / MoveIt Servo
- REMANI
- RealSense bag QoS

## 6.3 自研包与第三方 wrapper 的边界

- OCS2 配置只放 MPC/MRT、话题、frame、关节名、位置跟踪和命令限幅。
- OCS2 配置不得混入力控轴、刚度、阻尼、wrench、admittance、offset 等力控参数。
- 力控仿真默认参数归 `whole_body_force_control/config/`；实机部署 profile 归 `src/bringup/config/real/force_control.yaml`。
- 不得在多个包复制同一份参数文件。
- 不得让 launch 和 YAML 同时控制同一个数值。

## 6.4 launch 与 YAML 的边界

launch 只保留：

- 启动哪些功能；
- sim / real 选择；
- 是否可视化；
- 文件路径；
- 话题覆盖；
- 安全开关；
- 启动时序；
- 命令接口所有者。

YAML 承担：

- 控制器增益；
- MPC / planner 数值参数；
- EKF / SLAM / MoveIt / REMANI 数值参数；
- 力控轴、刚度、阻尼、限幅、滤波、超时。

# 7. Bringup Launch 协议

## 7.1 目录扁平化

所有核心 launch 文件直接放在：

```text
src/bringup/launch/
  <name>.launch.py
```

安装后对应：

```text
share/tracer_jaka_bringup/launch/<name>.launch.py
```

不再使用 `launch/common/`、`launch/real/`、`launch/sim/` 分层，也不在文件名中使用
`_real` / `_sim` 后缀。参数文件仍可放在 `config/common/`、`config/real/`、
`config/sim/`。

## 7.2 backend launch 协议

只有两个 launch 可以启动硬件或仿真：

```text
src/bringup/launch/wbmm_hardware_interface.launch.py
src/bringup/launch/mujoco_hardware_interface.launch.py
```

- 二者都实现 [`config/common/interface.yaml`](../src/bringup/config/common/interface.yaml)。
- 实机接口负责 Tracer、JAKA、IMU、LiDAR、FTS 和 `robot_state_publisher`。
- 仿真接口负责 MuJoCo bridge、`/clock`、仿真传感器和 `robot_state_publisher`。
- 其他核心 launch 默认不得启动二者。
- `d435_camera.launch.py` / `d455_camera.launch.py` 是可选相机传感器入口，
  只启动相机驱动，不启动机器人、`controller_manager` 或算法，不属于核心 backend 选择。

## 7.3 算法 launch 协议

其余核心 launch：

```text
localization.launch.py
ocs2.launch.py
remani.launch.py
remani_mpc.launch.py
remani_mpc_localized.launch.py
whole_body_force_control.launch.py
whole_body_force_control_profiles.launch.py
moveit.launch.py
```

必须满足：

- 只启动算法或工具节点；
- 不启动硬件、MuJoCo、传感器驱动或 `controller_manager`；
- 不声明 `backend`；
- 使用 canonical topic、frame 和 action；
- 必要参数由调用者显式传入，缺参数必须 fail-closed。

## 7.4 可选总入口

`wbmm.launch.py` 是唯一允许按 `hardware_backend:=none|real|mujoco` 选择 backend 的
入口：

- 默认 `hardware_backend:=none`；
- 默认不启动任何算法；
- 选择 backend 时也只调用 7.2 的两个 backend launch 之一。

## 7.5 hardware interface 唯一所有权

- 一个 launch 树只允许一个 `controller_manager`。
- 唯一所有者：

```text
src/bringup/launch/wbmm_hardware_interface.launch.py
```

- 其他 launch 不得重复创建 `controller_manager`。
- 控制器加载顺序：

```text
joint_state_broadcaster -> optional arm_controller -> fts_broadcaster
```

## 7.6 命令接口所有权

同一时刻只能有一个机械臂命令所有者：

- OCS2 / REMANI：`arm_controller`
- MoveIt 轨迹执行：`arm_trajectory_controller`
- MoveIt Servo：`arm_controller`，同时关闭轨迹执行
- 不允许 MoveIt 轨迹执行和 Servo 同时控制机械臂

## 7.7 实机安全门

唯一用户级实机写入门：

```text
hardware_write=true  -> 允许 JAKA 写入
hardware_write=false -> JAKA 只读，不使能 servo，不输出命令
```

默认值：

```text
false
```

禁止重新引入：

```text
jaka_read_only
safety_release
allow_trajectory_execution
```

实现规则：

- `jaka_hardware_interface` 直接解析 `hardware_write`。
- 不允许在 xacro/launch 中反相生成 `hardware_write -> read_only`。
- MoveIt 节点需要的 `allow_trajectory_execution` 由 `hardware_write` 直接推导。
- OCS2 的 `command_output_enabled` 由部署层根据 `hardware_write` 推导。

# 8. 实机代码

- 仿真和实机入口必须显式区分。
- 实机默认 `hardware_write=false`。
- 安全、力控、实机执行相关改动必须人工逐条审查。
- 没有安全门、急停、看门狗、超时和故障恢复说明时，不得建议实机执行。
- 隐藏 fallback 必须显式记录，禁止“悄悄改用其他接口”。

# 9. 测试与验证

- 每个核心模块必须有最小行为测试。
- 测试应帮助人理解模块职责，而不是追求覆盖率数字。
- Launch 修改至少需要：
  - 全部 launch `--show-args` 解析；
  - 静态组合测试；
  - host safety gate 测试；
  - sim 入口最小启动验证。
- 修改后必须说明：
  - 改了哪些文件；
  - 为什么改；
  - 如何验证；
  - 是否影响实机行为。

# 10. 交付格式

```
任务类型：
修改文件：
是否修改源码：
验证方式：
不确定项：
人工审查结论：DRAFT / APPROVED / REJECTED
```

# 11. Agent 自检

- 是否严格围绕任务；
- 是否顺手重构；
- 是否新增不必要文件或抽象；
- 参数归属是否符合第 6 节；
- launch 分层是否符合第 7 节；
- 是否重复创建 `controller_manager`；
- 是否重新引入旧安全参数；
- 是否隐藏 fallback；
- 是否影响实机行为；
- 是否更新项目地图；
- 人是否能重新建立对项目的认知。

# 12. 最终原则

> 先契约，后代码。
> 先最小闭环，后抽象。
> 每个文件必须有明确职责。
> 每个核心数据流必须能从入口追踪到输出。
> Agent 的价值是减少重复编码，而不是代替人的系统设计能力。
