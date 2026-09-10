# Agent 输出文档标准

> Status: ACTIVE  
> Author: Agent  
> Reviewer: TBD  
> Reviewed at: TBD  
> Review Level: L1  
> Warning: 本文档尚未经过人工审查，不能作为实现依据。

> 本文件定义 Agent 在仓库中执行“文档任务”时必须遵守的规则。  
> 核心原则：**Agent 可以读取整个仓库并进行分析，但文档任务只能修改 `docs/`；任何 Agent 输出都必须经过人工完整审阅后才能作为实现依据。**

---

# 1. 核心原则

Agent 执行文档任务时必须遵守以下最高优先级规则：

1. Agent 可以读取源码、配置、测试、文档和仓库结构，但只能修改：

```text
docs/
docs/img/
```

2. Agent 不得在文档任务中修改源码、配置、测试或系统运行行为。

3. Agent 必须严格区分：

```text
CURRENT   当前已经存在的实现
PROPOSED  建议设计或未来方案
TBD       当前无法确认，需要人工确认
```

4. Agent 对当前实现做出的技术结论必须能够追溯到：

```text
源码
配置
已有测试
数学契约
已有文档
实验记录
```

5. Agent 生成的文档只能标记为：

```text
DRAFT
```

Agent 不得自我审阅、自我批准或将自己的输出标记为 `APPROVED`。

6. 文档提出源码修改需求时，只能记录修改建议。

具体实现必须作为独立 Implementation Task，由人确认后再执行。

推荐流程：

```text
Agent 阅读仓库
      |
      v
Agent 分析问题
      |
      v
生成 docs/ 下的 DRAFT
      |
      v
Human Review
      |
      +-- APPROVED
      |
      +-- CHANGES_REQUESTED
      |
      +-- REJECTED
      |
      v
Implementation Task
      |
      v
代码修改
      |
      v
Code Review
```

---

# 2. 适用范围

本标准适用于 Agent 新建或修改的：

- Markdown 文档；
- 数学契约；
- 坐标系契约；
- 接口说明；
- 软件架构方案；
- 算法说明；
- 任务定义；
- 轨迹定义；
- 全身规划说明；
- 轨迹优化说明；
- MPC 说明；
- 力控说明；
- 接触控制说明；
- 强化学习任务定义；
- 实验设计；
- 图表说明；
- 伪代码；
- 文档中的代码片段。

本标准中的“文档任务”不包括直接修改：

- C++ 源码；
- Python 源码；
- CMake；
- `package.xml`；
- launch 文件；
- YAML 配置；
- URDF / MJCF；
- ROS2 驱动；
- 控制器源码；
- 测试源码；
- vendor 代码。

---

# 3. Agent 权限边界

## 3.1 允许读取

Agent 为了正确理解当前系统，可以读取：

```text
src/
include/
tests/
test/
config/
launch/
simulation/
scripts/
docs/
CMakeLists.txt
package.xml
URDF
MJCF
YAML
git diff
git log
仓库目录结构
```

读取源码是允许的。

文档必须尽可能基于当前仓库真实实现，而不是脱离实现独立设计。

---

## 3.2 允许修改

Agent 在文档任务中只允许修改：

```text
docs/
docs/img/
```

图片统一存放于：

```text
docs/img/
```

---

## 3.3 禁止修改

Agent 不允许：

- 修改 `src/**`；
- 修改 `include/**`；
- 修改测试逻辑；
- 修改 CMake；
- 修改 `package.xml`；
- 修改 launch；
- 修改 YAML；
- 修改 URDF / MJCF；
- 修改机器人驱动；
- 修改控制器；
- 修改 vendor 代码；
- 修改运行参数；
- 为了让文档成立而调整现有实现；
- 偷偷改变系统行为。

---

# 4. 源码修改建议的表达方式

如果文档分析发现当前实现需要修改，只能记录建议，不能直接修改代码。

统一使用：

```text
建议修改：

- 文件：
- 当前行为：
- 存在问题：
- 建议行为：
- 影响范围：
- 风险：
- 需要人工确认：
```

例如：

```text
建议修改：

- 文件：src/control/admittance_controller.cpp
- 当前行为：导纳输出直接作为末端目标位姿
- 存在问题：尚未明确 MPC 与导纳的 reference ownership
- 建议行为：由导纳层只生成 corrected reference，最终控制输入由 MPC 输出
- 影响范围：接触控制、MPC reference manager、实机控制链
- 风险：可能改变现有控制器职责边界
- 需要人工确认：是
```

文档任务到此结束。

后续代码修改必须作为独立任务处理。

---

# 5. 文档状态

Agent 输出的文档必须使用以下状态之一：

```text
DRAFT
REVIEW_PENDING
CHANGES_REQUESTED
APPROVED
REJECTED
DEPRECATED
```

其中 Agent 创建的新文档默认必须是：

```text
Status: DRAFT
```

Agent 不允许将自己生成的文档设置为：

```text
APPROVED
```

只有人工审查通过后，才可以修改为：

```text
Status: APPROVED
Reviewer: <name>
Reviewed at: <date>
```

---

# 6. 当前实现、建议设计和未知内容

Agent 必须明确区分事实、设计和未知信息。

推荐使用以下标记。

## 6.1 CURRENT

表示当前代码、配置或已有实验中已经存在的行为。

例如：

```text
[CURRENT]

当前 RRT-Connect 在关节空间中进行采样。
依据：src/planning/rrt_connect.py
```

---

## 6.2 PROPOSED

表示 Agent 或设计者提出的未来方案。

例如：

```text
[PROPOSED]

建议将 TaskTrajectory 与 WholeBodyTrajectory 分离，
避免任务空间参考和机器人实际状态轨迹混用。
```

---

## 6.3 TBD

表示当前信息不足，无法可靠判断。

例如：

```text
[TBD]

当前无法确认 OCS2 reference manager 是否已经承担
全身轨迹插值功能，需要人工结合实现进一步确认。
```

Agent 不得把 `PROPOSED` 或 `TBD` 写成已经实现的事实。

---

# 7. 证据和可追溯性

涉及当前实现、实验数据或系统行为时，应说明依据。

允许的依据包括：

- 源码文件；
- 类或函数；
- 配置项；
- 已有测试；
- 数学契约；
- 坐标系契约；
- 已批准设计文档；
- 实验日志；
- rosbag；
- benchmark；
- 实机记录。

推荐写法：

```text
依据：
- src/planning/rrt_connect.py
- docs/math_contract.md
- config/mpc.yaml
```

Agent 不得：

- 编造实验数据；
- 编造测试结果；
- 编造 benchmark；
- 编造机器人能力；
- 把未验证方案写成已验证；
- 把推测写成当前事实。

---

# 8. 不确定性规则

如果当前仓库信息不足，必须明确写：

```text
不确定，需要人工确认。
```

不能通过经验猜测后写成确定结论。

可以进行合理分析，但必须写明：

```text
推测：
原因：
需要确认：
```

例如：

```text
推测：

当前 TaskTrajectory 可能只描述任务空间目标，
WholeBodyTrajectory 负责机器人实际可执行状态。

原因：
现有接口中两者承担的数据维度不同。

需要确认：
检查 planner 输出和 MPC reference manager 的实际输入。
```

---

# 9. 文档内容要求

## 9.1 通俗易懂

文档必须：

- 使用移动机械臂领域常用术语；
- 使用规划、轨迹优化、MPC、力控领域常用术语；
- 避免为了“专业”而增加无意义抽象；
- 避免只罗列类名和接口名；
- 每个重要模块必须解释“为什么存在”；
- 每个算法必须解释解决什么问题；
- 每个公式必须解释物理意义；
- 每个接口必须解释输入和输出；
- 尽可能结合当前机器人系统给出例子。

复杂框架本身不等于技术深度。

---

# 10. 移动机械臂文档要求

涉及移动机械臂时，应根据文档主题明确以下内容。

## 10.1 机器人模型

说明：

- 底盘类型：差速 / 全向 / 固定；
- 机械臂自由度；
- 末端执行器；
- 是否包含夹爪；
- 是否包含移动底盘动力学。

---

## 10.2 状态和输入

不得假设整个项目只有一种统一状态定义。

每个规划器或控制器应明确自己的模型。

例如运动学模型可以定义：

$$
x =
[x_b,\ y_b,\ \psi_b,\ q_1,\dots,q_6]^T
$$

其中：

- $x_b, y_b$：底盘位置，单位 m；
- $\psi_b$：底盘航向角，单位 rad；
- $q_i$：机械臂关节角，单位 rad。

动力学或 MPC 模型可能需要增加速度：

$$
x =
[x_b,\ y_b,\ \psi_b,\ q,\ v_b,\ \omega_b,\ \dot q]^T
$$

必须说明：

```text
Model:
State x:
Input u:
State dimension:
Input dimension:
Units:
```

---

## 10.3 非完整约束

对于差速底盘，应明确其非完整约束。

典型形式：

$$
\dot y_b \cos\psi_b -
\dot x_b \sin\psi_b = 0
$$

必须说明该约束属于：

- 模型约束；
- 优化约束；
- 控制约束；

或者是否被运动学参数化隐式满足。

---

## 10.4 关节和运动约束

至少说明：

- 关节位置限位；
- 关节速度限位；
- 关节加速度限位；
- 底盘速度限制；
- 底盘角速度限制；
- 碰撞约束；
- 自碰撞；
- 环境碰撞。

---

## 10.5 导航与操作耦合

必须说明：

```text
导航解决什么问题？
机械臂规划解决什么问题？
什么时候需要联合规划？
底盘和机械臂之间如何耦合？
任务空间约束如何影响底盘运动？
```

不得简单将：

```text
底盘轨迹 + 机械臂轨迹
```

称为“全身规划”，除非两者在同一优化或约束体系中存在实际耦合。

---

# 11. 数学公式标准

## 11.1 Markdown 格式

行内公式：

```markdown
$x$
```

独立公式：

```markdown
$$
x = f(q)
$$
```

必须保证 Typora 可以正常显示。

---

## 11.2 符号一致性

数学符号优先遵守：

```text
docs/math_contract.md
```

如果当前文档发现源码和数学契约不一致，禁止静默选择其中一个。

必须写：

```text
发现不一致：

- math_contract.md：
- 当前源码：
- 影响：
- 需要人工确认：
```

---

## 11.3 公式说明

每个重要公式必须说明：

- 变量含义；
- 向量维度；
- 坐标系；
- 单位；
- 物理意义。

例如：

$$
F_e = F_d - F_m
$$

其中：

- $F_d$：期望接触力，单位 N；
- $F_m$：测量接触力，单位 N；
- $F_e$：力误差，单位 N；
- 所有力均表示在 `task_frame`。

---

# 12. 坐标系要求

涉及以下量时必须说明坐标系：

- position；
- orientation；
- twist；
- acceleration；
- wrench；
- Jacobian；
- force；
- torque。

建议项目单独维护：

```text
docs/frame_contract.md
```

至少统一以下 frame：

```text
world
map
odom
base_footprint
base_link
arm_base
ee_link
tool_frame
task_frame
ft_sensor_frame
camera_frame
```

必须明确：

```text
T_A_B
```

究竟表示：

> frame B 在 frame A 中的位姿

还是其他定义。

不得出现同一项目中变换方向含义不一致的情况。

---

## 12.1 四元数

ROS 常见四元数顺序：

```text
xyzw
```

内部数学库可能使用：

```text
wxyz
```

文档必须明确两者转换关系。

---

# 13. 轨迹定义要求

涉及轨迹时必须区分：

## 13.1 TaskTrajectory

描述“任务希望机器人完成什么”。

可能包含：

```text
时间
末端位置
末端姿态
接触法向
切向方向
期望 wrench
contact flag
task phase
```

---

## 13.2 WholeBodyTrajectory

描述“机器人实际应该如何运动”。

可能包含：

```text
底盘状态
机械臂关节状态
速度
控制输入
时间戳
```

Agent 不得默认两者是同一个概念。

---

## 13.3 轨迹连续性

必须根据控制器实际需求说明：

- 位置连续；
- 速度连续；
- 加速度连续；
- 是否需要 jerk 连续。

例如：

```text
RRT 路径：
通常只有离散 waypoint。

轨迹优化后：
可要求速度或加速度更平滑。

MPC reference：
需要保证时间参数和插值定义明确。
```

---

# 14. 全身力控要求

涉及力控时至少明确以下内容。

## 14.1 Wrench 定义

定义：

$$
w =
[F_x,F_y,F_z,\tau_x,\tau_y,\tau_z]^T
$$

并说明：

- 力单位 N；
- 力矩单位 N·m；
- wrench 所在 frame。

---

## 14.2 期望和测量值

必须明确：

```text
desired wrench
measured wrench
external wrench
filtered wrench
bias-compensated wrench
```

不能混用。

---

## 14.3 导纳和阻抗

涉及导纳控制时必须说明：

```text
force -> motion
```

例如：

$$
M_d \Delta \ddot x +
D_d \Delta \dot x +
K_d \Delta x
=
F_{\mathrm{ext}} - F_d
$$

涉及阻抗控制时必须说明：

```text
motion error -> force / torque
```

不得只写公式而不说明控制方向。

---

## 14.4 接触方向

必须定义：

- 接触法向；
- 接触切向；
- 接触平面；
- 力控制轴；
- 位置控制轴。

例如：

```text
normal axis:
force controlled

tangent x:
position controlled

tangent y:
position controlled
```

---

## 14.5 接触状态

至少考虑：

```text
FREE_SPACE
APPROACH
CONTACT
INSERTION
CONTACT_LOST
SAFE_HOLD
FAULT
```

具体状态可以根据项目简化，但必须定义异常情况如何处理。

---

## 14.6 力安全

至少说明：

- 最大允许力；
- 最大允许力矩；
- 力变化率限制；
- 接触丢失；
- 传感器异常；
- 通信超时；
- 安全停止。

Agent 不得将未经验证的力阈值写成实机安全参数。

---

# 15. 控制链所有权

涉及规划、导纳、MPC、RL 或硬件控制时，必须明确控制权。

任意时刻最终机器人 command 应有唯一 owner。

文档必须说明：

```text
Nominal Reference Owner:
Reference Correction Owner:
Final Command Owner:
Safety Override Owner:
```

推荐的数据流形式：

```text
Task Planner
     |
     v
Nominal Reference
     |
     v
Admittance / Contact Correction
     |
     v
Corrected Reference
     |
     v
MPC / Tracking Controller
     |
     v
Final Robot Command
     |
     v
Hardware
```

不得设计成：

```text
MPC --------> q_cmd
Admittance --> q_cmd
RL ---------> q_cmd
```

多个模块同时直接控制同一个 actuator command，除非存在明确仲裁器。

---

# 16. OCS2 / MPC 文档要求

涉及 OCS2 或 MPC 时必须说明：

- MPC 使用的状态 $x$；
- MPC 输入 $u$；
- dynamics；
- cost；
- constraints；
- reference trajectory；
- prediction horizon；
- execution horizon；
- solver frequency；
- command output；
- 与 planner 的职责边界。

应明确：

```text
Planner：
负责产生可执行任务参考或名义轨迹。

MPC：
负责根据当前状态在线跟踪并满足局部约束。

MPC 不自动替代全局规划器。
```

如果当前系统实现不同，应以 `[CURRENT]` 形式记录真实行为。

---

# 17. 强化学习文档要求

涉及机械臂或移动机械臂强化学习时必须明确：

```text
Observation
Action
Reward
Termination
Reset
Control frequency
Action scaling
Action frame
Safety limits
Policy role
Traditional controller role
```

---

## 17.1 Observation

必须区分：

```text
Real-world observable
Simulation observable
Privileged information
```

原则：

> 不得将实机无法直接获得的信息默认作为部署 policy 的 observation。

例如 MuJoCo 可以直接提供：

```text
contact pair
exact object pose
exact penetration depth
exact contact force
```

如果实机没有对应感知手段，则必须标记为：

```text
simulation privileged information
```

---

## 17.2 Action

必须明确 RL 输出是什么：

```text
joint position
joint velocity
joint torque
Cartesian delta pose
Cartesian velocity
desired force
admittance parameter
MPC reference correction
```

必须说明单位、范围和频率。

---

## 17.3 RL 与传统控制

必须明确：

```text
RL 决定什么？
Admittance 决定什么？
MPC 决定什么？
Safety Supervisor 决定什么？
```

不得仅写：

```text
使用 RL 实现力控插入
```

而没有描述控制层级。

---

# 18. 安全相关要求

涉及实机、接触、插入、碰撞、MPC 或 RL 时默认视为安全相关文档。

必须说明至少：

- emergency stop；
- watchdog；
- communication timeout；
- sensor invalid；
- force threshold；
- joint limit；
- velocity limit；
- collision detection；
- command timeout；
- SAFE_HOLD；
- FAULT；
- 恢复条件。

Agent 不得自行给出实机安全阈值并声称其安全。

未知阈值必须标记：

```text
TBD，需要人工实验确定。
```

---

# 19. 测试和验证规则

## 19.1 默认行为

Agent 执行普通文档任务时不得为了“证明自己正确”而：

- 修改测试；
- 修改源码；
- 修改配置；
- 修改仿真场景；
- 自动补测试使方案通过。

---

## 19.2 允许的验证

如果任务明确允许验证当前实现，Agent 可以执行已有的只读检查，例如：

```text
运行现有 unit test
运行现有 benchmark
读取已有日志
读取已有 rosbag
检查现有配置
```

前提：

- 不修改源码；
- 不修改测试；
- 不修改运行行为；
- 不把测试通过等价为人工审查通过。

文档中应记录：

```text
验证命令：
验证版本：
观察结果：
验证范围：
未覆盖内容：
```

测试结果只能描述：

```text
在当前版本和当前测试条件下观察到……
```

不得泛化为：

```text
系统已经完全正确。
```

---

# 20. 图片要求

图片只能放在：

```text
docs/img/
```

文件名使用：

```text
lowercase
数字
下划线
```

例如：

```text
whole_body_pipeline.svg
force_control_architecture.svg
frame_tree.svg
```

优先使用 SVG。

Markdown 使用相对路径：

```markdown
![全身控制数据流](img/whole_body_pipeline.svg)
```

禁止将图片放在：

```text
仓库根目录
src/
include/
build/
```

---

# 21. 推荐文档结构

根据主题可以删减无关章节，但不得机械套模板。

推荐：

```markdown
# 标题

> Status: DRAFT
> Author: Agent
> Reviewer: TBD
> Reviewed at: TBD
> Review Level: L1
> Warning: 本文档尚未经过人工审查，不能作为实现依据。

## 1. 背景与目标

## 2. 当前实现

## 3. 问题定义

## 4. 术语与符号

## 5. 输入与输出

## 6. 坐标系

## 7. 数学模型

## 8. 轨迹 / 任务定义

## 9. 算法或控制结构

## 10. 接口与数据流

## 11. 安全与边界

## 12. Proposed Changes

## 13. 待确认问题

## 14. 参考和依据
```

---

# 22. 人工审查等级

文档根据风险划分为：

| Level | 内容                                        | 最低要求                     |
| ----- | ------------------------------------------- | ---------------------------- |
| L0    | 背景、术语、普通说明                        | 1 人完整审阅                 |
| L1    | 架构、接口、任务、轨迹、数学契约、规划、MPC | 1 人逐项审阅                 |
| L2    | 力控、接触、RL 实机控制、安全限幅           | 1 人逐项审阅，推荐第二人复核 |
| L3    | 安全关键控制链、硬件执行链、急停逻辑        | 必须双人复核，并完成独立验证 |

移动机械臂全身规划和 OCS2 文档默认至少：

```text
L1
```

涉及：

```text
contact
force
admittance
impedance
RL hardware deployment
safety limit
```

默认至少：

```text
L2
```

Agent 不得计入 Reviewer 数量。

---

# 23. 人工审查必须确认的内容

L1 及以上文档至少检查：

- [ ] CURRENT / PROPOSED / TBD 是否明确；
- [ ] 当前实现描述是否有依据；
- [ ] 数学模型是否正确；
- [ ] 状态和输入是否明确；
- [ ] 维度是否匹配；
- [ ] 单位是否明确；
- [ ] 坐标系是否明确；
- [ ] frame transform 是否一致；
- [ ] 轨迹定义是否正确；
- [ ] Planner 与 Controller 职责是否明确；
- [ ] command ownership 是否明确；
- [ ] 是否存在未声明的系统行为变化；
- [ ] 不确定内容是否正确标记；
- [ ] 与当前源码是否存在冲突。

L2 及以上还必须检查：

- [ ] wrench frame；
- [ ] force / torque limit；
- [ ] contact state；
- [ ] contact lost；
- [ ] SAFE_HOLD；
- [ ] FAULT；
- [ ] watchdog；
- [ ] emergency stop；
- [ ] 实机和仿真差异；
- [ ] RL privileged information；
- [ ] 安全参数是否经过人工确认。

---

# 24. Source of Truth

不同内容具有不同事实来源。

## 24.1 当前实现行为

以以下内容为主要依据：

```text
源码
配置
运行系统
已有测试
```

---

## 24.2 数学符号

以：

```text
docs/math_contract.md
```

为规范来源。

---

## 24.3 坐标系

如果存在：

```text
docs/frame_contract.md
```

则以其作为规范来源。

---

## 24.4 目标架构

以人工批准的：

```text
APPROVED design document
```

作为设计依据。

---

## 24.5 出现冲突时

如果源码、契约和文档发生冲突：

```text
禁止 Agent 自动选择一个覆盖另一个。
```

必须记录：

```text
Conflict:

- Source A:
- Source B:
- Difference:
- Possible impact:
- Need human decision:
```

---

# 25. 禁止事项

Agent 在文档任务中不得：

1. 修改源码；
2. 修改测试逻辑；
3. 修改 CMake；
4. 修改 `package.xml`；
5. 修改 launch；
6. 修改 YAML；
7. 修改 URDF / MJCF；
8. 修改 vendor；
9. 偷偷改变运行行为；
10. 为了让文档正确而修改实现；
11. 编造实验结果；
12. 编造 benchmark；
13. 编造测试结果；
14. 编造当前实现；
15. 把建议写成已实现；
16. 把推测写成事实；
17. 把复杂框架当作技术深度；
18. 自我审阅；
19. 自我批准；
20. 将未审阅文档标记为 `APPROVED`；
21. 使用实机不可获得的信息而不声明其为 privileged information；
22. 在多个控制器之间制造不明确的 command ownership；
23. 用模糊语言掩盖技术不确定性。

---

# 26. 交付前检查表

Agent 在完成文档前必须检查：

- [ ] 只修改了 `docs/` 或 `docs/img/`；
- [ ] 未修改源码；
- [ ] 未修改配置；
- [ ] 未修改测试逻辑；
- [ ] 文档状态为 `DRAFT`；
- [ ] CURRENT / PROPOSED / TBD 已区分；
- [ ] 当前事实具有可追溯依据；
- [ ] 不确定内容已标记；
- [ ] 公式使用 `$...$` 或 `$$...$$`；
- [ ] 公式可在 Typora 中阅读；
- [ ] 数学符号符合 `math_contract.md`；
- [ ] 坐标系定义清楚；
- [ ] 单位定义清楚；
- [ ] 输入输出定义清楚；
- [ ] 轨迹定义清楚；
- [ ] 任务定义清楚；
- [ ] Planner / MPC / Force Control 职责明确；
- [ ] command owner 明确；
- [ ] 安全边界明确；
- [ ] 图片位于 `docs/img/`；
- [ ] 没有编造实验数据；
- [ ] 没有把“应该”写成“已经”。

---

# 27. Agent 最终输出格式

完成文档后，Agent 必须在回复中明确给出：

```text
变更文件：
- docs/xxx.md

图片：
- docs/img/xxx.svg

是否修改源码：
- 否

是否修改配置：
- 否

是否修改测试：
- 否

文档状态：
- DRAFT

审查等级：
- L1 / L2 / L3

待人工确认：
- ...

发现的源码 / 文档冲突：
- 无
或
- ...

验证：
- 未执行
或
- 已执行已有只读验证：...
```

必须明确写：

```text
未修改源码。
```

如果某项无法确认：

```text
不确定，需要人工确认。
```

---

# 28. 最终规则

> **Agent 可以读取整个仓库，但文档任务只能修改 `docs/` 和 `docs/img/`。**

> **Agent 可以分析源码，但不能为了文档修改源码。**

> **CURRENT、PROPOSED 和 TBD 必须严格区分。**

> **所有当前实现结论必须可追溯。**

> **任何 Agent 输出默认都是 DRAFT。**

> **Agent 不能审阅和批准自己的输出。**

> **未经过人工完整审阅的文档不能作为实现依据。**

> **文档提出源码变化后，必须通过独立 Implementation Task 执行。**

> **对于规划、MPC、力控和 RL，必须明确 reference ownership、command ownership 和 safety ownership。**

> **人始终保留最终技术决策权。**xxxxxxxxxx Status: DRAFTWarning: 本文档尚未经过人工审查，不能作为实现依据。
