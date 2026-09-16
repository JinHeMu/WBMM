# Agent 输出文档标准

> Status: ACTIVE
> Author: Agent
> Reviewer: TBD
> Reviewed at: TBD
> Review Level: L1
> Warning: 本文档尚未经过人工审查，不能作为实现依据。

本文件规定 Agent 执行文档任务时的最小规则。核心原则：**文档任务只能修改 `docs/`；所有输出必须经过人工完整审阅后才能作为实现依据。**

# 1. 权限边界

文档任务允许读取：

```text
src/  include/  tests/  test/  config/  launch/
simulation/  scripts/  docs/
CMakeLists.txt  package.xml  URDF  MJCF  YAML
git diff  git log
```

文档任务只允许修改：

```text
docs/
docs/img/
```

禁止修改：

```text
src/**  include/**  CMake  package.xml
launch  YAML  URDF / MJCF
驱动 控制器 测试源码 vendor 代码
```

不得为了让文档成立而调整实现或系统行为。

# 2. 源码修改建议

文档发现源码需要修改时，只能记录：

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

后续实现必须作为独立任务。

# 3. 文档状态

允许的状态：

```text
DRAFT
REVIEW_PENDING
CHANGES_REQUESTED
APPROVED
REJECTED
DEPRECATED
```

Agent 新建文档默认必须是：

```text
Status: DRAFT
```

Agent 不得将文档标记为 `APPROVED`。

# 4. CURRENT / PROPOSED / TBD

必须严格区分：

```text
[CURRENT]   当前已存在的行为
[PROPOSED]  建议设计或未来方案
[TBD]       无法确认，需要人工确认
```

不得把推测写成当前事实。

# 5. 证据与不确定性

当前实现结论必须能追溯到：

```text
源码
配置
已有测试
数学契约
坐标系契约
已有文档
实验记录
rosbag / benchmark / 实机记录
```

禁止编造实验、测试、数据、结论或机器人能力。

信息不足时必须写：

```text
不确定，需要人工确认。
```

可写推测，但必须标明：

```text
推测：
原因：
需要确认：
```

# 6. 内容要求

- 使用移动机械臂、规划、轨迹优化、MPC、力控领域常用术语。
- 解释模块为什么存在、算法解决什么问题、公式的物理意义、接口的输入输出。
- 不堆砌类名和接口名。
- 复杂框架不等于技术深度。

# 7. 移动机械臂必须说明

- 底盘类型：差速 / 全向 / 固定。
- 机械臂自由度、末端执行器、是否有夹爪。
- 状态、输入、维度和单位：
  ```text
  Model:
  State x:
  Input u:
  State dimension:
  Input dimension:
  Units:
  ```
- 差速底盘的非完整约束。
- 关节位置/速度/加速度限位。
- 底盘速度/角速度限位。
- 碰撞、自碰撞、环境碰撞。
- 导航与操作何时联合规划，不能简单把底盘轨迹加机械臂轨迹称为全身规划。

# 8. 数学与坐标

- 行内公式用 `$...$`，独立公式用 `$$...$$`，保证 Typora 可显示。
- 数学符号优先遵守 `docs/math_contract.md`。
- 发现源码与契约不一致时必须记录，不得静默选择。
- 每个重要公式说明变量、维度、坐标系、单位、物理意义。
- `position / orientation / twist / wrench / Jacobian / force / torque` 必须说明坐标系。
- 坐标系优先遵守 `docs/frame_contract.md`。
- 四元数必须注明 `xyzw` 或 `wxyz`。
- `T_A_B` 必须明确表示方位方向，禁止同一项目混用。

# 9. 轨迹定义

必须区分：

- `TaskTrajectory`：任务希望机器人完成什么。
- `WholeBodyTrajectory`：机器人实际应该如何运动。

并说明位置、速度、加速度、jerk 的连续性要求。

# 10. 力控与控制链

力控文档至少说明：

- wrench 定义、单位、frame。
- 期望与测量 wrench。
- 导纳/阻抗结构。
- 接触方向、接触状态。
- 力/力矩限幅、超时、SAFE_HOLD、FAULT。
- 控制链中谁生成参考、谁保证可行、谁执行。

控制链所有权必须唯一，不能同时存在多个机械臂命令所有者。

# 11. OCS2 / MPC 与 RL

OCS2 / MPC 文档必须说明：

- 状态、输入、模型。
- 目标函数、约束、horizon、频率。
- reference ownership。
- 跟力控、规划器、控制器的边界。

RL 文档必须说明：

- observation、action、reward、termination。
- 与传统控制和安全层的边界。
- 不得把 RL 写成安全兜底。

# 12. 安全与验证

- 实机默认 `execution_enabled=false`。
- 没有安全门、急停、看门狗、超时和故障恢复说明时，不得建议实机执行。
- 文档中的验证必须区分：
  - 静态检查；
  - 单元测试；
  - 仿真；
  - 实机。
- 未执行的内容必须标记 `TBD`。

# 13. 图片与推荐结构

图片统一放：

```text
docs/img/
```

推荐文档结构：

```text
# 标题
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

# 14. Source of Truth

优先级：

1. `docs/math_contract.md`：数学符号与状态/输入契约。
2. `docs/frame_contract.md`：坐标系和 TF 发布权。
3. 源码与配置：当前实现行为。
4. 已批准设计文档：目标架构。
5. `PROPOSED` / `TBD` 不得当作已实现。

冲突时必须记录：

```text
发现冲突：
- math_contract.md：
- frame_contract.md：
- 当前源码：
- 影响：
- 需要人工确认：
```

# 15. 审查等级

- L0：纯信息整理，仍需人工过目。
- L1：普通设计/接口文档，至少一人审查。
- L2：涉及算法契约、坐标、控制链，需相关模块负责人审查。
- L3：涉及实机安全、力控执行、急停，必须多人逐条审查。

Agent 只能输出 `DRAFT` 或 `REVIEW_PENDING`。

# 16. 禁止事项

- 在文档任务中修改源码、launch、YAML、测试。
- 自己批准自己的文档。
- 编造数据或结论。
- 把 `PROPOSED` 写成 `CURRENT`。
- 隐藏不确定性。
- 用文档修改来掩盖实现问题。

# 17. 交付前检查

- 是否只改了 `docs/` / `docs/img/`；
- 是否标明 `CURRENT / PROPOSED / TBD`；
- 是否提供依据；
- 是否说明不确定性和待确认项；
- 是否更新了相关索引文档；
- 文档是否为 `DRAFT`；
- 用户是否能快速理解结论、边界和风险。

# 18. 最终输出格式

```
任务类型：
修改文件：
是否修改源码：否
验证方式：
不确定项：
人工审查结论：DRAFT / APPROVED / REJECTED
```

# 19. 最终规则

> 文档任务只能改 `docs/` 和 `docs/img/`。
> 先契约，后代码。
> 不编造，不掩盖不确定性。
> 所有 Agent 输出都必须经过人工完整审阅。
