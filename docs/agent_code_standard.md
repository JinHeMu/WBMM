# Agent 代码编写与项目可掌控性标准

> Status: ACTIVE
> Author: Agent
> Reviewer: TBD
> Reviewed at: TBD
> Review Level: L1
> Warning: 本文档尚未经过人工审查，不能作为实现依据。

> 本文件规定 Agent 在本项目中创建、修改和重构代码时必须遵守的规则。
> 核心目标不是“让 Agent 尽可能多地写代码”，而是：
>
> **在使用 Agent 提高开发效率的同时，项目的目录结构、模块职责、数据流、运行入口和关键算法必须始终能够被人理解和掌握。**

# 1. 核心原则

本项目使用 Agent 编写代码时，必须遵守以下最高优先级原则。

## 1.1 人必须能够掌握整个项目

Agent 写完代码后，项目负责人至少应该能够回答：

```
这个目录是干什么的？

这个文件是干什么的？

程序从哪里启动？

数据从哪里来？

经过哪些模块？

最后输出到哪里？

Planner 在哪里？

Optimizer 在哪里？

Controller 在哪里？

Robot Adapter 在哪里？

Simulation 和 Real Robot 的边界在哪里？

一个新任务应该从哪里接入？

一个算法要替换时应该替换哪个模块？
```

如果这些问题无法快速回答，则说明当前架构已经过度复杂。

## 1.2 优先保持简单结构

Agent 不得为了“架构优雅”而主动引入：

- 多余抽象层；
- 多余 manager；
- 多余 factory；
- 多余 registry；
- 多余 plugin system；
- 多余 dependency injection；
- 多余 wrapper；
- 多余 adapter；
- 多余 interface hierarchy；
- 大量只有几行代码的小文件。

原则：

> **能够通过简单函数解决的问题，不创建复杂类。**

> **能够通过一个模块解决的问题，不创建三层抽象。**

> **能够通过明确调用关系解决的问题，不引入框架。**

# 2. 项目掌控优先于代码生成速度

Agent 不得以：

```
开发速度
代码行数
自动化程度
框架完整性
```

作为第一目标。

优先级为：

```
1. 人能否理解
2. 模块职责是否清楚
3. 数据流是否清楚
4. 是否方便替换算法
5. 是否方便仿真和实机部署
6. 是否容易测试
7. 最后才是减少人工编码工作
```

# 3. 每个文件必须有明确职责

每个主要源码文件必须能够用一句话解释：

```
这个文件负责什么？
```

例如：

```
kinematics/ur5e.py
负责 UR5e 的 FK、Jacobian 和 IK 基础运动学。

planning/rrt_connect.py
负责在关节空间搜索一条无碰撞离散路径。

trajectory/optimizer.py
负责将离散搜索路径优化为平滑、满足约束的轨迹。

control/admittance.py
负责根据外力生成笛卡尔位姿修正量。

adapters/mujoco/robot.py
负责把统一 Robot 接口转换为 MuJoCo API。

apps/run_insertion.py
负责组装插入任务各模块并启动程序。
```

如果一个文件无法一句话说明职责，通常意味着职责过多，需要重新设计。

# 4. 强制维护项目空间地图

项目必须存在一个可以快速了解整个仓库的文档，例如：

```
docs/project_map.md
```

它至少包含：

```
项目目录结构

每个一级目录职责

主要二级目录职责

主要运行入口

核心模块

模块依赖关系

数据流

仿真入口

实机入口

当前主要任务
```

示例：

```
src/hard_disk_robot/

core/
    基础数据结构和最小公共接口

kinematics/
    FK / IK / Jacobian

collision/
    碰撞检测

planning/
    RRT-Connect、A* 等路径搜索

trajectory/
    路径后处理和轨迹优化

control/
    导纳、MPC 接口、轨迹跟踪

contact/
    接触检测和接触状态估计

tasks/
    插拔硬盘等任务逻辑

adapters/
    MuJoCo / ROS2 / Real Robot 适配

apps/
    可直接运行的程序入口
```

Agent 如果：

- 新增一级目录；
- 删除一级目录；
- 改变主要模块职责；
- 新增新的运行入口；
- 改变核心数据流；

必须同步更新 `docs/project_map.md`。

# 5. 每次编码前必须先确定“修改边界”

Agent 在实现任务前必须明确本次修改涉及哪些模块。

例如：

```
任务：
增加 RRT-Connect 路径平滑功能。

预计修改：

planning/rrt_connect.py
    不修改搜索逻辑。

trajectory/path_smoother.py
    新增路径 shortcut 功能。

tests/test_path_smoother.py
    新增对应测试。

不修改：

kinematics/
collision/
control/
adapters/
```

原则：

> **先确定影响范围，再修改代码。**

不得在完成一个局部任务时顺便大规模重构无关模块。

# 6. 禁止“顺手重构”

Agent 不得因为：

```
这里看起来可以更优雅
这里可以重新设计一下
这个命名我不喜欢
这个接口可以统一
```

而主动修改任务范围之外的代码。

如果发现值得重构的问题，只能记录：

```
发现潜在重构项：

- 文件：
- 当前问题：
- 建议：
- 是否影响当前任务：
- 建议是否另开任务：
```

除非当前任务无法继续，否则不得顺手重构。

# 7. 修改文件数量必须可控

对于普通功能任务，应尽量控制修改范围。

推荐原则：

```
小功能：
1～3 个核心文件

中等功能：
3～8 个核心文件

超过约 8～10 个核心文件：
应重新检查是否正在发生架构扩散
```

这不是绝对限制。

但如果一个简单功能需要修改大量目录和文件，Agent 必须解释原因。

# 8. 新建文件必须有理由

Agent 不得习惯性创建新文件。

新建文件前应判断：

```
现有模块是否已经适合承载该功能？

新文件是否代表一个真正独立的职责？

未来是否存在替换或复用需求？

拆分后是否真的更容易理解？
```

不允许出现：

```
planner_manager.py
planner_factory.py
planner_registry.py
planner_context.py
planner_config_manager.py
planner_service.py
```

仅仅为了调用：

```
RRTConnectPlanner.plan()
```

# 9. 抽象只允许解决真实问题

允许创建接口或抽象层的典型情况：

```
MuJoCo 与真实机器人需要共用算法

RRT 和 A* 需要可替换

不同碰撞检测后端需要共用 Planner

MPC 和普通轨迹跟踪器需要统一上层调用
```

例如：

```
TrajectoryPlannerPort

    ├── RRTConnectPlanner
    ├── AStarPlanner
    └── KinoAStarPlanner
```

这是合理抽象，因为：

```
上层只关心：

trajectory = planner.plan(start, goal)
```

## 9.1 禁止为了未来假设提前抽象

不允许因为：

```
以后可能会有很多 planner
以后可能会支持很多机器人
以后可能会上云
以后可能接很多框架
```

而提前搭建复杂系统。

原则：

> **只为已经存在或近期明确存在的变化点设计接口。**

# 10. 算法代码和工程代码必须区分

项目尽量保持：

```
算法核心
    ↓
Backend-neutral

适配层
    ↓
MuJoCo / ROS2 / Hardware

应用层
    ↓
Task / Experiment
```

例如：

```
trajectory/optimizer.py
```

不得直接包含：

```
mujoco.mj_step()
rclpy.spin()
ROS topic publish
真实机械臂 SDK
```

这些应由：

```
adapters/
```

负责。

这样算法才能：

```
Python 单测
MuJoCo
ROS2
实机
```

共同复用。

# 11. Agent 必须优先写“可独立理解”的算法模块

对于核心算法，例如：

```
IK
Jacobian
RRT-Connect
A*
Trajectory Optimization
Admittance
Contact Detection
```

应尽可能保持：

```
输入明确
输出明确
依赖少
可以单独测试
```

例如：

```
path = planner.plan(start, goal)
```

比：

```
global_context.planner_manager
    .get_active_planner()
    .planning_service
    .request(...)
```

更适合本项目。

# 12. 不要求人记住每一行算法

项目负责人的掌握目标不是：

```
能够背出 RRT-Connect 每一行代码

能够手写 L-BFGS

能够记住 Pinocchio 所有 API
```

而是必须掌握：

```
算法解决什么问题

算法输入是什么

算法输出是什么

算法依赖什么信息

主要 cost / constraint 是什么

核心参数是什么

失败时会发生什么

代码位于哪里

上游是谁

下游是谁

如何替换
```

例如对于 RRT-Connect，负责人至少应知道：

```
输入：
start joint state
goal joint state

依赖：
collision checker
joint limits

输出：
离散 joint-space path

核心过程：
双向树
sample
nearest
extend
connect

下游：
trajectory optimizer
```

无需记住每一个循环实现。

# 13. 数据流必须清楚

核心系统不得出现无法追踪的数据传播。

推荐：

```
Task
  |
  v
Task Reference
  |
  v
Planner
  |
  v
Geometric Path
  |
  v
Trajectory Optimizer
  |
  v
Nominal Trajectory
  |
  v
Contact / Admittance Correction
  |
  v
Tracking Controller / MPC
  |
  v
Robot Adapter
  |
  v
MuJoCo / Real Robot
```

每个箭头都应该能够找到实际代码调用。

# 14. 必须明确程序入口

所有可执行功能必须存在明确入口。

例如：

```
apps/run_rrt_demo.py

apps/run_insertion_sim.py

apps/run_insertion_real.py
```

禁止需要用户记住：

```
先运行脚本 A
再 import B
然后手动执行 C
再开另一个 terminal 调 D
```

但文档中没有记录。

# 15. 工作目录必须稳定

Agent 不得依赖模糊的当前目录行为。

禁止：

```
open("../../../../../config.yaml")
```

或依赖：

```
必须刚好在某个目录运行才有效
```

必须明确：

```
项目根目录
资源目录
配置目录
模型目录
输出目录
```

路径处理应尽量基于：

```
package root
project root
explicit config path
```

而不是隐藏的 working directory 假设。

# 16. 运行方式必须被记录

每个主要应用必须能明确回答：

```
在哪里运行？

运行什么命令？

输入是什么？

输出是什么？

依赖什么环境？
```

例如：

```
功能：
MuJoCo 插入仿真

入口：
apps/run_insertion_sim.py

工作目录：
项目根目录

命令：
python -m hard_disk_robot.apps.run_insertion_sim

主要输入：
config/insertion.yaml

模型：
simulation/mujoco/models/...

输出：
logs/insertion/
```

# 17. 控制依赖数量

引入新的第三方依赖前必须说明：

```
为什么需要？

解决什么问题？

现有依赖是否已经能完成？

是否只是为了几行辅助功能？

是否增加部署困难？

是否影响 ROS2 / Ubuntu / 实机？
```

禁止为了：

```
一个小矩阵运算
一个简单配置解析
一个小工具函数
```

引入大型依赖。

# 18. 开源项目集成规则

对于：

```
OCS2
remain planner
OMPL
Pinocchio
MoveIt
MuJoCo
```

不得无原则复制整个工程。

必须首先判断属于哪种方式。

## 18.1 Library

例如：

```
Pinocchio
MuJoCo
Eigen
```

优先作为正常第三方库使用。

## 18.2 Adapter

如果外部系统本身复杂，但只需要调用部分功能，例如：

```
OCS2
MoveIt
Robot SDK
```

优先通过小型 Adapter 隔离。

## 18.3 Algorithm Extraction

如果真正需要研究或修改的是核心算法，例如：

```
remain planner 中的 A*
RRT
trajectory optimization
```

可以理解算法后重新实现项目需要的最小版本。

原则：

> **复用成熟库的基础能力，掌握自己研究相关的算法核心。**

# 19. 不允许 Open-Source Project 控制本项目架构

禁止出现：

```
因为 OCS2 是这样组织目录，
所以整个 WBMM 都照 OCS2 组织。

因为 MoveIt 使用 Plugin，
所以所有 Planner 都做成 Plugin。

因为 ROS2 使用复杂 package，
所以研究代码也全部复制 ROS2 风格。
```

外部框架应服务于本项目。

不能反过来让本项目成为外部框架的附属结构。

# 20. Adapter 必须保持薄

Adapter 只负责：

```
类型转换
API 转换
坐标或单位转换
backend 调用
```

不得在 Adapter 中隐藏核心算法。

例如：

```
MujocoRobotAdapter
```

可以负责：

```
read_joint_state()
send_joint_command()
get_wrench()
```

不应该同时实现：

```
RRT
IK
Trajectory Optimization
Admittance
Task Planning
```

# 21. Core 必须保持小

`core/` 只允许放真正全项目公共的内容。

例如：

```
Pose
Wrench
JointState
TrajectoryPoint

RobotPort
CollisionCheckerPort
TrajectoryPlannerPort
```

不得把所有工具都放入：

```
core/
common/
utils/
```

形成无法理解的“垃圾桶目录”。

# 22. Utils 目录限制

原则上避免大型：

```
utils/
```

如果一个函数具有明确领域含义，应放入对应领域模块。

例如：

```
quaternion_distance()
```

更适合：

```
core/rotation.py
```

而不是：

```
utils/misc.py
```

# 23. Task 和 Algorithm 必须分离

例如硬盘插入任务：

```
tasks/disk_insertion.py
```

负责：

```
APPROACH
ALIGN
CONTACT
INSERT
VERIFY
RETRACT
```

但不应该自己实现：

```
RRT
IK
MPC
Admittance
```

Task 负责：

> **什么时候使用什么能力。**

Algorithm 负责：

> **这个能力具体怎么算。**

# 24. 配置和算法分离

经常调整的实验参数应进入明确配置。

例如：

```
RRT step size
goal bias
trajectory weight
force threshold
admittance gain
MPC horizon
```

但不要为了“配置化”把所有内部常量全部搬入 YAML。

原则：

```
研究参数 / 实验参数
    -> config

算法内部固定实现细节
    -> code
```

# 25. 不允许配置文件成为第二套程序

禁止出现超大型 YAML：

```
algorithm: ...
planner_type: ...
manager_type: ...
factory_type: ...
plugin_class: ...
execution_graph: ...
...
```

导致程序真实逻辑隐藏在配置中。

配置应该：

> 调参数。

而不是：

> 编程序。

# 26. 每个核心模块必须有最小测试

核心算法至少应存在最小行为测试。

例如：

```
IK：
已知 q -> FK -> IK -> position error

Collision：
home free / known collision

RRT：
简单场景能够找到路径

Optimizer：
优化后 cost 下降

Admittance：
给定恒力后 offset 方向正确
```

测试重点是：

```
模块是否履行职责
```

而不是追求大量测试覆盖率数字。

# 27. 测试结构必须能够帮助人理解系统

测试名称应直接表达行为：

```
test_home_is_collision_free

test_rrt_connect_finds_path_around_obstacle

test_admittance_moves_along_force_direction
```

避免：

```
test_case_01
test_func
test_misc
```

测试本身也是理解项目的重要入口。

# 28. Agent 修改核心逻辑时必须说明算法变化

如果只是工程修改：

```
接口整理
路径修改
类型修复
```

应明确说明：

```
算法行为未改变。
```

如果修改：

```
cost
constraint
sampling
IK
force control law
MPC reference
```

必须说明：

```
算法行为发生变化。

修改前：
...

修改后：
...

原因：
...

可能影响：
...
```

# 29. 不允许隐藏 fallback

如果代码存在：

```
SciPy 不可用 -> 自定义 gradient descent

MPC 不可用 -> PID

传感器无数据 -> 默认 0
```

必须明确记录。

不得静默 fallback。

特别是机器人控制中：

```
传感器失败
控制器失败
规划失败
```

不得自动切换到可能危险的行为而没有日志和状态。

# 30. 实机代码必须显式区分

仿真和实机执行必须容易区分。

推荐：

```
adapters/mujoco/
adapters/ros2/
adapters/hardware/
```

应用入口例如：

```
run_insertion_sim.py
run_insertion_real.py
```

不得通过一个隐藏参数：

```
real=True
```

让不明确的程序突然控制真实机器人。

# 31. 实机执行必须存在安全门

涉及真实机器人运动时，Agent 不得默认：

```
程序启动 -> 立即执行运动
```

应至少具有明确的：

```
连接
初始化
状态检查
enable
执行
```

流程。

安全关键行为必须经过单独审查。

# 32. Agent 必须避免“大爆炸式提交”

大型功能应该拆成可以理解的阶段。

例如实现硬盘插入力控：

```
Step 1
建立 wrench 获取接口

Step 2
实现 contact detector

Step 3
实现 1D admittance

Step 4
扩展 6D admittance

Step 5
接入 Cartesian reference

Step 6
MuJoCo 验证

Step 7
接入 insertion task

Step 8
接入 RL
```

不要一个任务同时新增：

```
20 个文件
3 个 manager
4 个 interface
RL
MPC
Force Control
ROS2
MuJoCo
```

然后宣布“架构完成”。

# 33. 每一步都必须保持项目可运行

开发过程中尽量满足：

```
每一个逻辑阶段结束后：
项目结构完整
已有测试不被破坏
新的模块可以独立验证
```

避免长时间存在：

```
半重构状态
旧接口不能用
新接口也没完成
```

# 34. 每次修改后必须给出“项目认知摘要”

Agent 完成代码任务后，必须向人说明：

```
这次解决了什么问题？

改了哪些文件？

每个文件现在负责什么？

增加了什么新数据流？

程序从哪里进入？

如何运行？

哪些算法发生变化？

哪些算法没有变化？

哪些地方仍然是 TBD？
```

# 35. 强制文件变更说明

Agent 完成任务后必须输出：

```
新增文件：
- path/file.py
  作用：...

修改文件：
- path/file.py
  原作用：...
  本次修改：...

删除文件：
- ...

未修改的关键模块：
- ...
```

不能只写：

```
已完成修改。
```

# 36. 强制数据流说明

如果修改影响系统调用关系，必须给出简化数据流。

例如：

```
DiskInsertionTask
      |
      v
TaskTrajectory
      |
      v
RRTConnectPlanner
      |
      v
TrajectoryOptimizer
      |
      v
AdmittanceController
      |
      v
RobotAdapter
      |
      v
MuJoCo
```

人应该通过几十秒阅读重新建立项目整体认知。

# 37. 强制运行说明

每次增加可执行功能后必须说明：

```
入口：
工作目录：
运行命令：
输入：
输出：
依赖：
```

例如：

```
入口：
hard_disk_robot/apps/run_insertion_sim.py

工作目录：
repository root

运行：
python -m hard_disk_robot.apps.run_insertion_sim

输入：
config/insertion.yaml

输出：
MuJoCo simulation + log
```

# 38. 强制架构影响说明

每次任务结束时必须明确：

```
是否新增目录：
是否新增抽象层：
是否新增第三方依赖：
是否改变模块职责：
是否改变数据流：
是否改变运行入口：
是否改变算法：
是否改变实机行为：
```

如果均没有：

```
架构无变化。
```

# 39. 人工掌握检查

重要修改完成后，负责人应能够回答：

- 我知道新增文件为什么存在；
- 我知道修改文件原来做什么；
- 我知道程序入口在哪里；
- 我知道核心数据从哪里来；
- 我知道数据经过哪些模块；
- 我知道最终 command 从哪里产生；
- 我知道这个算法在哪里；
- 我知道如何替换这个算法；
- 我知道 MuJoCo 和实机的边界；
- 我知道主要配置在哪里；
- 我知道出现问题应该先看哪个模块。

如果大量问题回答不了：

> **暂停继续扩展功能，优先重新梳理架构。**

# 40. “看懂代码”的最低标准

项目负责人不要求理解所有代码细节。

对于每个核心文件，至少掌握四件事：

```
Why
为什么存在？

Input
输入是什么？

Output
输出是什么？

Where
在整个系统哪一层？
```

对于核心算法额外掌握：

```
核心思想
主要参数
主要约束
典型失败原因
```

达到这一程度即可认为“掌握该模块”。

# 41. Agent 不得制造知识黑箱

禁止以下工作方式：

```
用户提出需求
      |
      v
Agent 一次修改几十个文件
      |
      v
测试通过
      |
      v
用户不知道发生了什么
      |
      v
继续让 Agent 修改
      |
      v
项目只能靠 Agent 维护
```

这被视为项目失控。

# 42. 正确的 Agent 使用方式

推荐：

```
用户定义问题
      |
      v
Agent 分析影响范围
      |
      v
用户知道准备改什么
      |
      v
Agent 实现
      |
      v
测试
      |
      v
Agent 解释文件 + 数据流 + 运行方式
      |
      v
用户理解
      |
      v
进入下一功能
```

目标是：

> **Agent 负责降低编码成本。**

而不是：

> **Agent 取代项目负责人对系统的理解。**

# 43. 算法学习与工程开发的关系

对于研究相关算法，应遵循：

```
理解思想
    ↓
知道输入输出
    ↓
知道数学目标
    ↓
知道代码位置
    ↓
Agent 辅助实现
    ↓
通过实验理解行为
```

不要求：

```
脱离资料完整默写实现
```

但也不能：

```
完全不知道算法是什么
只知道 Agent 说测试通过
```

# 44. 对研究核心算法的额外要求

如果算法可能成为：

```
论文创新点
实验变量
baseline
核心研究模块
```

负责人需要比普通工程模块理解更深。

至少包括：

```
数学问题

输入输出

目标函数

约束

核心步骤

主要参数

计算复杂度大致来源

失败情况

与 baseline 的差异
```

这类模块不能长期作为完全不可理解的 Agent 黑箱。

# 45. 项目复杂度红线

出现以下情况时，应停止新增功能并进行整理：

```
不知道应该把新代码放哪里

两个以上模块职责高度重叠

同一种数据存在多个定义

同一个 command 有多个 owner

一个功能需要跨十几个模块修改

无法说清程序入口

无法说清当前工作目录

大量文件只有几行 wrapper

大量 Manager / Factory / Registry

必须让 Agent 搜索代码才能知道项目怎么运行

只有原 Agent 才能继续维护代码
```

这些都说明架构开始失控。

# 46. 推荐项目认知层次

负责人应从上到下掌握项目。

## Level 1：项目级

知道：

```
项目解决什么问题
有哪些一级模块
完整数据流
主要运行入口
```

## Level 2：模块级

知道：

```
每个目录做什么
主要文件做什么
模块之间如何调用
```

## Level 3：算法级

知道：

```
核心算法思想
输入输出
主要数学模型
关键参数
```

## Level 4：实现级

知道：

```
具体循环
数据结构
数值细节
API 使用
```

正常情况下：

> **Level 1 和 Level 2 必须牢牢掌握。**

研究核心模块：

> **掌握到 Level 3。**

一般工程实现：

> **不要求长期掌握所有 Level 4 细节。**

# 47. Agent 交付格式

每次完成代码任务后必须按照以下格式交付：

```
任务：
- ...

本次实现：
- ...

新增文件：
- path
  作用：

修改文件：
- path
  原职责：
  本次修改：

删除文件：
- 无 / ...

核心数据流：
...

运行入口：
...

工作目录：
...

运行命令：
...

算法变化：
- 无
或
- ...

架构变化：
- 无
或
- ...

新增依赖：
- 无
或
- ...

测试：
- ...

实机行为变化：
- 否 / 是

需要你重点理解的文件：
1. ...
2. ...
3. ...

你不需要深入关注的实现细节：
- ...

待确认：
- ...
```

# 48. Agent 自检清单

完成代码修改前必须检查：

- 本次修改是否严格围绕任务；
- 是否出现顺手重构；
- 是否创建了不必要的新文件；
- 是否创建了不必要的抽象层；
- 每个新增文件是否有一句话职责；
- 是否明确程序入口；
- 是否明确工作目录；
- 是否明确数据流；
- 是否明确模块上下游；
- 是否保持算法和 Adapter 分离；
- 是否保持 Task 和 Algorithm 分离；
- 是否保持 Core 足够小；
- 是否引入新依赖；
- 新依赖是否真的必要；
- 是否出现隐藏 fallback；
- 是否影响实机行为；
- 是否增加安全风险；
- 是否更新项目地图；
- 是否提供变更文件说明；
- 用户是否能够重新建立对项目的认知。

# 49. 最终原则

> **Agent 可以帮助写大量代码，但不能让项目只有 Agent 才能理解。**

> **项目结构必须首先服务于人的认知，而不是服务于框架。**

> **负责人必须牢牢掌握项目级和模块级结构。**

> **对于研究核心算法，应掌握其思想、数学问题、输入输出和关键参数，而不要求背诵所有实现。**

> **每个文件必须有明确职责。**

> **每个模块必须有明确输入和输出。**

> **每个应用必须有明确入口和工作目录。**

> **每个核心数据流必须可以从入口追踪到最终输出。**

> **不得通过无意义的 Manager、Factory、Registry、Wrapper 和 Adapter 堆叠制造复杂度。**

> **不得因为使用 Codex、Claude Code 或其他 Agent 而放弃对项目架构的理解。**

> **Agent 的价值是减少重复编码，而不是代替人的系统设计能力。**

> **当人无法解释项目结构时，应停止继续生成代码，先恢复对项目的掌控。**
