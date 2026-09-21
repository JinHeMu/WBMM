# 环境、碰撞与机械臂评价骨架

> Status: ACTIVE  
> Author: Agent  
> Reviewer: TBD  
> Reviewed at: TBD  
> 人工审阅通过前，本骨架不能作为安全或实机执行依据。

本文承接“设计机械臂评价接口”对话，复用当前 `wbmm_core::RobotModel`。
只确定模块职责、输入输出和目录，不实现地图查询、碰撞或评价算法。

## 1. 目录与职责

```text
src/
├── map/wbmm_environment/
│   ├── include/wbmm_environment/
│   │   ├── types.hpp             # 地图元数据、距离查询状态与结果
│   │   ├── esdf_grid.hpp         # ESDF 网格数据与查询入口
│   │   └── esdf_loader.hpp       # NPZ 文件加载入口
│   ├── src/{esdf_grid,npz_esdf_loader}.cpp
│   ├── test/test_environment_skeleton.cpp
│   ├── CMakeLists.txt
│   └── package.xml
├── robotics/wbmm_collision/
│   ├── include/wbmm_collision/
│   │   ├── types.hpp             # 连杆局部碰撞球、范围、状态与结果
│   │   ├── collision_model.hpp   # 机器人碰撞球集合
│   │   └── environment_collision_checker.hpp
│   ├── src/environment_collision_checker.cpp
│   ├── test/test_collision_skeleton.cpp
│   ├── CMakeLists.txt
│   └── package.xml
└── metrics/wbmm_robot_metrics/
    ├── include/wbmm_robot_metrics/
    │   ├── types.hpp             # 指标结果与 Jacobian 评价约定
    │   ├── arm_metrics.hpp       # 机械臂构型评价入口
    │   └── joint_limit_metrics.hpp
    ├── src/{arm_metrics,joint_limit_metrics}.cpp
    ├── test/test_metrics_skeleton.cpp
    ├── CMakeLists.txt
    └── package.xml
```

环境只回答空间点的距离问题；碰撞模块连接机器人运动学、碰撞球和环境；
评价模块只报告指标，不决定规划代价或控制策略。三个包均为 C++17 库，
没有 ROS 节点、launch 或命令输出。

按照 [AGENTS.md](../AGENTS.md)“先出现第二个真实实现，再抽接口”的规则，
本轮采用具体的 `EsdfGrid`，暂不添加对话中建议的 `DistanceField` 抽象基类。
地图加载仍与查询分开。未来有第二个实际地图后端时再评估抽象需求。

## 2. 输入输出约定 [PROPOSED]

现有状态仍为差速底盘与六关节的 9D 状态，输入为 8D：

$$
\begin{aligned}
x &= [x_b,y_b,\psi_b,q_1,\ldots,q_6]^T \\
u &= [v,\omega,\dot q_1,\ldots,\dot q_6]^T
\end{aligned}
$$

差速底盘不增加横向速度自由度，非完整约束与运动传播由现有模型和搜索器负责。
位置、距离、碰撞球半径和安全余量单位均为 m，关节角为 rad。
不改变 `TaskTrajectory` 或 `WholeBodyTrajectory`，本轮仅评价单个构型。

| 入口 | 输入 | 输出 |
|---|---|---|
| `NpzEsdfLoader::load` | 外部提供的 NPZ 路径 | 加载状态、只读网格指针、说明 |
| `EsdfGrid::query` | 地图 frame 名及该 frame 中的三维点 | 查询状态、距离、可选梯度 |
| `EnvironmentCollisionChecker::checkBase` | `Header`、`BaseState` | 仅底盘环境碰撞结果，不要求伪造关节状态 |
| `EnvironmentCollisionChecker::check` | `WholeBodyState`、底盘/机械臂/全身范围 | 环境碰撞结果、最小净间隙、最危险球标识 |
| `ArmMetrics::evaluate` | `WholeBodyState`、连杆 frame、评价约定 | 奇异值、最小/最大奇异值、条件数、秩、操作度、关节限位裕量 |
| `JointLimitMetrics::evaluate` | `JointState`、`RobotLimits` | 每关节归一化裕量、最小裕量 |

碰撞查询要求状态与地图 frame 一致，不自动把 `map` 和 `odom` 当成同一坐标系。
外层负责坐标转换；之后的实现应在不一致时返回 `kFrameMismatch`。
碰撞球中心在所属连杆的局部坐标系中，通过现有 FK 转到状态 frame。
底盘球也必须指定真实连杆名，不在公共类型中写死 Tracer/JAKA 几何尺寸。

规划中的净间隙约定为 ESDF 距离减去球半径及调用者指定的安全余量；
净间隙小于等于零属于碰撞。未知区域与地图外点没有有效净间隙，调用者应拒绝该状态。
梯度为距离相对于地图 frame 中位置的导数；`gradient_valid=false` 时不可使用。
ESDF 距离正负沿用输入地图，不把未知区域中的负占位值当成已观测障碍距离。

`ArmMetrics` 后续从 `RobotModel::frameJacobian` 的末尾六列取机械臂 Jacobian。
Jacobian 行序保持 `[linear; angular]`，参考点为请求连杆原点，表达于状态 frame。
选项区分平移任务与完整六维位姿任务；位姿任务明确区分原始混合单位与
使用调用者提供的特征长度缩放角速度行的评价。指标不得跨不同任务/缩放约定直接比较。
归一化关节裕量约定：中点为 1，边界为 0，超界为负；顺序沿用模型关节顺序。

## 3. 当前骨架行为 [CURRENT]

本轮新增类可构造、可编译、可链接，但加载、距离查询、碰撞检测和指标计算
均返回 `kNotImplemented`。未计算的数值为 NaN，网格加载结果没有有效指针。
`CollisionResult::isFree()` 只有在状态为 `kFree` 时才返回 true，
不会把占位、未知、越界或错误状态当成有效无碰撞结果。

ESDF 网格预留 `esdf/occupancy/observed` 数组与 `origin/voxel_size/shape/frame_id`
元数据。数组约定为 xyz、C 顺序，z 最快；origin 表示网格下边界，体素查询位置采用中心。
这些约定须在实现加载器时对具体文件做核对，而不能只依据文件名推断。
本地 nvblox 导出器已导出上述主要字段，另有 `bounds_max/source/unknown_is_occupied`；
当前未新增 NPZ 解析依赖，也未读取或加载实际地图。

现有 `wbmm_search::BaseCollisionChecker` 可在后续通过外层 lambda 调用
`checkBase(header, base).isFree()`；本轮不接线，占位结果会拒绝所有状态。
该入口未来只覆盖底盘环境碰撞，不代表固定导航构型的机械臂也已检查。
完整状态入口同样只负责环境碰撞，不包含自碰撞、连续轨迹碰撞或接触许可。

## 4. 后续工作 [TBD]

- 实际 `map1` 的路径、frame、原点约定和 observed 缺失时的处理。本次未在 `src/map/` 找到 `map1`。
- NPZ 解析依赖、文件校验、插值距离与梯度。
- 底盘与机械臂碰撞球数据来源、覆盖质量、余量数值及机器人型号配置。
- FK 球变换、模型/状态校验、未知与越界处理的具体实现。
- SVD、数值秩阈值、特征长度选择及关节裕量实现。
- 自碰撞需要独立几何和排除规则，目前没有占位的“通过”结果。
- 动态地图、轨迹汇总、优化梯度及规划器接入在实际需求出现后再添加。

## 5. 验证与审阅

最小测试只验证占位行为与状态语义，构建检查确认依赖、头文件和链接入口。
这些不代表地图可用、碰撞有效、指标有效、仿真通过或实机通过。
新增库不接入控制链；不改变既有实机默认禁用执行的安全门。
碰撞几何、坐标约定、未知空间策略和后续安全接入须人工逐条审查。

2026-09-18 静态验证记录 [CURRENT]：

- `wbmm_core` 与三个新增包在独立临时目录构建、安装成功。
- 三个新增包共 4 个骨架行为测试通过；原有 core 的 22 个测试通过。
- 独立调用方通过安装后的 CMake 导出目标找到库，完成编译、链接及占位调用。
- 未进行实际 NPZ 加载、碰撞算法验证、指标算法验证、仿真或实机验证。

依据：当前 `wbmm_core/robot_model.hpp`、`wbmm_search/kino_astar.hpp`、
`nvblox_map_exporter.py` 和只读参考的 REMANI `plan_env` / `mm_config`。
REMANI 的环境距离查询与机器人采样思想作为参考，机器人运动学继续复用 WBMM。
