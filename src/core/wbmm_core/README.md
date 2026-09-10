# wbmm_core

`wbmm_core` 是 WBMM 的第一版纯 C++ 核心合同包。

## 目标

只定义全项目稳定、跨模块共享的数据和最小接口，不实现：

- ROS2 节点；
- 搜索算法；
- 轨迹优化；
- OCS2 适配；
- MuJoCo / 实机 Adapter；
- 可视化。

## 当前内容

```text
include/wbmm_core/types.hpp
include/wbmm_core/trajectory.hpp
include/wbmm_core/robot_model.hpp
include/wbmm_core/validation.hpp
include/wbmm_core/wbmm_core.hpp
```

### `types.hpp`

定义：

- 时间、frame、位姿、速度、力/力矩；
- 底盘状态、关节状态、全身状态；
- 全身输入；
- 执行相位；
- 时钟域 `kSystem` / `kSimulation` / `kOcs2Mpc`；
- 基础有限性与四元数检查。

### `trajectory.hpp`

定义：

- `TaskTrajectory`：任务层要求末端做什么；
- `PhaseSchedule`：任务相位及其明确的起止时间；
- `SearchResult`：拓扑和初值，要求 `base_path`、`arm_seed`、`phases` 长度一致；
- `WholeBodyTrajectory`：规划完成后唯一的机器人名义运动轨迹；
- `TaskTrajectory` 和 `WholeBodyTrajectory` 都不包含力控；力控只在执行层修正已经规划好的名义轨迹。

### `validation.hpp`

提供第一版统一 fail-closed 校验：

- 空 `frame_id`；
- `NaN` / `Inf`；
- 非单位四元数；
- 关节名称重复或数组维度不一致；
- 非严格递增轨迹时间；
- 零 `revision`；
- 未指定时钟域或同一条轨迹内时钟域不一致；
- 轨迹中途切换 frame、base_model 或关节顺序；
- `feedforward_input` 与对应 `state` 的 base_model / 关节顺序不一致；
- 关节数量、数组维度或关节名称唯一性不满足结构契约；
- `base_model` 未指定或输入维度与 `base_model` 不匹配。

统一入口为：

```cpp
wbmm::core::validate(whole_body_state);
wbmm::core::validate(whole_body_input);
wbmm::core::validate(search_result);
wbmm::core::validate(whole_body_trajectory);
wbmm::core::validate(task_trajectory);
wbmm::core::validate(twist);
wbmm::core::validate(wrench);
wbmm::core::validate(phase_schedule, trajectory_duration);
```

其中：

- core 只检查“差速底盘 + 六关节”的结构契约；
- 具体关节名称、顺序和限位由 `RobotModel::validate()` 检查；
- JAKA 名称只存在于具体 `RobotModel` / Adapter。

### `robot_model.hpp`

定义最小 `RobotModel` 接口：

- 状态/输入维度；
- 底盘模型；
- 关节顺序；
- 关节和底盘限制；
- 正运动学；
- 全身 frame Jacobian，满足 `V = J(x)u`，当前 8D 输入对应 `6 x 8`；
  - `V = [linear_velocity; angular_velocity]`，前三行为线速度，后三行为角速度；
  - 参考点为 `link_name` 原点，不是质心；
  - 线速度和角速度均表达在 `state.header.frame_id`；
- 状态和输入校验。

## 设计原则

- 第一版只做最小核心，后续有真实需求再扩展。
- 算法模块不依赖 ROS、可视化、OCS2 或硬件。
- 关节必须按名称映射，不能假设顺序。
- 内部四元数为 `wxyz`，ROS 边界负责与 `xyzw` 转换。
- 第一版只实现差速底盘 + 六关节；具体关节名称、顺序和限位由 `RobotModel` 检查。
- 所有物理量使用 SI 单位。
- 变量名默认不加单位后缀；单位由本包契约统一规定（m、rad、m/s、rad/s、s、N·m）。

## 构建

`wbmm_core` 通过 `target_compile_features(... cxx_std_17)` 向所有下游传播 C++17 要求。

```bash
colcon build --packages-select wbmm_core
colcon test --packages-select wbmm_core
```

## 当前边界

本包只提供核心合同。搜索、优化、参考适配、跟踪、安全门和机器人 Adapter 均在后续包中实现。
