# wbmm_core

`wbmm_core` 是 WBMM 的纯 C++17 领域合同包。它定义“哪些数据可以在模块之间流动、模块可以请求哪些能力、失败如何表达”，不实现具体机器人、求解器、中间件或硬件驱动。

> 架构调整说明（2026-09-05）：科研快速开发版设计决定把 `wbmm_math` 合入本包的 `math/` 子目录，并把现有 Ports 裁剪为四个稳定接口。本包已先完成 math 合并；`wbmm_math` 包暂时保留为兼容转发层，Ports 裁剪按真实调用者分批进行。

## 边界

允许依赖：

- C++17 标准库；
- Eigen3（仅用于 `wbmm_core/math/` 的转换与线性代数）；
- 测试阶段的 `ament_cmake_gtest`（在线 ament 模式）或系统 GTest（离线模式）。

禁止依赖：

- ROS 2 消息、节点、topic、TF 或 ros2_control；
- 具体机器人 SDK、URDF/SRDF 或设备地址；
- Pinocchio、OCS2、REMANI、ESDF/nvblox、MoveIt 和 MuJoCo。

这些实现必须位于 adapter、backend 或 integration 包，并通过 `wbmm_core::...Port` 接入。

## 稳定合同

- 长度、角度、速度、力和时间采用 SI 单位；
- 空间数据携带 `frame_id` 和带 clock 类型的时间戳；
- 四元数内部顺序固定为 `wxyz`，ROS adapter 负责与消息的 `xyzw` 转换；
- 关节数据必须携带名称，所有 adapter 按名称和显式顺序映射；
- 禁止静默截断、补零或猜测关节顺序；
- 环境和碰撞模型使用非零 revision，规划结果必须回传所使用的 revision；
- `StateProviderPort` / `CommandSinkPort` 仅用于领域测试或非实时边界，不能绕过 ros2_control 的真实硬件资源所有权。

当前六关节差速底盘 adapter 必须保持：

```text
x = [base_x, base_y, base_yaw, q1, q2, q3, q4, q5, q6]  # 9D
u = [base_v, base_yaw_rate, qdot1, qdot2, qdot3, qdot4, qdot5, qdot6]  # 8D
```

`DifferentialDriveContract` 只验证模型、维度和关节顺序。实际向量打包必须由 adapter 完成，核心包不固定关节数量。

## Python 原型到 C++ 字段映射

| Python 原型字段 | C++ 类型/字段 | 约束 |
|---|---|---|
| `pose.frame_id` | `Pose::header.frame_id` | 不允许为空 |
| `pose.stamp` | `Pose::header.stamp` | clock 显式、时间非负 |
| `pose.position` | `Pose::position` | m、finite |
| `pose.orientation_wxyz` | `Pose::orientation` | 单位四元数、wxyz |
| `wrench.force` | `Wrench::force` | N、finite |
| `wrench.torque` | `Wrench::torque` | N·m、finite |
| `state.base` | `WholeBodyState::base` | x/y(m)、yaw(rad) |
| `state.joint_names` | `WholeBodyState::joints.names` | 非空、唯一、顺序显式 |
| `state.q` | `WholeBodyState::joints.positions_rad` | 与 names 等长 |
| `input.base` | `WholeBodyInput::base_command` | 差速 2D、全向 3D、固定 0D |
| `input.qdot` | `WholeBodyInput::joint_velocities_radps` | 与 joint_names 等长 |
| `trajectory.t` | `time_from_start_s` | 非负、严格递增 |
| `environment.revision` | `EnvironmentSnapshot::revision` | 非零、一致性可审计 |

## 使用方式

```cpp
#include <wbmm_core/wbmm_core.hpp>

wbmm::core::WholeBodyState state;
const wbmm::core::Status status = wbmm::core::validateWholeBodyState(state);
if (!status.ok()) {
  // 在 adapter/application 边界记录 status.code() 和 status.message()
}
```

独立构建与测试：

```bash
# ROS/ament 环境
colcon build --packages-select wbmm_core
colcon test --packages-select wbmm_core
colcon test-result --verbose

# 无 ROS/ament 的离线模式
cmake -S src/core/wbmm_core -B build/wbmm_core_offline -DWBMM_BUILD_OFFLINE=ON
cmake --build build/wbmm_core_offline
ctest --test-dir build/wbmm_core_offline --output-on-failure
```

## 当前成熟度

当前版本为 `0.1.0`：核心合同和离线单元测试已建立，但尚未接入现有运行链，也没有获得仿真或实机证据。任何字段语义变更都应先更新设计文档和测试；破坏兼容性的变更需要版本升级和 ADR。
