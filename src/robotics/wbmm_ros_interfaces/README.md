# wbmm_ros_interfaces

ROS 消息与 `wbmm_core` / Eigen 数据之间的转换接口包。

## 职责

- `wbmm::ros_interfaces::toCoreState` / `toEigenState`：Eigen 9D 状态与
  `wbmm::core::WholeBodyState` 互转。
- `wbmm::ros_interfaces::toCoreWrench` / `toEigenWrench`：Eigen 6D wrench 与
  `wbmm::core::Wrench` 互转。
- `wbmm::ros_interfaces::toCoreInput` / `toEigenInput` / `makeZeroWholeBodyInput`：
  Eigen 8D 输入与 `wbmm::core::WholeBodyInput` 互转。
- `wbmm::ros_interfaces::wrenchFromRos`、`wholeBodyStateFromMpcObservation`、
  `toMpcTargetTrajectories`：ROS 消息与 core 轨迹/传感器类型互转。

## 不负责

- Pinocchio 模型和全身运动学：见 `wbmm_pinocchio`。
- 力控、导纳、ROS 节点、安全逻辑。

## 依赖

`wbmm_core`、Eigen3、ROS 消息包。
