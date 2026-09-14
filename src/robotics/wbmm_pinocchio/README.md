# wbmm_pinocchio

基于 Pinocchio 的机器人模型与全身运动学修正包。

## 职责

- `wbmm::pinocchio::PinocchioRobotModel`：`wbmm::core::RobotModel` 的 Pinocchio 实现，
  负责 URDF 加载、FK、frame Jacobian、关节命名和限位校验。
- `wbmm::pinocchio::WholeBodyKinematics`：把末端修正量分配到差速底盘和机械臂关节，
  输出 9D 全身参考状态。

## 不负责

- ROS 消息转换：见 `wbmm_ros_interfaces`。
- 力控、导纳、轨迹发布、安全逻辑。

## 依赖

`wbmm_core`、Eigen3、Pinocchio。
