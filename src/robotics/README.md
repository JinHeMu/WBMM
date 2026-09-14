# robotics

机器人本体唯一描述来源。当前已建立 `tracer_jaka_description` 作为统一入口，后续继续合并 MuJoCo/OCS2 中的重复 URDF。

C++ 侧的通用机器人模型、全身运动学和数据转换拆分为两个包：

- `wbmm_pinocchio`：Pinocchio `RobotModel` 适配器、全身运动学修正。命名空间
  `wbmm::pinocchio`。
- `wbmm_ros_interfaces`：`wbmm_core` 与 Eigen / ROS 消息之间的转换接口。命名空间
  `wbmm::ros_interfaces`。
