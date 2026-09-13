# robot

机器人本体唯一描述来源。当前已建立 `tracer_jaka_description` 作为统一入口，后续继续合并 MuJoCo/OCS2 中的重复 URDF。

C++ 侧的通用机器人模型、全身运动学与消息转换位于 `wbmm_robot/` 包：

- `wbmm_robot`：Pinocchio `RobotModel` 适配器、全身运动学修正、
  `wbmm_core` ↔ Eigen / ROS 消息转换。
