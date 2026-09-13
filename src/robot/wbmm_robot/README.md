# wbmm_robot

WBMM 移动机械臂的通用机器人与运动学模块。

## 职责

- `PinocchioRobotModel`：实现 `wbmm::core::RobotModel`，提供 FK、frame Jacobian、
  关节名映射与限位校验；
- `WholeBodyKinematics`：把笛卡尔修正量分配到差速底盘 + 六轴机械臂，输出 9D 状态；
- `wbmm_conversions`：Eigen 9D/8D 向量与 `wbmm_core` 合同之间的转换；
- `wbmm_ros_conversions`：ROS 消息与 `wbmm_core` 之间的转换。

本包不包含力控算法、ROS 节点、MPC 问题定义或任务逻辑。

## 依赖方向

```text
wbmm_core
    ▲
    │
wbmm_robot   ← 机器人模型 / 运动学 / 消息转换
    ▲
    │
whole_body_force_control   ← 力控算法与 ROS 节点
```
