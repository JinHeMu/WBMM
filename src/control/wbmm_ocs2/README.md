# wbmm_ocs2

WBMM 自己的 OCS2 移动机械臂问题定义包。

## 目标

把原先写在 vendor `ocs2_mobile_manipulator` 中的项目定制迁出，形成一个小而
明确的、项目拥有的 MPC 问题定义包：

- 只实现 WBMM 当前真实使用的**轮式差速底盘 + 六轴机械臂**模型：

  ```text
  x = [base_x, base_y, base_yaw, q1..q6]   // 9D
  u = [v, omega, qdot1..qdot6]             // 8D
  ```

- 不实现 DefaultManipulator、FloatingArmManipulator、
  FullyActuatedFloatingArmManipulator 分支；
- 底层 DDP、SQP、OCS2 rollout、Pinocchio、自动微分复用上游 OCS2；
- 不依赖 ROS (rclcpp)、不依赖 `wbmm_core`（本阶段刻意不接入）。

## 内容

```text
include/wbmm_ocs2/
  WbmmModelInfo.h              9D/8D/6 维模型合同
  FactoryFunctions.h           从 URDF 创建 planar-base Pinocchio 模型
  PinocchioMapping.h           OCS2 状态/输入 <-> Pinocchio 映射
  PreComputation.h             Pinocchio FK/Jacobian 缓存
  Dynamics.h                   差速底盘 + 关节速度动力学
  WbmmInterface.h              OptimalControlProblem 组装入口
  cost/                        QuadraticInputCost / WholeBodyTrajectoryCost
  constraint/                  EndEffector / BodyRelative / SelfCollision /
                               EnvironmentCollision
  collision/                   EnvironmentGeometryInterface
```

## 与 ROS 包的关系

```text
OCS2 通用库 (ocs2_core / ocs2_ddp / ...)
        │
        ▼
wbmm_ocs2          ← 本包：MPC 问题定义
        │
        ▼
wbmm_ocs2_ros      ← ROS 节点、话题、TF、系统适配
```

`wbmm_ocs2_ros` 中的 MPC/MRT 节点只负责构造 `WbmmInterface` 并接入 ROS，
不再直接包含 MPC 问题定义。

## 配置兼容

迁移期继续使用原来 `task.info` 的 `wheelBasedMobileManipulator` 子树键，
例如：

```text
initialState.base.wheelBasedMobileManipulator
inputCost.R.base.wheelBasedMobileManipulator
jointVelocityLimits.lowerBound.base.wheelBasedMobileManipulator
```

`model_information.manipulatorModelType` 字段被忽略，模型固定为轮式移动机械臂。

## 上游来源

本包初期从 OCS2 `ocs2_mobile_manipulator` 移植了部分源码，保留了原
BSD-3-Clause 版权声明；转换到 `wbmm_ocs2` 命名空间并删除无关模型分支。
后续会继续把项目特有代价和约束补齐并保持 vendor 尽量接近上游。
