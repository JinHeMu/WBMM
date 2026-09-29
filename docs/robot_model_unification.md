# WBMM 机器人模型与运动学的当前路线

## 文件职责

| 文件 | 职责 |
|---|---|
| `wbmm_robot_model/src/urdf_description.cpp` | 用 urdfdom 读取 URDF，生成 link、joint、visual、collision 的 `RobotDescription` |
| `wbmm_robot_model/src/robot_model_config.cpp` | 定义并校验受控关节顺序、锁定关节、底盘参考 link、末端与碰撞分组 |
| `wbmm_robot_model/src/robot_model_config_loader.cpp` | 可选：从 YAML 读取上述配置 |
| `wbmm_robot_model/src/robot_model_builder.cpp` | 合并描述和配置，生成碰撞球及 `RobotModelDescription` |
| `wbmm_pinocchio/src/kinematic_model.cpp` | 从同一份 URDF 文本建立只读 Pinocchio Model，并映射状态关节与索引 |
| `wbmm_pinocchio/src/kinematics_data.cpp` | 持有可变 Pinocchio Data；给一个状态计算 FK，按需计算 Jacobian |
| `wbmm_pinocchio/src/pinocchio_robot_model.cpp` | 将以上运动学暴露为 `wbmm::core::RobotModel` 接口 |
| `wbmm_pinocchio/src/frame_kinematics.cpp` | 读取任意 frame 的位姿、Jacobian |
| `wbmm_pinocchio/src/sphere_kinematics.cpp` | 用同一份 FK 计算碰撞球球心及 Jacobian |

这些 Pinocchio 文件已经在同一个 `src` 文件夹。三个核心文件分别负责只读模型、可变计算数据和项目接口，保留为独立实现，便于定位计算与接口问题。

## URDF 到运动学

```text
URDF 文件或 XML 文本
  -> RobotDescriptionLoader::fromFile/fromXml (urdfdom)
  -> RobotDescription + RobotModelConfig (默认值、ROS 参数或可选 YAML)
  -> validate(config, description)
  -> buildRobotModelDescription(...) [需要碰撞球的规划器]
  -> KinematicModel::create(...) [Pinocchio 从 description.xml 建模]
  -> KinematicsData::update(state)
  -> FrameKinematics / SphereKinematics / PinocchioRobotModel
```

`urdfdom` 与 Pinocchio 分别解析同一份 URDF 文本，二者没有共享解析树。共享的是 URDF 内容、配置和受控关节顺序。带 `mimic` 的 URDF 在加载阶段明确报错；当前机器人不使用它。

运行时，`KinematicModel` 可在同进程内共享；`KinematicsData` 每个使用者各持有一份，不跨线程共享可变缓存。每次 `update(state)` 计算位姿。只有需要线性化时才指定 `UpdateMode::kJacobians`。

## 各消费者

- **规划器**：从 URDF 和 ROS 参数构造 `RobotModelDescription`，其碰撞球模型供 `EnvironmentCollisionChecker`、轨迹优化器使用；Pinocchio 的 `SphereKinematics` 批量取球心。MINCO 轨迹变量上的解析梯度仍由 `wbmm_collision/whole_body_trajectory_kinematics.cpp` 负责，和状态 FK 的自变量不同。
- **可视化**：读取 URDF 与 ROS 参数，`FrameKinematics` 生成各 link 位姿。
- **力控**：读取 URDF，用 `PinocchioRobotModel::forwardKinematics` 获取当前 TCP 位姿；导纳生成 7D 末端目标并发给 OCS2。旧的 `WholeBodyKinematics` 9D 逆解和 `base_share` 路线已移除。
- **OCS2**：`WbmmInterface` 目前仍按文件路径建立自己的 `PinocchioInterface`，使用 OCS2 原生的末端运动学与求解器数据。这条链没有接入上面的共享 `RobotModelDescription`，因此不能说整个仓库只解析一次 URDF。

当前球模型只消费 URDF 中的 sphere collision；非球几何记录到 `skipped_geometries`。配置不提供尚未实现的 primitive 球近似和自碰撞排除字段，防止出现“配置接受了但算法没用”的误解。
