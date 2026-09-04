# WBMM 启动入口与后端边界

本文是系统启动入口的所有权清单。完整系统组合只放在
`tracer_jaka_bringup`；算法包、任务包、驱动包和仿真包只提供节点、配置和可复用的
单后端启动文件。

## 1. 唯一组合入口

### MuJoCo 仿真

| 目的 | 命令 |
|---|---|
| 定位与二维 SLAM | `ros2 launch tracer_jaka_bringup slam_sim.launch.py` |
| OCS2 主闭环 | `ros2 launch tracer_jaka_bringup ocs2_sim.launch.py` |
| 静态 ESDF 校验 | `ros2 launch tracer_jaka_bringup ocs2_esdf_validation.launch.py` |
| nvblox 传感器与建图 | `ros2 launch tracer_jaka_bringup mujoco_nvblox_mapping.launch.py` |
| 已导出 ESDF 控制 | `ros2 launch tracer_jaka_bringup ocs2_mapped_esdf_control.launch.py` |
| 任务桌与六维力 | `ros2 launch tracer_jaka_bringup mujoco_task_table.launch.py` |
| ros2_control 后端 | `ros2 launch tracer_jaka_bringup mujoco_ros2_control.launch.py` |
| TA-WBMP 闭环 | `ros2 launch tracer_jaka_bringup ta_wbmp_mujoco_closed_loop.launch.py` |
| 六维导纳/恒力测试 | `ros2 launch tracer_jaka_bringup force_control_mujoco_test.launch.py` |
| 擦拭基线闭环 | `ros2 launch tracer_jaka_bringup wipe_sim.launch.py` |
| 正面板擦拭 | `ros2 launch tracer_jaka_bringup wipe_front_board_sim.launch.py` |
| 桌面擦拭 | `ros2 launch tracer_jaka_bringup wipe_table_sim.launch.py` |
| MoveIt | `ros2 launch tracer_jaka_bringup moveit.launch.py backend:=sim` |

### 真实机器人

| 目的 | 命令 | 默认安全状态 |
|---|---|---|
| 传感器、EKF 与 SLAM | `ros2 launch tracer_jaka_bringup real_slam.launch.py` | 不含任务控制 |
| D435 / D455 | `d435_real.launch.py` / `d455_real.launch.py` | 只发布传感器 |
| 固定地图定位 | `ros2 launch tracer_jaka_localization localization_real.launch.py` | 只定位 |
| REMANI + OCS2 固定地图链路 | `ros2 launch tracer_jaka_bringup remani_mpc_localized_real.launch.py` | 必须显式提供 ESDF；命令门默认关闭 |
| REMANI + OCS2 在线 SLAM 链路 | `ros2 launch tracer_jaka_bringup remani_mpc_real.launch.py static_esdf_file:=<现场地图.npz>` | 必须显式提供 ESDF；命令门默认关闭 |
| 擦拭实机链路 | `ros2 launch tracer_jaka_bringup wipe_real_pipeline.launch.py` | `jaka_read_only:=true`、`command_output_enabled:=false`、`safety_release:=false`、力控关闭 |
| MoveIt | `ros2 launch tracer_jaka_bringup moveit.launch.py backend:=real jaka_read_only:=true` | 只规划，不执行 |

## 2. 包职责

- `tracer_jaka_mujoco`：只实现 MuJoCo bridge、传感器、MJCF 场景和
  `bridge.launch.py`；不得依赖实机驱动、定位或任务包。
- `tracer_jaka_ocs2`、`whole_body_force_control`、`ta_wbmp`：只拥有算法节点、
  参数和局部演示；不得组合 MuJoCo 或实机驱动。
- `wipe_planner`：只拥有擦拭任务节点和规划预览；系统闭环入口由 bringup 持有。
- `tracer_jaka_localization`：拥有定位 launch、二维地图和定位配置，不依赖 bringup。
- `jaka_sdk_vendor`：唯一安装 JAKA SDK 头文件和动态库；JAKA 驱动只能通过
  `find_package(jaka_sdk_vendor)` 使用 SDK。

## 3. 仿真与实机接口合同

两种后端必须向上层提供同名 ROS 接口：

| 接口 | 约定 |
|---|---|
| 机器人状态 | `/joint_states`、`/odometry/filtered` |
| 底盘命令 | `/cmd_vel` 或由入口显式映射到唯一底盘控制器 |
| 机械臂命令 | `/arm_controller/commands` 或 `FollowJointTrajectory`，同一时刻只能有一个所有者 |
| 六维力 | `/fts_broadcaster/wrench`，`frame_id` 必须是传感器坐标系 |
| TF | `map->odom`、`odom->base_footprint`、机器人固定/关节 TF 分属唯一发布者 |

`backend:=sim|real` 只在组合层选择后端，不允许算法代码用工作空间绝对路径或根据
机器名判断部署环境。实机入口必须默认只读；闭环运动和力控均需独立显式解锁。

## 4. 新入口准入规则

新增 launch 前先判断：如果它同时启动两个及以上层（例如仿真 + 控制、驱动 +
定位、任务 + 控制），必须放入 bringup。如果只启动一个包内部节点，可留在该包。
变更后至少执行 launch Python 语法检查、package.xml 依赖检查和目标包构建；仿真运行
与实机运行分别记录，不能用“构建成功”代替闭环验证。
