# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概览

WBMM（Whole-Body Mobile Manipulation）—— ROS 2 Humble 工作区，Tracer 差速底盘 + JAKA Zu5 六轴臂的全身移动操作研究平台。

全链路：定位（EKF + slam_toolbox）→ MuJoCo 仿真或实机 → nvblox 三维 ESDF 建图 → REMANI 全局规划 → OCS2 MPC/MRT 控制。应用层以"擦拭/抛光"为任务，新增了任务感知规划（ta_wbmp）、连续接触参考规划（wipe_planner）、统一可视化（wbmm_visualization）与恒力/导纳控制（whole_body_force_control）。

仿真与实机共用同一套 ROS 2 话题接口：先在 MuJoCo 验证，再迁移实机。

## 技术合同约定（最高优先级）

本仓库以 `docs/architecture.md`（架构技术合同）与 `docs/frames.md`（坐标系技术合同）为基准：
**所有新功能、重构、Bug 修复必须先对照这两份文档再修改代码；实现与文档不一致时，以文档为基准更新代码或文档。**
下文为两份合同核心约定的浓缩版，完整约束以文档为准。若实现与合同不一致，先修订代码或按流程修订文档，而不是无视合同。

### 主链架构（当前状态）

```text
task.yaml（统一任务 YAML）
  -> ta_wbmp: TaskTrajectoryGenerator -> TaskAwarePlanner（生成 Plan: waypoints + q_pre + task_targets/normals）
      -> ExecutionCoordinator
           |-- /remani_planner/whole_body_goal (q_pre)  -> REMANI 碰撞感知导航
           |-- NAVIGATING: REMANI bridge 持有 MPC 参考  -> OCS2 MPC
           |-- /remani_planner/set_task_execution        -> REMANI TASK_EXEC（显式交权）
           |-- TASK_EXEC: Coordinator 持有 MPC 参考      -> OCS2 MPC/MRT（默认 20 Hz）
           |-- 可选 wrench -> whole_body_force_control -> 修正后 9D 参考
  -> MuJoCo / 实机闭环
```

- **同一时刻只有一个 MPC 参考所有者**：`NAVIGATING` = REMANI bridge；`TASK_EXEC` = TA-WBMP Coordinator。
- `wipe_planner` **已退出主链**（暂不物理删除），保留为接触安全回归基线；新功能优先落在 ta_wbmp；实机验证入口 `wipe_real_pipeline.launch.py` 仍可运行。

### 状态与控制约定（顺序不可变）

```text
x = [x_b, y_b, yaw_b, q1..q6]      # 9D 全身状态；x/y/yaw 为底盘相对规划坐标系
u = [v_b, omega_b, qdot1..qdot6]   # 8D 输入；v_b / omega_b 定义在 base_footprint / base_link local frame
```

- 规划坐标系：MuJoCo/主链默认 `odom`，实机定位可用 `map`，但代码要求 `world_frame == planner_frame`。
- 该 9D 状态同时用于 `/wbmm/*` 可视化契约、`ocs2_msgs/MpcObservation`、`sensor_msgs/JointState` 与各规划器内部 `Eigen::VectorXd`。

### 坐标系（完整帧树见 docs/frames.md）

```text
map -> odom -> base_footprint -> base_link -> jaka_base_link -> Link_1..Link_6 -> ... -> tool0
```

固定变换（代码依赖）：`base_footprint→base_link` z=0.147；`base_link→jaka_base_link` xyz=[0,0,0.221]、rpy=[0,0,-1.57]；`Link_6→Link_6_45` rpy=[0,0,-0.7854]；`tool0_and_camera_link→tool0` z=0.27。EE 统一用 `tool0`。

### 命名与力约定

- **先写 frame，再写物理量**：`p_ee^odom` 永远优于裸 `p_ee`；公式中的 Jacobian / 速度 / 力必须标注"相对哪个 frame"（`V_ee^odom = J_wb^odom(x) u`）。
- 力传感器 `/fts_broadcaster/wrench` 的 `frame_id` 是传感器自身系（`F^sensor`）；当前力控默认 `force_axis=z` + `absolute_force=true`，即 `F_control = |F_z^sensor|`；禁止未标注 frame 就混用 `F^sensor` / `F^ee` / `F^world`。
- 任务表面几何（YAML `surface.center / normal_into_room / axis_u / axis_v`）与 `task_normal` 都在规划坐标系表达；力控修正方向使用时再转换 `n_task^control = R_odom^control * n_task^odom`。

### ta_wbmp 主链关键接口

服务：`/ta_wbmp/execution/start`（Trigger）、`/ta_wbmp/execution/enable_force_control`（SetBool）、`/remani_planner/set_task_execution`（SetBool）；话题：`/remani_planner/whole_body_goal`（q_pre）、`/remani_planner/fsm_state`、`/ta_wbmp/execution/status`、`/ta_wbmp/execution/force_state`。

## 构建

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash

# 单包快速重建（日常最常用）
colcon build --packages-select <pkg> --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 全量（依赖全时）
colcon build --symlink-install --packages-up-to \
  tracer_jaka_description tracer_jaka_mujoco tracer_jaka_ocs2 remani_planner \
  wipe_planner grid_map hipnuc_imu lakibeam1 tracer_jaka_bringup \
  tracer_jaka_localization
```

- 构建产物在 `build/`、`install/`，日志在 `log/`（均 .gitignore）；使用前必须 `source install/setup.bash`；C++17。
- **`path_searching` 是独立 ament 包**：只编 `remani_planner` 不会重建它，需单独 `colcon build --packages-select path_searching`。它曾有 `#define inf 1 >> 30` 笔误（应为 `1 << 30`，否则 inf=0），会造成近距离大转角预接触目标在 `GEN_NEW_TRAJ` 阶段提前返回空轨迹并段错误。
- OCS2 首次启动做自动微分代码生成，耗时显著更长；修改 `task.info` 中动力学/运动学配置后需清理自动生成目录（如 `/tmp/ocs2_tracer_jaka_conservative/auto_generated`）再运行。
- 测试：`colcon test --packages-select <pkg> && colcon test-result --verbose`（ta_wbmp / wbmm_visualization 等包带 GTest）。
- 无头 MuJoCo：`export MUJOCO_GL=egl`；无网络 sandbox 下加 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`、`ROS_LOCALHOST_ONLY=1`。

## 运行入口

快速命令速查：`docs/QUICKSTART.md`（第一入口）；完整数据流与启动方式：根 README.md。

- **完整仿真**（定位+SLAM+REMANI+OCS2+RViz）：`ros2 launch tracer_jaka_bringup ocs2_sim.launch.py`（可选 `start_slam/start_remani:=false` 裁剪）
- 局部仿真：bringup 的 `slam_sim.launch.py`（定位/SLAM）、`mujoco_task_table.launch.py`（任务桌 + `fts_broadcaster` 六维力）、`ocs2_esdf_validation.launch.py`（静态 ESDF 校验）
- 擦拭预览（**不启动任何控制器**）：`wipe_planner wipe_plan_preview.launch.py`、`wipe_real_plan_preview.launch.py`、`ta_wbmp task_pipeline.launch.py scenario:=table|blackboard|ras`
- ta_wbmp 主链：`execution_pipeline.launch.py`（安全准备，`execution_enabled:=false` 时 `/ta_wbmp/execution/start` 拒绝执行）、bringup 的 `ta_wbmp_mujoco_closed_loop.launch.py force_control_enabled:=true|false`（MuJoCo 闭环）
- 实机：`tracer_jaka_bringup remani_mpc_localized_real.launch.py`（定位+REMANI+OCS2）、`tracer_jaka_bringup wipe_real_pipeline.launch.py`

**禁忌与安全**：
- `tracer_jaka_bringup/wipe_sim.launch.py` 是 MuJoCo/空 ESDF 验证组合，**严禁在实机运行**（会启动另一套 OCS2/REMANI，与实机链路重复、参考所有权冲突）。
- `wipe_real_pipeline.launch.py` 默认双安全门关闭（`jaka_read_only:=true`、`command_output_enabled:=false`、无力控）；只有显式打开后才输出运动。
- MuJoCo 力传感器来自接触求解器，接触瞬间可能有单帧力跳变，可能误触发预接触状态机（卡住排查见 wipe_planner README）。

## 架构

### 数据流（plan-then-track）

```
REMANI (vendor, 全局规划)                    8D  flat output: [x, y, q1..q6]
  → remani_to_ocs2_reference_bridge          重建 yaw、零速度退化处理、起点锚定
  → /mobile_manipulator_mpc_target           9D 状态 [x, y, yaw, q1..q6] + 8D 输入 [v, ω, q̇1..q̇6]
  → OCS2 MPC (SLQ/DDP) → MRT                 /mobile_manipulator_mpc_policy
  → Twist (底盘) + Float64MultiArray (臂)
```

**全仓库唯一状态约定（改任何规划器都必须保持一致）**：
`x = [base_x, base_y, base_yaw, joint_1..joint_6]`（9D），输入 `[base_v, base_yaw_rate, joint_velocity_1..6]`（8D）。joint order/命名贯穿 REMANI bridge、ta_wbmp、wipe_planner、wbmm_visualization、OCS2。

### src/ 目录职责（按"职责 + 部署场景"重构）

| 目录 | 内容 |
|---|---|
| `vendor/` | 上游/第三方：`ocs2_ros2`、项目 fork `remani_planner`、唯一 JAKA 二进制 SDK 包 `jaka_sdk_vendor`；差异见 `docs/vendor_patches.md` |
| `interfaces/` | `tracer_jaka_interfaces` |
| `robot/` | `tracer_jaka_description`、moveit_config |
| `drivers/` | 底盘 ugv_sdk/tracer_*、JAKA（jaka_hardware_interface 等）、IMU、Lakibeam、`force_torque/tracer_jaka_ft_tools` |
| `algorithms/` | `control/tracer_jaka_ocs2`、`control/whole_body_force_control`、`planning/ta_wbmp`、`visualization/wbmm_visualization`、`force_control/contact_control_core` |
| `perception/` | grid_map、esdf_simple_nav、`my_nvblox_bringup`（需同步进 Isaac ROS Docker）、localization |
| `applications/wiping/` | `wipe_planner`（任务应用） |
| `simulation/` | `tracer_jaka_mujoco`（MuJoCo bridge、模型和场景，不含实机或系统组合） |
| `bringup/` | `tracer_jaka_bringup`（仿真/实机顶层组合的唯一所有者；见 `docs/launch_ownership.md`） |

### 规划/控制模块（主链按技术合同）

- **ta_wbmp**（algorithms/planning）：**主链规划器**。TaskTrajectoryGenerator → TaskAwarePlanner（候选构型 IK + manipulability + SE2 grid A\* 导航）→ ExecutionCoordinator（导航、9D 到达检测、TASK_EXEC 交权、OCS2 滚动参考）。`task_pipeline.launch.py` 默认只规划+可视化；`execution_pipeline.launch.py` / `mujoco_closed_loop.launch.py` 连接实际链路。接口/扩展点见其 DESIGN.md、TASK_SCHEMA.md。
- **REMANI + bridge**（vendor + tracer_jaka_ocs2）：导航阶段参考所有者。到达 `q_pre` 后 REMANI 显式进入 `TASK_EXEC`（/remani_planner/fsm_state），导航/重规划/碰撞检查全部停止。
- **wipe_planner**（applications）：旧主链实现，**已退出主链**，保留为接触安全回归基线。hybrid A\*（base_x, base_y, base_yaw，差速运动原语）+ 热启动 arm IK + 导纳/恒力修正 + 虚拟时间 tau 进度管理；其 README 仍记录话题全表与预接触卡住排查方法。
- **whole_body_force_control**：恒力/导纳/全身力跟随数值模型 + MuJoCo 闭环测试，被 ta_wbmp / wipe_planner 调用。
- **tracer_jaka_ocs2**：OCS2 MPC/MRT/目标节点与 REMANI bridge（最老的控制链路，仍承担 MPC 求解）。

### 统一可视化契约（wbmm_visualization）

规划器只发布**数据契约**，`wbmm_viz_node` 统一渲染（消除 REMANI / TA-WBMP / WipePlanner 三套重复可视化）。契约全部用标准消息：

- `/wbmm/whole_body_trajectory`：9D `JointTrajectory`，joint_names 固定 `base_x, base_y, base_yaw, joint_1..6`（transient_local）
- `/wbmm/phase_schedule`：`"<t0> PHASE0;<t1> PHASE1;..."`，轨迹发布后立即发布
- `/wbmm/live_state`（JointState）、`/wbmm/live_phase`（String）
- 输出 `/wbmm/robot_mesh`、`/wbmm/time_segments`、`/wbmm/playback`、`/wbmm/base_path`、`/wbmm/ee_path`

phase 共享词汇：`NAVIGATE / PRECONTACT_ALIGN / PRECONTACT_APPROACH / TASK_CONSTRAINED`（WipePlanner 的 `waiting_navigation / remani_navigation / wipe_planning / continuous_contact_wiping`、`completed` 白 / `failed` 红 自动映射到标准颜色）。**单数据源约定：每条管线只能有一个轨迹源，同一 trajectory_topic 不能有两个发布者。**

### TF 与话题所有权（唯一性约束）

- TF：slam_toolbox 独有 `map→odom`；robot_localization 独有 `odom→base_footprint`；robot_state_publisher 负责 `base_footprint→arm`。
- `/mobile_manipulator_mpc_target` 只能有一个发布者：`wipe_real_pipeline` 强制标准 bridge `start_bridge:=false`，由 WipePlanner 独家中继（观察状态 odom→map，把 MPC 参考 base 位姿 map→odom；机械臂关节与 body-frame 输入不转换）。
- 力传感器：仿真与实机统一走 `/fts_broadcaster/wrench`。

### 力控要点

- 预接触状态机：`force_contact_detected_` / `force_hard_stop_`（**35 N 硬限位锁存**，复位需显式调 `/wipe_planner/enable_force_control` SetBool true）/ `initial_force_settled_`。
- 单帧尖峰拒绝参数：`force_spike_rejection_n`(8.0)、`force_spike_confirm_samples`(3)、`force_contact_confirm_samples`(3)、`force_hard_limit_confirm_samples`(3)——MuJoCo 接触瞬间的力跳变不会进入判断。
- 状态排查话题：`/wipe_planner/force_control_state`（`active_force_settling` / `over_force_retreat` / `active_force_paused`）、`/wipe_planner/normal_force`、`/wipe_planner/virtual_progress_rate`（=0 即卡住）。

## 其他非显然事项

- 静态地图在 `/home/a/WBMM/maps/`（`site_remani.npz`、`site_mesh.ply`、`site_2d.yaml/pgm`）；nvblox 导出/rosbag 产物在 `bag_export/`、`bags/`、`results/`（不提交 git，克隆后需自行准备）。
- nvblox 相关代码（`my_nvblox_bringup`）因 CUDA 依赖需先 `scripts/sync_to_isaac_ros_ws.sh` 同步进 Isaac ROS Docker 才可构建运行（跨 `ros_domain_id:=20` 与主机通信）。
- 任务几何以 YAML 为唯一事实源（如 `wipe_task_real_front.yaml` 的 `contact.offset`，负值为板前空描、正值为入板内规划，**禁止正偏移**）；RViz 中的桌面/白板 Marker 只是示意图，不代表规划器碰撞体。

## 文档导航

- 根 README.md —— 中文总览、数据流图、关键接口表
- `docs/QUICKSTART.md` —— 启动命令速查（仿真/实机/建图/排查）
- ta_wbmp：本包 README + `DESIGN.md` + `TASK_SCHEMA.md`
- wipe_planner：本包 README（话题全表、预接触卡住排查、实机链路）
- `docs/architecture.md`、`docs/frames.md` —— **架构与坐标系技术合同**（必读，见上文"技术合同约定"）
- `docs/MUJOCO_NVBLOX_REMANI_PIPELINE.md`、`docs/实机部署指南.md`、`docs/擦黑板恒力控制设计.md`
