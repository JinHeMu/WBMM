# TA-WBMP — 通用 Task-Aware Whole-Body Motion Pipeline

中文名：**面向任务约束的移动机械臂全身运动规划器**。

这是“创新点 2：任务驱动的移动机械臂名义全身运动规划”的可运行
**C++17 / ROS 2 Humble** demo。它把输入从单一 `Goal Pose` 提升为接触任务：表面、任务轨迹、预接触
间隙、末端法向、全身构型裕量和导航障碍。默认预览节点只规划和可视化，**不会启动
机器人、REMANI、OCS2 或发布任何控制指令**。独立执行协调器可连接实际 REMANI/OCS2 Topic，但其执行门默认关闭。

当前工程已收敛为三个稳定层次：

```text
TaskTrajectoryProvider / TaskTrajectoryGenerator
  → TaskAwarePlanner（任务约束全身规划与候选选择）
  → REMANI q_pre（导航阶段 bridge 持有 MPC 参考）
  → q_pre 到达后显式交权给 OCS2 MPC（可选 whole_body_force_control）
```

完整接口、扩展点和验证边界见 [DESIGN.md](DESIGN.md)，统一 YAML 字段见
[TASK_SCHEMA.md](TASK_SCHEMA.md)。

本包保留规划预览与算法内部 launch；连接 MuJoCo/REMANI/OCS2 的闭环组合入口为
`tracer_jaka_bringup/ta_wbmp_mujoco_closed_loop.launch.py`。

## 已实现的规划链

```text
Task YAML
  -> 完整擦拭路径采样
  -> X_task 候选全身构型搜索
       末端位置/法向 IK
       关节限位裕量
       全路径 manipulability
       未来任务平滑性评分
  -> 选择预接触全身状态
  -> A* 障碍规避导航 + 差速底盘 rotate-drive 参数化
  -> 固定底盘机械臂对齐
  -> 沿表面法向预接触
  -> 末端受约束全身擦拭轨迹
  -> 逐项约束验证与 ROS 2 发布
```

状态与现有 REMANI/OCS2 保持一致：

```text
x = [base_x, base_y, base_yaw, joint_1, ..., joint_6]  (9D)
```

与现有代码的关系：

- 参考 REMANI 的“搜索前端 + 连续轨迹后端”分层和 9D 全身状态；
- 参考 `wipe_planner` 的壁面法向 IK、预接触/接触分段、未来擦拭可达性检查；
- 通用力控数值模型由独立 `whole_body_force_control` 包提供；
- Coordinator 已把 `q_pre` 发给 REMANI，并在接管后把名义轨迹或力修正后的
  9D 轨迹直接发送给 OCS2。

主要 C++ 文件：

- `include/ta_wbmp/task_trajectory.hpp`：通用任务轨迹及生成器接口；
- `include/ta_wbmp/cost.hpp`：可注入候选代价接口；
- `include/ta_wbmp/extensions.hpp`：状态有效性与导航代价扩展接口；
- `include/ta_wbmp/planner.hpp`：规划数据结构和 `TaskAwarePlanner` 接口；
- `src/task_trajectory.cpp`：raster、RAS和通用waypoint轨迹生成；
- `src/planner.cpp`：候选任务构型、Pinocchio IK、A*、时间参数化和约束验证；
- `src/demo_node.cpp`：ROS 2 C++ 节点、轨迹/诊断/RViz 发布；
- `src/scenario_runner.cpp`：离线实验CSV/YAML输出；
- `src/execution_coordinator_node.cpp`：REMANI 导航、9D 到达检测、TASK_EXEC 所有权切换和 OCS2 滚动参考；
- `test/test_planner.cpp`：完整任务规划 GTest。

## 通用三场景

```bash
ros2 launch ta_wbmp task_pipeline.launch.py scenario:=table
ros2 launch ta_wbmp task_pipeline.launch.py scenario:=blackboard
ros2 launch ta_wbmp task_pipeline.launch.py scenario:=ras
```

对应配置为 `table_wipe.yaml`、`blackboard_wipe.yaml` 和
`ras_drawing.yaml`。三个场景经过同一个任务生成、候选选择、全身规划和执行契约流程。
RAS 路径是一笔连续完成的字符轨迹，几何包络严格为宽 `0.90 m`、高 `0.60 m`。

动态显示擦桌任务的完整 `NAVIGATE -> PRECONTACT_ALIGN -> PRECONTACT_APPROACH -> TASK_CONSTRAINED` 过程，并保留 REMANI 风格的时间窗整机快照（仅规划和 RViz，不控制机器人）：

```bash
ros2 launch ta_wbmp table_rviz_reproduction.launch.py
```

该视图由 `wbmm_viz_node` 直接读取 URDF 视觉网格，按 15 秒时间窗显示
半透明整机快照（`/wbmm/time_segments`）并循环播放整条轨迹
（`/wbmm/playback`）。可用 `snapshots:=2` 调整每段快照密度，用
`playback_rate:=1.0` 调整回放速度；默认以 2 倍名义速度循环播放完整任务。
RViz 中的木色桌板/桌腿用于说明作业场景，擦拭接触面仍严格使用
`table_wipe.yaml` 的 `0.16 m × 0.10 m` 小区域；桌子外观 Marker 本身不等同于
规划器中的环境碰撞体。
如果要匹配参考截图中较大的栅格构图，而不是当前小桌面任务几何，可运行：

```bash
ros2 launch ta_wbmp table_rviz_reproduction.launch.py \
  task_config:=wipe_demo.yaml snapshots:=24
```

执行协调器的安全准备模式：

```bash
ros2 launch ta_wbmp execution_pipeline.launch.py \
  scenario:=blackboard execution_enabled:=false
```

此模式会完成规划并进入 `READY`，但 `/ta_wbmp/execution/start` 会拒绝执行，且不会写实际 REMANI/OCS2 目标。实际联调方法、TF 和 Topic 唯一所有权要求见 [DESIGN.md](DESIGN.md)。

物理 MuJoCo 桌面组合入口（不启动 `wipe_planner`）：

```bash
# 纯 MPC 位置跟踪
ros2 launch tracer_jaka_bringup ta_wbmp_mujoco_closed_loop.launch.py force_control_enabled:=false

# MPC + 恒力修正
ros2 launch tracer_jaka_bringup ta_wbmp_mujoco_closed_loop.launch.py force_control_enabled:=true
```

当前该入口会启动仿真、REMANI、OCS2 和 Coordinator，但 Coordinator 的生产
执行门会因共享 REMANI/ESDF checker 尚未接入而 fail-closed，不会发送导航 goal。
因此它目前只能用于组合/接口检查，不能称为 L3/L4 闭环复现命令。完成共享
环境 adapter 并使 `checksEnvironment()` 返回 `true` 后，才允许恢复执行验收。

## 构建与运行

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ta_wbmp
source install/setup.bash
ros2 launch ta_wbmp demo.launch.py
```

与 REMANI example 一致，独立 demo 使用 `world` 作为 RViz Fixed Frame，并把
底盘和每个机械臂 Mesh 的绝对位姿直接发布到 MarkerArray，不依赖 `odom`、
`robot_state_publisher` 或机械臂 link TF。

运行圆柱曲面全覆盖测试：

```bash
ros2 launch ta_wbmp curved_demo.launch.py
```

曲面场景使用半径 `1.10 m`、圆周角 `[-0.55, 0.55] rad`、高度
`[0.72, 1.18] m` 的圆柱面片。规划器生成 5 条相邻蛇形覆盖带，底盘沿曲面
等距线运动。按有效工具宽度 `0.13 m` 设计，覆盖带间距 `0.115 m`，相邻带
重叠 `0.015 m`；底盘航向包含切向偏置补偿，机械臂逐点求解 IK，工具轴
逐点跟踪圆柱局部法向，而不是使用固定平面法向。

无图形运行并查看约束报告：

```bash
ros2 launch ta_wbmp demo.launch.py use_rviz:=false
ros2 topic echo /ta_wbmp/report --once
```

## 输出

规划器只发布数据契约，可视化统一由 `wbmm_visualization` 的
`wbmm_viz_node` 完成（launch 中内嵌，输入经 remap 接到 `/ta_wbmp/*`）：

- `/ta_wbmp/whole_body_trajectory`：9D `trajectory_msgs/JointTrajectory`
  数据契约（`base_x/base_y/base_yaw/joint_1..6`）；
- `/ta_wbmp/phase_schedule`：按点 phase 时间表（`std_msgs/String`，
  格式 `"<t0> <PHASE0>;<t1> <PHASE1>;..."`），轨迹发布后原子跟随；
- `/ta_wbmp/markers`：任务面、障碍、阶段轨迹和法向约束（不含整机快照）；
- `/ta_wbmp/phases`：规划阶段顺序；
- `/ta_wbmp/report`：候选数和每项约束的 JSON 验证结果。

`wbmm_viz_node` 统一渲染（固定 `/wbmm/*` 输出）：

- `/wbmm/robot_mesh`：当前整机 Mesh MarkerArray（命名空间 `current_robot`，
  使用 Mesh 自带材质）；
- `/wbmm/time_segments`：不同颜色的名义时间窗与整机网格快照；
- `/wbmm/playback`：当前时刻、活动时间窗和动态整机构型；
- `/wbmm/base_path` / `/wbmm/ee_path`：底盘与末端路径。

RViz 颜色：灰色为导航，黄色为预接触构型对齐，橙色为法向接近，绿色为
受约束任务段，蓝色箭头为末端法向约束。

## 时间分段显示

`wbmm_viz_node` 与 REMANI 的 `front_end_mm_mesh_vis` /
`back_end_mm_mesh_vis` 一样，从 URDF 视觉模型生成整机
`MESH_RESOURCE MarkerArray`。默认把 127 秒名义轨迹按 15 秒划分为多个
颜色不同的时间窗：

- `/wbmm/time_segments`：静态显示每个时间窗的底盘线、末端线、时间标签和
  半透明整机快照；
- `/wbmm/playback`：动态显示当前活动时间窗、已经播放的轨迹、当前整机
  构型和 `t / segment / phase` 标签；
- 默认以 5 倍速循环播放，不会向机器人发布控制指令。

整机模型使用 URDF 视觉网格并开启 Mesh 自带材质，每个回放时刻用固定
Marker ID 覆盖上一帧；RViz 配置采用 `world` Fixed Frame，因此不会等待
`odom` TF。所有可视化发布点仅在存在订阅者时执行 FK/marker 构造。

可在 `launch/demo.launch.py` 的 `wbmm_viz` 节点参数中调整：

```text
time_segment_duration  每段名义时间，默认 15.0 s
segment_snapshots      每段整机网格快照数，默认 2
playback_rate          名义时间/真实时间倍率，默认 5.0
playback_period        MarkerArray 刷新周期，默认 0.10 s
playback_loop          到终点后是否循环
```

## 当前 demo 边界

这是 C++ 名义规划验证版：导航障碍采用 2D 圆柱模型，机械臂使用 Pinocchio
运动学、关节限位和 manipulability；尚未把 ESDF 全身距离、REMANI 的
Kino-A*/MINCO/L-BFGS、环境碰撞对和 OCS2 在线执行并入本包。接口和阶段已
为这些后端保留，下一步应以 ESDF 代替圆柱障碍，并把任务约束加入 REMANI
轨迹优化代价/约束，而不是让 demo 的 A* 取代 REMANI。
