# MuJoCo → nvblox ESDF → REMANI → OCS2 完整仿真通道

> 快速命令速查见 [QUICKSTART.md](QUICKSTART.md)。

## 1. 通道结构

```text
MuJoCo 障碍物场景
  ├─ D455 RGB-D ───────────────┐
  ├─ 2D LaserScan ─ slam_toolbox ─ /map
  ├─ wheel odom + IMU ─ EKF ─ /odometry/filtered
  └─ TF / joint_states         │
                               ▼
Isaac ROS Docker: nvblox TSDF/ESDF
  ├─ mujoco_demo_map.nvblx
  ├─ mujoco_demo_remani_esdf.npz
  └─ mujoco_demo_2d.yaml + pgm
                               ▼
REMANI 前端规划 → /planning/trajectory
                               ▼
REMANI-to-OCS2 bridge → TargetTrajectories
                               ▼
OCS2 MPC/MRT → 底盘与机械臂命令 → MuJoCo
```

规划阶段的 `task_esdf_only.info` 已关闭 XML 人工障碍物约束。REMANI 的外部
环境碰撞依据只有 nvblox 导出的 NPZ；MuJoCo XML 仍保留障碍物，用来验证真实
控制过程中会不会撞上实体。

## 2. 演示场景

场景文件：

```text
src/simulation/tracer_jaka_mujoco/models/scene_nvblox_remani_demo.xml
```

- 中央箱体：让底盘不能直线前往目标，必须绕行。
- 上侧圆柱：限制另一侧通道，增强路线选择差异。
- 低横梁与两侧立柱：用于进阶全身规划实验。
- 终点货架：提供更复杂的末端区域和深度特征。

MuJoCo 机器人视觉/碰撞几何使用 group 1/3，环境使用 group 0。D455 和激光都
只采集 group 0，防止 nvblox 把机器人自身融合成障碍物。

## 3. 首次建图与导出

`my_nvblox_bringup` 的 Git 源码现在位于：

```text
/home/a/WBMM/src/perception/my_nvblox_bringup
```

首次运行或源码更新后，先在主机同步到 Isaac ROS 工作空间：

```bash
cd /home/a/WBMM
src/perception/my_nvblox_bringup/scripts/sync_to_isaac_ros_ws.sh
```

然后在 Docker 中重新执行 `colcon build --symlink-install
--packages-select my_nvblox_bringup`。bag、nvblx、npz 和导出的地图不进入 Git。

### 3.1 Docker：先启动 nvblox 与自动导出

先在 Isaac ROS Docker 中运行，让 nvblox 等待相机数据和建图完成信号：

```bash
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch my_nvblox_bringup mujoco_mapping_export.launch.py \
  ros_domain_id:=20 \
  rviz:=true
```

如需改变输出位置，增加 `output_dir:=/workspaces/isaac_ros-dev/bag_export`，或设置
环境变量 `NVBLOX_OUTPUT_DIR`。

### 3.2 主机：再启动 MuJoCo 自动覆盖扫描

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch tracer_jaka_mujoco mujoco_nvblox_mapping.launch.py \
  viewer:=false \
  ros_domain_id:=20
```

该专用建图 launch 默认使用 128×96 RGB-D，避免部分 FastDDS/Docker 环境对
640×480 大样本传输失败；普通仿真和实机 D455 分辨率不受影响。

该启动文件会运行定位、slam_toolbox、D455，并自动执行起点旋转、六个覆盖
航点和终点旋转。完成标志为：

```text
ESDF mapping coverage scan complete. The map can now be exported.
```

导出器从启动时就订阅 `/esdf_mapping_scan_done`，不会在扫描途中保存。导出成功
后整个 Docker launch 自动退出，以免 nvblox 长时间占用显存。
Docker 自动退出后，可在主机建图终端按 `Ctrl-C` 结束 MuJoCo 扫描进程，再进入
规划与控制阶段。

默认输出：

```text
/workspaces/isaac_ros-dev/bag_export/mujoco_demo_map.nvblx
/workspaces/isaac_ros-dev/bag_export/mujoco_demo_remani_esdf.npz
/workspaces/isaac_ros-dev/bag_export/mujoco_demo_2d.yaml
/workspaces/isaac_ros-dev/bag_export/mujoco_demo_2d.pgm
```

必须检查导出日志里的 `observed` 和 `occupied` 都大于 0。若二者为 0，说明
nvblox 没有收到深度帧，此文件不能用于规划。此时先检查：

```bash
ros2 topic hz /camera/d455/depth/image_raw
ros2 topic hz /camera/d455/depth/camera_info
ros2 topic echo /clock --once
ros2 run tf2_ros tf2_echo odom d455_depth_optical_frame
```

两端的 `ROS_DOMAIN_ID`、`RMW_IMPLEMENTATION=rmw_fastrtps_cpp` 必须一致；也不要
同时遗留多组 nvblox 进程，否则会浪费显存并造成话题诊断混乱。
两个专用 launch 还会把 `FASTDDS_BUILTIN_TRANSPORTS` 固定为 `UDPv4`，用于
绕过 host-network Docker 中“端点能发现、图像数据却无法通过”的 SHM 故障。

## 4. 使用保存地图进行规划与控制

建图端和 Docker 均退出后，在主机运行：

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch tracer_jaka_ocs2 mujoco_mapped_esdf_control.launch.py
```

默认自动目标为 `odom: (2.3, -1.1, 0)`。它要求底盘绕过中央箱体，机械臂保持
当前构型，是用于确认“ESDF → REMANI → OCS2 → MuJoCo”闭环的可重复基准。

可自定义目标：

```bash
ros2 launch tracer_jaka_ocs2 mujoco_mapped_esdf_control.launch.py \
  goal_x:=2.6 goal_y:=-1.2 goal_yaw:=0.0
```

也可关闭自动目标，在 RViz 使用 `2D Goal Pose`：

```bash
ros2 launch tracer_jaka_ocs2 mujoco_mapped_esdf_control.launch.py \
  auto_goal:=false
```

RViz 中应同时看到：保存的 2D `/map`、ESDF 障碍物点、REMANI 搜索/优化轨迹、
OCS2 预测轨迹以及 RobotModel。

## 5. 两种实验模式

### 稳定基准模式（默认）

`remani_freeze_manipulator:=true`。前端固定实测机械臂姿态，但仍检查该姿态沿
底盘路径是否与 ESDF 碰撞。此模式重点验证地图坐标、底盘绕障、桥接、MPC 和
MuJoCo 控制。

### 全身规划压力测试

```bash
ros2 launch tracer_jaka_ocs2 ocs2_esdf_validation.launch.py \
  mujoco_model:=$(ros2 pkg prefix tracer_jaka_mujoco)/share/tracer_jaka_mujoco/models/scene_nvblox_remani_demo.xml \
  esdf_file:=/home/a/workspaces/isaac_ros-dev/bag_export/mujoco_demo_remani_esdf.npz \
  map2d_yaml:=/home/a/workspaces/isaac_ros-dev/bag_export/mujoco_demo_2d.yaml \
  remani_freeze_manipulator:=false \
  remani_manipulator_max_vel:=0.35 \
  remani_manipulator_max_acc:=0.70
```

然后在 RViz 给出低横梁后的目标。该模式会恢复机械臂构型采样，适合后续单独
调节 REMANI 全身轨迹的连续性和 MuJoCo 执行器参数，不建议作为首个通道测试。

## 6. 关键接口

| 功能 | 话题/文件 |
|---|---|
| 深度图 | `/camera/d455/depth/image_raw` |
| RGB | `/camera/d455/color/image_raw` |
| 相机内参 | `/camera/d455/{depth,color}/camera_info` |
| 2D 激光 | `/scan` |
| 融合里程计 | `/odometry/filtered` |
| 2D 地图 | `/map` |
| 建图完成触发 | `/esdf_mapping_scan_done` |
| REMANI 目标 | `/goal_pose` |
| REMANI 输出 | `/planning/trajectory` |
| OCS2 目标 | `/mobile_manipulator_mpc_target` |
| 底盘控制 | `/base_controller/cmd_vel` |
| 机械臂控制 | `/arm_controller/commands` |
