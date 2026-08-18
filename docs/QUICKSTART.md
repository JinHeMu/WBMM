# WBMM 快速开始 / 启动命令速查

> 适用工作区：`/home/a/WBMM`
> 适用系统：Ubuntu 22.04 + ROS 2 Humble + Tracer + JAKA Zu5 + MuJoCo + OCS2 + REMANI + nvblox
> 如果当前目录尚未在文件系统层重命名，请把下文中的 `/home/a/WBMM` 替换为实际工作区路径。

本页只保留最常用的启动命令和参数速查。更详细的流程见：

- [MuJoCo → nvblox → REMANI → OCS2 完整通道](MUJOCO_NVBLOX_REMANI_PIPELINE.md)
- [D455 ESDF 仿真与实机运行指南](D455_ESDF_仿真与实机运行指南.md)
- [实机部署指南](实机部署指南.md)
- [工程目录重构设计](工程目录重构设计.md)

## 1. 环境准备

```bash
export ROS_DOMAIN_ID=0          # 多机/多租户时改成统一 ID
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export MUJOCO_GL=egl            # 无头服务器跑 MuJoCo 时使用
```

安装基础依赖：

```bash
sudo apt update
sudo apt install \
  ros-humble-robot-localization \
  ros-humble-slam-toolbox \
  ros-humble-nav2-map-server
```

## 2. 构建

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-up-to \
  tracer_jaka_description \
  tracer_jaka_mujoco \
  tracer_jaka_ocs2 \
  remani_planner \
  grid_map \
  hipnuc_imu \
  lakibeam1 \
  tracer_jaka_bringup

source install/setup.bash
```

如果只改单个包：

```bash
colcon build --packages-select tracer_jaka_ocs2 --symlink-install
```

## 3. 仿真快速启动

### 3.1 完整仿真：定位 + SLAM + REMANI + OCS2

```bash
ros2 launch tracer_jaka_ocs2 ocs2_sim.launch.py
```

在 RViz 中用 `2D Goal Pose` 发送目标。

常用裁剪参数：

```bash
# 不启动 SLAM
ros2 launch tracer_jaka_ocs2 ocs2_sim.launch.py start_slam:=false

# 不启动 REMANI
ros2 launch tracer_jaka_ocs2 ocs2_sim.launch.py start_remani:=false

# 不启动 RViz
ros2 launch tracer_jaka_ocs2 ocs2_sim.launch.py rviz:=false
```

### 3.2 只验证定位与二维 SLAM

```bash
ros2 launch tracer_jaka_mujoco slam_sim.launch.py
```

### 3.3 打磨/加工任务桌

```bash
ros2 launch tracer_jaka_mujoco task_table_sim.launch.py
```

查看仿真六维力：

```bash
ros2 launch tracer_jaka_mujoco task_table_sim.launch.py \
  init_keyframe:=task_contact fts_zero_on_start:=false
ros2 topic echo /jaka_fts_broadcaster/wrench
```

### 3.4 任务/擦拭预览

```bash
ros2 launch wipe_planner wipe_plan_preview.launch.py
```

## 4. MuJoCo → nvblox → REMANI → OCS2 建图闭环

### 4.1 Docker 侧：启动 nvblox 自动导出

```bash
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch my_nvblox_bringup mujoco_mapping_export.launch.py \
  ros_domain_id:=20 rviz:=true
```

### 4.2 主机侧：启动 MuJoCo 自动覆盖扫描

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch tracer_jaka_mujoco mujoco_nvblox_mapping.launch.py \
  viewer:=false ros_domain_id:=20
```

等待 Docker 侧出现：

```text
ESDF mapping coverage scan complete. The map can now be exported.
```

### 4.3 主机侧：使用导出的 ESDF 运行控制闭环

```bash
ros2 launch tracer_jaka_ocs2 mujoco_mapped_esdf_control.launch.py
```

自定义目标：

```bash
ros2 launch tracer_jaka_ocs2 mujoco_mapped_esdf_control.launch.py \
  goal_x:=2.6 goal_y:=-1.2 goal_yaw:=0.0
```

## 5. 实机启动

> 实机自主擦拭前必须先完成 [实机部署指南](实机部署指南.md) 中的安全与标定检查。

### 5.1 底盘 CAN

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0
```

### 5.2 实机定位与二维 SLAM

```bash
source /home/a/WBMM/install/setup.bash

ros2 launch tracer_jaka_mujoco real_slam.launch.py
```

如果驱动已单独启动：

```bash
ros2 launch tracer_jaka_mujoco real_slam.launch.py \
  start_imu:=false start_lidar:=false
```

### 5.3 实机 OCS2 控制雏形

```bash
ros2 launch tracer_jaka_ocs2 ocs2_real.launch.py
```

当前 `ocs2_real.launch.py` 还不是完整擦拭入口，正式上机前应使用统一的
`tracer_jaka_bringup` 实机 launch 替代。

### 5.4 擦拭任务预览

```bash
ros2 launch wipe_planner wipe_plan_preview.launch.py \
  wipe_task_file:=/absolute/path/to/wipe_task_real.yaml
```

## 6. 常用检查命令

```bash
# 话题频率
ros2 topic hz /odometry/filtered
ros2 topic hz /joint_states
ros2 topic hz /jaka_fts_broadcaster/wrench
ros2 topic hz /scan

# TF
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link tool0

# 控制器
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

## 7. 目录速查

| 路径 | 内容 |
|---|---|
| `src/vendor/` | OCS2、REMANI 等上游/第三方源码 |
| `src/interfaces/` | 跨层消息/服务/动作定义 |
| `src/robot/` | `tracer_jaka_description`、MoveIt 配置 |
| `src/drivers/` | Tracer、JAKA、IMU、Lakibeam 等硬件驱动 |
| `src/algorithms/` | OCS2 控制、REMANI 集成、力控算法 |
| `src/perception/` | 定位、建图、ESDF、nvblox |
| `src/applications/` | WipePlanner 等任务应用 |
| `src/simulation/` | MuJoCo 仿真后端 |
| `src/bringup/` | 顶层系统组合（当前为骨架，待收敛 launch） |
| `tools/` | 示例和仓库级脚本 |
| `data/` | 运行数据（默认 gitignore） |
| `docs/` | 架构、部署、建图、算法文档 |
