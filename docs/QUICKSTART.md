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


安装已验证的系统依赖：

```bash
sudo apt update
sudo apt install \
  libzip-dev \
  libompl-dev \
  python3-pip \
  python3-evdev \
  ros-humble-xacro \
  ros-humble-pinocchio \
  ros-humble-robot-localization \
  ros-humble-slam-toolbox \
  ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-nav2-amcl \
  ros-humble-control-msgs \
  ros-humble-controller-manager \
  ros-humble-ros2-controllers

python3 -m pip install --user mujoco mujoco_lidar
```

验证 MuJoCo Python 包：

```bash
python3 -c "import mujoco, mujoco_lidar; print(mujoco.__version__)"
```

### 1.1 安装 RealSense D455 环境

如果需要在实机上使用 D455 录制 RGB-D bag 或实时 nvblox 建图，需要安装 Intel RealSense 驱动和 ROS 2 封装：

```bash
# 安装 librealsense2 和 ROS2 realsense2_camera
sudo apt update
sudo apt install \
  librealsense2-dev \
  librealsense2-utils \
  ros-humble-realsense2-camera \
  ros-humble-realsense2-description
```

验证 RealSense 设备：

```bash
rs-enumerate-devices
```

如果 `rs-enumerate-devices` 能看到 D455，说明驱动安装成功。

本地静态地图目录为 `/home/a/WBMM/maps/`，需要包含：
`site_remani.npz`、`site_mesh.ply`、`site_2d.yaml` 和 `site_2d.pgm`。

## 2. 构建

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-up-to \
  tracer_jaka_description \
  tracer_jaka_mujoco \
  tracer_jaka_ocs2 \
  remani_planner \
  wipe_planner \
  grid_map \
  hipnuc_imu \
  lakibeam1 \
  tracer_jaka_bringup \
  tracer_jaka_localization

source install/setup.bash
```

如果只改单个包：

```bash
colcon build --packages-select tracer_jaka_ocs2 --symlink-install
```

> **REMANI 路径搜索修复（重要）**：`path_searching/include/path_searching/kino_astar.h`
> 中的 `#define inf 1 >> 30` 是笔误（应为 `1 << 30`，否则 `inf` 等于 0）。
> 它会导致近距离、大转角的预接触目标（如白板 pre-contact goal：
> 0.53 m 位移 + 90° 航向旋转）在 `GEN_NEW_TRAJ` 阶段提前返回空轨迹并段错误。
> 拉取包含该修复的代码后，**必须单独重新编译 `path_searching`**（它是独立
> ament 包，只编译 `remani_planner` 不会重建它）：

```bash
colcon build --packages-select path_searching
```

## 3. 仿真快速启动

### 3.1 完整仿真：定位 + SLAM + REMANI + OCS2

```bash
ros2 launch tracer_jaka_bringup ocs2_sim.launch.py
```

在 RViz 中用 `2D Goal Pose` 发送目标。

常用裁剪参数：

```bash
# 不启动 SLAM
ros2 launch tracer_jaka_bringup ocs2_sim.launch.py start_slam:=false

# 不启动 REMANI
ros2 launch tracer_jaka_bringup ocs2_sim.launch.py start_remani:=false

# 不启动 RViz
ros2 launch tracer_jaka_bringup ocs2_sim.launch.py rviz:=false
```

### 3.2 只验证定位与二维 SLAM

```bash
ros2 launch tracer_jaka_bringup slam_sim.launch.py
```

### 3.3 打磨/加工任务桌

```bash
ros2 launch tracer_jaka_bringup mujoco_task_table.launch.py
```

查看仿真六维力：

```bash
ros2 launch tracer_jaka_bringup mujoco_task_table.launch.py \
  init_keyframe:=task_contact fts_zero_on_start:=false
ros2 topic echo /fts_broadcaster/wrench
```

### 3.4 任务/擦拭预览

```bash
ros2 launch wipe_planner wipe_plan_preview.launch.py
```

### 3.5 本地静态 ESDF：MuJoCo + OCS2 + REMANI

使用 `maps/` 中已导出的地图验证完整规划控制闭环：

```bash
ros2 launch tracer_jaka_bringup ocs2_esdf_validation.launch.py \
  frame_id:=map \
  use_rviz:=true \
  viewer:=false
```

日志出现 `MPC node is ready`、`MRT node is ready` 与
`Loaded static ESDF` 即表示仿真规划链已就绪。

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

ros2 launch tracer_jaka_bringup mujoco_nvblox_mapping.launch.py \
  viewer:=false ros_domain_id:=20
```

等待 Docker 侧出现：

```text
ESDF mapping coverage scan complete. The map can now be exported.
```

### 4.3 主机侧：使用导出的 ESDF 运行控制闭环

```bash
ros2 launch tracer_jaka_bringup ocs2_mapped_esdf_control.launch.py
```

自定义目标：

```bash
ros2 launch tracer_jaka_bringup ocs2_mapped_esdf_control.launch.py \
  goal_x:=2.6 goal_y:=-1.2 goal_yaw:=0.0
```

### 4.4 离线 ROS bag → nvblox 容器建图（推荐）

适用于实机录制好的 D455 bag，例如：

```text
/home/a/WBMM/bags/d455_esdf_01
```

#### 4.4.1 把 bag 放入容器可见目录

推荐复制到容器挂载工作区：

```bash
mkdir -p /home/a/workspaces/isaac_ros-dev/bags

cp -r /home/a/WBMM/bags/d455_esdf_01 \
  /home/a/workspaces/isaac_ros-dev/bags/
```

或使用 rsync：

```bash
rsync -avP /home/a/WBMM/bags/d455_esdf_01/ \
  /home/a/workspaces/isaac_ros-dev/bags/d455_esdf_01/
```

也可以直接用 docker cp：

```bash
docker cp /home/a/WBMM/bags/d455_esdf_01 \
  isaac_ros_dev-x86_64-container:/workspaces/isaac_ros-dev/bags/
```

#### 4.4.2 启动离线 nvblox 建图

容器外：

```bash
xhost +local:docker
docker exec -it -u admin \
  --workdir /workspaces/isaac_ros-dev \
  isaac_ros_dev-x86_64-container bash
```
容器内：



```bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
export ISAAC_ROS_NVBLOX_PLUGIN_FORCE_FALLBACK_MATERIAL=1
cd /workspaces/isaac_ros-dev

ros2 launch my_nvblox_bringup d455_bag_esdf.launch.py \
  bag:=/workspaces/isaac_ros-dev/bags/d455_esdf_01 \
  map_output:=/workspaces/isaac_ros-dev/bag_export/new_site.nvblx \
  ply_output:=/workspaces/isaac_ros-dev/bag_export/new_site_mesh.ply \
  esdf_output:=/workspaces/isaac_ros-dev/bag_export/new_site_remani.npz \
  map2d_output:=/workspaces/isaac_ros-dev/bag_export/new_site_2d.yaml \
  unknown_is_occupied:=false \
  global_frame:=map \
  rviz:=false
```

等待日志出现：

```text
All synchronized depth inputs were processed; starting complete-map export
Saved nvblox mesh PLY: .../new_site_mesh.ply
Saved native nvblox map: .../new_site.nvblx
Saved REMANI ESDF: .../new_site_remani.npz
```

#### 4.4.3 实时 RViz 观察

如果想在建图过程中实时观察 mesh / 2D 地图，把上一条命令中的：

```bash
rviz:=false
```

改成：

```bash
rviz:=true
```


RViz 中：

- Fixed Frame 设为 `map`；
- 添加/确认话题：
  - `/map`：2D SLAM 地图
  - `/nvblox_node/mesh`：实时 nvblox mesh
  - `/nvblox_node/esdf_3d_pointcloud`：3D ESDF 点云

#### 4.4.4 导出结果

导出完成后，文件位于：

```text
/home/a/workspaces/isaac_ros-dev/bag_export/new_site.nvblx
/home/a/workspaces/isaac_ros-dev/bag_export/new_site_mesh.ply
/home/a/workspaces/isaac_ros-dev/bag_export/new_site_remani.npz
/home/a/workspaces/isaac_ros-dev/bag_export/new_site_2d.yaml
/home/a/workspaces/isaac_ros-dev/bag_export/new_site_2d.pgm
```

主机验证：

```bash
source /home/a/WBMM/install/setup.bash

ros2 launch tracer_jaka_bringup ocs2_esdf_validation.launch.py \
  esdf_file:=/home/a/workspaces/isaac_ros-dev/bag_export/new_site_remani.npz \
  ply_file:=/home/a/workspaces/isaac_ros-dev/bag_export/new_site_mesh.ply \
  map2d_yaml:=/home/a/workspaces/isaac_ros-dev/bag_export/new_site_2d.yaml \
  frame_id:=map \
  use_rviz:=true \
  viewer:=false
```

## 5. 实机启动

> 实机自主擦拭前必须先完成 [实机部署指南](实机部署指南.md) 中的安全与标定检查。

### 5.1 底盘 CAN JAKA

```bash
sudo modprobe gs_usb
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0
ros2 run jaka_driver jaka_login
```

### 5.2 实机底盘 / IMU / 雷达 / EKF / SLAM（建图模式）

启动底盘、IMU、Lakibeam、EKF、slam_toolbox 和 JAKA 只读状态：

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 launch tracer_jaka_bringup real_slam.launch.py \
  start_slam:=true \
  start_arm_pose:=false \
  start_jaka_hardware:=true \
  start_jaka_fts:=true \
  jaka_robot_ip:=10.5.5.100 \
  jaka_local_ip:=10.5.5.127 \
  rviz:=true \
  can_port:=can0 \
  serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_e6872e3dafebed119ff7429aa88ea882-if00-port0 \
  lidar_host_ip:=192.168.8.1 \
  lidar_sensor_ip:=192.168.8.2 \
  scan_topic:=/scan
```

如果驱动已单独启动：

```bash
ros2 launch tracer_jaka_bringup real_slam.launch.py \
  start_imu:=false start_lidar:=false
```

### 5.3 启动 D455

新终端：

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch tracer_jaka_bringup d455_real.launch.py
```

确认话题：

```bash
ros2 topic hz /camera/d455/depth/image_rect_raw
ros2 topic hz /camera/d455/color/image_raw
ros2 topic hz /scan
ros2 topic hz /odometry/filtered
```

### 5.4 录制 D455 bag

新终端：

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash

mkdir -p bags

ros2 launch tracer_jaka_bringup record_d455_esdf_bag.launch.py \
  output:=bags/d455_esdf_01
```

录制建议：

- 先静止 5 秒；
- 再缓慢旋转和移动，覆盖需要建图的区域；
- 录制结束后按 `Ctrl+C`，等待 zstd 压缩完成。

检查 bag：

```bash
ros2 bag info bags/d455_esdf_01
```

### 5.5 保存当前 2D 地图

在 slam_toolbox 建图过程中，随时可以保存当前 2D 地图：

```bash
mkdir -p /home/a/WBMM/maps

ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{
  name: {
    data: '/home/a/WBMM/maps/current_map'
  }
}"
```

保存后生成：

```text
/home/a/WBMM/maps/current_map.pgm
/home/a/WBMM/maps/current_map.yaml
```

### 5.6 实机持久地图定位 + REMANI-MPC

```bash
source /opt/ros/humble/setup.bash
source /home/a/WBMM/install/setup.bash

cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch tracer_jaka_bringup remani_mpc_localized_real.launch.py \
  can_port:=can0 \
  serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_e6872e3dafebed119ff7429aa88ea882-if00-port0 \
  lidar_host_ip:=192.168.8.1 \
  lidar_sensor_ip:=192.168.8.2 \
  map_file:=/home/a/WBMM/maps/map1/site_2d.yaml \
  static_esdf_file:=/home/a/WBMM/maps/map1/site_remani.npz \
  lib_folder:=/tmp/ocs2_tracer_jaka_conservative/auto_generated \
  initial_x:=0.0 \
  initial_y:=0.0 \
  initial_yaw:=0.0 \
  use_rviz:=true \
  use_joy:=false \
  tracking_error_replan_enabled:=false \
  freeze_manipulator:=false \
  manipulator_max_vel:=0.10 \
  manipulator_max_acc:=0.20 \
  mobile_base_max_wheel_omega:=1.5 \
  mobile_base_max_wheel_alpha:=3.0 \
  mobile_base_non_singul_vel:=0.05 \
  jaka_read_only:=false \
  command_output_enabled:=true \
  start_ocs2:=true \
  start_remani:=true \
  start_bridge:=true \
  start_arm_pose:=false \
  arm_max_delta_per_step:=0.05 \
  arm_max_command_velocity:=0.10
```
启动内容：
```bash
remani_mpc_localized_real.launch.py
├── real_slam.launch.py (start_slam:=false)
│   ├── Tracer CAN 驱动                     -> /odom
│   ├── Hipnuc IMU                         -> /IMU_data
│   ├── Lakibeam LiDAR                     -> /scan
│   ├── robot_localization EKF             -> /odometry/filtered
│   └── robot_state_publisher              -> 机器人静态/关节 TF
├── localization_real.launch.py
│   ├── map_server                         -> /map
│   └── AMCL                               -> map -> odom
├── odom_to_map_relay.py                   -> /odometry/filtered_map
├── ocs2_real.launch.py
│   ├── JAKA ros2_control
│   ├── joint_state_broadcaster            -> /joint_states
│   ├── fts_broadcaster                -> /fts_broadcaster/wrench
│   ├── arm_controller
│   └── OCS2 MPC/MRT（默认 dry-run）        -> 仅在双安全开关打开后发布控制命令
└── remani_mpc_tracking.launch.py（延迟 15 s）
    ├── REMANI（在 map 中规划）             -> /planning/trajectory
    └── REMANI -> OCS2 bridge（map -> odom）-> /mobile_manipulator_mpc_target
```

### 5.7 实机前方白板：规划预览

实机任务默认使用：

```text
config/wipe_task_real_front.yaml
```

其几何假设为：

- 机器人初始定位为 `map=(0,0,0)`；
- 竖直白板在机器人正前方 2 m，即平面 `map X=2.0 m`；
- 白板覆盖区域中心高度为 `Z=0.55 m`；
- 覆盖范围为宽 `0.90 m`、高 `0.40 m`；
- 9 条连续蛇形横线，中心线间距 5 cm；
- 默认 `contact.offset=-0.05`，即在白板前方 5 cm 空描；
- 不开启力控，仅跟踪几何参考轨迹。

`Z=0.55 m` 是当前真实 URDF 完整预接触和覆盖检查能够通过的最低中心高度。
直接使用 `Z=0.50 m` 时，法向预接触中段没有无碰撞 IK 解。

先构建目标包：

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash

ROS_LOG_DIR=/tmp/wipe_real_build_logs \
colcon build --packages-select wipe_planner --symlink-install

source install/setup.bash
```

启动纯规划预览：

```bash
ROS_LOG_DIR=/tmp/wipe_real_preview \
ros2 launch wipe_planner wipe_real_plan_preview.launch.py \
  use_rviz:=true
```

该命令只启动规划预览和白板 Marker，不启动 MuJoCo、实机驱动、REMANI、
OCS2 或任何控制输出。因为预览模式没有 AMCL/SLAM，它会单独发布一个零位姿
`map -> odom`，只用于让以 `odom` 为固定坐标系的 RViz 显示 `map` 中的规划
Marker；这个 TF 不代表真实定位结果。当前验证结果应接近：

```text
Plan published: 301 waypoints, about 725 s, 296 EE poses
```

（点数/时长随 Hybrid A* 展开和采样略有波动，属正常现象。）

如果已经同时运行了会发布真实 `map -> odom` 的定位系统，避免重复 TF：

```bash
ros2 launch wipe_planner wipe_real_plan_preview.launch.py \
  use_rviz:=true publish_preview_map_to_odom:=false
```

RViz 中应检查：

- 蓝色半透明白板位于机器人正前方 2 m；
- 黄色机器人为预接触位姿；
- 底盘轨迹满足差速约束，没有横向滑移；
- 绿色末端轨迹连续覆盖整个 `0.90 x 0.40 m` 区域；
- `tool0 +Z` 从房间侧指向白板。

### 5.8 实机白板：导航 → 预接触 → 蛇形全覆盖

完整实机启动文件为：

```text
tracer_jaka_bringup/launch/wipe_real_pipeline.launch.py
```

它组合第 5.6 节已经验证的持久地图实机 bringup，并增加 TF-aware
WipePlanner：

```text
AMCL                                         -> map -> odom
OCS2 observation (odom)                     -> WipePlanner 转换到 map
WipePlanner 白板/覆盖规划 (map)             -> REMANI 9D 预接触目标
REMANI 导航轨迹 (map)                       -> WipePlanner
WipePlanner 将底盘参考 map -> odom           -> OCS2/MRT
机械臂六关节参考保持不变                    -> JAKA 控制器
```

标准 REMANI bridge 在此 launch 中被强制设为 `start_bridge:=false`，因为
WipePlanner 同时负责导航轨迹中继和覆盖轨迹发布。必须保证
`/mobile_manipulator_mpc_target` 始终只有一个发布者。

#### 5.8.1 关闭运动输出的整链路检查

先使用默认三重安全门运行：

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash

ROS_LOG_DIR=/tmp/wipe_real_dry_run \
ros2 launch tracer_jaka_bringup wipe_real_pipeline.launch.py
```

默认安全参数为：

```text
jaka_read_only=true
command_output_enabled=false
force_control_enabled=false
start_bridge=false
safety_release=false
auto_goal=false
```

该 launch 已默认使用第 5.6 节的设备和地图参数：

```text
can0
/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_e6872e3dafebed119ff7429aa88ea882-if00-port0
LiDAR host 192.168.8.1
LiDAR sensor 192.168.8.2
/home/a/WBMM/maps/map1/site_2d.yaml
/home/a/WBMM/maps/map1/site_remani.npz
```

WipePlanner 默认延迟 20 秒启动，以等待定位、OCS2 和延迟启动的 REMANI。
启动后检查 TF：

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map base_footprint
```

检查参考轨迹所有权：

```bash
ros2 topic info /mobile_manipulator_mpc_target -v
```

必须只看到 WipePlanner 一个发布者。若同时看到 REMANI bridge，立即停止，
不要打开运动输出。

检查任务状态：

```bash
ros2 topic echo /wipe_planner/phase
ros2 topic echo /wipe_planner/force_control_state
ros2 topic echo /remani_planner/fsm_state
ros2 topic echo /wipe_planner/base_tracking_error
ros2 topic echo /wipe_planner/joint_tracking_error
ros2 topic echo /wipe_planner/ee_tracking_error
```

无力控时 `/wipe_planner/force_control_state` 必须为：

```text
disabled
```

完整执行阶段顺序为：

```text
waiting_navigation
-> remani_navigation
-> wipe_planning
-> continuous_contact_wiping
```

在关闭命令输出时，机器人不会实际到达导航目标，因此主要用于检查完整规划、
RViz、TF、目标发布者和参考轨迹方向。

#### 5.8.2 打开实机运动

只有在以下条件全部满足后，才允许打开运动：

- 急停可用，并有人员在急停旁监护；
- AMCL 中机器人初始位姿正确；
- RViz 白板位置和真实白板一致；
- 白板前方和底盘导航区域没有人员或障碍物；
- `/mobile_manipulator_mpc_target` 只有一个发布者；
- `map -> odom -> base_footprint` TF 连续稳定；
- 机械臂当前关节状态和 RViz 一致；
- 当前仍使用 `contact.offset=-0.05` 进行非接触空描。

启动实际运动：

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash

ROS_LOG_DIR=/tmp/wipe_real_motion \
ros2 launch tracer_jaka_bringup wipe_real_pipeline.launch.py \
  jaka_read_only:=false \
  command_output_enabled:=true \
  safety_release:=true \
  auto_goal:=true \
  force_control_enabled:=false
```

预期行为：

1. WipePlanner 根据白板几何预先生成完整全身覆盖轨迹；
2. 向 REMANI 发布包含底盘和六个关节的 9D 预接触目标；
3. 底盘从初始位置导航至距墙约 `0.70 m`，目标底盘 `X` 约为 `1.30 m`；
4. REMANI 到达后进入 `TASK_EXEC`，WipePlanner 取得唯一参考轨迹所有权；
5. 底盘保持白板安全距离，机械臂完成无碰撞关节对齐；
6. 末端沿白板法向完成预接触；
7. 底盘沿墙前直线前进/倒车，机械臂调整高度，完成 9 行蛇形覆盖；
8. 自适应虚拟进度会在跟踪误差增大时减速或暂停。

当前保守轨迹名义时间约 725 秒，即约 12 分钟。使用有效宽度不小于 5 cm
的擦拭头时，9 条轨迹能够形成连续材料覆盖。

> 该实机链路（导航 → 预接触 → 9 行蛇形覆盖）已在 MuJoCo 中完整跑通验证，
> 见下文第 5.9 节。

#### 5.8.3 从空描改为白板表面跟踪

默认配置：

```yaml
contact:
  offset: -0.05
```

表示末端在白板前方 5 cm 运行。必须先完成整条空描轨迹，再根据实测白板平面
逐步减小该负偏移。

若要进行纯几何表面跟踪，可以在
`src/applications/wiping/wipe_planner/config/wipe_task_real_front.yaml` 中改为：

```yaml
contact:
  offset: 0.0
```

但关闭力控并不等于不会撞击白板。`offset=0.0` 只能在以下条件下使用：

- 已重新测量白板的 `map` 坐标、法向和边界；
- 工具使用弹簧笔、浮动笔架或柔顺擦拭头；
- 从低速、局部轨迹开始验证；
- 不允许使用正偏移，因为正偏移会向白板内部规划。

实机上不要运行 `wipe_pipeline.launch.py`。它是 MuJoCo/空 ESDF 验证组合，
会启动另一套 OCS2/REMANI，与实机链路产生重复节点和参考轨迹所有权冲突。

### 5.9 MuJoCo 仿真验证（前方白板，推荐先跑）

与实机完全同一套任务配置（`wipe_task_real_front.yaml`、同样的保守限速），
用于在仿真中完整验证导航 → 预接触 → 9 行蛇形覆盖链路，模拟环境包含
白板实体（`scene_wipe_front_board.xml`），REMANI 使用全空 ESDF（仅验证
规划-跟踪链路，不验证碰撞规避）。

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash

# 无界面快速验证（推荐）
ros2 launch tracer_jaka_bringup wipe_front_board_sim.launch.py \
  viewer:=false use_rviz:=false

# 带 MuJoCo 窗口和 RViz 观察
ros2 launch tracer_jaka_bringup wipe_front_board_sim.launch.py
```

若 sandbox/无网卡环境下 DDS 枚举失败，加上：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1
```

预期日志里程碑（按时间顺序）：

```text
Coverage planned before navigation: 301 points, 725.3 s; ...      # ~12 s
Received whole-body goal in odom: base=(1.280, 0.450, 1.571), ... # ~35 s
[FSM]: from GEN_NEW_TRAJ to EXEC_TRAJ                             # 规划成功
[FSM]: from EXEC_TRAJ to TASK_EXEC                                # 开始执行
Executing base-hold arm alignment, surface-normal approach, ...   # 预接触
                                                                  #   └─ 此刻 RViz 才开始显示覆盖轨迹（蓝色全身轨迹）
Path progress: tau=..., contour=0.002 m, force=0.00 N, force_state=disabled
tool0 position ref=[0.950 ...], surface-normal gap≈46 mm          # 5 cm 空描
MPC tracking: base≈0.004 m, joint≈0.005 rad, EE≈0.006 m           # 稳态跟踪
```

该 MuJoCo 启动默认关闭 base/yaw-only 提前交接。只有完整 9D 预接触状态的
平方误差不大于 `0.02` 并连续稳定 `0.5 s` 后，WipePlanner 才能请求
`TASK_EXEC`。如果看到 `Trajectory finished but goal not reached` 或
`[TRIG]: ... GEN_NEW_TRAJ`，表示 REMANI 仍在导航模式中从实测状态继续规划，
不是已经开始接触任务。

已实测的跟踪质量（供对照）：

- 稳态跟踪误差：底盘 ~4 mm、关节 ~5 mrad、末端 ~6 mm；
- REMANI → WipePlanner 交接瞬间有一次瞬时误差（EE ~0.16 m、底盘 ~0.06 m），
  随后 2 s 内回到稳态；
- 每行末端 180° 翻转时臂重定向产生瞬时误差（EE 最大 ~0.19 m），2–4 s 恢复；
  末端会短暂离开板面并下探（最低约 z=0.16 m，不触地），属预期行为；
- 整条 725.3 s 覆盖轨迹可完整跑完，结束后保持最终位姿（零输入钳制）；
- 工具全程保持 45–47 mm 干跑间距，`force_state=disabled`、force=0.00 N。

## 6. 常用检查命令

```bash
# 话题频率
ros2 topic hz /odometry/filtered
ros2 topic hz /joint_states
ros2 topic hz /fts_broadcaster/wrench
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
