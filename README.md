# WBMM：Tracer-JAKA 全身移动操作（Whole-Body Mobile Manipulation）工作空间

本仓库是 Tracer 移动底盘与 JAKA Zu5 机械臂的 ROS 2 Humble 工作空间。它将
定位建图、MuJoCo 仿真、nvblox 三维 ESDF 建图、REMANI 全身规划与 OCS2 MPC
控制整合在同一套接口下，支持先在仿真验证，再迁移至真实机器人。

> 工作空间已更名为 **WBMM（Whole-Body Mobile Manipulation）**，源码目录也已按“职责 + 部署场景”重新组织。
>
> 📖 快速命令速查见 [docs/QUICKSTART.md](docs/QUICKSTART.md)。

## 系统能力

- **定位与二维建图**：轮式里程计 + Hipnuc IMU + Lakibeam 2D 雷达，经
  `robot_localization` 与 `slam_toolbox` 输出稳定的 `/odometry/filtered`、`/map`
  和标准 TF 树。
- **MuJoCo 仿真**：Tracer + JAKA Zu5、2D LiDAR、IMU、D435 末端相机和底盘固定
  D455 RGB-D 相机；同一 ROS 2 话题接口可复用到实机。
- **三维环境建图**：D455 RGB-D 数据在 Isaac ROS Docker 中由 nvblox 融合为
  TSDF、mesh 和 ESDF，并导出 REMANI 可读取的 `.npz` 地图。
- **移动操作规划与控制**：REMANI 负责 ESDF 约束下的全身轨迹，桥接节点转换为
  OCS2 `TargetTrajectories`，OCS2 MPC/MRT 输出底盘和机械臂控制命令。
- **离线复现**：可录制 D455 RGB-D、TF、定位与二维地图为 rosbag，再离线建立
  ESDF 并用于仿真规划验证。

## 总体数据流

```mermaid
flowchart LR
  subgraph Sensors[仿真或真实机器人]
    Odom[轮式里程计]
    Imu[Hipnuc IMU]
    Laser[Lakibeam 2D LiDAR]
    RGBD[D455 RGB-D]
    Joint[关节状态]
  end

  Odom --> EKF[robot_localization EKF]
  Imu --> EKF
  EKF --> Filtered[/odometry/filtered]
  Laser --> SLAM[slam_toolbox]
  Filtered --> SLAM
  SLAM --> Map[/map and map to odom]

  RGBD --> NVBlox[nvblox in Isaac ROS Docker]
  Filtered --> NVBlox
  Joint --> REMANI
  NVBlox --> ESDF[ESDF NPZ]
  ESDF --> REMANI[REMANI planner]
  Filtered --> REMANI
  REMANI --> Traj[/planning/trajectory]
  Traj --> Bridge[REMANI to OCS2 bridge]
  Bridge --> MPC[OCS2 MPC and MRT]
  Filtered --> MPC
  Joint --> MPC
  MPC --> Base[/base_controller/cmd_vel]
  MPC --> Arm[/arm_controller/commands]
```

TF 的发布权保持唯一，避免定位与 SLAM 阶段发生冲突：

```text
slam_toolbox:       map -> odom
robot_localization: odom -> base_footprint
robot_state_publisher:
                    base_footprint -> base_link -> sensor and arm links
```

## 仓库结构

| 路径 | 作用 |
|---|---|
| `src/vendor/` | 上游/第三方源码：`ocs2_ros2`、`remani_planner` |
| `src/interfaces/` | 跨层稳定消息/服务/动作契约：`tracer_jaka_interfaces` |
| `src/robot/` | 机器人唯一描述：`tracer_jaka_description`、`tracer_jaka_moveit_config` |
| `src/drivers/` | 真实硬件 I/O：底盘、JAKA 机械臂、夹爪、IMU、LiDAR |
| `src/algorithms/` | 公共算法：OCS2 控制、REMANI 集成、力控预留 |
| `src/perception/` | 定位/建图/ESDF：nvblox、grid_map、esdf_simple_nav、localization、mapping |
| `src/applications/` | 具体任务：`wiping/wipe_planner` |
| `src/simulation/` | 仿真后端：`tracer_jaka_mujoco` |
| `src/bringup/` | 顶层系统组合：`tracer_jaka_bringup` |
| `tools/` | 仓库级脚本和示例 |
| `data/` | 运行数据：bags、maps_generated、outputs、debug |
| `docs/` | 架构、ESDF、rosbag、REMANI 与实验记录文档 |

## 环境要求

主机建议使用 Ubuntu 22.04 + ROS 2 Humble。以下依赖已在本工作区的
MuJoCo、静态 ESDF 验证和实机持久地图入口中验证：

- MuJoCo Python 运行环境；关闭 MuJoCo viewer 的无头渲染使用 `MUJOCO_GL=egl`。
- `robot_localization`、`slam_toolbox`、Nav2 map server / AMCL。
- ROS 2 Control 的 `controller_manager` 与标准控制器（实机机械臂控制）。
- Tracer CAN、Hipnuc IMU 与 Lakibeam 驱动（仅实机）。
- Isaac ROS Docker + CUDA + nvblox（仅三维 ESDF 建图）。

安装系统依赖：

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

MuJoCo 安装验证：

```bash
python3 -c "import mujoco, mujoco_lidar; print(mujoco.__version__)"
```

静态地图文件放在 `/home/a/WBMM/maps/`，本工作区的默认文件为
`site_remani.npz`、`site_mesh.ply`、`site_2d.yaml` 与 `site_2d.pgm`。

## 构建主工作空间

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
  tracer_jaka_bringup \
  tracer_jaka_localization

source install/setup.bash
```

初次启动 OCS2 时可能触发模型自动微分代码生成，时间会明显长于后续启动。若修改
`task.info` 中影响动力学或运动学的配置，请按 OCS2 文档清理对应的自动生成目录后
重新生成。

## 常用入口

> 更简洁的启动命令速查见 [docs/QUICKSTART.md](docs/QUICKSTART.md)。
> 当前实际启动入口仍以各功能包为主；`tracer_jaka_bringup` 是规划中的统一顶层入口，待后续收敛。

### 1. MuJoCo：定位、SLAM、REMANI 与 OCS2

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch tracer_jaka_ocs2 ocs2_sim.launch.py
```

该入口启动 MuJoCo、EKF、slam_toolbox、REMANI、REMANI→OCS2 bridge、MPC/MRT
与 RViz。用 RViz 的 `2D Goal Pose` 向 `/goal_pose` 发送目标。

只验证定位与二维建图：

```bash
ros2 launch tracer_jaka_mujoco slam_sim.launch.py
```

### 2. MuJoCo：RGB-D 建图、ESDF 导出、规划控制

完整流程分为“Docker 建图”和“主机控制”两个阶段。详细步骤见
[MuJoCo → nvblox → REMANI → OCS2 教程](docs/MUJOCO_NVBLOX_REMANI_PIPELINE.md)。

在 Isaac ROS Docker 中先等待 nvblox：

```bash
ros2 launch my_nvblox_bringup mujoco_mapping_export.launch.py \
  ros_domain_id:=20 rviz:=true
```

在主机启动自动覆盖扫描：

```bash
ros2 launch tracer_jaka_mujoco mujoco_nvblox_mapping.launch.py \
  viewer:=false ros_domain_id:=20
```

导出后，使用保存的 ESDF 运行控制闭环：

```bash
ros2 launch tracer_jaka_ocs2 mujoco_mapped_esdf_control.launch.py
```

使用本地静态 ESDF 完整验证（MuJoCo + OCS2 + REMANI）：

```bash
export MUJOCO_GL=egl
ros2 launch tracer_jaka_ocs2 ocs2_esdf_validation.launch.py \
  frame_id:=map \
  use_rviz:=true \
  viewer:=false
```

### 3. MuJoCo：打磨/加工任务桌

加工桌场景包含一张 `2.40 × 1.20 × 0.75 m` 的桌子和两张
`0.34 × 0.24 × 0.44 m` 的板凳；桌面任务面带有 3×3 命名采样点，可作为
三维重建、覆盖轨迹生成和末端接触约束的公共基准。

```bash
ros2 launch tracer_jaka_mujoco task_table_sim.launch.py
```

场景文件是
[`scene_task_table.xml`](src/simulation/tracer_jaka_mujoco/models/scene_task_table.xml)。
后续若接入 REMANI/OCS2，应先用 D455/nvblox 将该场景建立为 ESDF，再以
`task_surface_*` sites 生成末端任务轨迹。

场景还提供与实机 `jaka_fts_broadcaster` 兼容的六维力接口。快速查看工具与桌面
接触产生的力和力矩：

```bash
ros2 launch tracer_jaka_mujoco task_table_sim.launch.py \
  init_keyframe:=task_contact fts_zero_on_start:=false
ros2 topic echo /jaka_fts_broadcaster/wrench
```

### 4. 真实机器人：定位与二维 SLAM

将 CAN、IMU 和 Lakibeam 接好后，集成启动：

```bash
sudo ip link set can0 up type can bitrate 500000
sudo chmod 777 /dev/ttyUSB0

source /home/a/WBMM/install/setup.bash
ros2 launch tracer_jaka_mujoco real_slam.launch.py
```

硬件接口约定如下：

| 数据 | 话题 | 坐标系 |
|---|---|---|
| 底盘里程计 | `/odom` | `odom -> base_footprint` |
| IMU | `/IMU_data` | `imu_link` |
| 2D 雷达 | `/scan` | `laser_link` |
| 融合定位 | `/odometry/filtered` | `odom -> base_footprint` |
| SLAM 地图 | `/map` | `map` |

如果驱动已在其他终端启动，请让 `real_slam.launch.py` 关闭对应重复驱动，避免重复
发布 TF。例如：

```bash
ros2 launch tracer_jaka_mujoco real_slam.launch.py \
  start_imu:=false start_lidar:=false
```

### 5. 实机或 rosbag：D455 nvblox ESDF

`my_nvblox_bringup` 在本仓库维护源码，但因 nvblox/CUDA 依赖 Isaac ROS Docker，
需要先同步到 Docker 共享工作空间：

```bash
cd /home/a/WBMM
src/perception/my_nvblox_bringup/scripts/sync_to_isaac_ros_ws.sh
```

进入 Docker 后构建：

```bash
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select my_nvblox_bringup
source install/setup.bash
```

真实机器人在线建图：

```bash
ros2 launch my_nvblox_bringup d455_esdf.launch.py
```

离线 rosbag 建图与导出：

```bash
ros2 launch my_nvblox_bringup d455_bag_esdf.launch.py \
  bag:=/workspaces/isaac_ros-dev/bags/d455_esdf_01
```

## 关键接口

| 目的 | 接口 |
|---|---|
| REMANI 目标输入 | `/goal_pose` |
| REMANI 轨迹输出 | `/planning/trajectory` |
| OCS2 参考输入 | `/mobile_manipulator_mpc_target` |
| 底盘控制输出 | `/base_controller/cmd_vel` |
| 机械臂控制输出 | `/arm_controller/commands` |
| nvblox 原生地图 | `.nvblx` |
| REMANI 静态距离场 | `.npz`（ESDF、occupancy、observed、origin、voxel_size） |

## 文档导航

- [快速开始 / 启动命令速查](docs/QUICKSTART.md)
- [MuJoCo → nvblox → REMANI → OCS2 完整通道](docs/MUJOCO_NVBLOX_REMANI_PIPELINE.md)
- [D455 ESDF 仿真与实机运行指南](docs/D455_ESDF_仿真与实机运行指南.md)
- [D455 RGB-D ESDF rosbag 录制教程](docs/D455_RGBD_ESDF_ROSBAG_录制教程.md)
- [REMANI 与 OCS2 集成说明](docs/REMANI_OCS2_INTEGRATION.md)
- [仓库总体 Pipeline](docs/总体%20Pipeline.md)
- [工程目录重构设计](docs/工程目录重构设计.md)
- [周报与实验整理](docs/WEEKLY_REPORT_2026-07-25_to_2026-07-31.md)

各功能包还提供更具体的说明：

- [MuJoCo 包 README](src/simulation/tracer_jaka_mujoco/README.md)
- [OCS2 包 README](src/algorithms/control/tracer_jaka_ocs2/README.md)
- [nvblox bringup README](src/perception/my_nvblox_bringup/README.md)

## Git 约定

仓库只保存源码、配置、场景、URDF 和可复现实验文档。以下生成数据默认不提交：

- `build/`、`install/`、`log/`；
- rosbag、SQLite 数据库、压缩文件；
- nvblox `.nvblx`、REMANI `.npz`、导出地图和临时快照；
- Python 缓存与 IDE 配置。

这样克隆仓库后只需准备依赖与传感器数据即可重建地图，而不会把大体积实验数据混入
Git 历史。
