# tracer_jaka_bringup

WBMM 的顶层系统组合包。本文以
`remani_mpc_localized_real.launch.py` 为主，说明如何在真实 Tracer + JAKA
平台上使用保存的 2D 地图进行 AMCL 定位、使用同一现场的 3D ESDF 进行 REMANI
规划，并由 OCS2 MPC/MRT 执行全身轨迹。

> **实机首选入口**：`remani_mpc_localized_real.launch.py`。
> `remani_mpc_real.launch.py` 是边运行边用 `slam_toolbox` 建图的旧流程；已有固定场地
> 地图时不要混用这两个入口，也不要同时启动 AMCL 和 `slam_toolbox` 来发布
> `map -> odom`。

## 1. 启动内容与坐标系

```text
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
│   ├── jaka_forward_controller
│   └── OCS2 MPC/MRT                       -> /cmd_vel 和机械臂位置命令
└── remani_mpc_tracking.launch.py（延迟 15 s）
    ├── REMANI（在 map 中规划）             -> /planning/trajectory
    └── REMANI -> OCS2 bridge（map -> odom）-> /mobile_manipulator_mpc_target
```

坐标系的发布权必须唯一：

```text
AMCL:                  map -> odom
robot_localization:    odom -> base_footprint
robot_state_publisher: base_footprint -> base_link -> LiDAR/IMU/JAKA links
```

REMANI 和静态 ESDF 使用持久化的 `map` 坐标系；OCS2/MRT 仍使用连续、不跳变的
`odom` 坐标系。bridge 每次通过 TF 动态执行 `map -> odom`，所以 AMCL 修正不会直接
造成控制坐标系跳变。`odom_to_map_relay.py` 只把 EKF 位姿转换到 `map`，twist 仍按
`base_footprint` 表达。

## 2. 部署前必须准备

### 2.1 软件与构建

推荐 Ubuntu 22.04 + ROS 2 Humble。先安装系统依赖：

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install \
  can-utils libzip-dev libompl-dev python3-yaml \
  ros-humble-xacro ros-humble-pinocchio \
  ros-humble-robot-localization ros-humble-slam-toolbox \
  ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager \
  ros-humble-nav2-amcl ros-humble-controller-manager \
  ros-humble-ros2-controllers ros-humble-joy
```

构建实机链路。`jaka_hardware_interface` 必须显式构建，因为当前它没有被
`tracer_jaka_ocs2/package.xml` 声明为运行依赖：

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-up-to \
  tracer_jaka_bringup \
  tracer_jaka_localization \
  tracer_jaka_mujoco \
  tracer_jaka_ocs2 \
  remani_planner \
  grid_map \
  tracer_base \
  hipnuc_imu \
  lakibeam1 \
  jaka_hardware_interface

source /home/a/WBMM/install/setup.bash
```

每个新终端都要 source ROS 和工作空间。可先确认入口已安装：

```bash
ros2 pkg prefix tracer_jaka_bringup
ros2 launch tracer_jaka_bringup remani_mpc_localized_real.launch.py --show-args
```

### 2.2 2D 地图与 3D ESDF

一次运行需要两个互相对齐的地图：

- `site_2d.yaml` + `site_2d.pgm`：供 `map_server` 和 AMCL 使用；
- `site_remani.npz`：供 REMANI 做三维碰撞检查。

仓库当前默认路径是：

```text
/home/a/WBMM/maps/site_2d.yaml
/home/a/WBMM/maps/site_remani.npz
```

`site_2d.yaml` 中的 `image` 可以是相对路径，但 PGM 必须位于相对路径能找到的位置。
部署到其他目录或其他用户名后，**不要依赖 launch 中写死的 `/home/a/WBMM` 默认值**，
启动时显式传入 `map_file` 和 `static_esdf_file`。

两个地图必须来自同一现场并使用同一个 `map` 原点、朝向和米制尺度。若 NPZ 只存在
固定平移误差，可用 `static_esdf_offset_x/y/z` 校正；这三个量是 **ESDF 相对 map 的
平移**，不是“本次开机 odom 原点”的补偿。当前入口没有 ESDF yaw 旋转参数，若有
旋转误差，应重新按正确坐标系导出地图，不能只靠 offset 修好。

部署前检查文件：

```bash
test -r /home/a/WBMM/maps/site_2d.yaml
test -r /home/a/WBMM/maps/site_2d.pgm
test -r /home/a/WBMM/maps/site_remani.npz
```

### 2.3 JAKA 网络

先确认工控机与机械臂控制柜在同一网段：

```bash
ip -br address
ping -c 3 10.5.5.100
```

默认配置为机器人 `10.5.5.100`、工控机 `10.5.5.127`。工控机网卡应设置静态地址，
并确保 JAKA EDG 使用的 UDP 端口没有被防火墙拦截。

> **当前实现限制**：顶层 launch 的 `robot_ip:=...`、`local_ip:=...` 目前只是透传到
> `ocs2_real.launch.py`，但该文件没有把它们注入 URDF。JAKA 硬件插件真正读取的是
> `src/simulation/tracer_jaka_mujoco/urdf/tracer_jaka_zu5_real.urdf` 顶部
> `<ros2_control><hardware>` 中的 `robot_ip` 和 `local_ip`。现场 IP 不同必须修改该
> URDF 后重新构建/重新 source；仅在启动命令里覆盖这两个参数不会生效。

### 2.4 CAN、IMU 与 LiDAR

```bash
sudo modprobe gs_usb
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0
candump can0
```

`candump` 看到报文后按 `Ctrl-C` 退出。IMU 建议把用户加入 `dialout` 组并重新登录；
临时调试也可授权当前设备：

```bash
ls -l /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB0
```

当前顶层可覆盖 `can_port`、`serial_port` 和 `scan_topic`。Lakibeam 的主机 IP、传感器
IP、UDP 端口、倒装和角度偏移没有从本入口透出，实际默认值位于
`src/simulation/tracer_jaka_mujoco/launch/real_slam.launch.py`：

```text
lidar_host_ip=0.0.0.0
lidar_sensor_ip=192.168.198.2
lidar_port=2368
lidar_inverted=false
lidar_angle_offset=0
```

现场不同需修改该文件（或以后给顶层增加参数透传）并重启。

## 3. 推荐的分阶段实机部署

### 阶段 A：断开执行能力，核对安全条件

- 清空机械臂和底盘工作区，首次测试将底盘架空或机械固定；
- 示教器、底盘遥控器和物理急停由专人握持；
- JAKA 切到允许外部控制的正确模式，但先不要发送目标；
- 确认按下物理急停能同时阻止底盘和机械臂运动；`Ctrl-C` 不是急停；
- 首次运行保持 `use_joy:=false`、`tracking_error_replan_enabled:=false`。

### 阶段 B：只验证底盘传感器、EKF 与 AMCL

终端 1 启动硬件与 EKF，但不启 SLAM，也不接入机械臂控制：

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch tracer_jaka_mujoco real_slam.launch.py \
  start_slam:=false \
  start_arm_pose:=false \
  rviz:=false \
  can_port:=can0 \
  serial_port:=/dev/ttyUSB0 \
  scan_topic:=/scan
```

终端 2 启动保存地图定位：

```bash
source /opt/ros/humble/setup.bash
source /home/a/WBMM/install/setup.bash

ros2 launch tracer_jaka_localization localization_real.launch.py \
  map_file:=/home/a/WBMM/maps/site_2d.yaml \
  scan_topic:=/scan \
  initial_x:=0.0 \
  initial_y:=0.0 \
  initial_yaw:=0.0
```

> **定位 launch 的当前注意项**：
> `src/perception/tracer_jaka_localization/launch/localization_real.launch.py` 中
> `lifecycle_manager_localization` 的 `node_names` 当前只有 `map_server`。标准 Nav2
> Humble 中 AMCL 也是生命周期节点；如果 `ros2 lifecycle get /amcl` 不是 `active`，
> 应把该列表改为 `['map_server', 'amcl']` 后重新构建/重启。临时验证也可以执行：
>
> ```bash
> ros2 lifecycle set /amcl configure
> ros2 lifecycle set /amcl activate
> ```
>
> 一键入口同样依赖 AMCL 激活；没有 `map -> odom` 时不要继续发送目标。

`initial_x/y/yaw` 是机器人 `base_footprint` 在保存地图中的初始位姿，单位分别为米、
米、弧度；它不是地图 origin。尽量测量后填写。若不确定，在 RViz 用
`2D Pose Estimate` 重新给 AMCL 初值，缓慢原地转动/短距离移动，确认激光与地图墙面
重合，再进入下一阶段。

检查定位：

```bash
ros2 topic hz /odom
ros2 topic hz /IMU_data
ros2 topic hz /scan
ros2 topic hz /odometry/filtered
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 node list | grep -E 'amcl|map_server|ekf|slam_toolbox'
```

应看到 AMCL、map_server 和 EKF，且不应看到 `slam_toolbox`。TF 应连续、无大幅跳动。
验证完先退出这两个 launch，避免下一步重复启动驱动和 TF。

### 阶段 C：准备一份保守的 OCS2 task 文件

顶层参数 `manipulator_max_vel/acc` 只限制 REMANI 生成的机械臂参考轨迹；最终控制输入
还受 OCS2 task 文件约束。建议复制一份实机保守配置，不直接破坏原标定：

```bash
cp /home/a/WBMM/src/algorithms/control/tracer_jaka_ocs2/config/task_real.info \
   /home/a/WBMM/src/bringup/tracer_jaka_bringup/config/task_real_conservative.info
```

在 `task_real_conservative.info` 的 `jointVelocityLimits` 中先使用：

```text
wheelBasedMobileManipulator lowerBound: -0.05, -0.20
wheelBasedMobileManipulator upperBound:  0.05,  0.20
arm lowerBound:  6 个 -0.15
arm upperBound:  6 个  0.15
```

单位依次为底盘线速度 m/s、角速度 rad/s、机械臂关节速度 rad/s。必要时还可提高
`inputCost.R`，让控制更平缓。修改 task 后要重启 MPC；如果遇到自动微分库仍复用旧
模型，可换一个新的 `lib_folder` 或清理该任务专用的 `/tmp` 生成目录后重启。

原 `task_real.info` 的 `environmentCollision.obstacles` 还包含一个测试用 `box_1`
（位置约为 `odom` 中 `[0.8, -0.6, 0.30]`）。它不是 REMANI 的 ESDF 障碍物，也不会
随 AMCL 的 `map -> odom` 自动变换。制作保守副本时必须结合现场决定是删除、关闭
`environmentCollision`，还是改成真实且与本次 `odom` 对齐的障碍；不要把测试方盒
原样带到实机并误以为它来自 NPZ 地图。关闭该项也意味着 OCS2 不再提供这层环境
碰撞约束，仍需依靠 REMANI ESDF、低速和实体安全措施。

### 阶段 D：一键启动保守配置

确认阶段 B 的节点全部退出后执行：

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch tracer_jaka_bringup remani_mpc_localized_real.launch.py \
  map_file:=/home/a/WBMM/maps/site_2d.yaml \
  static_esdf_file:=/home/a/WBMM/maps/site_remani.npz \
  task_file:=/home/a/WBMM/src/bringup/tracer_jaka_bringup/config/task_real_conservative.info \
  lib_folder:=/tmp/ocs2_tracer_jaka_conservative/auto_generated \
  initial_x:=0.0 \
  initial_y:=0.0 \
  initial_yaw:=0.0 \
  use_rviz:=true \
  use_joy:=false \
  tracking_error_replan_enabled:=false \
  freeze_manipulator:=true \
  manipulator_max_vel:=0.10 \
  manipulator_max_acc:=0.20 \
  start_arm_pose:=false
```

`freeze_manipulator:=true` 表示 REMANI 规划时保持当前实测臂型，适合先只验证底盘；
它不是切断机械臂电机，OCS2/JAKA 控制器仍会发布保持位置命令。启动后 OCS2 约在
10 s 接入，REMANI 约在 15 s 接入，整个过程中都要准备急停。

### 阶段 E：检查完整链路后再发小目标

```bash
ros2 control list_controllers
ros2 topic hz /joint_states
ros2 topic hz /odometry/filtered_map
ros2 topic echo /mobile_manipulator_mpc_observation --once

ros2 topic info /cmd_vel -v
ros2 topic info /joint_states -v
ros2 topic info /mobile_manipulator_mpc_target -v
```

期望结果：

- `joint_state_broadcaster` 和 `jaka_forward_controller` 为 `active`；
- `/odometry/filtered_map.header.frame_id` 为 `map`；
- `/cmd_vel` 只有 OCS2 MRT 一个控制发布者；
- `/mobile_manipulator_mpc_target` 只有 REMANI bridge 一个发布者；
- `/joint_states` 只来自真实 JAKA 的 `joint_state_broadcaster`；
- RViz 中 2D 地图、激光、机器人模型和 ESDF/轨迹显示位置一致。

第一次只在机器人前方空旷区发送约 `0.20~0.30 m` 的直线目标，目标必须位于 `map`
坐标系。观察规划轨迹无碰撞、指令方向正确后再逐步增大距离。确认底盘流程稳定后，
才将 `freeze_manipulator` 改为 `false`，仍保持低速，对机械臂发送很小的构型变化。

完整数据流为：

```text
RViz 2D Goal Pose (/goal_pose, frame=map)
  -> REMANI
  -> /planning/trajectory (map)
  -> remani_to_ocs2_reference_bridge + TF(map -> odom)
  -> /mobile_manipulator_mpc_target (odom)
  -> OCS2 MPC/MRT
  -> /cmd_vel + /jaka_forward_controller/commands
```

## 4. 顶层 launch 参数

下列参数可以直接在 `ros2 launch ... 参数:=值` 中覆盖，不需要改源码：

| 参数 | 默认值 | 作用与实机建议 |
| --- | --- | --- |
| `map_file` | `/home/a/WBMM/maps/site_2d.yaml` | 保存的 Nav2 2D 地图 YAML；换机器时显式传绝对路径 |
| `static_esdf_file` | `/home/a/WBMM/maps/site_remani.npz` | 与 2D 地图对齐的 REMANI ESDF NPZ |
| `static_esdf_offset_x/y/z` | `0.0/0.0/0.0` | ESDF 相对 `map` 的固定平移；先保持 0，只根据实测校准 |
| `initial_x/y/yaw` | `0.0/0.0/0.0` | 机器人在保存地图中的 AMCL 初值，yaw 单位 rad |
| `scan_topic` | `/scan` | AMCL 与 LiDAR 共用的 LaserScan 话题 |
| `odom_topic` | `/odometry/filtered` | OCS2/MRT 使用的连续 odom；通常不要改 |
| `map_odom_topic` | `/odometry/filtered_map` | REMANI 使用的 map-frame odometry；通常不要改 |
| `joint_state_topic` | `/joint_states` | 真实机械臂关节状态 |
| `task_file` | `tracer_jaka_ocs2/config/task_real.info` | OCS2 代价、约束和速度上限；首机建议传保守副本 |
| `urdf_file` | `tracer_jaka_zu5_real.urdf` | JAKA 硬件参数、运动学、碰撞体及传感器外参 |
| `lib_folder` | `/tmp/ocs2_tracer_jaka_real/auto_generated` | OCS2 自动生成库目录；不同 task 建议使用不同目录 |
| `manipulator_max_vel` | `0.3` | REMANI 机械臂参考最大速度 rad/s；首机建议 `0.10` |
| `manipulator_max_acc` | `0.5` | REMANI 机械臂参考最大加速度 rad/s²；首机建议 `0.20` |
| `freeze_manipulator` | `false` | `true` 时 REMANI 保持当前臂型；先验证底盘时设 `true` |
| `tracking_error_replan_enabled` | `false` | 跟踪误差自动重规划；调通前保持 `false` |
| `use_joy` | `false` | 必须保持 `false`，避免与 REMANI bridge 争抢 MPC target |
| `use_rviz` | `true` | 是否启动 OCS2 RViz |
| `can_port` | `can0` | Tracer CAN 接口 |
| `serial_port` | `/dev/ttyUSB0` | Hipnuc IMU 串口 |
| `start_imu` / `start_lidar` | `true/true` | 驱动已由外部启动时设 `false`，避免重复发布 |
| `start_arm_pose` | `false` | 实机 JAKA 在线时必须为 `false`，禁止假关节状态 |
| `robot_ip/local_ip` | `10.5.5.100/10.5.5.127` | **当前命令行覆盖不实际注入硬件**，见 2.3 节 |

## 5. 参数到底去哪里修改

| 想调整的内容 | 真正生效的位置 | 是否可由本入口覆盖 |
| --- | --- | --- |
| 地图文件、AMCL 初值、ESDF 平移、REMANI 臂速度/加速度 | `src/bringup/tracer_jaka_bringup/launch/remani_mpc_localized_real.launch.py` | 是，优先用 launch 参数 |
| AMCL 粒子数、激光模型、更新阈值、初始协方差 | `src/perception/tracer_jaka_localization/config/amcl_real.yaml` | 否，改 YAML 后重启 |
| EKF 融合项、频率、超时、IMU/轮速配置 | `src/simulation/tracer_jaka_mujoco/config/ekf_real.yaml` | 否，改 YAML 后重启 |
| LiDAR IP/端口/倒装/角度，驱动默认话题 | `src/simulation/tracer_jaka_mujoco/launch/real_slam.launch.py` | 顶层目前只透传 `scan_topic` |
| IMU 驱动原始配置 | `src/drivers/sensors/hipnuc_imu/config/hipnuc_config.yaml` | 顶层只透传串口和话题 |
| JAKA IP、本机 EDG IP、力传感器偏置 | `src/simulation/tracer_jaka_mujoco/urdf/tracer_jaka_zu5_real.urdf` 顶部 `<ros2_control>` | IP 参数当前不能靠顶层覆盖 |
| LiDAR/IMU/JAKA 安装外参、机器人碰撞体 | 同一个 `tracer_jaka_zu5_real.urdf` | 否，修改后重建/重启 |
| OCS2 底盘/机械臂最终速度上限 | `src/algorithms/control/tracer_jaka_ocs2/config/task_real.info` 的 `jointVelocityLimits` | 用 `task_file` 选择副本 |
| OCS2 输入平滑程度 | 同一 task 的 `inputCost.R` | 用 `task_file` 选择副本 |
| OCS2 跟踪权重 | 同一 task 的 `wholeBodyTracking.Q` | 用 `task_file` 选择副本 |
| OCS2 自碰撞/静态障碍物安全距离 | 同一 task 的 `selfCollision`、`environmentCollision` | 用 `task_file` 选择副本 |
| REMANI 车体尺寸、轮径、机械臂关节限位 | `src/vendor/remani_planner/plan_manage/config/mm_param.yaml` | 本入口只覆盖臂速度/加速度 |
| REMANI 搜索、优化、安全距离 | `src/vendor/remani_planner/plan_manage/config/remani_planner_param.yaml` | 否，改 YAML 后重启 |
| REMANI 跟踪误差重规划阈值 | 同一 `remani_planner_param.yaml`，并在 `remani_mpc_tracking.launch.py` 声明 | 顶层目前只透传启用开关 |
| AMCL 生命周期管理 | `src/perception/tracer_jaka_localization/launch/localization_real.launch.py` 的 `node_names` | 当前应确认包含 `amcl` |

REMANI 参数实际按以下顺序合并，后面的值覆盖前面的值：

```text
mm_param.yaml -> remani_planner_param.yaml -> exp0_param.yaml
              -> remani_mpc_tracking.launch.py 中的显式覆盖
              -> 本顶层 launch 透传值
```

修改源码目录中的 launch/YAML/URDF 后，`--symlink-install` 通常只需重启节点；若安装
空间不是符号链接、修改了 C++，或发现安装空间仍是旧文件，就重新执行对应的
`colcon build` 并重新 source。

## 6. “先保守”应同时限制哪些层

建议首轮采用下面的组合，而不是只改一个速度值：

| 层 | 首轮建议 | 原因 |
| --- | --- | --- |
| 目标源 | `use_joy=false` | 保证只有 REMANI bridge 发布 MPC target |
| 自动行为 | `tracking_error_replan_enabled=false` | 避免误差或定位抖动触发意外新轨迹 |
| REMANI 机械臂 | `freeze_manipulator=true`、`vel=0.10`、`acc=0.20` | 先验证底盘和坐标系 |
| OCS2 底盘 | task 中线速度 `±0.05 m/s`、角速度 `±0.20 rad/s` | 限制最终实际控制输入 |
| OCS2 机械臂 | task 中每关节 `±0.15 rad/s` | REMANI 限速之外再加执行层上限 |
| 目标距离 | 首次 `0.20~0.30 m`、正前方、无障碍 | 便于快速判断方向和坐标是否正确 |
| ESDF offset | 先全为 `0.0` | 未经测量不要用 offset “目测调图” |

调快时一次只改一组参数，每次保留日志和安全员。推荐顺序是：定位稳定性 → 底盘速度
→ 允许机械臂规划 → 机械臂速度/加速度 → 最后才启用误差自动重规划。

## 7. 常见故障

### 没有 `map -> odom`

检查 `/scan`、`/map`、AMCL 生命周期和初始位姿。确认没有另一个
`slam_toolbox`/AMCL 同时发布该 TF：

```bash
ros2 node list
ros2 lifecycle get /map_server
ros2 run tf2_ros tf2_echo map odom
```

### `/odometry/filtered_map` 没有数据

它要求 `/odometry/filtered` 和 `map -> odom` 同时存在。先分别检查 EKF 和 AMCL，
再检查 `odom_to_map_relay` 日志。

### 地图中机器人位置正确，但 REMANI 障碍物整体偏移

2D 地图和 ESDF 的原点不一致。先确认是否来自同一建图会话，再核对 NPZ 的 origin。
只有纯平移误差才使用 `static_esdf_offset_*`；存在 yaw 或尺度误差时重新导出。

### JAKA 无法连接或 EDG 超时

先核对 URDF 中的实际 IP，而不是只看 launch 命令；再检查主机静态 IP、路由、UDP
防火墙和控制柜模式。运行时日志应打印与现场一致的 Robot/Local IP。

### 控制器已启动但机器人意外尝试回到某个姿态

立即急停。检查真实 `/joint_states` 是否在 OCS2 启动前稳定发布，确认
`start_arm_pose=false`，并核对 task 的 `initialState.arm` 与当前启动策略。首次上机不要
把“控制器 active”当成“不会运动”。

### 目标发布者不止一个

```bash
ros2 topic info /mobile_manipulator_mpc_target -v
```

停止 joy target、手工 target 或其他测试节点，只保留
`remani_to_ocs2_reference_bridge`。

## 8. 安全边界

这个 launch 负责系统组合，不是经过安全认证的保护系统。软件限速、碰撞代价、状态
超时和零速度命令都不能替代物理急停、安全围栏、机械限位及现场监护。首次实机测试
至少做到：

- 底盘架空/机械固定，机械臂低速，工作区无人；
- 开机前核对地图、初始位姿、TF、关节状态和命令发布者；
- 地图或定位跳变、控制方向错误、持续振荡时立即物理急停；
- 不在人员附近测试自动重规划；
- 每次换地图、URDF、task 或控制器配置后，都从小目标和最低速度重新验收。

## 9. 其他入口

| 场景 | 命令 |
| --- | --- |
| 保存地图 + AMCL + REMANI/OCS2 实机闭环 | `ros2 launch tracer_jaka_bringup remani_mpc_localized_real.launch.py` |
| 在线 SLAM + REMANI/OCS2 旧实机流程 | `ros2 launch tracer_jaka_bringup remani_mpc_real.launch.py` |
| 实机仅传感器/EKF/SLAM | `ros2 launch tracer_jaka_mujoco real_slam.launch.py` |
| 实机仅保存地图定位 | `ros2 launch tracer_jaka_localization localization_real.launch.py` |
| 实机仅 OCS2/JAKA/底盘 | `ros2 launch tracer_jaka_ocs2 ocs2_real.launch.py` |
| MuJoCo 完整闭环 | `ros2 launch tracer_jaka_ocs2 ocs2_sim.launch.py` |
