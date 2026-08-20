# tracer_jaka_bringup

WBMM 的顶层系统组合包，负责把定位/SLAM、REMANI 规划、OCS2 MPC/MRT 和实机硬件接口组合成可一键启动的 launch。

当前状态：

- 已提供实机 REMANI-MPC 组合入口：`remani_mpc_real.launch.py`
- 仿真完整闭环仍建议使用 `tracer_jaka_ocs2/ocs2_sim.launch.py`
- WipePlanner 恒力擦拭专用组合入口仍在规划中，尚未收敛到本包

## 1. 已提供的 launch

### `remani_mpc_real.launch.py`

面向真实 Tracer + JAKA + REMANI + OCS2 的顶层入口。
它会同时启动：

```text
real_slam.launch.py
├── Tracer CAN 底盘
├── robot_state_publisher
├── Hipnuc IMU
├── Lakibeam LiDAR
├── robot_localization EKF
└── slam_toolbox

ocs2_real.launch.py
├── JAKA ros2_control + joint_state_broadcaster
├── jaka_forward_controller
├── OCS2 MPC
└── OCS2 MRT

remani_mpc_tracking.launch.py
├── REMANI planner
└── REMANI -> OCS2 reference bridge
```

该 launch 避免了三类重复：

- 不会启动 `real_slam.launch.py` 里的假 `arm_pose_publisher`
- 不会让 `ocs2_real.launch.py` 再启动一份 Tracer 底盘
- 不会让 `ocs2_real.launch.py` 再启动一份 `robot_state_publisher`

同时保证：

- MRT 和 REMANI 都使用 `/odometry/filtered` 作为状态/里程计输入
- `odom -> base_footprint` 由 EKF 发布
- 默认关闭手柄目标发布，`/mobile_manipulator_mpc_target` 只由 REMANI bridge 发布

## 2. 构建

```bash
cd ~/WBMM
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-up-to \
  tracer_jaka_bringup \
  tracer_jaka_mujoco \
  tracer_jaka_ocs2 \
  remani_planner \
  grid_map

source install/setup.bash
```

## 3. 实机启动

### 3.1 准备

1. 配置 CAN：

   ```bash
   sudo ip link set can0 down
   sudo ip link set can0 up type can bitrate 500000
   ```

2. 设置权限：

   ```bash
   sudo chmod 777 /dev/ttyUSB0
   ```

3. 设置 ROS 环境：

   ```bash
   export ROS_DOMAIN_ID=20
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   ```

4. 准备 REMANI 使用的 ESDF NPZ：

   将电脑上用 nvblox 离线导出的 `d455_bag_remani_esdf.npz` 放到实机，例如：

   ```bash
   mkdir -p ~/WBMM/maps
   ```

### 3.2 一键启动

```bash
cd ~/WBMM
source install/setup.bash

ros2 launch tracer_jaka_bringup remani_mpc_real.launch.py \
  static_esdf_file:=/home/你的用户名/WBMM/maps/d455_bag_remani_esdf.npz \
  static_esdf_offset_x:=0.0 \
  static_esdf_offset_y:=0.0 \
  static_esdf_offset_z:=0.0
```

如果现场机器人 IP 不是默认值，请按实际覆盖：

```bash
ros2 launch tracer_jaka_bringup remani_mpc_real.launch.py \
  robot_ip:=10.5.5.100 \
  local_ip:=10.5.5.127 \
  static_esdf_file:=/path/to/d455_bag_remani_esdf.npz
```

> 注意：`ocs2_real.launch.py` 中 `robot_ip/local_ip` 目前主要用于参数透传；如果 JAKA 硬件插件里的 IP 仍然是 URDF 中硬编码的地址，请先确认 URDF 中的 IP 和现场一致。

### 3.3 关闭 RViz

```bash
ros2 launch tracer_jaka_bringup remani_mpc_real.launch.py \
  use_rviz:=false \
  static_esdf_file:=/path/to/d455_bag_remani_esdf.npz
```

### 3.4 第一次实机调试推荐参数

```bash
ros2 launch tracer_jaka_bringup remani_mpc_real.launch.py \
  use_rviz:=true \
  static_esdf_file:=/path/to/d455_bag_remani_esdf.npz \
  static_esdf_offset_x:=0.0 \
  static_esdf_offset_y:=0.0 \
  static_esdf_offset_z:=0.0 \
  tracking_error_replan_enabled:=false \
  manipulator_max_vel:=0.2 \
  manipulator_max_acc:=0.4 \
  start_arm_pose:=false
```

## 4. 主要参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_rviz` | `true` | 是否启动 OCS2 RViz |
| `use_joy` | `false` | 是否启动手柄目标发布。REMANI bridge 运行时必须保持 `false` |
| `can_port` | `can0` | Tracer CAN 口 |
| `serial_port` | `/dev/ttyUSB0` | Hipnuc IMU 串口 |
| `robot_ip` / `local_ip` | `10.5.5.100` / `10.5.5.127` | JAKA 网络参数透传 |
| `task_file` | `task_real.info` | OCS2 task 文件 |
| `urdf_file` | `tracer_jaka_zu5_real.urdf` | 实机 URDF |
| `lib_folder` | `/tmp/ocs2_tracer_jaka_real/auto_generated` | OCS2 自动生成目录 |
| `odom_topic` | `/odometry/filtered` | MRT 和 REMANI 共同使用的里程计话题 |
| `joint_state_topic` | `/joint_states` | 机械臂关节状态话题 |
| `static_esdf_file` | `grid_map/maps/tracer_jaka_zu5_scene_esdf.npz` | REMANI 静态 ESDF；实机必须替换为 nvblox 导出 NPZ |
| `static_esdf_offset_*` | `0.0` | ESDF 坐标系到当前 `odom` 的平移偏移 |
| `planner_to_ocs2_*` | `0.0` | REMANI 到 OCS2 的固定平面变换 |
| `tracking_error_replan_enabled` | `false` | 是否启用跟踪误差自动重规划 |
| `manipulator_max_vel` | `0.3` | REMANI 机械臂最大关节速度 |
| `manipulator_max_acc` | `0.5` | REMANI 机械臂最大关节加速度 |
| `freeze_manipulator` | `false` | 是否让 REMANI 在底盘规划时保持当前臂型 |
| `start_arm_pose` | `false` | 是否启动假 arm pose 发布器；实机 JAKA 在线时必须为 `false` |
| `start_imu` | `true` | 是否启动 IMU 驱动 |
| `start_lidar` | `true` | 是否启动 LiDAR 驱动 |

## 5. 启动后检查

```bash
ros2 topic hz /odom
ros2 topic hz /odometry/filtered
ros2 topic hz /joint_states
ros2 topic hz /scan
ros2 topic hz /IMU_data

ros2 topic echo /mobile_manipulator_mpc_observation --once
ros2 topic echo /remani_planner/fsm_state --once
```

在 RViz 中：

- Fixed Frame 可设为 `map` 或 `odom`
- 使用 `2D Goal Pose` 发送目标
- 查看 `/global_traj`、`/planning/trajectory` 等 REMANI 显示

完整链路检查：

```text
/goal_pose
  -> REMANI planner
  -> /planning/trajectory
  -> remani_to_ocs2_reference_bridge
  -> /mobile_manipulator_mpc_target
  -> OCS2 MPC/MRT
  -> /cmd_vel + /jaka_forward_controller/commands
```

## 6. 坐标对齐提示

- nvblox 离线 ESDF 默认导出在录 bag 时的 `odom` 坐标系。
- 如果实机重新启动后 `odom` 原点与录 bag 时不一致，REMANI 看到的静态地图会偏移。
- 第一次调试请尽量让机器人在同一个固定起点启动，或者使用本次实时 nvblox 建图结果。
- 如果确有固定平移/旋转，通过 `static_esdf_offset_*` 和 `planner_to_ocs2_*` 进行刚体对齐。

## 7. 相关入口

| 场景 | 命令 |
| --- | --- |
| 实机 REMANI-MPC 一键启动 | `ros2 launch tracer_jaka_bringup remani_mpc_real.launch.py` |
| 仿真完整闭环 | `ros2 launch tracer_jaka_ocs2 ocs2_sim.launch.py` |
| 实机仅 OCS2 遥控 | `ros2 launch tracer_jaka_ocs2 ocs2_real.launch.py` |
| 实机仅定位/SLAM | `ros2 launch tracer_jaka_mujoco real_slam.launch.py` |

## 8. 安全提醒

该 launch 只是把 REMANI 和 OCS2 组合起来，**不是完整安全系统**。
第一次上机必须：

- 轮子悬空或机械固定
- 机械臂低速、工作区清空
- 急停/示教器/遥控器在操作者手边
- 先关 `tracking_error_replan_enabled`
- 确认 `/joint_states` 只来自真实 JAKA
- 确认只有一个节点发布 `/cmd_vel` 和机械臂位置命令
- 确认底盘命令超时、JAKA 状态超时、MPC policy 失效时能安全停止
