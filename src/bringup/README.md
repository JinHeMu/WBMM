# tracer_jaka_bringup

> 状态：CURRENT  
> 包名：`tracer_jaka_bringup`  
> 源码目录：`src/bringup`  
> 作用：WBMM（Tracer 差速底盘 + JAKA Zu5 机械臂）的顶层组合、参数分层、launch、RViz、脚本和回归测试包。

本文档以当前源码为准，重点说明：

- 参数文件如何分层、如何覆盖；
- 每个 launch 文件启动什么、主要参数是什么；
- 实机与 MuJoCo 仿真的推荐启动方式；
- OCS2、REMANI、定位、MoveIt、力控和相机录制等功能的使用边界。

---

## 0. 一分钟上手

### 0.1 编译

```bash
cd ~/WBMM
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to tracer_jaka_bringup
source install/setup.bash
```

也可以使用部署脚本：

```bash
./deploy/build.sh
```

### 0.2 实机最小只读启动

```bash
# 先初始化 CAN 和 JAKA
./deploy/start.sh

# 只启动硬件接口，hardware_write=false，JAKA 只读遥测，不写命令
ros2 launch tracer_jaka_bringup wbmm_hardware_interface.launch.py \
  hardware_write:=false \
  can_port:=can0 \
  jaka_robot_ip:=10.5.5.100 \
  jaka_local_ip:=10.5.5.127
```

### 0.3 实机 OCS2 末端保持

```bash
ros2 launch tracer_jaka_bringup real_ocs2_ee_hold.launch.py \
  hardware_write:=true \
  use_target:=true \
  use_rviz:=true
```

默认 `hardware_write:=false` 是干跑/只读模式；确认机器人状态、TF、控制器、急停和现场安全后再显式设为 `true`。

### 0.4 MuJoCo 仿真

```bash
ros2 launch tracer_jaka_bringup mujoco_hardware_interface.launch.py \
  scene:=empty \
  viewer:=true
```

### 0.5 MuJoCo + OCS2 末端保持

```bash
ros2 launch tracer_jaka_bringup mujoco_ocs2_ee_hold.launch.py \
  scene:=empty \
  initial_task_phase:=2 \
  use_target:=true \
  use_rviz:=true
```

### 0.6 仿真力控自动测试

```bash
ros2 launch tracer_jaka_bringup force_control_mujoco_test.launch.py \
  viewer:=true \
  use_rviz:=true
```

---

## 1. 包定位与设计边界

`tracer_jaka_bringup` 是系统组合层，不是驱动或算法实现层。

它主要负责：

- 提供统一的实机 / 仿真硬件入口；
- 提供统一的算法入口：定位、OCS2、REMANI、力控、MoveIt；
- 提供组合入口：`remani_mpc`、`remani_mpc_localized`、`wbmm`；
- 维护 `common / real / sim` 三层参数结构；
- 提供 RViz 配置、辅助脚本和 launch 组合的回归测试。

它不负责：

- JAKA / Tracer / 传感器驱动的具体实现；
- OCS2、REMANI、Pinocchio、力控算法核心；
- 直接声明某个算法一定适合当前现场；
- 绕过硬件安全门。

### 1.1 设计原则

1. **硬件后端与算法解耦**  
   `wbmm_hardware_interface.launch.py` 和 `mujoco_hardware_interface.launch.py` 提供同一套 ROS 接口；算法 launch 不直接启动驱动。

2. **配置分层**  
   `common` 保存公共算法结构和安全默认值，`real` 保存实机标定/安全差异，`sim` 保存 MuJoCo 差异。

3. **运动门单一**  
   实机 JAKA 写入和 OCS2/MoveIt 输出都受 `hardware_write` 控制。默认 `false`。

4. **顶层默认 fail-closed**  
   `wbmm.launch.py` 默认 `hardware_backend:=none`，且所有算法 `start_*:=false`，不会误启动任何硬件或算法。

5. **统一接口命名**  
   `config/common/interface.yaml` 是 topic、frame、rate 的规范性契约。实机和仿真后端都应对齐。

---

## 2. 目录结构

```text
src/bringup/
├── CMakeLists.txt
├── package.xml
├── README.md                       # 本文件
├── launch/                         # 所有 launch 文件
├── config/
│   ├── common/                     # 公共算法结构、接口契约、安全默认值
│   ├── real/                       # 实机差异和标定
│   └── sim/                        # MuJoCo 仿真差异
├── rviz/                           # RViz 配置
├── scripts/
│   ├── arm_pose_publisher.py       # 发布固定 arm-up 关节状态
│   ├── odom_to_map_relay.py        # odom -> map 里程计 pose relay
│   └── readiness_check.py          # 只读运行时审计
└── test/                           # launch/config 契约回归测试
```

`launch/` 文件安装到：

```text
install/tracer_jaka_bringup/share/tracer_jaka_bringup/launch/
```

`config/` 和 `rviz/` 安装到：

```text
install/tracer_jaka_bringup/share/tracer_jaka_bringup/config/
install/tracer_jaka_bringup/share/tracer_jaka_bringup/rviz/
```

---

## 3. 编译、安装与环境

### 3.1 普通工作空间编译

```bash
cd ~/WBMM
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to tracer_jaka_bringup
source install/setup.bash
```

### 3.2 部署脚本编译

```bash
cd ~/WBMM
./deploy/build.sh
```

部署脚本会读取 `deploy/env/real.env`，并写入：

- `deploy/logs/build-*.log`
- `deploy/metadata/deployment_info.txt`

### 3.3 只编译本包

如果依赖已经安装或已编译：

```bash
colcon build --symlink-install --packages-select tracer_jaka_bringup
```

注意：只编译本包不会自动编译它 launch 中依赖的 `wbmm_ocs2_ros`、`whole_body_force_control`、`tracer_jaka_mujoco`、驱动等包。

---

## 4. 统一 ROS 接口契约

`config/common/interface.yaml` 是 WBMM 的规范性接口契约，不是普通的节点参数文件。当前主要约定如下。

### 4.1 Topics

| 名称 | 类型 | 方向 | 必需 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 算法 -> 硬件 | 是 |
| `/arm_controller/commands` | `std_msgs/msg/Float64MultiArray` | 算法 -> 硬件 | 是 |
| `/arm_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 算法 -> 硬件 | 否 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 硬件 -> 算法 | 是 |
| `/wheel/odometry` | `nav_msgs/msg/Odometry` | 硬件 -> 算法 | 是 |
| `/imu/data` | `sensor_msgs/msg/Imu` | 硬件 -> 算法 | 是 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 硬件 -> 算法 | 是 |
| `/fts_broadcaster/wrench` | `geometry_msgs/msg/WrenchStamped` | 硬件 -> 算法 | 是 |
| `/clock` | `rosgraph_msgs/msg/Clock` | MuJoCo -> 算法 | 仿真 |
| `/camera/d455/color/image_raw` | `sensor_msgs/msg/Image` | 硬件 -> 算法 | 否 |
| `/camera/d455/depth/image_raw` | `sensor_msgs/msg/Image` | 硬件 -> 算法 | 否 |
| `/camera/d455/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 硬件 -> 算法 | 否 |
| `/camera/d455/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 硬件 -> 算法 | 否 |

### 4.2 Frames

| 语义 | frame |
|---|---|
| 地图 | `map` |
| 连续里程计 | `odom` |
| 底盘平面基准 | `base_footprint` |
| IMU | `imu_link` |
| LiDAR | `laser_link` |
| 力/力矩传感器 | `jk_se_vi_200_link` |
| 末端工具 | `tool0` |

### 4.3 期望频率

| 数据 | 频率 |
|---|---|
| `joint_states` | 100 Hz |
| `wheel_odometry` | 50 Hz |
| `imu` | 100 Hz |
| `scan` | 30 Hz |
| `fts_wrench` | 125 Hz |

---

## 5. 参数系统

### 5.1 运行时合并顺序

```text
config/common/<name>.yaml
  -> config/real/<name>.yaml 或 config/sim/<name>.yaml
  -> launch 显式参数
```

后加载的同名参数覆盖先加载的参数。显式 launch 参数优先级最高。

例如力控节点实际加载顺序为：

```text
common/force_control.yaml
  -> real/force_control.yaml 或 sim/force_control.yaml
  -> launch 中传入的 profile / gate 参数
```

### 5.2 配置文件清单

| 文件 | 作用 |
|---|---|
| `config/common/interface.yaml` | WBMM 统一 topic、frame、rate 契约 |
| `config/common/ekf.yaml` | 公共 `robot_localization` EKF 参数 |
| `config/common/slam_toolbox.yaml` | 公共 SLAM Toolbox 建图参数 |
| `config/common/ocs2.yaml` | OCS2 MPC/MRT/target 公共 ROS 参数 |
| `config/common/remani.yaml` | REMANI 公共跟踪/重规划/安全阈值 |
| `config/common/force_control.yaml` | 力传感器处理与导纳控制公共参数 |
| `config/common/moveit_bringup.yaml` | MoveIt 相关辅助节点参数；当前主要保留夹爪驱动参数 |
| `config/real/ekf.yaml` | 实机 EKF 覆盖：sensor timeout、时间偏移 |
| `config/real/force_control.yaml` | 实机力控覆盖：负载补偿模型、导纳门默认关闭 |
| `config/real/remani.yaml` | 实机 REMANI 覆盖：速度、加速度、冻结机械臂等 |
| `config/real/task.info` | 实机 OCS2 任务定义、双参考权重、碰撞与限位 |
| `config/real/d455_esdf_record_qos.yaml` | 实机 D455 ESDF 录制 QoS 覆盖 |
| `config/sim/ekf.yaml` | MuJoCo EKF 覆盖：IMU 配置、噪声协方差 |
| `config/sim/force_control.yaml` | 仿真力控覆盖：fake wrench、较低硬限幅、导纳默认开启 |
| `config/sim/remani.yaml` | 仿真 REMANI 覆盖：更高速度、允许机械臂运动 |
| `config/sim/slam_toolbox.yaml` | 仿真 SLAM 覆盖：较短雷达量程、较低更新频率 |
| `config/sim/ocs2.yaml` | 仿真 OCS2 覆盖：`use_sim_time`、输出、控制话题 |
| `config/sim/task.info` | 仿真 OCS2 任务定义 |

### 5.3 OCS2 关键参数

公共文件：`config/common/ocs2.yaml`

| 节点 | 参数 | 默认值 | 说明 |
|---|---|---|---|
| `wbmm_mpc_node` | `whole_body_target_topic` | `mobile_manipulator_whole_body_target` | 9D 全身目标 |
| `wbmm_mpc_node` | `ee_target_topic` | `mobile_manipulator_ee_target` | 7D 末端目标 |
| `wbmm_mpc_node` | `task_phase_state_topic` | `mobile_manipulator_task_phase_state` | 当前 TaskPhase 状态 |
| `wbmm_mpc_node` | `set_task_phase_service` | `mobile_manipulator_set_task_phase` | 运行时切换 phase |
| `wbmm_mpc_node` | `initial_task_phase` | `-1` | 使用 `task.info` 中的 initialPhase |
| `wbmm_mrt_node` | `command_output_enabled` | `false` | MRT 命令输出总门 |
| `wbmm_mrt_node` | `base_cmd_topic` | `/cmd_vel` | 底盘速度输出 |
| `wbmm_mrt_node` | `odom_topic` | `/odometry/filtered` | 状态反馈 |
| `wbmm_mrt_node` | `arm_cmd_topic` | `/arm_controller/commands` | 机械臂位置命令 |
| `wbmm_mrt_node` | `world_frame` | `odom` | OCS2 控制参考系 |
| `wbmm_mrt_node` | `ee_frame` | `tool0` | 末端 frame |
| `wbmm_mrt_node` | `arm_max_command_velocity` | `0.15` | 机械臂命令速度限制 |
| `wbmm_mrt_node` | `arm_max_delta_per_step` | `0.05` | 每步关节增量限制 |
| `wbmm_target_node` | `marker_frame` | `odom` | RViz 目标 marker frame |
| `wbmm_target_node` | `ee_frame` | `tool0` | 目标对应的末端 frame |
| `wbmm_target_node` | `input_dim` | `8` | 目标输入维度 |

仿真覆盖：`config/sim/ocs2.yaml`

- `use_sim_time: true`
- `command_output_enabled: true`
- `traj_horizon: 1.0`
- `arm_max_command_velocity: 0.50`
- `arm_max_delta_per_step: 0.50`
- `base_cmd_topic: /base_controller/cmd_vel`

实机没有单独的 `config/real/ocs2.yaml`，默认直接使用公共参数；实机运动门由 launch 的 `hardware_write` 控制。

### 5.4 定位关键参数

公共 EKF：`config/common/ekf.yaml`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `frequency` | `50.0` | EKF 更新频率 |
| `two_d_mode` | `true` | 二维模式 |
| `publish_tf` | `true` | 发布 `odom -> base_footprint` |
| `world_frame` | `odom` | EKF 世界 frame |
| `odom0` | `/wheel/odometry` | 轮式里程计输入 |
| `imu0` | `/imu/data` | IMU 输入 |

实机覆盖：`config/real/ekf.yaml`

- `sensor_timeout: 0.2`
- `transform_time_offset: 0.0`

仿真覆盖：`config/sim/ekf.yaml`

- `sensor_timeout: 0.1`
- `transform_time_offset: 0.05`
- IMU 使用角速度和 yaw 姿态
- `imu0_remove_gravitational_acceleration: true`
- 覆盖 `process_noise_covariance`

公共 SLAM：`config/common/slam_toolbox.yaml`

- `mode: mapping`
- `odom_frame: odom`
- `map_frame: map`
- `base_frame: base_footprint`
- `scan_topic: /scan`
- `resolution: 0.05`
- 开启 scan matching、loop closing

仿真覆盖：`config/sim/slam_toolbox.yaml`

- `use_sim_time: true`
- `max_laser_range: 8.0`
- `minimum_time_interval: 0.10`
- `scan_buffer_maximum_scan_distance: 10.0`

### 5.5 力控关键参数

公共文件：`config/common/force_control.yaml`

#### `force_sensor_processor`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `topics.raw_wrench` | `/fts_broadcaster/wrench` | 原始 F/T |
| `topics.processed_wrench` | `/whole_body_force_control/processed_wrench` | 处理后输出 |
| `force_sensor.sensor_frame` | `jk_se_vi_200_link` | 原始传感器 frame |
| `force_sensor.tcp_frame` | `tool0` | 导纳控制 frame |
| `tf_lookup_timeout` | `0.05` | TF 查询超时 |
| `tf_fallback_to_latest` | `false` | 是否回退 latest TF |
| `raw_timeout` | `0.10` | 原始 wrench 超时 |
| `tare_samples` | `50` | 自动 tare 采样数 |
| `filter_alpha` | `0.15` | 低通滤波系数 |
| `wrench_scale` | `[1,1,1,1,1,1]` | 六轴缩放 |
| `hard_force_norm_limit` | `50.0` | 合力范数硬限幅 |
| `hard_wrench_limit` | `[50,50,50,2,2,2]` | 六轴硬限幅 |
| `force_deadband_n` | `0.0` | 力死区 |
| `torque_deadband_nm` | `0.0` | 力矩死区 |
| `load_compensation.enable` | `false` | 是否启用负载重力补偿 |

#### `whole_body_force_control`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `state_frame` | `odom` | 参考输出 frame |
| `force_sensor.tcp_frame` | `tool0` | 处理后的 wrench frame |
| `force_sensor.force_timeout` | `0.10` | 力数据超时 |
| `admittance.enable` | `false` | 是否进入导纳控制 |
| `admittance.output` | `false` | 是否向 OCS2 发布参考 |
| `admittance.selected_axes` | `[true,false,false,false,false,false]` | 六轴选择，顺序 `[Fx,Fy,Fz,Tx,Ty,Tz]` |
| `admittance.mass` | `[3,3,3,0.3,0.3,0.3]` | 质量参数 |
| `admittance.damping` | `[45,45,45,4.5,4.5,4.5]` | 阻尼参数 |
| `admittance.stiffness` | `[0,0,0,0,0,0]` | 刚度参数；`K>0` 弹性返回，`K=0` 力跟随 |
| `admittance.max_velocity` | `[0.25,0.25,0.25,0.15,0.15,0.15]` | 六轴修正速度上限 |
| `whole_body.base_share` | `0.6` | 仅 legacy `whole_body_state` 模式使用 |
| `whole_body.max_base_velocity` | `0.20` | legacy 底盘速度上限 |
| `whole_body.max_joint_velocity` | `0.50` | legacy 关节速度上限 |
| `whole_body.max_ee_linear_velocity` | `0.2` | ee_pose 模式线速度上限 |
| `whole_body.max_ee_angular_velocity` | `0.5` | ee_pose 模式角速度上限 |
| `safety.observation_timeout` | `0.20` | 状态超时 |
| `safety.capture_settle_time` | `1.0` | 捕获稳定时间 |
| `safety.enforce_single_target_owner` | `true` | 强制单一目标所有者 |
| `output.mode` | `ee_pose` | 推荐输出模式；`whole_body_state` 为 legacy |

实机覆盖：`config/real/force_control.yaml`

- `admittance.enable: false`
- `admittance.output: false`
- 启用辨识后的末端负载补偿模型
- `load_compensation` 中包含 `mass_kg`、`gravity_direction_base`、质心和偏置

仿真覆盖：`config/sim/force_control.yaml`

- `raw_wrench: /whole_body_force_control/fake_wrench`
- `admittance.enable: true`
- `admittance.output: true`
- 选择三轴平移
- `stiffness: [150,150,150,0,0,0]`
- 硬限幅降低到 `[20,20,20,4,4,4]`

### 5.6 REMANI 关键参数

公共文件：`config/common/remani.yaml`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `tracking_error_replan_enabled` | 实机 false / 仿真 true | 跟踪误差触发重规划 |
| `tracking_error_position_threshold` | `0.30` | 位置误差阈值 |
| `tracking_error_yaw_threshold` | `0.45` | yaw 误差阈值 |
| `tracking_error_joint_threshold` | `0.30` | 关节误差阈值 |
| `tracking_error_persistence` | `0.30` | 误差持续时间 |
| `tracking_goal_position_tolerance` | `0.12` | 目标位置容差 |
| `tracking_goal_yaw_tolerance` | `0.20` | 目标 yaw 容差 |
| `tracking_goal_joint_tolerance` | `0.15` | 目标关节容差 |
| `max_consecutive_planning_failures` | `20` | 连续规划失败上限 |
| `manipulator_safe_margin` | `0.10` | 机械臂安全裕量 |
| `general_safe_margin` | `0.10` | 通用安全裕量 |
| `self_safe_margin` | `0.10` | 自碰撞安全裕量 |

实机覆盖：`config/real/remani.yaml`

- `tracking_error_replan_enabled: false`
- `manipulator_max_vel: 0.10`
- `manipulator_max_acc: 0.20`
- `mobile_base_max_wheel_omega: 1.0`
- `mobile_base_max_wheel_alpha: 2.0`
- `freeze_manipulator: true`

仿真覆盖：`config/sim/remani.yaml`

- `tracking_error_replan_enabled: true`
- `manipulator_max_vel: 1.57`
- `manipulator_max_acc: 3.14`
- `freeze_manipulator: false`

### 5.7 `task.info` 与 TaskPhase

`config/real/task.info` 和 `config/sim/task.info` 是 OCS2 的完整任务定义，不是普通 YAML。

当前双参考模式切换：

| phase | 名称 | whole-body 权重 | EE 权重 | 说明 |
|---|---|---|---|---|
| `0` | Navigation | `1.0` | `0.0` | 全身参考主导 |
| `1` | Transition | `0.5` | `0.5` | 全身与末端共同加权 |
| `2` | Execution | `0.0` | `1.0` | 末端执行主导，避免被初始全身参考拉回 |
| `3` | Retract | `0.5` | `0.0` | 全身重新主导 |

运行时通过服务切换：

```bash
ros2 service call /mobile_manipulator_set_task_phase \
  wbmm_ocs2_ros/srv/SetTaskPhase "{phase: 2}"

# 也可以使用包内脚本
ros2 run wbmm_ocs2_ros set_task_phase.py 2
```

`ocs2.launch.py` 的 `initial_task_phase` 可取 `-1..3`：

- `-1`：使用 `task.info` 里的 `modeSwitch.initialPhase`
- `0/1/2/3`：显式覆盖 MPC 和 MRT 初始 phase

力控 Execution 通常需要显式切到 `2`，或把 `initial_task_phase` 设为 `2`。

### 5.8 MoveIt 参数

`config/common/moveit_bringup.yaml` 当前主要保存 `dh_ag95_driver` 参数：

| 参数 | 默认值 |
|---|---|
| `baudrate` | `115200` |
| `gripper_id` | `1` |
| `max_position` | `100.0` |
| `max_force` | `100.0` |

`moveit.launch.py` 本身只暴露：

- `use_sim_time`
- `use_rviz`
- `hardware_write`

其中 `hardware_write` 直接映射到 MoveIt 的 `allow_trajectory_execution`。

---

## 6. Launch 文件总览

| Launch 文件 | 作用 | 启动硬件 | 启动算法 |
|---|---|---|---|
| `wbmm.launch.py` | 顶层可选组合入口 | 可选 real / mujoco | 可选 localization / OCS2 / REMANI / force / MoveIt |
| `wbmm_hardware_interface.launch.py` | 实机硬件接口 | 是 | 否 |
| `mujoco_hardware_interface.launch.py` | MuJoCo 硬件接口 | 仿真 | 否 |
| `localization.launch.py` | EKF + 可选 SLAM | 否 | 是 |
| `ocs2.launch.py` | OCS2 MPC/MRT | 否 | 是 |
| `remani.launch.py` | REMANI 规划器 + OCS2 桥 | 否 | 是 |
| `whole_body_force_control.launch.py` | 实机力控算法入口 | 否 | 是 |
| `whole_body_force_control_profiles.launch.py` | 仿真力控 profile 入口 | 否 | 是 |
| `moveit.launch.py` | MoveIt move_group + RViz | 否 | 是 |
| `remani_mpc.launch.py` | 定位 + OCS2 + REMANI 组合 | 否 | 是 |
| `remani_mpc_localized.launch.py` | EKF + AMCL + OCS2 + REMANI | 否 | 是 |
| `real_ocs2_ee_hold.launch.py` | 实机 + OCS2 末端保持 | 是 | 是 |
| `mujoco_ocs2_ee_hold.launch.py` | MuJoCo + OCS2 末端保持 | 仿真 | 是 |
| `force_control_mujoco_test.launch.py` | MuJoCo 力控 fake wrench 自动测试 | 仿真 | 是 |
| `force_control_20s_follow_test.launch.py` | MuJoCo 20 秒持续力跟随测试 | 仿真 | 是 |
| `force_control_infinite_follow_test.launch.py` | MuJoCo 无限场景持续力跟随测试 | 仿真 | 是 |
| `d435_camera.launch.py` | 臂端 D435 驱动 | 是 | 否 |
| `d455_camera.launch.py` | 底盘 D455 驱动 | 是 | 否 |
| `record_d455_esdf_bag.launch.py` | 实机 D455 ESDF 数据录制 | 是 | 否 |

---

## 7. 核心 Launch 参数详解

### 7.1 `wbmm.launch.py`

顶层组合入口。默认不启动任何硬件和算法。

```bash
# 查看所有参数
ros2 launch tracer_jaka_bringup wbmm.launch.py --show-args

# fail-closed 启动：只加载组合逻辑，不启动硬件/算法
ros2 launch tracer_jaka_bringup wbmm.launch.py \
  hardware_backend:=none \
  config_profile:=real
```

主要参数：

| 参数 | 默认值 | 可选值 | 说明 |
|---|---|---|---|
| `hardware_backend` | `none` | `none/real/mujoco` | 选择硬件后端 |
| `config_profile` | `real` | `real/sim` | 选择默认配置路径 |
| `use_sim_time` | `auto` | `auto/true/false` | `auto` 时 mujoco 为 true，其他为 false |
| `use_rviz` | `true` | bool | 传给 OCS2 / 力控 / MoveIt |
| `hardware_write` | `false` | bool | 实机运动门；也传给 OCS2 输出和 MoveIt |
| `viewer` | `true` | bool | MuJoCo viewer |
| `start_camera` | `false` | bool | MuJoCo 相机 |
| `publish_odom_tf` | `false` | bool | 仅 `hardware_backend:=mujoco` 时转发；real 分支未转发，实机后端保持默认 `false` |
| `scene` | `empty` | 见下 | MuJoCo 场景名 |
| `mujoco_model` | `""` | 路径 | 显式 MuJoCo XML，优先于 `scene` |
| `initial_pose` | `low` | `low/home/task_contact` | MuJoCo 初始 keyframe |
| `init_keyframe` | `""` | keyframe 名 | 显式 keyframe 覆盖 |
| `can_port` | `can0` | CAN 接口 | Tracer 底盘 |
| `robot_ip` | `10.5.5.100` | IP | JAKA 控制器 IP |
| `local_ip` | `10.5.5.127` | IP | 本机 JAKA 网口 IP |
| `start_localization` | `false` | bool | 启动 EKF / SLAM |
| `start_slam` | `true` | bool | 启动 SLAM Toolbox |
| `start_ocs2` | `false` | bool | 启动 OCS2 |
| `start_remani` | `false` | bool | 启动 REMANI |
| `start_force_control` | `false` | bool | 启动力控；会同时启动 OCS2 |
| `start_moveit` | `false` | bool | 启动 MoveIt |
| `urdf_file` | description 包 URDF | 路径 | 算法用 URDF |
| `task_file` | 按 profile 选择 | 路径 | OCS2 `task.info` |
| `ocs2_config` | `""` | 路径 | OCS2 覆盖配置；sim 默认 `config/sim/ocs2.yaml` |
| `remani_config` | `""` | 路径 | REMANI 覆盖配置 |
| `ekf_config` | `""` | 路径 | EKF 覆盖配置 |
| `slam_config` | `""` | 路径 | SLAM 覆盖配置 |
| `force_params_file` | `""` | 路径 | 力控覆盖配置 |
| `lib_folder` | 按 profile 生成 | 路径 | OCS2 生成库目录 |
| `odom_topic` | `/wheel/odometry` | topic | OCS2 / REMANI 使用的里程计 |
| `joint_state_topic` | `/joint_states` | topic | REMANI 关节状态 |
| `static_esdf_file` | `""` | `.npz` | REMANI 静态 ESDF；`start_remani:=true` 时必需 |

MuJoCo 场景名：

| `scene` | XML |
|---|---|
| `empty` | `scene_empty.xml` |
| `room` | `scene.xml` |
| `task_table` | `scene_task_table.xml` |
| `force_follow_infinite` | `scene_force_follow_infinite.xml` |
| `force_follow_5m` | `scene_force_follow_5m.xml` |
| `nvblox_remani_demo` | `scene_nvblox_remani_demo.xml` |
| `esdf_validation` | `scene_esdf_validation.xml` |

实机分支限制：

- 顶层 `wbmm.launch.py hardware_backend:=real` 只向 `wbmm_hardware_interface.launch.py` 转发 `hardware_write`、`can_port`、`robot_ip`、`local_ip`、`start_arm_controller` 和控制器名；
- 它不会转发 `publish_odom_tf`、`start_imu`、`start_lidar`、`start_jaka_fts` 等实机硬件参数；
- 需要精细控制实机传感器、TF 发布或 F/T broadcaster 时，直接使用 `wbmm_hardware_interface.launch.py`；
- 因为 real 分支不会转发 `publish_odom_tf:=true`，如果顶层实机组合没有启动 EKF，就不会有 `odom -> base_footprint` TF；OCS2/REMANI 需要完整 TF 时，应直接使用硬件入口并设置 `publish_odom_tf:=true`，或通过 `start_localization:=true` 启动 EKF。

组合规则：

- `hardware_write` 同时传给实机硬件、OCS2 的 `command_output_enabled` 和 MoveIt 的 `allow_trajectory_execution`，是顶层组合入口的统一运动门。
- `start_force_control:=true` 已经包含 OCS2，不能再同时设置 `start_ocs2:=true`，否则 launch 会报错。
- 顶层入口不转发 `admittance.enable` / `admittance.output`；`whole_body_force_control.launch.py` 的默认 `false` 会覆盖 profile YAML。仿真力控请优先使用 `whole_body_force_control_profiles.launch.py` 或专用测试 launch。
- `start_moveit:=true` 且没有 OCS2/力控时，硬件会尝试启动 `arm_trajectory_controller`。
- `start_moveit:=true` 且有 OCS2/力控时，硬件启动 `arm_controller`，由 OCS2/力控拥有机械臂命令。
- `start_remani:=true` 时必须提供存在的 `static_esdf_file`。

### 7.2 `wbmm_hardware_interface.launch.py`

实机硬件入口。

启动内容：

- `robot_state_publisher`
- `controller_manager/ros2_control_node`
- `joint_state_broadcaster` spawner
- 可选 `arm_controller` / `arm_trajectory_controller` spawner
- 可选 `fts_broadcaster` spawner
- `tracer_base_node`
- `hipnuc_imu/talker`
- `lakibeam1_scan_node`
- 可选 `arm_pose_publisher.py`

主要参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `start_base` | `true` | 启动 Tracer 底盘驱动 |
| `start_robot_state_publisher` | `true` | 启动 RSP |
| `start_arm_pose` | `false` | 发布固定 arm-up 关节状态 |
| `start_imu` | `true` | 启动 Hipnuc IMU |
| `start_lidar` | `true` | 启动 Lakibeam LiDAR |
| `start_jaka_hardware` | `true` | 启动 JAKA ros2_control |
| `start_jaka_fts` | `true` | 启动 JAKA F/T broadcaster |
| `start_arm_controller` | `false` | 是否 spawn 机械臂控制器 |
| `arm_controller_name` | `arm_controller` | spawn 的控制器名 |
| `controller_manager_timeout` | `30.0` | spawner 等待超时 |
| `hardware_write` | `false` | **唯一实机运动门**；false 时 JAKA 只读 |
| `jaka_robot_ip` | `10.5.5.100` | JAKA 控制器 IP |
| `jaka_local_ip` | `10.5.5.127` | 本机网口 IP |
| `can_port` | `can0` | Tracer CAN |
| `serial_port` | `/dev/ttyUSB0` | IMU 串口 |
| `wheel_odom_topic` | `/wheel/odometry` | 底盘里程计话题 |
| `publish_odom_tf` | `false` | tracer_base 是否发布 `odom -> base_footprint` |
| `imu_topic` | `/imu/data` | IMU 话题 |
| `scan_topic` | `/scan` | 雷达话题 |
| `lidar_host_ip` | `0.0.0.0` | 雷达 host IP |
| `lidar_sensor_ip` | `192.168.198.2` | 雷达 sensor IP |
| `lidar_port` | `2368` | 雷达端口 |
| `lidar_inverted` | `false` | 雷达是否反转 |
| `lidar_angle_offset` | `0` | 雷达角度偏移 |
| `configure_lidar` | `false` | 是否通过 HTTP 重配置雷达 |

使用注意：

- 启动 EKF 时保持 `publish_odom_tf:=false`，让 `robot_localization` 唯一发布 `odom -> base_footprint`。
- 只使用原始轮式里程计时，才考虑 `publish_odom_tf:=true`。
- `hardware_write:=true` 只打开 JAKA 写入能力，本 launch 自身不会产生运动目标。
- 如果使用 MoveIt，需要显式设置 `start_arm_controller:=true arm_controller_name:=arm_trajectory_controller`，或通过 `wbmm.launch.py start_moveit:=true` 组合。

### 7.3 `mujoco_hardware_interface.launch.py`

MuJoCo 仿真硬件入口。

启动内容：

- `tracer_jaka_mujoco/mujoco_bridge`
- 可选 `robot_state_publisher`

主要参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `scene` | `empty` | 命名场景 |
| `model` | `""` | 显式 XML，优先于 `scene` |
| `initial_pose` | `low` | `low/home/task_contact` |
| `init_keyframe` | `""` | 显式 keyframe，优先于 `initial_pose` |
| `viewer` | `true` | MuJoCo viewer |
| `start_robot_state_publisher` | `true` | 启动 RSP |
| `publish_odom_tf` | `false` | 是否由 MuJoCo 桥发布 `odom -> base_footprint` |
| `start_imu` | `true` | 发布 `/imu/data` |
| `start_lidar` | `true` | 发布 `/scan` |
| `start_camera` | `false` | 发布 D455 图像 |
| `start_fts` | `true` | 发布 `/fts_broadcaster/wrench` |
| `wheel_odom_topic` | `/wheel/odometry` | 轮式里程计 |
| `imu_topic` | `/imu/data` | IMU |
| `scan_topic` | `/scan` | 雷达 |
| `fts_topic` | `/fts_broadcaster/wrench` | F/T |
| `color_image_topic` | `/camera/d455/color/image_raw` | 彩色图 |
| `color_camera_info_topic` | `/camera/d455/color/camera_info` | 彩色相机信息 |
| `depth_image_topic` | `/camera/d455/depth/image_raw` | 深度图 |
| `depth_camera_info_topic` | `/camera/d455/depth/camera_info` | 深度相机信息 |

### 7.4 `localization.launch.py`

只启动定位算法，不启动硬件。

启动内容：

- `robot_localization/ekf_node`
- 可选 `slam_toolbox/async_slam_toolbox_node`

主要参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `start_ekf` | `true` | 启动 EKF |
| `start_slam` | `true` | 启动 SLAM |
| `use_sim_time` | `false` | 仿真时设 true |
| `ekf_base_config` | `config/common/ekf.yaml` | EKF 公共配置 |
| `slam_base_config` | `config/common/slam_toolbox.yaml` | SLAM 公共配置 |
| `ekf_config` | `""` | EKF 覆盖配置 |
| `slam_config` | `""` | SLAM 覆盖配置 |
| `wheel_odom_topic` | `/wheel/odometry` | EKF 轮式里程计输入 |
| `imu_topic` | `/imu/data` | EKF IMU 输入 |
| `scan_topic` | `/scan` | SLAM 雷达输入 |

TF 所有权：

```text
slam_toolbox:       map -> odom
robot_localization: odom -> base_footprint
robot_state_publisher:
                    base_footprint -> base_link -> sensors / arm
```

### 7.5 `ocs2.launch.py`

只启动 OCS2 算法，不启动硬件。

启动内容：

- `wbmm_mpc_node`
- `wbmm_mrt_node`
- 可选 `wbmm_target_node`
- 可选 RViz2

主要参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `base_config_file` | `config/common/ocs2.yaml` | OCS2 公共配置 |
| `config_file` | `""` | 可选覆盖配置 |
| `task_file` | `""` | 必需，OCS2 `task.info` |
| `urdf_file` | `""` | 必需，机器人 URDF |
| `lib_folder` | `""` | 必需，生成库根目录 |
| `use_sim_time` | `false` | 仿真时 true |
| `use_target` | `false` | 启动 RViz 交互目标节点 |
| `use_rviz` | `true` | 启动 RViz |
| `initial_task_phase` | `-1` | `-1` 使用 task 文件；`0..3` 显式覆盖 |
| `command_output_enabled` | `false` | MRT 命令输出总门 |
| `odom_topic` | `/odometry/filtered` | OCS2 状态反馈 |
| `rviz_config` | `wbmm_ocs2_ros` RViz | RViz 配置 |

MPC 与 MRT 使用独立的生成目录：

```text
<lib_folder>/mpc
<lib_folder>/mrt
```

### 7.6 `remani.launch.py`

只启动 REMANI 规划器与 REMANI -> OCS2 参考桥，不启动 OCS2 本身。

启动内容：

- `remani_planner/remani_planner_node`
- `wbmm_ocs2_ros/remani_to_ocs2_reference_bridge`

主要参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `use_sim_time` | `false` | 仿真时 true |
| `start_planner` | `true` | 启动 REMANI 规划器 |
| `start_bridge` | `true` | 启动参考桥 |
| `urdf_file` | `""` | 必需 |
| `static_esdf_file` | `""` | 必需，REMANI 格式 `.npz` |
| `odom_topic` | `/odometry/filtered` | 规划输入里程计 |
| `joint_state_topic` | `/joint_states` | 规划输入关节状态 |
| `planner_frame` | `odom` | 规划 frame |
| `target_frame` | `odom` | 桥输出目标 frame |
| `use_tf_transform` | `false` | frame 不同时必须为 true |
| `base_config_file` | `config/common/remani.yaml` | 公共 REMANI 配置 |
| `config_file` | `""` | 覆盖配置 |

规则：

- `planner_frame != target_frame` 时必须 `use_tf_transform:=true`，不能使用固定 launch 偏移。
- `static_esdf_file` 必须存在。
- 桥会读取 REMANI 输出的 `/planning/trajectory`，转换成 OCS2 参考。

### 7.7 `whole_body_force_control.launch.py`

实机 / 通用力控算法入口，不启动硬件。

启动内容：

- `ocs2.launch.py`
- `force_sensor_processor_node`
- `whole_body_force_control_node`

主要参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `use_rviz` | `true` | 启动 RViz |
| `use_sim_time` | `false` | 仿真时 true |
| `hardware_write` | `false` | 实机运动门 |
| `initial_task_phase` | `-1` | `-1` 使用 task 文件；力控 Execution 建议设 `2` |
| `admittance.enable` | `false` | 是否进入导纳控制 |
| `admittance.output` | `false` | 是否发布力控参考 |
| `urdf_file` | description URDF | OCS2 和力控使用 |
| `task_file` | `config/real/task.info` | 实机任务 |
| `ocs2_config` | `""` | 可选 OCS2 覆盖 |
| `force_base_params_file` | `config/common/force_control.yaml` | 公共力控参数 |
| `force_params_file` | `config/real/force_control.yaml` | 实机力控覆盖 |
| `lib_folder` | `/tmp/wbmm_ocs2_auto_generated` | OCS2 生成库目录 |
| `odom_topic` | `/wheel/odometry` | 状态反馈 |

安全门：

- `admittance.output:=true` 时必须同时 `hardware_write:=true`，否则 launch 直接报错。
- 默认 shadow 模式为 `hardware_write:=false`、`admittance.enable:=false`、`admittance.output:=false`。

### 7.8 `whole_body_force_control_profiles.launch.py`

仿真力控算法入口，不启动 MuJoCo。

启动内容：

- `wbmm_mpc_node`
- `wbmm_mrt_node`
- `force_sensor_processor_node`
- `whole_body_force_control_node`
- 可选 RViz2

参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `profile` | `sensor_z` | 选择实验 profile |
| `use_rviz` | `true` | 启动 RViz |

内置 profile：

| profile | 对应/推荐 MuJoCo 场景 | 特点 |
|---|---|---|
| `sensor_z` | `scene_force_follow_infinite.xml` | Z 轴平移导纳，`K=150` |
| `three_axis_admittance` | `scene_force_follow_infinite.xml` | 三轴平移弹性导纳 |
| `three_axis_follow` | `scene_force_follow_infinite.xml` | 三轴平移力跟随，`K=0` |
| `infinite` | `scene_force_follow_infinite.xml` | 单轴无限力跟随，`K=0` |
| `20s` | `scene_force_follow_5m.xml` | 有限行程弹性终点，`K=1` |
| `six_axis_sequence` | `scene_force_follow_infinite.xml` | 六轴力/力矩序列 |

注意：profile 字典里的 `mujoco_model` / `init_keyframe` 当前没有在 `_launch_nodes` 中用于启动硬件；对应 MuJoCo 场景需要单独用 `mujoco_hardware_interface.launch.py` 或测试入口启动。

该 launch 使用：

```text
config/common/ocs2.yaml + config/sim/ocs2.yaml
config/common/force_control.yaml + config/sim/force_control.yaml
config/sim/task.info
```

因此它假定仿真环境和 `use_sim_time=true`。

### 7.9 `moveit.launch.py`

只启动 MoveIt 和 RViz，不启动硬件。

启动内容：

- `moveit_ros_move_group/move_group`
- 可选 RViz2

参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `use_sim_time` | `false` | 仿真时 true |
| `use_rviz` | `true` | 启动 MoveIt RViz |
| `hardware_write` | `false` | 映射为 `allow_trajectory_execution` |

使用条件：

- 实机需要先启动带 `arm_trajectory_controller` 的硬件后端。
- MuJoCo 后端本身提供 `/arm_trajectory_controller/follow_joint_trajectory` action。
- `hardware_write:=true` 才允许 MoveIt 真正执行轨迹。

### 7.10 `remani_mpc.launch.py`

组合入口：定位 + OCS2 + REMANI，不启动硬件。

启动内容：

- `localization.launch.py`：EKF + 可选 SLAM
- 延迟 8 s：`ocs2.launch.py`
- 延迟 12 s：`remani.launch.py`

主要参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `use_sim_time` | `false` | 仿真时 true |
| `use_rviz` | `true` | 启动 RViz |
| `hardware_write` | `false` | OCS2 输出门 |
| `start_slam` | `true` | 是否启动 SLAM |
| `static_esdf_file` | `""` | **必需**，启动前检查存在 |
| `odom_topic` | `/odometry/filtered` | OCS2 / REMANI 输入 |
| `joint_state_topic` | `/joint_states` | REMANI 输入 |
| `scan_topic` | `/scan` | SLAM 输入 |
| `wheel_odom_topic` | `/wheel/odometry` | EKF 输入 |
| `imu_topic` | `/imu/data` | EKF 输入 |
| `urdf_file` | description URDF | 算法 URDF |
| `task_file` | `config/real/task.info` | OCS2 任务 |
| `ocs2_config` | `""` | 可选 OCS2 覆盖 |
| `remani_config` | `config/real/remani.yaml` | REMANI 覆盖 |
| `ekf_config` | `config/real/ekf.yaml` | EKF 覆盖 |
| `slam_config` | `""` | 可选 SLAM 覆盖 |
| `lib_folder` | `/tmp/wbmm_ocs2_auto_generated` | OCS2 生成库 |
| `planner_frame` | `odom` | REMANI 规划 frame |
| `target_frame` | `odom` | OCS2 目标 frame |
| `use_tf_transform` | `false` | frame 不同时必须 true |

### 7.11 `remani_mpc_localized.launch.py`

组合入口：EKF + AMCL 保存地图定位 + OCS2 + REMANI，不启动硬件。

启动内容：

- `localization.launch.py`：EKF，`start_slam=false`
- `tracer_jaka_localization/amcl_localization.launch.py`：`map_server` + `amcl`
- `odom_to_map_relay.py`
- `ocs2.launch.py`
- 延迟 15 s：`remani.launch.py`

主要参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `use_sim_time` | `false` | 仿真时 true |
| `use_rviz` | `true` | RViz |
| `hardware_write` | `false` | OCS2 输出门 |
| `start_ocs2` | `true` | 当前源码中已声明，但 OCS2 include 未加 `IfCondition`；实际仍会启动 OCS2 |
| `start_remani` | `true` | 是否启动 REMANI |
| `start_bridge` | `true` | 是否启动参考桥 |
| `task_file` | `config/real/task.info` | OCS2 任务 |
| `urdf_file` | description URDF | 算法 URDF |
| `ocs2_config` | `""` | OCS2 覆盖 |
| `remani_config` | `config/real/remani.yaml` | REMANI 覆盖 |
| `ekf_config` | `config/real/ekf.yaml` | EKF 覆盖 |
| `lib_folder` | `/tmp/wbmm_ocs2_auto_generated` | OCS2 生成库 |
| `odom_topic` | `/odometry/filtered` | EKF 输出 |
| `map_odom_topic` | `/odometry/filtered_map` | relay 输出 |
| `joint_state_topic` | `/joint_states` | REMANI 输入 |
| `wheel_odom_topic` | `/wheel/odometry` | EKF 输入 |
| `imu_topic` | `/imu/data` | EKF 输入 |
| `scan_topic` | `/scan` | AMCL / SLAM 输入 |
| `static_esdf_file` | `""` | `start_remani:=true` 时必需，且 `frame_id=map` |
| `map_file` | `tracer_jaka_localization/maps/factory_map.yaml` | AMCL 地图 |
| `initial_x` | `0.0` | AMCL 初始位姿 |
| `initial_y` | `0.0` | AMCL 初始位姿 |
| `initial_yaw` | `0.0` | AMCL 初始位姿 |

注意：

- `start_ocs2:=false` 在当前源码中不会阻止 OCS2 启动；需要关闭 OCS2 时应修改该 launch 或直接使用更底层的组件入口。

安全检查：

- `map_file` 必须存在；
- `start_remani:=true` 时 `static_esdf_file` 必须存在；
- ESDF NPZ 必须包含标量 `frame_id`，且值必须为 `map`；
- REMANI 使用 `map` frame，OCS2 继续使用 `odom`，桥通过 TF 转换。

### 7.12 `real_ocs2_ee_hold.launch.py`

实机 + OCS2 末端保持。

命令路径：

```text
external base teleop -> /cmd_vel -> tracer_base_node
OCS2 MRT             -> /arm_controller/commands -> JAKA arm
```

OCS2 的底盘输出被 remap 到：

```text
/ocs2/disabled_base_cmd
```

这样外部遥控拥有底盘，OCS2 只观察底盘运动并补偿机械臂。

主要参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `hardware_write` | `false` | 实机运动门，同时传给硬件和 OCS2 |
| `initial_task_phase` | `0` | `0=Navigation, 1=Transition, 2=Execution, 3=Retract` |
| `use_target` | `true` | 启动 RViz 交互末端目标 |
| `use_rviz` | `true` | RViz |
| `lib_folder` | `/tmp/wbmm_ocs2_real_ee_hold` | OCS2 生成库 |
| `base_config_file` | `config/common/ocs2.yaml` | OCS2 公共配置 |
| `ocs2_config` | `""` | OCS2 覆盖 |
| `task_file` | `config/real/task.info` | 实机任务 |
| `urdf_file` | description URDF | 算法 URDF |
| `robot_ip` | `10.5.5.100` | JAKA IP |
| `local_ip` | `10.5.5.127` | 本机 IP |
| `can_port` | `can0` | Tracer CAN |
| `serial_port` | `/dev/ttyUSB0` | IMU 串口 |
| `odom_topic` | `/wheel/odometry` | OCS2 状态反馈 |
| `publish_odom_tf` | `true` | tracer_base 发布 `odom -> base_footprint` |
| `start_base` | `true` | 启动底盘 |
| `start_imu` | `false` | 启动 IMU |
| `start_lidar` | `false` | 启动 LiDAR |
| `start_jaka_fts` | `false` | 启动 F/T broadcaster |
| `lidar_host_ip` | `0.0.0.0` | 雷达 host IP |
| `lidar_sensor_ip` | `192.168.198.2` | 雷达 sensor IP |

### 7.13 `mujoco_ocs2_ee_hold.launch.py`

MuJoCo + OCS2 末端保持。

命令路径：

```text
teleop_twist_keyboard -> /cmd_vel -> mujoco_bridge (base)
OCS2 MRT              -> /arm_controller/commands -> mujoco_bridge (arm)
```

OCS2 的 `/base_controller/cmd_vel` 被 remap 到：

```text
/ocs2/disabled_base_cmd
```

主要参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `scene` | `empty` | MuJoCo 场景 |
| `viewer` | `true` | viewer |
| `initial_pose` | `low` | 初始 keyframe |
| `use_target` | `true` | 启动交互目标节点 |
| `use_rviz` | `true` | RViz |
| `initial_task_phase` | `2` | 仿真默认 Execution |
| `command_output_enabled` | `true` | MRT 输出总门 |
| `lib_folder` | `/tmp/wbmm_ocs2_mujoco_ee_hold` | OCS2 生成库 |

注意：

- 该 launch 是仿真专用；
- MuJoCo 桥发布 `/wheel/odometry`，不启动 EKF；
- OCS2 观察底盘运动并输出机械臂命令。

### 7.14 力控实验 / 测试 Launch

#### `force_control_mujoco_test.launch.py`

启动：

- MuJoCo 硬件
- `whole_body_force_control_profiles.launch.py profile:=sensor_z`
- `whole_body_force_control_test.py` 假力测试器

主要参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `viewer` | `false` | MuJoCo viewer |
| `use_rviz` | `false` | RViz |
| `run_test` | `true` | 是否运行自动测试 |
| `low_force` | `5.0` | 低力阶段 |
| `high_force` | `12.0` | 高力阶段 |
| `pull_force` | `-8.0` | 反向拉力 |
| `baseline_duration` | `4.0` | 基线时长 |
| `low_force_duration` | `10.0` | 低力时长 |
| `high_force_duration` | `15.0` | 高力时长 |
| `release_duration` | `10.0` | 释放时长 |
| `pull_duration` | `12.0` | 拉力时长 |
| `final_release_duration` | `10.0` | 最终释放时长 |
| `report_file` | `/tmp/whole_body_force_control_test_report.json` | 报告文件 |

#### `force_control_20s_follow_test.launch.py`

启动：

- MuJoCo `scene:=force_follow_5m`
- `whole_body_force_control_profiles.launch.py profile:=20s`
- 20 秒持续力跟随测试器

参数：

| 参数 | 默认值 |
|---|---|
| `viewer` | `true` |
| `use_rviz` | `true` |
| `force` | `7.0` |
| `duration` | `20.0` |
| `report_file` | `/tmp/whole_body_force_control_20s_report.json` |

#### `force_control_infinite_follow_test.launch.py`

启动：

- MuJoCo `scene:=force_follow_infinite`
- `whole_body_force_control_profiles.launch.py profile:=infinite`
- 持续无限力跟随测试器

参数：

| 参数 | 默认值 |
|---|---|
| `viewer` | `true` |
| `use_rviz` | `true` |
| `force` | `7.0` |
| `duration` | `30.0` |
| `report_file` | `/tmp/whole_body_force_control_infinite_report.json` |

### 7.15 相机与录制 Launch

#### `d435_camera.launch.py`

臂端 RealSense D435，相机 frame 为 `d435i_link`。

参数：

| 参数 | 默认值 |
|---|---|
| `serial_no` | `''` |
| `camera_namespace` | `camera` |
| `camera_name` | `d435i` |
| `ros_domain_id` | `20` |
| `rmw_implementation` | `rmw_fastrtps_cpp` |

#### `d455_camera.launch.py`

底盘 RealSense D455，相机 frame 为 `d455_link`。

参数同 D435，但 `camera_name` 默认 `d455`。

#### `record_d455_esdf_bag.launch.py`

录制 D455 RGB-D、TF、定位、关节、雷达和 IMU 数据，用于离线 ESDF 建图。

参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `output` | `d455_esdf_bag` | 输出目录；每次录制使用新名字 |

该 launch 固定：

- `ROS_DOMAIN_ID=20`
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- `ROS_LOCALHOST_ONLY=0`

使用示例：

```bash
ros2 launch tracer_jaka_bringup record_d455_esdf_bag.launch.py \
  output:=/home/a/WBMM/bags/d455_esdf_$(date +%Y%m%d_%H%M%S)
```

---

## 8. 实机启动方式

### 8.1 前置条件

1. 已编译并 source 工作空间：

```bash
cd ~/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash
```

2. 已配置并启动硬件：

```bash
./deploy/start.sh
```

`deploy/start.sh` 只负责 CAN、JAKA 登录和初始化校验，不会启动完整 WBMM 栈。

3. 确认：

- Tracer CAN 接口 `can0` 为 `UP`；
- JAKA `jaka_login` 成功；
- 机器人周围安全，急停可用；
- 所有 `hardware_write` 先保持 `false`。

> **重要：`hardware_write` 不是跨 launch 的全局状态。**  
> 当硬件后端与算法入口分开启动时：
> - 硬件后端的 `hardware_write` 控制 JAKA 是否允许写入；
> - 算法入口的 `hardware_write` 控制 OCS2 MRT / MoveIt 是否发布命令；
> - 要真正运动，两个入口都必须显式为 `true`。
>
> 推荐流程：两者先都用 `false` 启动并完成检查，确认安全后同时重启为 `true`；或者一开始就在两个 launch 中都传 `true`。

### 8.2 最小只读硬件启动

```bash
ros2 launch tracer_jaka_bringup wbmm_hardware_interface.launch.py \
  hardware_write:=false \
  can_port:=can0 \
  jaka_robot_ip:=10.5.5.100 \
  jaka_local_ip:=10.5.5.127 \
  start_arm_controller:=false \
  start_jaka_fts:=true \
  publish_odom_tf:=false
```

检查：

```bash
ros2 topic hz /joint_states
ros2 topic hz /wheel/odometry
ros2 topic hz /fts_broadcaster/wrench
ros2 topic echo /joint_states --once
ros2 run tf2_tools view_frames
```

说明：上面使用 `publish_odom_tf:=false`，因为本流程没有启动 EKF；如果只是检查完整 TF 树，可以临时设为 `true`，但不要同时运行另一个 `odom -> base_footprint` 发布者。

### 8.3 实机 OCS2 末端保持

```bash
# 终端 1：硬件
ros2 launch tracer_jaka_bringup wbmm_hardware_interface.launch.py \
  hardware_write:=false \
  start_arm_controller:=true \
  arm_controller_name:=arm_controller \
  publish_odom_tf:=true

# 终端 2：OCS2
ros2 launch tracer_jaka_bringup ocs2.launch.py \
  task_file:=$(ros2 pkg prefix tracer_jaka_bringup)/share/tracer_jaka_bringup/config/real/task.info \
  urdf_file:=$(ros2 pkg prefix tracer_jaka_description)/share/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf \
  lib_folder:=/tmp/wbmm_ocs2_real_ee_hold \
  use_target:=true \
  use_rviz:=true \
  initial_task_phase:=2 \
  command_output_enabled:=false \
  odom_topic:=/wheel/odometry
```

或者使用组合入口：

```bash
ros2 launch tracer_jaka_bringup real_ocs2_ee_hold.launch.py \
  hardware_write:=false \
  use_target:=true \
  use_rviz:=true \
  initial_task_phase:=0
```

确认无误后再将 `hardware_write` 改为 `true`：

```bash
ros2 launch tracer_jaka_bringup real_ocs2_ee_hold.launch.py \
  hardware_write:=true \
  use_target:=true \
  use_rviz:=true \
  initial_task_phase:=0
```

外部底盘遥控：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

注意：OCS2 的底盘输出被 remap 到 `/ocs2/disabled_base_cmd`，`/cmd_vel` 归外部遥控。

### 8.4 实机 REMANI + OCS2（在线定位）

硬件和算法分开启动：

```bash
# 终端 1：硬件，先只读
ros2 launch tracer_jaka_bringup wbmm_hardware_interface.launch.py \
  hardware_write:=false \
  start_arm_controller:=true \
  arm_controller_name:=arm_controller \
  publish_odom_tf:=false

# 终端 2：EKF + SLAM + OCS2 + REMANI
ros2 launch tracer_jaka_bringup remani_mpc.launch.py \
  use_sim_time:=false \
  static_esdf_file:=/absolute/path/to/site.npz \
  hardware_write:=false \
  start_slam:=true \
  odom_topic:=/odometry/filtered
```

该入口要求：

- `static_esdf_file` 存在；
- 如果使用 SLAM，`map -> odom` 由 `slam_toolbox` 发布；
- `odom -> base_footprint` 由 EKF 发布；
- 硬件后端的 `publish_odom_tf` 应为 `false`。

确认规划与 TF 正常后，需要**同时**把硬件后端和算法入口都改为 `hardware_write:=true`。先停止上面两个终端，再启动：

```bash
# 终端 1：硬件，允许 JAKA 写入
ros2 launch tracer_jaka_bringup wbmm_hardware_interface.launch.py \
  hardware_write:=true \
  start_arm_controller:=true \
  arm_controller_name:=arm_controller \
  publish_odom_tf:=false

# 终端 2：算法，允许 OCS2 MRT 输出
ros2 launch tracer_jaka_bringup remani_mpc.launch.py \
  use_sim_time:=false \
  static_esdf_file:=/absolute/path/to/site.npz \
  hardware_write:=true \
  start_slam:=true \
  odom_topic:=/odometry/filtered
```

### 8.5 实机保存地图定位 + REMANI + OCS2

适用于已有 `map` 的现场。

```bash
# 终端 1：硬件，先只读
ros2 launch tracer_jaka_bringup wbmm_hardware_interface.launch.py \
  hardware_write:=false \
  start_arm_controller:=true \
  arm_controller_name:=arm_controller \
  publish_odom_tf:=false

# 终端 2：EKF + AMCL + OCS2 + REMANI
ros2 launch tracer_jaka_bringup remani_mpc_localized.launch.py \
  map_file:=/absolute/path/to/factory_map.yaml \
  static_esdf_file:=/absolute/path/to/site_map_frame.npz \
  initial_x:=0.0 \
  initial_y:=0.0 \
  initial_yaw:=0.0 \
  hardware_write:=false
```

安全检查：

- `map_file` 必须存在；
- `static_esdf_file` 必须存在；
- ESDF NPZ 中 `frame_id` 必须为 `map`；
- 初始位姿必须与现场实际位置一致。

确认 AMCL 收敛、TF 正常、规划路径安全后，同样需要**同时**把硬件后端和算法入口改为 `hardware_write:=true`：

```bash
# 终端 1：硬件
ros2 launch tracer_jaka_bringup wbmm_hardware_interface.launch.py \
  hardware_write:=true \
  start_arm_controller:=true \
  arm_controller_name:=arm_controller \
  publish_odom_tf:=false

# 终端 2：算法
ros2 launch tracer_jaka_bringup remani_mpc_localized.launch.py \
  map_file:=/absolute/path/to/factory_map.yaml \
  static_esdf_file:=/absolute/path/to/site_map_frame.npz \
  initial_x:=0.0 \
  initial_y:=0.0 \
  initial_yaw:=0.0 \
  hardware_write:=true
```

### 8.6 实机力控

先 shadow 模式：

```bash
# 终端 1：硬件
# 本流程未启动 EKF，因此由 tracer_base 发布 odom -> base_footprint。
# 如果另起 EKF，则把 publish_odom_tf 改为 false，避免重复发布 TF。
ros2 launch tracer_jaka_bringup wbmm_hardware_interface.launch.py \
  hardware_write:=false \
  start_arm_controller:=true \
  arm_controller_name:=arm_controller \
  publish_odom_tf:=true

# 终端 2：力控 shadow
# 本终端未启动 EKF，因此 OCS2 直接使用 /wheel/odometry。
# 如果另起 EKF，则把 odom_topic 改为 /odometry/filtered。
ros2 launch tracer_jaka_bringup whole_body_force_control.launch.py \
  hardware_write:=false \
  admittance.enable:=false \
  admittance.output:=false \
  initial_task_phase:=2 \
  odom_topic:=/wheel/odometry
```

检查：

```bash
ros2 topic hz /fts_broadcaster/wrench
ros2 topic hz /whole_body_force_control/processed_wrench
ros2 topic echo /whole_body_force_control/force_sensor_states
ros2 topic echo /whole_body_force_control/states
```

确认 F/T 零点、负载补偿、frame、TF、限幅均正常后，再由现场负责人决定是否打开。**硬件和力控算法两个终端都必须以 `hardware_write:=true` 重启**：

```bash
# 终端 1：硬件，允许 JAKA 写入
ros2 launch tracer_jaka_bringup wbmm_hardware_interface.launch.py \
  hardware_write:=true \
  start_arm_controller:=true \
  arm_controller_name:=arm_controller \
  publish_odom_tf:=true

# 终端 2：力控，允许导纳和参考输出
ros2 launch tracer_jaka_bringup whole_body_force_control.launch.py \
  hardware_write:=true \
  admittance.enable:=true \
  admittance.output:=true \
  initial_task_phase:=2 \
  odom_topic:=/wheel/odometry
```

力控节点故障恢复：

```bash
ros2 service call /whole_body_force_control/force_sensor/reset std_srvs/srv/Trigger "{}"
ros2 service call /whole_body_force_control/reset std_srvs/srv/Trigger "{}"
```

### 8.7 实机 MoveIt

```bash
# 终端 1：硬件，spawn arm_trajectory_controller
ros2 launch tracer_jaka_bringup wbmm_hardware_interface.launch.py \
  hardware_write:=false \
  start_arm_controller:=true \
  arm_controller_name:=arm_trajectory_controller \
  publish_odom_tf:=false

# 终端 2：MoveIt
ros2 launch tracer_jaka_bringup moveit.launch.py \
  use_sim_time:=false \
  use_rviz:=true \
  hardware_write:=false
```

确认规划路径安全后，需要**同时**把硬件后端和 MoveIt 入口改为 `hardware_write:=true`：

```bash
# 终端 1：硬件
ros2 launch tracer_jaka_bringup wbmm_hardware_interface.launch.py \
  hardware_write:=true \
  start_arm_controller:=true \
  arm_controller_name:=arm_trajectory_controller \
  publish_odom_tf:=false

# 终端 2：MoveIt
ros2 launch tracer_jaka_bringup moveit.launch.py \
  use_sim_time:=false \
  use_rviz:=true \
  hardware_write:=true
```

### 8.8 实机相机与数据录制

```bash
# 臂端 D435
ros2 launch tracer_jaka_bringup d435_camera.launch.py serial_no:=<serial>

# 底盘 D455
ros2 launch tracer_jaka_bringup d455_camera.launch.py serial_no:=<serial>

# 录制 ESDF 数据
ros2 launch tracer_jaka_bringup record_d455_esdf_bag.launch.py \
  output:=/home/a/WBMM/bags/d455_esdf_run1
```

### 8.9 运行时只读审计

```bash
ros2 run tracer_jaka_bringup readiness_check.py --ros-args \
  -p audit_duration:=5.0 \
  -p command_output_enabled:=false \
  -p mpc_target_topic:=/mobile_manipulator_mpc_target \
  -p base_command_topic:=/cmd_vel \
  -p arm_command_topic:=/arm_controller/commands
```

该脚本检查：

- 目标话题发布者数量；
- 干跑时 `/cmd_vel` 和 `/arm_controller/commands` 是否没有发布者；
- 是否存在重复的 TF child owner。

注意：`readiness_check.py` 的默认 `mpc_target_topic` 是 `/mobile_manipulator_mpc_target`；当前 OCS2 主要目标话题为：

```text
/mobile_manipulator_whole_body_target   # 9D 全身参考
/mobile_manipulator_ee_target           # 7D 末端参考
```

使用时应按当前目标所有者，把 `mpc_target_topic` 改成实际期望只有 1 个发布者的话题。

---

## 9. 仿真启动方式

### 9.1 MuJoCo 硬件接口

```bash
ros2 launch tracer_jaka_bringup mujoco_hardware_interface.launch.py \
  scene:=empty \
  viewer:=true \
  start_camera:=false \
  publish_odom_tf:=false
```

可选场景：

```bash
ros2 launch tracer_jaka_bringup mujoco_hardware_interface.launch.py scene:=room
ros2 launch tracer_jaka_bringup mujoco_hardware_interface.launch.py scene:=task_table
ros2 launch tracer_jaka_bringup mujoco_hardware_interface.launch.py scene:=force_follow_infinite
ros2 launch tracer_jaka_bringup mujoco_hardware_interface.launch.py scene:=force_follow_5m
ros2 launch tracer_jaka_bringup mujoco_hardware_interface.launch.py scene:=nvblox_remani_demo
ros2 launch tracer_jaka_bringup mujoco_hardware_interface.launch.py scene:=esdf_validation
```

手动控制：

```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear:{x:0.2}, angular:{z:0.3}}"

ros2 topic pub /arm_controller/commands std_msgs/Float64MultiArray \
  "{data:[0,0.5,1.0,0,0.5,0]}"
```

### 9.2 MuJoCo + OCS2 末端保持

```bash
ros2 launch tracer_jaka_bringup mujoco_ocs2_ee_hold.launch.py \
  scene:=empty \
  viewer:=true \
  initial_task_phase:=2 \
  use_target:=true \
  use_rviz:=true \
  command_output_enabled:=true
```

另开终端遥控底盘：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 9.3 仿真力控 Profile

方式一：分别启动 MuJoCo 和力控。

```bash
# 终端 1
ros2 launch tracer_jaka_bringup mujoco_hardware_interface.launch.py \
  scene:=force_follow_infinite \
  init_keyframe:=low \
  start_camera:=false \
  publish_odom_tf:=true

# 终端 2
ros2 launch tracer_jaka_bringup whole_body_force_control_profiles.launch.py \
  profile:=sensor_z \
  use_rviz:=true
```

方式二：使用自动测试入口。

```bash
ros2 launch tracer_jaka_bringup force_control_mujoco_test.launch.py \
  viewer:=true \
  use_rviz:=true
```

20 秒持续力跟随：

```bash
ros2 launch tracer_jaka_bringup force_control_20s_follow_test.launch.py \
  viewer:=true \
  use_rviz:=true \
  force:=7.0 \
  duration:=20.0
```

无限场景持续力跟随：

```bash
ros2 launch tracer_jaka_bringup force_control_infinite_follow_test.launch.py \
  viewer:=true \
  use_rviz:=true \
  force:=7.0 \
  duration:=30.0
```

六轴序列：

```bash
ros2 launch tracer_jaka_bringup whole_body_force_control_profiles.launch.py \
  profile:=six_axis_sequence \
  use_rviz:=false

ros2 run whole_body_force_control axis_wrench_sequence.py
```

### 9.4 仿真一键组合

启动 MuJoCo + EKF + SLAM + OCS2：

```bash
ros2 launch tracer_jaka_bringup wbmm.launch.py \
  hardware_backend:=mujoco \
  config_profile:=sim \
  scene:=room \
  start_localization:=true \
  start_slam:=true \
  start_ocs2:=true \
  odom_topic:=/odometry/filtered \
  use_rviz:=true
```

启动 MuJoCo + 定位 + REMANI + OCS2：

```bash
ros2 launch tracer_jaka_bringup wbmm.launch.py \
  hardware_backend:=mujoco \
  config_profile:=sim \
  scene:=nvblox_remani_demo \
  start_localization:=true \
  start_slam:=true \
  start_remani:=true \
  static_esdf_file:=/absolute/path/to/sim_esdf.npz \
  odom_topic:=/odometry/filtered \
  use_rviz:=true
```

注意：

- `wbmm.launch.py` 把 `hardware_write` 同时传给 OCS2 的 `command_output_enabled`；仿真中需要 OCS2 真正输出时要加 `hardware_write:=true`，只观察/规划时保持 `false`；
- `start_remani:=true` 时 `static_esdf_file` 必需；
- 需要 EKF 时 `publish_odom_tf:=false`；
- `start_force_control:=true` 已包含 OCS2，不要再同时设置 `start_ocs2:=true`。

关于 `wbmm.launch.py start_force_control:=true`：

- 它会包含 `whole_body_force_control.launch.py`；
- 但顶层入口当前不转发 `admittance.enable` / `admittance.output`；
- `whole_body_force_control.launch.py` 的默认值是两者都为 `false`，会覆盖 `config/sim/force_control.yaml` 中的 `true`；
- 因此**仿真力控 profile 建议直接使用**：

```bash
ros2 launch tracer_jaka_bringup mujoco_hardware_interface.launch.py \
  scene:=force_follow_infinite \
  publish_odom_tf:=true

ros2 launch tracer_jaka_bringup whole_body_force_control_profiles.launch.py \
  profile:=sensor_z \
  use_rviz:=true
```

或者使用自动测试入口 `force_control_mujoco_test.launch.py`。

### 9.5 仿真 MoveIt

```bash
ros2 launch tracer_jaka_bringup wbmm.launch.py \
  hardware_backend:=mujoco \
  config_profile:=sim \
  scene:=empty \
  start_moveit:=true \
  hardware_write:=true \
  use_rviz:=true
```

或分别启动：

```bash
ros2 launch tracer_jaka_bringup mujoco_hardware_interface.launch.py \
  scene:=empty \
  viewer:=true

ros2 launch tracer_jaka_bringup moveit.launch.py \
  use_sim_time:=true \
  use_rviz:=true \
  hardware_write:=true
```

---

## 10. 安全与运行注意事项

1. **`hardware_write` 是实机运动唯一总门**  
   默认 `false`。`true` 才允许 JAKA 写入和 OCS2/MoveIt 命令输出。

2. **不要同时启动多个 `map -> odom` 或 `odom -> base_footprint` 发布者**  
   - SLAM 模式：`slam_toolbox` 发布 `map -> odom`；
   - 保存地图模式：AMCL 发布 `map -> odom`；
   - EKF 模式：`robot_localization` 发布 `odom -> base_footprint`；
   - 硬件后端的 `publish_odom_tf` 在 EKF 启动时应为 `false`。

3. **REMANI 必须使用明确的静态 ESDF**
   - `static_esdf_file` 必须存在；
   - 在 `remani_mpc_localized` 中必须为 `frame_id=map`；
   - 不能只改 `frame_id` 而不改 ESDF 原点和几何。

4. **力控默认 shadow**
   - 实机 `config/real/force_control.yaml` 显式关闭 `admittance.enable` 和 `admittance.output`；
   - 打开 `admittance.output` 必须同时 `hardware_write:=true`；
   - 故障后按流程 reset，不要盲目重新使能。

5. **OCS2 `lib_folder` 不要并发混用**
   - MPC 和 MRT 分别使用 `<lib_folder>/mpc` 和 `<lib_folder>/mrt`；
   - 修改 `task.info` 中影响动力学的参数后，建议更换新目录或清理旧生成库。

6. **MoveIt 执行受 `hardware_write` 控制**
   - 仿真中同样默认 `false`，需要显式打开才会执行轨迹。

7. **`task.info` 中的碰撞 link 必须真实存在**
   - 修改 URDF 后要同步检查 `selfCollision.collisionLinkPairs` 和 `removeJoints`。

8. **不要让多个机械臂命令所有者同时运行**
   - OCS2/力控输出 `/arm_controller/commands`；
   - MoveIt 输出 `/arm_trajectory_controller/follow_joint_trajectory`；
   - 同时启动并打开多个所有者可能导致命令竞争；除非明确设计了仲裁，否则只保留一个机械臂命令所有者。

---

## 11. 辅助脚本

### 11.1 `arm_pose_publisher.py`

发布固定 arm-up 关节状态，便于没有真实关节状态时补全 TF 树。

```bash
ros2 run tracer_jaka_bringup arm_pose_publisher.py
```

通常通过 `wbmm_hardware_interface.launch.py start_arm_pose:=true` 启动。

### 11.2 `odom_to_map_relay.py`

把 `/odometry/filtered` 的 pose 从 `odom` 转到 `map`，输出 `/odometry/filtered_map`，twist 保持 body frame。

```bash
ros2 run tracer_jaka_bringup odom_to_map_relay.py --ros-args \
  -p odom_topic:=/odometry/filtered \
  -p output_topic:=/odometry/filtered_map \
  -p map_frame:=map \
  -p odom_frame:=odom \
  -p child_frame:=base_footprint
```

### 11.3 `readiness_check.py`

只读运行时审计：

```bash
ros2 run tracer_jaka_bringup readiness_check.py --ros-args \
  -p audit_duration:=5.0 \
  -p command_output_enabled:=false
```

---

## 12. 测试

运行 bringup 包测试：

```bash
colcon test --packages-select tracer_jaka_bringup
colcon test-result --verbose
```

当前测试覆盖：

- 核心算法 launch 不启动硬件后端；
- `wbmm.launch.py` 默认 fail-closed；
- REMANI 跨 frame 必须使用 TF；
- MoveIt 只使用 `hardware_write` 作为运动门；
- 实机 ros2_control 控制器名和配置路径；
- 力控 common/real/sim 配置分层和 profile 参数；
- 实机力控输出门；
- localized 实机 ESDF `frame_id=map` 安全门；
- launch 组合静态解析。

---

## 13. 常见问题

### 13.1 `wbmm.launch.py` 启动后什么都没有

这是设计行为。默认：

```text
hardware_backend:=none
start_localization:=false
start_ocs2:=false
start_remani:=false
start_force_control:=false
start_moveit:=false
```

必须显式选择后端和算法。

### 13.2 `start_force_control:=true` 和 `start_ocs2:=true` 同时设置报错

力控 launch 已经包含 OCS2，二者互斥。

### 13.3 `admittance.output:=true` 但 `hardware_write:=false` 报错

这是安全门，必须同时打开 `hardware_write`。

### 13.4 REMANI 报 `static_esdf_file` 不存在

`remani_mpc.launch.py` 和 `wbmm.launch.py start_remani:=true` 都要求显式存在的 ESDF 文件。请传入绝对路径。

### 13.5 localized 模式报 ESDF frame 错误

`remani_mpc_localized.launch.py` 要求：

- `map_file` 存在；
- `static_esdf_file` 存在；
- ESDF NPZ 中 `frame_id` 为标量且等于 `map`。

### 13.6 TF 树有重复 child

检查是否同时启动了：

- `slam_toolbox` 和 AMCL；
- EKF 和硬件后端的 `publish_odom_tf:=true`；
- 多个 `robot_state_publisher`。

可用 `readiness_check.py` 或：

```bash
ros2 run tf2_tools view_frames
```

### 13.7 力控没有输出

按顺序检查：

1. `/fts_broadcaster/wrench` 是否有数据；
2. `/whole_body_force_control/processed_wrench` 是否有数据；
3. `/whole_body_force_control/force_sensor_states` 是否 fault；
4. `admittance.enable` 和 `admittance.output` 是否为 true；
5. `hardware_write` 是否为 true；
6. OCS2 是否在运行且 `command_output_enabled` 已打开；
7. TF `sensor_frame -> tcp_frame` 是否可用。

---

## 14. 相关文档

- `src/bringup/config/README.md`：配置分层说明
- `docs/frame_contract.md`：TF 和 frame 契约
- `docs/math_contract.md`：状态、输入、轨迹和力控数学契约
- `src/control/whole_body_force_control/README.md`：力控算法包
- `src/control/wbmm_ocs2_ros/README.md`：OCS2 ROS 适配层
- `src/sim/tracer_jaka_mujoco/README.md`：MuJoCo 桥和传感器
- `src/map/tracer_jaka_localization/README.md`：AMCL / 保存地图定位
- `deploy/README.md`：实机部署脚本

---

> 本文档为 bringup 包使用说明，参数最终以源码和实际加载的 YAML 为准。
