# tracer_jaka_bringup

完整系统组合只允许从本包启动。仿真/实机入口总表和包边界见
[`docs/launch_ownership.md`](../../../docs/launch_ownership.md)。真实擦拭统一入口为：

```bash
ros2 launch tracer_jaka_bringup wipe_real_pipeline.launch.py
```

默认保持 `jaka_read_only:=true`、`command_output_enabled:=false`、
`safety_release:=false`、`auto_goal:=false` 和
`force_control_enabled:=false`。任何实机命令输出还必须显式设置第三道
`safety_release:=true`；组合不一致时 launch 会在创建硬件/控制节点前拒绝启动。

## MoveIt：统一选择仿真或真机

MoveIt、MoveIt Servo、RViz、controller manager 和仿真/实机接口均由本包统一编排。
`jaka_driver` 只保留驱动节点和可复用的手柄到 Servo 组件，不再安装系统启动文件。

默认使用 MuJoCo 仿真，并允许 MoveIt 执行规划轨迹：

```bash
ros2 launch tracer_jaka_bringup moveit.launch.py backend:=sim
```

仿真分支直接复用 MuJoCo bridge 提供的标准接口：

- 状态：`/joint_states`
- MoveIt 轨迹：`/arm_trajectory_controller/follow_joint_trajectory`
- Servo 位置命令：`/arm_controller/commands`
- F/T：`/fts_broadcaster/wrench`

真机首次只读验证：

```bash
ros2 launch tracer_jaka_bringup moveit.launch.py \
  backend:=real \
  jaka_read_only:=true
```

此时 `allow_trajectory_execution:=auto` 会解析为 `false`，只能规划，不能由 MoveIt
执行。确认网络、关节反馈、控制器和急停均正常后，真机运动必须同时显式设置：

```bash
ros2 launch tracer_jaka_bringup moveit.launch.py \
  backend:=real \
  jaka_read_only:=false \
  allow_trajectory_execution:=true
```

启用 MoveIt Servo 和手柄：

```bash
ros2 launch tracer_jaka_bringup moveit.launch.py \
  backend:=sim \
  use_servo:=true \
  use_joy:=true
```

启用 Servo 后，`allow_trajectory_execution:=auto` 会解析为 `false`，MoveGroup 仍可
规划但不能同时执行轨迹，确保 Servo 是唯一机械臂命令源。

真机 Servo 仍受 `jaka_read_only` 保护。`use_servo:=true` 时，真机加载
`arm_controller`；否则加载 `arm_trajectory_controller`，避免两个控制器争用同一组
关节命令接口。`moveit_real.launch.py` 和 `servo.launch.py` 已移动到本包并作为兼容入口
保留，新脚本应直接使用 `moveit.launch.py`。

WBMM 的顶层系统组合包。本文以
`remani_mpc_localized_real.launch.py` 为主，说明如何在真实 Tracer + JAKA
平台上使用保存的 2D 地图进行 AMCL 定位、使用同一现场的 3D ESDF 进行 REMANI
规划，并由 OCS2 MPC/MRT 执行全身轨迹。

> **实机首选入口**：`remani_mpc_localized_real.launch.py`。
> `remani_mpc_real.launch.py` 是边运行边用 `slam_toolbox` 建图的旧流程；已有固定场地
> 地图时不要混用这两个入口，也不要同时启动 AMCL 和 `slam_toolbox` 来发布
> `map -> odom`。

## 0. 当前实机进度摘要（部署记录）

> 最近一次现场进度：定位链已通过，MRT 重名已解决，正在分阶段验证 REMANI/OCS2 执行与机械臂动作。

### 已完成

- D0 定位链通过：
  - `/odometry/filtered_map` 正常，坐标系为 `map`；
  - 当前位姿约 `(2.995, 2.237, -1.534 rad)`；
  - 速度接近零；
  - JAKA 状态、F/T、命令安全闸通过。
- MRT 重名问题已修复：
  - `ocs2_real.launch.py` 和 `ocs2_sim.launch.py` 不再给 MRT 显式 `name='tracer_jaka_mrt_node'`，使用 C++ 节点自身名称；
  - `TracerJakaMrtNode.cpp` 中 OCS2 内部节点使用 `use_global_arguments(false)`，避免被 launch 层 `__node` 重命名成同一个主节点；
  - 期望 ROS 图只有：
    ```text
    /tracer_jaka_mrt_node
    /tracer_jaka_mrt_node_ocs2_internal
    ```

### 当前遇到的问题与对策

- REMANI 反复重规划：
  - 日志常见：
    ```text
    max right wheel omega is not feasible
    ```
  - 典型超限值约 `1.05 ~ 1.16 rad/s`，超过默认 `1.0 rad/s`（含 5% 容差后约 `1.05`）。
  - 原因：目标距离偏长或带转弯时，优化后轨迹的右轮转速略微超限。
  - 对策：
    - 先发正前方 `0.20 m` 小目标；
    - 必要时放宽：
      ```text
      mobile_base_max_wheel_omega:=1.3
      mobile_base_max_wheel_alpha:=3.0
      ```
    - 若仍不足，可继续放宽到：
      ```text
      mobile_base_max_wheel_omega:=1.5
      mobile_base_max_wheel_alpha:=3.5
      ```
- 机械臂测试时若 `arm_max_delta_per_step:=0.01`：
  - 会报：
    ```text
    [SAFETY] Arm joint ... command jump too large
    ```
  - 原因：MPC 预测命令单步变化超过 `0.01 rad`。
  - 对策：机械臂测试使用：
    ```text
    arm_max_delta_per_step:=0.05
    arm_max_command_velocity:=0.10
    ```
- 完整 base + arm 目标：
  - 话题：`/remani_planner/whole_body_goal`
  - 类型：`traj_utils/msg/WholeBodyGoal`
  - 字段：`header`、`base_pose`、`joint_names`、`joint_positions`
- 开局重定位：
  - 在 launch 命令中传 `initial_x/y/yaw`；
  - AMCL 已开启 `set_initial_pose:=true`；
  - 也可用 RViz `2D Pose Estimate` 或调用：
    ```bash
    ros2 service call /reinitialize_global_localization std_srvs/srv/Empty "{}"
    ```


### 机械臂 + 底盘联合测试指令（1 m / 2 m）

以下假设当前位姿仍为：

```text
x   = 2.995
y   = 2.237
yaw = -1.534 rad
```

如果实际位姿变化，请用公式重算：

```text
前方 d m：
x_front = x + d * cos(yaw)
y_front = y + d * sin(yaw)

后方 d m：
x_back  = x - d * cos(yaw)
y_back  = y - d * sin(yaw)

四元数：
z = sin(yaw / 2)
w = cos(yaw / 2)
```

#### 启动命令（机械臂测试推荐保守参数）

```bash
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

#### 1 m 前方，机械臂 `pose_00`

```bash
ros2 topic pub --once /remani_planner/whole_body_goal traj_utils/msg/WholeBodyGoal "{
  header: {
    frame_id: 'map'
  },
  base_pose: {
    position: {
      x: 3.032,
      y: 1.238,
      z: 0.0
    },
    orientation: {
      x: 0.0,
      y: 0.0,
      z: -0.694,
      w: 0.720
    }
  },
  joint_names: [
    'joint_1',
    'joint_2',
    'joint_3',
    'joint_4',
    'joint_5',
    'joint_6'
  ],
  joint_positions: [
    -0.515,
    1.5707,
    -1.5707,
    1.5707,
    1.5707,
    0.254
  ]
}"
```

#### 1 m 后方，机械臂 `up`

```bash
ros2 topic pub --once /remani_planner/whole_body_goal traj_utils/msg/WholeBodyGoal "{
  header: {
    frame_id: 'map'
  },
  base_pose: {
    position: {
      x: 2.958,
      y: 3.236,
      z: 0.0
    },
    orientation: {
      x: 0.0,
      y: 0.0,
      z: -0.694,
      w: 0.720
    }
  },
  joint_names: [
    'joint_1',
    'joint_2',
    'joint_3',
    'joint_4',
    'joint_5',
    'joint_6'
  ],
  joint_positions: [
    0.0,
    1.5707,
    0.0,
    1.5707,
    3.14159,
    0.785398
  ]
}"
```

#### 2 m 前方，机械臂 `pose_00`

```bash
ros2 topic pub --once /remani_planner/whole_body_goal traj_utils/msg/WholeBodyGoal "{
  header: {
    frame_id: 'map'
  },
  base_pose: {
    position: {
      x: 3.069,
      y: 0.238,
      z: 0.0
    },
    orientation: {
      x: 0.0,
      y: 0.0,
      z: -0.694,
      w: 0.720
    }
  },
  joint_names: [
    'joint_1',
    'joint_2',
    'joint_3',
    'joint_4',
    'joint_5',
    'joint_6'
  ],
  joint_positions: [
    -0.515,
    1.5707,
    -1.5707,
    1.5707,
    1.5707,
    0.254
  ]
}"
```

#### 2 m 后方，机械臂 `up`

```bash
ros2 topic pub --once /remani_planner/whole_body_goal traj_utils/msg/WholeBodyGoal "{
  header: {
    frame_id: 'map'
  },
  base_pose: {
    position: {
      x: 2.921,
      y: 4.236,
      z: 0.0
    },
    orientation: {
      x: 0.0,
      y: 0.0,
      z: -0.694,
      w: 0.720
    }
  },
  joint_names: [
    'joint_1',
    'joint_2',
    'joint_3',
    'joint_4',
    'joint_5',
    'joint_6'
  ],
  joint_positions: [
    0.0,
    1.5707,
    0.0,
    1.5707,
    3.14159,
    0.785398
  ]
}"
```

> 注意：一次只发一个目标，等上一条轨迹执行完成、机器人停稳后再发下一个。  
> 如果仍出现 `max right wheel omega is not feasible`，可继续放宽到：
>
> ```text
> mobile_base_max_wheel_omega:=1.5
> mobile_base_max_wheel_alpha:=3.5
> ```


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
│   ├── fts_broadcaster                -> /fts_broadcaster/wrench
│   ├── arm_controller
│   └── OCS2 MPC/MRT（默认 dry-run）        -> 仅在双安全开关打开后发布控制命令
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
  jaka_driver \
  jaka_hardware_interface \
  --cmake-args \
    -DBUILD_TESTING=OFF \
    -DBUILD_JAKA_JOY_TO_SERVO=OFF

source /home/a/WBMM/install/setup.bash
```

这里关闭的是测试目标，不影响实机运行功能。当前
`jaka_hardware_interface` 在启用 `BUILD_TESTING` 时还会查找
`ros2_control_test_assets`；仅部署实机时关闭测试可以避免因该测试依赖缺失而中断构建。
`BUILD_JAKA_JOY_TO_SERVO=OFF` 只关闭本次实机链路不用的 MoveIt Servo/夹爪摇杆组件，
不会关闭 `jaka_login`、`jaka_logout`、`jaka_edg_node` 或 JAKA SDK。

每个新终端都要 source ROS 和工作空间。可先确认入口已安装：

```bash
ros2 pkg prefix tracer_jaka_bringup
ros2 pkg executables jaka_driver
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

本机当前对应网卡连接名为 `Wired connection 1`（设备 `enp89s0`）。若检查发现工控机
误用了机器人的 `.100` 地址，可改为 `.127`；专用直连网卡不设置网关，避免抢占系统
默认路由：

```bash
sudo nmcli connection modify "Wired connection 1" \
  ipv4.method manual ipv4.addresses 10.5.5.127/24 \
  ipv4.gateway "" ipv4.never-default yes
sudo nmcli connection up "Wired connection 1"

ip -br address show enp89s0
ip route get 10.5.5.100
ping -c 3 10.5.5.100
```

执行 `nmcli connection up` 会让这张网卡短暂断开；如果正通过该网卡远程维护，先切换到
本地终端或其他管理网络。

`ocs2_real.launch.py` 会把顶层的 `robot_ip`、`local_ip` 注入
统一 ros2_control xacro 的实机后端参数。单独运行 `real_slam.launch.py`
的只读 JAKA 状态链时，对应参数名为 `jaka_robot_ip`、`jaka_local_ip`。

### 2.4 JAKA 通讯初始化（`jaka_login`）

`jaka_login` 是一个执行完即退出的一次性初始化工具。它依次执行：

1. 登录 JAKA 控制柜；
2. 机器人上电，等待 8 秒；
3. 机器人使能，等待 4 秒；
4. 设置关节伺服一阶低通滤波参数为 `2`；
5. 将力矩传感器模式设置为 `1`。

它不会主动下发关节运动目标，但“上电”和“使能”会让机械臂进入可执行状态。运行前必须
清空机械臂工作区、准备物理急停，并确认 NUC 已使用 `10.5.5.127/24`，机器人
`10.5.5.100` 能够 ping 通。

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash

ping -c 3 10.5.5.100
ros2 run jaka_driver jaka_login
```

默认机器人 IP 为 `10.5.5.100`。需要连接其他地址时，把 IP 作为位置参数传入：

```bash
ros2 run jaka_driver jaka_login 10.5.5.100
```

只有看到下面一行且进程以状态码 `0` 退出，才表示全部初始化步骤成功：

```text
JAKA communication initialization completed
```

任一步失败时程序会打印 `SDK error code` 并以非零状态退出。不要同时运行
`jaka_login` 与 `jaka_hardware_interface`、`ocs2_real.launch.py` 或完整实机 launch；
先让 `jaka_login` 正常退出，再启动 ros2_control/OCS2，避免两个进程同时占用 JAKA SDK
连接。

如果只需重新构建这一工具及其工作区依赖：

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build --symlink-install --packages-up-to jaka_driver \
  --cmake-args \
    -DBUILD_TESTING=OFF \
    -DBUILD_JAKA_JOY_TO_SERVO=OFF
source install/setup.bash
ros2 pkg executables jaka_driver
```

### 2.5 CAN、IMU 与 LiDAR

```bash
sudo modprobe gs_usb
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0
candump can0
```

本机的 `gs_usb` 适配器不支持 `restart-ms`，不要在 `ip link set` 命令中添加该参数；
若发生 Bus-Off，手动执行一次 `can0 down`，再用上面的 500 kbit/s 命令重新上线。

`candump` 看到报文后按 `Ctrl-C` 退出。IMU 建议把当前用户永久加入 `dialout` 组：

```bash
sudo usermod -aG dialout "$USER"
# 注销桌面会话后重新登录（仅新开终端不够），然后验证：
id -nG | tr ' ' '\n' | grep -x dialout
ls -l /dev/ttyUSB0
test -r /dev/ttyUSB0 && test -w /dev/ttyUSB0 && echo "TTYUSB permission OK"
ls -l /dev/serial/by-id/
```

如果机器上可能同时连接多个 USB 串口，建议把启动参数 `serial_port` 改成上面
`/dev/serial/by-id/` 中对应 IMU 的稳定路径，避免重插设备后 `ttyUSB0` 编号变化。

只有来不及注销重登的临时调试场景才直接放宽当前设备权限；USB 重插后该权限会失效：

```bash
sudo chmod 666 /dev/ttyUSB0
```

当前顶层可覆盖 `can_port`、`serial_port`、`scan_topic`，也可通过被包含的
`real_slam.launch.py` 参数覆盖 Lakibeam 的主机 IP、传感器 IP、UDP 端口、倒装和
角度偏移。默认值位于
`src/bringup/tracer_jaka_bringup/launch/real_slam.launch.py`：

```text
lidar_host_ip=0.0.0.0
lidar_sensor_ip=192.168.198.2
lidar_port=2368
lidar_inverted=false
lidar_angle_offset=0
```

本机实测的 Richbeam/Lakibeam 是 USB RNDIS 直连设备：主机
`192.168.8.1`、雷达 `192.168.8.2`，与源码默认值不同。启动本机实机链路时应显式覆盖：

```bash
lidar_host_ip:=192.168.8.1 lidar_sensor_ip:=192.168.8.2
```

可在启动前用 `ping -c 3 192.168.8.2` 验证；配置页标题应为
`LiDAR web panel - Richbeam`。现场设备若使用其他网段，以设备配置页和
`ip -br address` 的实际结果为准。

## 3. 推荐的分阶段实机部署

### 阶段 A：断开执行能力，核对安全条件

- 清空机械臂和底盘工作区，首次测试将底盘架空或机械固定；
- 示教器、底盘遥控器和物理急停由专人握持；
- JAKA 切到允许外部控制的正确模式，但先不要发送目标；
- 确认按下物理急停能同时阻止底盘和机械臂运动；`Ctrl-C` 不是急停；
- 首次运行保持 `use_joy:=false`、`tracking_error_replan_enabled:=false`。

### 阶段 B：验证底盘传感器、JAKA 真实状态、EKF 与 AMCL

先执行一次 JAKA 初始化，并等待它正常退出：

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run jaka_driver jaka_login
```

终端 1 启动底盘、传感器、EKF，以及 JAKA **只读** hardware interface；不启 SLAM，
不加载机械臂命令控制器：

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch tracer_jaka_bringup real_slam.launch.py \
  start_slam:=false \
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

这里的 JAKA 硬件参数固定注入 `jaka_read_only:=true`：会登录 EDG、读取真实关节角/速度
和六维力数据，但不会启用 servo mode、不会加载位置控制器，`write()` 也不会向机械臂
发送命令。`joint_state_broadcaster` 发布 `/joint_states`，力传感器广播器发布
`/fts_broadcaster/wrench`。力传感器启动时会采样约 0.5 秒计算零偏，这段时间保持
机械臂和末端负载静止。

终端 2 启动保存地图定位：

```bash
source /opt/ros/humble/setup.bash
source /home/a/WBMM/install/setup.bash

ros2 launch tracer_jaka_localization localization_real.launch.py \
  map_file:=/home/a/WBMM/maps/site_2d.yaml \
  scan_topic:=/scan \
  initial_x:=0.0 \
  initial_y:=0.0 \
  initial_yaw:=0.0 \
  start_esdf_visualization:=true \
  esdf_file:=/home/a/WBMM/maps/site_remani.npz \
  esdf_offset_x:=0.0 \
  esdf_offset_y:=0.0 \
  esdf_offset_z:=0.0
```

若要让后续 3D ESDF/REMANI 直接获得 `map` 坐标系下的里程计，再开终端 3（仍然不会
启动任何控制器）：

```bash
source /opt/ros/humble/setup.bash
source /home/a/WBMM/install/setup.bash

ros2 run tracer_jaka_bringup odom_to_map_relay.py --ros-args \
  -p odom_topic:=/odometry/filtered \
  -p output_topic:=/odometry/filtered_map \
  -p map_frame:=map \
  -p odom_frame:=odom \
  -p child_frame:=base_footprint
```

定位 launch 现在会同时管理 `map_server` 和 `amcl` 的生命周期，两者都应自动进入
`active`，并设置 `set_initial_pose=true`，因此命令行的 `initial_x/y/yaw` 会在启动时
真正用于初始化粒子滤波器。没有 `map -> odom` 时不要继续发送目标。

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
ros2 topic hz /odometry/filtered_map
ros2 topic echo /map --once
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic hz /joint_states
ros2 topic echo /fts_broadcaster/wrench --once
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers "{}"
ros2 node list | grep -E 'amcl|map_server|ekf|slam_toolbox'
```

`map_server`、`amcl` 应为 `active`；控制器列表只应看到
`joint_state_broadcaster` 和 `fts_broadcaster` 为 `active`，不应出现或启动
`arm_controller`/`arm_trajectory_controller`。应看到 AMCL、map_server 和 EKF，且
不应看到 `slam_toolbox`。RViz 的 Fixed Frame 设为 `map`，TF 应连续、无大幅跳动。
验证完先退出这两个 launch，避免下一步重复启动驱动和 TF。

`tf2_echo map base_footprint` 输出的 `(x, y, z)` 与姿态，就是机器人在持久化 `map`
坐标系中的实时位姿。只要 `site_2d.yaml` 与 `site_remani.npz` 来自同一现场且已对齐，
这同时也是机器人在 3D ESDF 中的坐标；AMCL 本身只估计平面 `x/y/yaw`，高度和机械臂
各 link 的 3D 位姿由机器人 TF 树补齐。完整 REMANI 启动入口还会通过
`odom_to_map_relay` 生成 `/odometry/filtered_map`，供规划器直接消费。

上述定位命令还会把保存的 ESDF 发布到 `/esdf_cloud`。`slam.rviz` 已加入
`Saved 3D ESDF` 的 PointCloud2 显示：红色接近/进入障碍，蓝色距离障碍较远；为了避免
显示近两百万个自由空间体素，只显示距离障碍表面 `0.35 m` 内、步长为 2 的 3D 体素。
这是保存地图的只读可视化，不会启动 nvblox，也不会改变 REMANI 使用的 ESDF。三个
`esdf_offset_*` 默认保持 `0.0`，只有完成 2D 地图与 3D ESDF 的固定平移标定后才修改。

### 阶段 C：确认保守的 OCS2 task 文件

顶层参数 `manipulator_max_vel/acc` 只限制 REMANI 生成的机械臂参考轨迹；最终控制输入
还受 OCS2 task 文件约束。仓库已经提供并默认选择下面的实机保守配置，不要再从
`task_real.info` 覆盖它：

```text
/home/a/WBMM/src/bringup/tracer_jaka_bringup/config/task_real_conservative.info
```

该文件当前在 `jointVelocityLimits` 中使用：

```text
wheelBasedMobileManipulator lowerBound: -0.05, -0.20
wheelBasedMobileManipulator upperBound:  0.05,  0.20
arm lowerBound:  6 个 -0.15
arm upperBound:  6 个  0.15
```

单位依次为底盘线速度 m/s、角速度 rad/s、机械臂关节速度 rad/s；机械臂
`inputCost.R` 的 scaling 也已从 `1e-2` 提高到 `5e-2`。修改 task 后要重启 MPC；如果
遇到自动微分库仍复用旧
模型，可换一个新的 `lib_folder` 或清理该任务专用的 `/tmp` 生成目录后重启。

原 `task_real.info` 的 `environmentCollision.obstacles` 还包含一个测试用 `box_1`
（位置约为 `odom` 中 `[0.8, -0.6, 0.30]`）。它不是 REMANI 的 ESDF 障碍物，也不会
随 AMCL 的 `map -> odom` 自动变换。制作保守副本时必须结合现场决定是删除、关闭
`environmentCollision`，还是改成真实且与本次 `odom` 对齐的障碍；不要把测试方盒
原样带到实机并误以为它来自 NPZ 地图。关闭该项也意味着 OCS2 不再提供这层环境
碰撞约束，仍需依靠 REMANI ESDF、低速和实体安全措施。

### 阶段 0：消除 MRT 重名

进入 D0 前，必须保证 ROS 图中只有一个 `/tracer_jaka_mrt_node` 主节点和一个
`/tracer_jaka_mrt_node_ocs2_internal` 内部节点：

```bash
# 关闭所有 launch 终端后
ros2 daemon stop
ros2 daemon start

pgrep -af tracer_jaka_mrt_node
ros2 node list | grep tracer_jaka_mrt
```

期望：

- 操作系统中只有一个 `tracer_jaka_mrt_node` 进程；
- ROS 图中只有 `/tracer_jaka_mrt_node` 与 `/tracer_jaka_mrt_node_ocs2_internal`。

如果出现两个完全相同的 `/tracer_jaka_mrt_node`，优先检查
`src/bringup/tracer_jaka_bringup/launch/ocs2_real.launch.py` 是否仍给 MRT
节点显式 `name='tracer_jaka_mrt_node'`。已改为使用 C++ 节点自身名称；同时
`TracerJakaMrtNode.cpp` 中 OCS2 内部节点使用 `use_global_arguments(false)`，避免被
launch 层 `__node` 重命名成同一个名字。修改后重新编译：

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select tracer_jaka_ocs2
source install/setup.bash
```

未消除重名前，不得打开 `command_output_enabled:=true`。

### 阶段 D0：一键启动 dry-run（允许规划，本链路不下发命令）

确认阶段 B 的节点全部退出后执行：

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch tracer_jaka_bringup remani_mpc_localized_real.launch.py \
  can_port:=can0 \
  serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_e6872e3dafebed119ff7429aa88ea882-if00-port0 \
  lidar_host_ip:=192.168.8.1 \
  lidar_sensor_ip:=192.168.8.2 \
  map_file:=/home/a/WBMM/maps/site_2d.yaml \
  static_esdf_file:=/home/a/WBMM/maps/site_remani.npz \
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
  mobile_base_max_wheel_omega:=1.0 \
  mobile_base_max_wheel_alpha:=2.0 \
  mobile_base_non_singul_vel:=0.02 \
  jaka_read_only:=true \
  command_output_enabled:=false \
  start_ocs2:=true \
  start_remani:=true \
  start_bridge:=true \
  start_arm_pose:=false
```

这个阶段有两层独立保护：`jaka_read_only:=true` 让 JAKA hardware interface 不启用
servo 且 `write()` 不访问机器人；`command_output_enabled:=false` 让 MRT 不创建
`/cmd_vel` 和 `/arm_controller/commands` 发布器。因此可以在 RViz 用
`2D Goal Pose` 发一个目标，验证 REMANI、map/odom 变换、MPC policy 和轨迹显示，底盘
和机械臂都不应动作。`freeze_manipulator:=true` 还会让 REMANI 保持当前实测臂型。
这里切断的是本 launch 的 MRT 输出，不会阻止工作空间外的遥控、teleop 或遗留节点向
`/cmd_vel` 发布；启动前必须退出这些节点，并保留底盘物理急停。

启动后等待约 20 秒，再执行：

```bash
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers "{}"
ros2 topic hz /joint_states
ros2 topic echo /fts_broadcaster/wrench --once
ros2 topic hz /odometry/filtered_map
ros2 topic echo /mobile_manipulator_mpc_observation --once
ros2 topic info /cmd_vel -v
ros2 topic info /arm_controller/commands -v
ros2 topic info /mobile_manipulator_mpc_target -v
```

dry-run 的期望结果：

- `joint_state_broadcaster`、`fts_broadcaster`、`arm_controller` 为
  `active`；forward controller 激活不代表能写硬件，此时硬件仍为只读；
- `/odometry/filtered_map.header.frame_id` 为 `map`，`map -> odom -> base_footprint` 连续；
- `/cmd_vel` 和 `/arm_controller/commands` 只有订阅者，没有 MRT 发布者；
- `/mobile_manipulator_mpc_target` 只有 REMANI bridge 一个发布者；
- 日志出现 `DRY-RUN safety gate active` 和 JAKA `ReadOnly=true`；
- RViz 中 2D 地图、激光、机器人、保存的 3D ESDF 和 REMANI 轨迹位置一致。

若任何一项不满足，停在本阶段，不要打开执行开关。

### 阶段 D1-A：只使能 JAKA hardware 写入

清空机械臂工作区并准备物理急停。退出 D0 后重启，改为：

```text
jaka_read_only:=false
command_output_enabled:=false
start_remani:=false
start_bridge:=false
freeze_manipulator:=true
```

此阶段 JAKA servo 会启用，但 MRT 仍不创建命令发布器。观察至少 30 秒：

- 机械臂保持启动时的实测姿态；
- 不应回零、跳变或抖动；
- `/cmd_vel` 发布者为 0；
- `/arm_controller/commands` 发布者为 0；
- 关节速度建议保持在 ±0.02 rad/s 内。

出现任何运动立即按物理急停。

### 阶段 D1-B：OCS2/MRT 当前姿态保持

D1-A 通过后，重启并只修改：

```text
jaka_read_only:=false
command_output_enabled:=true
start_remani:=false
start_bridge:=false
```

此时 OCS2/MRT 以真实 `/joint_states` 和 EKF 状态作为初始目标。检查：

```bash
ros2 topic info /cmd_vel -v
ros2 topic info /arm_controller/commands -v
ros2 topic echo /cmd_vel
ros2 topic echo /arm_controller/commands
ros2 topic echo /joint_states
```

验收标准：

- 两个命令话题各只有一个 MRT 发布者；
- 底盘保持 `|linear.x| < 0.01 m/s`、`|angular.z| < 0.03 rad/s`；
- 机械臂命令与实测关节角误差小于 0.03 rad；
- 连续观察至少 30 秒，无回零、跳动或振荡。

注意 `/joint_states.name` 当前顺序不是固定 `joint_1...joint_6`，比较命令与实测值
时必须按关节名称匹配，不能直接按数组下标比较。

### 阶段 E：恢复 REMANI 后发送小目标

退出 D1-B，再用阶段 D0 的完整命令启动，并将安全开关改为：

```text
jaka_read_only:=false
command_output_enabled:=true
start_remani:=true
start_bridge:=true
```

首次只在机器人正前方空旷区发送约 `0.20 m` 的直线目标，目标必须位于 `map`
坐标系。观察规划轨迹无碰撞、指令方向正确后再逐步增大距离。确认底盘流程稳定后，
才将 `freeze_manipulator` 改为 `false`，仍保持低速，对机械臂发送很小的构型变化。

此时 `/cmd_vel` 和 `/arm_controller/commands` 应各只有 OCS2 MRT 一个发布者；
`/joint_states` 仍只能来自真实 JAKA 的 joint-state broadcaster。任一命令话题出现多个
发布者，都应退出并排除冲突节点。

完整数据流为：

```text
RViz 2D Goal Pose (/goal_pose, frame=map)
  -> REMANI
  -> /planning/trajectory (map)
  -> remani_to_ocs2_reference_bridge + TF(map -> odom)
  -> /mobile_manipulator_mpc_target (odom)
  -> OCS2 MPC/MRT
  -> /cmd_vel + /arm_controller/commands
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
| `task_file` | `tracer_jaka_bringup/config/task_real_conservative.info` | 默认就是 OCS2 保守速度与代价配置；不要误传回 `task_real.info` |
| `urdf_file` | `tracer_jaka_description/urdf/tracer_jaka_zu5.urdf` | 运动学、碰撞体及传感器外参；硬件参数由 controlled xacro 注入 |
| `lib_folder` | `/tmp/ocs2_tracer_jaka_real/auto_generated` | OCS2 自动生成库目录；不同 task 建议使用不同目录 |
| `manipulator_max_vel` | `0.10` | REMANI 机械臂参考最大速度 rad/s |
| `manipulator_max_acc` | `0.20` | REMANI 机械臂参考最大加速度 rad/s² |
| `freeze_manipulator` | `true` | `true` 时 REMANI 保持当前臂型；底盘验证完成后才改 `false` |
| `mobile_base_max_wheel_omega` | `1.0` | REMANI 轮速上限 rad/s；轮径 0.07 m 时直线约 0.07 m/s |
| `mobile_base_max_wheel_alpha` | `2.0` | REMANI 轮角加速度上限 rad/s² |
| `mobile_base_non_singul_vel` | `0.02` | REMANI 非奇异最小规划线速度 m/s |
| `tracking_error_replan_enabled` | `false` | 跟踪误差自动重规划；调通前保持 `false` |
| `use_joy` | `false` | 必须保持 `false`，避免与 REMANI bridge 争抢 MPC target |
| `use_rviz` | `true` | 是否启动 OCS2 RViz |
| `jaka_read_only` | `true` | 第一层执行保护；`false` 才允许 hardware interface 写 JAKA |
| `command_output_enabled` | `false` | 第二层执行保护；`true` 才创建 MRT 底盘/机械臂命令发布器 |
| `start_ocs2` | `true` | 是否启动 MPC/MRT；dry-run 保持 `true` 以验证完整计算链 |
| `start_remani` / `start_bridge` | `true/true` | 是否启动规划器/OCS2 参考桥；保持测试阶段可把 `start_remani` 设为 `false` |
| `arm_max_delta_per_step` | `0.05` | 机械臂命令相对实测角的最大超前量 rad |
| `arm_max_command_velocity` | `0.15` | MRT 机械臂位置命令的斜率上限 rad/s |
| `can_port` | `can0` | Tracer CAN 接口 |
| `serial_port` | `/dev/ttyUSB0` | Hipnuc IMU 串口 |
| `start_imu` / `start_lidar` | `true/true` | 驱动已由外部启动时设 `false`，避免重复发布 |
| `start_arm_pose` | `false` | 实机 JAKA 在线时必须为 `false`，禁止假关节状态 |
| `lidar_host_ip/lidar_sensor_ip` | `0.0.0.0/192.168.198.2` | 会透传给 LiDAR 驱动；现场为 192.168.8.x 时必须显式覆盖 |
| `robot_ip/local_ip` | `10.5.5.100/10.5.5.127` | 会注入 OCS2 使用的 JAKA xacro/hardware interface |

## 5. 参数到底去哪里修改

| 想调整的内容 | 真正生效的位置 | 是否可由本入口覆盖 |
| --- | --- | --- |
| 地图文件、AMCL 初值、ESDF 平移、两层执行保护、REMANI 速度/加速度 | `src/bringup/tracer_jaka_bringup/launch/remani_mpc_localized_real.launch.py` | 是，优先用 launch 参数 |
| AMCL 粒子数、激光模型、更新阈值、初始协方差 | `src/perception/tracer_jaka_localization/config/amcl_real.yaml` | 否，改 YAML 后重启 |
| EKF 融合项、频率、超时、IMU/轮速配置 | `src/simulation/tracer_jaka_mujoco/config/ekf_real.yaml` | 否，改 YAML 后重启 |
| LiDAR IP/端口/倒装/角度，驱动默认话题 | `src/bringup/tracer_jaka_bringup/launch/real_slam.launch.py` | 顶层透传 host/sensor IP 与 `scan_topic`；端口等可直接作为嵌套 launch 参数传入 |
| IMU 驱动原始配置 | `src/drivers/sensors/hipnuc_imu/config/hipnuc_config.yaml` | 顶层只透传串口和话题 |
| JAKA IP、本机 EDG IP、力传感器偏置 | `tracer_jaka_description/urdf/tracer_jaka_zu5.ros2_control.xacro` | IP 可由 `robot_ip/local_ip`（OCS2）或 `jaka_robot_ip/jaka_local_ip`（real_slam）覆盖 |
| LiDAR/IMU/JAKA 安装外参、机器人碰撞体 | `tracer_jaka_description/urdf/tracer_jaka_zu5.urdf` | 否，修改后重建/重启 |
| OCS2 底盘/机械臂最终速度上限 | `src/bringup/tracer_jaka_bringup/config/task_real_conservative.info` 的 `jointVelocityLimits` | 用 `task_file` 选择配置 |
| OCS2 输入平滑程度 | 同一 task 的 `inputCost.R` | 用 `task_file` 选择副本 |
| OCS2 跟踪权重 | 同一 task 的 `wholeBodyTracking.Q` | 用 `task_file` 选择副本 |
| OCS2 自碰撞/静态障碍物安全距离 | 同一 task 的 `selfCollision`、`environmentCollision` | 用 `task_file` 选择副本 |
| REMANI 车体尺寸、轮径、机械臂关节限位 | `src/vendor/remani_planner/plan_manage/config/mm_param.yaml` | 轮速/轮加速度/非奇异速度及臂速度/加速度可由本入口覆盖 |
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
| 本启动链输出闸 | dry-run 使用 `jaka_read_only=true`、`command_output_enabled=false` | 同时切断 JAKA 写入和 MRT 两类命令发布器 |
| 目标源 | `use_joy=false` | 保证只有 REMANI bridge 发布 MPC target |
| 自动行为 | `tracking_error_replan_enabled=false` | 避免误差或定位抖动触发意外新轨迹 |
| REMANI 机械臂 | `freeze_manipulator=true`、`vel=0.10`、`acc=0.20` | 先验证底盘和坐标系 |
| REMANI 底盘 | 轮速 `1.0 rad/s`、轮加速度 `2.0 rad/s²` | 使规划参考本身也保持低速 |
| OCS2 底盘 | task 中线速度 `±0.05 m/s`、角速度 `±0.20 rad/s` | 限制最终实际控制输入 |
| OCS2 机械臂 | task 中每关节 `±0.15 rad/s` | REMANI 限速之外再加执行层上限 |
| 目标距离 | 首次 `0.20~0.30 m`、正前方、无障碍 | 便于快速判断方向和坐标是否正确 |
| ESDF offset | 先全为 `0.0` | 未经测量不要用 offset “目测调图” |

调快时一次只改一组参数，每次保留日志和安全员。推荐顺序是：定位稳定性 → 底盘速度
→ 允许机械臂规划 → 机械臂速度/加速度 → 最后才启用误差自动重规划。

## 7. 常见故障

### AMCL 报 `class differential ... does not exist`

ROS 2 Humble 要求 `robot_model_type` 使用完整 pluginlib 类名。当前配置应为：

```yaml
robot_model_type: nav2_amcl::DifferentialMotionModel
```

修改后重新编译并重新 source；如果日志仍显示 `differential`，说明终端还在使用旧的
install 空间：

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select tracer_jaka_localization
source install/setup.bash
```

### `fts_broadcaster` 初始化失败

若 JAKA 日志已经打印 6 个 `init pos` 且显示 `ReadOnly=true`，说明机械臂通讯和关节状态
读取已成功，失败仅发生在力传感器 broadcaster。Humble 的该 broadcaster 要求非空
`frame_id`；本功能包通过独立的 `fts_broadcaster.yaml`、全节点通配段 `/**` 和
spawner 的 `--param-file` 显式加载配置，并在
`joint_state_broadcaster` 启动完成后再顺序加载它。重新编译
`tracer_jaka_mujoco` 后重启终端 1。临时只验证关节数据时可设
`start_jaka_fts:=false`，这不会启用任何机械臂控制。Humble 中 broadcaster 的原始私有
话题会解析为 `/controller_manager/wrench`，启动文件已将它重映射为工程统一使用的
`/fts_broadcaster/wrench`。

这些修改只涉及“如何加载并发布”力传感器状态，没有修改硬件接口里的 F/T 原始读取、
零偏采样、滤波、力臂补偿或单位换算。现在统一的 `moveit.launch.py` 会在 real 分支中
按 joint state broadcaster、机械臂控制器、F/T broadcaster 的顺序加载；而
旧配置使用的控制器名是 `fts_broadcaster`，本只读链使用
`fts_broadcaster`，参数段必须按实际名称和当前 controller_manager 的命名空间规则
匹配。另外，本机当前安装的 Humble FTS broadcaster 会在构造阶段强制检查非空
`frame_id`，缺少它就直接初始化失败。以控制器列表为准，不要只看 launch 是否退出：

```bash
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers "{}"
```

### 没有 `map -> odom`

检查 `/scan`、`/map`、AMCL 生命周期和初始位姿。确认没有另一个
`slam_toolbox`/AMCL 同时发布该 TF：

```bash
ros2 node list
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map odom
```

如果看到 `Please set the initial pose`，先确认当前安装空间参数为 true：

```bash
ros2 param get /amcl set_initial_pose
```

当前 launch 会自动设为 `true`。若机器人实际不在命令行给出的初始位置，在 RViz 点击
`2D Pose Estimate` 后，在地图中的真实位置拖出朝向；也可以直接重启定位 launch，并把
`initial_x/y/yaw` 改成实测值。初始位姿错误时即使出现 `map` 坐标系，激光也不会与地图
正确重合。

### `/odometry/filtered_map` 没有数据

它要求 `/odometry/filtered` 和 `map -> odom` 同时存在。先分别检查 EKF 和 AMCL，
再检查 `odom_to_map_relay` 日志。当前 relay 使用“最新可用”的 `map -> odom` 修正来
转换实时 EKF pose：机器人静止时 AMCL 可能不刷新 TF 时间戳，不能用每条较新的 EKF
时间戳做精确查询，否则会因向未来外推而一直丢弃输出。

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
| 实机仅传感器/EKF/SLAM | `ros2 launch tracer_jaka_bringup real_slam.launch.py` |
| 实机仅保存地图定位 | `ros2 launch tracer_jaka_localization localization_real.launch.py` |
| 实机仅 OCS2/JAKA/底盘 | `ros2 launch tracer_jaka_bringup ocs2_real.launch.py` |
| MuJoCo 完整闭环 | `ros2 launch tracer_jaka_bringup ocs2_sim.launch.py` |
