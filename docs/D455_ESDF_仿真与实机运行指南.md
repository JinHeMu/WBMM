# Tracer + JAKA + D455：SLAM 与 3D ESDF 仿真/实机运行指南

> 适用工作区：
>
> - ROS 2、MuJoCo、定位与 SLAM：`/home/a/WBMM`
> - Isaac ROS、nvblox 与 ESDF：`/home/a/workspaces/isaac_ros-dev`
>
> 当前传感器分工：
>
> - 轮式里程计 + IMU + 2D Lakibeam：机器人定位和 `slam_toolbox` 建图。
> - 底盘固定式 D455：nvblox TSDF、Mesh 和 3D ESDF 建图。
> - 机械臂末端 D435：抓取识别；默认不参与 ESDF 建图。

## 1. 总体运行结构

```mermaid
flowchart LR
    subgraph LOC["定位与 2D SLAM（主机）"]
        W["轮式里程计"]
        I["IMU"]
        L["2D Lakibeam /scan"]
        E["robot_localization EKF"]
        S["slam_toolbox"]
        W --> E
        I --> E
        E -->|"odom → base_footprint"| TF["TF 树"]
        L --> S
        TF --> S
        S -->|"map → odom"| TF
    end

    subgraph RGBD["固定式 D455（主机）"]
        D["D455 RGB + Depth"]
        DT["d455_link → optical frames"]
        D --> DI["/camera/d455/..."]
        DT --> TF
    end

    subgraph NV["Isaac ROS Docker"]
        N["nvblox TSDF 融合"]
        M["实时 Mesh"]
        ES["3D ESDF"]
        V["RViz / 查询服务"]
        DI --> N
        TF --> N
        N --> M
        N --> ES
        M --> V
        ES --> V
    end
```

### 关键 TF

```text
map
└── odom                         slam_toolbox 发布
    └── base_footprint           robot_localization 发布
        └── base_link            URDF 固定关节
            ├── laser_link
            ├── imu_link
            ├── d455_link
            │   ├── d455_depth_optical_frame
            │   └── d455_color_optical_frame
            └── JAKA 机械臂
                └── d435i_link
```

D455 在当前 URDF 中的安装基准：

```xml
<joint name="base_to_d455" type="fixed">
  <parent link="base_link"/>
  <child link="d455_link"/>
  <origin xyz="0.25 0 0.24" rpy="1.5708 0 1.5708"/>
</joint>
```

仿真中实测：

```text
base_footprint -> d455_depth_optical_frame
translation = [0.255, 0.000, 0.387]
相机视线方向 = base_link +X（机器人正前方）
```

## 2. 通用环境要求

主机与 Docker 必须使用相同的 DDS 配置：

```bash
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

提供的 MuJoCo ESDF、D455 和 nvblox launch 已默认设置为上述值。但单独执行
`ros2 topic`、`tf2_echo` 或手动启动 `real_slam.launch.py` 前，仍建议在当前
终端显式导出。

进入 Isaac ROS 容器时必须使用 `admin` 用户：

```bash
docker start isaac_ros_dev-x86_64-container

docker exec -it -u admin \
  --workdir /workspaces/isaac_ros-dev \
  isaac_ros_dev-x86_64-container bash
```

不要使用容器内的 `root` 用户运行 ROS 2。当前机器上 root 用户可能无法通过
Fast DDS 发现主机发布的话题。

## 3. 首次构建或修改代码后的构建

### 3.1 构建主机工作区

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select tracer_jaka_mujoco

source install/setup.bash
```

### 3.2 构建 Isaac ROS 工作区

先进入容器，然后执行：

```bash
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select my_nvblox_bringup

source install/setup.bash
```

## 4. MuJoCo 仿真运行

仿真需要两个终端：

- 主机终端：MuJoCo、轮式里程计、IMU、2D 激光、EKF、slam_toolbox、D455。
- Docker 终端：nvblox、3D ESDF、Mesh 和 ESDF RViz。

### 4.1 主机启动完整仿真

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash

export MUJOCO_GL=egl

ros2 launch tracer_jaka_mujoco esdf_sensor_sim.launch.py
```

该 launch 默认执行：

```text
esdf_sensor_sim.launch.py
└── slam_sim.launch.py
    ├── bridge.launch.py
    │   ├── MuJoCo 物理仿真
    │   ├── /clock
    │   ├── /wheel/odometry
    │   ├── /imu/data
    │   ├── /scan
    │   ├── /camera/d455/color/image_raw
    │   └── /camera/d455/depth/image_raw
    ├── robot_localization
    └── slam_toolbox
```

常用覆盖参数：

```bash
# 打开 SLAM RViz
ros2 launch tracer_jaka_mujoco esdf_sensor_sim.launch.py \
  rviz:=true

# 相机渲染较慢时降为 15 Hz
ros2 launch tracer_jaka_mujoco esdf_sensor_sim.launch.py \
  camera_rate:=15.0

# 使用其他 ROS Domain
ros2 launch tracer_jaka_mujoco esdf_sensor_sim.launch.py \
  ros_domain_id:=21
```

`esdf_sensor_sim.launch.py` 默认关闭 MuJoCo 自带 viewer，因为 EGL 离屏相机和
viewer 同时运行时性能较差。需要检查机器人模型时，可单独运行：

```bash
ros2 launch tracer_jaka_mujoco bridge.launch.py \
  viewer:=true \
  camera:=false
```

### 4.2 Docker 启动仿真 nvblox

进入容器：

```bash
docker exec -it -u admin \
  --workdir /workspaces/isaac_ros-dev \
  isaac_ros_dev-x86_64-container bash
```

容器内：

```bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
export ISAAC_ROS_NVBLOX_PLUGIN_FORCE_FALLBACK_MATERIAL=1
ros2 launch my_nvblox_bringup mujoco_esdf.launch.py
```

仿真 nvblox 默认读取：

```text
/camera/d455/depth/image_raw
/camera/d455/depth/camera_info
/camera/d455/color/image_raw
/camera/d455/color/camera_info
```

默认打开 nvblox RViz，并显示：

```text
/nvblox_node/mesh
/nvblox_node/esdf_3d_pointcloud
```

只运行建图、不打开 RViz：

```bash
ros2 launch my_nvblox_bringup mujoco_esdf.launch.py \
  rviz:=false
```

只显示 Mesh，不生成局部 ESDF 可视化点云：

```bash
ros2 launch my_nvblox_bringup mujoco_esdf.launch.py \
  esdf_viz:=false
```

## 5. 实机运行

实机推荐使用三个终端：

1. 主机：底盘、IMU、Lakibeam、EKF、slam_toolbox。
2. 主机：固定式 D455。
3. Docker：nvblox、ESDF 和 RViz。

如果还需要抓取识别，可增加第四个主机终端启动末端 D435。

### 5.1 实机定位与 2D SLAM：推荐一键方式

连接底盘 CAN、IMU 和 Lakibeam 后：

```bash
sudo chmod 777 /dev/ttyUSB0

cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 launch tracer_jaka_mujoco real_slam.launch.py
```

该 launch 默认启动：

- Tracer 底盘与轮式里程计；
- `robot_state_publisher`；
- Hipnuc IMU；
- Lakibeam；
- `robot_localization`；
- `slam_toolbox`；
- SLAM RViz。

常用参数：

```bash
ros2 launch tracer_jaka_mujoco real_slam.launch.py \
  can_port:=can0 \
  serial_port:=/dev/ttyUSB0 \
  wheel_odom_topic:=/odom \
  imu_topic:=/IMU_data \
  scan_topic:=/scan \
  rviz:=true
```

如果传感器已经单独启动，可以关闭对应驱动：

```bash
ros2 launch tracer_jaka_mujoco real_slam.launch.py \
  start_base:=false \
  start_imu:=false \
  start_lidar:=false
```

### 5.2 实机定位：沿用已有传感器启动方式

如果继续使用原来的启动命令：

```bash
# IMU
sudo chmod 777 /dev/ttyUSB0
ros2 launch hipnuc_imu imu_spec_msg.launch.py

# Lakibeam
ros2 launch lakibeam1 lakibeam1_scan_view.launch.py
```

确保底盘里程计也已经发布，然后只启动 EKF 和 slam_toolbox：

```bash
cd /home/a/WBMM
source install/setup.bash

export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 launch tracer_jaka_mujoco localization.launch.py \
  wheel_odom_topic:=/odom \
  imu_topic:=/IMU_data \
  scan_topic:=/scan
```

如果实际话题不同，先执行：

```bash
ros2 topic list
```

再修改上述三个 topic 参数。

### 5.3 主机启动固定式 D455

只有一台 RealSense 时：

```bash
cd /home/a/WBMM
source install/setup.bash

ros2 launch tracer_jaka_mujoco d455_sensor.launch.py
```

同时连接 D455 和 D435 时，必须使用序列号区分：

```bash
ros2 launch tracer_jaka_mujoco d455_sensor.launch.py \
  serial_no:="'D455序列号'"
```

D455 默认发布：

```text
/camera/d455/depth/image_rect_raw
/camera/d455/depth/camera_info
/camera/d455/color/image_raw
/camera/d455/color/camera_info
```

RealSense 驱动发布：

```text
d455_link -> d455_depth_optical_frame
d455_link -> d455_color_optical_frame
```

URDF 的 `robot_state_publisher` 发布：

```text
base_link -> d455_link
```

### 5.4 Docker 启动实机 nvblox

容器内执行：

```bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash

ros2 launch my_nvblox_bringup d455_esdf.launch.py
```

实机启动默认使用持久地图模式：

- `map_clearing_radius_m:=-1.0`：不再删除机器人一定半径外的体素；
- `esdf_viz_follow_robot:=false`：ESDF 显示窗口固定在启动位置；
- 显示范围默认是 `12 m × 12 m × 3 m`，刷新率为 `0.2 Hz`，
  抽样倍率为 3；
- `esdf_viz_max_distance:=0.5`：只显示距离障碍物不超过 0.5 m 的体素，
  隐藏远处自由空间。

如果场地超过 8 m，可扩大固定显示范围。为避免服务响应和 RViz 数据量
过大，同时降低刷新率并提高抽样倍率：

```bash
ros2 launch my_nvblox_bringup d455_esdf.launch.py \
  esdf_viz_size_x:=12.0 \
  esdf_viz_size_y:=12.0 \
  esdf_viz_rate:=0.2 \
  esdf_viz_subsampling:=3
```

如果后续只需要机器人周围的局部避障距离场，可恢复滚动地图：

```bash
ros2 launch my_nvblox_bringup d455_esdf.launch.py \
  map_clearing_radius_m:=7.0 \
  esdf_viz_follow_robot:=true \
  esdf_viz_size_x:=4.0 \
  esdf_viz_size_y:=4.0 \
  esdf_viz_rate:=1.0
```

如果 D455 驱动的话题名称不同，可覆盖：

```bash
ros2 launch my_nvblox_bringup d455_esdf.launch.py \
  depth_image_topic:=/camera/d455/depth/image_rect_raw \
  depth_camera_info_topic:=/camera/d455/depth/camera_info \
  color_image_topic:=/camera/d455/color/image_raw \
  color_camera_info_topic:=/camera/d455/color/camera_info
```

### 5.5 可选：启动末端 D435 做抓取识别

```bash
cd /home/a/WBMM
source install/setup.bash

ros2 launch tracer_jaka_mujoco d435_sensor.launch.py \
  serial_no:="'D435序列号'"
```

D435 默认使用：

```text
根 frame：d435i_link
话题前缀：/camera/d435i/
用途：机械臂末端抓取、物体识别
```

默认 nvblox 不订阅 D435，因此机械臂运动不会改变 ESDF 主相机的安装姿态。

## 6. 当前 nvblox 参数

参数文件：

```text
/home/a/WBMM/src/perception/my_nvblox_bringup/config/nvblox_3d.yaml
```

| 参数 | 当前值 | 作用 | 建议 |
|---|---:|---|---|
| `voxel_size` | `0.05 m` | TSDF/ESDF 分辨率 | 常规导航保持 5 cm |
| `mapping_type` | `static_tsdf` | 静态环境 TSDF | 当前正确 |
| `esdf_mode` | `3d` | 真正三维 ESDF | 不要改成 2D |
| `integrate_depth_rate_hz` | `30` | 最大深度融合频率 | GPU 压力大可降到 15 |
| `integrate_color_rate_hz` | `5` | 颜色融合频率 | ESDF 不依赖高频颜色 |
| `update_mesh_rate_hz` | `5` | Mesh 更新频率 | RViz 卡顿可降到 2 |
| `update_esdf_rate_hz` | `10` | ESDF 更新频率 | 运动规划建议 10 Hz |
| `projective_integrator_max_integration_distance_m` | `5.0` | 最大深度融合距离 | 室内保持 4～5 m |
| `esdf_integrator_max_distance_m` | `2.0` | ESDF 截断距离 | 局部规划通常足够 |
| `map_clearing_radius_m` | `7.0` | 保留机器人附近地图 | 显存不足时减小 |
| `maximum_input_queue_length` | `20` | RGB-D 输入队列 | 延迟大时不要盲目增大 |

### ESDF RViz 可视化参数

这些参数只改变显示，不改变规划器内部 ESDF：

| launch 参数 | 默认值 | 说明 |
|---|---:|---|
| `esdf_viz` | `true` | 是否发布局部 3D ESDF 点云 |
| `esdf_viz_size_x` | `4.0` | 显示区域 X 尺寸 |
| `esdf_viz_size_y` | `4.0` | 显示区域 Y 尺寸 |
| `esdf_viz_min_z` | `-0.2` | 相对底盘最低显示高度 |
| `esdf_viz_size_z` | `3.0` | 显示区域高度 |
| `esdf_viz_rate` | `1.0` | ESDF 点云发布频率 |
| `esdf_viz_max_distance` | `1.5` | 显示的最大障碍距离 |
| `esdf_viz_subsampling` | `2` | 体素显示降采样 |

低负载示例：

```bash
ros2 launch my_nvblox_bringup d455_esdf.launch.py \
  esdf_viz_rate:=0.5 \
  esdf_viz_subsampling:=3 \
  esdf_viz_max_distance:=1.0
```

## 7. 需要修改参数时去哪里改

| 修改目标 | 文件 |
|---|---|
| D455 在机器人上的安装位置 | `/home/a/WBMM/src/simulation/tracer_jaka_mujoco/urdf/tracer_jaka_zu5.urdf` |
| D455 在 MuJoCo 中的位置和视线 | `models/tracer_jaka_zu5_robot.xml` |
| MuJoCo D455 频率、分辨率、FOV、话题 | `config/sensors.yaml` |
| 实机 D455 分辨率、帧率、命名空间 | `launch/d455_sensor.launch.py` |
| 实机 D435 分辨率、帧率、命名空间 | `launch/d435_sensor.launch.py` |
| 实机 EKF 融合配置 | `config/ekf_real.yaml` |
| 实机 slam_toolbox 配置 | `config/slam_toolbox_real.yaml` |
| nvblox 体素、距离和更新频率 | `my_nvblox_bringup/config/nvblox_3d.yaml` |
| nvblox 仿真输入话题 | `my_nvblox_bringup/launch/mujoco_esdf.launch.py` |
| nvblox 实机输入话题 | `my_nvblox_bringup/launch/d455_esdf.launch.py` |
| ESDF RViz 布局 | `my_nvblox_bringup/config/nvblox_esdf.rviz` |

### 修改 D455 安装姿态后的同步规则

如果修改了 URDF 中的：

```xml
<joint name="base_to_d455">
  <origin xyz="..." rpy="..."/>
</joint>
```

必须同步检查：

1. MuJoCo `tracer_jaka_zu5_robot.xml` 中 `body name="d455_link"` 的
   `pos` 和 `quat`；
2. MuJoCo `camera name="d455_depth"` 的视线是否仍朝机器人前方；
3. 实机 `tf2_echo base_link d455_link`；
4. 仿真 `tf2_echo base_footprint d455_depth_optical_frame`；
5. 修改后重新构建 `tracer_jaka_mujoco`。

## 8. 运行后的检查命令

### 8.1 检查定位和 SLAM

```bash
ros2 topic hz /scan
ros2 topic hz /imu/data          # 仿真
ros2 topic hz /IMU_data          # 当前实机默认
ros2 topic hz /odometry/filtered

ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map odom
```

### 8.2 检查仿真 D455

```bash
ros2 topic hz /camera/d455/depth/image_raw
ros2 topic echo /camera/d455/depth/camera_info --once
ros2 run tf2_ros tf2_echo base_footprint d455_depth_optical_frame
```

### 8.3 检查实机 D455

```bash
ros2 topic hz /camera/d455/depth/image_rect_raw
ros2 topic echo /camera/d455/depth/camera_info --once
ros2 run tf2_ros tf2_echo base_link d455_depth_optical_frame
```

### 8.4 检查容器是否收到主机数据

容器内：

```bash
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic list | grep d455
ros2 topic hz /camera/d455/depth/image_rect_raw
ros2 run tf2_ros tf2_echo odom d455_depth_optical_frame
```

仿真时把 `image_rect_raw` 换为 `image_raw`。

### 8.5 检查 nvblox

```bash
ros2 node info /nvblox_node
ros2 topic hz /nvblox_node/mesh
ros2 topic hz /nvblox_node/esdf_3d_pointcloud
```

主要输出：

```text
/nvblox_node/mesh
/nvblox_node/esdf_3d_pointcloud
/nvblox_node/get_esdf_and_gradient
/nvblox_node/back_projected_depth/d455_depth_optical_frame
```

注意：在 `esdf_mode: 3d` 下，内部 ESDF 不会通过
`/nvblox_node/static_esdf_pointcloud` 持续发布。当前功能包使用
`/nvblox_node/esdf_3d_pointcloud` 显示局部三维距离场。

### 8.6 NUC 录制、Docker 离线建立 ESDF

这个流程可以把跨机器 DDS 延迟与 nvblox/TF 配置问题分开诊断。NUC 先
本地录制深度、相机内参和 TF，笔记本再把数据放慢回放给 nvblox。

NUC：

```bash
cd ~/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p bags

ros2 launch tracer_jaka_mujoco record_d455_esdf_bag.launch.py \
  output:=bags/d455_esdf_01
```

先静止 5 秒，再缓慢运动。按一次 `Ctrl+C` 后必须等待压缩完成。
RGB-D 文件可能需要数分钟；出现 `Compressing file` 后不要再次中断，
直到返回命令提示符且目录中出现 `metadata.yaml`。检查核心话题：

```bash
ros2 bag info bags/d455_esdf_01
```

将整个 `d455_esdf_01` 目录复制到笔记本：

```bash
mkdir -p /home/a/workspaces/isaac_ros-dev/bags
rsync -avP ras@NUC_IP:~/WBMM/bags/d455_esdf_01/ \
  /home/a/workspaces/isaac_ros-dev/bags/d455_esdf_01/
```

Docker 内：

```bash
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch my_nvblox_bringup d455_bag_esdf.launch.py \
  bag:=/workspaces/isaac_ros-dev/bags/d455_esdf_01
```

该入口自动使用 `use_sim_time=true`、发布 `/clock`、开启 RGB 融合并默认以
`0.25` 倍速回放。离线 RGB-D 与 TF 均使用 Reliable QoS，深度融合限频提高
到 1000 Hz，并将 nvblox 输入队列扩大到 500。TSDF 权重衰减默认关闭
（`decay_tsdf_rate_hz:=0.0`）：nvblox 默认 5 Hz 衰减会在回放过程中持续
抹掉约 30 秒前未再被观测的体素，导致离线地图只剩最后一小段（这也是
之前"导出只有最后一帧"的原因）。导出前会读取 bag 元数据，
只有 `depth callback = processed = integrated = bag 深度帧数`（允许
`drain_max_pending_frames` 帧尾差，默认 3：bag 最后一帧深度没有更晚的
TF，仿真时钟也已停止，永远无法被处理）连续稳定后才保存 PLY、nvblx
和 ESDF，否则拒绝生成残缺文件。导出器在回放开始前就已启动，并预先
创建 save_timings/save_ply/save_map/ESDF 四个客户端；回放结束后由
`/nvblox_export_trigger`（std_msgs/msg/Bool）触发导出。这样避免回放
结束后才冷启动一个新 DDS 参与者去发现 nvblox 服务，消除曾出现的
`Waiting for save_timings service` 长时间不可达问题。ESDF 默认以 `esdf_use_aabb:=false`
导出 nvblox 全部已分配体素块（完整地图）；设为 `true` 时才使用
`esdf_min_*`/`esdf_size_*` 固定窗口。离线入口默认使用 ROS Domain 21，
与实机 Domain 20 隔离，避免实时话题和 bag 的时间戳混入同一个 nvblox。
若仍然积压，将 `rate` 降到 `0.10`。如果离线仍然失败，
应继续检查 bag 内的
`odom -> base_footprint -> d455_depth_optical_frame` 是否跳变。

录制端 `/tf` 使用 Best Effort，而回放端必须使用 Reliable。两端分别使用
各自功能包中的 `d455_esdf_bag_qos.yaml`，不要互相覆盖；否则 TF2 会报告
`incompatible QoS: RELIABILITY`，表现为回放后不存在 `odom`。

4 GB GPU 上，离线无界持久地图默认使用 `voxel_size:=0.10`。实时滚动局部
地图仍使用 5 cm；不要在 4 GB 显存上同时运行 MuJoCo nvblox 和 bag nvblox。

### 8.7 离线导出后主机显示/验证完整地图

Docker 内执行完整离线导出（`rviz:=false` 表示容器内不启动 RViz；需要容器内
实时查看时可去掉）：

```bash
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch my_nvblox_bringup d455_bag_esdf.launch.py \
  bag:=/workspaces/isaac_ros-dev/bags/d455_rgbd_esdf_02 \
  map_output:=/workspaces/isaac_ros-dev/bag_export/site.nvblx \
  ply_output:=/workspaces/isaac_ros-dev/bag_export/site_mesh.ply \
  esdf_output:=/workspaces/isaac_ros-dev/bag_export/site_remani.npz \
  map2d_output:=/workspaces/isaac_ros-dev/bag_export/site_2d.yaml \
  unknown_is_occupied:=false \
  rviz:=false
```

等待日志依次出现：

```text
Mapping completion received; exporting nvblox map
All synchronized depth inputs were processed; starting complete-map export
Saved nvblox mesh PLY: .../site_mesh.ply
Saved native nvblox map: .../site.nvblx
Saved REMANI ESDF: .../site_remani.npz
```

然后在**主机**终端（不是 Docker）启动显示端：

```bash
source /home/a/WBMM/install/setup.bash

ros2 launch tracer_jaka_ocs2 ocs2_esdf_validation.launch.py \
  esdf_file:=/home/a/workspaces/isaac_ros-dev/bag_export/site_remani.npz \
  map2d_yaml:=/home/a/workspaces/isaac_ros-dev/bag_export/site_2d.yaml \
  ply_file:=/home/a/workspaces/isaac_ros-dev/bag_export/site_mesh.ply
```

显示端无需修改：RViz 配置已包含 `/map`（2D Map）、
`/nvblox_ply_mesh`（Marker）和 `/esdf_cloud`（PointCloud2，intensity 为
距离），Fixed Frame 为 `odom`。主机侧的 `grid_map` 是 develop/egg-link
安装、`tracer_jaka_ocs2` 是 symlink 安装，源码改动会直接生效。

验证是否完整：

```bash
# 以当前 d455_rgbd_esdf_02 为例，期望接近：
# shape=(169,161,73), voxel=0.10 m
# observed=298340, occupied=89821
# 观测范围 x≈[-5.60,9.80], y≈[-6.20,8.30]
python3 /tmp/analyze_export.py \
  /home/a/workspaces/isaac_ros-dev/bag_export/site_remani.npz \
  /home/a/workspaces/isaac_ros-dev/bag_export/site_mesh.ply
```

### 8.8 本次已解决的导出器卡死问题

现象：旧版导出器在 bag 播放结束后才作为新进程启动，随后一直打印
`Waiting for save_timings service`，连 nvblox 的 `save_timings` 服务都发现
不了，最终只能 `timeout` 杀掉整条 launch。

原因：播放结束后冷启动一个新 ROS/DDS 参与者去发现已运行很久的 nvblox，
这一轮服务发现在个别环境下不稳定；与 TSDF decay 是否关闭没有直接因果。

解决方法（已内置到 `my_nvblox_bringup`）：

1. 导出器随回放一起启动，回放期间就创建并发现
   `save_timings / save_ply / save_map / ESDF` 四个服务客户端；
2. 回放结束并等待 `export_settle_time` 后，launch 向
   `/nvblox_export_trigger` 发布 `std_msgs/msg/Bool` 触发导出；
3. 导出器等待服务时主动 `rclpy.spin_once()` 处理 DDS graph 事件；
4. 所有 nvblox 服务调用都有超时（`service_call_timeout_sec`，默认 300），
   不再无限挂起。

后续使用不需要额外操作。若仍出现 `Waiting for save_timings service`，
优先确认：

```bash
# 容器内检查安装的 launch/脚本是否已是新版本
grep -n "trigger_topic\|_prepare_service_clients" \
  /workspaces/isaac_ros-dev/install/my_nvblox_bringup/share/my_nvblox_bringup/launch/d455_bag_esdf.launch.py \
  /workspaces/isaac_ros-dev/install/my_nvblox_bringup/lib/python3.10/site-packages/my_nvblox_bringup/nvblox_map_exporter.py
```

若确认已是新版，可把 `service_call_timeout_sec:=600` 调大后重试。

## 9. 常见问题

### 9.1 RViz 提示 `Frame [odom] does not exist`

依次检查：

```bash
ros2 topic echo /clock --once       # 只针对仿真
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_footprint
```

常见原因：

- 主机和容器 `ROS_DOMAIN_ID` 不一致；
- 容器使用 root 而不是 admin；
- 仿真 nvblox 没有使用 `use_sim_time=true`；
- EKF 尚未启动或没有收到里程计/IMU；
- Docker 启动早于主机 TF，启动后等待几秒即可。

### 9.2 只能看到反投影深度点，看不到地图

反投影点只表示 nvblox 收到了当前深度帧，不等于地图显示已经开启。

检查：

```bash
ros2 topic echo /nvblox_node/mesh nvblox_msgs/msg/Mesh --once
ros2 topic echo /nvblox_node/esdf_3d_pointcloud \
  sensor_msgs/msg/PointCloud2 --once --field width
```

RViz 的 Fixed Frame 应设为 `odom`。

### 9.3 ESDF/相机出现在机器人很远的位置

检查完整 TF：

```bash
ros2 run tf2_ros tf2_echo base_footprint d455_depth_optical_frame
ros2 run tf2_ros tf2_echo odom d455_depth_optical_frame
```

当前仿真的正常平移值应接近：

```text
[0.255, 0.000, 0.387]
```

实机允许有毫米级光学中心偏移，但不应相差数米。

### 9.4 相机方向反了

实机的 `d455_link` 必须遵守 ROS 相机基坐标约定：

```text
+X 向前，+Y 向左，+Z 向上
```

RealSense 驱动会从该坐标系生成 optical frame。CAD 模型自身的轴向补偿
只能写在 `<visual><origin .../></visual>` 中，不能写到
`base_to_d455` 固定关节。当前 URDF 已按此规则校正：

```text
base_link -> d455_link: rpy = 0 0 0
d455.STL visual:         rpy = 1.5708 0 1.5708
```

验证相机光轴：

```bash
ros2 run tf2_ros tf2_echo base_link d455_link
ros2 run tf2_ros tf2_echo base_link d455_depth_optical_frame
```

第一条的旋转应接近单位旋转。D455 前方物体的深度点变换到
`base_link` 后，其 `x` 应主要为正值。

### 9.5 机器人运动后旧 ESDF 消失

旧配置同时存在两个滚动窗口：

1. nvblox 每秒清除机器人 7 m 外的体素；
2. `/nvblox_node/esdf_3d_pointcloud` 只查询机器人周围
   `4 m × 4 m × 3 m` 的范围。

因此旧区域不在点云中不代表相机停止融合。当前实机默认已经关闭地图
半径清除，并把 ESDF 查询窗口固定在启动位置。低带宽的
`/nvblox_node/mesh` 也应保持显示。

注意：关闭清图后，GPU 显存会随着探索范围增长。长期导航或大场地应使用
滚动局部模式；短时间建图、标定和验证适合持久模式。

### 9.6 离线导出的地图只有最后一小段

现象：bag 回放时 RViz 里地图正常生长，但最终导出的 PLY/ESDF 只剩
最后一小段（看起来像"最后一帧"），`site_timings.txt` 里
`depth_image_callback ≈ processed ≈ integrated`。

原因不是丢帧，而是 nvblox 的 TSDF 权重衰减：`decay_tsdf_rate_hz` 默认
5 Hz、衰减因子 0.95，约 27 秒后所有未被重新观测的体素权重低于阈值而被
整块删除。机器人走过的地方边走边消失，只剩最后约 30 秒视野。实机
持久地图同样受影响。

处理：

```bash
ros2 launch my_nvblox_bringup d455_bag_esdf.launch.py \
  bag:=... decay_tsdf_rate_hz:=0.0
```

当前启动文件已把 `decay_tsdf_rate_hz` 默认改为 `0.0`。同时确认导出
ESDF 使用 `esdf_use_aabb:=false`（默认），固定 12 m × 12 m 窗口装不下
整幅地图时会把外围裁掉。若 bag 末尾 1~3 帧没有对应 TF 导致
`processed < callback`，把 `drain_max_pending_frames` 调到与帧尾差一致。
若回放结束后一直打印 `Waiting for save_timings service`，说明导出器
在播放结束后才冷启动并没能完成 DDS 服务发现；确认当前包已同步/构建
（导出器会随回放预启动），并把 `service_call_timeout_sec` 从默认 300
调大重试。

### 9.7 nvblox 或 RViz 卡顿

优先按顺序降低：

1. `esdf_viz_subsampling:=3`；
2. `esdf_viz_rate:=0.5`；
3. `esdf_viz_max_distance:=1.0`；
4. `camera_rate:=15.0`；
5. 只显示 Mesh：`esdf_viz:=false`。

不要先增大输入队列。队列过大会增加延迟，使当前深度帧与 TF 时间更难匹配。

### 9.8 同时连接 D455 和 D435 后相机串号

分别指定序列号：

```bash
ros2 launch tracer_jaka_mujoco d455_sensor.launch.py \
  serial_no:="'D455序列号'"

ros2 launch tracer_jaka_mujoco d435_sensor.launch.py \
  serial_no:="'D435序列号'"
```

并确认：

```text
D455 -> /camera/d455/...
D435 -> /camera/d435i/...
```

## 10. 推荐日常启动清单

### 仿真

主机：

```bash
cd /home/a/WBMM
source install/setup.bash
export MUJOCO_GL=egl
ros2 launch tracer_jaka_mujoco esdf_sensor_sim.launch.py
```

Docker：

```bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch my_nvblox_bringup mujoco_esdf.launch.py
```

### 实机

主机终端 1：

```bash
cd /home/a/WBMM
source install/setup.bash
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
sudo chmod 777 /dev/ttyUSB0
ros2 launch tracer_jaka_mujoco real_slam.launch.py
```

主机终端 2：

```bash
cd /home/a/WBMM
source install/setup.bash
ros2 launch tracer_jaka_mujoco d455_sensor.launch.py \
  serial_no:="'D455序列号'"
```

Docker：

```bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch my_nvblox_bringup d455_esdf.launch.py
```

可选 D435：

```bash
ros2 launch tracer_jaka_mujoco d435_sensor.launch.py \
  serial_no:="'D435序列号'"
```

## 11. 重要注意事项

1. 仿真和实机不要同时发布 `/camera/d455/...`。
2. 同一时刻只能有一个节点发布 `odom -> base_footprint`。
3. 同一时刻只能有一个节点发布 `map -> odom`。
4. nvblox 默认在 `odom` 中建立局部 ESDF；这是为了避免 SLAM 回环时 `map`
   坐标突跳直接影响局部规划。
5. D455 负责 ESDF，D435 负责末端识别，两者使用不同 topic 和 frame。
6. 修改 URDF 传感器位姿后，要同步 MJCF 并重新构建。
7. 重启 nvblox 会清空当前内存地图；修改 TF 后必须重启 nvblox，避免旧错误
   位姿的数据继续留在地图中。
