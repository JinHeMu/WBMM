# D455 RGB-D ESDF Rosbag 录制教程

本文用于在移动机械臂的 NUC 上录制 D455 RGB-D、定位 TF 和辅助传感器
数据，随后在笔记本 Docker 中离线建立 nvblox 三维 TSDF/ESDF。

适用环境：

- NUC：ROS 2 Humble，工作空间 `~/WBMM`
- 实机定位：`robot_localization + slam_toolbox`
- 深度相机：Intel RealSense D455
- ROS Domain：20
- DDS：`rmw_fastrtps_cpp`
- 离线 nvblox：`/home/a/workspaces/isaac_ros-dev`

---

## 1. 为什么要录制 bag

实时双机 ESDF 建图需要通过局域网传输 D455 原始 RGB-D：

- 深度 `640×480×30 Hz`：约 18.4 MB/s
- 彩色 `640×480×30 Hz`：约 27.6 MB/s
- 合计：约 46 MB/s，不含 DDS 和网络协议开销

在 NUC 本地录制后再离线回放，可以：

1. 排除局域网带宽和 DDS 丢包；
2. 反复使用同一段轨迹调试 nvblox；
3. 降速回放，避免 GPU 或 ESDF 可视化处理不过来；
4. 检查 TF、里程计和相机时间戳是否连续。

bag 不会修复错误的 TF。若录制的数据中没有动态 `/tf`，或者 TF 本身跳变，
离线 nvblox 仍然无法正确建图。

---

## 2. 本教程录制的话题

### 2.1 nvblox 必需话题

```text
/camera/d455/depth/image_rect_raw
/camera/d455/depth/camera_info
/camera/d455/color/image_raw
/camera/d455/color/camera_info
/tf
/tf_static
```

其中：

- 深度图和深度内参用于 TSDF/ESDF 几何融合；
- 彩色图和彩色内参用于给 nvblox Mesh 着色；
- `/tf` 提供机器人和相机的动态位姿；
- `/tf_static` 提供 `base_link -> d455_link -> optical frame` 等静态外参。

### 2.2 同时录制的诊断话题

```text
/odom
/odometry/filtered
/joint_states
/map
/map_metadata
/scan
/IMU_data
```

这些话题不是 nvblox 深度融合的直接输入，但可用于检查定位漂移、里程计、
激光 SLAM 和 IMU 状态。

不需要录制：

```text
/camera/d455/color/metadata
/camera/d455/depth/metadata
/camera/d455/extrinsics/depth_to_color
```

---

## 3. 第一次使用前同步和构建

确认 NUC 上存在：

```text
~/WBMM/src/simulation/tracer_jaka_mujoco/launch/record_d455_esdf_bag.launch.py
~/WBMM/src/simulation/tracer_jaka_mujoco/config/d455_esdf_bag_qos.yaml
```

构建：

```bash
cd ~/WBMM
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select tracer_jaka_mujoco

source install/setup.bash
```

确认使用的是新版录制入口：

```bash
grep -n "include-unpublished-topics" \
  ~/WBMM/src/simulation/tracer_jaka_mujoco/launch/record_d455_esdf_bag.launch.py

grep -n "sigterm_timeout" \
  ~/WBMM/src/simulation/tracer_jaka_mujoco/launch/record_d455_esdf_bag.launch.py
```

两个命令都必须有输出。新版入口会预先订阅指定话题，并允许 zstd 最多使用
10 分钟完成退出压缩。

---

## 4. 正确的启动顺序

建议至少使用三个 NUC 终端。

### 4.1 终端一：启动实机定位和 SLAM

```bash
cd ~/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

ros2 launch tracer_jaka_bringup real_slam.launch.py
```

必须确保：

- 底盘里程计正常；
- IMU 正常；
- Lakibeam `/scan` 正常；
- EKF 发布 `/odometry/filtered`；
- TF 中存在 `odom -> base_footprint`。

### 4.2 终端二：启动 D455

如果只连接一台 RealSense：

```bash
cd ~/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch tracer_jaka_bringup d455_real.launch.py
```

如果同时连接 D455 和 D435，指定 D455 序列号：

```bash
ros2 launch tracer_jaka_bringup d455_real.launch.py \
  serial_no:="'D455序列号'"
```

查询序列号：

```bash
rs-enumerate-devices
```

---

## 5. 录制前硬性检查

不要看到相机画面后立即录制。先在第三个终端设置相同 DDS 环境：

```bash
cd ~/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
```

### 5.1 检查相机频率

```bash
ros2 topic hz /camera/d455/depth/image_rect_raw
ros2 topic hz /camera/d455/color/image_raw
```

正常值应接近 30 Hz。短时间在 28～30 Hz 波动可以接受。

### 5.2 检查动态 TF

```bash
ros2 topic hz /tf
ros2 topic hz /odometry/filtered
```

必须持续输出频率，不能提示话题不存在。

### 5.3 检查完整相机位姿

```bash
ros2 run tf2_ros tf2_echo odom d455_depth_optical_frame
```

正常情况：

- 命令持续输出；
- 机器人静止时数值稳定；
- 机器人移动或旋转时数值连续变化；
- 不应出现数米级瞬间跳变；
- 不应持续出现 future/past extrapolation。

如果这一步失败，不要录制。先修复 TF 或确保 `real_slam.launch.py` 和
D455 使用同一个 Domain 20。

### 5.4 快速检查清单

开始录制前逐项确认：

- [ ] `/tf` 持续发布
- [ ] `/tf_static` 存在
- [ ] `/odometry/filtered` 持续发布
- [ ] 深度图接近 30 Hz
- [ ] RGB 图接近 30 Hz
- [ ] `odom -> d455_depth_optical_frame` 可查询
- [ ] NUC 磁盘剩余空间充足
- [ ] bag 输出目录名尚不存在

---

## 6. 开始录制

创建 bag 总目录：

```bash
mkdir -p ~/WBMM/bags
```

启动录制：

```bash
cd ~/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch tracer_jaka_bringup record_d455_esdf_bag.launch.py \
  output:=bags/d455_rgbd_esdf_02
```

每次必须使用新名字，例如：

```text
d455_rgbd_esdf_02
d455_rgbd_esdf_03
d455_rgbd_esdf_corridor
d455_rgbd_esdf_lab
```

如果目录已经存在，rosbag 不会覆盖。

### 6.1 正常订阅日志

启动日志中至少应看到：

```text
Subscribed to topic '/camera/d455/color/image_raw'
Subscribed to topic '/camera/d455/color/camera_info'
Subscribed to topic '/camera/d455/depth/image_rect_raw'
Subscribed to topic '/camera/d455/depth/camera_info'
Subscribed to topic '/tf'
Subscribed to topic '/tf_static'
Subscribed to topic '/odometry/filtered'
```

如果只有 RGB、深度和 `/tf_static`，没有 `/tf`，立即停止录制。此时通常是：

- `real_slam.launch.py` 没有运行；
- SLAM 和录制终端的 `ROS_DOMAIN_ID` 不一致；
- 录制端仍然使用旧版启动文件；
- DDS 实现不一致。

### 6.2 推荐录制动作

1. 录制开始后静止 5～10 秒；
2. 缓慢原地旋转约 90°；
3. 静止 2 秒；
4. 缓慢直线移动；
5. 避免高速旋转和急停；
6. 回头观察已经经过的障碍物；
7. 录制 1～3 分钟作为第一份测试数据。

慢速运动可以减少运动模糊、深度空洞和 TF 时间不同步带来的误差。

---

## 7. 正确停止录制

录制终端按一次：

```text
Ctrl+C
```

随后可能看到：

```text
Writing remaining messages from cache to the bag
Compressing file: ...db3
```

此时：

- 不要再次按 `Ctrl+C`；
- 不要关闭终端；
- 不要关闭 NUC；
- 不要开始复制 bag；
- 等待命令自然结束并返回 shell 提示符。

RGB-D 数据量较大，zstd 压缩可能需要数分钟。新版启动文件会等待最多
10 分钟，不会像旧版本一样在 15 秒后发送 `SIGKILL`。

正常结束后，不应出现：

```text
escalating to SIGTERM
escalating to SIGKILL
exit code -9
```

---

## 8. 录制完成后的完整性验收

### 8.1 检查文件

```bash
ls -lh ~/WBMM/bags/d455_rgbd_esdf_02
```

必须存在：

```text
metadata.yaml
```

以及一个或多个数据文件，例如：

```text
d455_rgbd_esdf_02_0.db3.zstd
```

根据 rosbag2 版本，也可能保留未压缩的 `.db3`。以 `metadata.yaml` 中记录的
文件为准。

### 8.2 检查 bag 信息

```bash
ros2 bag info ~/WBMM/bags/d455_rgbd_esdf_02
```

必须确认以下话题的 Count 非零：

```text
/camera/d455/color/image_raw
/camera/d455/color/camera_info
/camera/d455/depth/image_rect_raw
/camera/d455/depth/camera_info
/tf
/tf_static
/odometry/filtered
```

如果录制 60 秒、相机为 30 Hz，RGB 和深度图数量应分别接近 1800。

`/tf_static` 只有 1 条是正常的；`/tf` 必须有大量消息。

### 8.3 验收清单

- [ ] `metadata.yaml` 存在
- [ ] `ros2 bag info` 可以正常打开
- [ ] RGB 图 Count 非零
- [ ] 深度图 Count 非零
- [ ] `/tf` Count 非零且数量较多
- [ ] `/tf_static` Count 非零
- [ ] `/odometry/filtered` Count 非零
- [ ] 录制结束没有 `exit code -9`

只有全部满足，才能将 bag 复制到笔记本。

---

## 9. 缺少 metadata.yaml 时恢复

如果数据文件存在，但 `metadata.yaml` 缺失：

```bash
ros2 bag reindex --storage sqlite3 \
  ~/WBMM/bags/d455_rgbd_esdf_02
```

然后检查：

```bash
ros2 bag info --storage sqlite3 \
  ~/WBMM/bags/d455_rgbd_esdf_02
```

`reindex` 只能重建元数据，不能恢复没有录到的 `/tf`、里程计或相机消息。

如果目录同时残留 `.db3` 和不完整的 `.db3.zstd`，优先保留原始 `.db3`，
完成 `reindex` 和 `ros2 bag info` 验证后再处理压缩文件。不要在未确认数据
完整前删除任何文件。

---

## 10. 复制到笔记本

笔记本创建目录：

```bash
mkdir -p /home/a/workspaces/isaac_ros-dev/bags
```

如果可以解析 NUC 主机名 `ras`：

```bash
rsync -avP ras@ras:~/WBMM/bags/d455_rgbd_esdf_02/ \
  /home/a/workspaces/isaac_ros-dev/bags/d455_rgbd_esdf_02/
```

否则使用 NUC IP：

```bash
rsync -avP ras@NUC_IP:~/WBMM/bags/d455_rgbd_esdf_02/ \
  /home/a/workspaces/isaac_ros-dev/bags/d455_rgbd_esdf_02/
```

复制后在笔记本确认：

```bash
ls -lh /home/a/workspaces/isaac_ros-dev/bags/d455_rgbd_esdf_02
```

必须复制整个目录，不能只复制 `.db3` 或 `.db3.zstd`。

---

## 11. Docker 离线回放

进入容器：

```bash
docker exec -it -u admin \
  --workdir /workspaces/isaac_ros-dev \
  isaac_ros_dev-x86_64-container bash
```

构建并加载环境：

```bash
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select my_nvblox_bringup

source install/setup.bash
```

启动离线 RGB-D ESDF：

```bash
export ISAAC_ROS_NVBLOX_PLUGIN_FORCE_FALLBACK_MATERIAL=1

ros2 launch my_nvblox_bringup d455_bag_esdf.launch.py \
  bag:=/workspaces/isaac_ros-dev/bags/d455_rgbd_esdf_02
```

离线入口默认：

```text
ROS_DOMAIN_ID=21
use_sim_time=true
use_color=true
playback rate=0.5
ESDF 显示范围=20×20×3 m
ESDF 近障碍物显示阈值=0.5 m
离线持久地图体素=0.10 m
```

Domain 21 用于隔离实时机器人 Domain 20，防止实时话题和 bag 回放时间戳
混在同一个 nvblox 中。

NUC 录制端和 Docker 回放端的 `/tf` QoS 不相同：

- NUC recorder 使用 Best Effort，保证能够兼容订阅实机 `/tf`；
- Docker player 使用 Reliable，满足 TF2 监听器要求。

不要把 NUC 的录制 QoS 文件直接覆盖 Docker 内的回放 QoS 文件，否则会出现
`incompatible QoS: RELIABILITY`，导致回放后没有 `odom`。

如果处理仍然滞后：

```bash
ros2 launch my_nvblox_bringup d455_bag_esdf.launch.py \
  bag:=/workspaces/isaac_ros-dev/bags/d455_rgbd_esdf_02 \
  rate:=0.25
```

离线 bag 持久地图默认使用 10 cm 体素。与 5 cm 相比，三维体素数量理论上
减少约 8 倍，更适合 4 GB 显存。若使用更大显存并需要更高精度，可显式
设置 `voxel_size:=0.05`，但无界持久地图可能再次耗尽显存。

---

## 12. 常见问题

### 12.1 只有相机话题，没有 `/tf`

原因通常是录制时没有启动 `real_slam.launch.py`，或 Domain 不一致。

重新录制前必须确认：

```bash
ros2 topic hz /tf
ros2 run tf2_ros tf2_echo odom d455_depth_optical_frame
```

### 12.2 结束时出现 `exit code -9`

表示压缩过程中 rosbag 被强制杀死。确认使用的是包含：

```text
sigterm_timeout="600"
```

的新版录制启动文件，并且按一次 `Ctrl+C` 后耐心等待。

### 12.3 `No storage id specified`

先检查：

```bash
ls -lh BAG目录
```

如果没有 `metadata.yaml`：

```bash
ros2 bag reindex --storage sqlite3 BAG目录
```

当前离线启动入口已经显式使用 `--storage sqlite3`。

### 12.4 bag 很大

RGB-D 原始数据约 46 MB/s：

```text
1 分钟未压缩约 2.8 GB
3 分钟未压缩约 8.3 GB
```

实际 zstd 压缩率取决于场景。录制前使用：

```bash
df -h
```

确认磁盘空间，建议使用 SSD。

### 12.5 离线仍然丢图

判断：

- 离线稳定、实时不稳定：主要是网络/DDS问题；
- 0.25 倍速稳定、0.5 倍速不稳定：GPU或可视化处理负载过高；
- 离线仍然跳图：检查 bag 中的 `/tf` 和 `/odometry/filtered`；
- Mesh 保持、ESDF 点云消失：ESDF 查询显示范围不足；
- Mesh 和 ESDF 同时跳变：动态 TF 或里程计可能跳变。

---

## 13. 最短日常操作清单

```bash
# 1. 确认实机定位、D455 和 TF
ros2 topic hz /tf
ros2 topic hz /odometry/filtered
ros2 run tf2_ros tf2_echo odom d455_depth_optical_frame

# 2. 录制
cd ~/WBMM
source install/setup.bash
ros2 launch tracer_jaka_bringup record_d455_esdf_bag.launch.py \
  output:=bags/d455_rgbd_esdf_新编号

# 3. 按一次 Ctrl+C，等待压缩完成

# 4. 验收
test -f bags/d455_rgbd_esdf_新编号/metadata.yaml && echo "metadata OK"
ros2 bag info bags/d455_rgbd_esdf_新编号

# 5. 验收通过后再复制到笔记本
```
