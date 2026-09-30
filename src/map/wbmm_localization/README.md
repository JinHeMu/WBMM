# wbmm_localization

原 `tracer_jaka_localization`，提供 AMCL、Cartographer 2D 建图/纯定位及定位就绪检查。
硬件驱动和 EKF 由 bringup 启动，本包不控制机器人。
就绪检查、等待节点和检测算法均使用 C++17；Python 仅用于 ROS 启动文件及其辅助逻辑，Cartographer 配置使用 Lua。

## 准备

```bash
sudo apt install ros-humble-cartographer-ros ros-humble-cartographer-ros-msgs
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
colcon build --packages-select wbmm_localization tracer_jaka_bringup --symlink-install
source install/setup.bash
```

实机先启动传感器；已有硬件入口运行时不要重复启动：

```bash
ros2 launch tracer_jaka_bringup wbmm_hardware_interface.launch.py \
  start_jaka_hardware:=false publish_odom_tf:=false
```

默认输入为 `/scan`（`laser_link`）、`/imu/data`（`imu_link`）、
`/wheel/odometry`。EKF 输出 `/odometry/filtered`，唯一发布
`odom → base_footprint`；选定后端唯一发布 `map → odom`。
Cartographer 默认需要角速度、含重力的加速度及正确的传感器时间戳/外参。

## Cartographer 建图

```bash
mkdir -p /home/a/WBMM/maps/cartographer
ros2 launch tracer_jaka_bringup localization.launch.py \
  localization_backend:=cartographer_mapping \
  ekf_config:=/home/a/WBMM/src/bringup/config/real/ekf.yaml \
  save_state_file:=/home/a/WBMM/maps/cartographer/site.pbstream
```

移动机器人采集完整环境，结束后正常 Ctrl+C，等待最终优化和 `.pbstream` 写入，
不要强制杀进程。也可在运行中调用 `/write_state` 保存快照：

```bash
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '/home/a/WBMM/maps/cartographer/site.pbstream', include_unfinished_submaps: true}"
```

导出对应的显示地图：

```bash
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename /home/a/WBMM/maps/cartographer/site.pbstream \
  -map_filestem /home/a/WBMM/maps/cartographer/site -resolution 0.05
```

## Cartographer 纯定位

```bash
ros2 launch tracer_jaka_bringup localization.launch.py \
  localization_backend:=cartographer_localization \
  ekf_config:=/home/a/WBMM/src/bringup/config/real/ekf.yaml \
  state_file:=/home/a/WBMM/maps/cartographer/site.pbstream
```

加载冻结地图，自动开启新的定位轨迹，不要求人为提供起始位置。
`/map` 只包含冻结地图；当前轨迹仅保留少量子地图。
任意位置启动的全局匹配需要时间和足够的环境特征；对称环境可能误匹配。
PGM/YAML 无法替代 `.pbstream`，建图与定位的子地图分辨率应保持一致。

## 在 bringup 中使用

`localization.launch.py` 的 `localization_backend` 支持：

| 值 | 行为 |
|---|---|
| `auto`（默认） | 兼容旧参数：`start_slam=true` 建图，否则仅 EKF |
| `none` | 仅 EKF |
| `slam_toolbox` | 原 SLAM Toolbox 建图 |
| `amcl` | 保存的 YAML 地图定位，使用 `initial_x/y/yaw` |
| `cartographer_mapping` | Cartographer 建图 |
| `cartographer_localization` | 加载 `.pbstream` 纯定位 |

显式后端优先于旧 `start_slam`，同一次启动只运行一个全局定位/建图后端。
已有 EKF 时加 `start_ekf:=false`；仿真加 `use_sim_time:=true`。
只使用本包入口时可运行 `wbmm_localization cartographer_mapping.launch.py` 或
`cartographer_localization.launch.py`，但需要另外提供 EKF 和传感器。

已有硬件后端下，定位、规划与 OCS2 的组合入口：

```bash
ros2 launch tracer_jaka_bringup remani_mpc_localized.launch.py \
  localization_backend:=cartographer_localization \
  state_file:=/home/a/WBMM/maps/cartographer/site.pbstream \
  static_esdf_file:=/absolute/path/to/aligned_esdf.npz \
  hardware_write:=false
```

该兼容入口现在使用 WBMM 原生规划器。`wbmm.launch.py` 也支持同名后端参数，
配合 `start_localization:=true start_planning:=true`：保存地图模式自动将
规划里程计转换到 `map`，控制器继续消费原有 `odom` 反馈。

## 就绪状态与配置

```bash
ros2 topic echo /localization/ready
ros2 topic echo /localization/status
```

`ready` 是 Bool；`status` 是含时间戳、后端、状态和原因的 JSON 字符串。
检查扫描、里程计、Cartographer IMU 和 `map → odom` TF 是否新鲜，
扫描端点与地图的匹配比例，以及全局修正是否连续稳定。
默认稳定 3 秒、至少 5 帧；数据过期、匹配失败或修正跳变会撤销就绪。
启动超时仍继续检测，状态可以恢复。

组合入口默认等待就绪再启动规划器及轨迹桥；超时不启动。
`localization_timeout:=30.0` 可调整等待时间；
`wait_for_localization:=false` 可显式跳过。
这是**一次性的规划启动等待**，运行后定位丢失会发布状态，但不会自动停止
已启动的规划器、OCS2 或硬件。新数据判断应使用有时间戳的 `status`，
不要把可能残留的 Bool 或 EKF 协方差当成全局定位置信度。

- `config/cartographer_mapping.lua`：建图、传感器和 TF 配置。
- `config/cartographer_localization.lua`：纯定位和全局搜索参数。
- `config/readiness.yaml`：匹配、新鲜度和稳定时间阈值。
- 自定义配置可通过 `cartographer_config:=...`、`readiness_config:=...` 指定。

参数是待实机调试的初始值，就绪判据不保证定位全局唯一。
接入规划前必须确认 `.pbstream`、显示地图和 ESDF 的位置、朝向一致；
只将它们命名为 `map` 不代表已经配准。包更名、构建和合成数据测试不能代替
真实地图上的任意位置启动与定位精度验证。
