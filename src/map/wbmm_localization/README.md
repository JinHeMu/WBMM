# wbmm_localization

WBMM 的 Cartographer 建图、保存地图定位和就绪检测。功能节点使用 C++17，
Python 仅组织 ROS launch。按下面三个步骤使用即可。

## 先构建一次

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
colcon build --base-paths src/drivers/sensors/lakibeam1 src/map/wbmm_localization src/bringup \
  --packages-select lakibeam1 wbmm_localization tracer_jaka_bringup --symlink-install
source install/setup.bash
```

每个新终端先执行 `source /home/a/WBMM/install/setup.bash`。

## 1. 启动实机传感器

已有硬件入口在运行时，跳过这一步。

```bash
ros2 launch tracer_jaka_bringup wbmm_hardware_interface.launch.py \
  start_jaka_hardware:=false publish_odom_tf:=false
```

只启动一个 EKF。下面的建图/定位命令默认会启动 EKF；如果已有
`/ekf_filter_node` 或 `/odometry/filtered` 发布者，就在命令后加 `start_ekf:=false`。
新启动会先检查重复地图、里程计和 TF 发布者，有冲突就停止本次启动并提示原因。
不会关闭已运行的节点。

## 2. 新建地图并保存

另开终端，使用一个**未使用的新文件名**：

```bash
ros2 launch tracer_jaka_bringup localization.launch.py \
  localization_backend:=cartographer_mapping \
  ekf_config:=/home/a/WBMM/src/bringup/config/real/ekf.yaml \
  save_state_file:=/home/a/WBMM/maps/cartographer/site_v2.pbstream
```

慢速移动采集环境。完成后按一次 **Ctrl+C**，等待节点退出。
`save_state_file` 的意思就是“结束时把本次地图保存到这个文件”，无需手动调用
`finish_trajectory` 或 `write_state`。正常退出会尝试最终优化和保存。

确认文件存在，并检查终端没有写入失败：

```bash
ls -lh /home/a/WBMM/maps/cartographer/site_v2.pbstream
```

退出时可能出现 `Can't run final optimization ... Trying to finish trajectory`；
如果后面有 `Running final trajectory optimization`、`Optimizing: Done` 并正常退出，
这条 warning 本身不代表优化失败。不要强制杀进程。

已有地图保留，新建图不传 `state_file`。文件名已存在时换一个名字。
成功保存后再次运行同一个建图命令，会因输出文件已存在而停止；这不是 EKF 关不掉。
也可把输出写成 `save_state_file:="/home/a/WBMM/maps/cartographer/site_$(date +%Y%m%d_%H%M%S).pbstream"`，
每次按当前时间生成新文件名。

## 3. 使用刚保存的地图定位

先退出建图，再启动定位；文件路径与上一步相同：

```bash
ros2 launch tracer_jaka_bringup localization.launch.py \
  localization_backend:=cartographer_localization \
  ekf_config:=/home/a/WBMM/src/bringup/config/real/ekf.yaml \
  state_file:=/home/a/WBMM/maps/cartographer/site_v2.pbstream
```

`state_file` 的意思是“加载已有地图来定位”；定位不会向这个文件写入新地图。
若要使用目前已有的地图，改成 `maps/cartographer/site_new_20260930.pbstream` 的绝对路径。
启动后先静止，等待定位稳定再行驶。不要与另一套建图、AMCL 或 EKF 同时运行。

```bash
ros2 topic echo /localization/status
```

看到 `ready: true` 表示最近的激光与地图匹配且全局修正稳定；它不保证环境中的
位置唯一。启动时的 `constraint_builder_2d` score/translation 是匹配调试信息，
不能直接当成机器人实际移动量。

## 地图显示异常或机器人闪动

先检查发布者：

```bash
ros2 topic info /map --verbose
ros2 topic info /odometry/filtered --verbose
```

两者都应只有一个发布者。如果已有 EKF，再次启动加 `start_ekf:=false`。
若已经启动了两个，先退出重复的定位/建图终端，保留一套再重试。

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map odom
```

静止时第一条跳变，检查重复 EKF、底盘 TF 和输入数据；第一条稳定而第二条跳变，
检查地图质量、激光匹配和 IMU。`/map` 单发布者仍出现陌生形状时，核对实际后端
和加载文件；RViz 切回正确的 `/map` 显示或重新打开配置。不要先把跳变滤掉掩盖原因。
当前 RViz 使用不透明黑白栅格、单色激光，默认关闭 ESDF 叠加。

## 实机验证记录（2026-09-30）

已完成实机建图、Ctrl+C 退出保存、加载 `.pbstream` 定位，用户确认定位复测正常。
定位验证使用 `maps/cartographer/site_20260930_162058.pbstream`；记录中 IMU 约 100 Hz、
EKF 里程计约 50 Hz、激光约 29.49 Hz，旧地图匹配分数为 71.4% 和 81.2%。
EKF、Cartographer、地面地图节点及 RViz 均正常退出。

本次定位复测时机器人仍在建图起点，模型保持原位是正常现象。
这次验证不包含从其他位置启动的全局重定位或完整导航。
定位模式现在每约 1 秒保留静止轨迹节点，并缩短全局优化等待；建图模式保持原设置。

## 导航与坐标

TF 链为 `map → cartographer_map → odom → base_footprint`。
Cartographer 跟踪帧保留 `imu_link`；C++ 地面节点从静态外参读取高度，
使 `/map` 的地面 z 为 0，x/y 与原生子地图一致。它不改变匹配结果。
EKF 唯一发布 `odom → base_footprint`；Cartographer 唯一发布
`cartographer_map → odom`；桥接节点唯一发布 `map → cartographer_map`。

OCS2 控制继续用 `odom`，保存地图规划用 `map`，目标通过 TF 转换。
3D ESDF 也要与对外 `map` 的地面原点一致；不能只更换 frame 名称。
`.pbstream` 用于 Cartographer 定位；PGM/YAML 用于二维显示或 AMCL，不能互相替代。
如需导出 PGM/YAML：

```bash
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename /home/a/WBMM/maps/cartographer/site_v2.pbstream \
  -map_filestem /home/a/WBMM/maps/cartographer/site_v2 -resolution 0.05
```

其他后端：`none` 仅 EKF，`slam_toolbox` 建图，`amcl` 使用 YAML 地图；
`auto` 是兼容旧参数的 SLAM Toolbox 入口，想用 Cartographer 请明确指定后端。
直接使用 `wbmm_localization` 的两个 Cartographer launch 不启动硬件或 EKF。
组合规划入口的就绪等待只控制启动，运行后定位丢失不会自动停止机器人。

## 离线测试

```bash
ctest --test-dir build/wbmm_localization --output-on-failure
ctest --test-dir build/lakibeam1 -R 'scan_assembler_tests|scan_udp_ros' --output-on-failure
ctest --test-dir build/tracer_jaka_bringup \
  -R 'localization_backends|localized_real_safety_gate|common_launch' --output-on-failure
```

离线测试覆盖消息、TF、重复发布者和启动配置，定位精度及静止噪声需实机验证。
完整导航部署还需要安装规划、碰撞和可视化等依赖包。
