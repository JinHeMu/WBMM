# wbmm_visualization

通用全身移动机械臂（Tracer + JAKA Zu5）可视化：URDF 驱动的整机 mesh、
轨迹时间窗与播放动画。规划器只发布**数据契约**，本包负责渲染，从而消除
REMANI / TA-WBMP / WipePlanner 三套重复的可视化实现。

- 库 `wbmm_visualization_core`：`WholeBodyKinematics`（pinocchio URDF FK）、
  契约解析（`contract.hpp`）、marker 原语（`markers.hpp`）。
- 参考节点 `wbmm_viz_node`：订阅数据契约 → 发布统一 `/wbmm/*` 话题。

## 数据契约（全标准消息，无自定义消息）

| 内容 | 类型 | 默认话题 | QoS |
|---|---|---|---|
| 9D 全身轨迹 | `trajectory_msgs/JointTrajectory`，joint_names 固定 `base_x, base_y, base_yaw, joint_1..6` | `/wbmm/whole_body_trajectory` | transient_local |
| 按点 phase 表 | `std_msgs/String`，格式 `"<t0> <PHASE0>;<t1> <PHASE1>;..."`（t 为轨迹秒，区间 `[t_i, t_{i+1})` 取 `PHASE_i`，末项延到结尾；在轨迹发布后立刻发布） | `/wbmm/phase_schedule` | transient_local |
| 实时 9D 状态 | `sensor_msgs/JointState`（同 9 名 position，frame_id 必须与轨迹同帧） | `/wbmm/live_state` | reliable |
| 实时 phase | `std_msgs/String` | `/wbmm/live_phase` | reliable |

phase 名建议复用共享词汇（免费获得 `phaseColor` 着色）：
`NAVIGATE` / `PRECONTACT_ALIGN` / `PRECONTACT_APPROACH` / `TASK_CONSTRAINED`
（WipePlanner 的 `remani_navigation`、`waiting_navigation`、`wipe_planning`、
`continuous_contact_wiping` 自动映射到对应颜色；`completed` 白 / `failed` 红）。

## 输出话题（`/wbmm/*`）

| 话题 | 类型 | QoS | 说明 |
|---|---|---|---|
| `/wbmm/robot_mesh` | MarkerArray | transient_local | 当前整机 mesh（embedded materials；live 状态新鲜(<0.5 s)时用实时态，否则播放插值态；live phase 着色） |
| `/wbmm/time_segments` | MarkerArray | transient_local | 10 色时间窗：每段 base/EE 线 + `S<i> [a,b] s` 标签 + 段内 mesh 快照 |
| `/wbmm/playback` | MarkerArray | reliable | 播放动画：活动窗(alpha 0.22) + 已播放段(alpha 1.0) + `t=.. s | S<i> | <phase>` 标签 |
| `/wbmm/base_path` | `nav_msgs/Path` | transient_local | 基座路径（`base_path_phase_filter` 非空时只含该 phase 点） |
| `/wbmm/ee_path` | `nav_msgs/Path` | transient_local | 末端路径（库内 pinocchio FK 计算） |

- 所有姿态烘焙进 marker pose（frame_id 取自轨迹 header，回退 `odom`），不查 tf。
- 性能：每个发布点先查 `get_subscription_count() > 0`，无订阅者完全跳过 FK /
  marker 构造；播放定时器仅在 `playback_enabled` 时存在。
- 单数据源约定：每条管线一个轨迹源；不要对同一 `trajectory_topic` 运行两个
  发布者。

## 节点参数

`urdf_file`(必填)、`ee_frame`(tool0)、`trajectory_topic`、
`phase_schedule_topic`、`live_state_topic`、`live_phase_topic`、
`time_segment_duration`(15.0)、`segment_snapshots`(2)、`playback_enabled`(true)、
`playback_rate`(5.0)、`playback_period`(0.10)、`playback_loop`(true)、
`robot_mesh_rate`(30.0)、`publish_base_path`/`publish_ee_path`(true)、
`base_path_phase_filter`("")。

## 独立运行

```bash
ros2 launch wbmm_visualization wbmm_viz.launch.py use_rviz:=true \
  trajectory_topic:=/ta_wbmp/whole_body_trajectory \
  phase_schedule_topic:=/ta_wbmp/phase_schedule
```

或直接 `ros2 run wbmm_visualization wbmm_viz_node --ros-args -p urdf_file:=...`。
RViz 配置见 `rviz/wbmm_viz.rviz`。

## 测试

```bash
colcon build --packages-select wbmm_visualization
colcon test --packages-select wbmm_visualization && colcon test-result --verbose
```
