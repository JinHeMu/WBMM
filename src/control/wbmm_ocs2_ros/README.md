> CURRENT（2026-09-17）：bringup launch 已扁平化。`ocs2_sim.launch.py` /
> `ocs2_real.launch.py` 已删除；统一使用 `ocs2.launch.py` 算法入口，并由
> `wbmm_hardware_interface.launch.py` 或 `mujoco_hardware_interface.launch.py`
> 提供反馈/命令接口。本文下方旧命令仅作历史参考。

# wbmm_ocs2_ros

export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix tracer_jaka_gazebo)/share

从 2026-09 起，本包只保留 ROS 适配层（MPC/MRT 节点、话题、TF、参数、
可视化和 REMANI→OCS2 参考桥）；移动机械臂的 OCS2 问题定义
（9D/8D 状态输入、差速动力学、代价、约束、rollout、initializer）
已经迁移到项目自己的 `wbmm_ocs2` 包。本包不再直接依赖或修改
vendor 中的 `ocs2_mobile_manipulator`。

Tracer + JAKA 的 OCS2 全身 MPC/MRT 算法包。本包只安装节点、配置和可复用资源；
MuJoCo、实机驱动、定位、REMANI 与任务的组合入口已统一移动到
`tracer_jaka_bringup`，见 `docs/launch_ownership.md`。下文遗留的 Gazebo 说明仅作为
历史背景，不代表当前推荐后端。

## 1. 总体架构

```
                        ┌─────────────────────────────┐
                        │    tracer_jaka_gazebo       │
                        │  (Gazebo + ros2_control)    │
                        └───────────┬─────────────────┘
                                    │
            /wheel/odometry │  /joint_states
                                    │
                         ┌──────────▼──────────┐
                         │      wbmm_mrt       │
                         │   (本包) Bridge     │
                         └──┬───────────┬──────┘
       SystemObservation    │           │  optimal state/input
                            │           │
                            ▼           │
                   ┌────────────────┐   │
                   │    wbmm_mpc    │◄──┘ updatePolicy
                   │  (本包) SLQ    │
                   └────────▲───────┘
                            │ TargetTrajectories
                            │
              ┌─────────────┴────────────┐
              │     wbmm_target_node     │
              │  (本包) PoseStamped→TT   │
              └─────────────▲────────────┘
                            │
                       /target_pose  (RViz "2D Goal Pose")
```

| 节点 | 职责 |
|------|------|
| `wbmm_mpc_node`     | OCS2 SLQ 求解器，监听 observation，发布 policy |
| `wbmm_mrt_node`     | 把 odom + joint_states 拼成 SystemObservation；把 policy 转成底盘速度和机械臂位置命令 |
| `wbmm_target_node`  | 把 RViz 点的目标位姿转成 OCS2 `TargetTrajectories` |

## 2. 状态/输入向量约定

`manipulatorModelType = 1` (`wheelBasedMobileManipulator`):

\[
x = [\,x_b,\; y_b,\; \theta_b,\; q_1,\; \dots,\; q_6\,]^\top \in \mathbb{R}^{9}
\]

\[
u = [\,v,\; \omega,\; \dot q_1,\; \dots,\; \dot q_6\,]^\top \in \mathbb{R}^{8}
\]

当前 `src/bringup/config/sim/task.info` 启用全身轨迹代价，因此 `TargetTrajectories.desiredState`
编码 9D 全身状态：

\[
x_\text{des} = [\,x_b,\; y_b,\; \theta_b,\; q_1,\; \dots,\; q_6\,]^\top
\]

只有在 `endEffector.activate=true` 且 `wholeBodyTracking.activate=false` 时，
同一字段才使用 7D 末端位姿；两种语义不得同时启用。

## 3. 编译

依赖 (按 ROS 2 OCS2 移植版安装, 推荐 [legubiao/ocs2_ros2](https://github.com/legubiao/ocs2_ros2)):

```bash
cd ~/WBMM/src
# 把本包放到 src 下:
# wbmm_ocs2_ros/
# src/sim/tracer_jaka_mujoco/
# src/vendor/ocs2_ros2/   <- 你的 OCS2 移植包
cd ..
colcon build --symlink-install --packages-up-to wbmm_ocs2_ros
source install/setup.bash
```

## 4. 必做事项 (踩坑提醒)

### 4.1 在 `src/bringup/config/sim/task.info` 里填轮子 joint 名

`manipulatorModelType=1` 时 OCS2 不会建模轮子, 必须把所有车轮 / 转向 joint 列在 `removeJoints` 里, 否则 Pinocchio 会把它们当成 "arm DoF":

```
removeJoints {
  [0] "tracer_left_wheel"
  [1] "tracer_right_wheel"
  ; ...
}
```

具体名字打开你 `tracer_jaka.urdf.xacro` 看 transmission 那一段就是。

### 4.2 自检 `selfCollision.collisionLinkPairs` 里的 link 名

`src/bringup/config/sim/task.info` 里写了 `base_link / urbase_base_link / Link_2..6 / gripper_base_link`, 这些必须在你 URDF 里真的存在 (而且要带 `<collision>` 几何), 否则 OCS2 会段错误。可以用:

```bash
xacro tracer_jaka.urdf.xacro | grep -E '<link name='
```

把不存在的对从 `src/bringup/config/sim/task.info` 删掉。

### 4.3 关掉原 `gazebo.launch.py` 的双 RViz

目前你的 `gazebo.launch.py` 里 `rviz_node` 写的是 `condition=None`, 这导致 `use_rviz` 形同虚设。要么修一下:

```python
from launch.conditions import IfCondition
...
condition=IfCondition(use_rviz),
```

要么直接接受双 RViz, 在我提供的 `ocs2_sim.launch.py` 里我已经给原 launch 传了 `use_rviz:=false`, 但只有你修了上面那行才会真的关掉。

### 4.4 OCS2 codegen

第一次启动会调用 CppAD 自动生成微分 (`recompileLibraries=true`)。MPC 与 MRT
分别使用 `lib_folder/mpc` 和 `lib_folder/mrt`，避免两个进程并发改写同一动态库；
根目录默认是 `/tmp/wbmm_ocs2/auto_generated`。如果修改了 `src/bringup/config/sim/task.info` 中影响
动力学的参数，应使用新的生成目录或清理对应缓存后重新生成。

## 5. 跑起来

### 一键全部启动

```bash
ros2 launch tracer_jaka_bringup ocs2_sim.launch.py
```

该入口默认同时启动 MuJoCo、wheel odom + IMU EKF、slam_toolbox、
REMANI Planner、REMANI→OCS2 Bridge、OCS2 MPC/MRT 和 RViz2。定位与 TF
的唯一发布关系为：

```text
slam_toolbox:       map -> odom
robot_localization: odom -> base_footprint
robot_state_publisher:
                    base_footprint -> robot and sensor links
```

MuJoCo 原始里程计 `/wheel/odometry` 和 `/imu/data` 进入 EKF，MRT 默认读取
融合结果 `/odometry/filtered`。RViz2 的 Fixed Frame 为 `map`，并显示 `/map`
和 `/scan`。

REMANI 同样读取 `/odometry/filtered` 和 `/joint_states`，在 RViz2 使用
`2D Goal Pose` 发布 `/goal_pose` 后，数据链为：

```text
/goal_pose
    -> REMANI front-end + MINCO trajectory optimization
    -> /planning/trajectory
    -> remani_to_ocs2_reference_bridge
    -> /mobile_manipulator_mpc_target
    -> OCS2 MPC / MRT
```

默认加载 MuJoCo 场景生成的静态 ESDF，并使用 `x=2.0 m` 的场景到 odom
平移。替换 ESDF 时：

```bash
ros2 launch tracer_jaka_bringup ocs2_sim.launch.py \
  remani_static_esdf_file:=/absolute/path/to/map.npz \
  remani_static_esdf_offset_x:=0.0 \
  remani_static_esdf_offset_y:=0.0 \
  remani_static_esdf_offset_z:=0.0
```

不需要二维建图、只想运行定位和 MPC 时：

```bash
ros2 launch tracer_jaka_bringup ocs2_sim.launch.py start_slam:=false
```

不启动 REMANI、改用其他 OCS2 目标发布器时：

```bash
ros2 launch tracer_jaka_bringup ocs2_sim.launch.py start_remani:=false
```

CSV 目标与 REMANI 互斥。使用 CSV 时必须关闭 REMANI：

```bash
ros2 launch tracer_jaka_bringup ocs2_sim.launch.py \
  start_remani:=false use_csv_target:=true
```

默认 CSV 是 8 cm 前进的 MuJoCo 冒烟轨迹
`config/mujoco_smoke_trajectory.csv`；也可显式指定：

```bash
ros2 launch tracer_jaka_bringup ocs2_sim.launch.py \
  start_remani:=false use_csv_target:=true \
  trajectory_csv:=/absolute/path/to/trajectory.csv
```

无界面运行：

```bash
ros2 launch tracer_jaka_bringup ocs2_sim.launch.py \
  viewer:=false use_rviz:=false
```

### OCS2 ESDF 碰撞端到端验证

当前仓库提供的验证入口：

```bash
cd /home/a/WBMM
source install/setup.bash
ros2 launch tracer_jaka_bringup ocs2_esdf_validation.launch.py \
  viewer:=false use_rviz:=false hardware_write:=false
```

默认组合：

| 项目 | 默认值 |
|---|---|
| MuJoCo scene | `room`，与 `tracer_jaka_zu5_scene_esdf.npz` 对齐 |
| OCS2 task | `tracer_jaka_bringup/config/sim/task_esdf.info` |
| `environmentCollision` | `activate=true`, `backend=esdf` |
| ESDF NPZ | `grid_map/maps/tracer_jaka_zu5_scene_esdf.npz` |
| OCS2 `world_frame` | `odom`，必须与 NPZ `frame_id` 一致 |

`hardware_write:=false` 时只做 MPC/MRT dry-run；设为 `true` 后 MRT 会把
底盘和机械臂命令发给 MuJoCo。启动后在 OCS2 日志中应看到：

```text
#### backend: esdf
[INFO] [wbmm_mrt_node]: Publishing base commands as Twist on ...
[INFO] [wbmm_mrt_node]: Got first MPC policy. Entering MRT loop ...
```

替换 ESDF 时显式传入 `esdf_file`：

```bash
ros2 launch tracer_jaka_bringup ocs2_esdf_validation.launch.py \
  esdf_file:=/absolute/path/site_remani.npz \
  world_frame:=odom
```

当前 C++ `WbmmInterface` 不会做隐式 TF 或刚体变换：如果 NPZ 的
`frame_id` 与 OCS2 `world_frame` 不一致，会在启动时直接报错。真实
nvblox bag 导出的 ESDF 通常是 `map` frame；若要直接用于 OCS2 避障，
需要先在导出后转换到 `odom`，或者把 OCS2 状态、目标、力控和命令链
统一切换到同一个 frame，并确认 TF 与坐标原点确实一致。

纯 ESDF 可视化仍可用 `grid_map/esdf_rviz_publisher` 或
`my_nvblox_bringup/esdf_visualizer`，但它们不参与上述 OCS2 碰撞约束。

### 空场景 ESDF 全身导航 + EE 跟踪

整合入口：

```bash
cd /home/a/WBMM
source install/setup.bash
ros2 launch tracer_jaka_bringup remani_tracking.launch.py
```

默认组合：

| 项目 | 默认值 |
|---|---|
| MuJoCo scene | `esdf_validation`（机器人 + 地面） |
| REMANI/OCS2 ESDF | `/home/a/WBMM/maps/map1/site_remani.npz` |
| OCS2 task | `config/sim/task_esdf_tracking.info`（`esdf.frame=map`） |
| REMANI 配置 | `config/sim/remani_tracking.yaml` |
| world/planner frame | `map` |
| map→odom | 默认发布 identity 静态 TF，用于空场景 odom 别名 |
| RViz | 默认启动，Fixed Frame `map` |
| MuJoCo command output | `hardware_write=true` |

工作流：

1. 在 RViz 中选择 **2D Goal Pose**，向 `/goal_pose` 发一个平面目标。
   `remani_phase_bridge.py` 收到目标后把 OCS2 任务阶段切到
   `Navigation`（0），REMANI 负责全身导航。
2. 底盘到达目标并保持约 1 s 后，`remani_phase_bridge` 保持在
   `Navigation`（0），只标记导航完成，不再切到中间 `Transition`。
   这样不会出现“底盘继续追全身参考、机械臂同时追旧末端目标”的混合运动。
3. 在 RViz 的 **EE Target Marker**（InteractiveMarkers 命名空间
   `target_marker`）上右键选择 **Send target**，`WbmmTargetNode` 会发布
   7D 末端目标；`remani_phase_bridge` 收到后直接切到 `Execution`（2），
   OCS2 由末端跟踪 cost 主导。
4. 再次发送新的 2D Goal 时，阶段自动回到 `Navigation`（0），并清除上一次
   的末端目标；到达新目标后不会自动追回旧 EE target，必须重新发送 marker。

验证日志：

```text
[remani_phase_bridge]: Task phase -> 0 (new navigation goal)
[remani_phase_bridge]: Base reached goal ... waiting for EE target
[remani_phase_bridge]: Task phase -> 2 (end-effector target received)
```

说明：
- 为了让求解器稳定，演示 task 暂时关闭了
  `armManipulability.activate`，并将 `tracking_error_replan_enabled` 设为
  `false`，避免导航结束后 REMANI 与 OCS2 末端跟踪争抢 reference。
- `WbmmReferenceManager::getTargetTrajectories()` 始终返回 9D 全身参考，
  以保证 OCS2 solver/MPC 缓冲区维度一致；双参考 cost 仍通过
  `PhaseWeightedStateCost` 分别读取 9D/7D 目标。
- 该入口把 MuJoCo 的 `odom` 数值通过 identity `map->odom` 当作 `map`
  使用，适合空场景虚拟 ESDF 演示。接入真实定位时，应设置
  `publish_map_odom_tf:=false`，由 AMCL/SLAM 提供真实 `map->odom`。

### 低桌穿越场景

当前 `models/scene.xml`、`src/bringup/config/sim/task.info` 和
`tracer_jaka_zu5_scene_esdf.npz` 使用同一套低桌环境。尺寸来自
`tracer_jaka_zu5.urdf` 的碰撞几何：

| 项目 | 尺寸 / 高度 |
|---|---:|
| 底盘碰撞包络 | 约 `0.74 × 0.68 × 0.39 m` |
| 机械臂初始 `home` 总高度 | 约 `1.52 m` |
| 可穿越的折叠姿态总高度 | 约 `1.00 m` |
| 桌面下沿 | `1.10 m` |
| 桌腿中央净宽 | `1.34 m` |
| 桌子中心 | MuJoCo `(0, 0)` / odom `(2, 0)` |

启动后，机器人位于桌子西侧、机械臂处于不能直接通过的高姿态。在 RViz
选择 **2D Goal Pose**，把目标放在桌子另一侧约 `(3.8, 0)`，箭头朝向
`+x`。REMANI 的预期行为是：

```text
保持高姿态出发 -> 机械臂折叠 -> 从两排桌腿中间穿过 -> 机械臂恢复目标姿态
```

RViz 可以继续使用 `map` 作为 Fixed Frame；规划器会把 `/goal_pose`
自动转换到 `odom`。如果目标附近过于靠近东墙，可将目标改为
`(3.6, 0)`。

场景尺寸变化后，需要重新生成 REMANI 使用的静态 ESDF：

```bash
ros2 run grid_map mjcf_to_esdf \
  --xml "$(ros2 pkg prefix tracer_jaka_mujoco)/share/tracer_jaka_mujoco/models/scene.xml" \
  --output "$(ros2 pkg prefix grid_map)/share/grid_map/maps/tracer_jaka_zu5_scene_esdf.npz" \
  --voxel-size 0.02 \
  --bounds-min -1 -3 0 \
  --bounds-max 5 3 1.8
```

### 分两个终端 (调试时方便看 log)

```bash
# T1
ros2 launch tracer_jaka_gazebo gazebo.launch.py use_rviz:=false

# T2 (等机器人 spawn 完, 控制器都加载完)
ros2 launch wbmm_ocs2_ros ocs2_only.launch.py
```

## 6. 给 MPC 发目标

### 用 RViz 的 "2D Goal Pose" 工具

在 `wbmm_ocs2_ros.rviz` 里 `SetGoal` 工具的 topic 已经设成
`/goal_pose`，直接点击拖一下即可交给 REMANI。注意这里设置的是移动底盘
目标，并保持点击时的机械臂构型作为最终构型。

### 用脚本

```bash
ros2 run wbmm_ocs2_ros send_target.py 0.8 0.0 0.6 0 0 0
#                                          x   y   z   r p y
```

参数依次为 `x y z roll pitch yaw` (rad, frame=`odom`)。

### 验证

```bash
# observation 应该 ~100 Hz
ros2 topic hz /mobile_manipulator_mpc_observation

# policy 应该 ~100 Hz (mpcDesiredFrequency)
ros2 topic hz /mobile_manipulator_mpc_policy

# 输出
ros2 topic echo /diff_drive_controller/cmd_vel
ros2 topic echo /arm_controller/commands
```

## 7. 关键参数 (在 `launch/ocs2_sim.launch.py` 里改)

| 参数 | 默认 | 说明 |
|------|------|------|
| `mrt_loop_rate`     | 100 Hz | MRT 控制环频率 (= cmd_vel 发布频率) |
| `traj_horizon`      | 0.1 s  | 给 JTC 的轨迹采样时窗 |
| `traj_num_points`   | 5      | 时窗内采样点数 |
| `odom_topic`        | `/wheel/odometry` | 底盘位姿来源 |
| `joint_state_topic` | `/joint_states`               | 关节读数来源 |
| `base_cmd_topic`    | `/diff_drive_controller/cmd_vel` | TwistStamped |
| `arm_cmd_topic`     | `/arm_controller/commands` | MuJoCo JTC 的 `Float64MultiArray` 位置命令入口 |

## 8. 调权重

效果不好时常改这几个 (`src/bringup/config/sim/task.info`):

- `endEffector.muPosition / muOrientation` - 越大跟踪越凶, 但容易颤
- `inputCost.R.base.wheelBasedMobileManipulator.scaling` - 越大 base 越懒得动 (会让机械臂自己够)
- `inputCost.R.arm.scaling` - 越大手臂越懒得动 (会让 base 去够)

## 9. 文件清单

```
wbmm_ocs2_ros/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── ../../bringup/config/sim/task.info  # 部署 OCS2 task 配置
├── launch/
│   ├── ocs2_sim.launch.py       # Gazebo + OCS2 全套
│   └── ocs2_only.launch.py      # 仅 OCS2 (Gazebo 已在跑)
├── rviz/
│   └── wbmm_ocs2_ros.rviz
├── scripts/
│   └── send_target.py           # 命令行发目标
└── src/
    ├── WbmmMpcNode.cpp           # MPC 求解器
    ├── WbmmMrtNode.cpp           # MRT 执行与安全门
    └── WbmmTargetNode.cpp        # PoseStamped → TargetTrajectories
```

## 10. 跟你现有手动指令的等价关系

| 之前你手动发的 | OCS2 接管后由谁发 |
|----|----|
| `/diff_drive_controller/cmd_vel` (TwistStamped) | `wbmm_mrt_node` 自动发 |
| `/arm_controller/commands`                      | `wbmm_mrt_node` 自动发 |
| (新增) 末端目标位姿                              | 你点 RViz / 调 `send_target.py` |

也就是说, 一旦 OCS2 跑起来, 你就 **不要再手动 pub `cmd_vel` / `joint_trajectory`** 了, 否则会和 MRT 抢 topic。
