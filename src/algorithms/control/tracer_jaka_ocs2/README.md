# tracer_jaka_ocs2

export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix tracer_jaka_gazebo)/share

OCS2 全身 MPC 与 Tracer + JAKA 移动机械臂 Gazebo Harmonic 仿真的集成功能包。

## 1. 总体架构

```
                        ┌─────────────────────────────┐
                        │    tracer_jaka_gazebo       │
                        │  (Gazebo + ros2_control)    │
                        └───────────┬─────────────────┘
                                    │
            /diff_drive_controller/odom │  /joint_states
                                    │
                         ┌──────────▼──────────┐
                         │  tracer_jaka_mrt    │
                         │   (本包) Bridge     │
                         └──┬───────────┬──────┘
       SystemObservation    │           │  optimal state/input
                            │           │
                            ▼           │
                   ┌────────────────┐   │
                   │tracer_jaka_mpc │◄──┘ updatePolicy
                   │  (本包) SLQ    │
                   └────────▲───────┘
                            │ TargetTrajectories
                            │
              ┌─────────────┴────────────┐
              │  tracer_jaka_target_node │
              │  (本包) PoseStamped→TT   │
              └─────────────▲────────────┘
                            │
                       /target_pose  (RViz "2D Goal Pose")
```

| 节点 | 职责 |
|------|------|
| `tracer_jaka_mpc_node`     | OCS2 SLQ 求解器, 监听 observation, 发布 policy |
| `tracer_jaka_mrt_node`     | 把 odom + joint_states 拼成 SystemObservation; 把 policy 拆成 `TwistStamped` (cmd_vel) + `JointTrajectory` |
| `tracer_jaka_target_node`  | 把 RViz 点的目标位姿转成 OCS2 `TargetTrajectories` |

## 2. 状态/输入向量约定

`manipulatorModelType = 1` (`wheelBasedMobileManipulator`):

\[
x = [\,x_b,\; y_b,\; \theta_b,\; q_1,\; \dots,\; q_6\,]^\top \in \mathbb{R}^{9}
\]

\[
u = [\,v,\; \omega,\; \dot q_1,\; \dots,\; \dot q_6\,]^\top \in \mathbb{R}^{8}
\]

`TargetTrajectories` 的 desiredState 编码末端目标位姿 (size 7):

\[
x_\text{des} = [\,p_x,\; p_y,\; p_z,\; q_x,\; q_y,\; q_z,\; q_w\,]^\top
\]

## 3. 编译

依赖 (按 ROS 2 OCS2 移植版安装, 推荐 [legubiao/ocs2_ros2](https://github.com/legubiao/ocs2_ros2)):

```bash
cd ~/WBMM/src
# 把本包放到 src 下:
# tracer_jaka_ocs2/
# src/simulation/tracer_jaka_mujoco/
# src/vendor/ocs2_ros2/   <- 你的 OCS2 移植包
cd ..
colcon build --symlink-install --packages-up-to tracer_jaka_ocs2
source install/setup.bash
```

## 4. 必做事项 (踩坑提醒)

### 4.1 在 `config/task.info` 里填轮子 joint 名

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

`task.info` 里写了 `base_link / urbase_base_link / Link_2..6 / gripper_base_link`, 这些必须在你 URDF 里真的存在 (而且要带 `<collision>` 几何), 否则 OCS2 会段错误。可以用:

```bash
xacro tracer_jaka.urdf.xacro | grep -E '<link name='
```

把不存在的对从 `task.info` 删掉。

### 4.3 关掉原 `gazebo.launch.py` 的双 RViz

目前你的 `gazebo.launch.py` 里 `rviz_node` 写的是 `condition=None`, 这导致 `use_rviz` 形同虚设。要么修一下:

```python
from launch.conditions import IfCondition
...
condition=IfCondition(use_rviz),
```

要么直接接受双 RViz, 在我提供的 `ocs2_sim.launch.py` 里我已经给原 launch 传了 `use_rviz:=false`, 但只有你修了上面那行才会真的关掉。

### 4.4 OCS2 codegen

第一次启动会调用 CppAD 自动生成微分 (`recompileLibraries=true`), 大概要 30s ~ 2min。生成结果在 `lib_folder` (默认 `/tmp/ocs2_tracer_jaka/auto_generated`)。第二次起就秒启。如果改了 task.info 里影响动力学的参数 (e.g. baseFrame, eeFrame), 把这个目录删了重来。

## 5. 跑起来

### 一键全部启动

```bash
ros2 launch tracer_jaka_ocs2 ocs2_sim.launch.py
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
ros2 launch tracer_jaka_ocs2 ocs2_sim.launch.py \
  remani_static_esdf_file:=/absolute/path/to/map.npz \
  remani_static_esdf_offset_x:=0.0 \
  remani_static_esdf_offset_y:=0.0 \
  remani_static_esdf_offset_z:=0.0
```

不需要二维建图、只想运行定位和 MPC 时：

```bash
ros2 launch tracer_jaka_ocs2 ocs2_sim.launch.py start_slam:=false
```

不启动 REMANI、改用其他 OCS2 目标发布器时：

```bash
ros2 launch tracer_jaka_ocs2 ocs2_sim.launch.py start_remani:=false
```

CSV 目标与 REMANI 互斥。使用 CSV 时必须关闭 REMANI：

```bash
ros2 launch tracer_jaka_ocs2 ocs2_sim.launch.py \
  start_remani:=false use_csv_target:=true
```

无界面运行：

```bash
ros2 launch tracer_jaka_ocs2 ocs2_sim.launch.py \
  viewer:=false use_rviz:=false
```

### 使用真实 bag 导出的 ESDF 做纯地图验证

先在 Isaac ROS 容器中完成 bag 回放和自动导出：

```bash
docker exec -it -u admin --workdir /workspaces/isaac_ros-dev \
  isaac_ros_dev-x86_64-container bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch my_nvblox_bringup d455_bag_esdf.launch.py \
  bag:=/workspaces/isaac_ros-dev/bags/d455_rgbd_esdf_02
```

回放期间 nvblox RViz 显示 RGB-D mesh、3D ESDF 和 bag 中的 `/map`。
回放结束后，REMANI 使用的文件默认是：

```text
/home/a/workspaces/isaac_ros-dev/bag_export/d455_bag_remani_esdf.npz
```

然后在宿主机启动纯 ESDF 验证：

```bash
cd /home/a/WBMM
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch tracer_jaka_ocs2 ocs2_esdf_validation.launch.py
```

该入口有意隔离三种环境来源：

| 环境来源 | 状态 | 用途 |
|---|---|---|
| MuJoCo `scene_esdf_validation.xml` | 仅机器人和地面 | 保留动力学接触，不放桌子、墙和障碍物 |
| OCS2 `task_esdf_only.info` | `environmentCollision.activate=false` | 不再使用 info 中手工障碍物 |
| nvblox 导出的 NPZ | 启用 | REMANI 搜索、轨迹优化和碰撞检测的唯一外部环境 |

验证入口默认通过 map_server 加载
`/home/a/WBMM/src/bringup/tracer_jaka_bringup/maps/factory_map.yaml`。RViz 使用 `odom` 作为 Fixed Frame，
并显示保存的 `/map`、
`/esdf_cloud`、`/esdf_occ2d` 和机器人模型。`/esdf_cloud` 只显示已观测且靠近障碍物的
体素；`/esdf_occ2d` 是指定高度带内的 2D ESDF 占据投影。使用 RViz 的
**2D Goal Pose** 向 `/goal_pose` 发目标即可触发：

```text
真实场景 ESDF -> REMANI -> /planning/trajectory
              -> REMANI-to-OCS2 bridge -> MPC/MRT -> MuJoCo 机器人
```

自定义文件或降低显示量：

```bash
ros2 launch tracer_jaka_ocs2 ocs2_esdf_validation.launch.py \
  esdf_file:=/absolute/path/site_remani.npz \
  map2d_yaml:=/absolute/path/site_2d.yaml \
  esdf_display_distance:=0.35 esdf_display_stride:=3
```

这里要求导出 ESDF、仿真初始位姿和实际录包时的 `odom` 原点一致；若不
一致，应先做刚体坐标变换后再导出，不能只在 RViz 中旋转显示。

联合启动后的检查命令：

```bash
ros2 topic hz /wheel/odometry
ros2 topic hz /imu/data
ros2 topic hz /odometry/filtered
ros2 topic hz /scan
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

### 低桌穿越场景

当前 `models/scene.xml`、`config/task.info` 和
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
  --bounds-min -3 -3 0 \
  --bounds-max 3 3 1.8
```

### 分两个终端 (调试时方便看 log)

```bash
# T1
ros2 launch tracer_jaka_gazebo gazebo.launch.py use_rviz:=false

# T2 (等机器人 spawn 完, 控制器都加载完)
ros2 launch tracer_jaka_ocs2 ocs2_only.launch.py
```

## 6. 给 MPC 发目标

### 用 RViz 的 "2D Goal Pose" 工具

在 `tracer_jaka_ocs2.rviz` 里 `SetGoal` 工具的 topic 已经设成
`/goal_pose`，直接点击拖一下即可交给 REMANI。注意这里设置的是移动底盘
目标，并保持点击时的机械臂构型作为最终构型。

### 用脚本

```bash
ros2 run tracer_jaka_ocs2 send_target.py 0.8 0.0 0.6 0 0 0
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
| `odom_topic`        | `/diff_drive_controller/odom` | 底盘位姿来源 |
| `joint_state_topic` | `/joint_states`               | 关节读数来源 |
| `base_cmd_topic`    | `/diff_drive_controller/cmd_vel` | TwistStamped |
| `arm_cmd_topic`     | `/arm_controller/commands` | MuJoCo JTC 的 `Float64MultiArray` 位置命令入口 |

## 8. 调权重

效果不好时常改这几个 (`config/task.info`):

- `endEffector.muPosition / muOrientation` - 越大跟踪越凶, 但容易颤
- `inputCost.R.base.wheelBasedMobileManipulator.scaling` - 越大 base 越懒得动 (会让机械臂自己够)
- `inputCost.R.arm.scaling` - 越大手臂越懒得动 (会让 base 去够)

## 9. 文件清单

```
tracer_jaka_ocs2/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── task.info                # OCS2 配置 (你给的, 加了 removeJoints 注释)
├── launch/
│   ├── ocs2_sim.launch.py       # Gazebo + OCS2 全套
│   └── ocs2_only.launch.py      # 仅 OCS2 (Gazebo 已在跑)
├── rviz/
│   └── tracer_jaka_ocs2.rviz
├── scripts/
│   └── send_target.py           # 命令行发目标
└── src/
    ├── TracerJakaMpcNode.cpp    # MPC 求解器
    ├── TracerJakaMrtNode.cpp    # MRT + Gazebo 桥
    └── TracerJakaTargetNode.cpp # PoseStamped → TargetTrajectories
```

## 10. 跟你现有手动指令的等价关系

| 之前你手动发的 | OCS2 接管后由谁发 |
|----|----|
| `/diff_drive_controller/cmd_vel` (TwistStamped) | `tracer_jaka_mrt_node` 自动发 |
| `/arm_controller/commands`                      | `tracer_jaka_mrt_node` 自动发 |
| (新增) 末端目标位姿                              | 你点 RViz / 调 `send_target.py` |

也就是说, 一旦 OCS2 跑起来, 你就 **不要再手动 pub `cmd_vel` / `joint_trajectory`** 了, 否则会和 MRT 抢 topic。
