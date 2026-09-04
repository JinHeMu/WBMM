# Frames & State Conventions

> 技术合同：所有算法、Topic、控制器、可视化中的坐标/速度/力都必须明确标注“相对哪个 frame”。
> 若没有标注，默认按本文档最接近的上下文解释。

## 1. Frame Tree

当前机器人 URDF（`tracer_jaka_zu5.urdf`）与 ROS TF 树：

```text
map
  └── odom                     # 全局/规划坐标系（MuJoCo 默认也是 odom）
        └── base_footprint     # 底盘地面投影 / 里程计原点
              └── base_link    # 底盘本体
                    ├── laser_link
                    ├── imu_link
                    ├── d455_link
                    └── jaka_base_link
                          └── Link_0
                                └── Link_1   (joint_1)
                                      └── Link_2   (joint_2)
                                            └── Link_3   (joint_3)
                                                  └── Link_4   (joint_4)
                                                        └── Link_5   (joint_5)
                                                              └── Link_6   (joint_6)
                                                                    └── Link_6_45
                                                                          └── jk_se_vi_200_link
                                                                                ├── d435i_link
                                                                                └── tool0_and_camera_link
                                                                                      └── tool0
```

关键固定变换：

| Parent | Child | 说明 |
|---|---|---|
| `odom` | `base_footprint` | 里程计/定位输出 |
| `base_footprint` | `base_link` | 固定 `z=0.147` |
| `base_link` | `jaka_base_link` | 固定 `xyz=[0,0,0.221]`, `rpy=[0,0,-1.57]` |
| `jaka_base_link` | `Link_0` | 固定 |
| `Link_6` | `Link_6_45` | 固定 `rpy=[0,0,-0.7854]` |
| `Link_6_45` | `jk_se_vi_200_link` | 固定 |
| `jk_se_vi_200_link` | `tool0_and_camera_link` | 固定 |
| `tool0_and_camera_link` | `tool0` | 固定 `z=0.27` |

## 2. 9D Whole-Body State Frame

```text
x = [x_b, y_b, yaw_b, q1, q2, q3, q4, q5, q6]^T
```

- `x_b, y_b, yaw_b`：**底盘位姿在规划坐标系中的表达**。
  - MuJoCo/当前主链：规划坐标系 = `odom`
  - 实机定位：规划坐标系可能是 `map`，但代码要求 `world_frame == planner_frame`
- `q1..q6`：机械臂关节角，是 **URDF 关节变量**，不额外附加 frame。

因此更严格的写法：

```text
x = [ p_base^odom, yaw_base^odom, q_arm ]
```

## 3. Velocity / Control Frame

控制输入：

```text
u = [v_b, omega_b, qdot1, qdot2, qdot3, qdot4, qdot5, qdot6]^T
```

- `v_b`：底盘纵向速度，在 **base_footprint/base_link 的 local frame** 中定义。
- `omega_b`：底盘航向角速度，绕 **base_footprint/base_link 的 Z 轴**。
- `qdot_i`：机械臂关节速度，在各自关节轴 local frame 中定义。

全局速度与局部速度关系（差速底盘）：

```text
p_base_dot^odom = R(yaw_b) * [v_b, 0, 0]^T
yaw_b_dot = omega_b
```

## 4. EE Pose / Velocity Frame

末端执行器统一使用 `tool0`（或配置的 `ee_frame`）。

位置：

```text
p_ee^odom    # EE 原点在 odom 中的位置
p_ee^base    # EE 原点在 base_link 中的位置
p_ee^jaka    # EE 原点在 jaka_base_link 中的位置
```

姿态：

```text
R_ee^odom    # EE 坐标系到 odom 的旋转矩阵
R_ee^base    # EE 坐标系到 base_link 的旋转矩阵
```

速度统一建议写成：

```text
V_ee^odom     = [v_ee^odom, omega_ee^odom]      # 在 odom 系中表达
V_ee^base     = [v_ee^base, omega_ee^base]      # 在 base_link 系中表达
```

公式中的 Jacobian 必须同时标注“相对哪个 frame”：

```text
V_ee^odom = J_wb^odom(x) u
V_ee^base = J_wb^base(x) u
```

## 5. Force / Torque Frame

力传感器话题 `/fts_broadcaster/wrench` 的 `frame_id` 通常是传感器自身坐标系：

```text
F^sensor = [F_x^sensor, F_y^sensor, F_z^sensor]
T^sensor = [T_x^sensor, T_y^sensor, T_z^sensor]
```

当前力控默认读取 `force_axis=z`，并使用 `absolute_force=true`，即：

```text
F_control = |F_z^sensor|
```

如果需要把力转换到 EE 或世界系，必须显式使用 TF：

```text
F^ee    = R_sensor^ee    * F^sensor
F^world = R_sensor^world * F^sensor
```

禁止在未标注 frame 的情况下混用 `F^sensor`、`F^ee`、`F^world`。

## 6. Task Surface Frame

任务 YAML 中的表面几何使用规划坐标系（`odom`）：

```text
surface.center
surface.normal_into_room
surface.axis_u
surface.axis_v
```

`task_normal` 在 `Waypoint` 中也是规划坐标系表达：

```text
n_task^odom
```

力控修正方向由 `task_normal` 转换到控制坐标系后使用：

```text
n_task^control = R_odom^control * n_task^odom
```

## 7. Naming Rules

所有新代码/消息/文档必须遵守：

| 量 | 推荐命名 |
|---|---|
| 世界系中的 EE 位置 | `p_ee_world` 或 `p_ee_odom` |
| 基系中的 EE 位置 | `p_ee_base` |
| 世界系中的 EE 线速度 | `v_ee_world` |
| 基系中的 EE 线速度 | `v_ee_base` |
| 传感器系中的力 | `F_sensor` 或 `wrench.force`（原始） |
| EE 系中的力 | `F_ee` |
| 世界系中的力 | `F_world` |
| 关节角 | `q` 或 `joint_positions` |
| 关节速度 | `qdot` 或 `joint_velocities` |

> 原则：**先写 frame，再写物理量**。例如 `p_ee^odom` 永远比 `p_ee` 更安全。
