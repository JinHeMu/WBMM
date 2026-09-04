# 统一任务 YAML 规范（schema_version: 1）

一个任务文件同时描述任务轨迹生成、TA-WBMP 规划约束、REMANI 导航入口以及
MPC/可选力控执行参数。运行主链只接受这个文件，不再读取 WipePlanner YAML。

```yaml
task:
  schema_version: 1
  name: task_name
  frame_id: odom
  type: surface_contact

  surface:                         # 任务几何
    type: planar                   # planar | cylindrical
    center: [x, y, z]
    normal_into_room: [nx, ny, nz]
    axis_u: [ux, uy, uz]
    u_limits: [umin, umax]
    v_limits: [vmin, vmax]

  pattern:                         # TaskWaypoint 生成器
    type: raster                   # raster | ras | waypoint_sequence
    rows: 3
    columns: 5
    sample_spacing: 0.03
    tangential_speed: 0.025
    row_change_speed: 0.012

  approach:                        # q_pre 到任务入口
    clearance: 0.08
    speed: 0.015
    hold_duration: 1.0

  constraints:                     # TA-WBMP 候选和全程任务约束
    base_standoff_direction: [-1, 0, 0]
    base_policy: fixed             # fixed | follow_task
    standoff_samples: [1.55, 1.58, 1.62]
    # 沿 base_standoff_direction 的水平正交方向采样；不是沿工具轨迹切线。
    longitudinal_offset_samples: [-0.08, 0.0, 0.08]
    task_yaw: 0.0
    max_position_error: 0.012
    max_axis_error: 0.12
    minimum_normalized_joint_margin: 0.0
    minimum_manipulability: 0.0
    desired_tool_rotation: [r00, r01, r02, r10, r11, r12, r20, r21, r22]

  navigation:                      # q_start 和 REMANI 入口规划参数
    initial_state: [base_x, base_y, yaw, q1, q2, q3, q4, q5, q6]
    grid_resolution: 0.10
    robot_radius: 0.31
    base_speed: 0.10
    angular_speed: 0.30
    joint_speed: 0.05
    sample_period: 0.10
    obstacles: []

  solver:                          # 任务全身 IK/候选搜索
    max_iterations: 110
    damping: 0.0001
    max_joint_step: 0.18
    nominal_arm_seed: [q1, q2, q3, q4, q5, q6]
    search_arm_seed: [q1, q2, q3, q4, q5, q6]

  execution:
    mpc:
      reference_rate: 20.0
      reference_horizon: 3.0
      reference_dt: 0.08
      tracking_slow_squared_tolerance: 0.04
      tracking_stop_squared_tolerance: 0.16
    force_control:
      enabled: false               # launch 参数可覆盖
      mode: constant_force         # admittance | constant_force | force_follow
      wrench_topic: /fts_broadcaster/wrench
      force_axis: z
      absolute_force: true
      desired_force: 12.0
      mass: 3.0
      damping: 45.0
      stiffness: 300.0
      filter_alpha: 0.25
      max_offset: 0.015
      max_velocity: 0.010
      base_share: 0.0
      max_base_delta: 0.02
      max_joint_delta: 0.10
      sensor_timeout: 0.20
      progress_full_speed_error: 2.0
      progress_pause_error: 8.0
      progress_min_scale: 0.10
      hard_limit: 35.0
```

执行时只有一个 MPC 参考所有者：TA-WBMP Coordinator。力控关闭时直接发送
`tau_task`；力控打开时，Coordinator 调用 `whole_body_force_control` 对接触段
叠加表面法向顺应修正，再发送最终 9D 参考。REMANI 只负责到达 `q_pre`，其直接
OCS2 bridge 必须关闭。
