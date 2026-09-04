# WipePlanner

`wipe_planner` is the C++ continuous-contact whole-body reference planner for
the Tracer + JAKA wall-cleaning pipeline:

```text
REMANI navigation -> WipePlanner constrained reference -> OCS2 MPC -> robot
```

The default coverage geometry is now a continuous four-row snake polyline on a
horizontal table plane at `z=0.50 m`. It covers `1.60 m` along X and `0.20 m`
along Y; the tool stays pointed down and never lifts between rows. The mobile
base remains outside one long table edge and moves only forward/reverse along X,
driving the full table length on each of the four strokes.
The system-level `tracer_jaka_bringup/wipe_sim.launch.py` defaults to
trajectory-only validation: MuJoCo loads
`scene_esdf_validation.xml` (robot, ground and light only) and REMANI loads
`empty_esdf.npz`, whose occupancy is zero everywhere. No table, wall or other
environment obstacle is present; the configured `z=0.50 m` surface is therefore
a virtual task plane rather than a simulated contact body.

Select the previous vertical-wall RAS stroke by restoring a vertical surface
and setting:

```yaml
wipe_task:
  coverage:
    pattern: "ras"
```

The RAS dimensions remain configured through `coverage.width/height`. Horizontal
table raster dimensions come from `surface.x_limits/y_limits`, while the contact
height comes from `surface.center.z`.

The planner is base-search dominated. It first discretizes the selected coverage path,
then runs a REMANI-inspired Hybrid A* in `(base_x, base_y, base_yaw)`. Motion
primitives use the exact differential-drive/unicycle transition and include
forward/reverse, gear-switch, curvature, and curvature-change costs. Every
expanded base node carries a warm-started arm IK solution; nodes are rejected
when the tool pose is unreachable, `tool0 +Z` is not wall-normal, the base leaves
the configured wall-parallel standoff tube, or the interpolated arm motion enters
self-collision. The selected nodes therefore form one 9D whole-body path rather
than a base path patched to an independent pointwise IK result.

During each horizontal wipe the base faces along the wall and drives forward or
backward; row changes keep the base stationary while dense continuation IK moves
the arm. Joint-space collision checks are interpolated between task samples so
the output is continuous, not merely collision-free at isolated endpoints. The
normal reference uses a velocity-limited second-order admittance correction
without changing the base path. It is reset outside contact, so an unloaded
sensor cannot preload a wall-penetrating reference during navigation.
During the final approach, the wallward reference is kept within 0.1 mm of the
measured tool. Contact detection opens five nominal seconds before the planned
contact; this final search already uses the same slew-limited direct arm command
as contact control, so contact does not cause a controller-mode step. The first
0.5 N sample captures the actual normal contact plane and skips the remaining
open-loop approach distance; path time then freezes once while the initial force
settles. After that latch, ordinary force ripple continuously scales path speed
instead of restarting the two-second settling hold. A sustained error above
8 N pauses progress after 0.25 s; it resumes only after the error remains below
5 N for 0.5 s. This hysteresis keeps the active MPC horizon continuous through
raster corners while still stopping tangential motion for a genuine loss of
force regulation. A missing wrench stream freezes virtual progress and commands
a bounded retreat.
The 20 mm safety retreat is applied at 10 mm/s from the last command, rather
than as a one-cycle Cartesian jump.
The MuJoCo cleaning surface uses a compliant contact profile to represent the
foam/eraser layer expected on the real tool. Its active layer is 8 mm and the
pad collision margin is 3 mm; the previous 40 mm wall compliance was too deep
to preserve visual and force-contact agreement.
The 35 N hard limit is latched; after inspection,
reset it explicitly through `/wipe_planner/enable_force_control`
(`std_srvs/srv/SetBool`, `data: true`).

通用恒力、导纳、全身力跟随控制及其 MuJoCo 测试已迁移到独立的
`whole_body_force_control` 包；WipePlanner 只调用该包的控制库。

### 仿真预接触卡住排查

在 `ros2 launch tracer_jaka_bringup wipe_sim.launch.py` 的 MuJoCo 仿真中，
力传感器来自接触求解器，接触瞬间可能出现单帧或短时力跳变。如果它在预接触
阶段触发：

- `force_contact_detected_`（误判已接触）；
- `force_hard_stop_`（超过 35 N 并锁存）；
- `initial_force_settled_` 一直无法满足（接触力未稳定到目标力附近）；

WipePlanner 会把 `virtual_progress_rate` 置 0，机械臂看起来卡在初始/预接触点。

代码已加入三层保护：

```text
force_spike_rejection_n: 8.0
force_spike_confirm_samples: 3
force_contact_confirm_samples: 3
force_hard_limit_confirm_samples: 3
```

- 单帧力跳变不会进入接触检测、导纳和硬限位判断；
- 接触判定必须连续多帧超过阈值；
- 硬限位也必须连续多帧确认后才锁存。

如果仍然卡住，先看当前状态：

```bash
ros2 topic echo /wipe_planner/force_control_state --once
ros2 topic echo /wipe_planner/normal_force --once
ros2 topic echo /wipe_planner/virtual_progress_rate --once
```

- `active_force_settling`：接触力还没稳定，等待 `force_settle_hold` 秒；
  可适当调大 `force_progress_tolerance` 或调小 `force_filter_alpha`。
- `over_force_retreat`：硬限位锁存，执行：

  ```bash
  ros2 service call /wipe_planner/enable_force_control std_srvs/srv/SetBool "{data: true}"
  ```

- `active_force_paused`：力误差持续过大，等它回到 `force_progress_resume_error`
  以下；或调大 `force_progress_pause_error` / `force_progress_resume_error`。

仿真中如果接触瞬间冲击仍然偏大，可以降低 MuJoCo 接触刚度/提高阻尼，或把
`force_contact_threshold` 从 0.5 调到 2.0~3.0 N，避免把轻微力纹波当成真实接触。

```bash
colcon build --symlink-install --packages-select wipe_planner
source install/setup.bash
ros2 launch tracer_jaka_bringup wipe_sim.launch.py
```

### 50 个入口构型的 future-task 检查

`wipe_future_task_batch` 是一个不依赖 DDS、RViz 或控制器的 C++ 批处理程序。
它在相同末端预接触位姿约束下生成不同的 9D 全身状态
`[base_x, base_y, base_yaw, q1..q6]`，然后严格执行
`q_pre(i) -> 直接法向接触 -> 完整覆盖`。轨迹第一帧必须逐元素等于输入入口；
程序不会追加机械臂对齐段，也不会回到统一的标准预接触构型。

```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run wipe_planner wipe_future_task_batch \
  --urdf=$PWD/src/robot/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf \
  --task=$PWD/src/applications/wiping/wipe_planner/config/wipe_task.yaml \
  --output=$PWD/results/future_task_precontact_50 \
  --samples=50 \
  --sequence-offset=2026
```

输出包括 `rollouts.csv`、`trajectories.csv` 和 `summary.md`。`success` 表示从指定
入口直接完成法向靠近和全部覆盖点；
`nominal_execution_time_s` 是轨迹时间而非实际耗时；碰撞间隙当前严格指 URDF
非相邻几何体之间的自碰间隙。ESDF 环境间隙、实际执行时间、跟踪误差和接触力
质量必须在 MuJoCo/实机闭环批处理阶段另行记录，不能与此结果混为一谈。

用专用 RViz 场景查看这 50 个入口：

```bash
ros2 launch wipe_planner future_task_results.launch.py
```

该启动不运行 MuJoCo、REMANI、OCS2 或控制器，也不依赖 TF。所有机器人网格和
轨迹 Marker 均直接位于 `odom`：地面的绿色球是完整成功入口，红色球是失败入口；
淡色连线表示不同底盘/机械臂组合共同满足同一个末端预接触目标；彩色机器人与
曲线分别突出显示最短成功、最长成功和成功样本中的最小自碰间隙 rollout。

To inspect the complete plan without starting or moving a robot, use the
preview-only launch:

```bash
ros2 launch wipe_planner wipe_plan_preview.launch.py
```

This starts only `wipe_plan_preview_node` and RViz. It does not start MuJoCo,
REMANI, OCS2 MPC/MRT, or any controller, and the node has no velocity or arm
command publisher. The dedicated RViz view follows REMANI's visualization
convention: the yellow robot is the wall-normal pre-contact pose, blue ghosts
show the constrained whole-body coverage result, the green `Path` is the tool
coverage, and the gray `Path` is the differential-drive base motion. The result
is published once with transient-local durability, avoiding the frame drops
caused by repeatedly replacing hundreds of mesh markers.

WipePlanner first plans the full-board coverage path from the known wall geometry
and publishes it to RViz. It then adds a wall-normal pre-contact pose 0.12 m on
the room side of the wall. That complete base pose and six-joint state is sent to
REMANI on `/remani_planner/whole_body_goal`; wall contact is not part of REMANI's
navigation target. A complete 9D arrival still uses the squared error of
`[x,y,yaw,joint_1..joint_6]`, with yaw wrapped across pi. Near the terminal part
of navigation, a base/yaw-only handoff is also allowed: WipePlanner computes a
collision-free joint-space path from the measured arm to the pre-contact arm
state while holding the base reference fixed. This prevents REMANI's non-zero
base-speed model from producing yaw loops for a manipulator-only residual. At
the handoff
REMANI explicitly enters
`TASK_EXEC`: localization and visualization stay active, while navigation,
planning, replanning, and old-trajectory collision checks stop. The active FSM
state is available on `/remani_planner/fsm_state`.

Every reference sample sent to OCS2 is a full-body reference. Its state is
`[base_x, base_y, base_yaw, joint_1, ..., joint_6]` (9D), and its input is
`[base_v, base_yaw_rate, joint_velocity_1, ..., joint_velocity_6]` (8D).
Every precomputed contact state, including all six arm joint angles and the
wall-normal `tool0` +Z orientation, is preserved exactly. After handoff the arm
first completes its collision-checked alignment at the pre-contact clearance,
settles for one second, and uses a guarded two-stage approach whose final 20 mm
runs at 0.001 m/s. The first contact
reference is then held for four nominal seconds, and adaptive progress slows
further whenever whole-body tracking lags.

The OCS2 model does not contain wall-contact dynamics, so the MRT arm adapter
keeps consuming the OCS2 policy in free space, then changes ownership during
the final guarded approach and contact. It clears the velocity integrator and
tracks WipePlanner's force-corrected six-joint reference from
`/wipe_planner/contact_arm_reference`, with a 0.10 rad measured-state command
bound and 0.10 rad/s command slew. OCS2 continues to control the mobile base.
The validated nominal contact
tuning is 12 N, `M=2`, `D=200`, `K=50`, and a 0.001 m/s admittance velocity
limit. Coverage runs at 0.015 m/s on horizontal strokes and 0.008 m/s at row
changes.

The waypoint timestamps are nominal path time, not deadlines in wall-clock
time. A reference manager maintains a monotonic virtual time `tau`, projects
the measured 9D whole-body state onto a local part of the trajectory, and
publishes the MPC window as `x_ref(tau)`. Normally `tau_dot` converges to 1.0.
Whole-body tracking or progress lag slows it toward zero, while a robot slightly
ahead of the nominal progress may advance at up to 1.15. Thus a point marked
`tau=10 s` is reached when the robot reaches that path progress; it is not
skipped merely because ten seconds of wall time elapsed.

RViz uses a stable color convention: green is the constrained contact path,
and the green/red spheres are its start/end.
Opaque blue arrows on the complete-robot snapshots are the planned `tool0` +Z
axes and must point from the room into the wall.
The complete end-effector coverage is also published as a green
`nav_msgs/Path`. The active MPC horizon is separate: cyan whole-body ghosts and
base line plus a magenta end-effector line, refreshed at a bounded rate.
The blue arrow is the mobile-base component of `trajectory.front()` sent to
REMANI; the green start sphere is an end-effector point on the wall, not a base
navigation goal.
The OCS2 rolling prediction (`EE Trajectory`) is hidden by default because it is
replaced every MPC update and is not the planned cleaning path.

Useful topics:

- `/wipe_planner/preview/scene`: known board, path lines, endpoints and summary
- `/wipe_planner/back_end_mm_mesh_vis`: constrained whole-body snapshots
- `/wipe_planner/preview/base_path`: preview-only differential-drive base path
- `/wipe_planner/preview/ee_coverage_path`: preview-only constrained tool path
- `/wipe_planner/preview/status`: plan point count and nominal duration

- `/wipe_planner/whole_body_trajectory`: 9D `JointTrajectory` data contract for wbmm_visualization
- `/wipe_planner/phase_schedule`: phase schedule accompanying the trajectory
- `/wipe_planner/live_state`: live 9D `JointState` for the unified robot mesh
- `/wipe_planner/trajectory`: task-semantic markers and paths (no robot mesh snapshots)
- `/wipe_planner/base_path`: nonholonomic base path
- `/wipe_planner/ee_coverage_path`: complete constrained end-effector Path
- `/wipe_planner/active_whole_body_reference`: current rolling MPC reference
- `/wipe_planner/phase`: current reference owner/phase
- `/wbmm/robot_mesh`: unified whole-body mesh rendered by wbmm_visualization
- `/wbmm/time_segments`: trajectory time-window snapshots (published by wbmm_visualization)
- `/wbmm/base_path`, `/wbmm/ee_path`: unified path displays
- `/wipe_planner/normal_force`: filtered simulated force
- `/wipe_planner/admittance_offset`: bounded normal reference correction
- `/wipe_planner/contact_arm_reference`: force-corrected six-joint contact command
- `/wipe_planner/force_control_state`: guarded/settling/throttled/paused/safety state
- `/wipe_planner/force_progress_scale`: filtered 0--1 force-based path-speed scale
- `/wipe_planner/enable_force_control`: runtime `std_srvs/SetBool` switch
- `/wipe_planner/{base,joint,ee}_tracking_error`: MPC diagnostics
- `/wipe_planner/virtual_progress`: current path time `tau` in nominal seconds
- `/wipe_planner/virtual_progress_rate`: current `tau_dot`
- `/wipe_planner/progress_lag_error`: `tau - projected_tau` in seconds
- `/wipe_planner/contouring_error`: EE distance to the locally projected path

## Localized real-robot front-board pipeline

`tracer_jaka_bringup/launch/wipe_real_pipeline.launch.py` composes the localized real bringup
with one TF-aware WipePlanner reference owner. REMANI and all board geometry are
in `map`; OCS2/MRT remains in `odom`. The node converts the observed base state
from `odom` to `map` before planning, and converts every MPC reference base pose
back from `map` to `odom`. The six arm positions and the body-frame input
`[v, yaw_rate, joint_velocities...]` are unchanged. The standard REMANI bridge
is forced off by the pipeline so `/mobile_manipulator_mpc_target` has one owner.

The default task `wipe_task_real_front.yaml` assumes the localized start is
`map=(0,0,0)`, a vertical board is two metres ahead at `map X=2.0`, and the
reachable coverage centre is `Z=0.55`. It covers `0.90 x 0.40 m` with nine
continuous snake strokes. With a pad effective width of at least 5 cm this is
full material coverage. Force control is disabled and the default reference is
5 cm in front of the board (`contact.offset=-0.05`) for the first dry run.

Preview only (no robot/controller/REMANI):

```bash
ros2 launch wipe_planner wipe_real_plan_preview.launch.py use_rviz:=true
```

The standalone preview publishes an identity `map -> odom` because it does not
start AMCL/SLAM and the reusable RViz profile has `odom` as its fixed frame. If
a localization stack is already running, pass
`publish_preview_map_to_odom:=false` to avoid two TF publishers.

Complete stack with all real-motion gates closed:

```bash
ros2 launch tracer_jaka_bringup wipe_real_pipeline.launch.py
```

Only after TF, topic ownership, path placement and the physical emergency stop
have been checked, explicitly open all motion gates and request the goal:

```bash
ros2 launch tracer_jaka_bringup wipe_real_pipeline.launch.py \
  jaka_read_only:=false command_output_enabled:=true \
  safety_release:=true auto_goal:=true
```

Do not run `wipe_sim.launch.py` on hardware; it is the MuJoCo/empty-ESDF
validation composition and starts a different OCS2/REMANI stack.
