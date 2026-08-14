# WipePlanner

`wipe_planner` is the C++ continuous-contact whole-body reference planner for
the Tracer + JAKA wall-cleaning pipeline:

```text
REMANI navigation -> WipePlanner constrained reference -> OCS2 MPC -> robot
```

The planner is base-search dominated. It first discretizes the geometric raster,
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
the output is continuous, not merely collision-free at isolated endpoints. A
normal admittance correction can maintain force without changing the base path.

```bash
colcon build --symlink-install --packages-select wipe_planner
source install/setup.bash
ros2 launch wipe_planner wipe_pipeline.launch.py
```

To inspect the complete plan without starting or moving a robot, use the
preview-only launch:

```bash
ros2 launch wipe_planner wipe_plan_preview.launch.py
```

This starts only `wipe_plan_preview_node` and RViz. It does not start MuJoCo,
REMANI, OCS2 MPC/MRT, or any controller, and the node has no velocity or arm
command publisher. The dedicated RViz view follows REMANI's visualization
convention: blue ghosts show the constrained whole-body coverage result, the green
`Path` is the tool coverage, and the gray `Path` is the differential-drive base
motion. The result is published once with transient-local durability, avoiding
the frame drops caused by repeatedly replacing hundreds of mesh markers.

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
settles for one second, and approaches the wall at 0.02 m/s. The first contact
reference is then held for four nominal seconds, and adaptive progress slows
further whenever whole-body tracking lags.

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

- `/wipe_planner/trajectory`: decimated complete-robot snapshots and paths
- `/wipe_planner/base_path`: nonholonomic base path
- `/wipe_planner/ee_coverage_path`: complete constrained end-effector Path
- `/wipe_planner/active_whole_body_reference`: current rolling MPC reference
- `/wipe_planner/phase`: current reference owner/phase
- `/wipe_planner/normal_force`: filtered simulated force
- `/wipe_planner/{base,joint,ee}_tracking_error`: MPC diagnostics
- `/wipe_planner/virtual_progress`: current path time `tau` in nominal seconds
- `/wipe_planner/virtual_progress_rate`: current `tau_dot`
- `/wipe_planner/progress_lag_error`: `tau - projected_tau` in seconds
- `/wipe_planner/contouring_error`: EE distance to the locally projected path
