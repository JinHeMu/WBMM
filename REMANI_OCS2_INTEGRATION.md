# REMANI + Tracer/JAKA + OCS2 integration

## Implemented data flow

```text
tracer_jaka_zu5_real.urdf
        |
        +--> REMANI MMConfig
        |      - base_footprint -> Link_0 fixed transform
        |      - joint_1 ... joint_6 axes and origins
        |      - base/JAKA/tool/camera collision spheres
        |
MuJoCo scene -> tracer_jaka_zu5_scene_esdf.npz
        |
        +--> REMANI GridMap
               - direct signed-distance/occupancy import
               - no point-cloud mapping or EDT reconstruction
        |
REMANI /planning/trajectory (PolynomialTraj)
        |
remani_to_ocs2_reference_bridge
        |
/mobile_manipulator_mpc_target (TargetTrajectories)
        |
RosReferenceManager -> WholeBodyTrajectoryCost -> MPC -> MRT
```

REMANI plans `[x, y, q1 ... q6]`. The bridge reconstructs:

```text
OCS2 state: [x, y, yaw, q1 ... q6]
OCS2 input: [v, omega, qdot1 ... qdot6]
```

The input reference is evaluated analytically from the polynomial rather than
being filled with zeros.

## Build

```bash
cd /home/a/ocs2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to remani_planner tracer_jaka_ocs2 --symlink-install
source install/setup.bash
```

## Start

Start the robot and OCS2 MPC/MRT using the existing simulation or real launch:

```bash
ros2 launch tracer_jaka_ocs2 ocs2_sim.launch.py use_csv_target:=false
```

or:

```bash
ros2 launch tracer_jaka_ocs2 ocs2_real.launch.py use_joy:=false
```

Then start REMANI and the bridge:

```bash
ros2 launch remani_planner remani_mpc_tracking.launch.py \
  use_sim_time:=true \
  odom_topic:=/base_controller/odom \
  joint_state_topic:=/joint_states
```

`/base_controller/odom` is the odometry topic published by the current
`tracer_jaka_mujoco` bridge and is also the launch default.

The RViz2 window started by `ocs2_sim.launch.py` contains a
`REMANI Planning` display group. It shows:

- `/kinoastar/local_start_goal`;
- `/front_end_mm_mesh_vis`;
- `/back_end_mm_mesh_vis`;
- `/global_traj`;
- `/kinoastar/path`;
- `/optimal_ctrl_pts`; and
- `/model_vis/vis_mm`.

REMANI visualization markers use `grid_map.frame_id`, so this integration
publishes them in `odom`, matching the OCS2 RViz fixed frame.

The launch loads
`grid_map/maps/tracer_jaka_zu5_scene_esdf.npz` directly. It does not subscribe
to a camera, depth image or point cloud. Override `static_esdf_file` only when
using a newly generated MuJoCo map:

```bash
static_esdf_file:=/absolute/path/to/scene_esdf.npz
```

Regenerate the NPZ whenever the static MuJoCo scene changes:

```bash
ros2 run grid_map mjcf_to_esdf \
  --xml /absolute/path/to/scene.xml \
  --output /absolute/path/to/scene_esdf.npz
```

The converter includes static box, sphere, cylinder and capsule geoms. Bodies
with joints are deliberately excluded because their collision is represented
by the robot model rather than the static environment ESDF.

The NPZ coordinates are MuJoCo-world coordinates. In the current robot MJCF,
`base_footprint` is placed at world `x=-2`, while its planar joint coordinate
and published odometry start at `x=0`. The integration launch therefore uses
`static_esdf_offset_x:=2.0`, making `x_odom=x_mujoco+2`. Set the three
`static_esdf_offset_*` arguments to the actual fixed translation if this MJCF
placement changes. Rotation between the MuJoCo and planner axes is not
supported by direct import and must be removed when generating the grid.

If REMANI's planning coordinates and the OCS2 `odom` frame are not identical,
set the fixed planar transform:

```bash
planner_to_ocs2_x:=0.0 \
planner_to_ocs2_y:=0.0 \
planner_to_ocs2_yaw:=0.0
```

## Tracking-error replanning

The planner compares the measured whole-body state from odometry and
`/joint_states` with the REMANI trajectory at the current trajectory time.
This detects both geometric drift and an MPC controller falling behind the
time-parameterized reference.

By default, replanning is requested when any of these errors persists for
`0.30 s`:

- mobile-base position error greater than `0.30 m`;
- wrapped base yaw error greater than `0.45 rad`; or
- maximum absolute arm-joint error greater than `0.30 rad`.

The trigger has a `0.60 s` grace period after each new trajectory and a
`2.0 s` minimum interval between replans. On a trigger, REMANI keeps the
original goal, rebuilds the global path from the latest measured whole-body
state, and publishes a replacement polynomial trajectory. The bridge sees
the new first section (`trajectory_id=1`), clears its old assembly, and
publishes the replacement OCS2 target.

Trajectory time reaching the end is no longer sufficient to report success.
The measured state must also satisfy the configured goal tolerances. Otherwise
the same measured-state replanning path is used.

Tune the feature directly from the integration launch, for example:

```bash
ros2 launch remani_planner remani_mpc_tracking.launch.py \
  tracking_error_position_threshold:=0.25 \
  tracking_error_yaw_threshold:=0.40 \
  tracking_error_joint_threshold:=0.25 \
  tracking_error_persistence:=0.40 \
  tracking_error_min_interval:=2.5
```

Set `tracking_error_replan_enabled:=false` to disable the feature.

## Required checks before enabling hardware commands

1. Confirm `/joint_states` contains `joint_1` through `joint_6`.
2. Confirm the planner logs `Loaded static MuJoCo ESDF`.
3. Confirm `/mobile_manipulator_mpc_observation` is being published.
4. Click RViz2 `2D Goal Pose`; both `/goal_pose` and
   `/move_base_simple/goal` are supported.
5. Inspect `/mobile_manipulator_mpc_target` before enabling the controllers.
6. Test base-only, arm-only, reverse and gear-switch trajectories at reduced
   velocity.

Only one OCS2 target publisher should be enabled. In particular, keep
`use_csv_target:=false` in simulation and `use_joy:=false` on the real robot
while the REMANI bridge is active. The moving camera also requires
`robot_state_publisher` only if point-cloud mapping is enabled again later.

## Collision model note

The sphere centers, per-sphere radii, kinematic transforms, tool and camera
geometry come directly from `tracer_jaka_zu5_real.urdf`. Both the front-end
collision checks and the back-end trajectory costs use the individual
`0.035–0.085 m` radii. This is important for the current JAKA home posture:
using `0.085 m` for every sphere creates a false arm self-collision.

## Verified result

With the current headless MuJoCo bridge, a `/goal_pose` at `(0.5, 0.0)`:

1. moved the planner from `WAIT_TARGET` to `GEN_NEW_TRAJ`;
2. produced one successful 8-dimensional REMANI polynomial trajectory;
3. moved the FSM to `EXEC_TRAJ`;
4. assembled a `3.978 s` whole-body reference in the bridge; and
5. published `/mobile_manipulator_mpc_target` with 9-dimensional states and
   8-dimensional inputs.
