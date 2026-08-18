# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS 2 workspace for Model Predictive Control (MPC) of a mobile manipulator — a Tracer differential-drive base with a JAKA 6-DOF arm. It combines OCS2 (MPC framework) for real-time whole-body control with REMANI for global polynomial trajectory optimization.

## Build

```bash
cd /home/a/WBMM
# Full build of the relevant packages
colcon build \
  --packages-up-to tracer_jaka_ocs2 tracer_jaka_mujoco tracer_base remani_planner \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build only one package with dependencies
colcon build --packages-up-to remani_planner --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Quick rebuild of a single package (no deps)
colcon build --packages-select tracer_jaka_ocs2 --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

After building, source the workspace:
```bash
source install/local_setup.bash
```

The workspace uses C++17. Build output goes to `build/` and `install/`. Build logs accumulate under `log/`.

## Running

Quick command reference: [docs/QUICKSTART.md](docs/QUICKSTART.md).

**Simulation (MuJoCo + REMANI + OCS2)**:
```bash
# One-command full simulation pipeline
ros2 launch tracer_jaka_ocs2 ocs2_sim.launch.py

# Only SLAM/localization simulation
ros2 launch tracer_jaka_mujoco slam_sim.launch.py

# Task table / wiping preview
ros2 launch tracer_jaka_mujoco task_table_sim.launch.py
ros2 launch wipe_planner wipe_plan_preview.launch.py
```

**MuJoCo → nvblox → REMANI → OCS2**:
```bash
# Docker side
ros2 launch my_nvblox_bringup mujoco_mapping_export.launch.py \
  ros_domain_id:=20 rviz:=true

# Host side
ros2 launch tracer_jaka_mujoco mujoco_nvblox_mapping.launch.py \
  viewer:=false ros_domain_id:=20

# After export
ros2 launch tracer_jaka_ocs2 mujoco_mapped_esdf_control.launch.py
```

**Real robot**:
```bash
# CAN + localization/SLAM
sudo ip link set can0 up type can bitrate 500000
ros2 launch tracer_jaka_mujoco real_slam.launch.py

# OCS2 real control (not yet a full wiping entry)
ros2 launch tracer_jaka_ocs2 ocs2_real.launch.py
```

Always source the workspace first:
```bash
cd /home/a/WBMM
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Architecture — Data Flow

The system follows a **plan-then-track** pipeline with a bridge layer:

```
REMANI Planner (global, lower frequency)
   │  PolynomialTraj: 8-dim [x, y, q1..q6]
   ▼
RemaniToOcs2ReferenceBridge
   │  Reconstructs yaw from velocity direction
   │  Handles zero-velocity degeneracy and yaw unwrapping
   │  Anchors reference to current observation at trajectory start
   ▼
TargetTrajectories: 9-dim state [x, y, yaw, q1..q6] + 8-dim input [v, ω, q̇1..q̇6]
   │  Topic: /mobile_manipulator_mpc_target
   ▼
OCS2 MPC (SLQ/DDP solver)
   │  WholeBodyTrajectoryCost + QuadraticInputCost
   │  Publishes: /mobile_manipulator_mpc_policy
   ▼
MRT (Mode-Reference Trajectory) controller bridge
   │  Converts policy to hardware-level commands
   │  Publishes: Twist (chassis) + Float64MultiArray (arm joints)
   ▼
Tracer base + JAKA arm (simulated via MuJoCo or real hardware)
```

## Three Source Packages

### `src/vendor/ocs2_ros2/` — OCS2 Framework (ROS 2 port)
Upstream OCS2 libraries: `core`, `ddp`, `mpc`, `oc`, `pinocchio_interface`, `ros_interfaces`, `mobile_manipulator`, etc. Contains git submodules. These are the MPC solver, cost functions, constraints, and ROS integration layer. Rarely modified directly.

### `src/algorithms/control/tracer_jaka_ocs2/` — Robot-Specific Implementation
The main package is `tracer_jaka_ocs2`, which contains all ROS 2 executables:

| Executable | Source | Purpose |
|---|---|---|
| `tracer_jaka_mpc_node` | `TracerJakaMpcNode.cpp` | MPC solver — runs the SLQ/DDP optimization |
| `tracer_jaka_mrt_node` | `TracerJakaMrtNode.cpp` + `TracerJakaVisualization.cpp` | MRT controller bridge — converts MPC policy to chassis Twist + arm commands |
| `tracer_jaka_target_node` | `TracerJakaTargetNode.cpp` | Interactive marker in RViz for target pose with right-click "Send target" |
| `tracer_jaka_joy_target_node` | `tracer_jaka_joy_target_node.cpp` | Gamepad teleop target |
| `tracer_jaka_joy_whole_body_node` | `tracer_jaka_joy_whole_body_node.cpp` | Gamepad whole-body control ("carrot" mode) |
| `remani_to_ocs2_reference_bridge` | `remani_to_ocs2_reference_bridge.cpp` | Converts REMANI PolynomialTraj → OCS2 TargetTrajectories |
| `tracer_jaka_whole_body_trajectory_node` | `tracer_jaka_whole_body_trajectory_node.cpp` | Plays back pre-recorded whole-body CSV trajectories |
| `csv_path_visualizer_node` | `csv_path_visualizer_node.cpp` | Visualizes CSV paths |

Supporting packages: `src/simulation/tracer_jaka_mujoco` (MuJoCo sim + URDF models), `src/drivers/base/tracer_base` (CAN driver for real Tracer base), `src/perception/grid_map` (ESDF representation), `src/drivers/arm/*` (JAKA arm driver).

### `src/vendor/remani_planner/` — Global Planner
Sub-packages forming a layered planning pipeline:

| Package | Purpose |
|---|---|
| `plan_env` | Occupancy/signed-distance grid map from ESDF |
| `mm_config` | Mobile manipulator kinematic/dynamic configuration |
| `path_searching` | Kinodynamic path search in the grid map |
| `traj_opt` | Polynomial trajectory optimization (MINCO-based) |
| `traj_utils` | Shared data types and messages |
| `plan_manage` | Top-level planner node, replan FSM, and visualization |
| `quadrotor_msgs` | Message definitions (PolynomialTraj, etc.) |

`plan_manage` ties it together:
- `remani_planner_node.cpp` — main entry, spins the REMANIReplanFSM
- `remani_replan_fsm.cpp` — replan finite state machine (triggers replanning, manages trajectory lifecycle)
- `planner_manager.cpp` — orchestrates path search + trajectory optimization
- `planning_visualization.cpp` — RViz markers

Launch files: `exp0.launch.py` (simulation), `remni_mpc_tracking.launch.py` (joint REMANI+MPC tracking).

## Key Design Details

### State/Input Dimension Mapping
- REMANI plans in **flat output space**: `[x, y, q1...q6]` (8D)
- OCS2 works in **full state space**: `[x, y, yaw, q1...q6]` (9D) with input `[v, ω, q̇1...q̇6]` (8D)
- The bridge reconstructs `yaw` from `(vx, vy)` velocity direction; handles zero-velocity at start/stop/reversal points

### Task Configuration
The `.info` files in `tracer_jaka_ocs2/config/` define the OCS2 optimization problem:
- `task.info` — simulation task (cost weights, constraints, solver settings)
- `task_real.info` — real robot task

### MRT Controller Bridge
The MRT node translates the MPC policy into actual robot commands:
- Chassis: `geometry_msgs/Twist` or `TwistStamped` (configurable via `use_stamped_cmd`)
- Arm: `std_msgs/Float64MultiArray` published to the forward command controller

### REMANI Replan FSM
The replan finite state machine triggers global replanning when the environment changes (new obstacles detected via ESDF). It manages trajectory segments with different "gear" directions (forward/reverse).

## Documentation Files

- [README.md](README.md) — basic build/run commands
- [QUICKSTART.md](docs/QUICKSTART.md) — concise startup commands
- [总体 Pipeline.md](总体 Pipeline.md) — architecture diagram and state/control mapping table (Chinese)
- [REMANI_OCS2_INTEGRATION.md](docs/REMANI_OCS2_INTEGRATION.md) — detailed REMANI-OCS2 data flow and bridge design (Chinese/English mixed)
