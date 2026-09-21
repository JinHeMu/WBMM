# WBMM ROS Interface Contract

> Status: Phase 0 draft, version 1.  
> Scope: real `wbmm_hardware_interface.launch.py` and MuJoCo
> `mujoco_hardware_interface.launch.py`; all algorithm launch files.

## 1. Goal

The WBMM algorithm layer must not know whether it is connected to:

- the real Tracer + JAKA + sensors, or
- the MuJoCo simulated robot.

Both hardware interfaces therefore expose the same ROS topics, frames, units,
timestamps and command semantics. Backend-specific details stay behind the
hardware interface.

The machine-readable name list is
[`src/bringup/config/common/interface.yaml`](../src/bringup/config/common/interface.yaml).

## 2. Topic contract

| Direction | Topic | Type | Required | Notes |
|---|---|---:|---:|---|
| algorithms -> hardware | `/cmd_vel` | `geometry_msgs/msg/Twist` | yes | Base velocity command. |
| algorithms -> hardware | `/arm_controller/commands` | `std_msgs/msg/Float64MultiArray` | yes | Arm joint position command, ordered by `/joint_states.name`. |
| algorithms -> hardware | `/arm_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | no | Optional high-level arm action. |
| hardware -> algorithms | `/joint_states` | `sensor_msgs/msg/JointState` | yes | Match joints by `name`, not by array order. |
| hardware -> algorithms | `/wheel/odometry` | `nav_msgs/msg/Odometry` | yes | Raw wheel odometry. This is not `robot_localization` output. |
| hardware -> algorithms | `/imu/data` | `sensor_msgs/msg/Imu` | yes | Canonical IMU topic. |
| hardware -> algorithms | `/scan` | `sensor_msgs/msg/LaserScan` | yes | Canonical 2D LiDAR topic. |
| hardware -> algorithms | `/fts_broadcaster/wrench` | `geometry_msgs/msg/WrenchStamped` | yes | Force/torque at the configured sensor frame. |
| MuJoCo -> algorithms | `/clock` | `rosgraph_msgs/msg/Clock` | sim only | MuJoCo bridge is the clock producer. |
| hardware -> algorithms | `/camera/d455/color/image_raw` | `sensor_msgs/msg/Image` | no | RGB. |
| hardware -> algorithms | `/camera/d455/depth/image_raw` | `sensor_msgs/msg/Image` | no | Depth in metres. |
| hardware -> algorithms | `/camera/d455/color/camera_info` | `sensor_msgs/msg/CameraInfo` | no | RGB camera info. |
| hardware -> algorithms | `/camera/d455/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | no | Depth camera info. |

Legacy backend-specific names are not part of the contract:

- `/IMU_data` is replaced by `/imu/data`.
- `/odom` is not used for raw wheel odometry; use `/wheel/odometry`.

## 3. Frames contract

```text
map
 └── odom
      └── base_footprint
           ├── imu_link
           ├── laser_link
           ├── camera links
           └── JAKA links
```

| Frame | Meaning |
|---|---|
| `map` | Localization/map frame. Owned by localization algorithms, never by hardware. |
| `odom` | Continuous odometry frame. |
| `base_footprint` | Canonical base frame. |
| `imu_link` | IMU frame. |
| `laser_link` | 2D LiDAR frame. |
| `jk_se_vi_200_link` | F/T sensor frame. |
| `tool0` | End-effector/tool frame. |

The hardware interface may publish `odom -> base_footprint` only when configured
to do so. In the normal deployment, `robot_localization` owns that transform.
`robot_state_publisher` owns the robot-body transforms.

## 4. Clock and timestamp rules

- Real hardware uses system time.
- The MuJoCo interface publishes `/clock`; the bridge node itself uses
  `use_sim_time:=false` because it is the clock producer.
- Algorithm nodes in simulation use `use_sim_time:=true`.
- All stamped messages must have a non-zero, finite timestamp and a valid
  `frame_id` in the frame contract above.

## 5. QoS and rates

| Topic | Suggested QoS | Nominal rate |
|---|---|---:|
| `/joint_states` | reliable, keep last 10 | 100 Hz |
| `/wheel/odometry` | reliable, keep last 10 | 50 Hz |
| `/imu/data` | best effort, keep last 10 | 100 Hz |
| `/scan` | best effort, keep last 5 | 30 Hz |
| `/fts_broadcaster/wrench` | reliable, keep last 10 | 125 Hz |
| camera topics | best effort, keep last 5 | 15-30 Hz |

Exact rates are configuration parameters; the names and units are not.

## 6. Units and signs

- position: metres or radians
- velocity: metres/second or radians/second
- force: newtons
- torque: newton-metres
- wrench frame: `frame_id` in the message
- `/cmd_vel`: `linear.x` forward, `linear.y` left, `angular.z` counter-clockwise,
  expressed in `base_footprint`
- `odom` follows REP-103

## 7. Services and actions

- `/controller_manager/*` is a standard ros2_control implementation detail.
  Algorithm nodes must not depend on it.
- `/arm_trajectory_controller/follow_joint_trajectory` is optional but must use
  the same name in real and simulation when enabled.
- If a WBMM-specific hardware service is later required, it must live under
  `/wbmm/hardware/...`; do not use `/real/...` or `/sim/...`.

## 8. Backend rules

Real backend:

```bash
ros2 launch tracer_jaka_bringup wbmm_hardware_interface.launch.py
```

MuJoCo backend:

```bash
ros2 launch tracer_jaka_bringup mujoco_hardware_interface.launch.py
```

Algorithm launch files must not start either backend. They may start only
algorithms, read explicit config paths and consume the contract above.

## 9. OCS2 dual-reference / TaskPhase algorithm interface

The OCS2 dual-reference mode interface is an algorithm-to-algorithm contract
between REMANI, force control, the interactive target node, MPC and MRT. It is
independent of the hardware backend contract above.

| Interface | Type | Direction | Notes |
|---|---|---|---|
| `/mobile_manipulator_whole_body_target` | `ocs2_msgs/msg/MpcTargetTrajectories` | reference owner -> MPC | 9D `[x, y, yaw, q1..q6]`. |
| `/mobile_manipulator_ee_target` | `ocs2_msgs/msg/MpcTargetTrajectories` | reference owner -> MPC | 7D `[x, y, z, qx, qy, qz, qw]`. |
| `/mobile_manipulator_set_task_phase` | `wbmm_ocs2_ros/srv/SetTaskPhase` | external task state -> MPC | `int32 phase`, 0=Navigation, 1=Transition, 2=Execution, 3=Retract. |
| `/mobile_manipulator_task_phase_state` | `wbmm_ocs2_ros/msg/TaskPhaseState` | MPC -> MRT / external | latched status with requested phase, active phase and enable flag. |

Rules:

- The legacy `mobile_manipulator_mpc_target` topic is no longer part of the
  current dual-reference interface.
- Both reference topics use `reliable` QoS with a queue depth of 1.
- The phase state topic uses `reliable` + `transient_local` so a late-joining
  MRT receives the latest phase.
- Changing phase is a two-step transition: the requested phase is accepted
  immediately, while the active phase and the phase-weighted costs change on
  the next MPC `preSolverRun()`.
- MRT holds base and arm commands whenever the received MPC policy mode does
  not match the requested phase.
- Execution currently sets the whole-body phase weight to 0.0 until a separate
  base/posture regularization cost is implemented.

Usage:

```bash
ros2 service call /mobile_manipulator_set_task_phase \
  wbmm_ocs2_ros/srv/SetTaskPhase "{phase: 2}"

ros2 topic echo /mobile_manipulator_task_phase_state --once
```

Details, phase weight tables and validation records:
[wbmm_ocs2_dual_reference_mode_switch.md](wbmm_ocs2_dual_reference_mode_switch.md).
