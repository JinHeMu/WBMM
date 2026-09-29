# Nominal whole-body trajectory produced by the planner and consumed by the
# OCS2 reference bridge.
#
# This message is the ROS boundary for wbmm_core::WholeBodyTrajectory. It
# replaces the polynomial-coefficient interface used by the REMANI upstream:
# the planner samples its own internal representation and publishes explicit
# state/input samples, so no consumer has to re-derive yaw from a path tangent
# or reconstruct a yaw rate from a polynomial derivative.
#
# Contract:
#   - One joint order and one frame (header.frame_id) for the whole trajectory.
#   - Units: m, rad, m/s, rad/s, s.
#   - Sample index i refers to the same instant in every per-sample array.
#   - The OCS2 9D state at sample i is
#       [base_x[i], base_y[i], base_yaw[i], joint_positions[i, :]]
#     and the OCS2 8D input is
#       [base_linear_velocity[i], base_yaw_rate[i], joint_velocities[i, :]]
#     so base_linear_velocity/base_yaw_rate are feedforward INPUTS, not a
#     measured base velocity.

std_msgs/Header header

# Identifier of this trajectory. A consumer must treat a change of value as a
# different trajectory; it is not a monotonic sequence number.
string trajectory_id

# Revisions of the environment and collision model the trajectory was planned
# against. A consumer holding a different revision must not execute it.
uint64 environment_revision
uint64 collision_model_revision

# Sample times in seconds from the trajectory start, strictly increasing.
# All per-sample arrays below have exactly this length.
float64[] time_from_start

# Joint order for every joint_* array below. Constant for the whole trajectory.
string[] joint_names

# Differential-drive base pose per sample.
float64[] base_x
float64[] base_y
float64[] base_yaw

# Feedforward base input per sample: u = [v, omega].
float64[] base_linear_velocity
float64[] base_yaw_rate

# Flattened joint samples, index = sample_index * joint_names.size() + joint_index.
# Length is time_from_start.size() * joint_names.size().
float64[] joint_positions
float64[] joint_velocities

# ExecutionPhase per sample; length equals time_from_start.size().
# Values follow wbmm_core::ExecutionPhase.
uint8 PHASE_IDLE=0
uint8 PHASE_NAVIGATE=1
uint8 PHASE_PRE_EXECUTION=2
uint8 PHASE_EXECUTION=3
uint8 PHASE_TRACKING=4
uint8 PHASE_FINISH=5
uint8 PHASE_FAULT=6
uint8[] phase
