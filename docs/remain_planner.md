# REMANI-Planner for ROS 2 Humble

This workspace contains the ROS 2 Humble port of REMANI-Planner. The original
ROS 1 source tree is kept outside this workspace for comparison.

## Dependencies

On Ubuntu 22.04 with ROS 2 Humble Desktop installed:

```bash
sudo apt update
sudo apt install \
  python3-colcon-common-extensions \
  ros-humble-cv-bridge \
  ros-humble-pcl-conversions \
  ros-humble-tf2-geometry-msgs \
  ros-humble-ompl \
  libeigen3-dev \
  libompl-dev
```

## Build

```bash
cd /home/a/wbc/REMANI-Planner/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

If a previously built package has stale CMake flags, rebuild it with
`--cmake-clean-cache`.

## Run the local examples

The ROS 2 examples include a lightweight local simulator. It publishes the
point-cloud map, odometry, joint state, and gripper state, and executes the
polynomial trajectory emitted by the planner. No ROS 1 simulator packages are
required.

```bash
# Dense cuboid environment
ros2 launch remani_planner exp0.launch.py

# Bridge environment
ros2 launch remani_planner exp1.launch.py
```

The examples start in manual-goal mode and wait for RViz2's `2D Goal Pose`
tool. Useful launch arguments:

```bash
# Run without RViz
ros2 launch remani_planner exp0.launch.py rviz:=false

# Run the original preset waypoint sequence automatically
ros2 launch remani_planner exp0.launch.py target_type:=2 auto_start:=true
```

RViz2 displays the current mobile manipulator from the
`/model_vis/vis_mm` MarkerArray topic. The model follows the simulated odometry,
joint state, and gripper state. If the display was removed from a custom RViz2
configuration, add a `MarkerArray` display with fixed frame `world`, reliable
reliability, and transient-local durability.

The planned path is displayed using `/global_traj`, `/kinoastar/path`,
`/front_end_mm_mesh_vis`, and `/back_end_mm_mesh_vis`.

For a real robot or another simulator, keep the planner node and replace the
local simulator with publishers/controllers for the configured topics,
especially `/mm/car/odom`, `/mm/mani/joint_state`, the point cloud input, and
the trajectory/controller interface.
