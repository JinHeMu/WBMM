# my_nvblox_bringup

This directory is the Git-tracked source of truth:

```text
WBMM/src/perception/my_nvblox_bringup
```

Only this copy should be edited and committed. Because Isaac ROS nvblox and
CUDA live in the Isaac ROS container, synchronize this package into that
workspace before building; generated bags and maps remain outside Git.

The package has three layers:

- `nvblox_core.launch.py`: sensor-independent nvblox core.
- `mujoco_esdf.launch.py`: simulated fixed D455 topics and simulation time.
- `d455_esdf.launch.py`: real base-mounted D455 topic adapter plus nvblox.
- `d435_esdf.launch.py`: optional arm-mounted D435 adapter for manipulation.

It builds a 5 cm static TSDF and 3D ESDF in `odom`. The 2D laser remains the
localization sensor; RGB-D is responsible for the collision map. Simulation
keeps a rolling 7 m map, while the real D455 launch keeps integrated map blocks
by default.

## Build

On the host, synchronize the tracked source into the Isaac ROS workspace:

```bash
cd /home/a/WBMM
src/perception/my_nvblox_bringup/scripts/sync_to_isaac_ros_ws.sh
```

Set `ISAAC_ROS_WS=/another/isaac_ros-dev` if the Docker workspace has a
different host path. Then build the synchronized copy inside the container.

Inside the Isaac ROS container:

```bash
# Enter from the host as the container's admin user. On this machine the root
# user cannot discover the host Fast DDS shared-memory endpoints.
docker exec -it -u admin --workdir /workspaces/isaac_ros-dev \
  isaac_ros_dev-x86_64-container bash

cd /workspaces/isaac_ros-dev
colcon build --symlink-install --packages-select my_nvblox_bringup
source install/setup.bash
```

Map-producing launches accept `output_dir:=...`. Its default can also be set
with `NVBLOX_OUTPUT_DIR`; binary `.nvblx`, `.npz`, bag and generated map files
are intentionally excluded from the `WBMM` Git repository.

## MuJoCo first

On the host:

```bash
cd /home/a/WBMM
source install/setup.bash
export MUJOCO_GL=egl
ros2 launch tracer_jaka_bringup mujoco_esdf_sensor.launch.py
```

In the Isaac ROS container:

```bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch my_nvblox_bringup mujoco_esdf.launch.py
```

## Real D455 ESDF

Start the robot localization and SLAM stack first. On the host, start the D455
with frame names connected to the fixed `d455_link` in the robot URDF:

```bash
source /home/a/WBMM/install/setup.bash
ros2 launch tracer_jaka_bringup d455_real.launch.py
```

Then run nvblox inside the Isaac ROS container:

```bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch my_nvblox_bringup d455_esdf.launch.py
```

The host launch uses `camera_name:=d455`, so the RealSense root frame is
`d455_link`, matching the fixed camera link already present in the URDF.
The real launch disables radius-based map clearing and displays a fixed
12 m x 12 m x 3 m ESDF volume centered on the robot's startup pose. This makes
previously integrated space remain visible while the robot turns and moves.

For a larger site, increase the fixed visualization bounds:

```bash
ros2 launch my_nvblox_bringup d455_esdf.launch.py \
  esdf_viz_size_x:=12.0 esdf_viz_size_y:=12.0 \
  esdf_viz_rate:=0.2 esdf_viz_subsampling:=3
```

For local collision planning instead, restore the rolling behavior:

```bash
ros2 launch my_nvblox_bringup d455_esdf.launch.py \
  map_clearing_radius_m:=7.0 esdf_viz_follow_robot:=true \
  esdf_viz_size_x:=4.0 esdf_viz_size_y:=4.0 esdf_viz_rate:=1.0
```

If the D455 driver is already running with other topic names, remap them:

```bash
ros2 launch my_nvblox_bringup d455_esdf.launch.py \
  depth_image_topic:=/camera/depth/image_rect_raw \
  depth_camera_info_topic:=/camera/depth/camera_info \
  color_image_topic:=/camera/color/image_raw \
  color_camera_info_topic:=/camera/color/camera_info
```

In that mode, ensure its depth optical frame is connected to `d455_link`.
The arm-mounted D435 remains available through `d435_sensor.launch.py` and
`d435_esdf.launch.py`, but it is no longer the default ESDF sensor.

## Offline D455 bag ESDF

Copy the complete bag directory recorded on the NUC into the bind-mounted
workspace, for example `/workspaces/isaac_ros-dev/bags/d455_esdf_01`. Then run
one command in the Isaac ROS container:

```bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch my_nvblox_bringup d455_bag_esdf.launch.py \
  bag:=/workspaces/isaac_ros-dev/bags/d455_esdf_01
```

This launch is a lossless offline export pipeline. It uses reliable RGB-D QoS,
a 500-message nvblox TF queue, and disables periodic full-volume visualization
while fusion is running. These settings prevent large image messages from
being silently discarded merely to keep a live display responsive.

For an optional live view, start a separate RViz after mapping. The export does
not depend on RViz topics; it saves the complete internal nvblox layers:

- `/nvblox_node/mesh`: the incrementally fused RGB-D surface mesh;
- `/nvblox_node/esdf_3d_pointcloud`: the bounded 3D ESDF visualization;
- `/map`: the recorded slam_toolbox 2D occupancy map.

When playback finishes, it saves these persistent outputs under the container-
writable bind-mounted directory `/workspaces/isaac_ros-dev/bag_export`:

```text
d455_bag_map.nvblx          native nvblox layer cake
d455_bag_mesh.ply           fused nvblox triangle mesh
d455_bag_remani_esdf.npz   dense ESDF consumed by REMANI
d455_bag_nvblox_timings.txt callback/processed/integrated frame audit
d455_bag_2d.pgm/.yaml      latest recorded 2D SLAM map
```

The NPZ contains `esdf`, `occupancy`, `observed`, `origin`, `voxel_size`, and
`bounds_max`. Unobserved cells are marked occupied by default, so REMANI cannot
plan through camera-unseen space. By default the exported ESDF spans the
**complete mapped scene**: the exporter queries nvblox with `use_aabb=false`,
which returns a dense grid over all allocated ESDF blocks. To restrict the
query to a fixed box instead, pass `esdf_use_aabb:=true` together with the
`esdf_min_*` / `esdf_size_*` arguments:

```bash
ros2 launch my_nvblox_bringup d455_bag_esdf.launch.py \
  bag:=/workspaces/isaac_ros-dev/bags/d455_esdf_01 \
  map_output:=/workspaces/isaac_ros-dev/bag_export/site.nvblx \
  ply_output:=/workspaces/isaac_ros-dev/bag_export/site_mesh.ply \
  timings_output:=/workspaces/isaac_ros-dev/bag_export/site_timings.txt \
  esdf_output:=/workspaces/isaac_ros-dev/bag_export/site_remani.npz \
  map2d_output:=/workspaces/isaac_ros-dev/bag_export/site_2d.yaml
```

Wait for `All synchronized depth inputs were processed`, `Saved nvblox mesh
PLY`, `Saved native nvblox map`, and `Saved REMANI ESDF` before stopping the
launch. The exporter reads the expected depth count directly from bag metadata
and refuses to save if DDS, exact-time synchronization, or TF processing lost
any frame. The timing audit must show `ros/depth_image_callback` equal to
`ros/depth` and `ros/depth/integrate`, within `drain_max_pending_frames` (the
bag's final depth frame legitimately waits forever for TF at its own stamp and
the sim clock stops with the player). Offline fusion raises the depth limit to
1000 Hz so every unique recorded depth timestamp participates in the map.
`decay_tsdf_rate_hz` defaults to `0.0` (decay off); with nvblox's 5 Hz decay a
`static_tsdf` map silently erodes to the last ~30 s of observations.

The launch starts RGB-D nvblox with `use_sim_time=true`, waits five seconds for
its subscribers and TF buffer, then plays the bag with `/clock` at quarter
speed. The exporter node also starts before playback and creates its nvblox
service clients immediately, so DDS discovery is already complete when a
`std_msgs/msg/Bool` trigger is published after playback. This avoids a cold
post-playback discovery round-trip, which once manifested as an intermittent
`Waiting for save_timings service` hang at the end of replay. It uses ROS
domain 21 by default so live domain-20 robot topics cannot mix with replayed
timestamps. Periodic ESDF visualization is disabled during
offline fusion; the final service query exports the ESDF over all allocated
blocks by default (`esdf_use_aabb:=false`), or the configured AABB with
`esdf_use_aabb:=true`. Offline persistent mapping defaults to 10 cm voxels; this uses roughly
one eighth of the 3D voxel count of 5 cm mapping and is intended for 4 GB GPUs.
Real-time local mapping remains at 5 cm.

The NUC recorder and Docker player intentionally use different QoS: recording
accepts the live sensor profiles, while offline playback publishes RGB-D and TF
reliably. nvblox subscribes with its reliable `DEFAULT` profile for this launch.

If a machine still cannot keep up, reduce `rate` further. This changes runtime,
not map extent:

```bash
ros2 launch my_nvblox_bringup d455_bag_esdf.launch.py \
  bag:=/workspaces/isaac_ros-dev/bags/d455_esdf_01 \
  rate:=0.10
```

## Stable interfaces

- Depth input: `depth_image_topic`, `depth_camera_info_topic` launch args.
- Color input: `color_image_topic`, `color_camera_info_topic` launch args.
- TF: `odom -> base_footprint -> base_link -> d455_link -> optical frame`.
- Bounded 3D ESDF visualization cloud:
  `/nvblox_node/esdf_3d_pointcloud` (distance in the `intensity` field).
- Dense planner query: `/nvblox_node/get_esdf_and_gradient`
  (`nvblox_msgs/srv/EsdfAndGradients`).
- Mesh: `/nvblox_node/mesh`.

In 3D mode, upstream nvblox keeps the ESDF as an internal voxel layer and does
not periodically publish `/nvblox_node/static_esdf_pointcloud` (that topic is a
2D ESDF slice). This package therefore queries a bounded 3D volume and
publishes it for RViz. The simulation launch follows `base_footprint`; the real
D455 launch freezes the query center at the startup pose. The RViz profile also
shows the lower-bandwidth fused surface mesh by default.

Visualization load can be adjusted without changing the planner ESDF:

```bash
ros2 launch my_nvblox_bringup mujoco_esdf.launch.py \
  esdf_viz_size_x:=4.0 esdf_viz_size_y:=4.0 esdf_viz_size_z:=3.0 \
  esdf_viz_rate:=1.0 esdf_viz_subsampling:=2 \
  esdf_viz_max_distance:=1.5
```

Increase `esdf_viz_subsampling` to `3` or `4` if RViz/GPU transport is slow.
Set `esdf_viz:=false` to display only `/nvblox_node/mesh`.

The service request `frame_id` must match `global_frame` (default `odom`), or
nvblox intentionally returns an empty grid.

Example query for a 0.3 m cube:

```bash
ros2 service call /nvblox_node/get_esdf_and_gradient \
  nvblox_msgs/srv/EsdfAndGradients \
  "{update_esdf: true, visualize_esdf: true, use_aabb: true, \
  frame_id: odom, aabb_min_m: {x: -0.15, y: -0.15, z: 0.0}, \
  aabb_size_m: {x: 0.3, y: 0.3, z: 0.3}}"
```

The host and container must use the same `ROS_DOMAIN_ID` and DDS
implementation. All provided simulation and D455 launch files now set domain
20 and `rmw_fastrtps_cpp` automatically. Override `ros_domain_id:=...` only if
the rest of the robot uses another domain. Run the container as `admin`, not
`root`; Fast DDS discovery across the bind-mounted host fails for root here.
