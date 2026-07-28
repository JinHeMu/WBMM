# ocs2_ws

model predictive control of mobile_manipulator

## build

```bash
colcon build \
  --packages-up-to \
  tracer_jaka_ocs2 \
  tracer_jaka_mujoco \
  tracer_base \
  remani_planner \
  lakibeam1 \
  hipnuc_imu \
  --symlink-install \
  --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## run mujoco sim

```bash
ros2 launch tracer_jaka_ocs2 ocs2_sim.launch.py
```

```bash
ros2 launch remani_planner exp0.launch.py
```



## run real robot

```bash
# start tracer
sudo ip link set can0 up type can bitrate 500000
candump can0
# start imu
sudo chmod 777 /dev/ttyUSB0
ros2 launch hipnuc_imu imu_spec_msg.launch.py
# start lakibeamr
ros2 launch lakibeam1 lakibeam1_scan_view.launch.py
```

```bash
ros2 launch tracer_jaka_ocs2 ocs2_real.launch.py
```

## real robot mapping: wheel odom + Hipnuc + Lakibeam

The mapping launch uses the same public interfaces as the simulation:

- wheel odometry: `/odom`, `odom -> base_footprint`
- IMU: `/IMU_data`, frame `imu_link`
- 2D laser: `/scan`, frame `laser_link`
- fused local odometry: `/odometry/filtered`
- SLAM outputs: `/map`, `/map_metadata`, `map -> odom`

Build and source the relevant packages:

```bash
colcon build --packages-select \
  hipnuc_imu lakibeam1 tracer_base tracer_jaka_mujoco \
  --symlink-install
source install/setup.bash
```

Prepare CAN, IMU serial access and the Ethernet lidar interface. The host
Ethernet interface must be in `192.168.198.0/24` (normally
`192.168.198.1/24`) and the lidar is `192.168.198.2`.

```bash
sudo ip link set can0 up type can bitrate 500000
sudo chmod 777 /dev/ttyUSB0
ping 192.168.198.2
```

For mapping only, one launch starts the Tracer base, Hipnuc, Lakibeam,
robot_localization, slam_toolbox and RViz:

```bash
ros2 launch tracer_jaka_mujoco real_slam.launch.py
```

This applies the Lakibeam settings `30 Hz`, `filter=3`, `45°..315°` by HTTP.
To keep settings already stored in the lidar, use:

```bash
ros2 launch tracer_jaka_mujoco real_slam.launch.py configure_lidar:=false
```

For a USB/RNDIS-connected Lakibeam, use its USB address:

```bash
ros2 launch tracer_jaka_mujoco real_slam.launch.py \
  lidar_sensor_ip:=192.168.8.2
```

If the three hardware drivers are started separately, do not use the
Lakibeam `_view` launch because the SLAM launch already starts RViz:

```bash
ros2 launch hipnuc_imu imu_spec_msg.launch.py
ros2 launch lakibeam1 lakibeam1_scan.launch.py \
  frame_id:=laser_link output_topic0:=/scan configure_sensor:=true
ros2 launch tracer_jaka_mujoco real_slam.launch.py \
  start_imu:=false start_lidar:=false
```

When OCS2 already owns the base driver and robot state publisher, disable
their duplicate `odom -> base_footprint` TF and do not start them again:

```bash
ros2 launch tracer_jaka_ocs2 ocs2_real.launch.py publish_odom_tf:=false
ros2 launch tracer_jaka_mujoco real_slam.launch.py \
  start_base:=false start_robot_state_publisher:=false \
  start_imu:=false start_lidar:=false
```

Before moving, verify timestamps, frames and rates:

```bash
ros2 topic hz /odom
ros2 topic hz /IMU_data
ros2 topic hz /scan
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo base_footprint laser_link
ros2 run tf2_ros tf2_echo base_footprint imu_link
```




