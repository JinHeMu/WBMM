# ocs2_ws

model predictive control of mobile_manipulator

## build

```bash
colcon build   --packages-up-to tracer_jaka_ocs2  tracer_jaka_mujoco tracer_base remani_planner --symlink-install   --cmake-args -DCMAKE_BUILD_TYPE=Release
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
sudo modprobe gs_usb
cd ~/your_ws/src/ugv_sdk/scripts/
bash setup_can2usb.bash
candump can0
```

```bash
ros2 launch tracer_jaka_ocs2 ocs2_real.launch.py
```





