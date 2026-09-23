# WBMM Configuration Layout

Configuration is split into three layers:

```text
config/common/   # backend-independent algorithm structure and safe defaults
config/real/     # real-hardware differences only
config/sim/      # MuJoCo simulation differences only
```

Runtime merge order:

```text
common/<name>.yaml
  -> real/<name>.yaml  or  sim/<name>.yaml
  -> explicit launch arguments
```

The later layer overrides earlier values. This keeps shared algorithm and
interface definitions in one place while isolating real-hardware safety,
calibration and simulation-specific tuning.

Common files:

- `interface.yaml`
- `ocs2.yaml`
- `force_control.yaml`
- `ekf.yaml`
- `slam_toolbox.yaml`
- `remani.yaml`
- `moveit_bringup.yaml`

Profile files use the same base names under `real/` and `sim/`.
OCS2 `task.info` files remain separate because they are full text task
definitions, not ROS parameter YAML. The `task_esdf.info` variants enable
`environmentCollision.backend=esdf`; the runtime `esdf_file` parameter can
override the NPZ path without editing the task file. `task_esdf_tracking.info`
is the dual-reference navigation/EE-tracking profile used by
`remani_tracking.launch.py`; that entry now defaults to the map1 ESDF and
runs in the `map` frame (with an identity map->odom alias for the empty
MuJoCo demo). `sim/remani_tracking.yaml` disables REMANI tracking-error
replanning for that demo.
