# WBMM Deployment Scripts

This directory contains repeatable real-robot deployment helpers.

Current scope:

```text
deploy/
├── README.md
├── build.sh
├── start.sh
├── poweroff.sh
└── env/
    └── real.env
```

## Environment

Edit `deploy/env/real.env` before first use:

- `WBMM_WS`: workspace root, defaults to the repository root.
- `ROS_DISTRO`, `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`
- `JAKA_IP`, `JAKA_LOCAL_IP`
- `CAN_IFACE`, `CAN_BITRATE`
- `WBMM_BUILD_PACKAGES`
- `WBMM_BUILD_TYPE`

## build.sh

Builds the bringup-related package set:

```bash
./deploy/build.sh
```

It writes:

- a build log under `deploy/logs/`
- `deploy/metadata/deployment_info.txt` with git commit/dirty state

Dry run:

```bash
DRY_RUN=1 ./deploy/build.sh
```

## start.sh

Initializes and validates real hardware:

1. loads `gs_usb` if configured
2. configures Tracer CAN (`can0 @ 500000` by default)
3. validates the CAN interface is `UP`
4. runs `jaka_login` and validates successful initialization

Run:

```bash
./deploy/start.sh
```

Dry run:

```bash
DRY_RUN=1 ./deploy/start.sh
```

This script does **not** start the full WBMM stack. Start hardware/algorithm
launches separately, for example:

```bash
ros2 launch tracer_jaka_bringup wbmm_hardware_interface.launch.py
ros2 launch tracer_jaka_bringup ocs2.launch.py ...
```

## poweroff.sh

Runs JAKA logout/power-off and brings the Tracer CAN interface down:

```bash
./deploy/poweroff.sh
```

Dry run:

```bash
DRY_RUN=1 ./deploy/poweroff.sh
```

## Notes

- `start.sh` and `poweroff.sh` require `sudo` for CAN configuration unless the
  user already has permission.
- The scripts source `/opt/ros/<distro>/setup.bash` and the workspace
  `install/setup.bash`.
- `poweroff.sh` is best-effort: it attempts all shutdown steps and returns
  non-zero if any important step fails.
