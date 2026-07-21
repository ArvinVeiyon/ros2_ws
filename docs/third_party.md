# Third-party sources not vendored in this repo

These live under `src/` but are **gitignored**, because vendoring them would add hundreds of
megabytes and bury our own history. Recorded here so a workspace can be reproduced exactly.
Promote any of them to a real git submodule when convenient.

## OrbbecSDK_ROS2 — Gemini 336L camera wrapper

| | |
|---|---|
| Path | `src/OrbbecSDK_ROS2/` |
| Repo | https://github.com/orbbec/OrbbecSDK_ROS2.git |
| Commit | `ec6bc228b79656449bea289f2967a2f44ce52c57` |
| Size | ~201 MB working tree |
| SDK | 2.9.3 |
| Built | Release, on the RPi5 companion (~8 min for `orbbec_camera`, plus `_msgs`, `_description`) |

Reproduce:

```bash
cd ~/ros2_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
cd OrbbecSDK_ROS2 && git checkout ec6bc228b79656449bea289f2967a2f44ce52c57
cd ~/ros2_ws && colcon build --packages-select orbbec_camera_msgs orbbec_description orbbec_camera
```

Companion HTTPS→GitHub can hang on IPv6. If it does, fetch over SSH instead:

```bash
git -c url."git@github.com:".insteadOf="https://github.com/" clone https://github.com/orbbec/OrbbecSDK_ROS2.git
```

Requires the udev rule `99-obsensor-libusb.rules` (already installed on this companion).

### Notes for our use

- The wrapper publishes its **own TF tree** rooted at `camera_link`
  (`camera_link → camera_depth_frame → camera_depth_optical_frame`, −90/−90 optical rotation).
  Do **not** re-publish those frames; we only add the physical mount
  `base_link → camera_link` (see `launch/depth_to_scan.launch.py`).
- Launch with the **IMU enabled** — `enable_accel:=true enable_gyro:=true`. Both default to off,
  and without them `/camera/accel/sample` does not exist. The IMU is what let the camera mount
  pitch/roll be *measured* rather than guessed, and makes re-checking the mount a 30-second job.
- A `_low_cpu` launch variant exists if CPU gets tight.
- `rover-camera.service` (see `systemd/install_rover_units.sh`) runs this on boot.

## ldlidar_stl_ros2

`src/ldlidar_stl_ros2/` is a nested git repo, deliberately left untracked. The STL-19 hardware was
moved to another team on 2026-04-17; the package is kept only for reference.
