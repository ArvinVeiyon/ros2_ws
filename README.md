# ros2_ws

ROS 2 workspace for PX4 rover/drone integration, RC control, sensing, and collision-stop tuning.

## Environment
- OS target: Ubuntu 24.04
- ROS 2 distro: `jazzy` (`/opt/ros/jazzy`)
- Workspace layout:
  - source: `src/`
  - build artifacts: `build/`
  - install space: `install/`
  - logs: `log/`
- Local helper script: `fake_vo.py`

## Branching
- Primary working branch: `main_dev`
- Stable reference branch: `main`
- Latest release branch: `release/2026-02-22`
- Latest release tag: `release-20260222`

## Current Release Alignment (2026-02-22)
- `main_dev` -> `115a91b`
- `release/2026-02-22` -> `115a91b`
- `release-20260222` -> `115a91b`
- Status: working branch, current release branch, and current release tag are aligned to the same commit.

## Workspace Package Map (`src/`)
- PX4 interface and messages:
  - `px4_msgs`
  - `px4_ros_com`
  - `px4-ros2-interface-lib/px4_ros2_cpp`
- Vehicle control modes (C++):
  - `rov_manual` (`rov_manual_cpp`)
  - `rov_ext` (`rov_ext_cpp`)
  - `collision_manual_mode`
  - `rov_collision_stop` (`rov_collision_stop_cpp`)
- Python service/control packages:
  - `rc_control`
  - `tfmini_sensor`
  - `vision_streaming`
  - `optical_flow`
  - `obstacle_distance`
  - `arm_drone`

## Main Runtime Nodes
- `rc_control`:
  - `rc_control_node`
  - `channel_calibrator`
  - `rc_tool`
  - `sync_params`
- `tfmini_sensor`: `tfmini_node`
- `vision_streaming`: `vision_streaming_node`
- `optical_flow`: `optical_flow_node`
- `obstacle_distance`: `obstacle_distance_node`
- `arm_drone`: `arm_disarm`
- `rov_collision_stop_cpp`: `rov_collision_stop` executable
- `collision_manual_mode`: `collision_manual_node` executable
- `rov_manual_cpp`: `rov_manual` executable
- `rov_ext_cpp`: `rov_ext` executable

## Important Config Files
- RC mapping: `src/rc_control/config/rc_mapping.yaml`
- Camera switch params: `src/rc_control/camera_sw_params.yaml`
- Collision tuning: `src/rov_collision_stop/config/tune-20250910-104016.yaml`
- Indoor collision reference: `src/rov_collision_stop/config/indoor_ref.yaml`

## Build And Source
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Quick Verification
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 pkg list | rg "rc_control|tfmini_sensor|vision_streaming|rov_collision_stop_cpp|rov_manual_cpp|rov_ext_cpp|px4_ros_com|px4_msgs"
```

## Current Collision Tuning (2026-02-22)
- File: `src/rov_collision_stop/config/tune-20250910-104016.yaml`
- Updates:
  - `pre_brake_level`: `0.9 -> 0.5`
  - `slow_zone_m`: `2.0 -> 1.5`

## Typical Workflow
```bash
git checkout main_dev
git pull --ff-only
# make changes
git add <files>
git commit -m "your message"
git push origin main_dev
```

## Release Workflow
```bash
# from main_dev after validation
git checkout main
git merge --ff-only main_dev
git push origin main
```
