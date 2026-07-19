# ROS2 Workspace Architecture — Vind-Roz Companion

Date: 2026-07-19 | Companion: RPi5 Ubuntu 24.04 / ROS2 Jazzy | FC: PX4 **pxlabs-v1.17.0-2.0.0** @ `a52c38b07d`, built 2026-05-31 (fmu-v6xrt, verified via NuttShell `ver all`)
Companion ↔ FC: uXRCE-DDS over ttyAMA4 @ 921600 (microxrce-agent.service)

## 1. PX4 library state (audited + updated 2026-07-19)

| Repo (src/) | Pinned at | Policy | Why |
|---|---|---|---|
| `px4_msgs` | `d2c9ff2` (main snapshot 2025-07-29, 1.17-dev era) | **PINNED — runtime-proven** | FC runs a custom pxlabs 1.17 fork whose source is NOT on the companion → no static check possible. Compatibility is proven at runtime (DDS negotiated, all topics flow correctly, versioned `_v1` topics + rover topics present). Change only with on-bench re-verification; the lib's `messageCompatibilityCheck` gates every mode registration. |
| `px4-ros2-interface-lib` | **v1.6.1** `fd0108a` (updated from 1.6.0 today) | newest 1.x; compiles clean vs pinned px4_msgs (verified) | 1.6.1 adds ZYX quaternion→euler fix. **Since FC = PX4 1.17, lib 2.1.1 (native rover setpoint types) is a live candidate — M0 bench task: build 2.1.1 vs pinned px4_msgs + runtime compat check; fall back to 1.6.1 if it fails.** Local example experiments preserved on branch `local/manual-mode-experiments`. |
| `px4_ros_com` | `6d6fce9` (main + 1 local commit) | current (0 behind upstream) | frame conversion utils + examples. |
| `ldlidar_stl_ros2` | v3.0.3 + 2 mandatory local fixes | **never clobber** | pthread include + hardcoded ttyAMA3 (see memory); STL-19 hardware with other team. |

Fetch note: companion HTTPS to GitHub hangs (IPv6 issue) — fetch with
`git -c url."git@github.com:".insteadOf="https://github.com/" fetch` (SSH works).

### Firmware note (corrected 2026-07-19)
Earlier records said v1.16.0-rc1 @ c5b8445 — wrong; the FC was reflashed 2026-05-31 with the pxlabs
1.17 build. `~/PX4-Autopilot` (@ c5b8445) is an upstream clone used for tools/reference only, NOT the
firmware source. Getting the pxlabs fork checked out on the companion would enable static message checks.

## 2. Node graph

### Existing (running today)
```
PX4 fmu-v6xrt ◄─uXRCE-DDS ttyAMA4─► microxrce-agent ─► /fmu/in|out/* topics
  tfmini_node ──────────────► /fmu/in/distance_sensor        (TFmini ttyAMA2, 50Hz)
  obstacle_distance_publisher ► /fmu/in/obstacle_distance    (VL53L1X I2C, 10Hz, front 0–25°)
  optical_flow_node ─────────► /fmu/in/sensor_optical_flow   (manual launch only)
  rc_control_node ◄──────────  /fmu/out/input_rc             (cam switch CH9, shutdown CH10)
  rov_collision_stop ◄───────  VL53L1X                       (emergency stop, C++)
  vision_streaming_node ─────► RTP→WFB→GS                    (FPV, ffmpeg — NOT autonomy path)
```

### Planned (rover autonav, per docs/rover_autonav_requirements.md)
```
OrbbecSDK_ROS2 ─depth─► depthimage_to_laserscan ─/scan─► Nav2 + slam_toolbox
rover_odometry ◄─ /fmu/out/esc_status ; ─► /odom + TF odom→base_link ─► Nav2
             └────► px4_ros2 LocalPositionMeasurementInterface ─► EKF2 (fixes xy_valid)
Nav2 ─cmd_vel─► nav2_px4_bridge (px4_ros2 custom mode "AutoNav") ─► /fmu/in/rover_speed_setpoint + rover_attitude/rate
```

## 3. Required topics — live status (measured 2026-07-19, FC connected, VESCs powered)

### FC → companion (`/fmu/out/`)
| Topic | Needed by | Rate measured | Status |
|---|---|---|---|
| `esc_status` | rover_odometry (M1) | **49.7 Hz** | ✅ all 4 VESCs publishing |
| `vehicle_odometry` | monitoring EKF2 | **98.6 Hz** | ⚠️ `quality=0`, pose_frame=1 (NED) — expected indoors, fixed by M2 |
| `vehicle_local_position_v1` | bridge/monitoring | **49.8 Hz** | ⚠️ `xy_valid=false, v_xy_valid=false, heading_good_for_control=false, dead_reckoning=true` — the M2 target; `z_valid=true` |
| `vehicle_status_v1` | bridge (arming/mode) | 2.0 Hz | ✅ |
| `input_rc` | RC override monitoring | 9.6 Hz | ✅ RC link up |
| `sensor_combined` | optical_flow | 100.4 Hz | ✅ |
| `battery_status_v1` | failsafe/RTH later | 1.0 Hz | ✅ |
| `register_ext_component_reply`, `mode_completed`, `vehicle_command_ack` | px4_ros2 mode registration | event-driven | present |

### Companion → FC (`/fmu/in/`) — verified exposed by firmware
| Topic | Used by | Status |
|---|---|---|
| `rover_speed_setpoint`, `rover_rate_setpoint`, `rover_attitude_setpoint`, `rover_position_setpoint`, `rover_throttle_setpoint`, `rover_steering_setpoint` | **nav2_px4_bridge (M4)** | ✅ **firmware exposes full rover-rework setpoint set** (PX4 1.17 feature); all `Rover*Setpoint` msgs present in pinned px4_msgs (incl. `RoverVelocitySetpoint`) |
| `vehicle_visual_odometry` | navigation interface (M2) | ✅ exposed |
| `trajectory_setpoint`, `goto_setpoint` | fallback control path | ✅ exposed |
| `register_ext_component_request`, `config_overrides_request`, `arming_check_reply_v1`, `mode_completed`, `vehicle_command`, `offboard_control_mode` | px4_ros2 lib internals | ✅ exposed |
| `distance_sensor`, `obstacle_distance`, `sensor_optical_flow` | existing sensor nodes | ✅ in use today |

## 4. M4 bridge design note (updated by today's findings)

Firmware (PX4 1.17) accepts native rover setpoints. Preferred: **upgrade lib to 2.1.1** (native rover
setpoint types + `rover_velocity` example) — M0 bench task: build vs pinned px4_msgs + runtime
`messageCompatibilityCheck`. Plan B: stay on 1.6.1 and backport the small rover setpoint-type class
(`setpoint_types/experimental/rover/`) into `nav2_px4_bridge`. cmd_vel mapping either way:
`linear.x` → RoverSpeedSetpoint, `angular.z` → RoverRateSetpoint.
Fallback if the pxlabs rover module rejects them: `TrajectorySetpoint` velocity+yawspeed, then `DirectActuators`.

## 5. Gaps to close (= milestone M0 remainder)
1. Install Nav2 1.3.5 + slam_toolbox 2.8.2 (apt, verified available).
2. Install OrbbecSDK_ROS2 wrapper (Gemini 336L, depth-only for autonomy).
3. Set PX4 rover params via QGC/NuttShell (`RO_*`, `RD_WHEEL_TRACK=0.43` — all were 0 on 2026-05-30 dump) + EKF2 indoor profile (`EKF2_EV_CTRL`, `EKF2_GPS_CTRL`).
4. Bench test (wheels up): AutoNav skeleton mode sends rover setpoints → wheels respond.
