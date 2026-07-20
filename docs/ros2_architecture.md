# ROS2 Workspace Architecture — Vind-Roz Companion

Date: 2026-07-19 (updated 2026-07-20) | Companion: RPi5 Ubuntu 24.04 / ROS2 Jazzy | FC: PX4 **pxlabs-v1.17.0-2.0.0** @ `a52c38b07d`, built 2026-05-31 (fmu-v6xrt, verified via NuttShell `ver all`)
Companion ↔ FC: uXRCE-DDS over ttyAMA4 @ 921600 (microxrce-agent.service)

## 1. PX4 library state (audited + updated 2026-07-19)

| Repo (src/) | Pinned at | Policy | Why |
|---|---|---|---|
| `px4_msgs` | **`86d8239`** (release/1.17, branch `pinned-pxlabs-1.17`) | **PINNED — statically verified** | Re-pinned 2026-07-19: `check-message-compatibility.py` vs the real firmware (`pxlabs-fw` @ a52c38b) is a **full exact match**. The old `d2c9ff2` pin had `ArmingCheckRequest` v0 (2 fields) vs firmware v1 — it would have broken px4_ros2 mode registration. |
| `px4-ros2-interface-lib` | **release/1.17 `4a3370f`** (branch `pinned-1.17`) | match the firmware's release line | Has the native rover setpoint types (`RoverSpeedRateSetpointType`) and the `rover_velocity` example. **2.1.1 FAILS to build** against 1.17 msgs (uses `ConfigOverrides.disable_auto_set_home`, needs px4_msgs > 1.17) → all lib 2.x is blocked until a firmware upgrade. Local example experiments preserved on branch `local/manual-mode-experiments`. |
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

### Built and running (rover autonav, 2026-07-20)
```
rover_odometry/wheel_odometry_node ◄─ /fmu/out/esc_status ─► /odom @99.9Hz + TF odom→base_link
rover_ekf_bridge  ◄─ /odom ─► px4_ros2 LocalPositionMeasurementInterface
                             ─► /fmu/in/vehicle_visual_odometry @~40Hz (velocity only, BodyFRD)
                             ─► EKF2  [needs EKF2_EV_CTRL=4]  ⇒ xy_valid + v_xy_valid TRUE
autonav_mode ◄─ /cmd_vel ─► RoverSpeedRateSetpointType ─► PX4 "AutoNav" = External Mode 1 (nav_state 23)
                             clamps 0.8 m/s / 1.0 rad/s, 500 ms watchdog, zero-on-activate
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
| `vehicle_local_position_v1` | bridge/monitoring | **49.8 Hz** | ✅ **fixed 2026-07-20**: with `rover_ekf_bridge` + `EKF2_EV_CTRL=4` → `xy_valid=true, v_xy_valid=true, dead_reckoning=false`, `cs_ev_vel` set (was all-false / fake-pos before) |
| `vehicle_status_v1` | bridge (arming/mode) | 2.0 Hz | ✅ |
| `input_rc` | RC override monitoring | 9.6 Hz | ✅ RC link up |
| `sensor_combined` | optical_flow | 100.4 Hz | ✅ |
| `battery_status_v1` | failsafe/RTH later | 1.0 Hz | ✅ |
| `register_ext_component_reply`, `mode_completed`, `vehicle_command_ack` | px4_ros2 mode registration | event-driven | present |

### Companion → FC (`/fmu/in/`) — verified exposed by firmware
| Topic | Used by | Status |
|---|---|---|
| `rover_speed_setpoint`, `rover_rate_setpoint`, `rover_attitude_setpoint`, `rover_position_setpoint`, `rover_throttle_setpoint`, `rover_steering_setpoint` | **nav2_px4_bridge (M4)** | ✅ **firmware exposes full rover-rework setpoint set** (PX4 1.17 feature); all `Rover*Setpoint` msgs present in pinned px4_msgs (incl. `RoverVelocitySetpoint`) |
| `vehicle_visual_odometry` | **`rover_ekf_bridge` (L3)** | ✅ **in use** — EV velocity aiding. Note: EKF2 drops the whole sample unless the velocity vector is all-finite, and the lib NaN-fills unset fields → `velocity_z` must be sent (0 for a ground rover) |
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
3. ✅ done 2026-07-20 in part: rover params set via NuttShell; `EKF2_EV_CTRL=4` set and verified.
   Still to inspect: `RO_MAX_THR_SPEED`, `RO_SPEED_P/I`, `RO_YAW_RATE_P/I` (params are **not** readable
   over DDS — needs QGC or NuttShell).
4. ⚠️ partly done 2026-07-20: armed in AutoNav, `/cmd_vel` reaches the wheels, watchdog verified, yaw
   drives all four correctly — but forward drove only one wheel and did not scale. Manual RC test drove
   all four both directions, so hardware is good and the fault is in the closed-loop speed path.
   **A wheels-up bench cannot validate these loops** (speed and yaw-rate close on body motion that
   cannot happen) — retest forward on the floor, and stop `rover_ekf_bridge` during any wheels-up test
   so wheel odometry does not feed EKF2 motion that is not occurring.
