# Rover Autonomous Navigation — Requirements & Architecture v1

Date: 2026-07-19 | Status: **agreed baseline** | Owner: roz | FC firmware: PX4 pxlabs-v1.17.0-2.0.0 @ a52c38b07d (built 2026-05-31)
Scope decisions (aligned 2026-07-19): **indoor GPS-denied first** · **Nav2 full stack** · **forward-only depth sensing v1**

## 1. Goal

Make the Vind-Roz rover (4WD skid-steer, PX4 1.17 rover-differential airframe) navigate autonomously indoors:
given a goal point, it plans a global route, drives it, and avoids/reroutes around obstacles using the
Orbbec Gemini 336L depth camera — all commanded through the **px4-ros2-interface-lib** (control interface
out, navigation interface in). Maps to autonomy roadmap phase 2 + first half of phase 3/4.

## 2. Architecture

```
                    ┌────────────────────── RPi5 companion ──────────────────────┐
Orbbec 336L ──USB3──► OrbbecSDK_ROS2 ─depth─► depthimage_to_laserscan ─/scan─┐   │
                    │                                                        ▼   │
                    │   VESC ERPM (/fmu/out/esc_status)                   Nav2   │
                    │        │                                    (slam_toolbox: │
                    │        ▼                                     map, global   │
                    │   rover_odometry ──/odom + TF odom→base_link─► planner,    │
                    │        │                                     local costmap,│
                    │        ▼                                      controller)  │
                    │   px4_ros2 NAVIGATION interface                    │       │
                    │   (LocalPositionMeasurement → EKF2)             cmd_vel    │
                    │                                                    ▼       │
                    │   px4_ros2 CONTROL interface ◄──── nav2_px4_bridge node    │
                    │   (custom mode "AutoNav", TrajectorySetpoint vel+yawrate)  │
                    └───────────────────────────│─────────────────────────────────┘
                                          DDS ttyAMA4
                                                ▼
                                     PX4 fmu-v6xrt (rover diff) ──UAVCAN──► 4× VESC
```

- **Nav2 is the brain** (routing, costmaps, avoidance); **PX4 stays the muscle + safety authority**
  (arming, RC override, failsafes, motor control).
- PX4 receives our position estimate (navigation interface) so its own failsafes and mode logic stay sane
  indoors; Nav2 consumes `/odom` + TF directly.

## 3. Requirements

### R1 — Localization (indoor, GPS-denied)
- `rover_odometry` package (NEW): subscribes `/fmu/out/esc_status`, computes differential odometry
  (full math, verified constants, and ESC address map already documented in memory `rover_odometry.md`:
  ERPM→m/s ×0.000380, track 0.43 m, left={11,13} right={10,12}), publishes `/odom` (nav_msgs) + TF `odom→base_link`. ≥20 Hz.
- Feed EKF2 via px4_ros2 `LocalPositionMeasurementInterface` (lib v1.6.0, present in ws) so PX4 local
  position becomes valid indoors. PX4 side: enable EV fusion (`EKF2_EV_CTRL`), disable GPS gating
  (`EKF2_GPS_CTRL=0` indoor profile), heading from odom yaw (mag unreliable indoors).
- `slam_toolbox` (apt 2.8.2) over depth-derived `/scan` provides `map→odom` correction + the global map.
  Acceptance: return-to-start error < 0.3 m over a 20 m indoor loop.

### R2 — Perception
- OrbbecSDK_ROS2 wrapper (NEW install, todo #7) on the 336L (`usbcam-2bc50807-…`, USB3 BOX-B, role NAV-COLOR
  color is locked for future vision — autonomy uses **depth stream only**, never ffmpeg, per camera rule).
- Depth 640×400 @ 15 Hz → `depthimage_to_laserscan` (apt, installed) → `/scan` for costmap + SLAM.
  3D/voxel layer deferred (CPU budget).
- Coverage: forward ~90° H only + VL53L1X front ToF. **Planner must never command reverse into unseen
  space**: Nav2 configured with no reverse velocities; recovery behaviors limited to in-place rotation
  (skid-steer can turn on the spot) and forward re-plan.

### R3 — Planning (Nav2 1.3.5, apt)
- Global: planner server (NavFn/Smac 2D) on slam_toolbox map → auto-routing A→B.
- Local: controller server (start DWB; evaluate MPPI later) + local costmap (rolling, 4×4 m, obstacle
  layer from `/scan`, inflation ≥ rover half-width 0.30 m + margin).
- Footprint: rectangle of real chassis (measure; track 0.43 m + wheel width). Max speed indoor 0.8 m/s,
  max yaw rate 1.0 rad/s (below PX4 `RO_YAW_RATE_LIM`).

### R4 — Control bridge (px4_ros2 control interface)
- `nav2_px4_bridge` node (NEW, C++, follows lib examples): registers custom PX4 mode **"AutoNav"**
  selectable from RC/QGC; converts Nav2 `cmd_vel` → `TrajectorySetpoint` (body velocity + yaw rate).
- FC firmware is PX4 1.17 (pxlabs) and exposes the native rover setpoint topics → preferred path is
  interface-lib 2.1.1 (native rover setpoint types) or a backported rover setpoint-type class on 1.6.1;
  **M0 bench task decides** (build + runtime messageCompatibilityCheck + wheels-up response test).
  Fallbacks in order: `TrajectorySetpoint` vel+yawspeed, then `DirectActuators` (left/right normalized
  outputs, loop closed with wheel odom). Decision recorded after bench test.
- PX4 rover params must be set first (QGC/NuttShell, NOT pymavlink): `RD_WHEEL_TRACK=0.43`,
  `RO_MAX_THR_SPEED≈3.0`, `RO_SPEED_P/I`, `RO_YAW_RATE_P/I/LIM`, `RO_YAW_P` (all were 0 on 2026-05-30 dump).

### R5 — Safety (non-negotiable, each independently)
1. RC mode switch out of AutoNav = instant manual authority (PX4-native, no companion code in path).
2. `rov_collision_stop` (existing C++ node, VL53L1X) stays active as last-line forward e-stop, independent of Nav2.
3. Bridge watchdogs: `cmd_vel` older than 500 ms → zero setpoint; depth/`/scan` older than 1 s → controlled stop.
4. Nav2 costmap keeps ≥ 0.5 m stopping buffer at max speed.
5. Disarm on companion crash: PX4 offboard timeout behavior verified in M4 bench test.

### R6 — Compute budget (RPi5 8GB, shared with vision streaming x264)
- Total new load target ≤ 2.0 cores steady-state: Orbbec+laserscan ≤ 0.6, SLAM ≤ 0.5, Nav2 ≤ 0.6, bridge+odom ≤ 0.1.
- Measure at every milestone (`top`, `ros2 topic hz`); if exceeded: drop depth to 10 Hz / 320×200 before
  touching planner rates. FPV streaming must stay glitch-free (user-facing).

### R7 — Frames & interfaces
- TF: `map → odom → base_link → camera_link` (+ static camera mount transform, measure on rover).
- All `/fmu/*` topics BEST_EFFORT QoS (existing convention). REP-105 frame names, REP-103 conventions
  (PX4 NED ↔ ROS ENU conversion handled by px4_ros2 lib).

## 4. Build order — layered environment-first flow (re-ordered 2026-07-19 per roz)

Principle: prove each layer of the stack with QGC in the loop before building the next on top.

| L | Deliverable | Done when |
|---|---|---|
| L0 | **Transport + message alignment**: uXRCE-DDS + MAVLink links verified; px4_msgs = exact firmware match; ws rebuilt | all /fmu topics flow on the new message set; QGC connects |
| L1 | **Custom mode skeleton**: px4_ros2 "AutoNav" mode registers (lib 2.1.1 vs 1.6.1 decided here), messageCompatibilityCheck passes, arming checks wired | mode visible + selectable in QGC, no motion |
| L2 | **Mode I/O**: inputs (`cmd_vel`) / outputs (rover speed + yaw-rate setpoints) defined; wheels-up bench response test | commanded speed/yaw → correct wheel response, monitored in QGC |
| L3 | **Sensor fusion layer-by-layer**: `rover_odometry` → `/odom` + TF (pkg written, bench-verified constants); EKF2 feed via LocalPositionMeasurementInterface; TFmini/VL53L1X confirmed | hand-push 2 m → /odom 2 m ± 5 %; QGC shows xy_valid=true indoors |
| L4 | **Gemini SDK**: OrbbecSDK_ROS2 install; enumerate its topics; depth → `/scan` | /scan ranges correct vs tape measure, ≥ 10 Hz |
| L5 | Nav2 odom-frame: local costmap + point-goal with avoidance | 5 m goal around a placed obstacle, 5/5 runs |
| L6 | slam_toolbox map + global planner | A→B with auto-routing + re-route on new obstacle |
| L7 | Tuning + safety validation checklist pass | all R5 items demonstrated + logged |

Each layer lands as its own commit(s) on `main`; tag `v1.2.0` when L7 passes.

## 5. Status log
- **2026-07-19 (M0 in progress):** FC firmware identity corrected → pxlabs-v1.17.0-2.0.0 @ a52c38b; firmware source fetched to companion (remote `pxlabs`, branch `pxlabs-fw`). px4_msgs re-pinned to release/1.17 @ 86d8239 (branch `pinned-pxlabs-1.17`) = **static full match vs firmware** (old pin had ArmingCheckRequest v0≠v1 — M4 breaker, caught). interface-lib 1.6.1; 2.1.1 bench test pending. Workspace rebuild running. M1 pre-verified on bench: all 4 VESCs online 50 Hz, sign map {10:−1, 11:+1, 12:+1, 13:+1} (hand-spin test), idle noise ±35 ERPM → deadband ±40.

- **2026-07-20 — L0/L1/L3 closed, L2 partially closed. Layer order changed: L3 now precedes L2.**

  *Why the order changed.* L2 (wheels-up motion) cannot run before L3. Arming in AutoNav was refused
  (`VehicleCommandAck` result 1 TEMPORARILY_REJECTED, `pre_flight_checks_pass=false`) because
  `v_xy_valid=false`. Decoding `failsafe_flags` for nav_state 23 shows AutoNav requires
  `mode_req_local_position` + `mode_req_local_alt` + `mode_req_angular_velocity`, and **every** rover
  setpoint type in interface-lib release/1.17 (`speed_rate`, `throttle_rate`, `throttle_steering`,
  `speed_steering`, `throttle_attitude`) sets `config.velocity_enabled = true` in `getConfiguration()`.
  There is no open-loop bench shortcut through this library — a velocity estimate is mandatory.

  *L1 closed.* `autonav_mode` registers cleanly (messageCompatibilityCheck passes,
  `RegisterExtComponentReply`, arming-check replies flow). AutoNav = **External Mode 1**, nav_state 23
  (`COM_MODE0_HASH = -1639016601` = fnv1a("AutoNav")). Mode entry verified repeatedly.

  *Mode control moved off MAVLink onto DDS.* Injecting MAVLink into mavlink-router destabilises the
  px4_ros2 4 s FMU watchdog. Mode switching now uses `VehicleCommand` DO_SET_MODE on
  `/fmu/in/vehicle_command` (`tools/dds_setmode.py`), verified nav_state 4 → 23 → 4.

  *QGC "Unknown mode" is not a QGC bug.* The PXLABS QGC fork already has the full upstream wiring
  (`StandardModes.cc:96` → `PX4FirmwarePlugin::updateAvailableFlightModes` →
  `FirmwarePlugin::_updateFlightModeList`, which rebuilds `_modeEnumToString`). The FC streams
  AVAILABLE_MODES_MONITOR (436) at ~0.5 Hz and answers REQUEST_MESSAGE(435) with all 26 modes —
  index 19 is `AutoNav`, custom_mode `0x0b040000`, user-selectable. The real cause is the GCS uplink:
  commands injected at the relay reach the drone **0 of 8** times (6/6 locally), and downlink delivers
  only ~15 % of offered telemetry (176 kbit/s offered → 26 kbit/s at the relay, uniform across all
  message types). QGC's mode-name request never lands and `StandardModes` has no retry.

  *Arming blockers found and cleared.* (1) `Accels inconsistent` — accel instance 0 off by
  1.006 m/s² vs `COM_ARM_IMU_ACC` (0.7). Fixed by **quick accel calibration**, the one-position
  procedure for vehicles too large to rotate: `commander calibrate accel quick`, or
  `VehicleCommand` 241 with **param5 = 4** (the in-source comment saying "param5 = 3" is wrong; the
  code sends and checks 4). It writes offsets only. Alternatives for big vehicles: calibrate the FC
  off-vehicle then run level-horizon (param5 = 2), or deprioritise the bad instance
  (`CAL_ACCn_PRIO = 0`). Note **accel instance numbering is not stable** across boots/firmware.
  (2) `v_xy_valid=false` — cleared by L3 below.

  *L3 closed.* `rover_odometry` had a bug that made it discard every wheel: it compared
  `msg.timestamp` (absolute — uXRCE-DDS offsets only the top-level field) against nested
  `esc[i].timestamp` (raw boot-relative hrt), so every ESC looked impossibly stale. Fixed to measure
  staleness against the newest nested stamp and gate on `esc_online_flags`. `/odom` now publishes at
  99.9 Hz with `odom → base_link` TF. New package **`rover_ekf_bridge`** feeds that velocity to EKF2
  through `LocalPositionMeasurementInterface` → `/fmu/in/vehicle_visual_odometry` at ~40 Hz, velocity
  only (wheel-integrated position drifts unbounded), frame BodyFRD. Two gotchas: EKF2 drops the whole
  sample unless the velocity vector is all-finite (`ev_vel_control.cpp`: `vel.isAllFinite()`) and the
  lib fills unset fields with NaN, so **`velocity_z` must be sent explicitly** (0 for a ground rover);
  and inbound `timestamp_sample` *is* time-offset corrected by the FC, so ROS-clock stamps are correct.
  With `EKF2_EV_CTRL = 4` (bit 2, 3D velocity — default 0 = EV disabled): `cs_ev_vel` true,
  **xy_valid + v_xy_valid true, dead_reckoning false**, preflight passes, `cs_valid_fake_pos` gone.

  *L2 partial.* First armed run succeeded: AutoNav entered, vehicle armed, `/cmd_vel` reached the
  wheels, the 500 ms watchdog zeroed them, auto-disarm and Hold restore clean. Yaw drove all four
  wheels in the correct differential pattern. **Forward did not** — only addr 13 turned (~430 ERPM)
  and it did not scale between 0.2 and 0.4 m/s. A Manual-mode RC test then drove **all four wheels
  both directions at ±1500-1580 ERPM (±0.58-0.60 m/s)**, proving motors, ESCs and L/R allocation are
  good and confirming the addr-10 sign inversion. So the forward anomaly is in the closed-loop speed
  path, not hardware — and it is confounded: on stands both the speed and yaw-rate loops close on body
  motion that cannot occur (yaw ran ~9× the geometric expectation, classic wind-up), while
  `rover_ekf_bridge` was simultaneously feeding EKF2 wheel-derived fiction. **Treat the forward result
  as void; re-test on the floor.** For any future wheels-up bench work, stop `rover_ekf_bridge` first.

  *RC mapping observed:* ch2 = forward/reverse throttle (1023-1981), ch4 = steering (1116-1671),
  ch3 static 1001 (unused), ch1 static. `RC_MAP_FLTMODE` is still 0 — no mode channel mapped.

  *Tools added* (`tools/`): `dds_setmode.py`, `l2_watch.py`, `l2_test.py` (refuses to arm without
  `--wheels-are-up`, always disarms and restores Hold), `manual_drive_log.py`.

  *Forward-drive root cause found (`param show RO_*`, NuttShell).* **`RO_SPEED_LIM = 0.01` m/s.**
  `DifferentialSpeedControl.cpp:119` clamps every setpoint with
  `math::constrain(_speed_setpoint, -RO_SPEED_LIM, +RO_SPEED_LIM)`, so 0.2 and 0.4 m/s both collapse to
  0.01 m/s — exactly why the two runs gave identical 430/431 ERPM and why only the least-loaded wheel
  moved while the others stayed below break-away torque. The parameter's own doc calls it "Speed limit
  — used to cap speed setpoints", default -1 (disabled); 0.01 is a mis-set, saved value. Fix:
  `param set RO_SPEED_LIM 1.0` (just above autonav_mode's own 0.8 m/s clamp, keeping a hard FC-side cap
  as defence in depth; 3.0 would match `RO_MAX_THR_SPEED`) then `param save`, and retest on the floor.

  Everything else in the group is sane: `RO_MAX_THR_SPEED` 3.0, `RO_SPEED_P` 0.5, `RO_SPEED_I` 0.1,
  `RO_YAW_P` 2.0, `RO_YAW_RATE_P` 2.0, `RO_YAW_RATE_I` 0.1, `RO_YAW_RATE_LIM` 1.57; accel/decel/jerk
  limits all -1 (disabled). The yaw path is never clamped, which is why yaw drove all four wheels.

  *Parameters are MAVLink-only.* They are not exposed over DDS in any form, so reading or setting them
  needs NuttShell or the PARAM protocol. Budget for link disturbance: after several `mavlink_shell.py`
  sessions the FC heartbeat disappeared from `tcp:127.0.0.1:5760` (only a GCS-type heartbeat left) and
  param reads stopped answering. DDS was entirely unaffected throughout, so the FC was healthy and
  autonav work continued — but QGC cannot connect until `mavlink.router` is restarted. Stop
  `autonav_mode` before any MAVLink work so its 4 s watchdog cannot abort mid-session.

  *Open items:* apply the `RO_SPEED_LIM` fix and retest forward **on the floor**; restart
  `mavlink.router`; fix the GCS uplink; map `RC_MAP_FLTMODE` (still 0 — observed sticks: ch2 =
  forward/reverse, ch4 = steering, ch3 unused); run `autonav_mode` as a systemd unit with
  `Restart=always` (its 4 s FMU watchdog has aborted repeatedly, including with no MAVLink load).

  *L4/L5 prerequisites audited 2026-07-20 — none installed yet.* The Gemini 336L itself is ready:
  enumerated as `2bc5:0807` at **USB3 5000 Mbps**, device nodes unheld, and `vision_streaming` is on
  the LG FPV camera so there is no contention. Missing: **OrbbecSDK_ROS2 entirely** (no source, and no
  Orbbec udev rules — without the rule the wrapper cannot claim the device unprivileged; use the
  v2-main line for Gemini 330-series on Jazzy, cloned over SSH since companion HTTPS to GitHub hangs on
  IPv6), **Nav2 1.3.5**, **slam_toolbox 2.8.2**, and seven build deps (`nlohmann-json3-dev`,
  `libgflags-dev`, `ros-jazzy-camera-info-manager`, `ros-jazzy-diagnostic-updater`,
  `ros-jazzy-image-publisher`, `ros-jazzy-backward-ros`, `ros-jazzy-xacro`).
  `depthimage_to_laserscan` 2.5.1 is already present. Watch disk: **82% used, 11 GB free of 58 GB**.

## 6. Out of scope for v1
360° sensing, outdoor/GPS mode switching, YOLO/semantic perception (phase 5), multi-goal missions and
LLM mission brain (phase 6), rough-terrain traversability. Design keeps them pluggable (extra costmap
layers / behavior-tree nodes / localization sources).
