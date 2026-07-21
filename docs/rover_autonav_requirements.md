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

- **2026-07-21 — L4 closed. L5 unblocked.**

  *Everything the 2026-07-20 audit listed as missing is now installed:* Nav2 **1.3.12** +
  `nav2-bringup`, `slam_toolbox` **2.8.5** (both newer than the audited candidates), all seven build
  deps, and the Orbbec udev rule `99-obsensor-libusb.rules`. `OrbbecSDK_ROS2` is cloned to
  `src/OrbbecSDK_ROS2` @ `ec6bc22` and built Release — `orbbec_camera` (~8 min), `orbbec_camera_msgs`,
  `orbbec_description`. **It is still untracked in git**; pin it as a submodule or gitignore it,
  otherwise a fresh clone of this workspace cannot build.

  *Camera bring-up.* `ros2 launch orbbec_camera gemini_330_series.launch.py` (the 336L is a 330-series
  part; a `_low_cpu` variant exists if CPU gets tight). SDK 2.9.3 over **USB3.2**, depth 848x480@30
  Y16, color 1280x720@30 MJPG, init 1.5 s. Topics: `/camera/depth/image_raw`, `/camera/depth/points`,
  `/camera/color/image_raw{,/compressed}`, `camera_info`, `device_status`. The wrapper **publishes its
  own TF tree** rooted at `camera_link` (→ `camera_depth_frame` → `camera_depth_optical_frame`, with
  the −90/−90 optical rotation), so do not re-publish those frames — the only transform the integrator
  owes it is the physical mount.

  *`/scan` pipeline.* `launch/depth_to_scan.launch.py` — deliberately a **standalone launch file, not
  a package**, run by path until proven. It adds exactly two things: the `base_link → camera_link`
  static TF and `depthimage_to_laserscan_node` (`scan_height` 40, `scan_time` 0.033, range 0.3–8.0 m,
  `output_frame` `camera_depth_frame`). **Verified live: /scan at 20–21 Hz**, FOV ±0.80 rad (~92°,
  matching the 336L), real returns. Meets the L4 rate bar (≥ 10 Hz) with margin.

  *L4 acceptance is only half-met.* The bar is "ranges correct **vs tape measure**" — rate and
  plausibility are confirmed, the metric check is not, and the mount TF is a **placeholder**
  (`cam_x=0.20 cam_y=0.00 cam_z=0.15`, zero rpy, exposed as launch args). Nav2 builds its costmap from
  `/scan` transformed through that, so an unmeasured mount puts obstacles in the wrong place and the
  planner drives into them. Measure `camera_link` relative to `base_link` before L5. If the camera is
  pitched down at all, revisit `scan_height`: a downward tilt makes the ground register as an obstacle
  band.

  *Throughput note.* Depth delivers ~24 Hz against a configured 30, and `/scan` ~20 Hz — suspect CPU/USB
  contention with `vision_streaming` on the LG FPV camera (R6 compute budget). Not blocking; revisit if
  Nav2 costmap updates lag.

  *Disk pressure resolved.* Reclaimed **20.4 GB**: 17.85 GB of pre-2025 `~/.ros/log` debris (16,063
  files — the obsolete `camera_sw_node_obsolute.py` logged all 18 RC channels at INFO on every ~50 Hz
  RC callback, ~950 lines/s; the live `rc_control_node` is clean, and the obsolete file should be
  deleted from `src/rc_control/`), plus a 1.7 GB journal vacuum and 569 MB apt cache. **85% → 49%,
  29 GB free.** The SD card is fully partitioned (63.8 GB device, no unallocated space); the "64 GB vs
  58 G" gap is GB-vs-GiB plus ext4 overhead, not lost capacity.

  *Open items carried forward:* measured mount TF, then L5 (slam_toolbox on `/scan` + the live `/odom`,
  then Nav2 bringup); submodule-pin OrbbecSDK; delete `camera_sw_node_obsolute.py`. Still blocking
  *driving* a plan but not mapping: `RO_SPEED_LIM=0.01` unfixed and `mavlink.router` likely still wedged.
  **(Superseded by the 2026-07-21 evening entry below: the mount TF is now measured, `RO_SPEED_LIM` is
  fixed at 0.70, and the MAVLink link is healed. OrbbecSDK pinning and the obsolete file remain open.)**

- **2026-07-21 (evening) — geometry corrected, L2 blockers cleared, stack put under systemd.**
  A companion reboot wiped the manual `setsid` bring-up and, usefully, healed the wedged MAVLink link
  (FC heartbeat returned on `tcp:127.0.0.1:5760`). Params became readable again — and
  `PARAM_REQUEST_READ` via pymavlink does **not** re-wedge the link, unlike `mavlink_shell.py`. Prefer
  it. Beware that integer params arrive in `PARAM_VALUE`'s float field as a bit pattern: `EKF2_EV_CTRL`
  reads as `5.605e-45`, which *is* the integer 4, not corruption.

  *`RO_SPEED_LIM` fixed: 0.01 → 0.70* (user-applied, `param save`, readback-verified). This was the
  root cause of the forward-drive failure — every speed setpoint was being clamped to ±0.01 m/s, which
  is why 0.2 and 0.4 m/s produced identical wheel speeds. 0.70 deliberately sits *below*
  `autonav_mode`'s own 0.8 m/s clamp, so the FC is the binding limit, and above the ~0.58–0.60 m/s the
  drivetrain actually reaches.

  *Camera mount TF measured — L5 unblocked.* `base_link → camera_link` = **x −0.125, y 0.000, z 0.420**,
  zero rpy, verified live through `tf2_echo`. `cam_x` is negative because the lens sits 12.5 cm *behind*
  the rotation centre (wheelbase 0.43, lens 0.34 behind the front axle). Cross-check: lens 0.49 behind
  the front chassis edge, front axle 0.15 behind it → 0.34 + 0.09 = 0.43 = wheelbase. **Pitch and roll
  were measured, not assumed**, using the Gemini 336L's own IMU: enable with `enable_accel:=true
  enable_gyro:=true` (off by default), average `/camera/accel/sample`, and gravity read 9.785 of
  9.787 m/s² on a single axis with orthogonal components of −0.7° and +0.9° ⇒ level within ~1°.

  *Two geometry bugs, and they were masking each other.* The rover's **wheelbase is 0.43 m** and its
  **track is 0.31 m** — and 0.43 had been entered as the track width in *both* places that consume it:
  `rover_odometry` (`track_width`, used as `(v_right − v_left)/track`) and the FC (`RD_WHEEL_TRACK`,
  used as `Δv = ω × track`). Odometry therefore under-reported every yaw rate by ~28% while the FC
  commanded a ~39% oversized differential — errors in opposite directions, so the loop looked
  self-consistent while the underlying geometry was wrong. Both are now 0.31, saved and readback-
  verified. Straight-line odometry was never affected (that comes from `erpm_to_ms`). Note the FC's
  yaw-rate gains were tuned against the oversized value and may want revisiting after a floor run.
  A trap worth recording: `wheel_odometry_node` is launched without `--params-file`, so the YAML is
  never read and the **hardcoded `declare_parameter` default is what actually runs** — that silent
  override is what hid the bug. Both the YAML and the default are now corrected.

  *Wheels-up limit cycle — diagnosed, not a fault.* Armed on stands in Position mode the rover is quiet
  until a stick input, after which all four wheels swing full range ±1500 ERPM at ~1.2 s period and
  **never stop until disarm**, even with the stick centred. EKF2 meanwhile accumulated 4.25 m of
  phantom travel on a vehicle that never moved. Cause: `rover_ekf_bridge` feeds wheel-derived velocity
  into EKF2, so spinning wheels manufacture displacement; Position mode is a position *hold*, so it
  drives to "return", which spins the wheels, which manufactures more error the other way — an
  undamped limit cycle, since the only corrective action available is the one creating the error, and
  on stands there is no load or friction to damp it. `RO_SPEED_I` winds up and guarantees the
  overshoot. It re-confirms all four motors drive both directions at full range. It will **not** occur
  on the floor, where the rover actually arrives. Avoidance on stands: use Manual only, or stop
  `rover_ekf_bridge` (which drops `v_xy_valid` so Position/AutoNav cannot arm at all — the safe
  configuration). Never stop the bridge *while armed* in a mode that requires velocity: that trips a
  failsafe. Disarm first.

  *RC mapping resolved.* `RC_MAP_KILL_SW=8`, `RC_MAP_ARM_SW=5`, `RC_MAP_FLTMODE=6`, `NAV_RCL_ACT=6`
  (disarm on RC loss). Earlier notes recording "nothing mapped" were stale. Kill, arm and disarm were
  then **physically tested and confirmed working** — though in Manual; kill *inside AutoNav* remains
  untested. Caution: `SYS_STATUS`'s `rc` health bit read `True` while the transmitter was switched
  off, so it is not a reliable RC-presence check.

  *Stack moved to systemd* (`systemd/install_rover_units.sh`), replacing a manual bring-up that a
  reboot had wiped twice: `rover-camera`, `rover-scan`, `rover-odometry`, `rover-autonav-mode` all
  enabled and verified active. `Restart=always` also covers the recurring px4_ros2 4 s "no request from
  FMU" watchdog abort. **`rover-ekf-bridge` is installed but deliberately left `disabled`** — enabling
  it would recreate the limit cycle unattended on a rover that powers up on stands. Start it by hand on
  the floor; AutoNav cannot arm without it.

  *L2 remains open* and is now a genuinely valid test for the first time: rover on the floor, restart
  the bridge, drive 0.2 m/s and confirm 0.4 m/s produces roughly double the wheel speed — the check the
  `RO_SPEED_LIM` clamp previously made impossible.

## 6. Out of scope for v1
360° sensing, outdoor/GPS mode switching, YOLO/semantic perception (phase 5), multi-goal missions and
LLM mission brain (phase 6), rough-terrain traversability. Design keeps them pluggable (extra costmap
layers / behavior-tree nodes / localization sources).
