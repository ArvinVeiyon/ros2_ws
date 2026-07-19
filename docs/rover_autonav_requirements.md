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

## 4. Milestones — in build order

| M | Deliverable | Done when |
|---|---|---|
| M0 | Install Nav2 + slam_toolbox + Orbbec wrapper; set PX4 rover/EKF2 params; bench-verify offboard TrajectorySetpoint on rover-diff (wheels up) | mode accepts setpoints, wheels respond correctly to vel+yaw cmds |
| M1 | `rover_odometry` package → `/odom` + TF | push rover 2 m by hand → odom reads 2 m ± 5 %; rotation check vs. measured |
| M2 | Navigation interface feed → EKF2 | PX4 local position valid + sane indoors (QGC map track matches motion) |
| M3 | Depth pipeline → `/scan` | rviz shows correct obstacle ranges vs. tape measure, ≥ 10 Hz |
| M4 | `nav2_px4_bridge` AutoNav mode + watchdogs | teleop via `cmd_vel` through bridge, wheels-up then ground; RC override + timeouts verified |
| M5 | Nav2 odom-frame: local costmap + point-goal with avoidance (no map yet) | rover reaches 5 m goal around a placed obstacle, 5/5 runs |
| M6 | slam_toolbox map + global planner | A→B across mapped room with auto-routing + re-route on new obstacle |
| M7 | Tuning + safety validation checklist pass | all R5 items demonstrated + logged |

Each milestone lands as its own commit(s) on `main`; tag `v1.2.0` when M7 passes.

## 5. Out of scope for v1
360° sensing, outdoor/GPS mode switching, YOLO/semantic perception (phase 5), multi-goal missions and
LLM mission brain (phase 6), rough-terrain traversability. Design keeps them pluggable (extra costmap
layers / behavior-tree nodes / localization sources).
