# Vind-Roz — Goal & Roadmap

Date: 2026-07-23 | Status: **living document** | Owner: roz
FC firmware: PX4 pxlabs-v1.17.0-2.0.0 @ a52c38b07d (built 2026-05-31)

Single source of truth for *where we are going and why*. Sits above the detailed docs — read those
for the how:
- `rover_autonav_requirements.md` — full architecture, params, acceptance criteria
- `rover_autonav_collision_stop.md` — reflex safety layer + arm workflow
- `ros2_architecture.md` — transport / message / DDS alignment

---

## 1. The Goal, clearly defined

**North Star.** Vind-Roz is one RPi5 companion driving two vehicles — an aerial drone and a ground
rover — sharing the same PX4-class flight controller, WFB-NG link, camera suite, and onboard AI. The
end state is a platform that **navigates, senses, avoids, and completes missions autonomously**, with a
human supervising rather than piloting.

**PRIMARY campaign — OUTDOOR autonomous navigation (rover).**
Give it a **GPS waypoint** in open outdoor space; it plans a route, drives there by itself, and avoids
obstacles **360°** the whole way — no operator on the sticks. Sensor/localization suite:
- **LDRobot STL-19 360° LiDAR** — primary all-around obstacle sensing + SLAM
- **Orbbec Gemini 336L depth** — forward 3D layer (catches what a flat 2D lidar plane misses)
- **DroneCAN GPS** — absolute global localization → real "go to this coordinate"
- **Wheel + gyro odometry** (already built) — dead-reckoning through GPS dropouts / GPS-denied fallback

**STEPPING-STONE + fallback campaign — INDOOR GPS-denied.**
Everything through L4 is built and proven here (mode registration, DDS control, odometry→EKF2, `/scan`,
collision-stop). Indoor GPS-denied stays as the **fallback mode** the rover falls back to when GPS is
lost. It proves the whole autonomy brain before terrain + GPS are added.

We climb a deliberate **L0→L7 autonomy-brain ladder** (indoor, proven with a real drive) and, in
parallel, an **O1→O5 outdoor hardware/config track** (the two additions + outdoor tuning).

---

## 2. Sensor & localization architecture (revised for outdoor-primary)

```
   STL-19 360° lidar ──/scan──────────────► slam_toolbox (map) + Nav2 global/local costmap ─┐
   Gemini 336L depth ──/scan_depth─────────► Nav2 costmap layer (forward 3D, low/overhang) ─┤
                                                                                            ▼
   DroneCAN GPS ──UAVCAN──► PX4 EKF2 ◄─ wheel+gyro odom (rover_ekf_bridge, EV velocity)   Nav2
        │                      │                                                          planner
        └── absolute global ───┴── local position (GPS-denied fallback) ──► /odom + TF ──► + controller
                                                                                            │
                                                                              /cmd_vel ─────┘
                                                                                 ▼
                                                          autonav_mode (PX4 "AutoNav") → VESC drive
```

Fusion policy: **GPS primary outdoors** (bounded, absolute); **wheel+gyro odometry** carries through GPS
dropouts and is the sole source indoors/GPS-denied. Lidar owns SLAM + the 360° costmap; the depth camera
is a complementary forward 3D layer, **not** the SLAM source.

**Note on the 336L (corrected 2026-07-23):** it is **active-stereo + global shutter** (same class as the
Intel D456), so it **works outdoors** — the earlier "blind in sunlight" note was about structured-light
cameras and was wrong. Range/precision drop in bright sun but it stays functional; it is the forward 3D
layer, with the STL-19 covering 360°.

---

## 3. Autonomy-brain ladder (indoor stepping-stone) — L0–L7

| Layer | What it is | Status |
|---|---|---|
| **L0** | Transport + msg alignment (px4_msgs 1.17 exact-match, DDS live) | ✅ Done |
| **L1** | Custom "AutoNav" mode registers on FC (External Mode 1, nav_state 23) | ✅ Done |
| **L2** | Mode I/O + real drive test | ✅ Done — armed floor run PASSED 2026-07-22 |
| **L3** | Sensor fusion: `rover_odometry` → `/odom` → EKF2 via `rover_ekf_bridge` | ✅ Done (`v_xy_valid`, arms clean) |
| **L4** | Gemini 336L SDK + depth `/scan` @20Hz, camera mount TF measured, Nav2 + slam_toolbox installed | ✅ Done |
| **L5** | Nav2 goal navigation + obstacle avoidance (global plan, local costmap, controller → cmd_vel) | 🔲 Next |
| **L6** | SLAM mapping + routing (slam_toolbox, replan / reroute) | 🔲 After L5 |
| **L7** | Safety hardening (geofence, auto-return, failsafe modes) | 🟡 Reflex collision-stop done |

**Interstitial before L5:** **#20 yaw-gain tuning** (armed yaw ~700–850 rpm vs forward ~156; `RO_YAW_RATE_P/I`
2.0/0.1 tuned against the old oversized 0.43 m track, now 0.31) · **gyro-yaw drive-validation** (known
angle vs floor marks) · **`/scan` tape-measure range check**.

**Solid foundations:** DDS mode control (`tools/dds_setmode.py`); arm-in-Manual → software `DO_SET_MODE` →
AutoNav; RC map (arm ch5 / mode ch6 / kill ch8, physically tested); odometry epoch-fix; `RO_SPEED_LIM`
0.01→0.70; collision-stop reflex (fired at 0.59 m from a real wall).

---

## 4. Outdoor hardware + config track (PRIMARY target) — O1–O5

| Phase | What it is | Effort / status |
|---|---|---|
| **O1** | **STL-19 360° lidar re-integration** | 🔲 Low — driver already in `ros2_ws` |
| **O2** | **DroneCAN GPS integration** | 🔲 Moderate — CAN bus already live (VESCs) |
| **O3** | 2D-lidar SLAM (slam_toolbox on lidar `/scan`) + depth as a costmap layer | 🔲 Builds on L6 |
| **O4** | Outdoor Nav2 with GPS waypoints (`navsat_transform` / GPS waypoint follower) | 🔲 Builds on L5 |
| **O5** | Outdoor safety: terrain handling, dynamic obstacles, **GPS-loss failsafe → wheel/gyro dead-reckoning** | 🔲 Extends L7 |

### O1 — STL-19 LiDAR (LDRobot STL-19 360°)
Specs (from `codex-work/ldlidar_stl19_install_guide.md`):
- **360° 2D scan, 0.02–25 m range, ~10 Hz**, `sensor_msgs/LaserScan`
- UART **230400 baud, RX-only** (lidar TX → RPi RX; no commands sent to it), internal fixed-speed motor
- Wired to **`/dev/ttyAMA3`** (UART3, `dtoverlay=uart3-pi5` in `/boot/firmware/config.txt`) — currently
  **disabled** in the UART map; hardware went to another team 2026-04-17, so this needs the unit back
- TF `base_link → base_laser`, **0.18 m** height offset (re-measure for the actual mount)
- Package `ldlidar_stl_ros2` (node `LD19`) is in `ros2_ws/src` with **both upstream fixes already applied**
  (pthread include; hardcoded `/dev/ttyAMA3` since the `port_name` CLI arg is ignored)
- Re-enable path: reconnect hardware → enable `ttyAMA3` → build/enable `ldlidar.service` (unit template
  in the install guide); local edits saved in `codex-work/ldlidar_stl_local_edits_20260417.patch`
- ⚠️ **`/scan` topic conflict:** the lidar driver *and* `depth_to_scan` both publish `/scan`. Resolution:
  **lidar owns `/scan`** (SLAM + 360° costmap); remap depth to **`/scan_depth`** as a separate costmap
  layer. Do this remap when O1 lands.

### O2 — DroneCAN GPS
- The **DroneCAN/UAVCAN bus is already active** (VESC ESCs at addr 10–13), so the GPS is one more node on it.
- PX4 side: enable `UAVCAN_ENABLE` for the GPS subclass + set `EKF2_GPS_CTRL` to fuse GPS (params are
  MAVLink/QGC-only — needs the FC param path, not DDS). Verify `/fmu/out/vehicle_gps_position` populates
  and `vehicle_global_position` goes valid.
- Nav2 side: outdoor GPS nav via `navsat_transform_node` (robot_localization) or Nav2's GPS waypoint
  follower — a different launch config from the indoor SLAM one.
- **Module make/model: TBD — user to confirm** (affects the exact DroneCAN GPS driver params).

**Honest caveat (design awareness, not a blocker):** STL-19 is a **2D** lidar — one horizontal plane at
fixed height. On uneven outdoor terrain it can miss low obstacles or read a slope as a wall. That is
exactly why it is paired with the forward 3D 336L, which covers that blind spot. Outdoor terrain, dynamic
obstacles and GPS multipath near structures remain genuinely harder than indoor — the *fundamentals* are
covered, the *tuning* is real work at O3–O5.

---

## 5. Critical path

```
#20 yaw tuning ─▶ L5 Nav2 (indoor) ─▶ L6 SLAM (indoor) ──┐
  + gyro-yaw val.                                        ├─▶ O1 STL-19 ─▶ O2 GPS ─▶ O3/O4 outdoor SLAM+GPS-Nav2 ─▶ O5 ─▶ v1.2.0
                                                         │
   (proves the full autonomy brain indoors, GPS-denied) ┘   (adds 360° sensing + absolute localization → OUTDOOR primary)
```

Prove the brain indoors first (fast, everything unblocked), then bolt on the two hardware additions and
tune for outdoor. Indoor GPS-denied survives as the fallback mode.

---

## 6. Supporting debt (off the critical path, but gates or risks the goal)

**Blocks QGC-side work / broader ops:**
- 🔴 **GCS uplink effectively dead** — relay→drone commands land 0/8, downlink ~15%. Why QGC shows
  "Unknown mode"; why we work over DDS. Blocks QGC arming/mode/param writes and accel cal — and note O2's
  GPS params need the FC param path. → `project_gcs_link_degraded.md`
- Relay clock/NTP — recurring; needs local chrony on the companion. → `project_relay_ntp_setup.md`
- RELAY-STN #2 USB brownout — EU card exceeds the Pi4 USB budget; needs a powered hub.

**Firmware `dds_topics.yaml` — add on the next planned flash (not worth a flash on its own):**
Verified 2026-07-23: every topic Nav2 / slam_toolbox / L5–L7 consumes is already exposed and flowing
(`/scan` 25 Hz, `/tf`, `/cmd_vel`, `vehicle_attitude` 100 Hz, `vehicle_local_position_v1` 50 Hz,
`esc_status` 50 Hz, `failsafe_flags`, `collision_constraints`, `home_position_v1`). No reflash needed to
reach the first autonomous drive. When the pxlabs firmware is next reflashed, batch in:
- **`/fmu/out/vehicle_angular_velocity`** — filtered body yaw rate; helps #20 yaw tuning + gyro-yaw odom.
- **`/fmu/out/sensors_status_imu`** (optional) — per-IMU health, to diagnose accel inconsistency over DDS.
- (For O2, confirm `/fmu/out/vehicle_gps_position` + `vehicle_global_position` stream once GPS is enabled.)

**Cleanup / hygiene:**
- Multicam **Phase D** — rc_control + optical_flow → stable camera aliases, then delete the stale udev rule.
- Delete `src/rc_control/camera_sw_node_obsolute.py` (the 18 GB log offender).
- 🔴 **Security** — rotate the plaintext GitHub PAT in the `codex-work` remote URL; switch to SSH.
- Promote OrbbecSDK to a real git submodule (currently gitignored + documented in `third_party.md`).

**Aerial platform (other half of the North Star, deferred until the rover proves out):** offboard mission
node, battery-RTH, 360° avoidance, GPS-denied SLAM for flight, YOLOv8n vision, LLM mission brain
(AUTONOMY_ROADMAP phases 2–6). Inherits directly from the rover work.

---

## 7. Recommended order

1. **Next floor session:** #20 yaw tuning + gyro-yaw known-angle validation + `/scan` tape check.
2. **L5 (indoor):** slam_toolbox + Nav2 stack → first autonomous goal drive, GPS-denied.
3. **L6 (indoor):** SLAM map + routing/reroute.
4. **O1 → O2:** re-integrate STL-19 (remap depth to `/scan_depth`), then DroneCAN GPS.
5. **O3 → O4 → O5:** outdoor 2D-lidar SLAM + GPS-waypoint Nav2 + outdoor safety → tag `v1.2.0`.
6. In parallel when convenient: rotate the PAT; chip at the GCS-link diagnosis (also needed for O2 params).

---

## Safety invariants (never weaken)

RC override is PX4-native; the collision-stop reflex stays active independent of Nav2; `cmd_vel` > 500 ms
or `/scan` > 1 s watchdog stops the rover; no reverse into unseen space (forward 3D + 360° lidar close the
gaps outdoors). The autonav stack auto-starts from boot **except `rover-ekf-bridge`**, started by hand only
with the rover on the floor (wheels-up + bridge + closed-loop mode = self-sustaining limit cycle).
Outdoors, **GPS loss must fail over to wheel/gyro dead-reckoning + reflex stop, never to an uncontrolled state** (O5).
