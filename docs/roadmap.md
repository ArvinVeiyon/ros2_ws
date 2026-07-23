# Vind-Roz — Goal & Roadmap

Date: 2026-07-23 | Status: **living document** | Owner: roz
FC firmware: PX4 pxlabs-v1.17.0-2.0.0 @ a52c38b07d (built 2026-05-31)

This is the single source of truth for *where we are going and why*. It sits above the detailed
docs — read those for the how:
- `rover_autonav_requirements.md` — full architecture, params, acceptance criteria
- `rover_autonav_collision_stop.md` — reflex safety layer + arm workflow
- `ros2_architecture.md` — transport / message / DDS alignment

---

## 1. The Goal, clearly defined

**North Star.** Vind-Roz is one RPi5 companion driving two vehicles — an **aerial drone** and a
**ground rover** — sharing the same PX4-class flight controller, WFB-NG link, camera suite, and
onboard AI. The end state is a platform that can **navigate, sense, avoid, and complete missions
autonomously in GPS-denied environments**, with a human supervising rather than piloting.

**Current campaign.** *Indoor, GPS-denied autonomous navigation for the ROVER*: Orbbec depth →
`/scan` → Nav2 → custom PX4 "AutoNav" mode → VESC drive, with wheel + gyro odometry feeding EKF2.
Forward-only sensing v1.

The rover is the proving ground. Mode registration, DDS control, odometry→EKF2, Nav2, SLAM and the
safety reflexes all transfer to the aerial platform afterward.

We climb a deliberate **L0→L7 ladder** — each layer proven (with QGC / a real drive) before the next.

---

## 2. Status ladder

| Layer | What it is | Status |
|---|---|---|
| **L0** | Transport + msg alignment (px4_msgs 1.17 exact-match to firmware, DDS live) | ✅ Done |
| **L1** | Custom "AutoNav" mode skeleton, registers on FC (External Mode 1, nav_state 23) | ✅ Done |
| **L2** | Mode I/O + real drive test | ✅ Done — armed floor run **PASSED** 2026-07-22 |
| **L3** | Sensor fusion: `rover_odometry` → `/odom` → EKF2 via `rover_ekf_bridge` | ✅ Done (`v_xy_valid`, arms clean) |
| **L4** | Gemini 336L SDK + `/scan` @20Hz, camera mount TF measured, Nav2 + slam_toolbox installed | ✅ Done |
| **L5** | **Nav2 goal navigation + obstacle avoidance** (global plan, local costmap, controller → cmd_vel) | 🔲 **Next big one** |
| **L6** | SLAM mapping + routing (slam_toolbox builds the map, replan / reroute) | 🔲 Not started |
| **L7** | Safety hardening (geofence, auto-return, failsafe modes) | 🟡 Partial — reflex collision-stop done |

**Interstitial items (small, do before L5):**
- **#20 Yaw-gain tuning** — armed yaw drove wheels ~700–850 rpm vs forward ~156; `RO_YAW_RATE_P/I`
  (2.0/0.1) were tuned against the old oversized track (0.43→0.31 m). *Next concrete action.*
- **Gyro-yaw validation** — heading now from `/fmu/out/vehicle_attitude`; verified at rest
  (0.044°/12 s drift) but **not yet validated driving** a known angle. Do in the same floor session.
- **`/scan` tape-measure range check** — only rate + plausibility confirmed so far.

**Solid foundations already in place:** DDS mode control (`tools/dds_setmode.py`); the
arm-in-Manual → software `DO_SET_MODE` → AutoNav workflow; RC map (arm ch5 / mode ch6 / kill ch8, all
physically tested); the odometry timestamp-epoch fix; `RO_SPEED_LIM` 0.01→0.70; and the collision-stop
reflex that fired end-to-end at 0.59 m from a real wall.

---

## 3. Critical path

```
#20 yaw tuning ──▶ L5 Nav2 bring-up ──▶ L6 SLAM + routing ──▶ L7 safety ──▶ tag v1.2.0
  (+ gyro-yaw          │                      │
   validation,         │                      └─ reroute around obstacles, not just stop
   same session)       └─ FIRST autonomous goal-to-goal drive
```

**L5 is the highest-value next milestone** — the first time the rover plans a path to a goal and
drives it while avoiding obstacles. The collision-stop reflex is only the safety *floor*; L5 is the
navigation brain. Everything for it is unblocked (camera TF measured, Nav2 1.3.12 + slam_toolbox 2.8.5
installed, `/scan` + `/odom` live).

**Design constraint for L5:** Nav2 footprint follows the **0.405 m top plate**, not the 0.31 m wheel
track — the wheels sit inboard.

---

## 4. Supporting debt (off the critical path, but gates or risks the goal)

**Blocks QGC-side work / broader ops:**
- 🔴 **GCS uplink effectively dead** — relay→drone commands land 0/8, downlink ~15%. This is why QGC
  shows "Unknown mode" and why we do everything over DDS. Blocks QGC arming/mode/param writes and accel
  cal. Routed around, but it caps operator-station control. → `project_gcs_link_degraded.md`
- Relay clock/NTP — recurring; needs local chrony on the companion. → `project_relay_ntp_setup.md`
- RELAY-STN #2 USB brownout — EU card exceeds the Pi4 USB budget; needs a powered hub.

**Cleanup / hygiene:**
- Multicam **Phase D** — rc_control + optical_flow → stable camera aliases, then delete the stale udev rule.
- Delete `src/rc_control/camera_sw_node_obsolute.py` (the 18 GB log offender).
- 🔴 **Security** — rotate the plaintext GitHub PAT in the `codex-work` remote URL; switch to SSH.
- Promote OrbbecSDK to a real git submodule (currently gitignored + documented in `third_party.md`).

**Aerial platform (the other half of the North Star, deferred until rover autonomy proves out):**
offboard mission node, battery-RTH, 360° avoidance, GPS-denied SLAM for flight, YOLOv8n vision, LLM
mission brain. These are AUTONOMY_ROADMAP phases 2–6 and inherit directly from the rover work.

---

## 5. Recommended order

1. **Next floor session:** #20 yaw-gain tuning + gyro-yaw known-angle validation + `/scan` tape check.
2. **L5:** slam_toolbox on `/scan` + `/odom`, then the Nav2 stack → first autonomous goal drive.
3. **L6 / L7:** routing + reroute + safety hardening → tag `v1.2.0`.
4. In parallel when convenient: rotate the PAT (quick security win), then chip at the GCS-link
   diagnosis since it caps operator control.

---

## Safety invariants (never weaken)

RC override is PX4-native; the collision-stop reflex stays active independent of Nav2; `cmd_vel` > 500 ms
or `/scan` > 1 s watchdog stops the rover; no reverse into unseen space (forward-only sensing).
The autonav stack auto-starts from boot **except `rover-ekf-bridge`**, which is started by hand only
with the rover on the floor (wheels-up + bridge + closed-loop mode = self-sustaining limit cycle).
