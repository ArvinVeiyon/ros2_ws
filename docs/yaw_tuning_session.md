# Yaw-Gain Tuning — Floor Session Checklist (#20)

Owner: roz | Prereq: rover on the floor | Related: `roadmap.md` (L5 interstitial), `rover_autonav_requirements.md`, `rover_autonav_collision_stop.md`

Goal: retune `RO_YAW_RATE_P` / `RO_YAW_RATE_I` so armed yaw stops driving the wheels far harder than
forward (was ~700–850 rpm yaw vs ~156 rpm forward on stands — gains were tuned against the old oversized
0.43 m track, now 0.31 m). Two opportunistic checks fold into the same session.

---

## Preconditions (physical)

- [ ] Rover **on the floor**, clear run-out ahead (a few metres, no walls close)
- [ ] RC transmitter **on**, operator on the sticks — **arm = ch5, mode = ch6, kill = ch8**
- [ ] Motor bus powered (otherwise `/odom` stays silent — that is the tell)

## Bring-up

```bash
# 1. stack should already be up from boot
systemctl is-active rover-camera rover-scan rover-odometry rover-autonav-mode   # expect: active x4

# 2. start the bridge BY HAND (deliberately not enabled — AutoNav can't arm without it)
printf '1987\n' | sudo -S systemctl start rover-ekf-bridge

# 3. confirm EKF has real velocity (and /odom is flowing = bus powered)
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/tools/l2_watch.py        # want cs_ev_vel / v_xy_valid = True, dead_reckoning = False
ros2 topic hz /odom                         # want ~100 Hz
```

## Baseline first, THEN tune

```bash
# 4. BASELINE run — unchanged gains, on the floor, against the fixed 0.31 m track.
#    Arm in MANUAL via RC (throttle neutral) → l2_test does the software DO_SET_MODE -> AutoNav.
python3 ~/ros2_ws/tools/l2_test.py
```

Two questions the baseline answers:
1. Does **0.4 m/s give ~double the wheel rpm of 0.2** (confirms the `RO_SPEED_LIM` 0.01->0.70 fix on the floor)?
2. What is the **yaw-vs-forward rpm asymmetry now** (the reason we are tuning)?

```bash
# 5. tune RO_YAW_RATE_P / RO_YAW_RATE_I (currently 2.0 / 0.1) — ONE change, re-run, repeat.
#    Params are MAVLink-only. Use pymavlink PARAM_SET on tcp:127.0.0.1:5760 —
#    NOT mavlink_shell.py (that wedged the link before). Read back to confirm each write
#    (values arrive as INT32 bit-pattern in PARAM_VALUE's float field — decode, don't read the float).
```

## Same-session opportunistic (cheap, high value)

- [ ] **Gyro-yaw validation** — turn a **known angle** (90° or 360°) against floor marks, compare `/odom`
      yaw; A/B with `yaw_source:=wheels` to quantify the skid-steer slip error.
- [ ] **`/scan` tape-measure check** — one distance vs a tape measure (only rate/plausibility confirmed so far).

## Safety reminders

- **Thumb on kill (ch8)** the whole time — proven to work armed in AutoNav.
- **Never wheels-up with the bridge running** → self-sustaining front/back limit cycle; only disarm stops it.
- Avoid `pkill -f` / `pgrep -f` — they self-match the invoking shell (killed the session shell 3× before).
- Prefer pymavlink `PARAM_REQUEST_READ` / `PARAM_SET`; never `mavlink_shell.py` (wedges the FC heartbeat).

## Teardown

- [ ] Disarm
- [ ] `printf '1987\n' | sudo -S systemctl stop rover-ekf-bridge`
- [ ] Leave FC in **Hold**, rover safe
- [ ] Record the chosen `RO_YAW_RATE_P/I` values + the baseline/after rpm numbers back in `roadmap.md` and memory

---

## Results log (fill in during the session)

| Run | RO_YAW_RATE_P | RO_YAW_RATE_I | fwd rpm @0.2 | fwd rpm @0.4 | yaw rpm | Notes |
|-----|---------------|---------------|-------------|-------------|---------|-------|
| baseline 07-28 | 2.0 | 0.1 | `[-169,171,170,171]` | not run | `[-868,-762,813,-1065]` | L2 PASS. Yaw ~5x forward effort, ~40% L/R asymmetry. Run ended in **wall contact** — see below. |
| | | | | | | |
| | | | | | | |

## 🔴 2026-07-29 — ACHIEVED YAW RATE MEASURED. STOP: the yaw loop is not controlling.

`tools/yaw_response_log.py` (passive; run alongside `l2_test.py --live`) captured the missing number.

| Commanded | Raw gyro sustained | Raw gyro peak | `/odom` sustained | Observed by eye |
|---|---|---|---|---|
| **0.3 rad/s** | **5.70 rad/s** | **8.02 rad/s** | 7.28 rad/s | **~2 full turns in 2 s ≈ 6.3 rad/s** |

**≈21× the command, and ≈4× past the FC's own `RO_YAW_RATE_LIM` of 1.57 rad/s.** Three independent
sources agree, including the operator watching it. This is **not a gain-trim problem** — a rate loop
that overshoots its own hard limit fourfold is not regulating at all.

⛔ **Do not run armed yaw tests until this is understood.** It is also the mechanism behind the
2026-07-28 wall contact: this is what "yaw translates" really means at 6 rad/s.

Measure yaw rate from **`sensor_combined.gyro_rad[2]`** (99.6 Hz, −0.004 rad/s at rest; negate for ROS
FLU sense). `vehicle_angular_velocity` is not in this FC's `dds_topics.yaml`.

### Second bug found the same way: `erpm_to_ms` is ≥2.3× too small
`src/rover_odometry/config/rover_odometry.yaml:12` = `0.000380`. Back-calculated from the confirmed
6.28 rad/s spin (0.31 m track ⇒ ~0.97 m/s per side at ~1090 ERPM) the true scale is **≈0.00089**, and
slip only pushes that higher. It explains forward reading **0.081 m/s against 0.2 m/s commanded**.
This contaminates `/odom` → EKF2 EV velocity → and Nav2/slam_toolbox at L5.
**Fix it by measurement, not algebra: drive a tape-measured 2 m straight and compare `/odom`'s travel.**

## ⚠️ 2026-07-28 — WALL CONTACT during the baseline run. Read before the next floor run.

The rover reached the wall with the reflex collision-stop active and working. Two facts combined:

1. **Yaw is never gated.** `autonav_mode/include/autonav_mode/mode.hpp:99` only ever zeroes `speed`;
   `yaw_rate` passes through by design ("so the vehicle can back off / turn away"). The baseline yaw
   leg ran 760–1065 ERPM for 2 s with ~40% L/R asymmetry — an asymmetric spin **translates**, and
   nothing in the reflex stop can clamp it.
2. **Thresholds are measured at the camera, not the bumper.** `base_link` is **0.345 m back from the
   front plate tip** and `cam_x = 0.00`, so the camera sits 0.345 m behind the bumper. `stop_distance`
   0.60 m therefore leaves the bumper only **0.255 m** from the wall, with no allowance for braking
   distance. Logged blocks at 0.49–0.59 m put the bumper at 0.145–0.245 m.
   (Note the 07-26/27 remount **improved** this — the old `cam_x −0.125` left only 0.130 m.)

Planning a run on "front = 1.85 m clear" by budgeting only the forward legs (~0.7 m) is not sufficient;
budget the yaw leg's translation too, or run yaw tests facing open space.
