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
| baseline | 2.0 | 0.1 | | | | |
| | | | | | | |
| | | | | | | |
