# RC Input & UAVCAN ESC Output — measured configuration

Date: 2026-09-04 | Vehicle: Vind-Roz rover (4WD skid-steer) | FC: PX4 pxlabs-v1.17.0-2.1.0
Status: **all values below were READ FROM THE FC or MEASURED LIVE**, not copied from a snapshot.

> 🔑 **Read the FC, never a snapshot.** `python3 ~/ros2_ws/tools/set_param.py NAME` reads any parameter
> over MAVLink (`PARAM_REQUEST_READ`, `tcp:5760`). It does **not** wedge the link, unlike
> `mavlink_shell.py`. Writes are **RAM ONLY** until `param save`.

---

## 1. RC channel map

| Param | Value | Meaning |
|---|---|---|
| `RC_MAP_THROTTLE` | **2** | ch2 = forward/reverse throttle |
| `RC_MAP_FLTMODE` | 6 | flight-mode selector |
| `RC_MAP_ARM_SW` | 5 | arm switch |
| `RC_MAP_KILL_SW` | 12 | kill switch — ⚠️ **docs elsewhere say "ch8"; the PARAM is right, fix the docs** |
| `NAV_RCL_ACT` | 6 | disarm on RC loss |

Observed stick assignment: **ch2 = throttle, ch4 = steering, ch3 unused** (static 1001), ch1 static 1500.
**ch10 is the companion power channel** — 2014 = reboot, 1514 (middle) = shutdown, 1011 (down) = safe.
🔑 **Check ch10 before debugging any companion restart.** → `project_rc_ch10_reboots_companion`

## 2. Channel 2 (throttle) calibration — as read from the FC

| Param | Value |
|---|---|
| `RC2_MIN` | 1001.0 |
| `RC2_MAX` | **1986.0** |
| `RC2_TRIM` | 1001.0 |
| `RC2_REV` | 1.0 (not reversed) |
| `RC2_DZ` | ⚠️ **UNREAD** — no reply in 4 attempts. **Not "unset": unknown.** |

### 2.1 🔑 `RC2_TRIM == RC2_MIN` is a QGC artefact, and PX4 corrects it
Do not "fix" this parameter. `rc_update.cpp` **re-centres trim to `(MIN+MAX)/2` = 1493** whenever
`TRIM == MIN`, which is the standard QGC bipolar-calibration artefact. Verified arithmetically against
live data at every intermediate stick position:

| stick | ch2 (µs) | predicted `(ch2−1493)/493` | measured `manual_control_setpoint.throttle` |
|---|---|---|---|
| neutral | 1500 | +0.014 | **+0.0142** |
| ~50% | 1748 | +0.517 | **+0.5193** |
| ~90% | 1951 | +0.929 | **+0.9290** |
| full fwd | 2000 | +1.028 | **+1.0000** (clamped) |
| full rev | 1001 | −0.998 | **−1.0000** (clamped) |

### 2.2 ⚠️ Full stick OVERSHOOTS the calibrated range
**ch2 reads 2000 at full forward against `RC2_MAX` = 1986 — a +14 µs overshoot.** PX4 clamps the
normalised value to exactly ±1.0000, so it is harmless, but it means **full stick is a SATURATED
command**: the FMU emits its true maximum. Re-running RC calibration in QGC would widen `RC2_MAX` to
~2000 and remove the saturation, but nothing currently depends on it.

---

## 3. UAVCAN ESC output — and the full-reverse fault this configuration caused

| Param | ch1 | ch2 | ch3 | ch4 |
|---|---|---|---|---|
| `UAVCAN_EC_FUNC` | 101 | 102 | 101 | 102 |
| `UAVCAN_EC_MIN` | **110** | **110** | **110** | **110** |
| `UAVCAN_EC_MAX` | **8082** | **8082** | **8082** | **8082** |

`UAVCAN_EC_FAIL1` = −1 · `UAVCAN_ENABLE` = 3 · `UAVCAN_BITRATE` = 1000000 · `CA_R_REV` = 3 ·
`CA_AIRFRAME` = 6.
⛔ **`UAVCAN_EC_DIS1` DOES NOT EXIST** — PX4's UAVCAN_EC output block defines only min/max/failsafe.
Do not hunt for it.

### 3.1 🔴 Why MIN is 110 and MAX is 8082 — DO NOT "TIDY" THESE BACK TO 10 / 8191
**Original values 10 / 8191 made full reverse stop all four wheels.** PX4 has no disarmed parameter, so
`_disarmed_value` stays **0** and is sent whenever disarmed; the VESC fork therefore guards
**`if (raw < 100) stop`**. `MIN = 10` at full reverse also falls under that guard, and **the guard cannot
tell disarmed(0) from full-reverse(10)**. Forward (8191) is at the far end of the scale, which is why the
fault was **reverse-only**.

**The fix is deliberately symmetric:** `110 + 8082 = 8192`, so neutral lands on **exactly 4096** — the
value the VESC assumes. Raising MIN alone would put neutral at 4150 (+1.3%), which is inside the VESC's
±2% deadband but consumes 65% of it; the symmetric form removes the deadband dependence entirely.
Cost: ~1.3% of range at each end, invisible because PX4 already clamps at full stick.
⚠️ **Disarmed still sends 0, so the disarmed motor stop is unaffected.**

### 3.2 ⏭ The proper long-term fix is on the VESC side (NOT done)
The VESC should stop inferring arming from the command value and instead subscribe to
**`safety.ArmingStatus`, which PX4 already broadcasts**, after which the `<100` band can be deleted
entirely and the full range recovered at both ends.

🔴 **ORDERING HAZARD:** that `<100` band is **currently what stops the motors when disarmed**. Deleting
it before the arming subscription works would **remove the disarmed motor stop**. Do not reorder these.

Full evidence and the diagnosis method: §4 and §5 below, and `setup_manual.md` §A7.

---

## 4. Verified behaviour (armed, on stands, 2026-09-04)

| stick | ch2 | throttle | rpm (addr 10/11/12/13) | current |
|---|---|---|---|---|
| neutral | 1500 | +0.0142 | 0 / 0 / 0 / 0 | 0.00 A |
| full forward | 2000 | +1.0000 | +1511 / +1515 / +1580 / +1534 | 3.4–4.7 A |
| full reverse **(after fix)** | 1001 | −1.0000 | −1514 / −1513 / −1568 / −1534 | 3.1–4.5 A |
| full reverse *(before fix)* | 1001 | −1.0000 | **0 / 0 / 0 / 0** | **0.00 A** |

`esc_errorcount` was **0 = NONE** on all four ESCs in every state, and pack voltage held 25.0–25.4 V
throughout. ⚠️ **Verified on stands, unloaded — not yet driven on the ground.**

---

## 5. How to re-measure any of this — no MAVLink shell required

✅ **On DDS:** `/fmu/out/input_rc` · `/fmu/out/manual_control_setpoint` · `/fmu/out/esc_status`
❌ **Not on DDS:** `rc_channels` · `actuator_outputs` · `actuator_motors` · `vehicle_status`

The entire diagnosis above was done over DDS. ⛔ **Avoid `mavlink_shell.py`** — one session per FC boot,
and it killed the GCS MAVLink link on 08-16.

🔑 **`esc_status.esc_errorcount` carries the live VESC fault code** (the VESC fills it from
`mc_interface_get_fault()`), so **fault diagnosis needs no VESC Tool**:
`0`=NONE `1`=OVER_VOLTAGE `2`=UNDER_VOLTAGE `3`=DRV `4`=ABS_OVER_CURRENT `5`=OVER_TEMP_FET
`6`=OVER_TEMP_MOTOR `7`=GATE_DRV_OV `8`=GATE_DRV_UV `9`=MCU_UV `10`=WATCHDOG_RESET.

⚠️ **Any rclpy sampler needs a ~2.5 s DDS discovery warm-up before it starts counting.** A 3 s window
returned **zero samples on healthy topics** and looked exactly like a dead link. **Always prove a
non-zero baseline** (`esc_status` runs ~100 Hz) before believing silence.

⚠️ **The ESC address ↔ wheel map (10 = RF inverted, 11 = FL, 13 = RL, 12 = RR) has never been verified
against a physically turning wheel.** Do not read per-wheel meaning into an address until it is.
