# PX4 Parameter Audit — Rover

> **Audited 2026-08-14** against the flashed firmware source in `~/PX4-Autopilot`
> (rover_differential module + `src/lib/rover_control`), not from memory of what PX4 params exist.
> All values below were **read live off the vehicle** with `tools/set_param.py`.
> **Nothing was written.** Every value here is the state as found.
>
> 🔑 **This file is the detail. `MEMORY.md` carries only a pointer — pull from here when needed.**
> Companion docs: `autonav_reference.md` §5 (constants) · §10 (known faults) · `setup_manual.md`.

---

## 1. Verified correct — do not "fix" these

| parameter | value | verified against |
|---|---|---|
| `CA_AIRFRAME` | **6** = Rover (Differential) | enum in `control_allocator/module.yaml`; matches the vehicle |
| `RD_WHEEL_TRACK` | **0.310 m** | exactly the tape-measured track width in `autonav_reference` §5 |
| `CA_R_REV` | 3 | reversible motors bitmask — differential rover needs reverse |
| `UAVCAN_ENABLE` | 3 | sensors **and** actuators; required for the VESC ESCs |
| `BAT1_N_CELLS` | 6 | 6S ≈ 22–25 V, consistent with the 24 V Yalu hub motors |
| `COM_RC_IN_MODE` | 3 = RC or MAVLink, keep first | RC retains control |
| `RO_YAW_RATE_LIM` | 85.9 °/s | plausible cap for skid-steer |
| `FD_ACT_EN` | 1 | actuator failure detection on |

### ⚠️ `COM_DISARM_LAND` = 2.0 s — looks dangerous, is NOT. Do not change it.

The obvious fear is that a rover stopping for 2 s gets auto-disarmed mid-mission. It cannot,
because `RoverLandDetector::_get_landed_state()` ends with:

```cpp
return !_armed;   // If we are armed we are not landed.
```

**While armed, a rover is never "landed"**, so the timeout never starts. ⛔ **Do not "fix" this
parameter** — someone will otherwise disable a working failsafe on a false alarm.

🔴 **BUT it DOES return true in `AUTO_LAND`, `DESCEND` and `AUTO_RTL`.** Harmless for M0–M3.
**Re-check before M4**, where RTL is in play.

---

## 2. Findings

### P1 🔴 `RO_MAX_THR_SPEED` = 0.60 — its recorded basis is CONTRADICTED by later measurement

`throttle = setpoint / RO_MAX_THR_SPEED` (`RoverControl::speedControl`). If this is wrong, every
speed command is mis-scaled.

⚠️ **Correction to my first pass: it is not "unverified".** `setup_manual` §A7 records it being set
on 08-02 from a *"drivetrain measured at 0.58–0.60 m/s"*. **But on 08-12 the rover was measured at
~0.9 m/s** — 50% above the value that is supposed to be its full-throttle speed. So the parameter
has a stated basis and **that basis is contradicted**, which is a sharper finding than "unknown":
either the 08-02 drivetrain measurement was wrong, or something changed since. **Resolve which.**

⛔ **It cannot be measured from a closed-loop run.** That was attempted on 2026-08-13 and produced a
claim — *"understated 7.4×"* — that had to be retracted. In closed loop the throttle is a *function
of the error*, so `speed ÷ throttle` recovers the **controller**, not the plant. (Lag correlations
on that run: all `|r| ≤ 0.28`, a −0.27 simultaneous term against a +0.28 short-lag term, cancelling.)

**Needs an OPEN-LOOP throttle sweep** — RC Manual, stepped throttle, logging ERPM and taped distance
per step. That also separates a *gain* error from a *floor / minimum-duty offset*; the two need
different fixes and present evidence weakly favours a floor.

### P2 🔴 `RO_DECEL_LIM` = −1 (disabled) — **this blocks M2/M3**

Firmware doc, verbatim: *"Note that if it is disabled the rover will not slow down when approaching
waypoints in auto modes."* Waypoint overshoot is therefore guaranteed as configured today.

**Derivation of the suggested value** — from measured behaviour, not a round number. The 2026-08-12
wall contact recorded a coast of ~0.30 m from ~0.9 m/s:

```
a_coast = v² / 2d = 0.81 / 0.60 = 1.35 m/s²      (friction alone, no braking)
```

`RO_DECEL_LIM` must be **≤ what the vehicle can actually achieve**, or the profile is never tracked.

| `RO_DECEL_LIM` | % of capability | stop from 0.5 m/s | stop from 0.3 m/s |
|---|---|---|---|
| 0.3 | 22% | 0.42 m | 0.15 m |
| **0.5** | **37%** | **0.25 m** | **0.09 m** |
| 0.8 | 59% | 0.16 m | 0.06 m |
| 1.0 | 74% | 0.12 m | 0.04 m |

**Suggest `RO_DECEL_LIM` = 0.5 m/s²** — achievable with margin, and stops from 0.5 m/s in 0.25 m,
comfortably inside the 0.35 m reflex standoff.

**Suggest `RO_ACCEL_LIM` = 0.3 m/s²** — deliberately gentler than decel. A soft start cuts the wheel
slip that corrupts odometry, and stopping should never be slower than starting.

**Leave `RO_JERK_LIM` = −1** for now. It only bites once accel/decel are active; adding it blind
over-constrains the motion before anyone has seen the ramp behave.

⏭ **VERIFY AFTER SETTING:** does the rover actually track the ramp, and does it still stop inside
the standoff? **Gate on MEASURED speed, never the command** (`autonav_reference` §10).

### P3 ⚠️ `RO_SPEED_LIM` (0.70) > `RO_MAX_THR_SPEED` (0.60) — inconsistent

`calcSpeedSetpoint` constrains the setpoint to `±RO_SPEED_LIM`, then maps it through
`math::interpolate(setpoint, −RO_MAX_THR_SPEED, +RO_MAX_THR_SPEED, −1, 1)`, which **clamps at 1.0**.

So every setpoint between **0.60 and 0.70 saturates full throttle** — the top of the commandable
range is a dead zone that also defeats the speed loop.

**Suggest `RO_SPEED_LIM` = 0.60.** Revisit if P1 raises `RO_MAX_THR_SPEED`.

### P4 ⛔ `RO_SPEED_TH` = 0.10 — can be disabled (−1), but **DO NOT YET**

This is the mechanism behind the 0.14 m/s floor. `DifferentialSpeedControl.cpp:105`:

```cpp
_vehicle_speed = velocity_2d.norm() > _param_ro_speed_th.get() ? sign(...) * velocity_2d.norm() : 0.f;
```

Below the threshold the feedback is forced to **exactly zero**, so the loop believes it has stopped
and re-accelerates. Setting `−1` disables it (any norm ≥ 0 exceeds −1) and would remove the floor.

🔴 **But the threshold exists to reject noisy low-speed measurement, and ours is exactly that noise
— 46% zero-dropout.** Disabling it now feeds the dropout straight into the control loop and will
very likely make things worse.

🔑 **GATED ON THE ESC-DROPOUT FIX. The order matters.**

### P5 ⚠️ `RC_MAP_KILL_SW` = 12, but the docs say the kill switch is "ch8"

S1 passed, so the **function is verified** — this is a documentation discrepancy, not a fault.
But a safety procedure naming the wrong channel costs seconds in an emergency.

⛔ **Fix the docs. Do not change the parameter.** (`COM_KILL_DISARM` = 5.0 s is fine.)

### P6 ⏭ Read the VESC config on all four ESCs — VESC Tool, no vehicle movement

`si_motor_poles` (expect the pole-**pair** count, which is what gives R = 2) · `si_gear_ratio` ·
`si_wheel_diameter`. Confirms R = 2 directly and tests whether the four are configured consistently.

🔴 **`si_motor_poles` ↔ `erpm_to_ms` are a LINKED PAIR** — `autonav_reference` §10.

### P7 ✅ ~~Tuning params with unknown provenance~~ — **RETRACTED 2026-08-14. They ARE documented.**

I flagged `RO_YAW_RATE_I`, `RO_YAW_RATE_CORR`, `RO_YAW_RATE_P` as untraced **without reading
`setup_manual.md` §A7 first**, which records every one of them with its evidence:

* `RO_YAW_RATE_I` = **0** — 🔴 **deliberate: it was the windup source**, one of the two causes of
  #20. §A7 says **"never restore 0.1"**. ⛔ **Do NOT "add integral" — that is the documented fault.**
* `RO_YAW_RATE_CORR` = **1.8** — set 08-02 for the ~1.2 rad/s design point, mid usable band.
* `RO_YAW_RATE_P` = **0.08** — highest gain that does not hunt across the friction deadband.

🔑 **Lesson (`check_docs_before_measuring`): the manual beats my inference. Read §A7 before calling
any parameter untraced.**

Genuinely still open: `RO_YAW_P` = 2.0 (not in §A7) and `RO_SPEED_RED` = −1 (no speed reduction on
course error — relevant once autonomy turns).

### P8 🌍 Outdoor / M4 only — all defensible indoors, all must change before M4

`COM_LOW_BAT_ACT` = 0 (warning only) · `GF_ACTION` = 0 (geofence off) · `NAV_DLL_ACT` = 0 (no
datalink-loss action) · re-check `COM_DISARM_LAND` (fires in `AUTO_RTL`).

### P9 ⬜ NOT audited — stated so nobody assumes otherwise

* `EKF2_*` — ⚠️ **SHARED WITH THE DRONE.** Never change in isolation (`setup_manual` §A7).
* Sensor and RC calibration.
* Per-ESC output index assignment — which ESC index maps to which wheel in the mixer.

---

## 3. Full read — values as found, 2026-08-14

| parameter | value | | parameter | value |
|---|---|---|---|---|
| `RD_WHEEL_TRACK` | 0.310 | | `RO_YAW_P` | 2.0 |
| `RD_TRANS_TRN_DRV` | 0.0873 | | `RO_YAW_RATE_P` | 0.08 |
| `RD_TRANS_DRV_TRN` | 0.1745 | | `RO_YAW_RATE_I` | 0.0 |
| `RO_MAX_THR_SPEED` | 0.60 | | `RO_YAW_RATE_LIM` | 85.9 |
| `RO_SPEED_LIM` | 0.70 | | `RO_YAW_RATE_TH` | 3.0 |
| `RO_SPEED_P` | 0.5 | | `RO_YAW_RATE_CORR` | 1.8 |
| `RO_SPEED_I` | 0.1 | | `RO_YAW_ACCEL_LIM` | −1 |
| `RO_SPEED_TH` | 0.10 | | `RO_YAW_DECEL_LIM` | −1 |
| `RO_SPEED_RED` | −1 | | `RO_YAW_STICK_DZ` | 0.10 |
| `RO_ACCEL_LIM` | −1 | | `RO_YAW_EXPO` | 0.0 |
| `RO_DECEL_LIM` | −1 | | `RO_YAW_SUPEXPO` | 0.0 |
| `RO_JERK_LIM` | −1 | | `CA_AIRFRAME` | 6 |
| `COM_RC_IN_MODE` | 3 | | `CA_R_REV` | 3 |
| `COM_DISARM_LAND` | 2.0 | | `UAVCAN_ENABLE` | 3 |
| `COM_KILL_DISARM` | 5.0 | | `BAT1_N_CELLS` | 6 |
| `RC_MAP_KILL_SW` | 12 | | `BAT_LOW_THR` | 0.15 |
| `FD_ACT_EN` | 1 | | `BAT_CRIT_THR` | 0.07 |
| `GF_ACTION` | 0 | | `COM_LOW_BAT_ACT` | 0 |
| `NAV_DLL_ACT` | 0 | | `COM_OBL_RC_ACT` | 0 |

> `UAVCAN_SUB_ESC` / `UAVCAN_PUB_ESC` returned `<no reply>` — **wrong name or a busy link, NOT
> proof they are unset.** Retry before drawing any conclusion.

---

## 4. CHANGE LOG

### 2026-08-14 (later) — 🔴🔴 ALL THREE REVERTED AFTER A HARD WALL HIT IN MANUAL DRIVE

| parameter | applied | **reverted to** | status |
|---|---|---|---|
| `RO_DECEL_LIM` | 0.5 m/s² | **−1 (off)** | 🔴 **ROOT CAUSE of the hit** |
| `RO_ACCEL_LIM` | 0.3 m/s² | **−1 (off)** | 🔴 same fault (2.0 s stick ramp-up) |
| `RO_SPEED_LIM` | 0.60 | **0.70** | ⚠️ **not implicated** — reverted at operator instruction |

Reverted with `tools/set_param.py`, all three read back verified, autopilot `1:1`, `armed=False`.
🔴 **RAM ONLY — needs `param save`, or a reboot restores the values that caused the crash.**

> 🔴🔴 **WHY IT HAPPENED — `RO_ACCEL_LIM` / `RO_DECEL_LIM` ACT IN MANUAL MODE.**
>
> I applied them believing the firmware doc's *"will not slow down when approaching waypoints in
> auto modes"* described their whole scope, and filed verification as an AutoNav concern. The
> operator's next **RC manual** drive ended in a hard wall hit.
>
> **Verified in source** (`~/PX4-Autopilot`, `a52c38b07d`) — Manual does *not* bypass them:
>
> | step | file:line | fact |
> |---|---|---|
> | 1 | `commander/ModeUtil/control_mode.cpp:55` | `NAVIGATION_STATE_MANUAL` ⇒ `flag_control_allocation_enabled = true` |
> | 2 | `RoverDifferential.cpp:154` | that flag runs `DifferentialActControl::updateActControl()` |
> | 3 | `DifferentialActControl.cpp:75` | it slew-limits the **raw stick throttle** via `RoverControl::throttleControl(…, RO_ACCEL_LIM, RO_DECEL_LIM, RO_MAX_THR_SPEED, dt)` |
> | 4 | `RoverControl.cpp:59` | decel slew rate = **`RO_DECEL_LIM / RO_MAX_THR_SPEED`** |
> | 5 | `RoverControl.cpp:70` | with both at −1 both branches fail ⇒ **`setForcedValue`, stick straight to motors** |
>
> **Numbers:** decel 0.5/0.60 = **0.833 /s** ⇒ full throttle → 0 takes **1.2 s of powered driving
> after the stick is centred**; accel 0.3/0.60 = **0.5 /s** ⇒ 0 → full takes **2.0 s**.
> At the ~0.9 m/s this rover actually reaches, that is **~0.5 m of powered travel after "stop"**,
> plus the known ~0.30 m coast ≈ **3× the previous manual stopping distance** — and the stick feels
> no different. **The reflex does not cover this**: it gates AutoNav setpoints, not the operator.
>
> 🔑 **METHOD ERROR (the transferable lesson):** I scoped the parameters by their **description**
> instead of by **which controllers each `nav_state` enables**. ⛔ **Before changing any `RO_*`:
> grep `control_mode.cpp` for every mode enabling the controller that reads it — and check
> `DifferentialActControl` explicitly, because allocation is enabled in EVERY manual mode.**
> 🔑 **"Not yet verified on the vehicle" ≠ "not yet active."** I wrote the former and meant the
> latter. The change was live on the operator's very next stick input.
>
> ⚠️ **`RO_SPEED_LIM` back at 0.70 re-opens P3** (setpoints 0.60–0.70 saturate full throttle). That
> is a real but much milder defect, and it is **POSCTL-only**. Re-fix it on its own, never bundled.

⛔ **P2 BELOW IS SUPERSEDED — do not act on it as written.** Its analysis of *magnitudes* stands;
its assumption that the params are auto-only does not. Any future re-application must characterise
**manual** stopping distance on the floor, by tape, before the rover is driven near anything.

#### Superseded original entry, 2026-08-14 earlier — applied, then reverted the same day

| parameter | was | now | why (as written at the time) |
|---|---|---|---|
| `RO_DECEL_LIM` | −1 (off) | 0.5 m/s² | Unblocks M2/M3 waypoint slowdown (P2) |
| `RO_ACCEL_LIM` | −1 (off) | 0.3 m/s² | Soft start, less slip (P2) |
| `RO_SPEED_LIM` | 0.70 | 0.60 | Removes the saturation dead zone (P3) |

All three are **`RO_*` = rover-only** (rover control library, used by the rover modules).
✅ **No effect on the drone that shares this FC.** Contrast `EKF2_*`, which is shared.

> ✅ **PERSISTED 2026-08-14** — saved by the operator (`param save`) and verified across a real FC
> reboot at **51.9 s since boot**. 🔑 **The reboot itself was confirmed via `time_boot_ms` — "it
> survived a reboot" is not evidence unless you prove the reboot happened.** ⚠️ **Note what this
> verification did and did not establish: it proved the values were STORED. It said nothing about
> what they DO — and the flash-save is exactly why they were still live at the next drive.**

⛔ **Deliberately NOT changed:** `RO_MAX_THR_SPEED` (unverified — P1), `RO_SPEED_TH` (gated on the
dropout fix — P4), `RC_MAP_KILL_SW` (works; fix the docs instead — P5).

### 2026-08-14 — tool fix found while applying the above

`tools/set_param.py` took its arm check from **whichever HEARTBEAT arrived first**. A GCS at
**255:190** was advertising `base_mode` **0xc0**, which contains `MAV_MODE_FLAG_SAFETY_ARMED` (0x80),
so the tool refused every write against a **disarmed** rover.

🔴 **The mirror case is the dangerous one:** a GCS heartbeat *without* that bit would let a write
through to a **genuinely armed** vehicle, silently defeating the guard.

**Fixed:** the arm check now filters to the real autopilot (`component 1`, `autopilot != INVALID`)
and **refuses if no autopilot heartbeat arrives within 5 s** rather than assuming disarmed. It also
now prints the arm check it actually performed.

🔑 **Ground truth on this link:** autopilot is **`1:1`** (`autopilot=12` PX4, `type=10` GROUND_ROVER).
Anything at `255:190` is a ground station — **never read vehicle state from it.**
