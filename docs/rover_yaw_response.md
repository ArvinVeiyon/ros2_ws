# Rover yaw response — the measured curve, and what it means for #20

**Measured 2026-08-02** on the floor, armed, in MANUAL (the rate controller is not in the
loop in Manual, so this is the vehicle's raw response, not the controller's).

Method: hold the yaw stick steady, require `|steer|` constant within 0.05 for ≥0.8 s, then
average `sensor_combined.gyro_rad[2]` over the **settled half** of each hold. Averaging the
whole hold measures acceleration, not steady state.

`steer` here is `/fmu/out/rover_steering_setpoint.normalized_steering_setpoint`, i.e. the
normalized left/right speed difference, range -1..1.

---

## 1. The curve

| steer out | 0.055 | 0.230 | 0.268 | 0.281 | 0.348 | 0.425 | **0.484** | **0.573** | **0.935** |
|---|---|---|---|---|---|---|---|---|---|
| yaw rate (rad/s) | 0 | 0 | 0 | 0 | 0 | 0 | **0.67** | **1.51** | **4.11** |

**Two regions:**

```
steer < ~0.45   ->  NOTHING. The rover does not rotate at all.
steer > ~0.45   ->  yaw_rate ~= 7.6 * (steer - 0.40)   rad/s
```

Fit check: 0.573 predicts 1.35 (measured 1.51) · 0.935 predicts 4.10 (measured 4.11).

### 🔑 Minimum achievable yaw rate ≈ **0.67 rad/s**

**⛔ Never command a turn slower than this.** Below it the rover does not rotate — it sits
and grinds while the planner believes it is turning. Nav2 and `autonav_mode` must clamp.

### Why the deadband is so large
A skid-steer must **scrub all four tyres sideways** to rotate. At ~25 kg on a hard floor that
takes roughly 45% of full differential before anything moves. This is traction, not software:
the levers are weight, tyres and surface.

---

## 2. This explains #20 (the "yaw rate runaway, ~21x command")

Observed 2026-07-29: commanded **0.3 rad/s**, achieved **5.7-6.3 rad/s**, reproducible across
three armed runs and confirmed by the operator watching it.

**Mechanism, end to end:**

1. Command 0.3 rad/s. The *correct* feedforward lands **inside the 0-0.45 deadband**.
2. Nothing moves, so the error never decreases.
3. `RO_YAW_RATE_I` winds the integrator toward its limit — which is **1.0, the entire output
   range** (`_pid_yaw_rate.setIntegralLimit(1.f)` in `DifferentialRateControl.cpp`).
4. Friction finally breaks at near-maximum output.
5. `7.6 * (1.0 - 0.40)` ≈ **4.6 rad/s**, and with overshoot, the 5.7-6.3 rad/s observed.

**⇒ `RO_YAW_RATE_P` was never the culprit.** Lowering it 2.0 → 0.05 (40x) could not help,
because the integrator does the damage. The mechanism is **integral windup across a friction
deadband**.

---

## 3. Three misconfigurations found the same day, all pushing FF deeper into the deadband

| param | was | now | why it mattered |
|---|---|---|---|
| `RO_MAX_THR_SPEED` | 3.0 | **0.6** | Firmware doc: *"Speed the rover drives at maximum throttle", @unit m/s*. The drivetrain reaches ~0.58-0.60 m/s. It divides the feedforward in **both** the speed and yaw-rate loops, so FF was **5x too small**. |
| `RO_YAW_RATE_LIM` | 0.5 | **28.6** | 🔴 **It is `@unit deg/s`, not rad/s.** 0.5 meant 0.0087 rad/s — **6x below the 3 deg/s measurement deadband `RO_YAW_RATE_TH`** — so in Acro every yaw command was zeroed and the controller emitted exactly 0.0000. Measured: 1820 armed Acro samples, flat zero output. |
| `RO_YAW_RATE_CORR` | 1.0 | 3.0 | Firmware doc: *"Increase this value (x > 1) if the measured yaw rate is lower than the setpoint … particularly useful for skid-steered rovers … that cause a lot of friction when turning."* Written for this exact case, never touched. |

⚠️ `RO_YAW_RATE_LIM` is **not referenced by `DifferentialRateControl`** at all — only by
`DifferentialManualMode` and the *ackermann* modules. It therefore never constrained the
AutoNav path where the runaway happened. The older note *"exceeds RO_YAW_RATE_LIM 1.57 by
~4x"* assumed rad/s **and** assumed it applied; neither is true.

---

## 4. ⚠️ A linear feedforward cannot represent a deadband

PX4's feedforward is `FF = sp * wheel_track/2 * RO_YAW_RATE_CORR / RO_MAX_THR_SPEED` —
strictly proportional, through the origin. The real response has a **+0.40 offset**. So the
`CORR` required to hit the right output depends on the setpoint:

| target rate | steer needed | implied CORR |
|---|---|---|
| 0.67 rad/s | 0.488 | 2.8 |
| 1.0 rad/s | 0.532 | 2.1 |
| 2.0 rad/s | 0.663 | 1.3 |

**No single `CORR` serves both slow and fast commands.** Tune for the middle of the intended
operating range and let proportional feedback cover the rest.

---

## 5. Remaining tuning (bounded work, not diagnosis)

- `RO_YAW_RATE_CORR` → **~2.0-2.5** (currently 3.0, sized for the low end)
- `RO_YAW_RATE_P` → **~0.3-0.5** (currently 0.05, too weak for real authority)
- `RO_YAW_RATE_I` → **keep 0, or very small**. The deadband is precisely what makes windup
  dangerous; an integrator that can reach 1.0 will always eventually slam through it.
- Clamp commanded yaw rate to **≥ 0.67 rad/s** everywhere upstream.

**Test bench:** ACRO at low `P` with `I = 0`. It exercises the rate controller, needs **no
position estimate** (unlike AutoNav, which PX4 refuses to enter while armed when
`xy_valid` is false), and with `I = 0` the output is a direct function of stick — it cannot
creep to saturation.

---

## 6. Method notes that cost time

- **Always log `nav_state` alongside any control-loop probe.** In MANUAL, `manual()`
  publishes the steering setpoint straight from the stick and the rate controller never runs,
  so its output proves nothing about the loop. A probe without mode logging conflated the two
  and produced a false "positive feedback" reading.
- **Average the settled half of a hold**, not the whole hold, or transients dominate.
- Two hypotheses were killed by measurement and should not be re-proposed:
  **gyro sign** (operator drove right-then-left: +5.23 then −5.40 rad/s, correct for PX4 FRD)
  and **"the measurement never reaches the estimator"** (`ATTITUDE.yawspeed` is fed from
  `vehicle_angular_velocity`, reads ±0.0015 rad/s at rest and tracks real rotation).
- Manual full stick gives **5.2-5.4 rad/s**, so the "runaway" was ≈ full-stick output rather
  than an exotic instability.
