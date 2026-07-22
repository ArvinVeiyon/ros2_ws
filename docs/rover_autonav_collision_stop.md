# Rover AutoNav — Reflex Collision-Stop & Arming Workflow

Date: 2026-07-23 | Status: **implemented + validated on floor** | Owner: roz
Relates to: `rover_autonav_requirements.md` (R3 safety envelope, R5 watchdog), autonomy roadmap **phase 2 (collision stop)**

## 1. Why

The `autonav_mode` executor was a pure `/cmd_vel` → rover-setpoint passthrough: whatever velocity it
was handed, it drove, with **no awareness of obstacles**. During the first armed floor test (2026-07-22)
the rover drove toward a wall on a canned forward command and had to be stopped with the RC kill switch.
Full obstacle avoidance / path planning / rerouting is **Nav2 + slam_toolbox (L5)** and is not yet running.
This reflex is the **safety floor** that must exist regardless: it *stops* for an obstacle it can see; it
does not yet *route around* it.

## 2. Where it lives — and why there

The reflex is implemented **inside the executor** (`autonav_mode/include/autonav_mode/mode.hpp`), at the
single funnel every velocity command passes through on its way to the motors. Consequently it applies to
**anything** that publishes `/cmd_vel` — the L2 test script, Nav2 later, a joystick — none can bypass it.
An interposer node on the `/cmd_vel` topic could be routed around; a clamp in `updateSetpoint()` cannot.

```
/cmd_vel ─┐
          ├─► autonav_mode::updateSetpoint()
/scan ────┘        │  if commanded forward AND obstacle in front cone < stop_distance → speed = 0
                   ▼
             RoverSpeedRateSetpoint → FMU → VESCs
```

## 3. Behaviour

- Subscribes `/scan` (`sensor_msgs/LaserScan`, `SensorDataQoS`) alongside `/cmd_vel`.
- Each scan, computes the **nearest valid return in a ±20° forward cone** (`sector_half_angle`).
- In `updateSetpoint()`, if the command is **forward** (`speed > 0`) **and** that nearest return is
  inside `stop_distance` → forward is clamped to **0**. **Reverse and yaw always pass through**, so the
  vehicle can still back off or turn away.
- **Hysteresis:** blocks at `stop_distance` (0.60 m), releases only past `clear_distance` (0.75 m) — no
  chatter at the threshold.
- **Fail-safe:** if `/scan` is stale/absent for longer than `scan_timeout` (0.5 s), forward is blocked
  when `require_scan=true` — **no blind driving** if perception dies.
- **Directional:** obstacles outside the ±20° cone are ignored (e.g. an object at −37° during testing was
  correctly not treated as ahead). Head-on walls fill the cone and are caught.

### Parameters (`collision.*`)

| Param | Default | Meaning |
|---|---|---|
| `collision.enabled` | `true` | master enable |
| `collision.stop_distance` | `0.60 m` | block forward closer than this |
| `collision.clear_distance` | `0.75 m` | release only past this (hysteresis) |
| `collision.sector_half_angle` | `0.35 rad` (≈20°) | half-width of the forward cone |
| `collision.scan_timeout` | `0.5 s` | `/scan` older than this ⇒ perception stale |
| `collision.require_scan` | `true` | stale/absent scan ⇒ block forward (fail-safe) |

### Passive diagnostic

An always-on 5 Hz timer (`diagTick`) logs the block decision **edge-triggered** (only on
`clear ↔ BLOCK` transitions), *even while disarmed/inactive*. This is what makes on-stands validation
possible with no arming and no motion. Example: `collision-diag: BLOCK forward (scan_fresh=yes front=0.59m)`.

### Known limitation

The Orbbec reads returns closer than `range_min` (0.30 m) as invalid, and those are filtered out — an
obstacle pressed against the lens could read as "clear". In practice `stop_distance` 0.60 m is well
inside the valid range, so a wall is caught long before that. `/scan` geometry: 848 beams, FOV ±46°,
range 0.30–8.0 m.

## 4. Arming workflow (important)

**AutoNav is a custom px4_ros2 external mode (nav_state 23) and cannot be armed directly via RC.**
Flipping the RC arm switch (ch5) arms into whatever the RC *mode* switch (ch6) asserts — which is
**Manual (nav_state 0)** — because an external mode has no RC switch slot. Arming in AutoNav via RC
therefore always lands in Manual, and the test correctly refuses to drive.

**Working path — arm in Manual, then software-switch to AutoNav:**
1. Operator arms via RC (ch5) with **throttle neutral** → armed in **Manual**.
2. Companion sends `DO_SET_MODE main=4 sub=11` → **AutoNav**. It **takes and holds** (the RC mode switch
   does not yank it back; verified with a 2 s hold check).
3. `onActivate` holds zero until `/cmd_vel` arrives; the run proceeds.

`tools/l2_test.py` implements this: it **tolerates an already-armed-in-Manual start** (wheels must be
stopped), skips the RC-arm wait, sends AutoNav, and verifies it holds before moving. It **never
software-arms** — the operator is always the arming authority. (Note: the operator cannot see the
script's live stdout, so "arm on cue" is unreliable; **arm-first is the intended flow**.)

### Safety interlocks around arming
- On stands, **AutoNav must not be armed with the EKF bridge running**: wheels-up + bridge +
  closed-loop = self-sustaining front/back limit cycle (only disarm stops it). Validate the brake
  **passively** on stands (diagnostic + `/scan`), and only arm on the **floor** where odometry is real.
- The RC **kill (ch8)** is the final authority and is proven to work armed inside AutoNav.

## 5. Validation (2026-07-22 / 23)

- **On stands, passive (no arm/motion):** front cone tracks a board 2.4→0.4 m and back; flips to BLOCK
  at <0.60 m, sustained while held, releases past 0.75 m; hysteresis band observed; stale-scan fail-safe
  observed; directional (off-axis object ignored).
- **On floor, armed (L2 PASS):** arm-in-Manual → AutoNav switch held; all 4 wheels responded to forward
  and yaw; watchdog zeroed motors on `/cmd_vel` stop; auto-disarm + Hold on exit.
- **Armed collision-stop, end-to-end:** during the armed run the rover drove toward a wall and the reflex
  fired — `collision-stop: forward blocked (front=0.59m)` — stopping forward ~0.59 m short of the wall.
  This is the actuator-path clamp firing while armed, on a real wall. **Proven.**

## 6. Not yet done

- Nav2 + slam_toolbox (L5): the actual routing / avoidance / rerouting brain.
- Yaw-gain tuning: armed yaw produced much higher wheel RPM (~700–850) than forward (~156) — see
  requirements R-tuning / memory todos #20.
- Consider widening the forward cone or adding side sectors once Nav2 costmaps are in.
