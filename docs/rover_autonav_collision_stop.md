# Rover AutoNav — Reflex Collision-Stop & Arming Workflow

> **Dimensions and camera-mount geometry live in [`rover_geometry.md`](rover_geometry.md)** —
> `front_overhang`, the corridor half-width and the plate width all come from there.

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
- **Distances are CLEARANCE AT THE FRONT BUMPER, not raw `/scan` range.** `/scan` originates at
  `camera_link`, which sits `front_overhang` behind the front plate tip, so the raw range has the
  overhang subtracted before any comparison. `front_overhang` was **measured 0.337 m** on 2026-07-28
  (rover parked square against a flat wall, zero gap: 178 consecutive scans, min == max == 0.337 m);
  it agrees with the 0.345 m `base_link`→plate-tip figure in the requirements doc to within 8 mm.
  **Re-measure the same way after any camera remount.**
- In `updateSetpoint()`, the block decision is evaluated **every tick** (not only when driving forward),
  so the hysteresis state tracks reality continuously. When blocked:
  - forward (`speed > 0`) is clamped to **0**;
  - **yaw is CAPPED to `blocked_yaw_rate`** (0.30 rad/s), not freed — enough authority to rotate away,
    not enough to lunge;
  - **reverse always passes through**, so the vehicle can back off.
- **Hysteresis:** blocks at `stop_distance` (0.35 m at the bumper), releases only past `clear_distance`
  (0.50 m at the bumper) — no chatter at the threshold.

> **Why yaw is capped (2026-07-28 wall contact).** Yaw used to pass through untouched. A skid-steer
> "spin" with unequal left/right wheel speeds **translates** — the baseline yaw leg ran 760–1065 ERPM
> with ~40% L/R asymmetry — so an ungated yaw command drove the rover into a wall the forward brake
> could see perfectly well and had no authority over. Compounding it, the thresholds were then measured
> at the *camera*: 0.60 m of raw range was only ~0.26 m of real bumper clearance.
- **Fail-safe, part 1 — STALE scan:** if `/scan` is stale/absent for longer than `scan_timeout`
  (0.5 s), forward is blocked when `require_scan=true`.
- **Fail-safe, part 2 — BLIND scan** *(added 2026-08-10)*: if a scan arrives on time but carries
  fewer than `min_valid_fraction` (0.35) valid rays across the **whole** scan, forward is blocked
  exactly as if it were stale.

> 🔴 **"No blind driving if perception dies" was WRONG until 2026-08-10, and the rover hit a wall
> proving it.** The stale check above covers a scan that stops *arriving*. It did not cover one that
> arrives **empty**: zero valid rays in the corridor gave `min_x = inf` ⇒ clearance `inf` ⇒
> "farther than `clear_distance`" ⇒ **unblocked**. With `scan_fresh=yes` throughout, the reflex
> ratcheted forward through alternating glimpses —
> `BLOCK 0.35 → clear inf → BLOCK 0.11 → clear 0.38 → BLOCK 0.26 → clear inf → BLOCK 0.08` —
> ~0.3 m closer each cycle, until contact. The threshold was never wrong; it blocked at exactly
> 0.35 m **whenever it could see**.
>
> The gate is evaluated **before** the clearance test, so a blind frame can never release a block
> earned by the last frame that could see. Health is counted over the whole scan, not the corridor,
> because a genuinely empty room also returns nothing ahead.
>
> **Recognition cue:** `collision-diag: BLOCK forward (BLIND) (scan_fresh=yes valid=NN% ...)`.
> Healthy is **87.5%** whole-scan (560/640) on this camera; blind is ~0%.
>
> ⏭ **Validation status: gate logic proven by forcing the threshold above the healthy fraction; a
> real occluded-lens test has NOT been run.** No armed run until it has. Full record:
> `autonav_reference.md` §8 and §10.
- **Directional:** obstacles outside the ±20° cone are ignored (e.g. an object at −37° during testing was
  correctly not treated as ahead). Head-on walls fill the cone and are caught.

### Parameters (`collision.*`)

| Param | Default | Meaning |
|---|---|---|
| `collision.enabled` | `true` | master enable |
| `collision.stop_distance` | `0.35 m` | block forward closer than this — **at the bumper** (raw scan 0.69 m) |
| `collision.clear_distance` | `0.50 m` | release only past this — **at the bumper** (raw scan 0.84 m) |
| `collision.front_overhang` | `0.337 m` | scan origin → front bumper; measured, re-measure after a remount |
| `collision.blocked_yaw_rate` | `0.30 rad/s` | max \|yaw\| while blocked (cap, not cancel) |
| `collision.min_valid_fraction` | `0.35` | **perception-health gate**: fraction of the WHOLE scan that must carry a valid range before the clearance test runs at all. Healthy 0.875, blind ~0. Below it, blocks like a stale scan |
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
