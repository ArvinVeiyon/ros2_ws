# Vind-Roz AutoNav — Technical Reference

> **Rev. 2026-08-09.** The stable engineering reference for autonomous navigation on the Vind-Roz
> rover: what it is meant to do, what it physically is, which numbers are measured and trustworthy,
> and which constraints cannot be designed around. Written to be cited by section, not read once.
>
> Companion to `docs/autonomy_plan.md` (modes and tests), `docs/rover_geometry.md` (the authority on
> every dimension) and `docs/rover_yaw_response.md` (the authority on yaw).
>
> **Every number marked measured has a method recorded alongside it. Treat anything without one as
> an assumption awaiting a test.**

| § | Section |
|---|---|
| [1](#1-goal-and-scope) | Goal and scope |
| [2](#2-the-four-questions) | The four questions |
| [3](#3-platform-inventory) | Platform inventory |
| [4](#4-geometry-and-frames) | Geometry and frames |
| [5](#5-calibrated-constants) | Calibrated constants |
| [6](#6-odometry-and-heading) | Odometry and heading |
| [7](#7-motion-envelope) | Motion envelope |
| [8](#8-perception) | Perception |
| [9](#9-mapping-and-localization) | Mapping and localization |
| [10](#10-known-faults) | Known faults |
| [11](#11-inviolable-rules) | Inviolable rules |
| [12](#12-test-ladder-and-status) | Test ladder and status |
| [13](#13-fitness-assessment--dated) | **Fitness assessment — dated** |
| [14](#14-method-notes) | Method notes |

---

## 1. Goal and scope

"Autonomous" is not one capability. State **how autonomous, for which application**, before
building — the answer changes the hardware and the safety case.

### Application A — indoor surveillance · FIRST TARGET

The operator picks a room or a patrol route on a saved house map and presses go. The rover
localizes itself, routes there, avoids people and moved furniture, streams video, returns to base,
repeats.

Everything above "drive 3 m forward" in this application is a **localization** problem, not an
obstacle-avoidance one.

### Application B — outdoor GPS mission · LATER

Drive to a GPS waypoint in open ground with 360° obstacle avoidance and no operator. Blocked on
hardware (DroneCAN GPS, 360° lidar) and on Application A being proven first.

> ⚠️ **Scope boundary.** This rover shares a flight controller and a companion computer with an
> aerial drone on a different airframe. **Any FC parameter change made for the rover must be
> reverted before flight**, and is called out where it applies.

---

## 2. The four questions

Every autonomous vehicle answers four questions. Ours are in very different states, and confusing
them wastes weeks.

| # | Question | Provided by | State |
|---|---|---|---|
| **Q1** | Where am I? | Localization | ⚠️ works at mapped viewpoints; **not yet fairly measured** |
| **Q2** | What is around me? | Perception | ✅ forward sector only, permanently |
| **Q3** | How do I get there? | Planning | ⬜ configured, unproven |
| **Q4** | What if it goes wrong? | Failsafe | ⚠️ kill switch **passed 2026-07-22** (§12); reflex proven; no failsafe policy for lost localization or no-route |

> 🔑 Q1 has historically been treated as gating everything. It gates **mapped patrol**. It does
> **not** gate point-and-go, which needs no map and is available whenever the safety tests pass.

---

## 3. Platform inventory

| Element | Part | Notes |
|---|---|---|
| Companion | Raspberry Pi 5, BCM2712, 4-core, 8 GB | Ubuntu 24.04.1 aarch64, ROS 2 Jazzy |
| Flight controller | Custom Pixhawk 6X-RT (in-house PCB) | NXP i.MX RT1176 M7+M4, PX4 `pxlabs-v1.17.0-2.0.0` |
| IMUs | ICM-42688-P · ICM-45686 · BMI088 | three independent, ~400 Hz each |
| Magnetometer | BMM350, 50 Hz | 🔴 **suspect** — single, internal, inside a metal body |
| Depth camera | Orbbec Gemini 336L | USB3, 640×360 registered depth + colour, min range 0.308 m |
| Drive | 4× VESC over DroneCAN, skid-steer | addresses 10–13; ESCs doze at rest |
| Rangefinder | TFmini, `ttyAMA2` | 0.3–12 m, **drone-bound** |
| 360° lidar | LDRobot STL-19 | ⬜ **not fitted** — allocated to the drone |

### CPU budget — the binding constraint

Four cores, and perception fills them. Measured steady state with the full stack plus localization:

```
rtabmap (localization)   ~79%      camera wrapper          ~57%
wheel_odometry           ~53%      MicroXRCE agent         ~20%
claude (tooling)       55-86%      rc_control              ~11%
```

⛔ **There is no GPU escape.** The Pi 5 has no H.264 encoder (`rpivid` is decode-only),
`h264_v4l2m2m` reports no valid device, `h264_vaapi` has no driver, `card0` is `v3d` (3D only).
Everything heavy is already multi-threaded, so more threads cannot help. The only real offload would
be a camera with onboard H.264.

---

## 4. Geometry and frames

ROS REP-103 throughout: **+x forward, +y left, +z up**. `base_link` is the skid-steer rotation
centre, on the ground plane.

| Dimension | Value | Note |
|---|---|---|
| Top plate length | **0.730 m** | the widest and longest extent — **this is the footprint** |
| Top plate width | **0.450 m** | wheels sit *inboard* of the plate |
| Ground → top plate | **0.235 m** | |
| Track (hub to hub) | **0.310 m** | ⚠️ NOT the widest extent |
| Wheelbase (hub to hub) | 0.430 m | ⚠️ **never use as track** |
| Rotation centre → front plate tip | **0.345 m** | defines where `base_link` sits |
| `front_overhang` (scan origin → bumper) | **0.337 m** | ✅ measured: parked square to a wall, zero gap, 178 consecutive scans with min == max |
| `/scan` range scale | **0.9845** (−1.55%) | ✅ **Two-point tape calibration 2026-08-12.** Tape 1.960 m → `/scan` 1.932; tape 0.345 m → `/scan` 0.342. Fitting `scan = a·tape + b` gives **a = 0.9845, b = +2.3 mm** — i.e. a pure scale error with **no meaningful offset**. 🔑 Two consequences: **`front_overhang` 0.337 m is confirmed to within 2 mm** (a wrong overhang would appear as an offset), and `/scan` reads **short**, so the reflex triggers slightly **early** — the safe direction. Apply the 1.55% when comparing `/scan` travel against any other ruler |
| Corridor half-width | **0.275 m** | plate half-width 0.225 + 50 mm heading margin |
| `base_link → camera_link` | **(0.000, 0.000, 0.305) m** | camera is centred |
| Floor forward tilt | **+0.37°** | RANSAC plane fit |
| Camera roll residual | **1.54°** | ⚠️ **uncorrected**, from `z = +0.00638x +0.02681y −0.01203`, one location only |

**Why the camera stays centred.** Centring puts the 0.300 m minimum-range circle 45 mm *behind* the
bumper, so there is no blind strip ahead of the vehicle. It trades a known, static, croppable object
(the rover's own plate in frame) for a genuine blind spot. Moving the camera breaks `cam_x = 0`,
`front_overhang` and the TF chain simultaneously.

> ⛔ **Never derive a dimension — look it up.** Two shipped bugs came from assumed geometry: the
> wheelbase used as the track (under-reporting every yaw rate by 28%), and a footprint built from
> the wheelbase rather than the plate. Both reached running code.

---

## 5. Calibrated constants

Every value here was measured against an external reference. **The method matters as much as the
number** — it tells you when the value stops being valid.

| Constant | Value | How it was obtained |
|---|---|---|
| `erpm_to_ms` | **0.003900** | Wall-referenced, 5 powered runs both directions, 0.146–0.364 m/s. Odometry over-reported ground distance by **18.8%** (mean 1.188, sd 0.029). Residual after correction **~2.4%**. ✅ **INDEPENDENTLY CORROBORATED 2026-08-12**: a tape across `house_map_v4` agreed on the long axis to within a few percent — **3.200 m mapped vs 3.1 m taped**, i.e. the map reads **+3.2% large**, just above the ~2.4% residual quoted here and nowhere near a gross error. The caveat *"includes mean slip — re-measure on a different floor"* stands, but the constant has now survived a ruler sharing none of its assumptions. ⚠️ Note the geometric value is 0.004633 and a 2.13 m tape drive gave 0.004463 — **both disagree with 0.003900 by ~16–19%, and the map says 0.003900 is the right one.** Do not "correct" it toward the geometric figure without repeating the map check. |
| ~~`erpm_to_ms` 0.004633~~ | **superseded 2026-08-09** | The slip-free eRPM→*wheel rotation* figure from a hand push over 5 counted revolutions. Correct as a description of the drivetrain, **wrong for odometry**, which needs ground distance. Itself replaced an assumed-geometry value that was 12.2× too small. Full history in `config/rover_odometry.yaml`. |
| `track_width` | 0.310 m | Tape, hub centre to hub centre |
| `deadband_erpm` | 5.0 | Standstill noise: 2930 samples per wheel, min 0, max 0 |
| Camera gyro bias | +0.72 °/s | Large but stable; re-estimated live at rest because it creeps thermally (+0.008 °/s over 3 min) |
| Camera gyro scale | 0.9996 – 1.0012 | Three full circles against a wall at 6.1, 9.3 and 19.5 °/s. **Not rate-dependent.** |
| Camera clock rate | +0.0022% of ROS time | 0.008° per revolution. ⚠️ gyro stamps are **device uptime**, not ROS time |
| Colour ↔ depth sync | 0.3 ms | From a recorded bag |

### PX4 rover parameters

```
RO_YAW_RATE_P     0.08        RO_MAX_THR_SPEED   0.6    m/s
RO_YAW_RATE_I     0.0         RO_YAW_P           2.0
RO_YAW_RATE_CORR  1.8         RO_SPEED_LIM       0.70   m/s
RO_YAW_RATE_LIM   85.9        ⚠ deg/s, NOT rad/s
```

> 🔴 **`RO_YAW_RATE_I = 0` is deliberate.** Integral windup was one of the two causes of the yaw
> problem. **Never restore it to 0.1.** Re-read all of these off the FC after every reboot —
> `tools/set_param.py NAME` reads and writes over MAVLink and refuses while armed.

---

## 6. Odometry and heading

> 🔑 **THERE IS NO VIO ON THIS VEHICLE, AND NEVER HAS BEEN.** Stated plainly because its absence has
> been mistaken for its presence: nothing here performs visual-inertial odometry. Position is
> **wheel ERPM + camera gyro dead reckoning** and nothing else (`wheel_odometry_node`). RTAB-Map
> does **not** supply visual odometry either — `rtabmap_localization.yaml` sets `odom_frame_id: odom`
> ("take odom from TF, not the message"), so it *consumes* this dead reckoning and adds `map→odom`
> corrections on top, exactly like AMCL. Consequences: local, short-range navigation needs no VIO and
> is already served (§13); map-relative navigation depends entirely on RTAB-Map committing those
> corrections, which is **unmeasured** (§9). Adding VIO is not the fix for anything currently known
> to be broken.
>
> ⚠️ **PX4 never knows where it is, by design.** `rover-ekf-bridge` feeds the FC **velocity only**,
> via `LocalPositionMeasurementInterface`. Measured 2026-08-10 with the bridge stopped:
> `xy_valid false`, `v_xy_valid false`, `heading_good_for_control false`, `dead_reckoning true`.
> Position lives in ROS, not in the flight controller. Do not look for it there.

**Wheels for distance, a gyro for heading.** This is a skid-steer: it has no steering axle and can
only rotate by forcing all four tyres to scrub sideways. Slip is not a defect to minimise, it is the
turning mechanism — which means `(v_right − v_left) / track` measures a quantity the wheels
**cannot observe at all**.

### Heading source priority

| Source | Drift (stationary) | Status |
|---|---|---|
| `camera_gyro` — Gemini 336L, 195 Hz | **0.0015 – 0.0028 °/s** | ✅ **default**, unaffected by driving |
| `gyro` — PX4 EKF attitude | 0.005 – **1.106** °/s | 🔴 fallback only — erratic, sign flips |
| `wheels` | n/a | ⛔ cannot work — diagnostic only |

### Measured drift — every figure, wall-referenced, rover confirmed stationary

| Condition | Wall says (truth) | Source says | Rate |
|---|---|---|---|
| FC, parked cold, 476 s | −0.07° | −2.46° | **0.005 °/s** |
| FC, at rest after a drive, 96 s | 0° | −18.6° | **0.19 °/s** |
| FC, 111 s after a 0.364 m/s drive | +0.01° | −18.88° | **0.170 °/s** |
| FC, 21 s after a turn | −0.18° | **+23.23°** | **1.106 °/s** ← worst |
| FC, 41 s after a turn | −0.07° | −7.17° | 0.175 °/s |
| FC, one 4-minute session, total | ~0° | **+27.1°** | — |
| **Camera gyro, cold, 9×20 s windows** | — | — | sd **0.0028 °/s** |
| **Camera gyro, straight after driving** | — | — | sd **0.0015 °/s** |
| **✅ `/odom` yaw AFTER THE FIX, 144 s, 538 samples** | +0.060° (= the wall's own noise) | **+0.040°** | **✅ 0.00028 °/s** |

**That last row is the acceptance result.** `/odom` yaw now tracks the wall to within 0.02° over
two and a half minutes — the drift is *below the reference's own measurement noise*, and roughly
**3950× better** than the FC's worst case. Over a 263 s mapping run that is 0.07° of heading error
instead of tens of degrees.

The flight-controller heading was measured, against a wall, inventing **+23.23° in 21 seconds** on a
rover confirmed stationary, and **+27.1° across one 4-minute session**. It is erratic rather than
biased — the sign flips, and it is not proportional to motor effort — so **no gain or offset can
compensate it**.

**The gyros are not at fault.** Three independent IMUs from three vendors do not drift 4000°/hour
together, and the stored `CAL_GYRO*_ZOFF` values are normal (−0.100 / +0.242 / −0.038 °/s). What
degrades is the EKF's **fused yaw**. Two untested suspects:

- the single internal magnetometer (`EKF2_MAG_TYPE=1`, the only yaw observer indoors)
- GPS aiding (`EKF2_GPS_CTRL=7`) — ⚠️ **the publish rate is contradicted and unresolved.** Recorded
  earlier as `vehicle_gps_position` publishing at **3.3 Hz**; measured **0.0 Hz over 6 s on
  2026-08-10**, with one advertised publisher but no messages. Both may have been true when taken.
  **Resolve before using this as evidence either way** — if GPS is silent, it is a weaker suspect
  for the fused-yaw fault than the magnetometer, and the test order should change accordingly.

### Implementation requirements for the camera-gyro path

- Rotate into `base_link` **via TF**, never a hand-derived axis map — a remount must not silently
  invert the sign.
- Integrate **`header.stamp` deltas, never arrival time.** An arrival-time probe under-read rotation
  by up to 4.5% because dropped samples at peak CPU applied the following lower rate across the gap.
- **Refuse to integrate across gaps** (>50 ms) and count them.
- Re-estimate bias from a rolling at-rest window; do not calibrate once.

> ⚠️ **A gyro gives no absolute heading.** It integrates rate. It fixes map-building drift; it does
> not tell you which way is north. Also: this **couples odometry to `rover-camera`**, which is the
> component that degrades silently (§10).

---

## 7. Motion envelope

What this vehicle physically can and cannot do. Plan around it; it will not be tuned away.

### Yaw: a large friction deadband

| steer output | 0.055 | 0.348 | 0.425 | 0.484 | 0.573 | 0.935 |
|---|---|---|---|---|---|---|
| yaw rate (rad/s) | 0 | 0 | 0 | **0.67** | **1.51** | **4.11** |

```
above the deadband:   yaw_rate ≈ 7.6 × (steer − 0.40)   rad/s
minimum achievable:   ≈ 0.67 rad/s  (38 °/s)
time constant:        ≈ 2 s
```

**There is no slow rotation.** Below ~38 °/s the rover does not turn at all; above it, it snaps.
Design for **coarse discrete turns at ~1.2 rad/s**. A short pulse still gives a small *angle* — the
floor is on rate, not on angle — so **tap-and-stop** is how to make small heading changes.

### Speed

Drivetrain reaches ~0.58–0.60 m/s. Mapping and careful work run at 0.15–0.25 m/s. Powered runs slip
~19% against ground truth (§5).

> 🔑 **PX4 Manual bypasses the yaw-rate loop entirely.** Firmware-verified: `manual()` publishes a
> *steering* setpoint; only `acro()` publishes a rate setpoint. So manual drives are **not
> yaw-gated** — but the collision reflex **does not run in Manual either**, and the operator is then
> the only safety layer. AutoNav publishes speed + rate, so the yaw loop gates every AutoNav task.

---

## 8. Perception

A 2D lidar sees one horizontal slice at one height. A depth camera sees a 3D volume ahead. They are
complementary, not redundant — and we only have the second.

| Obstacle | 2D lidar | Depth camera |
|---|---|---|
| Table top, desk edge | ❌ passes underneath | ✅ |
| Chair seat, shelf, overhang | ❌ | ✅ |
| Low box, threshold, cable | ❌ passes over | ✅ |
| Stairs / drop-off | ❌ | ✅ |
| Wall, door frame | ✅ | ✅ |
| **Behind and beside** | ✅ | ❌ **physically cannot** |

The depth camera covers a **92° forward wedge** with ~4 m of usable range. The other **268° is
unmeasurable, permanently**, until a 360° lidar is fitted. That single fact drives most of §11.

### Scan pipeline

```
/camera/depth/image_raw ──▶ depthimage_to_laserscan ──▶ /scan      (has base_link→camera_link TF)
/camera/depth/points    ──▶ pointcloud_to_laserscan  ──▶ /scan_3d   (NO TF, parallel)
```

`/scan` is the one with the transform — **never repoint `rover-scan.service` at `/scan_3d`.**
`/scan_3d` is steadier close in (6 mm versus 74 mm) and agrees to 8 mm at 1.34 m, but it **contains
the rover's own top plate by design** — every consumer must reject its own footprint.

> ⛔ **Radial range cuts cannot express a rectangular body.** A dropped ray reads as *infinite
> clearance*. The scan pipeline was fail-**open** until this was closed with `range_min` 0.31 plus a
> per-ray rectangular footprint reject. **Check any new consumer for the same failure.**
>
> 🔴 **That instruction was already written here, and the collision reflex was still missed.** It
> read `min_x = inf` as "nothing there" and drove into a wall on 2026-08-10 (§10). The warning above
> is not sufficient on its own — a consumer must be checked, not merely warned.

### Scan health — how to tell "sees nothing" from "cannot see"

An empty scan and a blind scan are the same value (`inf`) and must be separated by a *different*
measurement: the fraction of the **whole scan** carrying a valid range. Whole-scan, not sector,
because health must not depend on where the vehicle is pointed — a genuinely empty room also yields
zero returns in the corridor.

| Quantity | Measured 2026-08-10 | Method |
|---|---|---|
| Whole-scan valid rays | **87.5%** (560/640), spread <0.5% | 400 consecutive scans, rover parked, room lit |
| Forward-sector valid rays | **80.4%** (222/276) | same run, ±0.35 rad sector |
| **Blind — lens fully occluded** | **0.1 – 8.2%** steady (mean 3.1%) | **MEASURED 2026-08-10**, two 18 s occlusions on stands, 90 samples |
| Partial occlusion (transition) | up to **29.4%** blind-side, down to **51.2%** healthy-side | same run — the hand entering/leaving the FOV |

> ⚠️ **"Blind ≈ 0%" was an assumption and it was wrong.** Measured steady blind is 0.1–8.2%, not 0.
> Margin to the threshold is **~4×**, not infinite. The transition band (29.4% ↔ 51.2%) is where a
> *partially* covered lens sits, and 0.35 falls inside it — which is the intended behaviour: as
> validity falls the gate errs toward blocking.

⇒ **Threshold 0.35** (`collision.min_valid_fraction`), ~2.5× below healthy and ~4× above blind.
Anything that destroys returns — a starved camera, a non-reflective surface, an obstacle inside the
0.308 m depth minimum — drives this **down**, so the gate fails safe. Cost: a genuinely open space
deeper than `range_max` also reads low and blocks. That is the correct direction to be wrong.

---

## 9. Mapping and localization

**RTAB-Map RGB-D, not slam_toolbox.** slam_toolbox does 2D scan matching and needs a 360° ring; 92°
gives it too little overlap. RTAB-Map matches visual features in 3D and closes loops by *recognising
places*, which is designed for a narrow forward FOV. **The limitation is the algorithm choice, not
the sensor.**

### Pipeline

```
record a bag on the Pi  ──▶  replay-map at 0.3× (ROS_DOMAIN_ID=42)  ──▶  localize on the Pi
```

Live-streaming RGB-D off the vehicle is not viable: `image_transport` offers only `raw_pub` here, so
it is ~100 Mbit/s rather than 13. **SD write ceiling is 27.4 MB/s**, which is why mapping bags are
recorded at 15/15 fps (17.3 MB/s) rather than 30/30 (~34 MB/s).

### What made the difference

| Cause | Effect on the map | Fix |
|---|---|---|
| FC heading drift | up to 1.1 °/s of phantom rotation | camera gyro (§6) |
| 19% odometry over-report | **every wall drawn several times** — 4.3 m of error in a 3.2 × 3.9 m room | `erpm_to_ms` (§5) |
| Plate mask silently ignored | 11.8% of the cloud was the rover's own body, traced along the driven path | `Grid/DepthDecimation: 2` |

> ⚠️ **The decimation trap — the rover maps its own body.** `Grid/DepthRoiRatios` crops the bottom
> 35% of each frame to hide the rover's top plate. RTAB-Map requires the cropped height to divide
> **exactly** by `Grid/DepthDecimation`. At 640×360 the crop leaves 234 rows, and 234/4 = 58.5 — so
> the mask was **discarded on every frame**. 234/2 = 117 works. **Recheck whenever the colour height
> changes.**
>
> **Two independent ways to recognise it:**
>
> - **Visually — how it was actually caught.** The plate is drawn at *every position the rover
>   occupied*, so the map shows **a trail through the floor tracing exactly where the vehicle drove**.
>   The operator spotted this by eye before any log was read. **31,299 of 266,053 points = 11.8% of
>   the cloud** in `house_map_v4`.
> - **In the log.** `util3d.cpp:1251::cloudsRGBFromSensorData() Cannot apply ROI ratios [...] cannot
>   be divided exactly by decimation parameter (4). Ignoring ROI ratios...` — **once per frame**, 740
>   times in one 488 s replay. Loud, and dismissed as noise for weeks.
>
> ⚠️ Earlier notes recorded this as a *"real parameter gap, **negligible effect**"*. **That assessment
> was wrong and is withdrawn.** `tools/map_review.py` now strips the rover body unconditionally, so a
> map review survives the bug recurring.
>
> ⚠️ **Unproven:** it was further claimed that the plate marks the driven corridor as *obstacle* and
> would wall Nav2 in. The grid comparison does **not** support that — ray tracing from later poses
> clears what the plate marked from earlier ones. The **cloud** contamination is measured; the
> **grid** consequence was inference and is withdrawn pending evidence.

### Localization settings that matter

`RGBD/MaxOdomCacheSize: "0"`. The default requires a second localization corroborated against the
odometry travelled in between; with a drifting heading that never converges and localization waits
indefinitely. Setting 0 accepts the first fix. **Once heading is trustworthy this is worth
revisiting** — corroboration is a real quality filter when the odometry deserves trust.

---

## 10. Known faults

| Fault | Signature | Action |
|---|---|---|
| 🔴 **Speed commands are not honoured — the vehicle runs far faster than asked** | Commanded vs measured (`/odom`), 2026-08-12: **0.05 → ~0.11 · 0.06 → 0.146 · 0.25 → ~0.9 m/s.** The ratio *grows* with the command, and there is a floor near **0.1 m/s** — the rover cannot be asked to go slower. Nothing errors; the setpoint is emitted correctly and the wheels simply spin faster than commanded. | ⛔ **SAFETY-CRITICAL, not tuning.** Stopping distance is set by the coast, so the vehicle cannot be commanded to stay inside the speed band its own stopping distance requires. **This is what drove the rover into a wall on 2026-08-12**: a run sized from a 0.25 m/s command executed at ~0.9 m/s and stopped 0.020 m short. ⛔ **Gate every moving test on measured `/odom` speed, never on the command** (`--max-measured-speed`). Suspect the PX4 rover-module speed scaling (`RO_*`) |
| ⚠️ **An `/odom`-denominated distance limit is not a limit at crawl** | A `--max-travel` backstop reads in `/odom` metres, and `/odom` under-reads **22%** below ~0.15 m/s, so a 1.70 m limit only fires after ~2.2 m of real ground. | Use a backstop that does not depend on odom scale — clearance (`--min-bumper`). Applies to any guard built on `/odom` distance |
| **A disarmed external mode reads as selected but is NOT RUNNING** | `nav_state` = 23 and QGC agree the mode is chosen, yet `/fmu/in/rover_speed_setpoint` is **silent** and `"AutoNav activated"` never appears in the journal. `px4_ros2` sets `is_active` only when `nav_state` matches **and** (`armed` OR `activate_even_while_disarmed`) — `ModeBase::vehicleStatusUpdated`. Library default is **false**, so `updateSetpoint()` is never called. 🔑 **A silent setpoint topic then looks exactly like a reflex that held — this is a false-PASS generator, and it produced one aborted run on 2026-08-11.** | Fixed for AutoNav: constructed with `activateEvenWhileDisarmed(true)`, so the loop runs disarmed with the wheels unable to turn. **Never read a quiet setpoint topic as evidence; prove the channel carries a non-zero baseline first** |
| **`/scan` dies and `rover-scan` still reads `active`** | Kill `depthimage_to_laserscan_node` (S2 does this deliberately) and `/scan` **never comes back**, while `systemctl is-active rover-scan` says `active` and `Restart=always` never fires. Cause: the unit's MAIN process is the *launch*, which survives on its other child, the `static_transform_publisher` — so systemd sees a healthy unit. Cost 10 minutes of a dead sensor on 2026-08-11. | `sudo systemctl restart rover-scan`, then **verify `/scan` is actually publishing**. ⛔ `is-active` is not evidence for any multi-node launch unit |
| **Camera colour collapses under CPU load** | Colour rate falls (19.9 → 11.2 Hz in six minutes) while depth holds. Every health check still passes; no error logged. | Measure the colour rate; restart `rover-camera` |
| **Half-dead camera restart** | Unit active, parameters answer, gyro and accel streams start — **depth and colour never do**, silently. `systemctl is-active` does not catch it. | Restart again; verify with `tools/camera_restart_check.py` |
| **`/odom` dies at rest** | VESCs doze; only address 13 stays online. Intermittent. | Handled — `publish_at_rest` emits a genuine zero-velocity sample |
| **Tooling steals cores** | `claude` 55–86%, its npm update check 84%, the pemmican MOTD hook 100% | Both hooks disabled; `tools/cpu_catcher.sh` traps the rest |
| **Glossy dark surfaces** | A blue steel almirah maps ~1.5× thicker than matte walls and lands closer than it is. *Suspected, not proven.* | Errs toward the rover — **fails safe** for navigation |
| 🔴 **Collision reflex fails open on a fresh-but-empty scan** *(FIXED 2026-08-10, awaiting validation)* | Alternating `BLOCK`/`clear` in the collision-diag log with **`scan_fresh=yes` throughout**, each `clear` reading `bumper=inf`. The rover creeps ~0.3 m closer per cycle. | Perception-health gate, §8. **Recognition cue: `collision-diag: BLOCK forward (BLIND)` with `scan_fresh=yes`** |

> 🔴 **THE 2026-08-10 00:30 WALL CONTACT — the reflex drove the rover into a wall while working
> exactly as written.** It is a **ratchet**, and the threshold was never wrong:
>
> ```
> BLOCK bumper=0.35 raw=0.68 → clear bumper=inf → BLOCK 0.11 → clear 0.38 → BLOCK 0.26 → clear inf → BLOCK 0.08
> ```
>
> It blocked at exactly 0.35 m **whenever it could see**. Between glimpses the corridor returned
> **zero valid rays** ⇒ `min_x = inf` ⇒ `frontClearance() = inf` ⇒ `inf > clear_distance` ⇒
> `_blocked = false` ⇒ forward permitted. The existing fail-safe covered a **stale** scan
> (`if (!scan_fresh) return _require_scan`); it did not cover a **fresh but empty** one. `mode.hpp`
> even stated the assumption in a comment: `_front_min_range = min_x; // inf => nothing in the corridor`.
> **"Cannot see" was being read as "nothing there."**
>
> **Fix (2026-08-10):** a perception-health gate placed *before* the clearance test, so a blind frame
> can never reach the release branch and clear a block earned by the last frame that could see.
> Constants and rationale in §8; threshold is the parameter `collision.min_valid_fraction`.
>
> ❓ **Still not established: why the wall vanished from the scan.** Candidates — inside the 0.308 m
> depth minimum, poor IR return off that surface, or the wall leaving the ±20° corridor as the rover
> yawed. The gate holds regardless of which it was, but the cause is a separate open question.

> ⛔ **A restart does not clear a CPU-starvation latch.** The documented video-pipeline failure is
> identical in appearance to a wedged camera and is cured only by removing the CPU contention first.
> **Stop the competing consumers, then restart.**

---

## 11. Inviolable rules

These follow from physics and geometry, not from preference. **Each has already cost something.**

1. **Never reverse into unseen space.** The depth camera is forward-only and the collision reflex
   cannot see behind. There is no sensor that would catch it. Reversing is an operator-supervised
   action, never an autonomous one.

2. **Never clear a spin from any `/scan`.** A 92° wedge cannot certify the 268° the rover would
   rotate through.

3. **Nav2's default spin and back-up recoveries must stay deleted.** They are precisely the two
   manoeuvres this vehicle cannot clear. A stock Nav2 config reintroduces them silently.

4. **The kill switch must be proven in AutoNav before every armed autonomous campaign.** ✅ Passed
   2026-07-22, re-confirmed 2026-08-10 (§12). It is the backstop for every armed test after it, so
   re-confirm it after any change to the mode, the setpoint type or the tune — not once, forever.

5. **Never key a camera by `/dev/videoN` or by-id.** Use `usbcam-<vidpid>-<serial>-i<iface>`. The
   Orbbec's video nodes appear and vanish with `rover-camera`, and the Pi's own `rpivid` and
   `pispbe-*` devices renumber them.

6. **Never record resolution, frame rate or bitrate as fact.** Read them live. **Negotiated is not
   delivered** — colour has negotiated 30 fps while delivering 8.

7. **Absence of a return is not absence of an obstacle.** Every perception consumer must distinguish
   "nothing is there" from "I cannot see", because both arrive as the same value. A consumer that
   treats `inf`, an empty scan or a dropped ray as *clearance* is fail-**open**, and this vehicle has
   now produced that bug twice — once in the scan pipeline (§8), once in the collision reflex, where
   it caused a wall contact (§10). **Gate on sensor health before acting on sensor value.**

---

## 12. Test ladder and status

Safety tests gate capability tests. **Pass criteria, not opinions.**

| ID | Test | Pass criterion | Status |
|---|---|---|---|
| **S1** | Kill switch in AutoNav | Wheels stop immediately, disarms | ✅ **PASSED** 2026-07-22, **re-confirmed 2026-08-10** — see below |
| **S2 stands** | Sensor loss gates a real setpoint (disarmed) | Emitted speed forced to 0 while blind, and **non-zero while seeing** | ✅ **PASSED 2026-08-11** — `tools/s2_stands_test.py`. Baseline 0.150 seeing → 0.000 under two covers, 30 consecutive blocked samples, zero leakage, recovery ≤1 s. Needed `activateEvenWhileDisarmed(true)` (§10) |
| **S2 floor** | Sensor loss while actually driving | Forward blocked within 0.5 s of losing `/scan` | ✅ **PASSED 2026-08-11** — armed, floor, 0.08 m/s, `tools/s2_sensor_loss_test.py`, **3 runs**. Decision latency (last `/scan` → emitted setpoint zero) **509 / 514 ms** against a 500 ms `scan_timeout` — ~10 ms of overhead, twice. Wheels reached zero **1000–1078 ms** after the last scan in all three |
| **Standoff ≥300 mm from a WALL** | Drive at an obstacle, `/scan` healthy | Settled bumper clearance ≥ 0.30 m | ✅ **PASSED 2026-08-12, n=3, and TAPE-CONFIRMED** — `tools/collision_standoff_test.py`. Settled **0.315 / 0.316 / 0.342 m** by `/scan`; the third taped **0.345 m**. Trigger fires within **1–6 mm** of the 0.350 m spec every time. ⚠️ **The pass carries a SPEED BOUND**: at ~0.11 m/s it leaves 0.345 m; at **~0.9 m/s it left 0.020 m and the rover CONTACTED the wall** (2026-08-12). The threshold is not what sets standoff — **the coast is**, and the coast scales with speed |
| **S2b** | Camera loss (`rover-camera` killed) | Forward blocked; heading loss handled | ⬜ untested — harsher than S2: loses `/scan` **and** the heading gyro together, a risk created by making heading depend on the camera (§6) |
| **S3** | Yaw loop diagnosis | Open vs closed loop | ✅ solved — friction deadband + windup |
| **T1** | Speed tracking, 5 s at 0.2 m/s | Sustained `/odom` within ±20% | ✅ validated |
| **T2** | Straight goal, clear 2 m corridor | Arrives within 0.20 m, no collision stop | ⬜ not run |
| **T3** | One offset obstacle | Routes around, keeps inflation clearance | ⬜ not run |
| **T4** | Fully blocked corridor | Stops cleanly, reports failure, **does not spin or reverse** | ⬜ not run |
| **T5** | Dynamic obstacle | Stops before contact | ⬜ not run |
| **T6** | Map build + re-localize | Pose recovered after restart, no operator input | ⚠️ map good; **pose not fairly measured** |
| **T7** | Return to base | Routes home from an arbitrary mapped point | ⬜ not run |

> ✅✅ **S1 RE-CONFIRMED END-TO-END 2026-08-10 on the current stack.** With `rover-ekf-bridge`
> running, AutoNav engaged and held (`nav_state 23`), the rover drove at 0.15 m/s reaching
> **155 rpm peak**, ch8 was hit, and it **disarmed with the wheels at zero in the same 50 Hz
> sample** — measured latency 0 ms, i.e. **under 20 ms**, which is the resolution limit of the
> measurement rather than the switch. `rover-autonav-mode` did not restart during the run.
> Tool: `tools/s1_kill_test.py` (never arms, never disarms; aborts before motion if AutoNav
> does not hold). This reproduces the July result on the camera-gyro heading, the corrected
> odometry scale and the current tune.
>
> ✅ **S1 originally passed 2026-07-22 — the "untested" record was wrong.**
> The L2 floor test log records *"kill switch (ch8) works ARMED inside AutoNav — now confirmed
> live"*, and independently, during a software-armed run in the same session the rover drove at a
> wall and the operator *"stopped it with the KILL switch (ch8) before impact"*. Ch8 stopped a
> moving, armed rover in AutoNav, twice evidenced.
>
> 🔑 **The precondition, and why it looked broken again on 2026-08-09:** AutoNav uses
> `RoverSpeedRateSetpointType`, whose configuration sets `velocity_enabled = true`, so PX4 requires
> a valid local **velocity** estimate. Indoors the only source is **`rover-ekf-bridge`** — the
> July run records *"rover-ekf-bridge started → v_xy_valid true → AutoNav arms"*. With the bridge
> disabled, `failsafe_flags` reads `local_velocity_invalid: true`, and PX4 — which relaxes mode
> requirements while disarmed and enforces them armed — refuses the switch. Measured: disarmed
> `DO_SET_MODE 4/11` → nav_state 23 and holds; armed → refused, as is every other mode.
>
> ⇒ **S1 is not a pending test. It is a passed test with a precondition that must be restored
> before any AutoNav run: start `rover-ekf-bridge` (on the FLOOR only — the limit-cycle hazard is
> wheels-UP), confirm `v_xy_valid`, then arm.**

```
S1 kill ─▶ S2 sensor loss ─▶ T1 speed ─▶ T2 straight goal
                                             │
     S3 yaw ─────────────────────────────────┴─▶ T3 ─▶ T4 ─▶ T5 ─▶ T6/T7
```

---

## 13. Fitness assessment — dated

**A verdict is only meaningful with a date on it.** This section records what was fit for use, when,
and on what evidence. Re-assess rather than assume; supersede rather than edit in place.

### 2026-08-10

| Subsystem | Verdict | Evidence |
|---|---|---|
| **Odometry (heading)** | ✅ **fit** | drift **0.00028 °/s** (144 s parked, below the wall's own 0.06° noise) · scale **0.04–0.47%** over three full circles at 6.1, 9.3 and 19.5 °/s · unaffected by driving |
| **Odometry (distance)** | ✅ **fit** | ~**2.4%** residual after the `erpm_to_ms` correction, from 5 wall-referenced runs (was 19%) |
| **Map `house_map_v4` — visual / cloud layer** | ✅ **fit** | operator-verified: shape, objects and floor correct, **walls single**. One object (a glossy steel almirah) maps ~1.5× thick and closer than reality — errs toward the rover, so it fails safe. **This is the layer localization matches against** |
| **Map `house_map_v4` — 2D occupancy grid** | ⚠️ **NOT ESTABLISHED** | 🔑 **one database, two products, only one verified.** The operator check validated the cloud; the grid a *planner* consumes was never examined. Known grid-specific problems: the rover's own plate was 11.8% of the cloud (fixed only for FUTURE maps via `Grid/DepthDecimation: "2"`), and the v5 reprocess gained ray-tracing spikes and **was not adopted**. Check the grid before blaming a planner |
| **Localization** | ❌ **NOT MEASURED** | both 2026-08-09 attempts were invalid — one truncated window, one starved camera. Do not quote a figure until it is re-run per §14 |
| **Localization — corrections committed?** | ❌ **RECORDED FAILURE, UNRESOLVED** | `rtabmap_localization.yaml` records relocalization firing **4 good fixes in 12 min and committing NONE** — `map→odom` did not move. Accuracy is irrelevant if the transform never updates. **This single unknown gates all map-relative navigation**, and costs one run to settle |
| **Collision reflex — perception gate (sensing)** | ✅ **VALIDATED 2026-08-10** | occlusion test on stands, two 18 s covers: validity 87.5% → 0.1–8.2%, gate blocked continuously, recovered to clear in **≤0.5 s** both directions. **The run reproduced the ratchet**: while blind the corridor reported `0.52 → 0.34 → 0.78 → inf → 0.62 → inf → 2.63`, every one of which the *ungated* logic would have read as clearance |
| **Collision reflex — blocking motion (acting), SETPOINT** | ✅ **VALIDATED 2026-08-11** | S2 stands, `tools/s2_stands_test.py`, AutoNav engaged **disarmed** at 0.15 m/s commanded. Baseline 27 s seeing → emitted setpoint **0.150** (this control is what makes the zeros meaningful); two covers → **0.000** at **0.0–3.8%** validity, **30 consecutive blocked samples, zero leakage**, recovery inside one sample (**≤1 s**, the sampling period). Required `activateEvenWhileDisarmed(true)` — see §10 |
| **Collision reflex — blocking motion (acting), WHEELS** | ✅ **VALIDATED 2026-08-11** | S2 floor, armed, 0.08 m/s, 3 runs. On sensor loss the wheels stop: **decision 509 / 514 ms** (n=2, 5 ms spread) vs a 500 ms `scan_timeout`; **coast 569 / 486 ms** (83 ms spread — mechanical, not the reflex); total 1000–1078 ms (n=3). 🔑 **The two halves have OPPOSITE fixes** — a slow decision means `scan_timeout`/update rate, a slow coast means a speed limit or braking — so never quote the combined number as the reflex's latency |
| **Stopping DISTANCE from a wall (≥300 mm)** | ⚠️ **MEASURED ONCE, BY 15 mm, ONE WITNESS** | 2026-08-11: fired at 0.347 m, coast 0.034 m, **settled 0.315 m vs 0.300 required**. The reflex triggers where it says it does (3 mm error). But the margin is 15 mm, n=1, and **`/scan` is the only witness** — no tape. 🔑 **The coast model predicted it**: ½·v·t with the S2 coast time gives 0.036 m against 0.034 m measured, so coast *is* the term that sets standoff, and it scales with speed |
| **Speed tracking at CRAWL** | 🔴 **BROKEN — the vehicle ignores low speed commands** | Commanded **0.060 m/s**, actual **~0.146 m/s** (wall closed 1.543 m in ~10.6 s) — **2.4× the command**. `/odom` velocity read 0.04–0.21 m/s throughout, never near 0.060. T1 validated tracking at 0.2 m/s (±20%); there appears to be a floor near 0.12–0.15 m/s, plausibly friction-deadband compensation overshooting. ⛔ **You cannot currently buy standoff margin by slowing down** — asking for less speed does not produce less speed |
| **`/odom` scale is SPEED-DEPENDENT** | 🔴 **CONFIRMED 2026-08-12 — this is the answer to the 17%** | Two runs at the same wall, ~8× apart in speed: at **~0.9 m/s `/odom` over-reads +5%**; at **~0.11 m/s it under-reads −22%** (−22.3% after the `/scan` scale correction), reproduced twice. One mechanism fits: **wheel slip rising with speed.** Implied constant **0.00366 at speed** vs **~0.0049–0.0051 at crawl**. The fast figure sits beside the configured **0.003900**, which is why `house_map_v4` — built at normal driving speed — taped true. 🔑 **`erpm_to_ms` cannot be "fixed" with a new single number**: any correction must be speed-aware, or it breaks the regime the map was built in. ⚠️ The crawl figure **exceeds the geometric 0.004633**, which slip cannot do — so either the assumed 0.1524 m wheel diameter is small or the 516.7 ERPM-s/5-rev figure is off. **Unexplained; do not fit a story to it** |
| **Map scale (`house_map_v4`) vs tape** | ✅ **VALIDATED 2026-08-12 — and it vindicates `erpm_to_ms = 0.003900`** | Full-height wall planes measured from the cloud, then taped: **Y mapped 3.200 m vs taped 3.1 m** (map **+3.2% large**); **X mapped 2.450 m vs taped 2.6 m** (map **−5.8% small**). Operator quoted Y as "3.1, call it 3.1–3.2" — 3.1 is the reading, the rest is margin. 🔑 **The two axes err in OPPOSITE directions, which no global scale factor can produce** — and a 19% compression would have required Y to tape 3.83 m. The X shortfall is therefore **local**: an angled room would bias X *high* (D/cos θ), so what remains is a **full-height object standing ~0.15 m proud of the real wall**, which the full-height filter cannot distinguish from masonry. 🔑 **A named suspect already exists** — the glossy steel almirah recorded below as mapping *closer than reality*, same direction and rough magnitude. **Not confirmed**: all four planes measure 48–76 cm "thick" because the room is not square to the map frame, so thickness cannot single it out. For navigation the mapped 2.45 m is the *more* useful number — it is the free span. Method + plan view: `tools/map_review.py`, 0.20–1.40 m slab, 40 mm cells, only cells covering >75% of the slab vote |
| **S1 kill switch** | ✅ **passed** | 2026-07-22, **re-confirmed 2026-08-10**: 155 rpm peak, disarm and wheels-zero in the same 50 Hz sample (**<20 ms**) |

**Standing caveats on the odometry verdict — both permanent, neither a defect:**

- **No absolute heading.** A gyro integrates rate. It fixes map-building drift; it will never tell
  you which way is north. Anything needing absolute heading needs a different sensor.
- **Heading is coupled to `rover-camera`**, which is the component that degrades silently under CPU
  load (§10). Fallback to the FC gyro is automatic and loud, but the FC gyro is unusable — so a
  camera failure degrades heading to unusable, it does not merely reduce accuracy. The queued
  `vehicle_angular_velocity` DDS bridge would remove this coupling.

⇒ **What this permits:** dead reckoning over a mapping run (~1° heading, ~2% distance).
**What it does not permit:** ⛔ **the first autonomous drive (T2) — withdrawn 2026-08-10.** The
2026-08-09 assessment permitted T2; the wall contact that night showed the brake it relies on could
fail open (§10). The reflex is now validated on stands in both halves — sensing (A1, 2026-08-10) and
setpoint-acting (S2 stands, 2026-08-11) — and **S2 floor passed the same night**, armed at 0.08 m/s:
on sensor loss the wheels stop, with a decision latency of 509/514 ms against a 500 ms timeout.
**T2 at crawl speed is therefore permitted.** What is *not* permitted is raising the speed: the
≥300 mm standoff is unmeasured at current parameters, and the derived figures put it at risk somewhere
near 0.2–0.25 m/s. Run `tools/collision_standoff_test.py` before any speed increase.
Also not permitted: anything depending on a localization number, until localization is measured.

---

## 14. Method notes

How to get trustworthy numbers on this machine. **Each of these was learned by getting it wrong
first.**

- **Use an absolute reference.** Park square to a flat wall and RANSAC-fit a line to `/scan`:
  perpendicular distance to ±1 mm, bearing to ±0.26°, completely independent of the wheels. This
  single rig found the heading fault, the odometry scale error and the gyro validation.
  Tool: `tools/wall_probe.py`.

- **Check what else is running first.** No rate or CPU number means anything until
  `ps -eo pid,pcpu --sort=-pcpu` has been read. A "7.5 Hz cloud" was once a runaway at 72.7%.

- **Start the measurement, then go quiet.** Issuing commands during a perception measurement
  starves the camera and produces confident wrong answers.

- **Sample long enough.** A 110-second window and a 215-second window of the *same run* gave
  opposite conclusions about localization quality.

- **A grep that finds nothing proves the pattern absent, not the event.** A success string that
  differed by a few words once produced a headline "never relocalized" for a system that was
  relocalizing fine.

- **Settle ambiguity with a one-directional mechanism**, not with argument. Where two explanations
  fit, find the test whose outcome only one of them survives.

- **Record raw inputs in bags.** Without `/fmu/out/esc_status`, odometry can only be rescaled after
  the fact, never recomputed.

- **Rescaling recorded TF equals re-driving with a corrected constant**, exactly, because heading is
  gyro-derived and independent of `erpm_to_ms`. Same-bag re-mapping is the *rigorous* experiment,
  not merely the convenient one.

- **RANSAC planes; never percentiles.** p5/p50 gave opposite wrong answers for the floor tilt.
