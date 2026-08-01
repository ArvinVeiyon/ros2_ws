# Rover geometry — physical dimensions, camera mount, and what depends on them

**Single source of truth for the vehicle's physical numbers.** Written 2026-08-01 because these were
scattered across a planning section, a launch-file comment and two memory files, and a stale copy of
one of them (top plate `0.405 m`) had propagated into the collision-stop corridor and the Nav2
footprint. If you change a number here, work through [§6 Who consumes these](#6-who-consumes-these).

> **Convention.** All coordinates are in `base_link`: **+x forward, +y left, +z up** (ROS REP-103).
> `base_link` is the **skid-steer rotation centre**, on the ground plane below the plate centre-line.

---

## 1. Body

Re-measured **2026-07-26**; supersedes the 07-21 figures, several of which were wrong.

| Quantity | Value | Note |
|---|---|---|
| Top plate length | **0.730 m** | |
| Top plate width | **0.450 m** | ⚠️ 07-21 said 0.405 — **wrong**, and the bad value reached shipped code |
| Ground → top plate | **0.235 m** | ⚠️ 07-21 said 0.180 — wrong |
| Wheelbase (hub to hub) | 0.430 m | |
| Track (hub to hub) | 0.310 m | **not** the widest extent — wheels sit *inboard* of the plate |
| Front plate tip → front axle | 0.130 m | 07-21 said 0.150 |
| Rotation centre → front plate tip | **0.345 m** | defines where `base_link` sits |

Decomposition check: `0.130 + 0.430 + 0.170 = 0.730` ✓

**Footprint in `base_link`** — the plate is the widest and longest extent, so it is the footprint:

```
x from -0.385 to +0.345      (0.730 long, rotation centre 0.345 back from the tip)
y from -0.225 to +0.225      (0.450 wide)
circumscribing radius = sqrt(0.385^2 + 0.225^2) = 0.446 m
```

> ⚠️ **Use the plate, never the track or the wheelbase.** A footprint built from the 0.43 m wheelbase
> and a 0.405 m width gives a radius of 0.295 m, which leaves most of the rover *outside* its own
> footprint. That exact mistake was live in `nav2_forward.yaml` until 2026-08-01.

---

## 2. Depth camera mount

Remounted **2026-07-26** onto a custom printed bracket. Published by
`launch/depth_to_scan.launch.py` as a static TF; `rover-scan.service` launches it with no overrides,
so the launch-file defaults **are** the live TF.

| | Value |
|---|---|
| `base_link → camera_link` | **(0.000, 0.000, 0.305) m** |
| pitch | **0.0406 rad = 2.33° nose-down** |
| roll | 0.0100 rad = 0.57° |
| yaw | 0 |

**`cam_z 0.305 = 0.235 plate + 0.070 bracket`** — the camera sits **70 mm above the top plate**.

**Why `cam_x = 0`** — the camera is on the skid-steer rotation centre, so spinning in place is *pure
rotation* to scan matching. Off-centre, the sensor traces an arc and injects **fake translation on
every turn**, and the error recurs each time. Localization is the main open gap for autonomy, so this
property is worth protecting.

**Why the bracket is 70 mm and not 55** — the USB exits downward and plug plus cable occupy ~60 mm; at
60 mm the plug lands on the plate and the camera rocks on its connector (unmeasurable pitch error plus
cable strain). The hard floor is **17 mm** (`0.345·tan 2.79°`): below that the rover's own deck enters
the 2D `/scan` band as a permanent obstacle ~0.35 m ahead and the collision reflex never releases.
70 mm is ~4× that floor.

**Why the height came DOWN from 0.42** — `scan_height: 40` is only **±2.79°**, so the 2D `/scan` is a
thin slab covering `cam_z ± 0.049·d`; **`cam_z` chooses which horizontal plane the rover senses.** At
0.42 it was blind to anything under 371 mm at 1 m — most obstacles shorter than the rover's own
chassis. At 0.305 that becomes 256 mm. Do **not** raise `scan_height` to compensate: 120 rows would
see 0.20 m at 1.5 m but pull floor detection in to 2.9 m. The real fix is STL-19 owning `/scan`.

---

## 3. Depth camera optics — live-verified 2026-08-01

Read from `/camera/depth/camera_info`, not from a datasheet:

| | Value |
|---|---|
| Depth resolution | 848 × 480 |
| `fx` = `fy` | 409.85 |
| HFOV | **91.9°** |
| VFOV | **60.7°** (half 30.35°) |
| Lower FOV edge, incl. 2.33° pitch | **32.68° below horizontal** |
| **Minimum valid depth** | **0.308 m** (measured; nothing closer is ever returned) |

Depth is configured 30 fps but delivers ~15 Hz — see `project_perception_3d_costmap` memory.

---

## 4. The camera sees its own top plate — derived, then confirmed

This is a real, permanent feature of the mount, not a fault. Settled 2026-08-01 after being
misdiagnosed twice.

The camera is 0.070 m above the deck, looking down with its lower edge 32.68° below horizontal:

```
lower FOV edge crosses plate level at   0.070 / tan(32.68°)          = 0.109 m ahead
nearest returnable point at plate level  sqrt(0.308^2 - 0.070^2)     = 0.300 m ahead
plate front edge                                                      = 0.345 m ahead
==> the plate is RETURNED over 0.300 .. 0.345 m — a 45 mm sliver
```

**Measured in the live cloud: x = 0.301 … 0.347 m, z mean 0.231 m.** The model matches the returns to
~2 mm and the plate height to 4 mm. It reads as an *arc at constant range* rather than a flat deck
because its inner boundary **is** the minimum-range circle — everything nearer is in the blind zone.

**Full-FOV consequence:** the deck occupies roughly the **bottom third of the depth image** (the plate
tip sits 11.5° below the optical axis ≈ 83 px). Harmless for the 2D `/scan`, whose ±20 px band clears
it 4×. **Anything consuming the FULL CLOUD must exclude it** — `cloud_to_scan` and the Nav2 voxel
layer both do, or they mark a permanent obstacle around the rover's own nose.

### Do NOT move the camera forward to avoid this

Moving forward by Δ deletes the sliver once `Δ ≥ 45 mm` (the plate edge falls inside the min-range
circle). It is still the wrong trade:

| Δ forward | plate visible | blind strip **ahead of the bumper** |
|---|---|---|
| **0 (current)** | 45 mm sliver | **none** |
| 45 mm | gone | none |
| 100 mm | gone | 55 mm |
| 345 mm (at the nose) | gone | **300 mm** |

1. **Centring puts the 0.300 m min-range circle *inside* the footprint** — 45 mm behind the bumper —
   so there is **no blind strip ahead of the rover at all**. Moving forward pushes that circle out in
   front of the bumper. That trades a **known, static, croppable** object for a **genuine blind
   spot**: software can remove a fixed object, nothing adds back a blind spot.
2. **It breaks `cam_x = 0`** and its rotation-centre property (§2), costing localization quality to
   delete a 45 mm sliver.
3. Any remount invalidates `front_overhang`, the TF and the extrinsics, and requires the full
   as-built re-check at the foot of `launch/depth_to_scan.launch.py`.

**Correct fix: exclude the footprint geometrically in software.**

---

## 5. Derived quantities used by the safety path

| Quantity | Value | Origin |
|---|---|---|
| `front_overhang` (scan origin → bumper) | **0.337 m** | **Measured** 2026-07-28: rover parked square against a flat wall, zero gap, forward-sector min range read 0.337 m over **178 consecutive scans with min == max**. Agrees with the 0.345 m nominal to 8 mm. |
| Corridor half-width | **0.275 m** | plate half-width 0.225 + ~50 mm for heading error |
| Floor plane in `base_link` | `z = +0.00638x +0.02681y −0.01203` | RANSAC, 9833 inliers, residual RMS 9.9 mm, 2026-08-01, **one location** |
| Floor forward tilt | **+0.37°** | from that fit; crosses the 0.12 m height band only at x = 20.7 m |

> ⚠️ `front_overhang` is what makes the collision thresholds mean **bumper clearance** rather than
> lens clearance. Getting it wrong put the rover into a wall on 2026-07-28: 0.60 m of raw range was
> only 0.255 m of real clearance. **Re-measure it the same way after any remount.**

> ⚠️ The floor plane is from **one location**. Repeat at two or three spots before generalising.
> Measure it with a **RANSAC plane fit** — percentiles do not work, because depth noise grows with
> range, so p5 sinks and p50 climbs and they give opposite wrong answers (±2.6° against a true +0.37°).

---

## 6. Who consumes these

Change a number above and these must all be revisited:

| File | Consumes |
|---|---|
| `launch/depth_to_scan.launch.py` | `cam_x/y/z`, `cam_pitch/roll` — **this file IS the live TF** |
| `launch/cloud_to_scan.launch.py` | `min_height`, `max_height`, `range_min` (the footprint/plate exclusion) |
| `src/autonav_mode/include/autonav_mode/mode.hpp` | `kFrontOverhang`, `kCorridorHalfWidth`, the sector-vs-corridor geometry |
| `src/rover_nav2/config/nav2_forward.yaml` | `footprint` (local **and** global costmaps — keep them identical), `min_obstacle_height`, `obstacle_min_range` |
| `docs/rover_autonav_collision_stop.md` | the threshold table |
| `docs/rover_autonav_requirements.md` §R3 | the footprint paragraph |

**Lesson recorded 2026-08-01:** the stale `0.405 m` plate width survived in two shipped files for a
week after being superseded. When a measured dimension changes, grep the tree for the **old value**,
not just the name of the thing.

---

## 7. Sensors that are NOT on the vehicle

| Sensor | Status 2026-08-01 |
|---|---|
| **VL53L1X front ToF** | **SUPERSEDED — this was the pre-depth-camera collision system.** Not mounted. `obstacle_distance` and `rov_collision_stop` remain in the tree but are built-and-idle: no systemd units, no live nodes. |
| **STL-19 360° lidar** | With another team. No coverage behind or beside; never clear a spin from the depth `/scan`. |

**The VL53L1X path is not a gap to be closed — it is the design the depth camera replaced.** A single
0-25° beam at 10 Hz was the best available before the Gemini 336L; the depth path supersedes it on
every axis that matters here:

| | VL53L1X (old) | depth camera (current) |
|---|---|---|
| Field of view | 0-25°, one beam | **91.9° fan** |
| Height awareness | none | **0.12-0.45 m band** |
| Rate | 10 Hz | **~29 Hz** on `/scan_3d` |
| Near coverage | from 0.20 m | from 0.308 m — **inside the bumper**, so no forward blind strip (§4) |

**Consequences of the current design, stated plainly so nobody re-derives them:**
- The reflex lives inside `autonav_mode`'s executor, the single funnel to the motors, so it applies
  regardless of who publishes `/cmd_vel`.
- It runs off the same sensor and the same computer as everything else, so it is **not independent**
  of the companion. That is inherent to the design, not an omission.
- Perception loss **fails safe**: `collision.require_scan = true` blocks forward motion on a stale
  scan, so a dead camera stops the rover rather than blinding it.
- The one thing the old path could still offer is survival of a **companion-computer crash**, since
  `/fmu/in/obstacle_distance` feeds PX4's collision prevention on the FC. Only worth revisiting if
  unattended operation ever demands it — not a prerequisite for anything on the current ladder.

**Cleanup:** `rov_collision_stop` and `obstacle_distance` are dead code paths kept for reference.
Retiring them belongs with todo #17 (`camera_sw_node_obsolute.py`).
