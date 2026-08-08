# Indoor mapping — plan, measured costs, and the next session's steps

**Written 2026-08-02 00:30. Updated 2026-08-02 11:30 with the Step 1–3 results.** Goal: a map of the
house good enough for autonomous indoor navigation. Everything below was measured on this hardware.
**Nothing has been driven yet** — the first driving step is the §6 shakedown run.

Companion docs: [`rover_geometry.md`](rover_geometry.md) (dimensions, camera mount),
[`autonomy_plan.md`](autonomy_plan.md) (the ladder), [`rover_autonav_collision_stop.md`](rover_autonav_collision_stop.md).

---

## 1. The architecture — map offline, localize onboard

```
Pi      drive in MANUAL + `ros2 bag record` (colour, depth, camera_infos, /odom, /tf)
  ↓     copy the bag off
Laptop  RTAB-Map offline — free of realtime pressure, can run slower than realtime
  ↓     copy back a 2D occupancy grid + the RTAB-Map database
Pi      AMCL + Nav2 on the saved map — LOCALIZATION ONLY
```

**Why this shape:** mapping and localizing have very different costs. Building the map — feature
extraction, loop closure, graph optimisation — is the expensive half. **Localizing against a map you
already have is far cheaper**: AMCL on a 184-beam `/scan_3d` is nothing like RTAB-Map. The Pi never
needs to do the expensive half.

Offline processing is also *better*, not merely cheaper: you can use more features, finer resolution
and multiple passes, and re-run with different settings on the same data instead of re-driving the
house.

### Why colour at all
Colour is **not** for seeing obstacles — depth already does that better. It is for **loop closure**:
drive a circuit, come back to the hall, and odometry has drifted so the map holds two offset copies of
it. Loop closure recognises "I have been here before", snaps them together and corrects everything in
between. Appearance is very good at that; geometry alone is weak, because one corner looks like any
other corner. **Without colour the map slowly bends and rooms end up in the wrong place.**

### Why not stream to the laptop live
`image_transport` on this system has **only `raw_pub`** — the `compressedDepth` plugin fails to load
(visible in every `rover-camera` start). Depth would go uncompressed at ~12 MB/s ≈ 100 Mbit/s against a
13 Mbit/s WFB link. **Record locally, transfer afterwards.**

---

## 2. Measured costs

### 2.1 CURRENT — 2026-08-02, colour AND depth both 640×360, `depth_registration:=true`

Baseline with the reconfigured camera: `/scan` **30.5 Hz**, `/scan_3d` **30.0 Hz**, load1 **2.12**,
camera container **48.9%** of a core.

| | baseline | `ros2 bag record` | RTAB-Map `rgbd_odometry` |
|---|---|---|---|
| CPU | camera 48.9% | — | 🔴 **94.6% of a core** |
| load1 (4 cores) | **2.12** | 🔴 **15.18** | 🔴 **10.44** |
| `/scan` | **30.5 Hz** | **23.1 Hz** (−23%) | **19.1 Hz** (−37%) |
| `/scan_3d` | **30.0 Hz** | — | **20.0 Hz** |
| bag growth | — | ✅ **16 MB/s** | — |

🔴 **RTAB-Map still does not fit realtime — the 640×360 hypothesis is FALSIFIED.** Its CPU went *up*
(79.6% → 94.6%), and its own throughput barely moved: **0.25–0.39 s per update (~3 Hz) with
0.78–1.02 s of delay.** **The offline architecture in §1 stands.**
⚠️ **Confounded, state it honestly:** resolution dropped *and* registration turned on in the same
change, so the CPU rise cannot be attributed to one alone — and the old 79.6% was a *cheaper but
incorrect* config, since RGB-D needs registration. What is solid: **in the correct config at
640×360 it still does not fit.** Isolating them needs a 640×360 + registration-OFF run.
✅ Odometry *quality* was healthy throughout — 150–271 inliers, std dev ~0.002 m. It works; it is
just too slow.

✅ **The SD card is no longer the binding constraint.** Growth **29.3 → 16 MB/s**, ~40% under the
measured **27.4 MB/s** ceiling; 27 GB free now buys **≈28 minutes** of driving, not 15.
⇒ **A USB SSD is OPTIONAL, not required. Do not buy hardware for this yet.**

🔴 **WITHDRAWN: "recording does not degrade the safety path."** That was measured against an
already-degraded **13.4 Hz** baseline, where a 14.2 Hz reading looked like an improvement — it was
noise, not headroom. Against a clean **30.5 Hz** baseline, recording costs a real **23% of `/scan`**
and drives load1 to **15.18**, largely I/O wait (SD writes park processes in D-state, which Linux
counts as load). **Recording is far cheaper on disk but NOT free on the safety path** — and §4 means
there is no collision reflex during the run, so drive slowly.

### 2.2 SUPERSEDED — 2026-08-01/02, colour 1280×720, depth 848×480, registration OFF

Kept only so the deltas above are checkable. **Do not quote these numbers.**

| | baseline | `ros2 bag record` | RTAB-Map `rgbd_odometry` |
|---|---|---|---|
| CPU | — | 42.6% of a core | 79.6% of a core |
| load1 | 2.25 | 5.64 | 9.44 |
| `/scan` | 13.4 Hz | 14.2 Hz | 7.0 Hz |
| `/scan_3d` | 23.2 Hz | 25.3 Hz | 17.3 Hz |
| bag growth | — | 29.3 MB/s (over the ceiling) | — |

⚠️ **METHOD LESSON — both of these bag conclusions were distorted by the same fault:** the baseline
itself was degraded, which made the disk look fatal and the `/scan` cost look free. **Never conclude
"no cost" from a delta taken against a degraded baseline. Fix the baseline, then measure.**

---

## 3. Sensor rationale — settle this once

**92° is not a weakness of the 336L; it is what the whole RGB-D class is.** The D455 is ≈87°×58°;
the 336L measured **91.9°×60.7°** depth here, so it is *wider*. The D455's advantages are a longer
baseline (better accuracy at range) and a global shutter — not FOV.

**The 336L is the better fit for this vehicle.** Its measured minimum range is **0.308 m**; the D455's
longer baseline pushes its minimum to roughly half a metre. With the bumper at 0.345 m, a ~0.5 m
minimum would put the blind circle *in front of the bumper*. As built, the blind circle sits **37 mm
inside** the footprint, so there is no blind strip ahead at all.

**Colour is slightly wider than depth — 92.9°×61.2° vs 91.9°×60.7°** — so registering depth into the
colour frame **loses no field of view**. Verified from the live `camera_info`.

⚠️ **The lidar comparison is a different sensor class, not a competing product.** The STL-19 has been
assigned to the drone; the rover is camera-only. Its permanent consequences (no rear/side coverage,
never clear a spin from a scan, ~3 m usable range) are in `autonomy_plan.md`.

---

## 4. Safety for the mapping run — read before driving

**Drive in MANUAL.** Verified in the real firmware (`pxlabs-fw`,
`DifferentialManualMode.cpp`): `manual()` publishes a **steering setpoint directly**, while only
`acro()` publishes a **rate setpoint**. The ~21× yaw-rate runaway (#20) lives in the rate controller,
which Manual never invokes. **So a mapping run is NOT blocked on #20.**

> 🔴 **BUT THE COLLISION REFLEX DOES NOT APPLY IN MANUAL.** It lives inside `autonav_mode`'s executor,
> a *custom* PX4 mode; in Manual, PX4 never routes through it. **There is no automatic stop. The
> operator is the only safety layer.** This is the opposite of how the rover behaves in AutoNav.

**#20 still gates every autonomous mode** — Acro, Stabilized, Position, Offboard and Auto all use the
rate controller.

---

## 5. Steps 1–3 — DONE 2026-08-02

| Step | Outcome |
|---|---|
| **1. Camera reconfig** | ✅ Applied. Drop-in adds `depth_registration:=true color_width:=640 color_height:=360`, keeps `point_cloud_decimation_filter_factor:=3`. Half-dead check passed first try. `/scan` 28.0, `/scan_3d` 29.7 Hz. |
| **2. Re-measure RTAB-Map** | 🔴 **Hypothesis falsified — it still does not fit.** 94.6% of a core, ~1 s delay unchanged. §2.1. |
| **3. Re-measure bag** | ✅ **16 MB/s, ~28 min on 27 GB — SSD no longer required.** But recording costs 23% of `/scan`. §2.1. |

Config detail worth keeping: **640×360 preserves 16:9 and therefore the FOV; 640×480 would not.**
🔑 **`depth_registration:=true` forces depth to the COLOUR resolution** — depth is now 640×360, no
longer its native 848×480, and carries the colour intrinsics. Fewer cloud columns (~213 not 283
after decimation 3, ~0.42°/col ≈ 2.2 cm at 3 m — still well inside a 5 cm costmap cell).
Revert: `/etc/systemd/system/10-point-cloud-decimation.conf.bak-20260802`.

```bash
# 🔴 MANDATORY after ANY camera restart — it can come up "active" with depth+colour DEAD, no error:
journalctl -u rover-camera --since -1min | grep "depth Frame - Width"
# absent => restart again. Also confirm a real topic rate; systemctl is-active does NOT catch this.
```
⚠️ `Odom/Strategy` and `Vis/MaxFeatures` are **string** params — `-p Odom/Strategy:=0` throws
`InvalidParameterTypeException` and quoting does not help. Use a params file or omit them.
⚠️ `pgrep -f rgbd_odometry` **matches its own command line** and will falsely report the node alive.
⚠️ Re-check for contaminating processes **mid-measurement**, not only at the start.

---

## 6. Next — do these in order

**Decision 2026-08-02: try it ON THE ROVER first**, using the bag route that Step 3 just validated.
The alternative architecture in §7 stays open as the documented fallback.

**Why rover-first:** it is ready now and needs zero new setup, whereas the laptop route needs ROS2
Jazzy + the OrbbecSDK wrapper + `rtabmap_ros` + DDS over WFB before it teaches anything. And the
genuinely open questions — does `/odom` survive turns, does loop closure fire in these rooms, how
slow must we drive — are about **the house and the rover, not about where RTAB-Map runs.** A bag
answers all of them regardless of where it is processed. It is also **re-runnable**: tune RTAB-Map
and re-process the same data instead of re-driving.

### Step 4 — SHAKEDOWN RUN (do this before the full house)
**3–5 minutes, one room, one deliberate loop** — drive out, around, and back over your own path.
Exercises the whole chain end to end at low cost and tells you whether a 28-minute run is worth it.
- **Manual mode. Slow.** Motion blur destroys feature matching; fast turns are where matching fails.
- 🔴 **There is NO collision reflex in Manual — the operator is the only safety layer.** §4.
- Record: `/camera/color/image_raw`, `/camera/color/camera_info`, `/camera/depth/image_raw`,
  `/camera/depth/camera_info`, `/odom`, `/tf`, `/tf_static`.
- Record to the SD — Step 3 proved it fits. Watch `/scan` (expect ~23 Hz) and disk free.

### Step 5 — offline on the laptop
RTAB-Map over the shakedown bag. **Judge: did loop closure fire, and is `/odom` sane through the
turn?** If yes, the architecture is proven and the full run is justified. If loop closure never
fires, fix that before driving the house — a longer run will not fix a bent map.

### Step 6 — the full house run
Only after Step 5 passes. Cover room by room, revisit doorways, **make deliberate LOOPS.** ~28 min
of capacity; keep an eye on disk.

### Step 7 — map back to the Pi
Export a 2D occupancy grid; copy the grid and database back; run AMCL + Nav2 against it on the Pi —
**localization only**, which is the cheap half (§1).

---

## 7. Alternative architecture — camera cabled to a laptop (open, not chosen)

Proposed 2026-08-02. **Plug the Gemini 336L into a laptop that rides on/beside the rover, run
RTAB-Map live there, copy the map back.** Kept because §2.1 makes it the likely answer if the
shakedown shows RTAB-Map needs live tuning.

**Wins:** no bag, no SD limit, no 28-min ceiling, no 23% `/scan` cost, no 94.6% CPU fight — and
**live feedback**, so you see whether loop closure fires instead of finding out afterwards.
**Marginal cost is low:** the laptop needs `rtabmap_ros` for offline processing *anyway*; the only
new requirement is the Orbbec wrapper plus a USB3 port.

🔴 **The camera MUST stay bolted to the rover — re-route the cable, do not hand-carry it.** The
mount geometry is load-bearing (`cam_x 0.00`, `cam_z 0.305`, pitch 2.33° nose-down, as-built in
`rover_geometry.md`). Hand-carrying builds a map from a trajectory the rover cannot reproduce.

**The design question: where does `/odom` come from?**
- **Preferred — laptop joins the ROS2 domain over WFB and subscribes to `/odom`.** This works
  *because* the images no longer cross the network: `/odom` is tiny, and it was uncompressed depth
  at ~100 Mbit/s that made live streaming impossible (§1). USB now carries the images.
- Visual-only odometry — simpler, no network, but fragile on blank walls and motion blur.

**Costs to accept:** the rover loses `/scan` and `/scan_3d` for the whole run (the Orbbec is the
only depth sensor, and `rover-camera.service` will retry-loop unless stopped); enclosure access to
the USB is unverified and already blocked the port `4-1` test; USB3 passive cable is ~3 m, so the
laptop rides on or immediately beside the rover.

---

## 8. Open questions

**Answered 2026-08-02:**
- ~~Does 640×360 make RTAB-Map fit?~~ **No — falsified, it got worse. §2.1.**
- ~~How much does SW depth registration cost?~~ **Camera container 34.2% → 48.9% of a core, i.e.
  more than the resolution drop saved.** ⚠️ the baseline was a single `ps` snapshot — directional.
- ~~Is the ~1 s delay CPU starvation or intrinsic?~~ **Provisionally INTRINSIC — a 4× cut in pixel
  count moved it essentially not at all** (1.02 → 0.96 s worst). Fine for mapping, unusable for control.

**Still open:**
- **Is `/odom` good enough through turns?** Validated on straight-line speed only; gyro-yaw (#21) is
  open. RTAB-Map can run on *visual* odometry instead, which sidesteps it — worth trying both.
- **What is the drift over a full circuit, and does loop closure actually fire?** The real test of
  whether any of this works. **Step 5 is the first chance to answer it.**
- **How slow is slow enough** to keep motion blur from breaking feature matching?
- **Is 92° enough in these room sizes?** ~3 m usable range costs map quality in large rooms (§3).

---

## 9. RESULT — 2026-08-08. The map is built, and the last defect was ray tracing

`house_map_v2.pgm` / `.yaml` (+ `house_map_v2.db`) is the first map on this vehicle that reads as
a floor plan. It comes from the wheel-odometry replay of the 08-07 bag, plus one parameter.

**The defect: `Grid/RayTracing` was `false`.** With it off, RTAB-Map writes free cells only where
it segments *ground* points. On this rover that is almost nothing — the camera sits at 0.305 m and
sees very little floor, and the numbers show it: over 480 nodes the stored cells hold **17 210
ground points against 550 146 obstacle points, a 32:1 ratio**. So the grid became "every surface
above `Grid/MaxGroundHeight` = 0.10 m", with no carved interiors:

| | RayTracing false | **RayTracing true** |
|---|---|---|
| occupied | 34.2 % (19 300 cells) | **12.5 %** |
| free | 3.9 % (2 190 cells) | **34.1 %** |
| unknown | 61.9 % | 53.4 % |

⚠️ **Ray tracing is not optional on a low-mounted RGB-D rover.** The usual reason to leave it off —
"the sensor already reports free space" — assumes a lidar sweeping the floor plane. It does not
hold here.

⚠️ **`Grid/DepthRoiRatios: "0.0 0.0 0.0 0.35"` was added for correctness and changes the map by
0.1 %** (occupied 12.4 → 12.5 %). It was the one hole in the plate-masking family
(`Kp/RoiRatios` ✅ · `Vis/RoiRatios` ✅ · `Grid/DepthRoiRatios` ❌ · `*/DepthAsMask` ✅). Do not
expect it to fix anything.

### 🔴 Reading the exported PGM — this cost a session

`rtabmap-reprocess -g2` writes RTAB-Map's own greyscale convention, **not** Nav2's:

| | RTAB-Map export | Nav2 `map_server` |
|---|---|---|
| unknown | **89** | 205 |
| free | **178** | 254 |
| occupied | **0** | 0 |

Only `occupied` agrees. `0` is the *largest* class in a bad map and the *smallest* in a good one,
so reading `0` as free inverts the diagnosis exactly — that is how the 08-07 session concluded
"walls are not marked occupied" when the truth was "almost everything is marked occupied".
**Disambiguate by mechanism, not by eye:** enabling `Grid/RayTracing` can only ever *add* empty
cells, so whichever value grows is free.

The export also has **y increasing with row index**, the opposite of the PGM/Nav2 convention, and
carries **no origin**. Both are recovered and validated in §9.1.

### 9.1 Turning the export into a Nav2 map

1. Flip vertically (`nav[::-1]`) and remap 178→254, 89→205, 0→0.
2. Recover the origin by rasterising the stored `obstacle_cells` at the optimized poses
   (`rtabmap-export --poses --opt 2`) and correlating that against the export's occupied mask.
   Result: **origin `[-10.554, -4.843, 0.0]`**, 0.05 m cells, 257 × 221 — 5846 of 7103 occupied
   cells overlap, so the offset is unambiguous.
3. **Validate on the trajectory, always.** Every optimized pose must land on a free cell:
   **285 of 292 free, 7 occupied, 0 unknown** — the 7 are poses hard against furniture at 5 cm
   resolution. This single check confirms the origin, the flip, and the colour convention at once.

### 9.2 Wall geometry — MEASURED, and better than it looks

⚠️ An earlier draft of this section said "walls 0.3–0.5 m thick and doubled". **That was eyeballed
off the rendered image and is wrong.** Measured two independent ways over the occupied cells:

| | distance-transform ridge (2 × dist to nearest non-occupied) | run-length |
|---|---|---|
| median | **0.10 m** | 0.10 m horiz / 0.15 m vert |
| p90 | 0.22 m | 0.45 / 0.50 m |
| max | 0.71 m | 1.85 / 2.45 m |

**The typical wall is 2 cells — 0.10 m, which is about the floor for a 0.05 m grid.** Only the top
decile is thicker, and the maxima are furniture and clutter, not doubled walls. The visually
prominent black blobs are what made the eyeball estimate wrong.

⇒ **Map geometry is NOT the bottleneck.** Do not spend effort on the pose graph
(`RGBD/OptimizeMaxError`, closure count) on the strength of how the picture looks — measure the
thickness distribution first and only act on the tail if a planner actually complains.

### 9.3 Verified end-to-end in `map_server`

Not just geometrically validated — actually loaded. `nav2_map_server` on `ROS_DOMAIN_ID=42`
configures, activates, and publishes `/map`: **257 × 221 @ 0.05 m, origin (−10.554, −4.843)**,
**unknown 53.4% (30 349) / free 34.1% (19 345) / occupied 12.5% (7103)**. All 292 optimized poses
fall in bounds — **285 free, 7 occupied, 0 unknown** — on the published `OccupancyGrid`, which
confirms the vertical flip independently of the PGM-side check in §9.1.

Parameters now live in `src/rover_nav2/config/rtabmap_mapping.yaml` — promoted out of session
scratch so this is reproducible.

### 9.4 🔴 RTAB-Map localization aborts on a one-frame depth glitch (2026-08-08)

Localization measured on the Pi against `house_map_v2.db`, live camera, `vision_streaming` active:
**~27.5% of ONE core steady-state** (46% startup transient), 6.7% RAM, **RTAB-Map 71–87 ms per cycle
against a 0.5 s budget (~16% duty)**, delay 0.19 s, 291 nodes in working memory,
`/localization_pose` 1.8 Hz, **`map->odom` published** — the AMCL replacement works, with no
measurable `/scan` cost. Against mapping's `rgbd_odometry` at 79.6% with `/scan` halved, it fits
with room to spare. ⚠️ **Rover was stationary — this is a floor, not a ceiling.**

**But it died after ~13 minutes**, FATAL in `Memory.cpp:4579::createSignature()`:
`image=(640/360)` vs `depth=(1280/800)`. RTAB-Map requires depth ≤ colour, and the assertion
**aborts the process** (`terminate called after throwing UException`) — not a dropped frame.

**Confirmed by monitoring `/camera/depth/image_raw` for 15 min:** at t=341.5 s the topic emitted
**one 1280×800 frame** and was back to 640×360 0.2 s later. `/camera/depth/image_unaligned` runs at
exactly 1280×800, so the glitch is the *unaligned* stream briefly surfacing on the aligned topic.

⚠️ **Correction:** an earlier draft said the unit passes no `depth_registration` argument. **Wrong —
I read only the base unit.** The drop-in `10-point-cloud-decimation.conf` sets
`depth_registration:=true color_width:=640 color_height:=360 color_fps:=15 depth_fps:=15`.
Registration IS on, which is exactly why a *stray unaligned* frame is so surprising.

⇒ **This gates any mapped navigation.** One bad frame in ~6 minutes kills localization outright, so
a supervisor/restart is a bandage, not a fix. Options, in order of preference: force
`depth_registration` explicitly in the unit; drop mismatched frames in a relay ahead of RTAB-Map;
or pin the depth profile so the native stream cannot exceed the colour frame.

### 9.5 🔴 THE CAMERA IS STILL IN MAPPING CONFIGURATION

The same drop-in carries its own warning: **"REVERT color_fps/depth_fps BEFORE ANY AUTONOMOUS
DRIVING. depth_fps:=15 halves /scan (~30 -> ~15 Hz), which widens the collision-reflex reaction
gap."** That revert has not happened — `depth_fps:=15` is live right now.

**This explains a measurement I got wrong today.** I read `/scan` at 15.7 Hz and `/scan_3d` at
14.1 Hz against a remembered 23.5 / 29.2, and attributed the shortfall to load from my own session.
It is not load — **it is `depth_fps:=15`, by design, left over from the mapping run.**

⇒ **Revert the fps lines before any AutoNav driving**, including the guarded-manual case: the reflex
is live in AutoNav and is running at half its intended update rate. Restore from
`/etc/systemd/system/10-point-cloud-decimation.conf.bak-20260802`, keeping the decimation and
registration lines, then `systemctl daemon-reload && systemctl restart rover-camera` — and run the
half-dead camera check afterwards.
