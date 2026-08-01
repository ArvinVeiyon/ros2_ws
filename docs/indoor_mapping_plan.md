# Indoor mapping — plan, measured costs, and the next session's steps

**Written 2026-08-02 00:30.** Goal: a map of the house good enough for autonomous indoor
navigation. Everything below was measured on this hardware. Nothing has been driven yet.

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

## 2. Measured costs — 2026-08-01/02, rover stationary

| | baseline | `ros2 bag record` | RTAB-Map `rgbd_odometry` |
|---|---|---|---|
| CPU | — | **42.6% of a core** | **79.6% of a core** |
| load1 (4 cores) | **2.25** | 5.64 | **9.44** |
| `/scan` | 13.4 Hz | **14.2 Hz** ✅ | **7.0 Hz** 🔴 halved |
| `/scan_3d` cloud | 23.2 Hz | **25.3 Hz** ✅ | 17.3 Hz |

✅ **Recording does not degrade the safety path.** 🔴 **RTAB-Map realtime does** — and its own
throughput was 0.29–0.43 s per update (~3 Hz) with **0.83–1.04 s of delay**. That was *only* the
odometry node, without the map/loop-closure node and without depth registration, so the true realtime
cost is higher. **This is why mapping is offline.**

### The binding constraint is the SD card, not CPU
- Bag growth **29.3 MB/s** against a measured **27.4 MB/s** sustained-write ceiling — it writes *at*
  the card's limit, so any hiccup drops messages.
- 27 GB free ⇒ **≈15 minutes of driving.**
- ⇒ **A USB SSD is effectively required for a real run.** None is attached.
- Dropping colour to 640×360 shrinks the bag *and* RTAB-Map's cost — one change, both problems.

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

## 5. Next session — do these in order

### Step 1 — reconfigure the camera (one restart)
Add to `/etc/systemd/system/rover-camera.service.d/10-point-cloud-decimation.conf`:
```
depth_registration:=true color_width:=640 color_height:=360
```
Keep `point_cloud_decimation_filter_factor:=3`. **640×360 preserves 16:9 and therefore the FOV;
640×480 would not.** Then:
```bash
sudo systemctl daemon-reload && sudo systemctl restart rover-camera
# 🔴 MANDATORY half-dead check — a restart can come up "active" with depth+colour DEAD, no error:
journalctl -u rover-camera --since -1min | grep "depth Frame - Width"
# absent => restart again. Also confirm a real topic rate; systemctl is-active does NOT catch this.
```

### Step 2 — re-measure RTAB-Map at 640×360
```bash
ros2 run rtabmap_odom rgbd_odometry --ros-args \
  -r rgb/image:=/camera/color/image_raw -r depth/image:=/camera/depth/image_raw \
  -r rgb/camera_info:=/camera/color/camera_info -r odom:=/rtab_odom \
  -p frame_id:=base_link -p approx_sync:=true
```
Watch `update time=` and `delay=` in its log, and CPU. **If it drops from 79.6% to ~20%, realtime may
be back on the table** and the offline step becomes optional rather than necessary.
⚠️ `Odom/Strategy` and `Vis/MaxFeatures` are **string** params — `-p Odom/Strategy:=0` throws
`InvalidParameterTypeException` and quoting does not help. Use a params file or omit them.

### Step 3 — re-measure bag growth at 640×360
Expect well under the 27.4 MB/s ceiling. **If it is, the SSD stops being a hard requirement.**

### Step 4 — the mapping run
- **USB SSD if the bag is still near the ceiling.** Record to it, not the SD.
- **Manual mode. Slow.** Motion blur destroys feature matching, and fast turns are where scan matching
  fails. There is no collision reflex — §4.
- **Make deliberate LOOPS.** Return to places already driven. That is exactly what loop closure needs
  and what stops the map bending. Cover room by room, revisit doorways.
- Record: `/camera/color/image_raw`, `/camera/color/camera_info`, `/camera/depth/image_raw`,
  `/camera/depth/camera_info`, `/odom`, `/tf`, `/tf_static`.

### Step 5 — offline on the laptop, then back
RTAB-Map over the bag; export a 2D occupancy grid; copy the grid and database back; run AMCL + Nav2
against it on the Pi.

---

## 6. Open questions

- **Does 640×360 actually make RTAB-Map fit?** The whole plan's cost hinges on this and it is untested.
- **Is the ~1 s odometry delay caused by CPU starvation, or intrinsic?** If it survives a CPU fix, 1 s
  is fine for mapping (drive slowly) but unusable for control.
- **How much does SW depth registration cost?** `align_mode` is `SW`; it has never been measured.
- **Is `/odom` good enough through turns?** It is validated on straight-line speed only, and the
  gyro-yaw odometry item (#21) is open. RTAB-Map can run on *visual* odometry instead, which sidesteps
  it — worth trying both.
- **What is the drift over a full house circuit, and does loop closure actually fire?** The real test
  of whether any of this works.
