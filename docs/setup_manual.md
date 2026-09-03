# Vind-Roz — Setup & Operations Manual

> **Rev. 2026-08-09.** How to bring this rover up from nothing, what every running node is, and how
> to verify each stage. Procedural. For *why* the numbers are what they are, see
> [`autonav_reference.md`](autonav_reference.md).
>
> **Reading the status marks:**
> ✅ verified on this vehicle · ⚠️ recorded but not re-verified recently · 📋 standard procedure,
> **not yet performed on this vehicle** — verify before relying on it.
>
> **This manual is expected to grow.** Part F is a stub list of the sections still to be written
> (map creation, QGC map integration, mission execution).

| Part | Contents |
|---|---|
| [A](#part-a--flight-controller-px4) | Flight controller (PX4) — firmware, airframe, calibration, **parameter changelog (A7)** |
| [B](#part-b--companion-computer) | Companion computer — OS, workspace, build |
| [C](#part-c--node-inventory) | Node inventory — every service, its job, parameters and topics · **PX4 external modes (C10)** |
| [D](#part-d--bring-up-and-verification) | Bring-up and verification |
| [E](#part-e--routine-operations) | Routine operations |
| [F](#part-f--not-yet-written) | Not yet written |

---

# Part A — Flight controller (PX4)

## A1. Firmware

| Item | Value |
|---|---|
| Board | Custom Pixhawk 6X-RT (in-house PCB, **not** Holybro) |
| Target | `px4_fmu-v6xrt` |
| Version | `pxlabs-v1.17.0-2.0.0`, commit `a52c38b07d` |
| Source | `~/PX4-Autopilot`, remote `pxlabs`, branch `pxlabs-fw` ✅ **this is the real firmware source** |

```bash
cd ~/PX4-Autopilot
git fetch pxlabs && git checkout pxlabs-fw
make px4_fmu-v6xrt_default          # build
make px4_fmu-v6xrt_default upload   # flash over USB
```

⚠️ The FC is shared with an aerial drone on a different airframe. **Any parameter changed for the
rover must be reverted before flight.** Changes that fall into this category are marked 🚁 below.

## A2. Airframe

Rover, differential (skid-steer). Sets `RO_*` parameters and the differential rover module.

## A3. Sensor calibration

Do these in QGroundControl → Vehicle Setup → Sensors, **with the rover on level ground and
disarmed.** Order matters: gyro first, then accel, then level, then mag.

| Step | Procedure | Status |
|---|---|---|
| **Gyro** | Leave completely still, ~10 s | 📋 |
| **Accelerometer** | Six orientations as prompted | 📋 |
| **Level horizon** | Rover level, on its wheels | 📋 |
| **Magnetometer** | Rotate through all axes | 📋 ⚠️ **see A4** |
| **RC** | Full-range sweep of every stick and switch | 📋 |

> 🔴 **The magnetometer on this vehicle is suspect.** It is a single internal BMM350 inside a metal
> body, and it is the only sensor that observes yaw indoors. Measured against a wall, the fused
> EKF yaw invented **+23.23° in 21 seconds** on a stationary rover. Calibrating it may not fix that,
> and the rover does **not** depend on it — heading comes from the camera gyro
> (`autonav_reference.md` §6). Calibrate it for the drone's sake, not the rover's.

### Verifying a calibration

```bash
python3 ~/ros2_ws/tools/set_param.py CAL_GYRO0_ZOFF      # read one
vcgencmd get_throttled                                    # 0x0 = clean power
```

`tools/set_param.py NAME [value]` reads and writes any PX4 parameter over MAVLink (`tcp:5760`) and
**refuses while armed** ✅. Parameters are not exposed over DDS at all, so this is the only route
from the companion.

## A4. Rover control parameters

✅ Read off the FC 2026-08-08 and re-verified. **Re-read after every FC reboot.**

```
RO_YAW_RATE_P     0.08        RO_MAX_THR_SPEED   0.6    m/s
RO_YAW_RATE_I     0.0         RO_YAW_P           2.0
RO_YAW_RATE_CORR  1.8         RO_SPEED_LIM       0.70   m/s
RO_YAW_RATE_LIM   85.9        ⚠ deg/s, NOT rad/s
```

> 🔴 **`RO_YAW_RATE_I = 0` is deliberate** — integral windup was half the cause of the yaw problem.
> Never restore 0.1.
> 🔴 **`RO_YAW_RATE_LIM` is in deg/s.** It was once set to 0.5 believing rad/s, which is 6× below the
> measurement deadband, and every yaw command in Acro came out exactly zero.

## A5. Estimator and interface parameters

| Parameter | Value | Meaning |
|---|---|---|
| `EKF2_MAG_TYPE` | 1 | magnetometer in use 🚁 |
| `EKF2_GPS_CTRL` | 7 | all GPS aiding bits on 🚁 |
| `EKF2_EV_CTRL` | 4 | external vision: **velocity only**. `9` = pos+yaw is the target for VIO |
| `EKF2_IMU_CTRL` | 7 | |

## A6. Serial ports

| Port | Use | Baud |
|---|---|---|
| `ttyAMA0` | MAVLink → mavlink-router | 921600 |
| `ttyAMA1` | free | — |
| `ttyAMA2` | TFmini rangefinder | 115200 |
| `ttyAMA3` | STL-19 lidar (not fitted) | 230400 |
| `ttyAMA4` | **uXRCE-DDS agent** | 921600 |

## A7. Parameter changelog — what was changed, and the test that justified it

**Every deliberate FC parameter change on this vehicle, with its evidence.** Append here when you
change one; a value without a reason becomes undeletable folklore.

### Rover control (`RO_*`) — airframe-specific, does **not** affect the drone

| Date | Parameter | Was → Now | Test / evidence |
|---|---|---|---|
| 08-02 | `RO_MAX_THR_SPEED` | 3.0 → **0.6** | Firmware doc: *"speed at maximum throttle"*, m/s. Drivetrain measured at 0.58–0.60 m/s. It divides the feedforward in **both** the speed and yaw-rate loops, so FF was **5× too small**. |
| 08-02 | `RO_YAW_RATE_LIM` | 0.5 → 28.6 → **85.9** | 🔴 Unit is **deg/s, not rad/s**. 0.5 meant 0.0087 rad/s — 6× *below* the 3 deg/s measurement deadband `RO_YAW_RATE_TH` — so in Acro every yaw command was zeroed. **Measured: 1820 armed Acro samples, flat 0.0000 output.** Final 85.9 = 1.50 rad/s, matched to the usable band. |
| 08-02 | `RO_YAW_RATE_CORR` | 1.0 → 3.0 → **1.8** | Firmware doc recommends >1 for skid-steer friction. Final 1.8 set for the ~1.2 rad/s design point, the middle of the usable band. |
| 08-02 | `RO_YAW_RATE_P` | → **0.08** | Highest gain that does not hunt across the friction deadband. |
| 08-02 | `RO_YAW_RATE_I` | → **0.0** | 🔴 **The windup source.** One of the two causes of #20 (with the friction deadband). **Never restore 0.1.** |
| **08-14** | `RO_DECEL_LIM` | −1 (off) → 0.5 → 🔴 **REVERTED to −1** | 🔴🔴 **THIS CHANGE CAUSED A HARD WALL HIT IN MANUAL DRIVE, SAME DAY.** Set on the belief it was auto-mode-only ("waypoint slowdown"). **It is not** — see the box below. ⛔ **DO NOT RE-APPLY without reading it.** |
| **08-14** | `RO_ACCEL_LIM` | −1 (off) → 0.3 → 🔴 **REVERTED to −1** | Same fault, same revert. The 0.3 soft start also ramps the **manual stick**, 0→full over 2.0 s, which invites over-sticking. |
| **08-14** | `RO_SPEED_LIM` | 0.70 → 0.60 → **REVERTED to 0.70** | Reverted at operator instruction with the other two ("change all to old value"). ⚠️ **This one was NOT implicated** — it only acts in POSCTL (`DifferentialManualMode::position()`), not Manual. Reverting restores the real P3 defect: setpoints in **0.60–0.70 saturate full throttle**. → `px4_param_audit.md` P3 |

> 🔴🔴 **THE 08-14 SLEW-LIMIT INCIDENT — READ BEFORE TOUCHING `RO_ACCEL_LIM` / `RO_DECEL_LIM`**
>
> **`RO_ACCEL_LIM` and `RO_DECEL_LIM` ACT IN MANUAL MODE. They are NOT auto-only.** I set them
> believing the firmware doc's "approaching waypoints in auto modes" described their whole scope,
> and the rover hit a wall hard under RC on the operator's next drive.
>
> **The path, verified in source** (`~/PX4-Autopilot`, `a52c38b07d`):
> `control_mode.cpp:55` — `NAVIGATION_STATE_MANUAL` sets `flag_control_allocation_enabled = true`
> → `RoverDifferential.cpp:154` — so `DifferentialActControl::updateActControl()` **runs in Manual**
> → `DifferentialActControl.cpp:75` — which slew-limits the **raw stick throttle** through
> `RoverControl::throttleControl(..., RO_ACCEL_LIM, RO_DECEL_LIM, RO_MAX_THR_SPEED, dt)`
> → `RoverControl.cpp:59` — decel slew = **`RO_DECEL_LIM / RO_MAX_THR_SPEED`** = 0.5/0.60 = 0.833 /s.
>
> 🔑 **THE EFFECT: RELEASING THE STICK NO LONGER STOPS THE ROVER.** With both at −1, line 70
> (`setForcedValue`) passes the stick **straight to the motors** — centre the stick and the motors
> cut instantly. With them set, the throttle **ramps down over up to 1.2 s while still driving**.
> At the ~0.9 m/s this rover actually reaches that is ~0.5 m of *powered* travel after "stop",
> **on top of** the ~0.30 m mechanical coast — roughly **3× the manual stopping distance**, with no
> change in how the stick feels and **no reflex protection, because the reflex gates AutoNav
> setpoints, not the operator.**
>
> 🔑 **THE METHOD ERROR, which is the transferable part:** I reasoned about the parameters from
> their *documented purpose* instead of tracing **which controllers each `nav_state` enables**. A
> rover param is not scoped by its description — it is scoped by the `flag_control_*_enabled` bits.
> ⛔ **Before changing any `RO_*`, grep `control_mode.cpp` for every mode that enables the controller
> that reads it, and check `DifferentialActControl` explicitly — allocation is enabled in EVERY
> manual mode.** Relatedly: "not yet verified on the vehicle" is **not** the same as "not yet
> active". These were live on the operator's next stick input.
>
> ⚠️ **`tools/set_param.py` writes RAM ONLY.** The revert above is RAM until `param save` runs —
> **and an unsaved revert means a reboot silently restores the values that caused the crash.**
> **Always finish with `param save`, then verify AFTER a reboot you have proven happened.**
| 08-02 | `RO_YAW_P` | → **2.0** | Applied with the final tune. |
| 08-02 | `RO_SPEED_LIM` | → **0.70** | m/s. |

⚠️ `RO_YAW_RATE_LIM` is **not referenced by `DifferentialRateControl`** at all — only by
`DifferentialManualMode` and the ackermann modules. It never constrained the AutoNav path where the
runaway occurred. An older note claiming otherwise assumed rad/s *and* assumed it applied; neither
was true. Full derivation: `rover_yaw_response.md` §3 and §5.

### Estimator (`EKF2_*`) — 🚁 **shared with the drone**

| Parameter | Value | Status |
|---|---|---|
| `EKF2_MAG_TYPE` | 1 | **as-found, not changed by us.** Magnetometer in use — a suspect for the yaw fault (§A3) |
| `EKF2_GPS_CTRL` | 7 | **as-found.** All GPS aiding bits on; the other yaw-fault suspect |
| `EKF2_EV_CTRL` | 4 | **as-found.** External vision = velocity only. `9` (pos+yaw) is the target for the VIO route |
| `EKF2_IMU_CTRL` | 7 | **as-found** |

> ⚠️ **None of the `EKF2_*` values above were changed by us** — they are what the FC reports. Do not
> read this table as a change history for them.
>
> 🚁 **When you do change one, it affects the drone.** `EKF2_*` and sensor calibration are shared;
> `RO_*` are rover-airframe-only. Any `EKF2_MAG_TYPE=5` or `EKF2_GPS_CTRL=0` test for the yaw fault
> **must be reverted before flight** — record the change here with its revert state.

### ⚠️ Gaps in this record

- **Pre-2026-08-01 history is unknown.** Parameters changed before the yaw investigation have no
  recorded justification.
- **Sensor calibration has no record at all** on this airframe (§A3 is marked 📋 for that reason).
- Values must be **re-read after every FC reboot** — they have not always survived one.

```bash
python3 ~/ros2_ws/tools/set_param.py RO_YAW_RATE_P          # read
python3 ~/ros2_ws/tools/set_param.py RO_YAW_RATE_P 0.08     # write (refuses while armed)
```

## A8. DDS topic bridge

Topics crossing FC ↔ ROS 2 are declared in
`~/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml`. Adding one requires a **firmware
rebuild and flash**.

⏭ `vehicle_angular_velocity` is present but **commented out at line 60**. Enabling it would give
three fused IMUs at 397.9 Hz and decouple odometry from the camera — see `autonav_reference.md` §6.

---

# Part B — Companion computer

## B1. Baseline

| Item | Value |
|---|---|
| Host | `Vind-Roz`, Raspberry Pi 5, 8 GB |
| OS | Ubuntu 24.04.1 aarch64, kernel 6.8.0-1048-raspi |
| ROS | ROS 2 Jazzy, Python 3.12.3 |
| Workspace | `~/ros2_ws`, branch `main` |

> ⚠️ **The boot clock is wrong until NTP steps it.** `uptime -s`, `who -b` and unit timestamps
> disagree across a boot — do not correlate journals across one.
> ⚠️ **sudo needs a password:** `printf '<pw>\n' | sudo -S <cmd>`

## B2. Dependencies

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-rtabmap-ros ros-jazzy-nav2-bringup \
                 ros-jazzy-depthimage-to-laserscan ros-jazzy-pointcloud-to-laserscan \
                 ros-jazzy-tf2-ros python3-pymavlink
```

| Component | Version | Notes |
|---|---|---|
| RTAB-Map | 0.22 | |
| MicroXRCEAgent | v3.0.0-2 | |
| mavlink-router | c20337b | |
| px4-ros2-interface-lib | release/1.17 | |

### Third-party sources under `src/` that are **gitignored**

Vendoring these would add hundreds of megabytes and bury our own history, so they are cloned rather
than committed. **Recorded here so a workspace can be reproduced exactly** — promote to real git
submodules when convenient.

| Package | Path | Source | Pin |
|---|---|---|---|
| `OrbbecSDK_ROS2` | `src/OrbbecSDK_ROS2/` | `github.com/orbbec/OrbbecSDK_ROS2.git` | commit `ec6bc228b79656449bea289f2967a2f44ce52c57`, SDK 2.9.3, ~201 MB |
| `ldlidar_stl_ros2` | `src/ldlidar_stl_ros2/` | nested git repo, untracked | **pending re-integration** — on return the lidar owns `/scan` and `depth_to_scan` remaps to `/scan_depth` |

> 🔴 **LOCAL PATCH — NOT UPSTREAM. RE-APPLY AFTER ANY RE-CLONE.**
> `~/codex-work/orbbec_unaligned_depth_guard_20260808.patch` (todo #26). **Because the clone is
> gitignored, a fresh checkout silently loses this and the RTAB-Map abort returns.**
>
> The launch default is `align_mode:=SW`, so depth is aligned to colour in software. When a frameset
> arrives without a usable colour frame the wrapper skips alignment — then publishes the still-native
> **1280×800** depth frame on `/camera/depth/image_raw` anyway, where everything downstream assumes
> the aligned 640×360. RTAB-Map asserts depth ≤ colour in `Memory.cpp::createSignature()` and
> **aborts the process**, killing localization outright. Upstream guards `logFrameInfoOnce()` against
> exactly this case — but only the *logging*, not the *publishing*.
>
> The patch adds `isDepthAlignedToTarget()` and drops both the depth image and the point cloud for
> such a frameset, with a throttled WARN so the drop is never silent. `/camera/depth/image_unaligned`
> still carries the native frame, which is what that topic is for. No effect when `depth_registration`
> is off. **Measured: 1 h 23 m with 0 aborts, against ~13 min before.**

## B3. Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Single package after an edit:

```bash
colcon build --packages-select rover_odometry --symlink-install
sudo systemctl restart rover-odometry
```

> 🔴 **`rover-odometry` has `StartLimitBurst=10`.** A crash-looping deploy latches `failed` and stops
> retrying — clear it with `sudo systemctl reset-failed rover-odometry` before starting again.

## B4. ⚠️ Config files that are NOT loaded at runtime

`config/rover_odometry.yaml` is **installed but never read.** The unit runs `ros2 run` with no
`--params-file`, so **only the `declare_parameter` defaults in the node take effect.** Editing the
yaml alone changes nothing. Keep both in sync; change the node default when you want a new value to
apply. The same trap may exist for other packages — check the unit's `ExecStart` before assuming a
yaml is live.

---

# Part C — Node inventory

## C1. Service summary

| Service | State | Purpose |
|---|---|---|
| `rover-camera` | ✅ enabled | Orbbec Gemini 336L wrapper — depth, colour, IMU, point cloud |
| `rover-scan` | ✅ enabled | depth image → `/scan` **and** the `base_link→camera_link` TF |
| `rover-scan-3d` | enabled, **inactive** | height-aware `/scan_3d`, runs *alongside* `rover-scan` |
| `rover-odometry` | ✅ enabled | VESC eRPM → `/odom` + `odom→base_link` TF |
| `rover-autonav-mode` | ✅ enabled | PX4 custom mode "AutoNav" + the collision reflex |
| `rc_control_node` | ✅ enabled | RC-channel actions: camera switch, shutdown |
| `microxrce-agent` | ✅ enabled | uXRCE-DDS bridge on `ttyAMA4` |
| `mavlink.router` | ✅ enabled | MAVLink fan-out, incl. `tcp:5760` |
| `vision_streaming` | enabled, **inactive** | FPV H.264 stream |
| `rover-ekf-bridge` | **disabled on purpose** | EV → PX4; wheels-up limit-cycle hazard |
| `tfmini` | **disabled** | ⚠️ **must be enabled for the DRONE** |

## C2. `rover-camera`

```
ros2 launch orbbec_camera gemini_330_series.launch.py \
  enable_accel:=true enable_gyro:=true enable_point_cloud:=true \
  point_cloud_decimation_filter_factor:=3 depth_registration:=true \
  color_width:=640 color_height:=360 color_fps:=30 depth_fps:=30
```

| Setting | Value | Why |
|---|---|---|
| `depth_registration` | true | RTAB-Map RGB-D needs depth in the colour frame. **Forces depth to 640×360** and the colour intrinsics |
| `color 640×360` | | keeps 16:9 and the full colour FOV — **640×480 would not** |
| `point_cloud_decimation_filter_factor` | 3 | full-res cloud ran at 10 Hz / 3.37 MB; this gives ~22 Hz / 0.37 MB |
| `color_fps` / `depth_fps` | 30 | ⚠️ **pin to 15/15 while recording a bag** (SD ceiling 27.4 MB/s), then revert |

**Key topics:** `/camera/color/image_raw` · `/camera/depth/image_raw` · `/camera/depth/points` ·
`/camera/color/camera_info` · `/camera/gyro/sample` (195 Hz) · `/camera/accel/sample`

⚠️ **`ros2 param set` does nothing** for the decimation — the live lever is the **service**
`/camera/set_point_cloud_decimation`.

## C3. `rover-scan` — `launch/depth_to_scan.launch.py`

> 🔴 **`is-active` IS NOT EVIDENCE FOR THIS UNIT, because it launches TWO nodes.** The unit's main
> process is the *launch*. Kill `depthimage_to_laserscan_node` alone — which `tools/s2_sensor_loss_test.py`
> does deliberately — and the launch survives on its other child, the `static_transform_publisher`.
> systemd therefore sees a perfectly healthy unit: **`Restart=always` never fires, `is-active` says
> `active`, and `/scan` is silent indefinitely.** This cost 10 minutes of a dead sensor on 2026-08-11.
>
> **After any test that kills the scan node:** `sudo systemctl restart rover-scan`, then **verify `/scan`
> is actually publishing** (subscribe to it — do not read a unit state). The same trap applies to any
> multi-node launch unit; `rover-camera` has its own version of it (§D2).

Two nodes:

**`static_transform_publisher`** — publishes `base_link → camera_link` from launch arguments
`cam_x/y/z`, `cam_yaw/pitch/roll`. Current: `(0.000, 0.000, 0.305)`, pitch 0.0406, roll 0.0100 rad.
⛔ This is the **only** source of that transform — never repoint this service at `/scan_3d`.

**`depthimage_to_laserscan_node`**

| Parameter | Value |
|---|---|
| `scan_height` | 40 |
| `scan_time` | 0.033 |
| `range_min` | 0.3 (below the 336L's usable floor) |
| `range_max` | 5.0 |
| `output_frame` | `camera_depth_frame` |

## C4. `rover-scan-3d` — `launch/cloud_to_scan.launch.py`

`pointcloud_to_laserscan` → **`/scan_3d`**, in parallel with `/scan`. Height-aware, and 12× steadier
close in (6 mm vs 74 mm). **Publishes no TF.** ⚠️ Contains the rover's own top plate by design —
every consumer must reject its own footprint.

## C5. `rover-odometry` — `wheel_odometry_node`

VESC eRPM from `/fmu/out/esc_status` → `/odom` + `odom→base_link` TF.

| Parameter | Default | Note |
|---|---|---|
| `erpm_to_ms` | **0.003900** | ground-distance scale, includes measured slip |
| `track_width` | 0.31 | m, hub to hub |
| `wheel_signs` | `[-1,1,1,1]` | only address 10 reports inverted |
| `left_addresses` / `right_addresses` | `[11,13]` / `[10,12]` | |
| `deadband_erpm` | 5.0 | |
| `esc_timeout` | 0.30 | s |
| **`yaw_source`** | **`camera_gyro`** | `camera_gyro` → `gyro` → `wheels` fallback chain |
| `camera_gyro_topic` | `/camera/gyro/sample` | |
| `camera_gyro_timeout` | 0.3 | s (~58 missed samples at 195 Hz) |
| `camera_gyro_max_gap` | 0.05 | s — larger stamp gaps are counted, **not integrated** |
| `camera_bias_window` | 12.0 | s of at-rest samples in the rolling bias mean |
| `camera_bias_min_samples` | 400 | refuses to integrate on a thinner estimate |
| `publish_at_rest` | true | emits zero-velocity when dozing ESCs would otherwise silence `/odom` |

**Expected startup log:**

```
wheel odometry up: ... yaw_source=camera_gyro
camera gyro unavailable — falling back to FC attitude     <- normal, pre-bias
camera gyro TF locked: base_link <- camera_gyro_optical_frame
camera gyro bias established: +0.69 deg/s from 400 at-rest samples
heading source: CAMERA GYRO (336L, bias-corrected)
```

⚠️ `/odom` is **RELIABLE** QoS — a BEST_EFFORT subscriber reads zero and mimics a dead topic.

## C6. `rover-autonav-mode` — `autonav_mode`

PX4 custom mode registered via `px4_ros2`. Publishes speed + yaw-rate setpoints, and **hosts the
collision reflex in its executor** — so the reflex is *not* independent of the companion, and
perception loss fails safe.

| Parameter | Default | Meaning |
|---|---|---|
| `collision.enabled` | true | |
| `collision.stop_distance` | 0.35 m | |
| `collision.clear_distance` | 0.50 m | hysteresis |
| `collision.sector_half_angle` | 0.35 rad | |
| `collision.corridor_half_width` | 0.275 m | plate half-width + heading margin |
| `collision.front_overhang` | 0.337 m | makes thresholds mean **bumper** clearance |
| `collision.footprint_front` | 0.345 m | |
| `collision.footprint_half_width` | 0.225 m | |
| `collision.footprint_margin` | 0.06 m | |
| `collision.blocked_yaw_rate` | 0.3 rad/s | |
| `collision.scan_timeout` | 0.5 s | |
| `collision.require_scan` | true | **no scan ⇒ no forward permission** |
| `collision.scan_topic` | `/scan` | ⏭ `/scan_3d` is the candidate, not yet switched |

Internal limits: `kMaxSpeed 0.8 m/s`, `kMaxYawRate 1.0 rad/s`, `kCmdVelTimeout 0.5 s`.

## C7. `rc_control_node`

Watches RC channels for operator actions — camera switch and shutdown. Parameters:
`channel_index`, `tolerance`, `hold_time`.

## C8. `microxrce-agent` / `mavlink.router`

```
MicroXRCEAgent serial --dev /dev/ttyAMA4 -b 921600
mavlink-routerd                      # config in /etc/mavlink-router/
```

MAVLink reaches the companion at **`tcp:127.0.0.1:5760`** — this is what `tools/set_param.py` uses.

> ⛔ Never infer radio health from MAVLink rates at `tcp:5760` — that measures the whole chain, not
> the link.

## C9. Built but idle

`obstacle_distance`, `rov_collision_stop`, `collision_manual_mode`, `optical_flow` — superseded by
the depth camera. No units, no live nodes. Slated for deletion.

---

## C10. PX4 external modes (px4_ros2) — how they work, how to add one, how they fail

AutoNav is not a PX4 flight mode. It is an **external mode**: a ROS 2 process that registers itself
with the flight controller at runtime and then supplies setpoints. Understanding the handshake
matters, because most of the ways it "breaks" are the handshake behaving exactly as designed.

### C10.1 The registration handshake

```
companion                                   flight controller
─────────                                   ─────────────────
NodeWithMode<AutoNavMode> starts
ModeBase(node, "AutoNav")
  └── RegisterExtComponentRequest   ──────▶  name, register_arming_check,
      (/fmu/in/…)                            register_mode, register_mode_executor
                                  ◀──────  RegisterExtComponentReply
                                             success · px4_ros2_api_version
                                             mode_id · arming_check_id
if !success                → throw, exit 250
if api_version mismatch    → throw, exit 250
if no reply before timeout → "timeout while registering external component"

then, continuously:
                                  ◀──────  arming check requests
  arming check replies            ──────▶
```

**Both halves must keep running.** After a successful registration the FC keeps polling the mode's
arming check; if those requests stop arriving the library throws
`"Timeout, no request received from FMU, exiting (this can happen on FMU reboots)"` and the process
exits with status 250. `Restart=always` then restarts it. **That exit is the library working
correctly**, not a crash in our code.

### C10.2 Mode ID → `nav_state`, and how to select the mode

The FC assigns a `mode_id` **at registration time**. External modes are selected over MAVLink /
`VehicleCommand` with `DO_SET_MODE`:

| `param1` | `param2` (main) | `param3` (sub) | Selects |
|---|---|---|---|
| 1 | 4 | **11** | EXTERNAL1 ← **AutoNav lives here**, `nav_state 23` |
| 1 | 4 | 12 … 18 | EXTERNAL2 … EXTERNAL8 |
| 1 | 4 | 3 | Hold |
| 1 | 1 | 0 | Manual |

```bash
# verified 2026-08-09: AutoNav is EXTERNAL1
ros2 topic echo /fmu/out/vehicle_status_v1 --once | grep -E "nav_state|arming_state"
```

⚠️ `param1 = 1` means "custom mode"; it is **not** optional. `from_external = true` and
`target_system = target_component = 1` are required — see `tools/l2_test.py::send_cmd`.

### C10.3 🔴 MODE REQUIREMENTS — the part that catches everyone

**A mode's requirements are not declared by the mode. They are derived from its setpoint type.**

`AutoNavMode` constructs a `RoverSpeedRateSetpointType`, whose `getConfiguration()` returns:

```cpp
config.control_allocation_enabled = true;
config.rates_enabled              = true;
config.attitude_enabled           = false;
config.velocity_enabled           = true;    // ← THIS creates the requirement
config.position_enabled           = false;
```

`velocity_enabled = true` means **PX4 requires a valid local velocity estimate** to run the mode.

> 🔑 **PX4 RELAXES MODE REQUIREMENTS WHILE DISARMED AND ENFORCES THEM WHILE ARMED.**
>
> This single fact produces the most confusing symptom in the system. Measured 2026-08-09:
>
> | | `DO_SET_MODE 4/11` | result |
> |---|---|---|
> | **disarmed** | accepted | `nav_state 23`, holds ≥20 s |
> | **armed** | **refused** | `nav_state` stays 0 — **as does every other mode, including Hold** |
>
> It presents as "the external mode is broken". It is not. It is the estimate requirement being
> enforced. QGroundControl shows it as a **navigation error**.

**Where the velocity estimate comes from indoors:** nowhere, unless **`rover-ekf-bridge`** is
running. There is no GPS indoors, and the bridge is the only external-vision velocity source.

```bash
# the check that tells you which situation you are in
ros2 topic echo /fmu/out/failsafe_flags --once | grep -E "local_velocity_invalid|local_position_invalid"
#   local_velocity_invalid: true   → AutoNav will be refused while armed
#   local_velocity_invalid: false  → good to arm
```

**Measured 2026-08-09/10:** bridge stopped → `local_velocity_invalid: true`, mode refused armed.
Bridge started → `false` within seconds → AutoNav engaged, held, drove, and S1 passed. The 2026-07-22
floor test recorded the same thing: *"rover-ekf-bridge started → v_xy_valid true → AutoNav arms."*

⛔ **Start the bridge on the FLOOR only.** Wheels-up + bridge + closed loop is a self-sustaining
limit cycle that only a disarm stops. **Stop it again afterwards.**

### C10.4 Mode lifecycle hooks

| Hook | When | AutoNav's use |
|---|---|---|
| constructor | process start | create the setpoint type, declare params, subscribe `/cmd_vel` and `/scan` |
| `onActivate()` | mode entered | hold zero until a `/cmd_vel` arrives |
| `updateSetpoint(dt)` | every control cycle while active | apply the collision reflex, then publish speed + yaw rate |
| `onDeactivate()` | mode left | stop |
| arming check | continuously, FC-driven | reports whether the mode is safe to arm into |

The reflex lives inside `updateSetpoint`, at the single funnel to the motors — so **nothing that
commands `/cmd_vel` can bypass it**, whether that is a test script, Nav2 or a joystick.

### C10.5 Adding a new external mode

1. Subclass `px4_ros2::ModeBase`, give it a unique name string.
2. Choose a setpoint type — **this determines the mode requirements** (C10.3). Pick the weakest one that
   does the job; every capability you enable adds an estimator requirement that must be satisfiable
   in your environment.
3. Implement `updateSetpoint()`, and `onActivate()`/`onDeactivate()` if state must be reset.
4. Wrap with `px4_ros2::NodeWithMode<YourMode>` in a `main.cpp`.
5. Add a systemd unit with `Restart=always` — the process legitimately exits on FC reboot.
6. It will land in the **next free EXTERNAL slot**. Do not hard-code sub=11 without checking; sweep
   11–18 and read `nav_state` back.
7. Verify: registration in the journal, `nav_state` reachable **disarmed**, then `failsafe_flags`
   clear, then armed.

### C10.6 Failure modes and their signatures

| Symptom | Cause | Action |
|---|---|---|
| `nav_state` stays 0 **armed**, works disarmed | mode requirement unmet — almost always `local_velocity_invalid` | start `rover-ekf-bridge` (C10.3) |
| `Timeout, no request received from FMU` → exit 250 | FC stopped polling the arming check; normal on FC reboot | systemd restarts it; check it settles |
| Restart every ~12 s, forever | **TRAP 1c** — the FC is holding a stale external-mode slot. A stop-and-wait does **not** clear it | **stop the service, reboot the FC, then start the service** — in that order |
| `Registration failed` / api version mismatch | `px4-ros2-interface-lib` and PX4 firmware out of step | match the lib branch to the firmware |
| Mode enters but nothing moves | the reflex is blocking forward, or no `/cmd_vel` | `journalctl -u rover-autonav-mode | grep collision-diag` |

> ⚠️ **`NRestarts` is cumulative since boot, not a rate.** On 2026-08-09 a count of 22 was read as an
> active crash loop; the service in fact had 20 minutes of uptime and the real cause was C10.3. Check
> the restart *timestamps* before concluding anything from the count.

# Part D — Bring-up and verification

## D1. Cold start

Services are `enabled` and start at boot. After boot, wait ~60 s, then:

```bash
systemctl is-active rover-camera rover-scan rover-odometry rover-autonav-mode \
                    rc_control_node microxrce-agent mavlink.router
```

## D2. Camera — ALWAYS verify, never trust `is-active`

```bash
python3 ~/ros2_ws/tools/camera_restart_check.py            # right after a restart
python3 ~/ros2_ws/tools/camera_restart_check.py --since=-120min --secs=15
```

Checks the unit, that **both** depth and colour streams actually started, live rates, depth↔colour
alignment, and unaligned-depth warnings.

> 🔴 **A restart can come up half-dead** — unit active, parameters answering, **depth and colour
> never started, nothing logged**. A second restart clears it.
> ⚠️ The stream-start check greps a **3-minute** journal window; outside it, it false-fails. **Trust
> the rates.** Use `--since=-120min` (equals form; `--since -120min` fails to parse).

Expected healthy: depth ~28–30 Hz, colour ~18–21 Hz at 30 fps configured.

## D3. Odometry and TF

```bash
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link camera_link
journalctl -u rover-odometry -n 20 | grep "heading source"
```

## D4. Flight controller link

```bash
python3 ~/ros2_ws/tools/set_param.py RO_YAW_RATE_P     # should read 0.08
ros2 topic hz /fmu/out/esc_status
```

## D5. Before any armed test

1. `vcgencmd get_throttled` → `0x0`
2. Camera verified (D2)
3. FC parameters re-read (A4) — they do not survive every reboot
4. `ps -eo pid,pcpu --sort=-pcpu` → nothing unexpected eating a core
5. 🔴 **S1 kill-switch test in AutoNav — see the conflict in D6 before relying on it**

## D6. Arming into AutoNav

**AutoNav is a custom `px4_ros2` external mode (nav_state 23) and CANNOT be armed directly via RC.**
Flipping the RC arm switch (ch5) arms into whatever the RC *mode* switch (ch6) asserts — which is
**Manual (nav_state 0)** — because an external mode has no RC switch slot. Arming "into AutoNav" from
RC therefore always lands in Manual, and a correctly-written test refuses to drive.

**Working path — arm in Manual, then software-switch:**

1. Operator arms via RC (ch5) with **throttle neutral** → armed in **Manual**.
2. Companion sends `DO_SET_MODE main=4 sub=11` → **AutoNav**. It takes and holds; the RC mode switch
   does not yank it back (verified with a 2 s hold check).
3. `onActivate` holds zero output until `/cmd_vel` arrives.

`tools/l2_test.py` implements this. It tolerates an already-armed-in-Manual start (wheels must be
stopped), skips the RC-arm wait, sends AutoNav and verifies it holds before moving. It **never
software-arms — the operator is always the arming authority.** The operator cannot see the script's
live stdout, so "arm on cue" is unreliable; **arm first** is the intended flow.

### Safety interlocks

- ⛔ **Never arm AutoNav on stands with the EKF bridge running.** Wheels-up + bridge + closed loop is
  a self-sustaining front/back limit cycle that only a disarm stops. Validate the brake **passively**
  on stands (diagnostic + `/scan`); arm only on the **floor**, where odometry is real.
- The RC **kill (ch8)** is the final authority.

> ✅ **RESOLVED 2026-08-09 — the kill switch PASSED on 2026-07-22.** The L2 floor-test record has
> *"kill switch (ch8) works ARMED inside AutoNav — now confirmed live"*, and independently, during
> a software-armed run the same day the rover drove at a wall and the operator *"stopped it with
> the KILL switch (ch8) before impact"*. `autonomy_plan.md`'s "S1 untested" was the stale record.
>
> 🔴 **BUT THE PROCEDURE ABOVE IS INCOMPLETE WITHOUT THIS PRECONDITION.**
> AutoNav uses `RoverSpeedRateSetpointType` (`velocity_enabled = true`), so PX4 requires a valid
> local **velocity** estimate. Indoors the only source is **`rover-ekf-bridge`**. The July run
> records *"rover-ekf-bridge started → v_xy_valid true → AutoNav arms"*, and *"bridge stopped
> after → safe"*.
>
> With the bridge stopped, `failsafe_flags` reads `local_velocity_invalid: true` and **the mode
> switch is refused while armed** — measured 2026-08-09: disarmed `DO_SET_MODE 4/11` → nav_state
> 23 and holds ≥20 s; armed → refused, as is every other mode including Hold. PX4 relaxes mode
> requirements when disarmed and enforces them when armed, so this looks like a broken mode and
> is not one.
>
> **Step 0 of the procedure above: start `rover-ekf-bridge` and confirm `v_xy_valid`.**
> ⛔ **ON THE FLOOR ONLY.** The limit-cycle hazard is wheels-UP: bridge + closed loop + wheels off
> the ground is self-sustaining and only a disarm stops it. Stop the bridge again afterwards.

---

# Part E — Routine operations

## E1. Recording a mapping bag

```bash
# 1. pin the camera to 15/15 (SD write ceiling)
sudo tee /etc/systemd/system/rover-camera.service.d/20-mapping-fps.conf   # ExecStart with 15/15
sudo systemctl daemon-reload && sudo systemctl restart rover-camera
python3 ~/ros2_ws/tools/camera_restart_check.py          # verify, expect a half-dead retry

# 2. record
ros2 bag record -o ~/map_run_$(date +%Y%m%d_%H%M%S) \
  /camera/color/image_raw /camera/color/camera_info \
  /camera/depth/image_raw /camera/depth/camera_info \
  /scan /tf /tf_static /odom /camera/gyro/sample

# 3. afterwards: delete the drop-in, daemon-reload, restart, verify 30/30
```

⛔ **Also record `/fmu/out/esc_status`** — without it odometry can only be rescaled after the fact,
never recomputed. (Learned the hard way; not yet in the command above by default.)

## E2. Replay-mapping

```bash
export ROS_DOMAIN_ID=42        # MANDATORY — the live stack is on domain 0
ros2 run rtabmap_slam rtabmap --ros-args \
  --params-file src/rover_nav2/config/rtabmap_mapping.yaml \
  -p database_path:=/home/roz/house_map_vN.db \
  -r rgb/image:=/camera/color/image_raw \
  -r depth/image:=/camera/depth/image_raw \
  -r rgb/camera_info:=/camera/color/camera_info
ros2 bag play <bag> --clock --rate 0.3
```

> 🔴 **Shut it down by signalling the rtabmap BINARY, not the `ros2 run` wrapper.** SIGINT to the
> wrapper never reaches the child, and the database closes with `0 words / Optimized graph: 0 poses`
> — the whole graph lost. Find it with `pgrep -f /opt/ros/jazzy/lib/rtabmap_slam/rtabmap`, allow
> ~90 s to drain first, and confirm `Saving database ... done!` in the log.

## E2b. Verifying a map — MANUAL, BY THE OPERATOR

**Do this before localizing in a new map.** A map can be internally consistent, score well on every
statistic, and still be a picture of nowhere. The only person who can say whether it is the actual
room is the person who drove it.

```bash
python3 ~/ros2_ws/tools/map_review.py ~/house_map_vN.db
# writes <name>_review.png, <name>_plan.png and the exported cloud beside the database
```

The tool exports the cloud and the keyframe poses, **removes the rover's own body**, and renders
three views plus a plan view with the map axes and the driven path.

> **Why the rover body is stripped.** The mask that crops the plate out of each depth frame is
> discarded whenever the cropped height does not divide exactly by `Grid/DepthDecimation`
> (`autonav_reference.md` §9). In `house_map_v4` that was **31,299 of 266,053 points — 11.8% of the
> cloud**: the plate drawn at *every position the rover occupied*, so the map showed **a trail
> through the floor tracing exactly where the vehicle drove.** That is what makes a perfectly good
> map look broken. `--keep-rover` shows the contamination if you want to inspect it.
>
> **Recognising it without the tool:**
> - **Visually** — a band of clutter following the driven path, thickest where the rover lingered.
> - **In the log** — `util3d.cpp:1251 ... Cannot apply ROI ratios ... Ignoring ROI ratios`, **once
>   per frame** (740 times in one 488 s replay). If you see that during a mapping run, **the map you
>   are building contains the rover.**

### The five questions — put these to the operator

| | Question | What the answer means |
|---|---|---|
| **Q1** | **Any wall drawn twice?** Two parallel lines a few cm apart in the plan view, or a corridor at two slightly different angles. | **The one that decides it.** Doubling is the drift signature. In `house_map_v3` every wall was multiplied — the cause was a 19% odometry over-report, not heading. None ⇒ the geometry is sound. |
| **Q2** | Is the shape the room? Proportions, doorways, where corners fall. | Wrong shape ⇒ the map is not of this place; stop and rebuild. |
| **Q3** | Recognisable objects in true colour, in the right places *relative to each other*? | Confirms the map is metrically sound, not merely self-consistent. |
| **Q4** | Anything **missing** that was driven past? | A coverage gap, **not** a geometry error — different problem, different fix (drive it again). |
| **Q5** | Is the floor flat? Orbit to eye level and sight along it. | A sloping or bending floor is accumulated **pitch** error. The wall-calibration rig only ever checks yaw, so nothing else catches this. |

### Reading the numbers the tool prints

- **`room` vs `driven`** — the grid always extends beyond where the rover went, because the camera
  sees ~4 m ahead. Those outer regions were observed at distance and never approached, so they are
  the least accurate part of the map. In v4: a 2.98 × 3.92 m room from a 1.92 × 1.69 m driven area.
- **Rover body percentage** — should be near zero once `Grid/DepthDecimation` is right. A large
  number means the ROI mask is being discarded again.

⚠️ **Glossy dark surfaces map badly** and this is expected, not a failure of the run: a blue steel
almirah came out ~1.5× thicker than matte walls and closer than it really is. The error is toward the
rover, so it **fails safe** for navigation. Do not "fix" it by deleting points from the cloud — that
changes the picture, not the map.

## E3. Localizing

```bash
ros2 run rtabmap_slam rtabmap --ros-args \
  --params-file src/rover_nav2/config/rtabmap_localization.yaml \
  -r rgb/image:=/camera/color/image_raw \
  -r depth/image:=/camera/depth/image_raw \
  -r rgb/camera_info:=/camera/color/camera_info
```

Success is `map→odom` **changing**, not merely existing — it is restored from the database at
startup. Watch `ros2 run tf2_ros tf2_echo map odom`.

## E4. Measuring anything

```bash
ps -eo pid,pcpu --sort=-pcpu | head        # ALWAYS first
bash ~/ros2_ws/tools/cpu_catcher.sh        # traps short-lived CPU spikes
python3 ~/ros2_ws/tools/wall_probe.py      # absolute reference: wall distance ±1 mm, bearing ±0.26°
```

> 🔑 **Start the measurement, then go quiet.** Issuing commands during a perception measurement
> starves the camera and produces confident wrong answers.

## E5. Recovery

| Symptom | Action |
|---|---|
| No MAVLink | check `ttyAMA0`, the PX4 MAVLink instance, `mavlink.router` |
| No DDS topics | `microxrce-agent`, `ttyAMA4`, the XRCE parameter on the FC |
| No `/scan` or no cloud | **half-dead camera first** (D2) |
| Localization degrading | **measure the colour rate**, then restart the camera |
| Service latched `failed` | `sudo systemctl reset-failed <unit>` |
| No video | CPU-starvation latch first — a restart alone does not clear it |

⚠️ Remote recovery path is **WFB → relay `:2222` → companion `:22`.** There is still no onboard
**Wi-Fi** fallback — `wlan0` is off at the device-tree level. The wired path below is the only
other way in.

## E5b. Wired fallback — when WFB *and* the Wi-Fi uplink are both down

Added 2026-09-03. Config: `/etc/netplan/60-eth0-recovery.yaml` (+ a `RequiredForOnline=no`
drop-in in `/etc/systemd/network/10-netplan-eth0.network.d/`). `eth0` is `optional`, so an
unplugged cable never delays boot, and its DHCP route metric is **300** — it can never outrank
the Wi-Fi uplink's metric 50 while that uplink is alive.

Plug a cable into the Pi's Ethernet port. Three ways in, in order of what the far end offers:

| Far end | Companion gets | How you connect |
|---|---|---|
| Router / phone tether / laptop sharing | DHCP lease, route metric 300 | whatever the router hands out |
| Laptop set to "automatic" (no DHCP server) | `169.254.x.x` link-local | `ssh roz@Vind-Roz.local` — avahi/mDNS, **zero config both ends** |
| Laptop set to static `10.41.10.50/24` | `10.41.10.1/24`, always | `ssh roz@10.41.10.1` — no name resolution needed |

`10.41.10.0/24` is the PX4 ethernet subnet and was already reserved for `eth0`. It collides with
nothing in use: `192.168.1.0/24` = LAN uplink, `10.5.5.0/24` = WFB/relay tunnel, `10.5.7.0/24` =
relay cluster plan. A future wired FC link (PX4 defaults to `10.41.10.2`) still fits alongside it.

⚠️ The address only appears **once carrier is present** — `ip -br addr show eth0` reading empty
with no cable in is correct, not a fault. Verify after plugging in with
`networkctl status eth0` (expect `State: routable`), not with `is-active`.

---

# Part F — Not yet written

Stubs, in the order they will likely be needed:

- **F1 — Map creation, end to end.** ✅ *Partly written:* E1 records the bag, E2 replay-maps,
  **E2b verifies the result with the operator**. Still missing: the **drive pattern** as a procedure
  (station spacing, how much rotation at each, how to close the loop) and **quantitative acceptance
  criteria** to sit alongside E2b's five questions.
- **F2 — Map export and QGC integration.** Getting an occupancy grid into QGroundControl, frame and
  origin conventions, how the operator picks a goal on it.
- **F3 — Mission definition and execution.** How a patrol route is expressed, stored and run; what
  AutoNav consumes; abort and resume behaviour.
- **F4 — Nav2 configuration.** Costmap layers, footprint, inflation, and **the recovery behaviours
  that must stay deleted** (`autonav_reference.md` §11).
- **F5 — Outdoor bring-up.** DroneCAN GPS, STL-19 re-integration (lidar then owns `/scan`, depth
  remaps to `/scan_depth`), GPS waypoint missions.
- **F6 — Drone configuration.** The same FC on the aerial airframe: which parameters differ, and the
  revert checklist.
