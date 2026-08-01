# FPV video — specification, architecture, faults and change history

**Consolidated 2026-08-01** from ~800 lines of session notes that previously lived only in Claude
memory. That log is retired; this file is the durable record. Everything below was measured on this
hardware, not taken from a datasheet.

---

## 1. Specification

**Job.** Take MJPEG from a USB FPV camera, encode H.264, emit RTP to the local WFB video port, which
carries it over the 5 GHz link to the ground station and QGC.

| | |
|---|---|
| Service | `vision_streaming.service` → `vision_streaming_node.py` |
| Config | `/etc/vision_streaming.conf` |
| Output | RTP → `127.0.0.1:5602` → `wifibroadcast@drone` |
| Encoder | **software x264 only** — see §1.2 |
| Operator control | QGC → `pxlabs_cli` → `vision_config_manager` (§2.3) |

### 1.1 Config keys — what actually takes effect

| Key | Works? | Notes |
|---|---|---|
| `resolution` | ✅ | The main CPU lever. 640×360 measured ffmpeg **78–95% → 25.7%** of a core |
| `bitrate` | ✅ effective, ❌ not settable from QGC | `set-cam-params` takes resolution/fps/format only; the `3000K` default applies only when the key is absent |
| `format` | ✅ | |
| `camera_id` | ✅ | **The stable key.** `usbcam-<vidpid>-<serial>-i<iface>` |
| `camera_name` | ⚠️ | A `/dev/videoN` path. Goes stale; see §4.1 |
| **`fps`** | 🔴 **INERT** | The node parses `resolution`/`bitrate`/`format` only and never passes `-framerate`. Verified live: conf said 15 while `v4l2-ctl --get-parm` reported 30. QGC→conf works; **conf→ffmpeg is the missing hop.** The QGC control does nothing and its readout is not trustworthy |

> 🔴 **Never record resolution / fps / bitrate as fact anywhere.** They are operator-set from QGC and
> change without notice. A previous note asserting "now 1280×720, user confirmed KEEP" was already
> false when written and propagated into three files. **Read `/etc/vision_streaming.conf` or the
> running ffmpeg args at the moment you need the value.**

### 1.2 The Pi 5 has no hardware H.264 encoder

`v4l2h264enc` is **missing**; `v4l2-ctl --list-devices` shows only `rpivid`, which is **decode-only**.
ffmpeg lists `h264_v4l2m2m` but that is a wrapper with no m2m encoder device to bind. **All H.264 on
this platform is software x264.** Encoder choice is not a lever.

GStreamer 1.24.2 is installed with every needed plugin and offers **no performance advantage** — same
x264 cost. Its only edge is surfacing device faults as bus messages rather than scraped stderr.
**Stay with ffmpeg**: `-progress` is what makes the watchdog possible at all.

**MJPEG passthrough** would cut CPU to near zero and is all-keyframe (genuinely better over a lossy
link), but 720p30 MJPEG is ~8–15 Mbit/s against a 13 Mbit/s MCS1 PHY already ~34% used — **it does not
fit at 720p.** 640×480 (~3–5 Mbit/s) would. Read defect 1 in §5 before attempting it.

### 1.3 Cameras

Both are FPV-capable and both are proven on port **6-2**. Swap from QGC only.

| | See3CAM_CU135 | LG Smart Cam |
|---|---|---|
| id | `usbcam-2560c1d1-241D8306-i00` | `usbcam-30c9009d-01.00.00-i00` |
| draw | **100 mA** | **500 mA** |
| real rate | **60 fps** 720p MJPG over USB 2.0 | 30 fps |
| CPU (720p) | ≈48.6% of a core @16 fps | ≈84.6% of a core @30 fps |

> ✅ **Correction, kept because it misled for days:** "See3CAM only does ~16 fps on the 480M bus" was
> **wrong**. Measured at 1280×720 MJPG: auto-exposure in a dark room **16.6 fps**, manual exposure
> 5 ms **59.4**, 1 ms **59.6**. Long auto-exposure in low light caps the rate — normal camera physics,
> not a bus limit. The tell: fps was ~15–16 at 640×480, 720p *and* 1080p alike; a bandwidth limit
> would have made the small mode much faster. **You do not need a blue USB3 port for full rate.**

**Orbbec Gemini 336L is not part of this path** — autonomy only, ROS2 wrapper, never ffmpeg.

---

## 2. Architecture

### 2.1 Pipeline

```
USB camera (MJPEG) → ffmpeg: decode → yuv420p → libx264 → -f rtp → 127.0.0.1:5602
                       │
                       └─ -progress → stall + throughput watchdog
```

> ⛔ **The ffmpeg command line (`vision_streaming_node.py` ~176–207) is VETOED for changes by the
> user.** Do not propose `-fps_mode`, `-g 30`, `-tune zerolatency`, `-pkt_size`, or `fps`→`-framerate`
> again. Recorded so the analysis is not repeated: with `-f rtp` the muxer forces constant frame rate,
> so ffmpeg **dup-pads a 15 fps camera to 60 fps — 600 frames encoded from 151 real ones**, 4× the work
> for the same picture. `-fps_mode passthrough` halves CPU (94% → 42%). It is understood and rejected.

> ⛔ **User rule, verbatim:** *"do not hardcode the frame rate in the ffmpeg — it will be handled from
> QGC through the config file."* A literal `-framerate 30` is forbidden.
> **Unresolved, ask before deciding:** does that rule also forbid passing the *conf's own* value
> through (`-framerate {conf fps}` — config-driven, nothing hardcoded)? If it does, QGC's fps control
> stays permanently decorative. **Check first:** `v4l2-ctl -d <dev> --list-formats-ext` — the camera may
> not offer 15 fps at all, in which case the missing hop is only half the story.

### 2.2 Watchdog — two independent checks

| Check | Trips on | Settings |
|---|---|---|
| **Stall** | no new frames | `STALL_TIMEOUT_S` 10, `STARTUP_GRACE_S` 30 |
| **Throughput floor** | frames still arriving but far too few | `RATE_FLOOR_FPS` 5.0, `RATE_WINDOW_S` 20, `RATE_GRACE_S` 30 |

The throughput floor exists because the stall check only tested *liveness*: any frame-counter
increment reset the timer, so a **30× throughput collapse was invisible by construction**.

It deliberately **does not reset the backoff** — a long run ending degraded has not earned a fast
retry, and resetting would restart-loop every ~60 s under sustained load.

Verified with `systemctl set-property --runtime vision_streaming.service CPUQuota=5%` → fired in ~30 s
(*"only 2.6 fps over the last 22s (floor 5 fps) after 216s — throughput collapse, not a stall"*).
Healthy 212 pkt/s produced no false trip over 75 s.

> ⚠️ **The watchdog only works because libx264 is in the pipeline.** With `-c copy`, `-progress` emits
> no `frame=` key at all (only `bitrate drop_frames dup_frames out_time* progress speed total_size`).
> **Any move to MJPEG passthrough would silently disable it** — `saw_frames` never becomes true, so it
> would kill a healthy stream every 30 s forever. If that path is taken, key liveness on `out_time_us`.

### 2.3 `vision_config_manager` — the operator's writer

A **separate program** at `/usr/local/bin/vision_config_manager`, not part of this ROS package. QGC
drives it: `G-Control.exe → pxlabs_cli.exe → SSH relay:2222 → companion:22`.

It stops the service, edits `/etc/vision_streaming.conf`, and restarts. Both setters now stop the
service only *after* the work that can `sys.exit(1)`, and are wrapped so `SystemExit` triggers
`ensure_service_up()` before re-raising.

> ⚠️ `resolve_camera()` calls `discover_cameras()` first, so **every** QGC camera operation — not just
> `list` — probes every video node with `--list-formats-ext`, **including the live streaming one.**

---

## 3. Faults — two classes that look identical

Both present as "no picture in QGC". Distinguish them before acting.

### 3.1 🔴 CPU-starvation latch — CHECK THIS FIRST, needs no hardware work

**Symptom:** no picture; camera enumerated and healthy; radio flawless; service `active`; **node logs
completely silent** — no warning, no error, no restart, for over an hour.

**Mechanism:** ffmpeg loses a CPU race against the rover stack (`rover-camera` ~41%,
`wheel_odometry` ~26%, `MicroXRCEAgent` ~13%, `depthimage_to_laserscan` ~9%), falls behind, and
**never catches up. It is a LATCH, not a steady state** — output degrades to ~3 s bursts every ~14 s,
**7–28 pkt/s against 208 healthy**, indefinitely. QGC shows black rather than stutter because with
~10 s gaps it rarely catches a keyframe.

> ⛔ **Restarting the service does NOT clear it** — it re-enters the same load and falls behind within
> seconds.

**FIX (60 s, reversible):**
```bash
sudo systemctl stop rover-camera rover-scan rover-odometry
```
→ **7 → 208 pkt/s**, and it stays healthy after those services are restarted. Measured both ends.

Dropping resolution to 640×360 cuts ffmpeg to 25.7% of a core and largely defuses this.

### 3.2 ⛔ Camera wedge — no software recovery exists. Do not build one.

The camera stops feeding frames while staying enumerated. Tested against a live-wedged unit, in
escalating order — **every one returned zero frames:**

| test | result |
|---|---|
| `ffmpeg -c copy` (no encoder at all) | 0 packets muxed in 20 s |
| `uvcvideo` unbind → rebind `6-2:1.0` | 0 packets |
| USB de-authorise → re-authorise (full re-enumeration) | 0 packets |
| retry at 640×480 (lowest isoc bandwidth) | 0 packets |
| GStreamer `v4l2src ! image/jpeg ! fakesink` | 0 buffers, hung |

**Only physical VBUS removal clears it.** Do not build "escalate to a USB port reset after N
zero-frame starts" — it is measured **not viable**. The node can be made to fail loudly; it cannot be
made to self-heal on this hardware.

**The kernel is silent throughout** — zero `-71`/`-110`, no babble, no bandwidth error, no port reset,
no xhci errors. `throttled=0x0`, core 0.85 V. The device stops filling isoc packets and the host
controller is perfectly happy.

> 🔴 **"The LG is faulty hardware" was asserted and is WRONG, or at best intermittent** — it later ran
> fine on the same port. What is *not* separated: LG internally faulty vs connector 6-2 bad under load.
> See3CAM draws 100 mA, LG 500 mA, and a contact-resistance fault produces exactly this result.
> `ext5v-report` reads the rail **upstream** and is blind to a drop at the connector.
> **Discriminator not yet run:** put the LG in the free port `4-1` (different host controller, different
> power path) and soak.

### 3.3 Most "stalls" are cold-start failures, not stalls

186 `no new frames` events bucketed by ffmpeg runtime at failure:

| runtime at failure | count | meaning |
|---|---|---|
| **exactly 32 s** | **97** | `STARTUP_GRACE_S`(30) + one watchdog tick; `saw_frames` never true |
| 30 / 34 / 36 s | 25 | same band |
| 38–70 s | ~30 | ambiguous |
| >100 s | **9** | the only true mid-stream stalls |

**A failure at 32 s with `stalled_for == runtime` means ffmpeg got zero frames from the very first
one.** Only 9 of 186 are the "streams fine then dies" case.

**The recovery path is the real outage amplifier:** one glitch produced **19 consecutive 32 s failures
= 20 minutes of dead video**. Stall → ffmpeg wedged in a v4l2 ioctl ignores SIGTERM → SIGKILL at 5 s →
camera never gets a clean STREAMOFF → next open yields no frames → repeat. At max backoff the cycle is
**30 s backoff + 32 s grace + 5 s kill ≈ 67 s per attempt.**

---

## 4. Measurement traps — these cost days

1. **Per-second samples of `video tx incoming` on port 8102 land inside the burst gaps and read 0.**
   That produced a wrong "WFB's video listener is dead" verdict. **Always use a cumulative delta over
   ≥30 s**, never per-second snapshots.
2. **A manually launched ffmpeg sending to `127.0.0.1:5602` never moves WFB's counter**, though the
   service's does. Cause not established. ⇒ **Never A/B ffmpeg flags via the WFB counter.** Use
   ffmpeg's own `N packets read / frames decoded / frames encoded` summary, or the live service.
3. **Toggling `auto_exposure` mid-stream stalls the camera for >12 s and trips the watchdog** — this
   caused a false alarm. **Stop the service before touching v4l2 controls.** Also
   `exposure_time_absolute` is `inactive` in Auto Mode and writes return `Permission denied` (a v4l2
   rule, not sudo): set auto→manual, write, then auto again.

**Best single diagnostic.** Stop the service, run ffmpeg by hand with `-loglevel verbose`, and read
`N packets read / N frames decoded` plus `*** N dup!`. **x264 dup-padding means the live RTP stream is
not the live camera.** Healthy baseline: **152 packets in 10 s = 15.2 fps, 0 decode errors, speed
0.99×**. ~5 `No JPEG data found` lines in the first second after any start are benign sensor settling.

---

## 5. Known code defects — verified against the live file, all UNAPPLIED

1. **`-progress` has no `frame=` key under `-c copy`** — see §2.2. Blocks MJPEG passthrough.
2. **`fps` in the conf is never read** — see §1.1.
3. **No `-g` / GOP control.** x264 default `keyint` = 250 frames ≈ **8.3 s at 30 fps**; over a lossy
   radio one lost keyframe smears for up to 8 s. Strongest candidate for what reads as "WFB is flaky".
   ⛔ The fix (`-g 30`) is vetoed — see §2.1.
4. **`frame=0` counts as progress** (`drain_progress`): `0 > last_frame(-1)` advances
   `last_progress_at` without setting `saw_frames`, so the 30 s grace silently becomes ~50 s.
5. **Backoff is not reset on the stall path.**
6. **`rclpy.shutdown()` RCLError fires on every QGC camera change**, not just manual restarts.
   Harmless, but dumps a traceback exactly when you would be checking whether the change worked.
7. **The `camera_name` fallback is dangerous.** `/dev/video0` did not exist for most of one day.
   If sysfs resolution ever fails, ffmpeg is pointed at a node that is not the camera.

### 5.1 Open feature — bitrate control from QGC (agreed, not started)

`bitrate` never tracks a resolution change because QGC cannot set it. Companion half, designed:
`set-cam-params` gains an **optional** `--bitrate` (must stay optional — the shipped QGC build calls
it without), validated `^\d+[KM]?$`; `update_cam_params_config()` gains `bitrate=None` and replaces or
appends the line as resolution/fps/format already do. **Do not hand-edit the conf as a workaround.**

---

## 6. Device numbering — never key on `/dev/videoN`

- **The Orbbec's `/dev/videoN` nodes appear and disappear with `rover-camera.service`.** Wrapper
  running → OrbbecSDK/libusb claims the device, uvcvideo detaches, **no Orbbec nodes**. Wrapper stopped
  and camera plugged → uvcvideo binds and creates **8 nodes**, and the Orbbec takes **`video0`**.
  The same question got opposite answers hours apart; both observations were correct for their moment.
- `video19`–`video37` are the Pi 5's own on-chip `rpivid` + `pispbe-*` on the `axi` bus — not cameras.
- Renumbering comes from those ISP nodes claiming low numbers at boot, plus USB rebinds.

> 🔴 **Key every camera by `usbcam-<vidpid>-<serial>-i<iface>`, never `/dev/videoN` or by-id.**
> This is also the root trigger of the `rc_control` retry storm (see that node's history).

---

## 7. Change history

| Date | Change |
|---|---|
| 07-27 | Node `164420e` — capture node resolved **by sysfs index**, plus the stall watchdog. First clean 34+ min run. |
| 07-27 | `vision_config_manager` **v2.2.1** — both setters stop the service *after* anything that can `sys.exit(1)`; `SystemExit` wrapped to call `ensure_service_up()`. Under v2.1.0/v2.2.0 a bad `set-cam-params` took the feed down. |
| 07-28 | **v2.2.3** — root cause of "resolution changes silently fail to persist" was **`fs.protected_regular=2`**: `/tmp` is sticky and world-writable, so the kernel refused a root `O_CREAT` open of a **roz-owned** `/tmp/vision_streaming.conf`, and `sudo cp` rewrote it in place preserving ownership so it never healed. Fix: `staging_path()` = `tempfile.mkstemp()` per invocation. The fixed `/tmp` path is gone from all four write sites. codex-work `94e5ef6`. |
| 07-28 | Earlier bug, same symptom: `update_resolution_only_config()` matched its section by **substring of the device path** (`if "camera_name" in line and device in line`). A stale `camera_name` meant the match never fired and the file was rewritten byte-identical while reporting success. **Match on `camera_id` instead.** |
| 07-30 | LG proven unrecoverable in software (§3.2); See3CAM swapped onto the same port and worked immediately. |
| 07-31 | Soak: **41.8 min clean**, 99.99% delivered at the relay. |
| 08-01 | **Throughput-floor watchdog** added and verified live — `a5fb348`. |

---

## 8. Standing rules

1. ⛔ **Do not modify the ffmpeg command line.** Vetoed; the alternatives are analysed in §2.1.
2. ⛔ **Never hardcode a frame rate.** It comes from QGC via the conf.
3. ⛔ **Do not build software USB recovery.** Measured not viable (§3.2).
4. **Cameras are configured only from QGC.** Never hand-edit `/etc/vision_streaming.conf`. A camera
   swap is not an exception.
5. **Never record resolution / fps / bitrate as fact.** Read them live.
6. **Never key a camera by `/dev/videoN`.** Use the `usbcam-` id.
7. **Stop the service before touching v4l2 controls.**
