# Vind-Roz Rover — Autonomy Plan, Operating Modes and Test Definition

Status: 2026-08-01. Owner: roz. Companion doc to `roadmap.md` (ladder/status) — this
one defines **what the rover does for a user**, not what the ladder step is called.

---

## 1. The question this document answers

"Autonomous" is not one thing. Before building, we state **how autonomous, for which
application**, because the answer changes what hardware and safety we need.

Every autonomous vehicle must answer four questions. Ours are in very different states:

| | Question | Provided by | Our state |
|---|---|---|---|
| Q1 | **Where am I?** | Localization | ❌ **The main gap** |
| Q2 | **What is around me?** | Perception | ✅ Forward sector only |
| Q3 | **How do I get there?** | Planning | 🔧 Configured, unproven |
| Q4 | **What if it goes wrong?** | Failsafe | ⚠️ Partial |

**Q1 is the wall.** Obstacle avoidance is nearly solved; knowing *where the rover is*
is not. Every capability above "drive 3 m forward" — go to a room, patrol, return
home — is a localization problem, not an avoidance problem.

---

## 2. Sensor roles — what each is FOR

A 2D lidar sees **one horizontal slice at one height**. A depth camera sees a **3D
volume ahead**. These are complementary, not redundant.

| Obstacle | 2D lidar (STL-19) | Depth camera (Gemini 336L) |
|---|---|---|
| Table top, desk edge | ❌ passes underneath | ✅ |
| Chair seat, shelf, overhang | ❌ | ✅ |
| Low box, threshold, cable | ❌ passes over | ✅ |
| Stairs / drop-off | ❌ | ✅ |
| Wall, door frame | ✅ | ✅ |
| **Behind and beside** | ✅ | ❌ **physically cannot** |
| Long range, dark room | ✅ | ⚠️ weaker |

**Depth camera = safe while going forward, and sees a class of obstacle the lidar never
will. Lidar = safe while turning/reversing, and robust geometric SLAM.**
A lidar-only rover drives under a table and wedges itself.

### 2.1 ⚠️ We are currently using the depth camera badly
Today: `depth image -> depthimage_to_laserscan -> /scan`. That **flattens a 3D depth
image into one horizontal line**, discarding the exact advantage the camera has. No
point cloud is published at all (verified 2026-08-01: only `/camera/depth/image_raw`).
We have configured a 3D sensor to imitate a mediocre 2D lidar and then judged it as one.
**Fix is configuration, not hardware:** enable the point cloud, add a Nav2 `voxel_layer`.

### 2.2 ⚠️ Correction: indoor mapping does NOT require the lidar
> **→ The concrete plan, measured costs and next steps now live in
> [`indoor_mapping_plan.md`](indoor_mapping_plan.md).** Mapping runs OFFLINE on a laptop; the Pi
> records a bag and later does localization only. Drive the run in MANUAL — it bypasses the yaw rate
> loop, so mapping is **not** blocked on #20, but there is **no collision reflex in Manual.**

An earlier claim that "mapping the house needs the STL-19" was **over-generalised and
is withdrawn**. What is true: **`slam_toolbox` needs it**, because it does 2D scan
matching and 92 deg of view gives too little overlap.
**RGB-D visual SLAM (RTAB-Map) is designed for exactly this camera** — it matches
visual features in 3D, so a forward-facing narrow FOV is its normal case, and it closes
loops by *recognising places*, which works fine at 92 deg. It outputs a 3D map plus a 2D
occupancy grid Nav2 can use.
⇒ **Level M3 below is NOT hardware-blocked.** It is blocked on configuration and CPU.

### 2.3 The real limiter is CPU, not FOV
4 cores already carry camera + depth->scan + odometry, with a documented starvation
failure mode. Nav2 + voxel costmap + RTAB-Map on top is the genuine risk.
**Measure headroom before designing around RTAB-Map.**

---

## 3. Application A — INDOOR SURVEILLANCE (first target)

**What the user does:** picks a room or a patrol route on a saved house map, presses go.
**What the rover does:** localizes itself, routes there, avoids people and moved
furniture, streams video, returns to base, repeats.

This application is **repeat-the-same-route in a semi-static environment**. That shape
matters: the house does not change much, so **map once and re-localize** is exactly
right, and the hard part is dynamic obstacles (people) and reliable re-localization.

### Features required

| # | Feature | Why | State |
|---|---|---|---|
| A1 | House map, built once, saved | Routing between rooms | ❌ RTAB-Map |
| A2 | Re-localize on startup | "Where am I?" after power-on | ❌ |
| A3 | Global route planning | Room to room, around walls | 🔧 Nav2 configured |
| A4 | Dynamic obstacle avoidance | People, moved chairs | ✅ forward only |
| A5 | 3D obstacle layer | Table tops, low objects | ❌ needs point cloud |
| A6 | Patrol / waypoint executor | Route, not single goal | ❌ |
| A7 | Video streaming during patrol | It IS the surveillance | ✅ but costs 21% of `/scan` |
| A8 | Return to base | End of patrol, low battery | ❌ needs A1+A2 |
| A9 | Safe stop on lost localization | Do not drive blind | ❌ |

**A7 is a real conflict:** FPV streaming costs `/scan` 28.4 -> 22.3 Hz and worsens the
reaction gap to 235 ms. For surveillance we WANT video while driving. Resolve by driving
slower while streaming, or by proving the CPU budget.

**A8 note:** PX4 RTL drives to a **GPS** home position. Indoors there is no GPS, so
"return to base" means *routing home on our own map* — it only exists once A1+A2 exist.
Until then the honest failsafe is **stop and wait for the operator**.

---

## 4. Application B — OUTDOOR GPS MISSION

**User question answered directly: does it plan a map, or follow a pre-made map?**

**Neither, for localization — outdoors GPS answers Q1 directly.** No prior map is needed
to know where the rover is. That is the fundamental difference from indoors.

So the split is:

```
GLOBAL  (where to go)   <- GPS waypoints. Known before starting. No map needed.
LOCAL   (what is in the way) <- depth camera, discovered while driving. Cannot be known in advance.
```

**The depth camera's outdoor job is the local layer only.** The mission line is drawn on
GPS coordinates; the camera decides how to get around the tree that is not on any map.

Two variants:

| | Prior map | Behaviour | Trade-off |
|---|---|---|---|
| **B1 Reactive** | none | Plan straight between waypoints, avoid what it sees | Simple. **Can trap itself** in a concave obstacle / dead end — no global knowledge |
| **B2 Surveyed** | map built on a first pass | Global planner routes around known obstacles | Needs a mapping run first |

**Start with B1.** It is the standard first outdoor capability and needs no mapping.
B2 becomes attractive for a route driven repeatedly.

### Mission execution and return
1. Operator sets waypoints on a geographic map.
2. Rover drives waypoint to waypoint; global plan from GPS, local avoidance from camera.
3. On obstacle: local planner deviates, then rejoins the route.
4. On mission end / low battery / failsafe: **return home**.

⚠️ **Architecture decision to make consciously — who owns the mission?**

| | PX4 owns it | Nav2 owns it |
|---|---|---|
| Mission, geofence, RTL | PX4 native | Nav2 + `navsat_transform` |
| Obstacle avoidance | reflex layer only | full local planning |
| Complexity | lower | higher |
| Matches roadmap O4 | no | **yes** |

Mixing them badly is a classic failure. `roadmap.md` O4 picks Nav2. **Decide before
building outdoor**, because it determines whether RTL is a PX4 behaviour or a Nav2 one.

### Depth camera outdoors — honest limits
* ⛔ **"Range ~3-5 m usable" was WRONG — corrected 2026-09-20 against the DATASHEET.**
  **Orbbec Gemini 336L: depth range 0.17-20 m+, OPTIMAL RANGE 0.25-6 m**, spatial precision
  **<=0.8% @ 2 m and <=1.6% @ 4 m**, depth FOV H90 V65, up to 1280x800@30fps.
  ⚠️ **Those precision figures are quoted at 1280x800. We run 640x360** — a quarter of the pixels,
  so disparity precision at range is worse and **we have not quantified how much.**
  (Live sanity check the same day, from inside the small test room: returns to 7.883 m, p99.9
  6.833 m — consistent with the datasheet, and a tail rather than one stray pixel.)
* 🔑 **THE 5 m LIMIT IS OURS, NOT THE CAMERA'S.** `range_max: 5.0` in `depth_to_scan.launch.py` is a
  deliberate indoor geometry choice: at `cam_z` 0.305 the **floor enters the scan band at 6.25 m**
  and would be published as an obstacle. The comment there says "5.0, not 8.0 ... indoors the extra
  range was never used. **Re-derive if cam_z changes.**" ⏭ **Outdoors that re-derivation is exactly
  the work**, because range_max is what caps safe mission speed.
* ⚠️ Speed arithmetic: at 0.6 m/s a 235 ms perception gap is ~14 cm.
* 92 deg FOV => cannot see a hazard approaching from the side.
* Gemini 336L **is** outdoor-capable (the old "blind in sunlight" note was wrong).

### The companion's role outdoors — DEFINED (2026-09-20)

**Division of labour.** Outdoors the split is clean, and it is the opposite of indoors:
PX4 already solves *where am I* and *where do I go*. It solves **nothing** about *what is in
front of me*.

| Layer | Owner | Why |
|---|---|---|
| Where to go — waypoints, geofence, RTL, battery failsafe | **PX4** | native, GPS-backed, works with the companion switched off |
| Where the vehicle is | **PX4 + GPS** | no map and no SLAM needed outdoors — see the GLOBAL/LOCAL split above |
| **What is in the way** | **COMPANION** | 🔴 PX4 has **no perception at all** on a rover |
| **Whether it is still safe to continue** | **COMPANION** | nothing else is watching |
| Low-level control, arming, kill switch | **PX4** | ⛔ never move any of this to the companion |

#### 🔴🔴 THE GAP: TODAY A MISSION HAS NO OBSTACLE PROTECTION WHATSOEVER

Two independent facts, both read out of the **flashed** firmware source (`a52c38b07d`,
`~/apps/PX4-Autopilot`) on 2026-09-20:

1. **The collision reflex is never called outside AutoNav.** It lives in
   `AutoNavMode::updateSetpoint()`, and `px4_ros2` only calls that while the mode is *active*,
   i.e. `nav_state == 23`. A mission runs as `AUTO_MISSION`. The reflex is not bypassed or
   overridden — **it does not execute at all.**
2. **PX4's own collision prevention cannot cover for it.** `CollisionPrevention` is referenced
   only by `flight_mode_manager/tasks/ManualPosition` and `StickAccelerationXY`. **No rover
   module references it**, and even on a multicopter it is a *manual-mode* feature, not a mission
   one. `obstacle_distance` is inert on this vehicle. (Confirms the standing note in
   `px4_vio_collision`, now verified against source rather than assumed.)

⇒ **Upload a mission today, arm, and the rover drives the line and hits whatever is on it.**
The depth camera will be running, publishing `/scan` at ~29 Hz, and completely ignored.
This is the single most important thing to fix before any outdoor mission runs.

#### The three jobs the companion must do

**1. Environment awareness — perception that runs regardless of flight mode.**
`/scan` and `/scan_3d` already run as their own services, independent of `autonav_mode`, so the
*data* is mode-independent today. What is missing is a **consumer** that is also mode-independent.

**2. The safety envelope — decide, and intervene.**
A supervisor node that watches perception + vehicle state in **every** mode and acts on its own
authority. ⚠️ It must fail safe on perception loss exactly as the reflex does
(`collision.require_scan=true`, `min_valid_fraction` 0.35) — a blind supervisor must stop the
vehicle, not wave it through.

**3. Intervention — the levers that actually exist.**
All of these go out on `/fmu/in/vehicle_command`, which is **already bridged over DDS**; none of
them needs a firmware change.

| Lever | Command | Effect | When |
|---|---|---|---|
| **Pause** | `DO_SET_MODE` → `AUTO_LOITER` | holds; mission resumable | obstacle ahead — the default response |
| **Slow** | `DO_CHANGE_SPEED` | reduces mission speed | degraded perception, rough ground |
| **Abort** | `NAV_RETURN_TO_LAUNCH` | returns home | persistent block, low battery |
| **Hard stop** | `COMPONENT_ARM_DISARM` (force) | motors off | imminent collision, last resort |
| **Detour** | `DO_REPOSITION` | drive to a point | ⛔ not recommended — fights the running mission |

⛔ **`DO_PAUSE_CONTINUE` does NOT exist in this firmware** — verified absent from the whole tree.
Do not design around it.
⛔ **The RC kill (`ch12`) stays the human's, always.** It is not a companion lever and must not
become one.

#### How far the companion should go — pick one, they are a ladder

| | What it does | Can it get *around* an obstacle? | Cost |
|---|---|---|---|
| **S1 Supervisor** | watches, and slows / pauses / aborts | **No** — mission fails safe and stops | Small. One node. **This is the safety floor and must exist regardless of what else is built.** |
| **S2 Handover** | on block, switch to AutoNav (mode 23), let Nav2 drive around, hand back to mission | **Yes** | Moderate — reuses `autonav_mode` + Nav2 exactly as they already work |
| **S3 Nav2 owns the mission** | `navsat_transform`, Nav2 plans GPS-to-GPS | Yes | Large, and duplicates PX4's geofence / RTL / failsafes |

**Recommendation: build S1 first, then S2.** S1 is what makes an outdoor mission *safe*; S2 is what
makes it *useful*. S3 is the "Nav2 owns it" column of the architecture table above — ⚠️ it remains
the open decision recorded there, but note that **S2 gets reactive avoidance without settling it.**

#### What the depth camera specifically buys you outdoors

It is the **entire local layer** — the only sensor on the vehicle that can see an obstacle. Its
measured limits (see *Depth camera outdoors* above) are what set the safety envelope, and they
translate directly into mission constraints:

* **Range caps mission speed — and the cap is OURS, not the sensor's.** Datasheet optimal range is
  **6 m**; `/scan` is clipped to **5.0 m** by `range_max`, chosen so the floor (which enters the
  band at 6.25 m at `cam_z` 0.305) is not read as an obstacle.
  🔑 **Note how close those three numbers sit: clip 5.0, floor intrusion 6.25, optimal range 6.0.**
  So simply raising `range_max` buys ~1 m and then runs out of *optimal* range anyway. ⏭ **The real
  outdoor levers are depth RESOLUTION (640x360 -> 1280x800 restores the datasheet precision, at CPU
  cost) and `cam_z`/pitch (which sets where the floor enters).** Raising the clip alone is not the win.
  Stopping distance must fit inside what the camera can see, perception gap included. Floor braking
  measured 0.19 m at 0.52 s from low speed — ⚠️ **unmeasured at mission speed**, and must be
  re-measured before any speed is signed off.
* **Depth FOV H90° (91.9° measured on our intrinsics) ⇒ it cannot see a hazard approaching from the
  side.** A moving obstacle on a converging course is outside the design envelope. Say so in the
  operating limits; do not pretend otherwise.
* **Gemini 336L is genuinely outdoor-capable** — the old "blind in sunlight" note was wrong.
* ⛔ **Do not quote "/scan does not track beyond ~3 m" as a range limit — it is not one.** That
  note came from a *narrow-target tracking* test, where a corridor subtends only ±4° at 3 m and the
  angular resolution runs out. It says nothing about detecting a wall or a tree. Live `/scan` in the
  small room returned finite beams to **3.628 m** against a 5.0 m clip, with 70% of beams finite.
  🔑 **The real degradation at range is ANGULAR — small objects, not large ones.**

#### The PX4 nav interface — what is actually on the wire

Read out of `dds_topics.yaml` in the **flashed** firmware (`a52c38b07d`) on 2026-09-20:
**32 topics bridged out, 38 in.** ⛔ Anything not in that file needs a firmware change and a
reflash — check the list before designing around a topic.

**What a supervisor can READ today — enough to build S1 with no firmware change:**

| Topic | Gives you |
|---|---|
| `/fmu/out/failsafe_flags` | 🔑 **PX4's own failsafe state.** Don't re-derive what PX4 already decided |
| `/fmu/out/position_setpoint_triplet` | the **active mission waypoint** (previous / current / next) |
| `/fmu/out/vehicle_status` | `nav_state` + arming — ⚠️ the `_v1` naming trap applies |
| `/fmu/out/vehicle_global_position` · `vehicle_local_position` | where PX4 thinks it is, and `eph` |
| `/fmu/out/home_position` | RTL target |
| `/fmu/out/battery_status` · `vehicle_land_detected` | the other two abort triggers |

🔴 **GAP: `mission_result` is NOT bridged** (verified absent). So the companion **cannot read
mission progress, item index, or completion** directly — it must infer them from
`position_setpoint_triplet`. Design S1 around that, or add the topic and reflash.

**What it can COMMAND:** `/fmu/in/vehicle_command` (the intervention levers above), plus the whole
rover setpoint family — `rover_position_setpoint`, `rover_speed_setpoint`, `rover_attitude_setpoint`,
`rover_rate_setpoint`, `rover_throttle_setpoint`, `rover_steering_setpoint`. **All six levels of the
rover stack are addressable over DDS**, so S2 has options beyond the external mode.
`rover_position_setpoint` is the interesting one — `DifferentialPosControl` consumes it and it
carries `start_ned`, `arrival_speed` and `cruising_speed`, i.e. a *position* goal with a speed
profile. ⚠️ It is **mode-gated** — which drive modes route to `PosControl` has not been checked.
Verify before designing on it.

🔑 **`/fmu/in/aux_global_position` is already bridged in** — the path for feeding an external
global position to the EKF. Outdoors that is a fallback/augmentation route if GPS alone proves
insufficient; it needs no firmware change.

#### Gap list — what does not exist yet

- [ ] **Mode-independent safety supervisor (S1).** Nothing watches perception outside AutoNav. 🔴 blocker for any outdoor mission.
- [ ] **GPS.** DroneCAN GPS not fitted (outdoor item O2) — without it there is no mission at all.
- [ ] **Mission-speed braking measurement.** All stopping figures are low-speed; the envelope above is unvalidated at mission speed.
- [ ] **Perception-loss behaviour in mission mode.** Defined for AutoNav, undefined for `AUTO_MISSION`.
- [ ] **Mission progress visibility.** `mission_result` is not bridged, so S1 must infer mission state from `position_setpoint_triplet` — or add the topic and reflash.
- [ ] **Companion-failure behaviour.** R5.5 covers offboard timeout; a *mission* keeps running if the companion dies, because PX4 does not depend on it. That is correct for control and **wrong for safety** — decide what should happen and write it down.

---

---

## 5. OPERATING MODES — what we build and test

Modes are cumulative: each keeps every safety property of the one below.

> 🔑 **M-numbers are the only mode numbering. Do not introduce a third scheme.** This file already
> carries two ladders (M0–M4 here, L0–L5 below) and `roadmap.md` was **archived** precisely because
> its L-numbers clashed with them. Operator goal language maps onto the modes as follows — use the
> M-number in the docs and keep the plain-English name for conversation:
>
> | Operator phrasing | Mode | Needs a map? | Needs localization? |
> |---|---|---|---|
> | "drive without hitting things" | **M1** | no | no |
> | "point and go, it steers around things" | **M2** | no | no |
> | "go to a named place" | **M3** | yes | **yes** |
> | "patrol a route" | **M3** + waypoint list | yes | **yes** |
>
> ⚠️ **A goal list that jumps from "don't hit things" straight to "go to a place on the map" skips
> M2 — and M2 is the cheapest real autonomy we have.** It proves the entire chain with **no map and
> no localization**, i.e. without the one capability that is currently unmeasured. Anything that
> defers M2 behind mapping work is deferring the only autonomy currently within reach.
>
> ⛔ **None of these modes require VIO, and this vehicle has none.** Position is wheel + camera-gyro
> dead reckoning; RTAB-Map consumes it rather than replacing it. See `autonav_reference.md` §6.
>
> ✅ **DOC FAULT CLOSED 2026-09-04 — it was never a duplication.** The 08-10 note read Appendices A
> and B as two rival copies of one ladder. They are not: **Appendix A = what each layer IS and what
> to build; Appendix B = its outcome and definition of done.** They agree layer-for-layer
> (L0 ✅ · L1 🔧 · L2 ❌ · L3 ❌ · L4 ❌ · L5 ❌ optional). **Both are authoritative, for different
> questions.** Nothing to delete.
>
> 🔑 **LADDER OWNERSHIP — settled 2026-09-04. There are three schemes and only these three:**
>
> | Scheme | Answers | Where |
> |---|---|---|
> | **M0-M4** | what the rover DOES for a user — the **goal** ladder | §5 of this file |
> | **L0-L5** | the **capability layers** we must build | Appendices A + B |
> | **R1-R7** | the **requirements** themselves | `rover_autonav_requirements.md` §3 |
>
> 🗄 **RETIRED: the L0-L7 build-order ladder in `rover_autonav_requirements.md` §4.** *That* was the
> third clashing L-scheme (its L5 = "Nav2 local costmap", ours = "goal reasoning"). Its L0-L4 are all
> closed and its L5/L6/L7 are just M2 / M3 / the S+T tests. **Do not plan off it.**
> ⏭ **Live plan and gate order: `memory/todos.md` → "REQUIREMENTS REALIGNMENT 2026-09-04".**

### M0 — MANUAL (baseline)
Operator drives on RC. No autonomy. **State: working.**

### M1 — GUARDED MANUAL
Operator drives on RC; the rover **refuses to hit what it can see ahead**.
* Autonomy: obstacle override only.
* Package `collision_manual_mode` exists, unproven.
* **Purpose: proves the safety floor before anything drives itself.**

### M2 — ASSISTED POINT-AND-GO  *(the depth-camera capability, available now)*
Operator gives a **local goal ahead** ("3 m that way"). Rover drives there, steering
around obstacles. **No map, no SLAM** — both costmaps roll in `odom`.
* Config: `rover_nav2/config/nav2_forward.yaml` (written 2026-08-01).
* Deliberately removed: **spin and back-up recoveries, and reverse** — all three drive
  the rover through space nothing on it can see.
* **Purpose: proves the whole chain** planner -> controller -> `/cmd_vel` ->
  `autonav_mode` -> PX4 -> motors. When the lidar arrives, only the perception source
  changes; everything downstream is already proven.

### M3 — MAPPED PATROL (indoor surveillance = Application A)
Operator selects a location or route **on a saved house map**. Rover localizes, routes,
avoids dynamic obstacles, returns to base.
* Needs: RTAB-Map (A1/A2), 3D voxel layer (A5), waypoint executor (A6), return (A8).
* **Not hardware-blocked** (see 2.2). Blocked on config + CPU headroom.

### M4 — GPS MISSION (outdoor = Application B)
Operator sets GPS waypoints. Rover executes with PX4 safety underneath.
* Needs: DroneCAN GPS, mission ownership decision, geofence, RTL policy.

---

## 6. FAILSAFE POLICY — the layer that must hold in every mode

🔴🔴 **SCOPE WARNING, 2026-09-20: the reflex rows below hold in AutoNav ONLY.** `px4_ros2` calls
`updateSetpoint()` only while `nav_state == 23`, so in `AUTO_MISSION` they are **never executed**,
and no rover module uses PX4's `CollisionPrevention`. This table is therefore **not yet** a
policy for every mode. → §4 *The companion's role outdoors* for the gap and the fix.
✅ **Param values in this table were read live off the FC 2026-09-20 — not from a snapshot.**

| Trigger | Correct response | Owner | State |
|---|---|---|---|
| Operator kill (**RC ch12**) | Motors off | PX4/RC | ✅ `RC_MAP_KILL_SW`=12 read live. Pressed in **armed AutoNav** 09-19, rover stopped — ⚠️ not a formal pass (no latency/tape). ⛔ it is **ch12, never ch8** |
| Obstacle inside stop distance | Block forward, cap yaw | our reflex, in-executor | ✅ proven armed — 🔴 **AutoNav only** |
| `/scan` stale > 0.5 s | Block forward | our reflex | ✅ built — 🔴 **AutoNav only** |
| `/cmd_vel` stale > 0.5 s | Zero setpoint | `autonav_mode` | ✅ proven — 🔴 **AutoNav only** |
| **Localization lost / degraded** | **Stop and hold** | — | ❌ **does not exist** |
| **No route to goal** | **Stop + notify, or return home** | — | ❌ **does not exist** |
| RC link lost | Hold or return home | PX4 `NAV_RCL_ACT` | ✅ **=1, HOLD** (read live 09-20) |
| Battery low | Return home | PX4 `COM_LOW_BAT_ACT` | ❌ **=0, warning only** (read live 09-20) |
| Geofence breach | Hold / RTL | PX4 `GF_ACTION` | ❌ **=0, no action** (read live 09-20) — outdoor prerequisite |

⛔ **CORRECTED 2026-09-20 — the previous text here claimed `NAV_RCL_ACT = 6` (Disarm on RC loss).
It reads `1` (Hold) on the FC.** The doc was stale; RC loss does **not** disarm. Hold is a sane
rover default, but ⚠️ it is still **not** "return home" — losing RC mid-patrol parks the rover
where it stands. Moving it to `2` (Return) is an outdoor-mission decision for M3/M4, and needs
GPS + a configured home first.
⚠️ Two more rows are unconfigured and are **outdoor blockers, not preferences**:
`COM_LOW_BAT_ACT`=0 means a flat battery produces a warning and nothing else, and `GF_ACTION`=0
means the geofence does nothing at all. 🔑 **Read these live before every mission — ⛔ never quote
this table as authority, it went stale once already.**

---

## 7. TEST DEFINITION — pass criteria, not opinions

Safety tests gate capability tests. **S-tests first.**

| ID | Test | Pass criterion |
|---|---|---|
| **S1** | **Kill switch in AutoNav** (armed, 0.15 m/s, hit **ch12**) | Wheels stop immediately. ⚠️ **PX4 kill ≠ disarm — settle which you mean before re-running.** Pressed in armed AutoNav 09-19 and the rover stopped, but with no latency or tape measurement, so **S1 is INCONCLUSIVE, not passed.** ⛔ the channel is **ch12** (`RC_MAP_KILL_SW`=12, read live) — every "ch8" in the older docs is disputed, see `px4_param_audit.md` §P5 |
| **S2** | Sensor loss (stop `rover-scan` while driving) | Forward blocked within 0.5 s |
| **S3** | Yaw loop diagnosis (outdoor, `--yaw 0.2` and `0.4`) | `steering/setpoint` ~0.102 = OPEN loop (no gain fixes it) / ~0.052 = CLOSED (tuning job) |
| **T1** | Speed tracking, 5 s leg at 0.2 m/s | **Sustained** `/odom` within +/-20% of command |
| **T2** | M2 straight goal, clear corridor 2 m | Arrives within 0.20 m, collision-stop never fires |
| **T3** | M2 with one offset obstacle | Routes around, keeps inflation clearance, arrives |
| **T4** | M2 fully blocked corridor | Stops cleanly, reports failure, **does NOT spin or reverse** |
| **T5** | Dynamic obstacle (step into its path) | Stops before contact. Tells us if 22 Hz + 0.55 m inflation is enough margin |
| **T6** | M3 map build + re-localize | Map covers the area; after restart, pose recovered without operator input |
| **T7** | M3 return to base | Routes home from an arbitrary point on the map |

```
S1 kill ─▶ S2 sensor loss ─▶ T1 speed ─▶ T2 straight goal
                                             │
     S3 yaw (outdoor) ───────────────────────┴─▶ T3 avoid ─▶ T4 blocked ─▶ T5 dynamic ─▶ T6/T7 mapped
```

T3 onward all require turning, so **S3 gates them**. T1/T2 are straight-line and can run
indoors as soon as the workspace build lands.

---

## 8. Critical path

```
S1+S2 safety ─▶ T1/T2 prove M2 ─▶ enable point cloud + voxel layer (A5)
                                        │
                                        ├─▶ CPU headroom measurement  ─▶ RTAB-Map ─▶ M3 (indoor surveillance)
                                        │
                                        └─▶ S3 yaw ─▶ DroneCAN GPS ─▶ mission-ownership decision ─▶ M4 (outdoor)
```

**Immediate blockers, in order:**
1. ~~S1 kill switch in AutoNav~~ — ✅ **PASSED** 2026-07-22, re-confirmed 2026-08-10 (<20 ms).
   **Now blocking instead: validate the collision reflex's perception-health gate, then S2.** The
   reflex failed open on a fresh-but-empty scan and caused a wall contact on 2026-08-10; the fix is
   written but only proven synthetically. See `autonav_reference.md` §10.
2. S3 yaw loop — unknown open/closed; gates every turning test.
3. Point cloud + voxel layer — unlocks the camera's actual capability.
4. Failsafe policy decisions (`NAV_RCL_ACT`, lost-localization, no-route) — design, no hardware.

---

# APPENDIX A — The autonomy layers, concretely

Added 2026-08-01 after a fair challenge: "nowhere do I see any autonomy, these are
all existing features, the only question is where we run the mission."

That is correct, and it is the most important framing in this document.
**Waypoint following and threshold-based obstacle stopping are AUTOMATION, not
autonomy.** Every one of them is a fixed response to a fixed input. A vehicle that
follows waypoints and stops for obstacles is a train with a bumper.

The layers below separate what we CONFIGURE (L0/L1, mostly existing software) from
what we must WRITE (L2-L5, which does not exist off the shelf).

---

## L0 — PLUMBING ✅ done
**"The rover does what it is told."**

| Component | Status |
|---|---|
| `rover_odometry`: ERPM -> `/odom` + TF | ✅ (scale fixed 2026-08-01) |
| `rover_ekf_bridge`: `/odom` -> EKF2 EV velocity | ✅ |
| `depthimage_to_laserscan`: depth -> `/scan` | ✅ (but see 2.1 — wasteful) |
| `autonav_mode`: `/cmd_vel` -> PX4 rover setpoints | ✅ |
| Arm workflow, kill switch, watchdogs | ✅ (kill untested in AutoNav) |

**Contract:** give it a velocity, it drives. Nothing decides anything.

---

## L1 — AUTOMATION 🔧 in progress
**"The rover goes where it is told, without hitting what it can see."**

| Build | New? |
|---|---|
| Nav2 costmaps + planner + controller (`rover_nav2`) | config, written |
| **Point cloud + `voxel_layer`** — stop flattening 3D to 2D | **config, TODO** |
| Waypoint sequencer — drive a LIST of goals, not one | **small new node** |
| Collision reflex inside the executor | ✅ exists |

**Inputs:** goal pose. **Outputs:** `/cmd_vel`. **Decisions made: none.**
**Test:** T1-T5 in section 7.

⚠️ **Finishing L1 does not give an autonomous rover.** It gives a remotely-commanded
rover that does not crash. Its value is that L2-L5 have nothing to stand on without it.

---

## L2 — SELF-AWARENESS ❌ not started — *mandatory for unattended operation*
**"The rover knows when it does not know."**

Today the rover drives with identical conviction whether it is perfectly localized or
completely lost. That is the difference between a demo and something you leave running.

**Build: a health/confidence monitor node publishing `/rover_health`.**

| Watches | Signal | Why (learned the hard way) |
|---|---|---|
| Localization | `eph`, `xy_valid`, `dead_reckoning`, SLAM covariance | `eph` grew to 682 m on 07-28 and nothing noticed |
| Odometry sanity | commanded vs achieved velocity; `esc_online_flags` | ESC doze silenced `/odom`; scale was 12.2x wrong for weeks |
| Perception | `/scan` rate, sector coverage %, empty-scan count, staleness | 79.7% coverage looked fine, was aimed at open room |
| Compute | CPU load, per-node liveness, topic rates | CPU starvation latched ffmpeg silently for >1 h |
| Actuation | does commanded motion produce expected motion | yaw ran 21x command and only a human noticed |
| Power | battery voltage, ESC current | — |

**Output:** per-subsystem status + one overall level — `OK / DEGRADED / UNSAFE` — **with
a reason string**. Never a bare boolean.

**Test:** inject each fault deliberately (stop `rover-scan`, unplug camera, load the
CPU, block a wheel) and assert the correct classification and reason.

**Note:** every entry in that table is a real failure this project already hit and
diagnosed by hand. L2 is the layer that would have caught them automatically.

---

## L3 — DECISION LAYER ❌ not started — *mandatory*
**"The rover decides what to do when the plan fails."**

Nav2 has no answer for "blocked, now what". Its recoveries are spin and back-up —
mechanical twitches, both disabled here because they are blind on a 92 deg sensor.

**Build: a supervisor node above Nav2** (a state machine / behaviour tree). It owns
the mission and consumes `/rover_health` + Nav2 result codes.

| Situation | Decision to make |
|---|---|
| Goal reached | next waypoint |
| No valid path | retry / skip this leg / return / hold |
| Blocked > N s | wait (person passing) -> reroute -> abort |
| Health DEGRADED | slow down, drop video bitrate, shorten goals |
| Health UNSAFE | stop, hold, notify |
| **Localization lost** | **stop immediately — never drive blind** |
| Battery low | return to base |
| Mission complete | return, dock, report |

**This is where "safe return to home" actually lives.** It is not a PX4 feature indoors:
PX4 RTL drives to a GPS home, and on a rover it drives there with NO obstacle avoidance.

**Test:** force each row, assert the chosen action.

---

## L4 — SEMANTICS ❌ not started — *this IS the surveillance product*
**"The rover understands what it sees, not just that something is there."**

Today every obstacle is an identical lethal costmap cell. For surveillance that is
exactly wrong: a chair is furniture, a person is the event we exist to detect, an open
door is a state change. **An occupancy grid cannot express any of that.**

**Build:**
1. Detector on the RGB stream (YOLOv8n) -> labelled 2D detections
2. Project detections into 3D using the depth image -> labelled obstacles
3. Semantic costmap layer — e.g. keep a larger berth around people than around walls
4. **Event generation** — "person detected, room X, time T" + snapshot. This is the
   actual output of a surveillance rover.

⚠️ **CPU is the blocker.** YOLOv8n on Pi 5 CPU is roughly 1-3 fps on 4 cores that are
already oversubscribed. Options: run detection only while stopped, drop resolution, or
add an accelerator (Hailo / Coral). **Measure before designing around it.**

---

## L5 — GOAL REASONING ❌ optional for now
**"The rover turns a goal into behaviour."**

"Patrol the house" -> which rooms, what order, how long to dwell, what about the closed
door, when to come back. **Today the operator decomposes this and hands over waypoints.
When the rover decomposes it, that is autonomy.**

Also: memory across runs — the sofa moved, this corridor is usually blocked, *this is
different from yesterday*. For surveillance, noticing the difference IS the product.

Candidate: the existing Ollama/phi3 stack for natural-language mission spec. Lowest
priority — a hand-written route is a perfectly good substitute until L2-L4 are solid.

---

## What this changes about priorities

```
L0 ✅ ── L1 🔧 finish quickly, it is the foundation ── ▶ DEMO CEILING
                                                        │
                                    L2 self-awareness ──┤ mandatory unattended
                                    L3 decisions       ─┤ mandatory unattended
                                    L4 semantics       ─┘ the product
```

The real project is **L2 + L3 + L4**. Nav2 and PX4 are the actuation layer underneath
them, not the goal. Time spent perfecting Nav2 tuning past "good enough" is time not
spent on the layers that make the rover actually autonomous.

---

# APPENDIX B — Outcome of every stage

What you can DO at the end of each layer, how you know it is finished, and — equally
important — what you still cannot do. The last column exists to stop us claiming a
capability we have not built.

**The milestone that matters is not "autonomous". It is UNATTENDED.**
A rover you must watch is a demo regardless of how it navigates. Unattended operation
requires L1 + L2 + L3 together, and no single one of them is sufficient.

---

## L0 — PLUMBING ✅ ACHIEVED 2026-08-01

**Outcome:** *"Tell the rover a velocity and it executes it, and the telemetry describing
what it did is TRUE."*

**Definition of done**
- [x] `/odom` publishes continuously, including at rest (ESC doze handled)
- [x] Odometry scale verified against ground truth by two independent methods
- [x] Commanded velocity produces the expected wheel response, armed
- [x] `/cmd_vel` and `/scan` watchdogs proven to zero the motors
- [x] Arm workflow repeatable (RC arm in Manual -> software switch to AutoNav)

**Still cannot:** go anywhere by itself. Every metre is commanded by a human or script.

**Why this mattered more than it looked:** the odometry scale was wrong by 12.2x for
weeks. Every speed limit, every collision margin and every controller gain computed on
top of it was meaningless. L0 is not plumbing you can skip — it is the layer that makes
every number above it real.

---

## L1 — AUTOMATION 🔧 IN PROGRESS

**Outcome:** *"Give the rover a goal or a route, and it drives there without hitting
anything it can see, while an operator watches."*

**Definition of done**
- [ ] T1 speed tracking — sustained `/odom` within +/-20% of command
- [ ] T2 straight goal, clear corridor — arrives within 0.20 m, reflex never fires
- [ ] T3 single offset obstacle — routes around it, keeps inflation clearance
- [ ] T4 fully blocked — stops cleanly, reports failure, does NOT spin or reverse
- [ ] T5 dynamic obstacle — stops before contact
- [ ] Point cloud + `voxel_layer` live — detects a table top, not just table legs
- [ ] Waypoint sequencer drives a LIST of goals

**Still cannot:** recover from a blocked path, notice that it is lost, or be left alone.

**Boundary to be honest about:** this is the demo ceiling. A rover that follows waypoints
and stops for obstacles is a train with a bumper. Finish it because L2-L5 need it, not
because it is the goal.

---

## L2 — SELF-AWARENESS ❌ NOT STARTED

**Outcome:** *"At any instant the rover states how much it should be trusted, and why.
Every subsystem failure this project has hit by hand is now caught automatically."*

**Definition of done** — each fault injected deliberately, classified correctly, with the
right reason string, within 2 s:
- [ ] Stop `rover-scan` -> `UNSAFE: perception stale`
- [ ] Aim camera at a blank/glossy wall -> `DEGRADED: sector coverage 30%`
- [ ] Load all 4 cores -> `DEGRADED: control loop starved`
- [ ] Let `eph` grow past the gate -> `UNSAFE: localization diverged`
- [ ] Block a wheel -> `DEGRADED: commanded/achieved mismatch`
- [ ] Let ESCs doze -> reported as at-rest, NOT as a fault
- [ ] Battery below threshold -> `DEGRADED: battery low`
- [ ] `/rover_health` never emits a bare boolean — always level + reason

**Still cannot:** act on any of it. L2 only reports; deciding is L3.

**Test method:** faults must be INJECTED, not waited for. A monitor validated only by
normal operation is untested.

---

## L3 — DECISION LAYER ❌ NOT STARTED

**Outcome:** *"The rover completes a multi-waypoint mission, or abandons it safely and
comes home, WITHOUT a human in the loop."*

**This is the unattended milestone.** It is the first stage whose outcome is a change in
what the operator has to do, rather than a change in what the rover can do.

**Definition of done** — a full patrol run with induced failures, no intervention:
- [ ] Reaches every reachable waypoint in order
- [ ] Path blocked by a person -> waits, then reroutes, then skips the leg
- [ ] Permanently blocked leg -> skips it and continues the mission
- [ ] Health DEGRADED -> reduces speed / shortens goals and continues
- [ ] Health UNSAFE -> stops and holds, does not keep driving
- [ ] Localization lost -> stops IMMEDIATELY, never drives blind
- [ ] Battery low -> abandons mission and returns to base
- [ ] Mission end -> returns to base unaided
- [ ] Runs to completion with the operator out of the room

**Still cannot:** tell a person from a chair. Every obstacle is still an anonymous
lethal cell.

**Note:** "return to base" is built HERE, not inherited from PX4. PX4 RTL targets a GPS
home and, on this rover, drives there with no obstacle avoidance whatsoever.

---

## L4 — SEMANTICS ❌ NOT STARTED

**Outcome:** *"The rover reports WHAT it saw, WHERE and WHEN — and treats a person
differently from furniture."*

For indoor surveillance this outcome IS the product. L1-L3 deliver a rover that patrols
safely; L4 is the first stage that produces something a user actually wants to read.

**Definition of done**
- [ ] Detector runs on the live RGB stream at a measured, stated frame rate
- [ ] Detections projected into 3D and placed correctly in the costmap
- [ ] Person-aware costmap — larger berth around people than around walls, verified
- [ ] Events emitted as `{label, room/pose, timestamp, snapshot}`
- [ ] Precision/recall measured on a held-out set — stated as numbers, not "works"
- [ ] CPU cost measured; control-loop rates confirmed unaffected

**Still cannot:** decide mission strategy for itself.

**Known risk:** YOLOv8n is roughly 1-3 fps on 4 already-oversubscribed cores. If the
measurement says the budget is not there, the honest outcomes are: run detection only
while stopped, or add an accelerator. Do not silently starve the control loop for it.

---

## L5 — GOAL REASONING ❌ OPTIONAL

**Outcome:** *"Give the rover a goal in human terms and it decides the behaviour —
including noticing when today is different from yesterday."*

**Definition of done**
- [ ] A goal such as "patrol the ground floor" decomposes into a sensible route unaided
- [ ] Unreachable room -> reorders or defers rather than failing the mission
- [ ] Map persists across runs; changes are detected and reported
- [ ] Novelty reported ("this door was closed yesterday")

**Still cannot:** anything outside the physical envelope L0-L1 provides — a smarter brain
does not extend the sensor's 92 deg or make a blind reverse safe.

---

## Summary — what each stage buys

| Layer | The one-line outcome | Operator must... |
|---|---|---|
| **L0** ✅ | Telemetry is true, velocity commands execute | drive it |
| **L1** 🔧 | Goes to a goal without hitting visible obstacles | watch it |
| **L2** | States its own trustworthiness, with reasons | watch it, but now informed |
| **L3** | **Completes or safely abandons a mission alone** | **leave the room** |
| **L4** | Reports what it saw, where and when | read the report |
| **L5** | Turns a stated goal into behaviour | state a goal |

**The step-change is L3.** Everything before it produces a better-behaved remote-control
vehicle. L3 is where the operator stops being required.
