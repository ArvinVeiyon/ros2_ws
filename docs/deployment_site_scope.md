# Deployment scope — the site, and what the venue changes

> Written 2026-09-20 from an operator scoping decision. **Additive only.**
> ⛔ **Changes no goal, no ladder, no feature list.** Application A (A1–A9) and Application B stay
> exactly as written in `autonomy_plan.md` §3 and §4. This file records **where the rover actually
> deploys** and **what that venue changes about risk** — nothing else.

---

## 1. The decision

* ✅ **The product is Case B — a mission executed on a PREVIOUSLY BUILT MAP.**
  (Case A, a mission relative to the start point, is *not* the product — see
  `px4_companion_interface.md` §9.3.)
* ⇒ 🔴 **Relocalization is on the critical path and there is no way around it.**
  The Case A shortcut does not apply. The 0-accepted-of-20 failure is **the** blocker.
  → `indoor_mapping_slam` §17.
* ✅ **"Indoor" does NOT mean "inside a house".** The venue is a **bounded site**:
  * a **yard**, or
  * a **manufacturing unit / warehouse**,
  * possibly both, with transitions between them.
* ⚠️ **GPS may or may not be present**, and may be present *intermittently* within one site.

## 2. What this does NOT change

* **Application A's shape is already correct** — `autonomy_plan.md` §3: *"picks a patrol route on a
  saved map, presses go"*, *"repeat-the-same-route in a semi-static environment"*, *"map once and
  re-localize"*. That is exactly this product.
* **A1–A9 remain the right feature list.** ⛔ Do not renumber or rewrite them.
* ⇒ 🔑 **The only thing wrong in the existing docs is the VENUE ASSUMPTION**, not the plan:
  §3 is written as a house, §4 as open outdoor GPS. **The product is A's shape at site scale.**

## 3. GPS is an AID, not the architecture

* ⛔ **It cannot be the primary position source** — it may be absent, and inside a unit it will be.
* ⛔ **Its absence cannot be assumed either** — in a yard it may be good.
* ⇒ **Map-relative localization is primary. GPS is optional and additive**, useful to:
  * initialize / disambiguate which part of the site the rover is in,
  * bound drift on long outdoor legs,
  * provide a home position for a return that PX4 itself understands.
* **Plumbing already exists:** `/fmu/in/aux_global_position` (`dds_topics.yaml:205`), with
  `EKF2_AGP_CTRL` currently **0** (read live 2026-09-20).
* ⚠️ **Mixing sources needs care.** Map-relative pose and GPS must agree on a frame, or the EKF is
  fed two conflicting truths. ⛔ Not a config exercise — design it before enabling both.
* 🔑 **A8 "return to base" already flags this** in §3: PX4 RTL drives to a **GPS** home, so on a
  site without GPS, returning home means **routing home on our own map**.

## 4. 🔴 What the venue makes WORSE — site-specific risk

Each of these is harder in a yard or a plant than in a house. None is a new requirement; all change
the weight of an existing one.

* 🔴🔴 **Negative obstacles become critical, and we are blind to them.**
  * Yards have **potholes, drains, gratings, kerbs, loading-dock edges, ramps**. A house floor is flat.
  * ⛔ **The current pipeline cannot see any of them.** `/scan_3d` filters to a band **0.12–0.45 m**
    above the frame, so points *below* ground are discarded; `/scan` takes a thin slab where a hole
    reads as **clear road**.
  * ⚠️ `autonomy_plan.md` §2 scores the depth camera ✅ for "stairs / drop-off" — that is the
    **sensor**, not our **pipeline**.
  * ⇒ **Scope decision required: in, with a real design — or out, with the blindness written into
    the operating limits.** ⛔ It must not stay silently absent.
* 🔴 **Dynamic obstacles are VEHICLES, not people.**
  * Forklifts and trucks are fast, heavy, and **approach from the side**.
  * ⛔ Depth FOV is **H90° forward only** — nothing to the sides or behind (`px4_companion_interface.md` §4).
  * ⇒ this is a **hard envelope limit**, not a tuning problem. Either accept it as an operating
    constraint (segregated route, speed limit) or add sensing. → A4 is marked "✅ forward only".
* ⛔ **CORRECTED 2026-09-20 (operator).** An earlier draft said *"scale multiplies drift"*. **Wrong.**
  🔑 **Relocalization is an ABSOLUTE fix, not an increment.** Each successful match resets error to
  the match accuracy; it does **not** accumulate across matches. That is how the stack is already
  built — our `/odom` is the incremental part, RTAB-Map supplies the `map→odom` correction on top.
  * ⇒ **Error model: `match accuracy` + `drift since the LAST SUCCESSFUL match`. Bounded, not growing.**
  * ⇒ ⛔ **Site size on its own multiplies nothing.** A large site with good map coverage throughout
    is fine; a small site with one featureless stretch is not.
* ⚠️ **So the real variable is MATCH RATE AND COVERAGE, not area.** The question is never "how big
  is the site" but **"what fraction of the route can be matched, and how long is the longest gap?"**
  * Size the odometry budget against **the longest unmatched stretch**, not the route length.
* 🔴 **Perceptual aliasing is the repetitive-site risk — and it is worse than no match.** Identical
  racking aisles and repeated bays can produce a **confident match on the WRONG place**. A missed
  match degrades gracefully into odometry; a false match puts the rover somewhere it is not.
  ⛔ Do not treat "relocalization succeeded" as self-validating — gate on match quality.
* ⚠️ **The correction JUMP is itself an operational hazard.** An absolute fix after a long gap
  arrives as a **pose discontinuity**: `map→odom` steps, and the global costmap and plan step with
  it. ⚠️ Unmeasured here. Decide what the controller should do during a large jump **before**
  running a site route.
* ⚠️ **Featureless expanses** — flat concrete aprons, blank walls — are where the gaps come from.
  ⛔ Do not assume a bigger map is an easier map, but the reason is **coverage**, not accumulation.
* ⚠️ **Lighting transitions.** Driving through a roller door is a large, fast exposure swing —
  a known visual-odometry failure mode. The house case never exercised it.
* ⚠️ **Weather and surface.** Wet concrete, gravel, standing water. The Orbbec store listing
  describes the 336L as **IP65** — ⛔ confirm against the datasheet before relying on it outdoors
  in rain. Traction and braking on loose or wet surfaces are **unmeasured**; every stopping figure
  we have is from a dry indoor floor.

## 5. What to do with this

* ⏭ **No plan changes proposed here.** The next decisions, in order:
  1. **Relocalization** — unchanged as the blocker, now confirmed unavoidable (§1).
  2. **Negative obstacles** — take the scope decision (§4).
  3. **Side sensing** — accept the forward-only envelope in writing, or change the sensor set.
  4. **GPS role** — decide aid-vs-absent before wiring `aux_global_position`.
* 🔑 **Survey the actual site before designing further.** Surface, lighting, traffic and GPS quality
  are all site facts, and every one of them is currently assumed.
* 🔑 **The survey's localization question is COVERAGE, not size:** walk the intended route and ask
  *where can it match, and how long is the longest unmatched stretch?* That number — not the site
  area — sizes the odometry budget and decides whether the route is viable as drawn.

## References

* Application A shape and A1–A9: `autonomy_plan.md` §3
* Outdoor GPS mission and the companion's role: `autonomy_plan.md` §4
* Case A / Case B, `EKF2_EV_CTRL`, position injection: `px4_companion_interface.md` §9
* Relocalization failure: `memory/project_indoor_mapping_slam.md` §17
* Sensor envelope and FOV: `px4_companion_interface.md` §4, `rover_geometry.md`

---

## 6. Aligning the current work — cross-checked against comparable systems

> Added 2026-09-20 after an operator challenge: *"from the beginning I told you do not rely on
> odometry."* The record supports that. This section records the cross-check and the realignment.
> ⛔ Still additive — no goal is deleted or renumbered here.

### 6.1 What comparable systems actually do

* **Warehouse / site AMRs localize on a 2D LiDAR**, matching live scans against an occupancy grid.
  Wheel odometry is fused in as the **motion prior between scans** — ⛔ **not as the position source**.
* **Depth cameras are used for the obstacle class LiDAR misses** — forklift tines, overhanging or
  cantilevered loads, debris lying flat on the floor.
* ⇒ 🔑 **Industry split: LiDAR = localization + 360° safety. Depth camera = 3D forward obstacles.**
* ✅ **Our own `autonomy_plan.md` §2 already says exactly this** — *"Lidar = safe while
  turning/reversing, and robust geometric SLAM"*. The plan was right; the hardware did not follow.

### 6.2 What the record shows

* 🔴 **The STL-19 is NOT FITTED — it was allocated to the drone** (`autonav_reference.md:85`).
* 🔴 `autonav_reference.md:317`: the depth camera covers a 92° forward wedge; **"the other 268° is
  unmeasurable, permanently, until a 360° lidar is fitted. That single fact drives most of §11."**
* ⇒ **The odometry-heavy architecture is a CONSEQUENCE of the missing LiDAR, not a design choice
  that was argued for on merit.** The operator's original instruction — do not rely on odometry —
  matches standard practice. It could not be followed because the sensor went to the other vehicle.

### 6.3 What of the odometry work was, and was not, wasted

* ✅ **NOT wasted — odometry is required regardless.** In the corrected error model (§4) odometry is
  precisely what carries the estimate **between absolute fixes**, and that is the role industry
  fuses it in. A calibrated `erpm_to_ms` and the speed-dependent error curve are needed either way.
  ⛔ **Do not re-open the scale.** → `rover_odometry`
* ⚠️ **Limited transfer — the odom-frame Nav2 tuning.** DWB critic weights tuned with
  `global_frame: odom` and no map will need re-tuning once a `map` frame exists. Finish T3 to
  *good enough*, then stop; ⛔ do not polish it.
* 🔴 **A real design problem, worth naming:** `rover-ekf-bridge` feeds the EKF **from `/odom`**, so
  the controller regulates against its own under-read — **circular feedback sitting in the safety
  path**. → `autonav_reference.md` §10. ⛔ This is not fixed by better odometry calibration.

### 6.4 The realignment

* 🔑 **The product is Case B on a site (§1). Its blocker is localization, and the industry answer
  for this exact venue is a 2D LiDAR.**
* 🔴 **Visual relocalization is failing 0 of 20 on the map's own bag, at the GEOMETRY stage.**
  ⛔ Continuing to tune it is optimizing the path the industry does **not** take for this venue.
* ⇒ **Highest-leverage move: fit a 360° LiDAR** — recover the STL-19 or buy a second unit.
  It addresses **four** open problems at once:
  * **localization** — geometric scan matching, robust on concrete and racking where visual
    features are sparse or repetitive (§4);
  * **the 268° blind arc** — the side-approaching forklift, currently an envelope limit (§4);
  * **reverse and pivot safety** — the recoveries deliberately removed from M2;
  * **the "lidar owns `/scan`" path is already designed** — `setup_manual.md` §269 / §F5, with
    `depth_to_scan` remapping to `/scan_depth`.
* ✅ **The depth camera then moves to its correct role** — 3D forward obstacles, the class LiDAR
  misses, which is what `autonomy_plan.md` §2 always said it was for.
* ⚠️ **It does not solve everything.** A 2D LiDAR still cannot see **negative obstacles** (§4) —
  potholes and dock edges remain an open scope decision either way.

### 6.5 Immediate implication for what is on the bench

* **T3 (`ObstacleFootprint.scale`)** — finish to a working value, take the 3 runs, **then stop.**
  It is M2 local avoidance, genuinely reusable as a *chain*; its odom-frame *tuning* is not.
* ⛔ **Do not start new work that assumes odometry is the position source.**
* ⏭ **Next decision is a hardware one, not a software one:** LiDAR, yes or no. Everything in §6.4
  waits on it.
