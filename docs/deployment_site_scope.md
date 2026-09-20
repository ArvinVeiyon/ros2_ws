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
* ✅ **SEQUENCING DECIDED 2026-09-21: INDOOR ("internal mode") IS THE FIRST TARGET.**
  The manufacturing unit / inside comes first; the yard and outdoor GPS work follow.
  🔑 This **agrees with what is already written** — `autonomy_plan.md` §3 is titled
  *"Application A — INDOOR SURVEILLANCE (first target)"*. ⛔ Nothing is reordered; this only makes
  the venue explicit (a unit, not a house) and confirms the order.

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

### 6.4 ✅ ARCHITECTURE DECIDED 2026-09-20 (operator) — camera localizes, LiDAR guards

* ✅ **The DEPTH CAMERA is the localization sensor**, feeding **both the local and the global planner.**
* ✅ **A LiDAR is NOT the primary localization sensor.** Its jobs are **collision and obstacle
  avoidance** and the **268° blind arc**.
* ⛔ **This overrides the LiDAR-primary suggestion previously drafted here.** Do not re-propose it.
* ⚠️ **REFINED 2026-09-21 — "not primary" is not the same as "excluded".** RTAB-Map is a **LiDAR
  *and* visual SLAM library**, and the literature's reason for scoring it above AMCL is precisely
  that it **fuses both**. So a fitted LiDAR may contribute **geometric constraint** indoors without
  displacing the camera. 🔑 **Fusion, not replacement.** ⛔ Do not read this as re-opening the
  primary-sensor decision — the camera stays primary.

**Why this is reasoned, not a preference — record it so it is not re-litigated:**

* **A 2D LiDAR is a single horizontal slice, and a yard is mostly open.** Scan matching needs
  vertical structure to bite on; an open apron offers little, so 2D geometry can be **degenerate
  outdoors** in exactly the venue we are targeting (§1).
* **One sensor spans the whole site.** The camera works indoors *and* outdoors and through the
  transition; a 2D LiDAR degrades outdoors — sunlight, rain, and no walls to match.
* **It is the sensor we actually have, characterised.** Gemini 336L is fitted, calibrated and
  G0-closed; the STL-19 is **not fitted and is allocated to the drone** (§6.2).
* **Visual SLAM for AMRs is industrially deployed**, not experimental.
* 🔑 **This split is BETTER on safety than LiDAR-primary would have been.** Putting the LiDAR on
  collision and obstacle duty directly closes the **268° blind arc** and the side-approaching
  forklift (§4) — the risk a LiDAR-primary architecture would have left open by using it for
  localization instead.

### 6.5 What the decision does NOT change

* 🔴🔴 **Relocalization failing 0 of 20 is now THE critical path, not a side path.**
  Choosing camera-primary **commits to fixing it** — the blocker does not move, it becomes central.
  It fails at **geometry, not appearance**. → `indoor_mapping_slam` §17.
* ⛔ **Negative obstacles are still unsolved and unassigned** (§4). Neither sensor as configured
  sees a pothole; the `/scan_3d` band discards sub-ground points.
* 🔴 **Circular feedback stays** — `rover-ekf-bridge` feeds the EKF from `/odom` (§6.3).

### 6.6 Camera-primary: the known failure modes, and the practical mitigation

⚠️ These are design inputs, not objections. Each has a standard answer.

| Risk at a site | Why it bites | Mitigation |
|---|---|---|
| **Featureless concrete apron** | too few visual features to match | **fiducial markers** (below) |
| **Repetitive racking / identical bays** | perceptual aliasing — a confident match on the WRONG bay (§4) | fiducials carry a **unique ID**, which removes the ambiguity outright |
| **Roller-door lighting transition** | large fast exposure swing, a known VO failure | exposure strategy; validate the transition explicitly |
| **Direct sun / glare / darkness** | stereo degrades | 336L is IR-pass; ⛔ still verify on site |

* 🔑 **FIDUCIAL MARKERS (AprilTag) ARE THE PROVEN ANSWER** for visual localization in feature-poor
  or repetitive sites — cheap to print and affix, and documented to give **more robust tracking and
  relocalization** than markerless SLAM. They fix the two worst risks above at once.
* ✅ Favourable for us: indoor sites have **stable artificial lighting** and rarely need long-range
  perception — both work in a camera's favour.
* ⚠️ **Placement is a real design task, not decoration.** A planar marker has one surface normal and
  is only readable within a limited cone about it — a rack-facing tag seen edge-on down an aisle is
  useless. Plan coverage along the **actual route**, tied to the §5 survey.

### 6.7 Immediate implication for what is on the bench

* **T3 (`ObstacleFootprint.scale`)** — finish to a working value, take the 3 runs, **then stop.**
  The avoidance *chain* is reusable; its odom-frame *tuning* is not.
* ⛔ **Start no new work that assumes odometry is the position source.**
* ⏭ **Next work is relocalization**, now on the primary path by decision. Before more tuning:
  1. **Why does it fail at geometry?** 0/20 on the map's own bag is a pipeline fault, not a map fault.
  2. **Decide on fiducials** — they change what "good relocalization" has to achieve unaided.
  3. **Survey the route for coverage** (§5) — longest unmatched stretch sizes the odometry budget.
* ⏭ **LiDAR is now a SAFETY question, not a localization one** — worth fitting for the 268° arc and
  reverse/pivot safety, on its own merits and its own timeline.


---

## 7. Settled 2026-09-21 — the questions that kept coming back

Each of these was argued out and resolved. ⛔ Recorded so they are not re-opened from scratch.

### 7.1 Planning is not localization

* **Localization** answers *"where am I?"* → it **produces** a pose. GPS, VIO, scan matching, visual relocalization.
* **Planning**, local and global, answers *"how do I get there without hitting things?"* → it
  **consumes** a pose and produces a path or a velocity.
* 🔑 **PX4's own avoidance stack is evidence for the split, not against it.** `local_planner` is a
  **VFH+\* vector-field-histogram** planner; `global_planner` is a **graph planner over an octomap**;
  the depth camera supplies the **obstacle information** to both. And PX4 states plainly of the
  global planner: *"For the map to be good enough for navigation, **accurate global position and
  heading are required**."* — it requires pose as an **input**.
* ⇒ ✅ **"Depth camera feeds both the local and global planner" is CORRECT** and is what our Nav2
  already does. ⛔ It is not a claim that the camera localizes. On this rover the localization job
  belongs to **RTAB-Map**, which is the component returning 0 of 20.
* ⚠️ `PX4-Avoidance` is a **multicopter** package and no rover module consumes `obstacle_distance`
  — ⛔ not transplantable to this vehicle.

### 7.2 VIO replaces ENCODERS, not GPS

* **VIO** = relative motion, drifting, *"relative to a local starting position"* (PX4's own words).
  A drone needs it because it **has no wheels and therefore no odometry at all**.
* **Relocalization / scan matching** = absolute pose on a known map. That is the GPS-shaped hole.
* ⇒ 🔑 **On this rover VIO matters LESS than on a drone**, because wheel encoders already cover the
  incremental half. What is missing is the **absolute** half. ⛔ Adding VIO would improve motion
  *between* fixes; it would not produce a single fix.
* ⇒ **What the companion should publish to PX4 is the MAP-RELATIVE pose from relocalization**, not
  raw VIO. Same topic, different source. → `px4_companion_interface.md` §9.

### 7.3 Why drones use VIO indoors and ground robots use LiDAR — it is the MOTION MODEL

* **2D scan matching assumes planar motion**: a fixed horizontal slice of a fixed world.
* ✅ **A rover satisfies that assumption** — floor-constrained, effectively 3-DOF (x, y, yaw), scan
  plane parallel to the ground.
* ❌ **A drone violates it constantly** — roll and pitch tilt the scan plane, climbing moves the
  slice to a different part of the room, and a 2D LiDAR gives **no altitude at all**. Plus weight
  and power on an airframe.
* ⇒ ⛔ **The drone world's preference for VIO is NOT evidence that LiDAR is weak indoors.** The very
  assumption that makes 2D LiDAR work is the one the rover meets and the drone does not.

### 7.4 ✅ The commonality argument — a real reason for camera-primary

* **A camera-based stack transfers to the drone. A 2D-LiDAR-based one does not.**
* The two vehicles already **share the FC**, so a shared perception stack has real value.
* ⇒ **Rover alone, indoors → LiDAR is the stronger localization sensor. Rover + drone, one stack →
  the camera is the only option that serves both.** Both defensible; the choice turns on how much a
  shared stack is worth. ✅ **Decided: camera-primary (§6.4).**
* ⚠️ **Open question:** the STL-19 is *"not fitted — allocated to the drone"*. If that allocation is
  for **localization**, it is on the vehicle a 2D LiDAR helps **least** and absent from the one it
  helps **most**. ⏭ **Check what job it was actually assigned** — the allocation may be backwards.

### 7.5 ⛔ The camera is NOT replaceable by a LiDAR

Raised because "then I will buy a better LiDAR instead". **No.** A 2D LiDAR cannot see:

* **A person lying on the floor.** ISO 3691-4 tests personnel detection with a **70 mm × 400 mm
  horizontal cylinder on the ground**. A LiDAR scanning at 200–300 mm passes straight over it.
* **Forklift tines, overhanging or cantilevered loads, partially occupied shelves** — the documented
  weakness of single-plane perception.
* **Negative obstacles** — a hole is invisible to a horizontal plane. The camera can see one
  geometrically, once sub-ground points stop being discarded (§4).
* **Anything semantic** — drivable surface, person vs pallet. ⛔ "Find the road" needs a camera.
* ✅ And the camera is **already bought, mounted and calibrated (G0 closed)** — a working asset.
* ⇒ **The question was never "camera or LiDAR". It is "do we ADD a LiDAR to the camera we have".**

### 7.6 🔴 Safety rating is a separate purchase — settle it early

* **ISO 3691-4** governs driverless industrial trucks: personnel detection, speed control, behaviour
  in shared spaces. Where people may be present it expects a **safety-rated laser scanner** with
  layered fields (outer warning slows, inner protective stops).
* Scanners must be **IEC 61496 Type 3**; anything without it *"should not be selected as the primary
  safeguard"*. Typically **PL d** across the whole chain.
* 🔴 **Neither the Gemini 336L nor an STL-19P is safety-rated.**
* ⇒ if the unit has people in it, a **third, certified device** is required — separate from both the
  navigation LiDAR and the camera. ⛔ Do not assume one hobby unit can carry a site audit.
* ⚠️ Applicability depends on the deployment and the customer — ⏭ **confirm before site selection**,
  because it drives cost and mounting more than anything else discussed here.

### 7.7 ⚠️ 2D LiDAR in a manufacturing unit — real caveats if one is fitted

* ✅ **Localization: the textbook best case** — abundant geometric structure, weak/repetitive
  texture, and total immunity to lighting.
* ⚠️ **Scan-plane height is the critical mounting decision.** A plane at pallet height sees stock,
  pallets and tines — things that **move**. Localizing against movable objects gives a map that goes
  stale. **Mount to see structure, not inventory.**
* ⚠️ **Range:** the STL-19P is **12 m, ±45 mm**. Dense unit: fine. Large open hall: it may see
  nothing fixed. ⏭ **Only the site dimensions settle it** — part of the §5 survey.
* ⚠️ **Specular returns** — polished floors, bare metal machinery, glass partitions.
* ⚠️ **Layout churn** — units reconfigure; LiDAR maps need maintenance.

### 7.8 ⏭ The next action is a DIAGNOSIS, not a purchase

* 🔴🔴 **Relocalization returns 0 accepted of 20 on the map's OWN recorded bag, failing at
  GEOMETRY not appearance.** That is a **pipeline fault**.
* ⛔ **Buy nothing until it is diagnosed.** A new sensor feeding a broken pipeline buys nothing, and
  the diagnosis decides which sensor is even the right one.
* 🔑 **This is true under every architecture discussed above.** It is the one item that does not
  depend on any of the open choices.

---

## 8. Indoor-first — the working order

1. 🔴 **Diagnose the 0/20 relocalization failure** (§7.8). Blocking, and independent of every other choice.
2. **Finish T3 to a working value, take the 3 runs, then stop** (§6.7). The avoidance *chain* is
   reusable; its odom-frame *tuning* is not.
3. **Survey the unit** (§5) — dimensions, surface, lighting, traffic, and the **longest unmatched
   stretch** on the intended route.
4. **Decide fiducials** (§6.6) — they change what unaided relocalization has to achieve.
5. **Confirm the safety-rating question** (§7.6) before site selection.
6. ⏭ **Then** negative obstacles (§4), LiDAR-for-the-blind-arc (§7.4), and outdoor/GPS —
   `autonomy_plan.md` §4 — in that order.

## References

* Application A shape and A1–A9: `autonomy_plan.md` §3
* Outdoor GPS mission and the companion's role: `autonomy_plan.md` §4
* Case A / Case B, `EKF2_EV_CTRL`, position injection: `px4_companion_interface.md` §9
* Relocalization failure: `memory/project_indoor_mapping_slam.md` §17
* Sensor envelope and FOV: `px4_companion_interface.md` §4, `rover_geometry.md`

---

