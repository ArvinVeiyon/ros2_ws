# PX4 ↔ Companion interface — outdoor QGC mission

> **Status:** reference, written 2026-09-20. **Additive only.**
> ⛔ **This document changes no goal, no ladder and no plan.** M0–M4, L0–L5 and R1–R7 stay exactly
> as they are — see `autonomy_plan.md` §5 and `rover_autonav_requirements.md` §3. This file only
> records **what PX4 already provides**, **what it does not**, and **where the companion attaches**.
> Every claim is cited to the flashed firmware tree `~/apps/PX4-Autopilot` @ **`a52c38b07d`** — the
> hash actually on the FC — or to a live reading, dated.

---

## 1. The external world comes from a QGC mission

* **The operator draws waypoints in QGC on a geographic map.** That is the external world input;
  nothing else defines *where to go*.
* **QGC uploads the mission to the FC over MAVLink** and PX4 stores it — `src/modules/mavlink/mavlink_mission.cpp`.
* ⛔ **The mission lives on the FC, not on the companion.** The companion does not receive, store or
  replay it.
* **Mission items are latitude / longitude.** They therefore require GPS.
  * ⛔ **No GPS is fitted today** — outdoor item **O2**. Until it is, there is no mission at all.

## 2. The chain QGC → wheels, inside PX4

* `mavlink_mission.cpp` → stores the mission
* `src/modules/navigator/mission.cpp` → sequences it, publishes **`position_setpoint_triplet`** (previous / current / next)
* `rover_differential/DifferentialDriveModes/DifferentialAutoMode` → subscribes `position_setpoint_triplet`, publishes **`rover_position_setpoint`**
  (`DifferentialAutoMode.hpp:91,94`)
* `rover_differential/DifferentialPosControl` → subscribes `rover_position_setpoint` + `vehicle_local_position` (`DifferentialPosControl.hpp:107-109`)
* → speed / steering setpoints → control allocation → DroneCAN `RawCommand` → the four VESCs
* 🔑 **The companion is not in this chain.** A mission executes end-to-end with the companion
  switched off.

## 3. What PX4 ALREADY HAS — do not rebuild any of this

* **Mission execution for a differential rover** — `DifferentialAutoMode` exists and is wired.
* **The nav states the rover module handles** (`RoverDifferential.cpp:104-126`):
  * `AUTO_MISSION` · `AUTO_LOITER` · `AUTO_RTL` · `OFFBOARD` · `MANUAL` · `ACRO` · `STAB` · `POSCTL`
* **Geofence** — `src/modules/navigator/geofence.cpp`, params in `geofence_params.c`
* **Return to launch** — `AUTO_RTL`, with `home_position` published
* **Failsafe logic and reporting** — PX4 decides, and publishes `failsafe_flags`
* **Arming, kill switch, RC override** — ⛔ **these stay PX4's, permanently.** The kill switch is
  `ch12` (`RC_MAP_KILL_SW`=12, read live 2026-09-20).
* **Six levels of rover setpoint**, all real uORB topics: position, speed, attitude, rate, throttle, steering.

## 4. What PX4 DOES NOT HAVE — plan around these

* ⛔ **No perception of any kind on a rover.** PX4 never knows what is in front of it.
* 🔴 **`CollisionPrevention` does not apply to rovers.** It is referenced only by
  `flight_mode_manager/tasks/ManualPosition` and `Utility/StickAccelerationXY`; **no rover module
  references it**, and it is a *manual-mode* feature even on multicopters.
  * ⇒ **`obstacle_distance` is bridged in but inert.** ⛔ Do not build avoidance on it.
* 🔴🔴 **In `AUTO_MISSION` there is no obstacle protection at all.** Our reflex lives in
  `AutoNavMode::updateSetpoint()`, which `px4_ros2` calls **only while `nav_state == 23`**.
  A mission is `AUTO_MISSION`, so the reflex is **never executed** — not overridden, not called.
  * → the gap, the three jobs and the S1/S2/S3 ladder are in `autonomy_plan.md` §4.
* ⛔ **`DO_PAUSE_CONTINUE` does not exist in this firmware** — verified absent from the whole tree.
  Do not design around it.
* ⚠️ **No terrain, slope or negative-obstacle awareness.** Not a PX4 gap and not ours either —
  "rough-terrain traversability" is **out of scope for v1** by decision
  (`rover_autonav_requirements.md` §6).

## 5. Where the companion attaches — read side

All line numbers are `src/modules/uxrce_dds_client/dds_topics.yaml`. **31 out, 38 in** (comment-aware
— line 60 is a *commented-out* entry and is not live).

* **Mission state**
  * `/fmu/out/position_setpoint_triplet` (**:46**) — the active waypoint, the same input `DifferentialAutoMode` sees
  * 🔴 `mission_result` is **NOT bridged** — no progress, no completion, no failure. See §7.
* **Vehicle state**
  * `/fmu/out/vehicle_status` (**:92**) — `nav_state` + arming. ⚠️ it is `vehicle_status_v1` on the wire
  * `/fmu/out/failsafe_flags` (**:35**) — 🔑 PX4's own failsafe decision; do not re-derive it
  * `/fmu/out/vehicle_land_detected` (**:63**)
* **Position and navigation**
  * `/fmu/out/vehicle_global_position` (**:77**) · `/fmu/out/vehicle_local_position` (**:85**) — carries `eph`
  * `/fmu/out/vehicle_gps_position` (**:81**, type `SensorGps`) — fix type, satellites, accuracy
  * `/fmu/out/home_position` (**:103**) — the RTL target
* **Power**
  * `/fmu/out/battery_status` (**:23**)

## 6. Where the companion attaches — publish side

* **Intervention — `/fmu/in/vehicle_command` (:187).** Verified accepted by `Commander.cpp`:

| Intent | Command | Effect |
|---|---|---|
| Pause | `DO_SET_MODE` → `AUTO_LOITER` | holds; mission resumable |
| Slow | `DO_CHANGE_SPEED` | reduce mission speed |
| Abort | `NAV_RETURN_TO_LAUNCH` | return home |
| Hard stop | `COMPONENT_ARM_DISARM` (force) | motors off — last resort |
| Detour | `DO_REPOSITION` | ⛔ not recommended, fights the running mission |

* **Direct control (only if S2 handover is built)**
  * `/fmu/in/rover_position_setpoint` (**:220**) — position goal with `start_ned`, `cruising_speed`, `arrival_speed`
  * `/fmu/in/rover_speed_setpoint` (**:223**)
  * `/fmu/in/trajectory_setpoint` (**:172**) — the `OFFBOARD` route (`DifferentialOffboardMode.hpp:81`)
  * ⚠️ **All are mode-gated.** Which nav_state routes to which controller is **not yet verified** —
    check before designing on them.
* **Position augmentation**
  * `/fmu/in/aux_global_position` (**:205**) — external global position, if GPS alone proves insufficient
* **Today's actual path** — unchanged: Nav2 → `/cmd_vel` → `autonav_mode` → `RoverSpeedRateSetpointType`
  → PX4, as an external mode registered at **`mode_id` 23**.

## 7. What must be ADDED to PX4

* Full list with field-level justification: **`autonomy_plan.md` §4 → *What must be ADDED to PX4***.
* Summary:
  * **Tier 1 (blocking):** `mission_result` · `position_controller_status` · `geofence_result`
  * **Tier 2:** `vehicle_angular_velocity` (⏭ already staged, **commented out at `dds_topics.yaml:60`** — an uncomment) · `navigator_mission_item` · `rtl_status`
  * **Not needed:** `obstacle_distance` (inert) · `distance_sensor` · `sensor_gps` (redundant) · `mission` · `actuator_armed`
* ⚠️ **Two-sided change:** rebuild + reflash PX4 **and** rebuild `px4_msgs` on the companion from the
  same commit. 🔑 A mismatch raises **no error** — the topic silently never connects.

## 8. Parameters that are wrong for outdoor — all read live 2026-09-20

* `COM_LOW_BAT_ACT` = **0** — a flat battery warns and does nothing else
* `GF_ACTION` = **0** — the geofence takes no action at all
* `NAV_RCL_ACT` = **1** (Hold) — RC loss holds; it does **not** return home
* ⛔ **Each is an operator decision.** ⛔ **Never write a vehicle parameter without an explicit yes.**
* → context in `autonomy_plan.md` §6

## References

* Firmware tree: `~/apps/PX4-Autopilot` @ `a52c38b07d` — the hash flashed on the FC
* Bridge definition: `src/modules/uxrce_dds_client/dds_topics.yaml`
* Rover module: `src/modules/rover_differential/`
* Mission: `src/modules/mavlink/mavlink_mission.cpp`, `src/modules/navigator/mission.cpp`
* Companion mode: `~/ros2_ws/src/autonav_mode/include/autonav_mode/mode.hpp`
* Companion role, gap analysis and build ladder: `autonomy_plan.md` §4
* Safety requirements incl. **R5.6** (mission-mode gap): `rover_autonav_requirements.md` §3
