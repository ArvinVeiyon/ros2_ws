# Vind-Roz documentation — start here

**Two entry points cover most questions. Go deeper only when they point you there.**

| | Document | Answers |
|---|---|---|
| 📐 | **[`autonav_reference.md`](autonav_reference.md)** | *What is true, and why.* Goal · the four questions · platform + CPU budget · geometry · **calibrated constants with their methods (§5)** · odometry and heading · motion envelope · perception · mapping · known faults · **inviolable rules (§11)** · test ladder · method notes |
| 📕 | **[`setup_manual.md`](setup_manual.md)** | *What to do.* FC firmware, calibration and the **parameter changelog (§A7)** · companion build · **node inventory (Part C)** · bring-up and verification · **arming into AutoNav (§D6)** · routine operations · what is still to be written (Part F) |

**Read §5 before trusting any constant. Read §11 before designing any behaviour. Read Part C before
touching a node. Read Part E before running a procedure.**

## Deeper single topics

Kept separate because they hold derivations and raw data the summaries cannot carry.

| Document | Uniquely holds |
|---|---|
| [`rover_geometry.md`](rover_geometry.md) | **The authority on every dimension** and the camera mount, with how each was measured |
| [`rover_yaw_response.md`](rover_yaw_response.md) | The full yaw derivation — response tables, the deadband analysis, why each parameter moved |
| [`px4_param_audit.md`](px4_param_audit.md) | **Full PX4 parameter audit (2026-08-14)** — every value as read off the vehicle, what is verified correct, findings P1–P9 with derivations, and a dated change log. ⚠️ **The canonical *changelog* remains `setup_manual.md` §A7**; this file carries the audit and reasoning |
| 🔗 *`rc_configuration.md`* — **NOT IN THIS REPO** | **Moved to `codex-work` (Companion_Computer_Pxlabs), repo root**, 2026-09-04, with `px4_vesc_dronecan_implementation.md`. RC input + UAVCAN ESC output as read off the FC, and ⛔ **why `UAVCAN_EC_MIN/MAX` are 110/8082 and must not be "tidied" back to 10/8191**. ⚠️ `setup_manual.md` §A7 (below) remains the canonical param changelog and stays here |
| [`vision_streaming.md`](vision_streaming.md) | The FPV video fault record — the CPU-starvation latch and the camera wedge |
| [`autonomy_plan.md`](autonomy_plan.md) | Operating modes M0–M4 and the autonomy-level appendices |
| [`rover_autonav_requirements.md`](rover_autonav_requirements.md) | The AutoNav requirements set |
| [`indoor_mapping_plan.md`](indoor_mapping_plan.md) | The indoor mapping plan and its measured costs |
| [`rover_autonav_collision_stop.md`](rover_autonav_collision_stop.md) | Reflex design rationale and its floor validation. ⚠️ its arming workflow is now in `setup_manual.md` §D6, and **its claim that the ch8 kill is proven in AutoNav contradicts every other record — see the flag in §D6** |
| [`ros2_architecture.md`](ros2_architecture.md) | 07-19 node graph and the FC↔companion topic audit. ⚠️ **stale** — `setup_manual.md` Part C is read off the live system and supersedes the node inventory |

[`archive/`](archive/) holds superseded documents, each with a note saying what replaced it.

## Conventions

- ✅ verified on this vehicle · ⚠️ recorded but not re-verified · 📋 standard procedure, **not yet
  performed here** · 🔴 do not violate · 🚁 shared with the drone, revert before flight
- **Every number should carry the method that produced it.** A value without one is an assumption
  awaiting a test — say so rather than implying it is measured.
