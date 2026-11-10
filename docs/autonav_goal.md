# AutoNav goal — operator direction, 2026-11-10 (authoritative)

> This is the operator's statement of what AutoNav is for, what it is not, the architecture
> decisions, and the order of work. Where another document disagrees, this one wins and the other
> is to be corrected. The M0–M4 / L0–L5 / R1–R7 ladders in [`autonomy_plan.md`](autonomy_plan.md)
> and [`rover_autonav_requirements.md`](rover_autonav_requirements.md) keep their numbering.
> All existing rules stay in force: no parameter write without an explicit yes, describe before
> executing.

## The goal

The operator points to a place on a map. The rover finds a path its chassis can actually drive,
judged from terrain seen by the depth camera, follows it while avoiding obstacles, and knows its
position with or without GPS, indoors or outdoors. It stops and reports if blocked, lost or
unhealthy. No one holds the RC stick.

## AutoNav is

- Point-and-go ahead with obstacle avoidance, no map (**M2**).
- Go to a named place / patrol on a saved map (**M3**).
- Terrain-aware path choice: the **drivable** path, not the shortest. Slope, step, roughness, drops.
- Position without GPS; GPS is an aid when present.
- Health monitor, return to base, blocked/lost behaviour.

## AutoNav is not (for now)

- Exploring or mapping unknown places unattended.
- Person/object detection (a later layer, Hailo then).
- A certified safety system for working around people.
- RC driving with collision stop (that is the safety floor only).

## Architecture decisions

- **QGC / G-Control is UI only:** map view, click goal, status. The companion plans and executes
  (Nav2). PX4 drives.
- **Gemini 336L, centre-mounted, forward 92°:** terrain, obstacles ahead, visual localization.
- **The rover moves forward; areas it has passed are remembered** in the rolling map for turns and
  skids. Unknown cells = blocked. Look before turning.
- **A 2D 360° lidar will be added soon** (RPLIDAR C1 or D500 / STL-19P, whichever is bought):
  side/rear obstacles, 360° collision stop, localization aid. The camera stays primary for terrain.
- **A second Pi will be added for telemetry + WFB-NG only.** This Pi keeps autonomy.

## Facts corrected by this direction

| # | Fact | Status |
|---|---|---|
| 1 | **Geometry closed.** The frame rebuild did not change geometry. Footprint 0.73 × 0.450 m, track and wheelbase stand. No re-tape. `ObstacleFootprint` 32.0 and T3 stand. | ✅ Recorded 2026-11-11. Every "re-tape before trusting" note is withdrawn. |
| 2 | **Pi 5 V rail fixed.** Supply is now a TI-based synchronous DC-DC converter (not the XL4015), set to 5.2 V. Verify once after reboot; if it passes, lift the "no armed autonomy" hold. | 🔴 **Verification FAILED 2026-11-11 01:05 — hold stays.** See below. |
| 3 | **STL-19** was given to a separate Ackermann rover project run by another person. It is not on the drone and not available here. | ✅ Recorded 2026-11-11. |

### Fact 2 — verification record, 2026-11-11 01:05 (boot of 2026-11-10 23:37)

Pass criteria from the direction: mean ≈ 5.1 V, no dips below 4.80 V, `get_throttled` = `0x0`.

| Measure (`ext5v-report 60`) | Required | Measured |
|---|---|---|
| Mean | ≈ 5.1 V | **4.915 V** |
| Max | — | 5.055 V |
| Dips below 4.80 V | 0 | **223** |
| Dips below 4.90 V | — | 568 |
| Lowest row seen | — | 4.698 V |
| `vcgencmd get_throttled` | `0x0` | **`0x50000`** (`0x50005` live at 01:05:08) |
| Kernel "Undervoltage detected" this boot | 0 | **142** |

Not closed. The numbers look like the old supply (mean 4.94 V on 11-05), so the first thing to
confirm is that the new converter is the one actually feeding the Pi and where its 5.2 V is being
lost. ⚠️ The window included RC driving with all four ESCs on USB.

## Order of work

1. Finish 350 W motor tuning (including the rear wheels).
2. Yaw re-check in Manual; resolve heading.
3. `PreferForward` spin guard back in `nav2_forward_flat.yaml`.
4. Session B standoff ladder, then T4, T5 ⇒ **M2 done**.
5. Desk, in parallel: relocalization 0/20 diagnosis.
6. Fit the lidar: Nav2 obstacle source, then reflex, then localization. One change at a time.
7. Wire `/scan_3d` into the planner (G4), then height map + traversability cost. Test course:
   ramp, kerb, hole.
8. Visual odometry for no-GPS outdoor; GPS as aid.
9. M3 mapped patrol + health monitor + return to base.
10. QGC map view and click-to-go.

Live status of each step is tracked in the memory `todos.md`, section "AUTONAV GOAL — ORDER OF WORK".
