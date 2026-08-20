# FC hardfault analysis — 2026-08-16

Standalone working document. **Not yet folded into `autonav_reference.md` or memory** — the
conclusion below is strong but one question (see OPEN) has to be answered before acting on it.

---

## 1. The data

Nine fault logs recovered from the FC's SD card and analysed. Seven were on the card at the time of
writing (`/fs/microsd/fault_*.log`); two had already been backed up to `~/fc_faults/`.

| when (FC clock) | task | site | cfsr | decoded | user stack | pc |
|---|---|---|---|---|---|---|
| 05:01:15 | `wq:uavcan` | `chip/imxrt_irq.c:272` | `0x00080000` | UF:NOCP | 100% | `300ce0a2` |
| 05:02:42 | `mavlink_if1` | `armv7-m/arm_memfault.c:101` | `0x00000082` | MM:DACCVIOL,MMARVALID | 100% | `300a3a66` |
| 09:53:36 | `wq:INS0` | `chip/imxrt_irq.c:272` | `0x00080000` | UF:NOCP | 100% | `300fe59e` |
| 09:54:34 | `uxrce_dds_client` | `chip/imxrt_irq.c:263` | `0x00000400` | BF:IMPRECISERR | 100% | `3024a650` |
| 10:17:31 | `uxrce_dds_client` | `chip/imxrt_irq.c:272` | `0x00010000` | UF:UNDEFINSTR | 100% | `301bf4a2` |
| 10:18:35 | `uxrce_dds_client` | `chip/imxrt_irq.c:272` | `0x00010000` | UF:UNDEFINSTR | 100% | `30162922` |
| 10:24:01 | `wq:INS0` | `chip/imxrt_irq.c:272` | `0x00080000` | UF:NOCP | 100% | `300fe57e` |
| 10:26:22 | `wq:uavcan` | `armv7-m/arm_memfault.c:101` | `0x00000082` | MM:DACCVIOL,MMARVALID | 87% | `3025c764` |
| 10:31:58 | `wq:uavcan` | `chip/imxrt_irq.c:263` | `0x00000100` | BF:IBUSERR | 100% | `2fd930d2` |

Firmware is **identical in all nine**: git-hash `f0889f3d108b73294872a9271158f9ea6f61604e`,
built May 31 2026.

Distribution: **tasks** `wq:uavcan` 3, `uxrce_dds_client` 3, `wq:INS0` 2, `mavlink_if1` 1 ·
**fault types** NOCP 3, DACCVIOL 2, UNDEFINSTR 2, IMPRECISERR 1, IBUSERR 1 · **PCs** all nine distinct.

## 2. What it means

🔑 **Four tasks, five fault types, nine distinct PCs — every axis varies. That is not a software
defect.** A code bug repeats: same task, same PC, same fault. Nothing here repeats.

🔑 **What the fault types share is CODE INTEGRITY, not logic.**
- `UNDEFINSTR` — the CPU fetched bytes that are not a valid instruction.
- `IBUSERR` — the instruction *fetch itself* failed.
- `NOCP` — usually garbage that happened to decode as a coprocessor instruction.
- `IMPRECISERR` — a buffered write failed behind the core.
- `DACCVIOL` — data access violation.

Five different symptoms of one underlying thing: **the processor executing or fetching data that is
not what the firmware put there.**

🔑 **The PCs support that reading.** Eight lie in `0x300a…0x3025`, which on i.MX RT is the external
**FlexSPI flash** region — instructions are fetched over a serial flash bus. The ninth, `2fd930d2`,
is *below* that base: a branch to nowhere.

⚠️ **The "100% user stack" column is suggestive, not proof.** PX4 derives stack usage from watermark
colouring, and a fault that corrupts the stack defeats that calculation. Do not report this as nine
stack overflows. The one reading 87% argues the number is at least sometimes meaningful.

## 3. What this rules out

⛔ **uXRCE is not implicated as a cause.** It is 3 of 9, tied with `wq:uavcan`. That ranking tracks
how much CPU each task consumes, which is exactly what you expect when the fault strikes whichever
task is running. Being the most frequent victim is not evidence of being the cause.
⛔ **Not a bug in DroneCAN, MAVLink or the INS either**, for the same reason.
⛔ **Not the specific FC board** — the same firmware hash faulted before and after the board swap.

## 4. ✅ ANSWERED — every one of these nine is AFTER the FC swap

**Operator, 2026-08-16: "whatever you seen here is after fc swap."** So the swap predates 05:01:15
(FC clock) and all nine faults are from the **replacement board**.

🔴 **This is the load-bearing fact of the whole document.** A *new* board, running the same firmware
hash, reproduced the identical scatter — four tasks, five fault types, nine PCs. Combined with §2,
that means:

- ⛔ **The board was never the cause.** Replacing it changed nothing, which is what the earlier
  "FIRMWARE not hardware, DON'T SWAP THE FC" call predicted. That call is now confirmed by
  experiment, at the cost of a swap.
- ⇒ The cause is in what the swap did **not** change: the firmware image `f0889f3d`, the
  configuration, the wiring/rail feeding the FC, the SD card, or the peripherals on the buses.

### 🔑 The other half: the faults STOPPED
Nothing has faulted since **10:31:58** — verified ~16:20, roughly **six hours clear**, and the card
holds no later fault file. So the cluster is bounded at both ends.

⇒ **The highest-value question is no longer "why does it fault" but "what changed at ~10:31 that
stopped it".** The operator was physically working on the vehicle through that window (camera
rotated to reach the FC; an ESC removed around 11:17–11:22 per the handover ledger). Whatever was
disturbed is the strongest lead available, and it is a *physical* lead, not a firmware one.

⚠️ **The restarts since are a DIFFERENT failure.** 14:57:23 and 15:05:43 (seen as `microxrce-agent`
`create_participant` re-establishes) wrote **no fault log at all**. A hardfault always commits one;
a hard power loss does not. Do not pool these with the nine above — that pooling is what made the
earlier "≥9 reboots" count ambiguous in the first place.

⚠️ **Do not build a timeline from the filenames.** The FC clock is inconsistent across these files
(`05:xx` vs `09:xx`/`10:xx`, roughly an IST offset apart), so the ordering within the window is not
trustworthy. What *is* trustworthy: all nine predate 10:32, and none exist after it.

## 5. Tooling — two real bugs found while getting this data

🔴 **`tools/fc_fault_backup.py` reports "no fault logs on the FC" unconditionally.** Its
`list_faults()` reads `ftp.dir_contents`, which **does not exist** in this pymavlink; directory
results land in **`ftp.list_result`** as `DirectoryEntry` objects. The tool has therefore been blind
for as long as it has existed, and it produced a confident false negative here ("no fault_*.log
found on the FC") while seven logs sat on the card. **Fix before trusting it again.**
Its extra `pump(m, ftp, 'ListDirectory')` is also redundant — `cmd_list` already calls
`process_ftp_reply` internally.

🔑 **How to actually pull a file over MAVLink FTP here** (this took several attempts):
1. Send `OP_ResetSessions` with an **empty bytearray** payload, not `None`, and *pump the reply*.
   PX4 has very few FTP sessions and does not free them on client exit — every failed attempt leaks
   one, and the next run dies with `OpenFileRO failed, no sessions available`.
2. Use a **fresh `mavutil` connection AND a fresh `MAVFTP` object per file.** Reusing the object
   after a completed transfer makes the following file return empty.
3. Pass a **`callback=`** to `cmd_get`. With a callback, pymavlink buffers to memory and hands the
   handle over; without one the burst-EOF nack is logged as an error and the local file is left at
   **0 bytes** even though the whole transfer succeeded.
4. `burst nack ... req_opcode:15 ... ofs:<filesize> [6]` is **EOF, i.e. success**, not a failure.

⛔ **The MAVLink shell cannot substitute.** `cat` of a 44 KB fault log loses data: an earlier attempt
lost the tail, and the attempt here lost the **head** (capture began mid-stack-dump at `0x2024ada8`),
which is the only part that matters. Use FTP.

## 6. Context gathered alongside (not conclusions)

- `UAVCAN_ENABLE = 3` — sensors **and motors**, automatic config. Not idle: the four VESCs are
  DroneCAN nodes, so `wq:uavcan` services four live nodes continuously.
- `UXRCE_DDS_CFG = 103` (enabled on a serial port), `UXRCE_DDS_PTCFG = 0`.
- ⚠️ The param is **`UXRCE_DDS_CFG`**, not `XRCE_DDS_CFG`; the latter returns `<no reply>` and must
  not be read as "unset".
- FC's own 5 V rail, sampled live at idle via `listener system_power`: **min 5.017, mean 5.132,
  max 5.194 V**, n=35, `brick_valid` always 1, no overcurrent flags. Healthy at idle. This says
  nothing about the rail during the fault window.
- ⚠️ **The FC's 5 V and the Pi's 5 V are separate feeds from the battery.** `/var/log/ext5v/` and
  `ext5v-logger` measure the **Pi's** rail only and are NOT a witness for the FC. (Separately, and
  unrelated to the FC: the Pi's rail reached 4.808 V today against its own ~4.8 V brownout figure.)
- PX4 does not stream `POWER_STATUS` here, even after `MAV_CMD_SET_MESSAGE_INTERVAL`. Use
  `listener system_power` over the MAVLink shell instead.

## 7. Next steps if this is picked up again

1. ⛔ **Do not swap the FC again.** It has been tried; §4 shows the replacement board faults
   identically. Another board is the one experiment already known to fail.
2. **Work the "what stopped it at 10:31" lead first** (§4). It is physical and recent, and a fault
   that has been absent for six hours cannot be provoked by reading logs.
3. If it recurs, the cheap discriminator is whether a **fault log is written**: hardfault yes, power
   loss no. Check `ls /fs/microsd` before assuming which failure occurred.
4. Only if it recurs *and* writes logs: the code-integrity reading (§2) points at instruction fetch,
   so the questions are flash integrity and re-flashing from a known-good image.
5. All nine logs are preserved in **`~/fc_faults/`** (copied out of the session scratchpad, which is
   discarded). ⚠️ `fault_2026_08_16_09_53_36.log` is partial — 34655 of 44763 bytes — but its header
   is intact, which is the part §1 uses.

---

## 8. 2026-08-16 late — operator evidence that reframes this whole document

Four statements from the operator, in the order given:

1. **"last 2 month we using same firmware version"** — clean the whole time.
2. **"it happen after hard hit"** — a physical impact.
3. **"nothing inside components all worked well after one day"** — inspected, then ~a day quiet.
4. **"it start happen when we doing rc work and kept happen there after."**
5. 🔴 **"whenever you start working localization i see the hard fault it alway hapen like that."**

### 8.1 What this kills
⛔ **The firmware image is exonerated.** An image that ran two months clean is not corrupt. A
hypothesis raised earlier in this session ("re-flash from a known-good image") is **withdrawn** — §7
step 4 should no longer be worked.
⛔ **Combined with §4, the elimination is now near-total.** The operator has replaced the **FC board**
(new CPU *and* new FlexSPI flash chip), replaced the **power module**, and **removed every motor
connection** — and it still faults. Nothing that was swapped is the cause.
⛔ **Not a ROS 2 topic flood / uXRCE spam.** Checked directly this session: the only `/fmu/in/*`
publisher is `autonav_mode` (the second endpoint on each topic is `_CREATED_BY_BARE_DDS_APP_`, the
agent itself), and `ros2 topic hz` measured **zero messages in 15 s** on every setpoint topic. The
`ros2_px4_translation_node.service` crash-loop dies at `Package 'translation_node' not found`, i.e.
before any DDS participant exists, so it never publishes. Independently of that measurement, §2
already rules a flood out on signature: it would hit **one** task with a **repeating** PC.

### 8.2 What survives — the leading theory
🔑 **Marginal supply voltage on the FC's feed, provoked by load, enabled by impact damage.**

The trigger set the operator names — **localization** and **RC work** — is exactly the set of
**largest current draws** on the vehicle. RTAB-Map localization plus the camera pins the Pi (and
`claude` itself runs 55–86% of a core alongside it); RC work draws motor current. Per §6 the FC's
5 V and the Pi's 5 V are **separate feeds from the same battery**, so a spike on either sags the
shared source.

🔑 **This is precisely what the fault signature has been saying since §2.** `UNDEFINSTR`, `IBUSERR`
and `NOCP`, with eight of nine PCs in the **FlexSPI external-flash region**, mean the CPU fetched
instructions that were not what the firmware put there. Instruction fetch over a serial flash bus is
voltage- and timing-marginal — a droop too brief to trip the brownout reset corrupts the fetch and
faults **whichever task happens to be running**. That mechanism predicts the otherwise baffling
scatter of §2: four tasks, five fault types, nine distinct PCs.

⚠️ **Corroboration already in this document:** §6 records the FC rail healthy *at idle*
(min 5.017 V) and explicitly notes that says nothing about the fault window — and separately that
**the Pi's rail reached 4.808 V against its own ~4.8 V brownout figure**. The system is already
running at the edge under load.

🔑 **Why the swaps did not help, under this theory:** the damaged element is in the part no swap
touched — the **wiring harness / crimps / connector feeding the FC**, the **carrier board** if this
FMU is modular, or the **battery connector**. Replacing the power *module* does not replace the
harness downstream of it.

### 8.3 The discriminating test
**Log the FC's OWN rail while starting localization, and watch for the dip.**
- ✅ Use `listener system_power` over the MAVLink shell, or `SYS_STATUS` / `BATTERY_STATUS` over
  MAVLink. ⚠️ **`ext5v-logger` and `/var/log/ext5v/` measure the PI's rail and are NOT a witness for
  the FC** (§6) — do not repeat that substitution.
- Start from a quiet baseline, then bring up localization, and compare against the idle figures in
  §6 (min 5.017, mean 5.132 V).
- A visible droop confirms it. Then **reseat/replace the FC power harness and crimps** before
  anything else.
- ⚠️ The ULogs in `/fs/microsd/log/` record `system_power` and `VIBRATION` continuously. A ULog
  covering one of the nine faults would show the rail in the seconds before the crash **without
  reproducing anything** — that is the cheapest evidence available and should be pulled first.

⚠️ **The newest fault log was deleted by the operator before it could be read.** Keep the next one:
per §7.3 the presence or absence of `fault_*.log` is the only cheap discriminator between a hardfault
and a power loss — and under this theory both are expected, from the same root cause.

### 8.4 Correction to §4's headline lead
§4 asked "what changed at ~10:31 that stopped it" and pointed at the ESC removed ~11:17–11:22. That
framing assumed the faults had stopped for good. They had not — the operator reports recurrence.
**The lead to work is §8.3, not the 10:31 boundary.** The quiet interval is better explained by *not
running heavy load* during it than by any physical change.

---

## 9. 2026-08-16 late — first ULog evidence, and a limit on what ULogs can prove

Pulled `/fs/microsd/log/2026-08-16/10_25_08.ulg` (338 881 B, complete) over MAVFTP.
Preserved in **`~/fc_ulogs/`**.

### 9.1 Which window it covers
Data spans t=0→65.9 s since boot; the log FILE was opened at t=64.958 ("Armed by RC switch"), so
**boot ≈ 10:24:03** — i.e. immediately after the **10:24:01** fault. The vehicle armed at ~10:26:12,
"Takeoff detected", disarmed 0.9 s later, and the log closed normally on disarm.

🔑 **No log captures a fault.** This one ends on a clean disarm ~73 s BEFORE the 10:26:22
`wq:uavcan` fault. The logger only runs while armed, so the faults — which happened while
**disarmed** — fall in the gaps. Do not expect to find the crash inside a ULog.

### 9.2 What was healthy in the ~2 min before a `wq:uavcan` fault
| witness | reading |
|---|---|
| `can_interface_status.io_errors` | **0**, over 163 694 frames_rx / 3 975 frames_tx |
| `esc_status` | `esc_count`=4, `esc_online_flags`=15 (**all four ESCs online**) |
| per-ESC `esc_errorcount` / `failures` | **0** on all four |
| ESC voltages | 24.77 – 25.02 V |
| `dronecan_node_status` | node 10, `health`=0, `mode`=0 (OPERATIONAL), uptime 7 616 s |
| `battery_status.voltage_v` | 24.665 – 24.699 V (current 0.21 – 1.09 A) |
| `system_power.voltage5v_v` | 5.173 V, `brick_valid`=1 |
| FC `cpuload` | 0.303 – 0.385 |
| dropouts | 0 |

⇒ **The CAN bus was electrically clean and the ESCs healthy ~73 s before a `wq:uavcan` hardfault.**
That argues against a *gradual* degradation and against a noisy/erroring bus as the visible trigger.

### 9.3 🔴 The limit — ULogs CANNOT resolve the voltage-sag hypothesis
**`system_power` appears exactly ONCE in 66 s. `battery_status` appears 6 times.** At those rates a
transient droop of the kind §8.2 proposes is invisible by construction.

⛔ **Do not treat "the ULog shows a healthy rail" as evidence against the sag theory** — it has no
power to show otherwise. §8.3's "pull a ULog to see the rail" step is therefore **weaker than stated
there**; it is good for CAN/ESC health (§9.2) and useless for transients.
✅ To test the sag properly, either raise the `SDLOG_PROFILE` / logging rate for `system_power`, or
measure **live at high rate** while applying load. MAVLink `SYS_STATUS` streams battery voltage at
~5 Hz here and is the cheapest live witness.

### 9.4 New operator evidence — the CAN correlation
**"it happened after i connected can of all the motors"**, and (later) **"just now disconnected"**.

⚠️ **This sits in tension with the earlier report that the faults persisted after all motor
connections were removed.** The ordering of those two statements has NOT been established and is now
the single highest-value unknown — resolve it before ranking causes.

🔑 If faults require the motor CAN to be connected, then `wq:uavcan` being the joint-most-faulting
task (3/9) stops being incidental, and **§3's dismissal of DroneCAN needs revisiting** — that
dismissal rested on the task distribution merely tracking CPU share.

⚠️ **Connecting motor CAN changes TWO things at once — separate them:**
1. **Bus/software:** four DroneCAN nodes appear; `wq:uavcan` does real work.
2. **Electrical:** four ESCs power up — inrush, switching noise, and current draw on the **shared
   battery** (§8.2). A hard hit could have damaged CAN wiring, a connector, an ESC, or the FC's CAN
   transceiver.

### 9.5 The A/B the operator is already half-way through
The motor CAN is disconnected **as of now**. That sets up the cleanest experiment available:
1. Run sustained load (localization) with CAN **disconnected** → no faults?
2. Reconnect CAN, repeat the same load → faults return?
A clean split confirms the CAN/ESC branch; faults in both confirm the general-load branch.
⚠️ Keep every `fault_*.log` this produces (§7.3).

---

## 10. 2026-08-16 late — THE BREAK. Two operator facts + a stack-usage pattern

### 10.1 The two facts
1. 🔴🔴 **"no it never hardfault when can disconnected."** ⇒ **motor CAN connected is NECESSARY for
   the fault.** This supersedes the earlier ambiguity flagged in §9.4.
2. 🔴🔴 **"in CAN only gnd and CANh and CANl only goes to ESC"** ⇒ the harness carries **GND, CAN_H,
   CAN_L and NO POWER LINE.**

### 10.2 ⛔ The voltage-sag theory of §8.2 is DEAD — retracted
Connecting a 3-wire CAN harness with no power conductor **adds no current draw**. If faults require
that connection and that connection cannot change the load, load is not the mechanism. §8.2 is
**withdrawn**; do not re-propose it on the strength of the FlexSPI-region PCs alone.
⚠️ The "localization triggers it" correlation of §8 is therefore **not** about CPU/current either.
Its likely explanation is mundane: localization sessions are simply when the vehicle sits powered up
with the CAN connected for long stretches. Treat it as an exposure-time correlation, not a cause.

### 10.3 🔑 The stack-usage pattern across all nine logs
Parsed the **User stack** block from every log in `~/fc_faults/`:

| log | task | stack used/size | % | pc |
|---|---|---|---|---|
| 05:01:15 | `wq:uavcan` | 3600/3600 | **100.0** | `300ce0a2` |
| 05:02:42 | `mavlink_if1` | 3056/3056 | **100.0** | `300a3a66` |
| 09:53:36 | `wq:INS0` | 5976/5976 | **100.0** | `300fe59e` |
| 09:54:34 | `uxrce_dds_client` | 7872/7872 | **100.0** | `3024a650` |
| 10:17:31 | `uxrce_dds_client` | 7872/7872 | **100.0** | `301bf4a2` |
| 10:18:35 | `uxrce_dds_client` | 7872/7872 | **100.0** | `30162922` |
| 10:24:01 | `wq:INS0` | 5976/5976 | **100.0** | `300fe57e` |
| 10:26:22 | `wq:uavcan` | 3140/3600 | 87.2 | `3025c764` |
| 10:31:58 | `wq:uavcan` | 3600/3600 | **100.0** | `2fd930d2` |

🔑🔑 **Eight of nine are EXACTLY saturated — `used == size` to the byte — across FOUR tasks with FOUR
DIFFERENT stack sizes (3056, 3600, 5976, 7872).** That is not a coincidence, and the **87.2%
outlier proves the metric is capable of reporting a non-full stack**, so the saturation is real
information rather than a broken calculation.

⚠️ **Correction to §2.** That section's caution — "do not report this as nine stack overflows" — was
right *in isolation*, but combined with §10.1 the pattern now carries weight. Equally, §2's argument
"four tasks ⇒ not a software defect" is **unsound**: memory corruption from a single source produces
faults in arbitrary tasks at arbitrary PCs, which is exactly this table.

⚠️ **What cannot be separated from these logs alone:** whether each task genuinely overflowed, or
whether one corrupting agent destroyed the stack-colouring paint of whatever it hit. Both produce
`used == size`. Do not assert "nine stack overflows" as fact.

### 10.4 ⇒ Unifying reading: something in the CAN/DroneCAN path is scribbling memory
It is the only story that fits every constraint at once: CAN-dependent (§10.1), not electrical
(§10.2), memory-corruption fingerprint (§10.3), survived an FC board swap (§4), survived a power
module swap, ran clean for two months before a **hard hit**, and puts `wq:uavcan` at joint-most-
faulting (3/9) — the task that only does real work when the bus is connected.

Two live branches, **both** requiring CAN connected:

**(A) DroneCAN receive path corrupting memory.** Possibly driven by malformed or unexpected frames
from an ESC damaged in the impact. Note `can_interface_status.io_errors` was **0** (§9.2) — frames
valid at the CAN layer, so a defect at the DroneCAN/application layer is consistent with that.

**(B) CAN GND carrying return current / a ground offset.** The harness shares GND with four ESCs
that ARE powered (24.9 V, §9.2) from the battery. If the impact damaged the **main power ground**,
motor return current can divert through the thin CAN ground wire — injecting noise and offsets into
the FC. Physical, impact-caused, and invisible to a board or power-module swap.

### 10.5 The two discriminating tests
1. **Reconnect the ESCs to CAN ONE AT A TIME**, with load, watching for a fault after each. If one
   specific ESC reproduces it, branch (A) with that ESC as the source. 🔑 ADDR map: 10=right-front
   (INVERTED) · 11=front-left · 13=rear-left · 12=right-rear.
2. **Measure the main power ground.** Voltage between FC GND and ESC GND with the ESCs powered, plus
   continuity/resistance of the main battery ground harness. A damaged main ground confirms (B).
3. Supporting: with CAN connected, watch `wq:uavcan` stack high-water live (`top` / `work_queue
   status`) and see whether it climbs toward 3600.

⚠️ Still unexplained under (A): why two clean months on this firmware. An ESC damaged by the impact
changing what it transmits is the most economical answer, and test 1 checks it directly.

---

## 11. 2026-08-16 — VESC config audit (`PXLABS_BLDC_VESC6_MK5`, branch `pxlabs-6.06-rover-uavcan_main`)

Repo `Motp_Config_Bldc/`, single commit **f80e578, 2026-08-15** (the day before the faults — but that
is the GitHub upload date, not necessarily the flash date). Enums verified against `datatypes.h` in
the same repo: `can_baud_rate` 2=500K, 3=1M · `can_mode` 0=VESC, 1=UAVCAN, 2=COMM_BRIDGE, **3=UNUSED**.

### 11.1 ⛔ It does NOT explain the hardfaults
- The only out-of-range value found is `uavcan_esc_index = 7`, and PX4 **bounds-checks it**:
  `esc.cpp:135` at the exact firmware commit `f0889f3d` reads
  `if (msg.esc_index < esc_status_s::CONNECTED_ESC_MAX)` (=8). **No out-of-bounds write is possible
  from an ESC index.** A hypothesis raised in this session is therefore **withdrawn**.
- The ULog (§9.2) shows `esc_online_flags = 15` = bits 0–3, so ESC indices were **0–3 at fault
  time** — the index-7 config was NOT in effect then.
- Motor configs are consistent and healthy (§11.4).

### 11.2 🔴 The real gap: two of four August app configs DO NOT EXIST
`vesc_appconf_Aug_front_Right_2026.xml` and `vesc_appconf_Aug_Rear_Right_2026.xml` are named as app
configs but **contain `<MCConfiguration>`** (motor configs, 9898 B vs ~6975 B for a real app config).
⇒ **There is NO recorded CAN configuration for right-front (id 10) or rear-right (id 12).**
🔑 Those are exactly the two whose `can_mode` / `can_baud_rate` / `controller_id` you would need in
order to rule out a bus-level fault. **Read them back off the hardware — do not trust this repo.**

### 11.3 🔴 Landmines in the folder (correct today, dangerous if flashed)
| file | issue |
|---|---|
| `vesc_appconf_Aug_front_left_2026.xml` | `uavcan_esc_index = 7` on id 11, while id 13 uses index 3. For a 4-motor rover indices should be 0–3. If flashed, PX4 marks an ESC online at **bit 7** while `esc_armed_flags = (1<<4)-1` covers only bits 0–3 (`esc.cpp:150`) — breaking `esc_status` and the wheel↔ESC mapping odometry depends on. |
| `vesc_appconf.xml`, `vesc_appconf_Tested_06_feb_26.xml` | `can_mode = 3` = **CAN_MODE_UNUSED** — flashing either takes that ESC OFF the bus. |
| `vesc_appconf_Left_Front.xml` | `can_baud_rate = 2` (**500 K**) while every other config is 3 (1 M) — a baud mismatch produces continuous error frames. |
| **all app configs except the two Aug ones** | carry `controller_id = 11`. Flashing one to the wrong ESC creates a **DUPLICATE NODE ID** on the bus — the one config-side mechanism that could genuinely cause CAN chaos. |

⚠️ Nothing distinguishes these by filename alone. This folder is a loaded gun for the next flash.

### 11.4 ✅ Motor configs (`*__15_Aug_26.xml`) are fine
`si_motor_poles`=14, `si_gear_ratio`=3, `si_wheel_diameter`=0.083, `motor_type`=2, `foc_sensor_mode`=2,
`l_current_max`=25, `l_abs_current_max`=35, `l_watt_max`=350, `m_invert_direction`=1 — **identical on
all four**. The values that differ (`foc_motor_r`, `foc_motor_l`, `foc_motor_flux_linkage`,
`foc_observer_gain`) are per-motor FOC detection results and SHOULD differ. Nothing here can fault an FC.
✅ `si_motor_poles` agrees across all four — no repeat of the [[si_motor_poles ↔ erpm_to_ms]] hazard.
⚠️ Minor unresolved: `si_gear_ratio = 3` contradicts the recorded "direct-drive hub, no gearbox".
Harmless for our odometry (VESC ERPM is pole-corrected, NOT gear-corrected) but reconcile it.

### 11.5 Next check on this branch
Read the LIVE config back from each of the four ESCs — `controller_id`, `can_mode`, `can_baud_rate`,
`uavcan_esc_index` — and confirm: four DISTINCT ids (10/11/12/13), all `can_mode`=1, all baud=1M,
indices 0–3 with no duplicates. Two of the four are unrecorded (§11.2), so this is the only way to know.

---

## 12. 🔴🔴 RESOLVED (strong): the added custom CAN message `VESC_RTDATA` floods `wq:uavcan`

**Operator, 2026-08-16: "now i found the issue i add additional can message maybe that casue that
issue."** Traced and quantified. This is the best-supported explanation and it fits EVERY constraint.

### 12.1 What was added
`libcanard/canard_driver.c:515 sendRtData()` broadcasts a **custom** message:
- `VESC_RTDATA_ID = 20601`, `VESC_RTDATA_MAX_SIZE = (544+7)/8 = ` **68 bytes** → a **multi-frame**
  DroneCAN transfer of ~10 CAN frames (7 payload bytes/frame + tail byte, + 2-byte transfer CRC).
- Payload is ~35 floats: volt_in/d/q, 4× MOSFET + 3× motor temps, curr_motor/in/d/q, **full IMU
  (roll/pitch/yaw, accel xyz, gyro xyz)**, erpm, rpm, duty, Ah/Wh used+charged, encoder_pos,
  battery_level, fault_code, vesc_id.
- Gated by `can_status_msgs_r1` **bit 0**, sent at `can_status_rate_1`.

### 12.2 The configured rate — and the measured result
Every app config in the repo has `can_status_rate_1 = 50` (Hz) and `can_status_msgs_r1` with **bit 0
SET** (9 = 0b1001, or 25 = 0b11001 on rear-left id 13). **So RTData ships at 50 Hz from every ESC.**

Predicted: 4 ESCs × 50 Hz × (~10 RTData + ~3 ESC-Status frames) ≈ **2600 frames/s**.

✅ **MEASURED from `10_25_08.ulg` `can_interface_status` (10 samples over 0.92 s):**
| | |
|---|---|
| **RX 2553 frames/s** | vs TX 126 frames/s |
| **≈33% bus load @ 1 Mbit** | (~130 bits/frame incl. stuffing) |
| `io_errors` | **0** — the frames are perfectly valid CAN, there are simply a great many |

⇒ **RTData accounts for ~2000 of the 2553 frames/s — roughly 78% of all CAN traffic.** Without it the
bus would carry ~600 frames/s.

### 12.3 Why this explains everything the other theories could not
| observation | explained |
|---|---|
| never faults with CAN disconnected (§10.1) | no frames → `wq:uavcan` idle |
| CAN harness is GND/CAN_H/CAN_L, no power (§10.2) | it is **data volume**, not current — as the operator said |
| PX4 firmware `f0889f3d` ran **2 months clean** | the change was on the **VESC side**, not the FC's |
| `wq:uavcan` joint-most-faulting (3/9) | it processes every one of those 2553 frames/s |
| faults in `INS0`, `mavlink_if1`, `uxrce_dds_client` too | memory corruption from one overflowing task lands anywhere (§10.3) |
| stack saturated in 8/9 logs (§10.3) | `wq:uavcan` user stack is only **3600 bytes** |
| `io_errors = 0` (§9.2) | valid frames, excessive rate — a load problem, not a bus fault |
| FC board swap + power module swap both useless (§4) | neither touches CAN traffic |
| ⚠️ the "hard hit" onset | **NOT explained by this.** Most likely coincidental with the reflash. Do not force-fit it. |

🔑 **PX4 does not subscribe to message ID 20601.** Every one of those ~2000 frames/s is pure RX
dispatch overhead on the FC — cost paid, no value received.

### 12.4 ⚠️ What is NOT yet proven
The mechanism is quantified and every constraint fits, but the **exact code path that exhausts the
stack has not been traced**. Treat §12 as a strong, testable conclusion — confirmed by the A/B in
§12.5, not before.

### 12.5 ✅ THE FIX / THE TEST (cheap, reversible, decisive)
**Clear bit 0 of `can_status_msgs_r1` on all four ESCs** → RTData off, ESC Status still 50 Hz (which
wheel odometry needs — do NOT just lower `can_status_rate_1`, that would starve `esc_status` too).
Expected: ~2553 → ~600 frames/s, bus load 33% → ~8%. Then run the load that used to fault it.

If RTData telemetry is actually wanted, **move it to the r2 schedule instead**: clear `..._msgs_r1`
bit 0, set `..._msgs_r2` bit 0, with `can_status_rate_2 = 5` (already 5 in every config) — 10× less
RTData traffic (~200 frames/s).

🔴 **TRAP:** `conf_general.c:2254` force-sets `can_status_msgs_r1 = 0b00001111` (bit 0 ON) during the
CAN ID-assignment routine. **Re-running ID assignment will silently re-enable RTData.** Re-verify the
mask after any ID reassignment.

⚠️ Re-measure `can_interface_status.frames_rx` afterwards to confirm the drop — that is the
independent ruler, not the absence of faults over a short window.

---

## 13. ⚠️ CORRECTION to §12 — `can_status_msgs_r1` bits 1–5 are INERT in UAVCAN mode

**Operator, 2026-08-16: "now i removed 4 and 5 which i introduced newly so far no issue."**
That refers to VESC status messages **4 and 5** = mask bits **3 and 4** (`send_can_status()` maps
bit N → `comm_can_send_status(N+1)`, `comm_can.c:1451-1501`). Mask history in the repo:
`25` = 0b11001 (bits 0,3,4 = status 1,4,5) on id 13 · `9` = 0b1001 (bits 0,3) on the rest.

### 13.1 🔴 Removing status 4 and 5 changes NOTHING on the wire here
`comm_can.c:1513` gates the entire VESC-native status path:
```c
if (conf->can_mode == CAN_MODE_VESC) {           // == 0
        send_can_status(conf->can_status_msgs_r1, conf->controller_id);
}
```
**These ESCs run `can_mode = 1` (CAN_MODE_UAVCAN), so status 1–6 were NEVER transmitted.**

✅ **The measurement proves it independently.** 2553 frames/s ÷ (4 ESCs × 50 Hz) = **12.8 frames per
cycle per ESC**, which is exactly `EscStatus (14 B → 3 frames) + RTData (68 B → 10 frames) = 13`.
**There is no room in the measured traffic for status 4 and 5.** They contributed zero.

### 13.2 🔑 The only bit that matters is BIT 0
Bit 0 is special — it is read in **two** places:
- `comm_can.c` (VESC-native status 1) — inert here, gated by `can_mode`.
- **`canard_driver.c:1459` — gates `sendRtData()`, and this one is NOT gated by `can_mode`.**

⇒ **`25 → 9` or `9 → 1` leaves RTData fully enabled at 50 Hz.** Only clearing **bit 0** (mask → 0,
or any even value) removes the ~2000 frames/s.

### 13.3 ⚠️ Therefore "so far no issue" is not yet attributable
If the mask still has bit 0 set, the bus load is **unchanged at ~2553 frames/s** and the quiet period
is a short observation window, not a fix. ⛔ Do not close this on absence-of-faults over minutes —
that is exactly the "never read a quiet topic as evidence" trap.

✅ **Settle it with the ruler, not the silence:** read `can_interface_status.frames_rx` twice and
compute the rate.
- **~2553 frames/s** ⇒ nothing changed; RTData still on; clear **bit 0**.
- **~600 frames/s** ⇒ RTData is off and §12.5 is confirmed.

---

## 14. ⛔ §13 WAS WRONG — "4 and 5" are ESC INDICES, not status messages

**Operator: "4 and 5 are can index i removed now so far no issue."** §13 answered a question that was
never asked (VESC status-message bits) and should be disregarded on that point. Its one durable
finding stands and is worth keeping: **`can_status_msgs_r1` bits 1–5 are inert while `can_mode = 1`**
(`comm_can.c:1513` gates the whole native-status path on `CAN_MODE_VESC`), and **bit 0 is the only
bit that matters** because `canard_driver.c:1459` uses it for RTData without any `can_mode` gate.

### 14.1 What was actually removed
Two ESC entries reporting `uavcan_esc_index` **4** and **5** — beyond the four real motors (0–3).
Recall §11.3: the repo already showed index drift (`uavcan_esc_index = 7` on id 11, `3` on id 13), so
indices on this vehicle were demonstrably not being kept to 0–3.

### 14.2 ✅ Confirmed removed, and confirmed quiet
- Live DDS `/fmu/out/esc_status`: **`esc_count = 4`, `esc_online_flags = 15`** = bits 0–3 only.
  Indices 4 and 5 are gone.
- MAVFTP listing of `/fs/microsd`: **no `fault_*.log` present** ⇒ nothing has hardfaulted since.
  (§7.3: a hardfault ALWAYS commits one.)

### 14.3 ⚠️ Timeline caveat — the one ULog cannot corroborate this
`10_25_08.ulg` (§9.2) also shows `esc_online_flags = 15`, i.e. **indices 4/5 were NOT present during
the nine logged faults** (all ≤ 10:31:58). So either they were introduced after ~10:31 and caused the
LATER faults — including the one whose log was deleted — or the log simply predates them.
🔑 **Do not claim the nine logged faults are explained by indices 4/5.** The evidence for 4/5 is the
operator's A/B, not the ULog.

### 14.4 🔑 This is the SAME mechanism as §12, and they reinforce each other
Extra ESC indices = extra nodes broadcasting **EscStatus + RTData** — ~13 frames per 50 Hz cycle each
(§12.2). Two extra streams ≈ **+1300 frames/s** on top of the measured 2553, i.e. ~3900 frames/s and
~50% bus load at 1 Mbit. Removing them removes that load from `wq:uavcan` — the task with the
**3600-byte stack** that faulted most (§10.3). **Root cause remains CAN load on `wq:uavcan`;
indices 4/5 were the increment that pushed it over.**

### 14.5 ⚠️ Still exposed — headroom, not a clean fix
If RTData (bit 0) is still enabled, the bus is **back to ~2553 frames/s / ~33% load**, which is where
it sat when the nine logged faults occurred. The margin has been restored, not the cause removed.
✅ **Recommended:** also clear **bit 0** (§12.5) → ~600 frames/s / ~8%, or move RTData to the r2
schedule at 5 Hz.
✅ **Verify with the ruler:** `can_interface_status.frames_rx` sampled twice. ~2553 = RTData still on.
✅ **Keep ESC indices strictly 0–3** and fix `uavcan_esc_index = 7` on id 11 (§11.3) before it returns.

---

## 15. RESOLVED reading of "4 and 5" — and what it implies about `can_mode`

**Operator supplied the VESC Tool selector: Status 1 = RPM/Current/Duty · 2 = Ah · 3 = Wh ·
4 = TempFET/TempMotor/CurrentIn/PIDpos · 5 = VoltageIn/Tachometer · 6 = ADC/PPM.
"i selected 4 and 5 earlier from this."**

⇒ **§13's reading was correct; §14's reversal was wrong.** "4 and 5" = VESC **status messages**,
i.e. `can_status_msgs_r1` bits 3 and 4. §14.1 ("ESC indices") is withdrawn; §14.2–14.5 (verification,
timeline caveat, RTData headroom) remain valid.

Mask arithmetic matches the repo exactly:
| mask | bits | messages | where seen |
|---|---|---|---|
| `9` | 0,3 | Status **1, 4** | most app configs |
| `25` | 0,3,4 | Status **1, 4, 5** | `vesc_appconf_Aug_2026.xml` (id 13, rear-left) |
| `1` | 0 | Status **1** only | after the operator's removal |

### 15.1 🔑 The deduction: at least one ESC is probably NOT in UAVCAN mode
Both native-status threads are gated on `can_mode`:
```c
comm_can.c:1513   if (conf->can_mode == CAN_MODE_VESC) send_can_status(conf->can_status_msgs_r1, id);
comm_can.c:1538   if (conf->can_mode == CAN_MODE_VESC) send_can_status(conf->can_status_msgs_r2, id);
```
There is no other native-status transmit path. **So with `can_mode = 1` (UAVCAN), Status 1–6 are
never sent, whatever the mask says.**

⇒ If removing Status 4 and 5 genuinely changed behaviour, then **at least one ESC is running
`can_mode = 0` (CAN_MODE_VESC)**, not UAVCAN. That is exactly the gap flagged in §11.2: **the app
configs for right-front (id 10) and rear-right (id 12) are NOT in the repo**, and one repo config
(`vesc_appconf_Left_Front.xml`) does carry `can_mode = 0`.

### 15.2 ⚠️ Why a mixed-mode bus would be worse than its frame count suggests (HYPOTHESIS)
VESC-native frames use extended IDs of the form `controller_id | (packet_type << 8)` — nothing like
DroneCAN's node-id/data-type layout. To PX4's libuavcan every such frame decodes as a transfer from
some arbitrary source node and data type, and **each distinct (node, type) pair can allocate transfer
reassembly state from libuavcan's FIXED memory pool.** Pool exhaustion / allocation failure in
`wq:uavcan` is a far better fit for memory corruption than raw frame count alone — and enabling
Status 4 and 5 adds more distinct bogus packet types.
⚠️ **Not verified.** Stated as the leading mechanism to test, not as fact.

### 15.3 ⏭ The check that settles it
**Read `can_mode` off all four ESCs.** Any ESC not at `1` (UAVCAN) is emitting VESC-native frames
that PX4 must chew on for no benefit.
1. Set **all four** to `can_mode = 1`, all `can_baud_rate = 3` (1 M), ids **10/11/12/13** distinct,
   `uavcan_esc_index` **0–3** distinct (fix the `= 7` on id 11, §11.3).
2. Then `can_status_msgs_r1` is irrelevant for the native path — but **bit 0 still drives RTData**
   (§13.2), so clear it too for headroom (§12.5).
3. **Verify with `can_interface_status.frames_rx`**, not with silence.

---

## 16. 🔴🔴 2026-08-18 — IT IS NOT FIXED. Two new hardfaults tonight, and RTData is still on

One MAVLink shell session (the only one spent this FC boot), two reads. Both answered.

### 16.1 The faults never stopped — §14.2's "confirmed quiet" is WITHDRAWN
`ls /fs/microsd` returned **two fault logs dated today**:
```
fault_2026_08_18_17_07_36.log
fault_2026_08_18_17_08_51.log
```
🔑 **Three independent rulers put them at the same moments, so the FC-clock offset is settled: the
filenames are UTC, local is IST = UTC+5:30.**

| fault file (FC clock, UTC) | ⇒ IST | `microxrce-agent` session re-established | lag |
|---|---|---|---|
| `17:07:36` | **22:37:36** | **22:37:40** | +4 s |
| `17:08:51` | **22:38:51** | **22:38:57** | +6 s |

Third ruler: `can_interface_status.timestamp` = 463.3 s since FC boot at the time of the read,
putting boot at **~22:38:2x IST** — i.e. the FC currently running is the one that came up after the
*second* fault. ⇒ **Two hardfault-reboots, 75 s apart, ~40 min after power-on.**

⛔ **The `esc_status`-based "confirmed quiet" of §14.2 is withdrawn.** It was a two-day-stale
observation, and §13.3's warning against closing this on absence-of-faults was correct.
⚠️ The vehicle was **disarmed and stationary** for both, with motor CAN connected. Consistent with
§10.2's exposure-time reading: it faults when it sits powered up with CAN attached, not when driven.

### 16.2 ✅ The ruler, applied at last: RTData is STILL ON
`listener can_interface_status -n 10`, `io_errors: 0` on every sample:

| | measured 08-18 | §12.2 measured 08-16 |
|---|---|---|
| span | 0.918374 s (10 samples) | 0.92 s (10 samples) |
| **`frames_rx`** | **2594.8 /s** | 2553 /s |
| `frames_tx` | 55.5 /s | 126 /s |
| bus load @ 1 Mbit | **33.7 %** | ~33 % |
| `io_errors` | **0** | 0 |

🔑 **Per §13.3's own decision rule — ~2553 ⇒ RTData still on, ~600 ⇒ fixed — this is 2594.8. Nothing
changed. Bit 0 of `can_status_msgs_r1` was never cleared.**

🔑 **The per-cycle arithmetic confirms it independently:** 2594.8 ÷ (4 ESCs × 50 Hz) = **12.97
frames per ESC per cycle** = `EscStatus (14 B → 3 frames) + RTData (68 B → 10 frames) = 13`, the
exact signature of §13.1. There is no room in that number for anything else — so the ~2000 frames/s
of RTData (≈78 % of the bus) is still being dispatched to a message ID **PX4 does not subscribe to**.

### 16.3 ⇒ What this settles
✅ **§14.5 was right and is now demonstrated, not predicted:** removing VESC Status 4 and 5 restored
*margin*, it did not remove the cause. The bus sits at the same ~33 % load it had during the nine
logged faults, and it faults there.
✅ **§12 (CAN load on `wq:uavcan`) survives every test put to it** and remains the root cause.
⛔ **Companion-side flooding is dead as a hypothesis — re-measured 08-18, independently of §8.1.**
Only `mavlink-routerd` speaks MAVLink (the QGC path; zero clients connected at the time); no
pymavlink/MAVProxy/MAVSDK/MAVROS, no leaked shell, no cron/timer. On DDS the sole `/fmu/in/*`
publisher is `autonav_mode`, measuring **0 msgs in 12 s** on every setpoint topic; only
`arming_check_reply_v1` moves, at 3.4 Hz, and that rate is set by the FC's own request.

### 16.4 ⚠️ One confound specific to tonight, and its limits
The operator has an **old DC-DC converter fitted for testing**, and the Pi's rail was logging
`throttled=0x50005` (under-voltage *and* throttling live) with dips to 4.58 V, including at
**22:38:29–33** — seconds before the second fault.
⛔ **This does NOT revive §8.2.** That theory died on §10.1+§10.2 (faults require a CAN harness that
carries no power conductor), and the Pi's 5 V is a **separate feed** from the FC's (§6). The rail is
a confound for *tonight's two faults only*; it cannot explain the nine from 08-16, when the rail
logged `0x0` all day.
✅ **The 2594.8 frames/s measurement is wholly independent of the converter** and stands regardless.
⏭ Re-check for new `fault_*.log` once the original converter is back, to close this out cleanly.

### 16.5 ⏭ Next — unchanged from §12.5/§15.3, now with evidence behind it
1. 🔴 **Clear bit 0 of `can_status_msgs_r1` on all four ESCs** (mask → even value), or move RTData to
   the r2 schedule at 5 Hz. Expect **2594.8 → ~600 frames/s**, 33.7 % → ~8 %.
   🔴 **TRAP (§12.5):** `conf_general.c:2254` force-sets the mask back to `0b00001111` during CAN
   ID assignment — re-verify after any ID reassignment.
2. **Read `can_mode` off all four ESCs**; any not at `1` (UAVCAN) emits native frames PX4 chews on
   for nothing (§15.1 predicts at least one is at `0` — the id 10 and id 12 configs missing from the
   repo, §11.2).
3. **Verify with `frames_rx`, not with silence.** Method: `listener can_interface_status -n 10`,
   Δ`frames_rx` ÷ Δ`timestamp`. ⚠️ One shell session per FC boot — never retry an empty result.
4. ⏭ **Pull `fault_2026_08_18_17_07_36.log` and `..._17_08_51.log` over MAVFTP** (recipe in §5) and
   check the task and User-stack block. §12/§10.3 predict `wq:uavcan` at **3600/3600**. Not yet done.

### 16.6 ✅ §15.1 DISPROVEN — all four ESCs ARE in `can_mode = 1`
**Operator, 2026-08-18: "i reverted all the 4 and 5"** (all four ESCs, not a subset).

🔑 **The frame count settles this without touching VESC Tool.** §16.2 measured **12.97 frames per ESC
per cycle** = `EscStatus (3) + RTData (10) = 13` exactly. An ESC running `can_mode = 0` would emit
VESC-native status frames *in addition* to the DroneCAN pair — one such ESC sending two native
frames per 50 Hz cycle would read as ~13.5/cycle. **There is no room in 12.97.**

⇒ ⛔ **§15.1's deduction ("at least one ESC is probably NOT in UAVCAN mode") is WITHDRAWN.** Its
premise was "if removing Status 4 and 5 genuinely changed behaviour" — and §16.1 shows it did **not**
(two faults after the change). Both halves now agree: no ESC is in `can_mode = 0`.
⇒ ⛔ **§15.2's mixed-mode / libuavcan pool-exhaustion hypothesis is moot** — it required a
`can_mode = 0` ESC to exist. Do not work it.
⇒ ✅ **§15.3 step "read `can_mode` off all four ESCs" is ANSWERED — drop it.** Do not spend a VESC
Tool session on it.

🔑 **This also explains why every change so far has failed to fix anything.** `can_mode = 1` gates
the native-status path (`comm_can.c:1513`), so the mask bits 1–5 were always inert — the FC swap,
the power-module swap, the ESC index work and now Status 4+5 all left the one ungated path
untouched. **`canard_driver.c:1459` reads bit 0 for `sendRtData()` with NO `can_mode` gate.**

⇒ **The single remaining action is to clear bit 0** (§12.5/§16.5). Everything else has been
eliminated by measurement.

---

## 17. 🔴🔴 2026-08-19 — The fault logs finally read: it is STACK EXHAUSTION, dominantly `uxrce_dds_client`, and it faults on BOTH firmware versions

### 17.1 What is on the card now — and what was lost
MAVFTP listing 08-19 (recipe §5, all three pulled, `END Fault Log` trailer verified — note the
trailer is **`END`, uppercase**; `fc_fault_backup.py`'s `End Fault Log` marker never matches and
must be fixed along with §5's `list_result` bug):
```
fault_2026_08_18_18_09_36.log   44793 B   (= 23:39:36 IST)
fault_2026_08_18_18_16_14.log   44793 B   (= 23:46:14 IST)
fault_2026_08_18_18_17_26.log   44763 B   (= 23:47:26 IST)
```
⛔ **§16.1's two logs (`17_07_36`, `17_08_51`) are GONE from the card** — presumably wiped during
the firmware-restore work. They were never pulled. Lost. The three above are in `~/fc_faults/`.

### 17.2 The three new faults — all three user stacks 100 % FULL
| log (UTC) | task | fault | cfsr | user stack |
|---|---|---|---|---|
| 18:09:36 | `uxrce_dds_client` | memfault (arm_memfault.c:101) | **0x82 = DACCVIOL+MMARVALID** | **0x1ec0/0x1ec0 FULL** |
| 18:16:14 | `uxrce_dds_client` | memfault (arm_memfault.c:101) | **0x82 = DACCVIOL+MMARVALID** | **0x1ec0/0x1ec0 FULL** |
| 18:17:26 | `wq:INS0` | imxrt_irq.c:272 | 0x00080000 = UFSR NOCP | **0x1758/0x1758 FULL** |
IRQ stack ~0x7c/0x800 in all three — interrupts are innocent.
🔑 **cfsr 0x82 on a task whose stack watermark reads exactly full is the textbook MPU stack-guard
hit: `uxrce_dds_client` OVERFLOWED ITS OWN STACK.** Its size 0x1ec0 = 7872 B =
`PX4_STACK_ADJUSTED(8000)` at `uxrce_dds_client.cpp:914`.

### 17.3 Re-reading ALL 12 logs by task (the 08-16 nine + these three)
`uxrce_dds_client` ×5 · `wq:uavcan` ×3 · `wq:INS0` ×3 · `mavlink_if1` ×1.
**11 of 12 show user stack used == size** (exception: 10:26:22, wq:uavcan at 0xc44/0xe10).
Stacks are heap-allocated and adjacent — one task's overflow scribbles into its neighbours, so
several tasks reading "full" does not need several independent overflows. The most frequent faulter
and the one caught red-handed on the MPU guard is **`uxrce_dds_client`**.

### 17.4 🔑 It faults on BOTH firmwares — the downgrade did not and can not fix it
- 08-16 nine faults: FW git-hash `f0889f3d` = **pxlabs-v1.17.0-2.0.0**.
- 08-18 three faults: FW git-hash `a52c38b07d` = **r2-Beta** — i.e. the "older restored" firmware
  had ALREADY hardfaulted three times the night it was restored.
Both builds carry the `esc_status` DDS topic (added `e8d3288637`, r2-Beta lineage). Upstream
corroboration that the client runs near its stack limit stock: PX4 issue #22323 reports
`uxrce_dds_client` stack FILLED > 93 %.

### 17.5 How this composes with §12 (CAN load) rather than replacing it
The RTData flood (§13) is real and measured; it loads `wq:uavcan` and feeds `esc_status` updates
into the DDS client at 100 Hz. But the *mechanism of death* now visible in the logs is stack
exhaustion, and the client overflows even while the vehicle sits disarmed. §12's "clear bit 0"
remains correct for bus margin; it is no longer expected to stop the hardfaults by itself.

### 17.6 ⏭ Actions (supersedes §16.5's ordering)
1. 🔴 **Raise the `uxrce_dds_client` stack**: `uxrce_dds_client.cpp:914`
   `PX4_STACK_ADJUSTED(8000)` → e.g. `PX4_STACK_ADJUSTED(10000)`, rebuild, flash. Pair with the
   queued `vehicle_angular_velocity` uncomment (dds_topics.yaml:60) — one flash, two items.
2. Still clear RTData bit 0 on all four ESCs (§12.5) — bus margin, and it halves the client's
   `esc_status` work indirectly.
3. After the flash: `uxrce_dds_client status` prints its live stack fill — read it once
   (⚠️ one MAVLink shell session per boot).
4. Fix `fc_fault_backup.py`: `list_result` (§5) AND the `END Fault Log` trailer case (§17.1).

### 17.7 🔴 2026-08-19 00:25 — OPERATOR VERSION HISTORY REWRITES §17.4, and 2.0.0 faults AGAIN
**Operator (08-19):** the 08-15 flash was **2.1.0 with a STALE version string** — built from an
UNCOMMITTED tree, so it reports the 2.0.0 hash `f0889f3d`. Verified: the `f0889f3d` COMMIT does not
contain 2.1.0's four topic additions (`vehicle_angular_velocity`, `rover_speed_status`,
`rover_attitude_status`, `rover_rate_status`) and its committed `dds_topics.yaml` is byte-identical
to `a52c38b07d`'s. ⇒ **hash `f0889f3d` in a fault log means "2.1.0 in disguise", NOT 2.0.0.**
⚠️ The exact 2.1.0 source is therefore UNRECORDED anywhere — it existed only as a dirty tree.
Operator also confirms prior swaps during debugging: FC (with §4's result), SD card, DC-DC module.

**Corrected fault ledger:** 08-16 nine + 08-18 two (lost) = **2.1.0**. 08-18 23:39/23:46/23:47 +
**08-19 00:25:20 IST (`fault_2026_08_18_18_55_20.log`, `wq:INS0`, stack 0x1758/0x1758 FULL, NOCP)**
= **2.0.0** (`a52c38b`), the build that ran ~2 months clean before 08-15.
⚠️ 00:25 log pulled PARTIAL (22466/44778, header intact); FC up ~32 min at fault — §10.2's
exposure signature again. Rail log 23:35→00:25: min 4.94 V, throttled 0x0 — §16.4 confound absent.

🔑 **A binary that was stable for two months now faults 4× in 50 min ⇒ the trigger is
ENVIRONMENTAL, not the firmware version.** The operator's "suspect the companion" is half-right:
companion *MAVLink/DDS spamming* stays eliminated (§16.3), but the environment did change —
prime suspect is the **ESC RTData flood (§13)**: `conf_general.c:2254` force-sets the status mask
during CAN-ID assignment, and VESC config work preceded the fault era. If the flood began then,
it explains BOTH firmwares faulting now AND two clean months before.
⏭ **Discriminating test, do FIRST: clear bit 0 on all four ESCs (§12.5) and let the rover sit
powered ≥60 min disarmed, CAN attached.** Faults stop ⇒ flood was the trigger. Faults continue ⇒
raise the client stack (§17.6.1) and re-examine. Ask the operator WHEN the VESC CAN-ID/status-mask
work was done — it dates the flood's start.

### 17.8 2026-08-19 — Operator: CAN side is at its clean-era state ⇒ RTData flood EXONERATED as trigger
Operator confirms all VESC settings were restored to their old state; measured bus rate after the
restore is 2548.3 frames/s = unchanged. ⇒ **the flood existed throughout the two clean months and
cannot be what changed on 08-15/16.** §12's load framing stays true as *margin*, not trigger.
⛔ Per operator decision: no further VESC/CAN work in this investigation.
**Remaining un-eliminated deltas vs the clean era:** which FC is fitted (original vs §4
replacement) · which SD card (new one fitted during debugging?) · harness/connector state after
all the swaps · companion-side DDS interaction profile (grew all August).
⏭ **Next discriminator — the agent-off soak:** stop `microxrce-agent`, leave the FC powered,
disarmed, CAN attached ≥75 min (every fault to date happened with an ACTIVE DDS session; without
one the client idles shallow). Card check afterwards works over MAVLink FTP (routerd path,
independent of DDS). Fault with no agent ⇒ companion fully exonerated, FC-internal. No fault ⇒
DDS-session interaction implicated ⇒ raise the client stack (§17.6.1) and retest.

### 17.9 🔴🔴 2026-08-19 00:42 — FAULT LOOP ~every 3 min, every fault inside MY activity window; VESC closed WITH EVIDENCE; 4 logs lost
**VESC closed properly:** repo `PXLABS_BLDC_VESC6_MK5` `Motp_Config_Bldc` — `can_status_msgs_r1`
is **ODD in every dated app config Feb→Aug** (9 = bits 0,3 on Feb/Apr/Aug-FL; 25 = bits 0,3,4 on
Aug-id13). **Bit 0 (RTData) has been set since at least 06-Feb ⇒ the flood ran through the two
clean months ⇒ NOT the trigger. Operator was right; closed on evidence, not assumption.**

**Tonight's full fault ledger (all on 2.0.0 `a52c38b`, all disarmed):** 23:39:36 · 23:46:14 ·
23:47:26 · 00:25:20 · **00:29:46 · 00:29:51 (5 s after!) · 00:32:23 · 00:35:59** IST.
⚠️ The last four were DELETED UNPULLED (operator asked for pull-then-delete; four new logs landed
between the pull and the delete and the delete took the whole card). Headers lost, timestamps kept.
🔴 **Fix the tooling: delete must re-list and pull-verify EACH file in the same pass, never
delete from a stale listing.**

🔑🔑 **The correlation that reframes everything: all 8 faults tonight fall inside the windows where
THIS Claude session was actively talking to the FC** — DDS topic probes (~23:36-40), MAVFTP pulls,
the one mavlink shell session (~00:29-30), FTP re-pull (~00:32-36), FTP list+delete (~00:40-42).
During the two clean months no such sessions existed; the 08-15/16 fault era coincided with heavy
Claude FTP/shell debugging (the §16 mavlink-session-exhaustion night!). §16.3 eliminated *steady*
companion traffic — it never tested *bursty MAVLink FTP/shell + DDS probe* load. mavlink_if1 was a
faulting task on 08-16.
⇒ **Working hypothesis: companion DEBUG-TOOLING interaction (MAVLink FTP burst reads, shell
sessions, DDS probes) is the environmental delta.** Operator's companion suspicion sharpened, not
refuted.

**Agent-off soak running since 00:38:47** (CH10 read 1011=safe before stop). ⚠️ First ~4 min
contaminated by the FTP deletes; **hands off the FC completely until the 01:53 check.** Card is
now EMPTY ⇒ any file at check time is a new fault with its own timestamp. If faults continue with
agent off AND no MAVLink activity ⇒ FC-internal after all. If quiet ⇒ session-interaction load
implicated (DDS and/or MAVLink; separate them next).

### 17.10 2026-08-19 — Fault #9 (00:42:09 IST, `fault_..._19_12_09.log`, pulled+verified) BREAKS THE STACK PATTERN
During the FTP-delete burst (~00:40-42, agent already OFF): task **`hpwork`**, user stack
**0x11c/0x6d0 = 16% — NOT full**, r0-r3/r10 = 0xffffffff, cfsr 0x82 with mmfsr/bfsr reading
0xffffffff, **hfsr 0x40000000 = FORCED hardfault**. First fault with a healthy stack and trashed
registers ⇒ **§8's corruption signature, not stack overflow. The full-stack readings elsewhere may
be symptoms of the same corruption, not independent overflows.**
⇒ DDS NOT required for faulting (agent was off). MAVLink FTP burst was active at the moment —
companion-interaction correlation now 9/9 tonight. No fault since 00:42:09 under true silence.
⏭ Soak continues to 01:53 check. If quiet ⇒ bursty companion interaction (MAVLink path included)
is the trigger; next separate FTP-only vs DDS-only. If faulted ⇒ FC-internal.
⚠️ QGC via mavlink-routerd stayed connected all night (as in the clean months) — acceptable
control, but a routerd-off soak is the follow-up if the quiet soak still faults.

### 17.11 2026-08-19 ~01:00 — Session wrap: every hardware suspect eliminated; ONE survives
**Hardware ledger (operator, 08-19):** FC #2 + new SD were fitted together YESTERDAY (08-18).
⇒ the 08-16 nine faults ran on FC #1 + OLD SD. **Three boards fault identically (FC #1, §4 loaner,
FC #2); both SD cards fault; firmware rolled back; VESC restored; rails clean. ALL eliminated.**
**Sole surviving suspect: companion debug-interaction** (MAVLink FTP bursts, shell sessions, DDS
session churn) — 10/10 faults tonight inside interaction windows; began 08-15/16 WITH the
debugging campaign (08-16 = the shell-exhaustion day, correlation never checked then); did not
exist in the two clean months.
**Upstream corroboration (08-19):** PX4 #22160 — fmu-v6xrt hardfault during MAVLink-FTP log
download, "seems SD related", OPEN · #27463 — ungraceful companion disconnect hardfaults 6X/6X-RT
(release/1.17) · draft fix PR #27108 names the mechanism: session-teardown/reconnect races,
threads touching destroyed resources · NXP RT1176 XIP fetch-corruption class matches §8.
**Fault #10 ledger for the night** (all 2.0.0, all pulled+verified except the 4 lost): 23:39:36 ·
23:46:14 · 23:47:26 · 00:25:20 · [00:29:46 · 00:29:51 · 00:32:23 · 00:35:59 LOST] · 00:42:09
(hpwork, stack 16%, trashed regs — §17.10).
**State left overnight:** `microxrce-agent` STOPPED (00:38:47) — restart it tomorrow before any
DDS work. Card EMPTIED 00:46. Quiet soak running; 01:53 timer will do one FTP card-check and
record. FC uxrce_dds_client idles "Running, disconnected". CH10 was 1011 (safe).
⏭ **Tomorrow: (1) read the 01:53/overnight soak verdict here · (2) re-challenge = start agent +
deliberate FTP-burst loop, expect fault within ~45 min if hypothesis holds · (3) if confirmed,
mitigation = stop bursty FTP/shell against the FC (pull logs by SD removal or single throttled
sessions) + consider #27108-class fix / raise client stack when flashing next · (4) fix
`fc_fault_backup.py` (list_result + `END Fault Log` trailer); working puller saved to
`tools/ftp_pull_faults.py`.**

### 17.12 🔴🔴 2026-08-19 00:58:23 — SOAK VERDICT: FAULT WITH NO COMPANION AT ALL. §17.9-17.11's interaction hypothesis is DEAD
`fault_..._19_28_23.log` (pulled+verified): `wq:INS0`, imxrt_irq.c:272, NOCP, stack 0x1758 FULL,
**pc 0x301b551e / lr 0x300f2617 = EXACT same crash site as the 00:25:20 fault.**
Conditions: agent STOPPED 20 min · last companion FTP touch 00:46:02 = **12 min of total silence
before the fault** · only QGC steady telemetry (present in the clean months too).
⛔ **The 10/10 interaction correlation was a BASE-RATE TRAP** — with faults every 3-6 min and
near-continuous debugging, overlap was guaranteed. The soak was the controlled test and it
falsified the hypothesis in one arm. **Companion FULLY exonerated (steady §16.3 + bursty here).**
🔑 The recurring identical crash site in wq:INS0 (2×, plus its 3 earlier NOCP faults) points at a
deterministic FC-side path, not random external disturbance.
**Surviving candidates (both dated 08-15, both survive every swap):**
1. 🔴 **PARAM CONTAMINATION SURVIVING THE ROLLBACK** — params live on FC storage; rolling firmware
   back does NOT roll params back. 2.1.0 changed rover modules (4 new topics) and its param
   migration ran on 08-15; the restored 2.0.0 now runs with a param set the clean era never had.
   wq:INS0 = EKF/INS queue is tonight's dominant faulter. ⏭ **TEST FIRST tomorrow: factory-reset
   params → load repo `PXlabs_Differential_Rover_NXP_tested_2026-05-29.params` (the tested
   clean-era set, in-repo since e8d3288637) → soak.** ⚠️ re-set UXRCE_DDS_CFG/serial after reset.
2. **Binary identity** — was the restored "2.0.0" the STORED `.px4` from the repo or a REBUILD?
   A rebuild is not bit-identical to the 2-month-clean binary. Ask the operator.
   Also date the power-module swap (with FC#2 on 08-18, or earlier?).
State: agent still stopped · card holds nothing unpulled (19_28_23 pulled; delete next session
after re-list) · 01:53 timer will fire harmlessly. Fault #11 of the night.

### 17.13 🔴🔴 2026-08-20 — Card cleared (6 pulled, 20 archived) · 3 NEW faults today with NO agent session driving them · ⛔ THE STACK-EXHAUSTION HEADLINE (§17.2/§17.3) IS FALSIFIED

**Cleanup done properly this time.** Pull-verify-delete with a **re-list every round** (the §17.9
rule, now implemented in `tools/ftp_pull_faults.py --delete`; its `End Fault Log` marker was also
case-fixed to `END`, closing §17.6.4's first half). Six logs came off the card, card verified
EMPTY, `~/fc_faults/` now holds **20 logs**. ✅ The re-list design paid for itself immediately:
`19_31_37` came down truncated **three times** (17208 / 14101 / 17686 B) and only completed on
round 4 — a single-pass tool would have deleted a partial backup.

**Card contents found (beyond §17.12's ledger):**
| log (UTC) | IST | task | note |
|---|---|---|---|
| `1970_01_01_00_01_10` | — | `wq:nav_and_controllers` | fault at **70 s uptime, no time source yet** ⇒ a boot with no GPS/MAVLink time. Almost certainly a reboot-after-fault that re-faulted immediately. |
| `2026_08_18_19_31_37` | 08-19 01:01:37 | `wq:uavcan` | fault #12 of that night, 3 min after §17.12's |
| `2026_08_20_17_08_11` | **08-20 22:38:11** | `uxrce_dds_client` | NEW |
| `2026_08_20_17_11_17` | **08-20 22:41:17** | `uxrce_dds_client` | NEW |
| `2026_08_20_17_14_41` | **08-20 22:44:41** | `uxrce_dds_client` | NEW |

**Conditions for today's three (recorded at 23:05, ~20 min after the last):** companion booted
22:13 · `microxrce-agent` **ACTIVE** (restarted 08-19 01:03:07 — §17.11/§17.12's "left stopped" is
STALE) · normal ROS stack up (camera, odometry, scan, rc_control, **vision_streaming running**) ·
`mavlink-routerd`/QGC connected · **NO Claude session, no FTP, no shell** (this session started
~23:00, after all three). FC uptime read 20.6 min at 23:05 ⇒ it rebooted out of the 22:44:41
fault and has been clean 20 min; card re-listed empty.
⇒ Faults occur under the ordinary operating stack with nobody debugging. §17.12's exoneration of
the companion *debug-tooling* holds and widens.
🔑 **Burst shape repeats: first fault ~25-32 min after power-on, then every ~3 min.** (Today
22:38 ≈ 25 min after companion boot; §17.7's 00:25 fault was ~32 min after FC boot; §10.2's
"exposure" signature.) Something ACCUMULATES or crosses a threshold — it is not a uniform-random
event. **Any soak shorter than ~40 min proves nothing.**

#### 17.13.1 ⛔ The retraction: nothing ever faulted *while its stack was deep*
Parsing all 20 logs for `size` vs `used` (the coloring watermark) vs **sp depth at the fault**
(`top - sp`, i.e. what was ACTUALLY in use when the CPU died):

| | uxrce_dds_client (n=7) | wq:INS0 (n=4) | wq:uavcan (n=4) | others (n=5) |
|---|---|---|---|---|
| stack size | 7872 | 5976 | 3600 | 1744-3056 |
| watermark `used` | FULL ×7 | FULL ×4 | FULL ×3, 87% ×1 | FULL ×4, 16% ×1 |
| **sp depth at fault** | **320-1152 B (4-15%)** | 816-3408 B | 824-936 B | 152-1152 B |

🔑🔑 **A genuine stack overflow faults AT THE INSTANT sp crosses the guard — sp would sit at or
below the stack bottom, i.e. sp depth ≈ size. It does so in NONE of the 20 logs.** The DDS client
died with **4-15 % of its stack in use**. So §17.2's "textbook MPU stack-guard hit:
`uxrce_dds_client` OVERFLOWED ITS OWN STACK" and §17.3's "11 of 12 stacks full ⇒ exhaustion"
are **WITHDRAWN**: `used` is watermark-only and independent of sp (proven by the 16 % and 87 %
outliers — the field does report partial values when a stack genuinely wasn't dirtied). A
watermark reading full means **the colouring pattern was destroyed all the way down**, which is
what §17.10 already concluded from fault #10 alone. It is true of essentially every log.
⇒ **PX4 #22323 ("client stack filled >93 %") is NOT our fault mode, and raising the client stack
(§17.6.1) is not expected to fix anything.** Keep it only as free headroom on the next flash.

#### 17.13.2 🔑 What the fault classes actually say: instruction-side corruption
Decoding `cfsr` across all 20 (UFSR = bits 16+, BFSR = 8-15, MMFSR = 0-7):
- `0x00080000` **NOCP** ×5 — coprocessor/FPU access denied
- `0x00000082` **DACCVIOL+MMARVALID** ×6 — data access violation
- `0x00010000` **UNDEFINSTR** ×4 — *the CPU fetched something that is not an instruction*
- `0x00020000` **INVSTATE** ×2 — both with **`pc = 0x00000000`** (`wq:nav_and_controllers`,
  `wq:uavcan`): branched through a **null/garbage function pointer**
- `0x00000100` **IBUSERR** ×1 — instruction-bus fetch failure · `0x00000400` IMPRECISERR ×1

**UNDEFINSTR + IBUSERR + INVSTATE + pc=0 are instruction-side failures**, spread across six
unrelated tasks. Deep stacks do not produce those; **corrupted code fetches and corrupted pointers
do.** Every faulting `pc` lies in `0x30xxxxxx` = the **FlexSPI XIP region** (RAM is `0x20xxxxxx`),
so the RT1176 is executing in place from external flash when it dies. This is exactly §8's
corruption signature and the **NXP RT1176 XIP fetch-corruption class** flagged in §17.11 — now
supported by 20 logs rather than one. Two further tells: the 08-20 17:14:41 log reports
`cfsr:0x82` but `mmfsr`/`bfsr` = `0xbbd7b352` (impossible — both are 8-bit fields), and fault #10
had them at `0xffffffff`: **the fault-capture struct itself is being trashed.**

#### 17.13.3 ⏭ Where this leaves the investigation
Unchanged and still first: **§17.12's param-contamination test** (factory-reset → load repo
`PXlabs_Differential_Rover_NXP_tested_2026-05-29.params` → soak **≥45 min**, re-set `UXRCE_DDS_CFG`
after). Params are one of the two things that survived every board/card/firmware swap.
What §17.13 adds to the target list, given the mechanism is corruption of code/pointers rather
than stack depth:
1. **FlexSPI/XIP integrity** — clock/voltage margin on the external flash, and whether the
   restored binary was a REBUILD vs the stored `.px4` (§17.12 item 2, still unanswered by the
   operator). A rebuilt image lays code out differently in XIP space.
2. **Power-module swap date** — still unanswered, and now more relevant: XIP fetch corruption is
   a classic rail-margin symptom, and §16.4's rail log only covers one night.
3. ⛔ Do NOT spend the next flash on the client stack alone; pair any flash with the queued
   `vehicle_angular_velocity` DDS uncomment (`dds_topics.yaml:60`).
✅ Tooling: `ftp_pull_faults.py` now does list→pull→verify→delete per round with re-listing and
refuses to delete anything whose backup lacks the `END Fault Log` trailer. `fc_fault_backup.py`
remains broken and unused — delete it or fix it, don't reach for it.

### 17.14 ✅✅ 2026-08-20 — CAN ELIMINATED BY REMOVAL: the faults happened with NO BUS PRESENT AT ALL
**Operator (08-20): all CAN interfaces were removed from the motor controllers BEFORE tonight's
boot.** Confirmed from PX4's side at 23:0x — `/fmu/out/esc_status` declared but **zero messages**
against a live baseline (`/fmu/out/input_rc` **100.2 Hz**).
⇒ **The three faults at 22:38:11 / 22:41:17 / 22:44:41 IST occurred with no CAN bus in the vehicle
at all.** §12 (RTData flood), §13, §14 and the whole CAN-load family are now **DEAD AS A TRIGGER BY
REMOVAL**, not merely by the config-history argument of §17.8/§17.9. ⛔ **Do not re-open CAN.**
The §12 load framing survives only as *margin* for whenever the bus is reconnected.

🔑 **VICTIM-SHIFT — do not misread it as progress.** All three of tonight's faults are
`uxrce_dds_client`; **`wq:uavcan` did not fault once**, which is exactly what an idle CAN stack
should do when there is no bus to service. The set of faulting tasks tracks **which tasks are
BUSY**, not one defective task — further support for §17.13's generic-corruption reading over any
per-task bug. Expect future logs to be dominated by the DDS client purely because it is now the
busiest thing running; **a change in *which* task faults is a change in the victim population, not
evidence about the cause.**

**Status at 23:26:** FC uptime **41.2 min**, card empty ⇒ clean since the 22:44:41 fault, which is
past §17.13's 25-32 min first-fault window and the longest quiet stretch tonight. ⚠️ **Not yet
meaningful** — 08-18/19 showed the same burst-then-gap shape, and §17.12's fault landed ~20 min
into a soak. Needs ≥60 min clean before it counts as anything.

**Suspect list after this elimination** (everything else in §17.11 was already cleared — 3 FC
boards, both SD cards, firmware rollback, rails, and the companion in both steady and bursty form):
1. 🔴 **PARAM CONTAMINATION** (dated 08-15, survives every swap) — still the prime suspect and
   still the next test. ⚠️ **Needs operator sign-off before running: a factory param reset is not
   rover-only — this FC is shared with the DRONE**, and the drone needs its own `EKF2_*` set. Plan
   the restore (repo `PXlabs_Differential_Rover_NXP_tested_2026-05-29.params`, then re-set
   `UXRCE_DDS_CFG`/serial) before wiping anything.
2. **Binary identity** — restored 2.0.0 = the stored `.px4`, or a REBUILD? Still unanswered.
3. **Power / FlexSPI-XIP margin** — power-module swap date still unanswered; §17.13's
   instruction-fetch signature makes this more relevant than it looked.

### 17.15 ✅✅ 2026-08-20 — Suspects 2 and 3 ANSWERED by the operator: PARAM CONTAMINATION IS THE LAST ONE STANDING
**Operator (08-20), answering §17.12/§17.14 directly:**
1. **Binary identity — ELIMINATED.** The restored build is the **old, custom-built, known-good
   version that ran ~3 months with no issue** — not a fresh rebuild of uncertain provenance.
   §17.12 item 2 and §17.14 suspect 2 are closed: **the binary is not the variable.**
2. **Power module — swapped MONDAY 2026-08-17** ("from Monday we're checking with a new power
   module"). 🔑 **That is one to two days AFTER the fault era began** (2.1.0 flashed Sat 08-15;
   the nine faults ran Sun 08-16). ⇒ **it CANNOT be the trigger** — and since ~12 further faults
   have landed on 08-18, 08-19 and 08-20 with it fitted, **the new module did not fix it either.**
   Fault ledger by module: **9 faults on the OLD module (08-16) · 12+ on the NEW (08-18 →).**

⇒ 🔴🔴 **EVERY OTHER CANDIDATE IS NOW ELIMINATED BY TEST OR BY DATE.** Three FC boards · both SD
cards · the firmware version (rolled back) · **the firmware binary itself (3 months clean)** ·
VESC/CAN (removed entirely, §17.14) · **both power modules** · the companion in steady (§16.3) and
bursty (§17.12) form · the rails as far as they have been measured.
**The ONLY delta left that is dated 08-15 and survives every swap is the PARAMETER SET** — 2.1.0's
migration ran on 08-15, params live in FC storage, and **rolling firmware back does NOT roll params
back**. `wq:INS0` (the EKF/INS queue) being a repeat faulter fits a contaminated EKF param set.
⏭ **This is now the whole investigation. Next session: the param reset test (§17.12), with the
operator sign-off §17.14 flagged — the FC is shared with the DRONE, so a factory wipe is not
rover-only and the drone's `EKF2_*` set must be planned for before anything is erased.**

#### 17.15.1 Rail measurement — what was taken, and the gap that remains
60 s at idle, 08-20 23:3x: **battery 25.081-25.092 V (n=301, 11 mV spread, mean 25.086)** — the
pack and the new module's sense are rock steady, consistent with §16.4/§17.7. ⚠️ But this is the
BATTERY, at idle, on an immobile rover; it is not the thing the XIP-corruption mechanism cares
about.
🔴 **The FC's OWN 5 V rail has never been measured and is currently NOT OBSERVABLE:** PX4 does not
stream `POWER_STATUS` (60 s of requesting the extended-status group returned nothing — it is
largely an ArduPilot message), and `system_power` is **not in `dds_topics.yaml`**. The only route
today is `listener system_power` in a MAVLink shell — ⛔ **do not spend the one-shell-per-boot on it
while the FC is in its longest clean stretch.**
⏭ **Add `system_power` to `dds_topics.yaml` alongside the queued `vehicle_angular_velocity`
(line 60) on the next flash** — one flash, three items, and it makes FC rail margin continuously
observable instead of permanently hypothetical.

### 17.16 2026-08-20 23:35:19 IST — fault #21: a SEVENTH victim task, and the quiet-gap figure that invalidates short soaks
`fault_2026_08_20_18_05_19.log` (pulled + `END Fault Log` verified, card re-listed clean).

| field | value |
|---|---|
| task | **`wq:ttyS3`** — a serial-driver work queue, **never seen faulting before** |
| site / cfsr | `arm_memfault.c:101` · `0x00000082` = DACCVIOL+MMARVALID |
| pc / lr | `0x300bb3e2` / `0x300bbc7f` — **XIP region**, as always |
| user stack | size `0x6a8` (1704 B), watermark **FULL**, **sp only `0x1e8` = 488 B = 29 % deep** |
| mmfsr / bfsr | **`0xffffffe4`** — impossible for 8-bit fields ⇒ **the fault-capture struct is trashed again** |

✅ **Every §17.13 claim holds on the 21st log:** watermark full while sp sits shallow (29 %),
instruction/data fault in XIP space, and the capture struct itself corrupted — the third log to
show that (with fault #10's `0xffffffff` and 08-20 17:14:41's `0xbbd7b352`).

🔑 **This is §17.14's victim-shift prediction coming true, not a new failure mode.** With CAN
removed the busiest things on the FC are the DDS client and the serial drivers — and the victim
duly moved to a serial work queue. Seven distinct tasks have now faulted (`uxrce_dds_client`,
`wq:INS0`, `wq:uavcan`, `wq:nav_and_controllers`, `mavlink_if1`, `hpwork`, `wq:ttyS3`).
**Nothing survives this except a mechanism that corrupts whatever is running.**
⚠️ Which link `ttyS3` serves is **NOT RECORDED ANYWHERE** (not in `reference_uart_map`, not in the
docs). Read back 08-20: `UXRCE_DDS_CFG=103` (**DDS on TELEM3**) · `MAV_0_CONFIG=101` (TELEM1) ·
`MAV_1_CONFIG=102` (TELEM2) · `GPS_1_CONFIG=201` (GPS1) — so `ttyS3` is one of those four, but the
ttySN→port mapping is board-specific and unconfirmed. ⏭ **Cheap add-on next time a MAVLink shell is
open anyway: confirm the mapping.** ⛔ Not worth burning the one-shell-per-boot on by itself.

#### 17.16.1 🔴 CORRECTION to §17.13's burst shape — short soaks prove even less than stated
Tonight's intervals, FC boot 22:44:41 (it rebooted out of the 22:44:41 fault):
`22:38:11 → 22:41:17` (+3.1 min) → `22:44:41` (+3.4 min) → **`23:35:19` (+50.6 min)**.
⇒ The "every ~3 min" figure describes **within a burst only**. A **50.6-minute quiet gap occurred
naturally, with no intervention**, and this fault landed at **50.6 min of uptime** — well outside
§17.13's "first fault ~25-32 min after power-on" window.
🔑🔑 **THEREFORE: a quiet stretch shorter than ~90 min is NOT evidence of a fix.** The 41-minute
clean run recorded in §17.14 looked encouraging and meant nothing — it was simply inside the
observed gap. **Set the param-reset soak (§17.15) to ≥90 min, and judge it on fault COUNT over a
fixed window, never on "it has been quiet for a while".**
