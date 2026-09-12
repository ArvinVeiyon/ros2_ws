#!/usr/bin/env python3
"""Shared rulers and constants for the field-measurement tools. Import, don't copy.

WHY THIS FILE EXISTS
  The address-10 ERPM sign map was copy-pasted into four separate files. When it
  went stale (2026-09-12) `/odom` and three tools all became wrong independently,
  and fixing it meant editing four places. `erpm_to_ms` has the same history: it
  sat at 0.000380 in one tool for months after being corrected everywhere else.
  One module owns these now. ⛔ If you find yourself re-declaring a constant that
  lives here, you are recreating that bug.

WHAT IS IN HERE
  * the ESC address map, sign map and ERPM->m/s scale, with their provenance
  * QoS profiles that actually work against PX4 over uXRCE
  * a rate ruler that uses a direct rclpy subscriber
  * a requirements gate: declare what a routine needs, get a specific refusal
  * segment/fit helpers shared by the measurement routines

🔴 THE RATE RULER IS NOT OPTIONAL. On this box `systemctl is-active` reads
  `active` while a node publishes nothing -- seen 2026-09-04 and again 09-12,
  when every unit was green with depth, /scan and /odom all at 0.00 Hz. And the
  ros2 CLI is itself unreliable: `topic hz` and `topic echo` have both returned
  nothing on a healthy topic. Measure with a subscriber or do not claim a rate.
"""
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

# --------------------------------------------------------------------------
# Drivetrain constants -- SINGLE SOURCE OF TRUTH. Keep in step with
# src/rover_odometry/rover_odometry/wheel_odometry_node.py.
# --------------------------------------------------------------------------

DRIVE_ADDRS = (10, 11, 12, 13)        # ESC addresses. 10=RF 11=FL 12=RR 13=RL
LEFT_ADDRS, RIGHT_ADDRS = {11, 13}, {10, 12}

# 🔴 ALL +1 SINCE 2026-09-12. Address 10 was carried as -1.0 on a note reading
# "only addr 10 reports inverted ERPM". Measured on stands in both directions
# over 38,396 samples, that is false: forward every address reports positive,
# reverse every address goes negative together, and the operator confirmed the
# wheels visually. The old -1.0 made the right side average ~9 ERPM instead of
# ~1500 -- it cancelled itself and /odom published about half the true speed.
WHEEL_SIGNS = {10: 1.0, 11: 1.0, 12: 1.0, 13: 1.0}

# Tape-validated. ⚠️ SPEED-DEPENDENT: /odom over-reads ~5% near 0.9 m/s and
# under-reads ~22% at crawl. ⛔ Do not extrapolate it to no-load ERPM (~1500);
# that is ~6x outside the range it was fitted over and conflicts ~10x with the
# 0.58-0.60 m/s drivetrain figure in setup_manual A7. Unresolved.
ERPM_TO_MS = 0.003900
TRACK_WIDTH = 0.31                    # m, tape-measured 2026-07-21
FRONT_OVERHANG = 0.337                # m, bumper ahead of the scan origin
SCAN_SCALE = 0.9845                   # /scan reads SHORT; verified at a wall 09-11

# --------------------------------------------------------------------------
# Physical dimensions -- from autonav_reference.md §4, which is the canonical
# table. ⛔ ITS OWN RULE: "Never derive a dimension — look it up." Two shipped
# bugs already came from assumed geometry (the wheelbase used as the track,
# under-reporting every yaw rate by 28%; and a footprint built from a guess).
# --------------------------------------------------------------------------

DIMENSIONS = {
    # name:            (value_m, provenance)
    'plate_length':    (0.730, 'autonav_reference §4 — longest extent, THE footprint'),
    'plate_width':     (0.450, 'autonav_reference §4 — wheels sit INBOARD of the plate'),
    'ground_to_plate': (0.235, 'autonav_reference §4'),
    'track':           (0.310, 'tape 2026-07-21, hub to hub — ⚠️ NOT the widest extent'),
    'wheelbase':       (0.430, '⚠️ NEVER use as track — that bug shipped once already'),
    'centre_to_tip':   (0.345, 'rotation centre to front plate tip; defines base_link'),
    'front_overhang':  (0.337, 'scan origin to bumper; confirmed to 2 mm by the tape fit'),
    'cam_z':           (0.305, '0.235 plate + 0.070 bracket'),
    'wheel_diameter':  (0.1524, '6 inch. ⛔ the VESC config says 0.083 and is WRONG — '
                                'documented as ignore-do-not-fix'),
}

# ⚠️ WIDTH — TRACED 2026-09-12, and it is 5 engineering sources against 1
# commercial one. 0.450 is carried by: the AutoNav Technical Reference artifact
# §4 (bc09dd55, rev 08-09, which states "wheels sit INBOARD of the plate" and
# calls the plate "the widest and longest extent"), autonav_reference.md §4 in
# the repo (an identical table), both Nav2 footprints (±0.225), the reflex's
# footprint_half_width 0.225, and the corridor half-width 0.275 derived as
# 0.225 + 50 mm. 0.560 appears in ONE place and its copies: the Solar-Farm
# Inspection UGV price quotation (artifact a1f7eb07, Rev 8, 2026-09-10),
# "Footprint 730 x 560 mm" — a commercial spec, quite possibly a product
# envelope rather than this plate. rover_autonav_requirements.md:69, MEMORY.md,
# todos.md:554 and project_rover_autonav.md:308 all repeat that figure.
# 🔑 The safety code already uses 0.450, so there is NO gap unless a tape says
# otherwise. ⏭ Still worth two tape numbers — the plate edge-to-edge, and the
# widest point of the whole vehicle — because if anything protrudes, THAT is
# what Nav2 and the reflex need. ⛔ Do not resolve it by counting documents.
DISPUTED_WIDTH = (0.450, 0.560)

NAV2_FOOTPRINT = [(0.345, 0.225), (0.345, -0.225), (-0.385, -0.225), (-0.385, 0.225)]

# --------------------------------------------------------------------------
# QoS
# --------------------------------------------------------------------------

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=5)

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST, depth=10)


def qos_for(topic):
    """PX4 topics need TRANSIENT_LOCAL; ROS sensor topics do not."""
    return PX4_QOS if topic.startswith('/fmu/') else SENSOR_QOS


# --------------------------------------------------------------------------
# ESC helpers -- key on ADDRESS, never on array position
# --------------------------------------------------------------------------

def esc_by_address(msg):
    """{address: (rpm_raw, current)} for the four DRIVE ESCs only.

    ⛔ `esc[]` is ALWAYS 8 entries long and slots 5 and 6 are BRAKES, not drive
    ESCs. Indexing by position picks up a phantom. Entries with timestamp == 0
    are unpopulated and are dropped.
    """
    out = {}
    for i in range(len(msg.esc)):
        e = msg.esc[i]
        addr = int(e.esc_address)
        if addr in DRIVE_ADDRS and e.timestamp != 0:
            out[addr] = (float(e.esc_rpm), float(e.esc_current))
    return out


def linear_speed(rpm_raw, signs=None):
    """Mean forward speed [m/s] from RAW per-address ERPM. None if a side is dark.

    A one-sided average is not a speed -- if only one side reports, say so rather
    than halving the answer silently, which is exactly the failure the stale sign
    map produced.
    """
    signs = signs or WHEEL_SIGNS
    left = [v * signs.get(a, 1.0) for a, v in rpm_raw.items() if a in LEFT_ADDRS]
    right = [v * signs.get(a, 1.0) for a, v in rpm_raw.items() if a in RIGHT_ADDRS]
    if not left or not right:
        return None
    return (sum(left) / len(left) + sum(right) / len(right)) / 2.0 * ERPM_TO_MS


def bumper_clearance(scan_min_range):
    """Bumper clearance [m] from a raw /scan minimum range."""
    return scan_min_range * SCAN_SCALE - FRONT_OVERHANG


# --------------------------------------------------------------------------
# Rate ruler
# --------------------------------------------------------------------------

class RateRuler(Node):
    """Counts messages on named topics with a direct subscriber."""

    def __init__(self, specs):
        super().__init__('rover_diag_rate_ruler')
        self.counts = {t: 0 for t, _ in specs}
        self.last = {t: None for t, _ in specs}
        for topic, msg_type in specs:
            self.create_subscription(
                msg_type, topic,
                lambda m, t=topic: self._hit(t, m), qos_for(topic))

    def _hit(self, topic, msg):
        self.counts[topic] += 1
        self.last[topic] = msg

    def run(self, duration):
        t0 = time.time()
        while time.time() - t0 < duration:
            rclpy.spin_once(self, timeout_sec=0.05)
        elapsed = time.time() - t0
        return {t: c / elapsed for t, c in self.counts.items()}


def measure_rates(specs, duration=5.0):
    """{topic: Hz} over `duration`. specs = [(topic, msg_type), ...]."""
    started = not rclpy.ok()
    if started:
        rclpy.init()
    node = RateRuler(specs)
    try:
        return node.run(duration)
    finally:
        node.destroy_node()
        if started:
            rclpy.shutdown()


# --------------------------------------------------------------------------
# Requirements gate
# --------------------------------------------------------------------------

class Requirement:
    """One thing a routine needs before it may run.

    `topic` + `min_hz` + `msg_type` for a live stream; `param` for a readable
    PX4 parameter. `why` is printed on failure -- a refusal that does not say
    what to do about it just moves the guesswork.
    """

    def __init__(self, topic=None, msg_type=None, min_hz=None, param=None, why=''):
        self.topic, self.msg_type, self.min_hz = topic, msg_type, min_hz
        self.param, self.why = param, why

    def label(self):
        return self.topic if self.topic else f'param {self.param}'


def check_requirements(reqs, duration=5.0, param_url='tcp:127.0.0.1:5760'):
    """Measure every requirement. Returns (ok, lines) -- never raises.

    🔑 Rates are measured, never inferred from `systemctl is-active`, which has
    read `active` on a completely dead chain more than once on this box.
    """
    lines, ok = [], True

    specs = [(r.topic, r.msg_type) for r in reqs if r.topic]
    rates = measure_rates(specs, duration) if specs else {}

    for r in reqs:
        if r.topic:
            hz = rates.get(r.topic, 0.0)
            good = hz >= (r.min_hz or 0.0)
            mark = 'ok  ' if good else 'FAIL'
            lines.append(f'  [{mark}] {r.topic:34s} {hz:7.2f} Hz '
                         f'(need >= {r.min_hz:.1f})')
            if not good:
                ok = False
                if hz == 0.0:
                    lines.append('         0.00 Hz — the node may read `active` and '
                                 'still publish nothing.')
                    lines.append('         Restart the CAMERA first, then rover-scan / '
                                 '-scan-3d / -odometry, in that order.')
                if r.why:
                    lines.append(f'         needed for: {r.why}')

    param_names = [r.param for r in reqs if r.param]
    if param_names:
        values = read_params(param_names, url=param_url)
        for name in param_names:
            val = values.get(name)
            good = val is not None
            lines.append(f'  [{"ok  " if good else "FAIL"}] param {name:28s} {val}')
            if not good:
                ok = False
                lines.append('         <no reply> is a WRONG NAME far more often than '
                             'a busy link — check the spelling against the firmware.')
    return ok, lines


# --------------------------------------------------------------------------
# PX4 parameters (READ ONLY -- this module never writes)
# --------------------------------------------------------------------------

_TYPE_INT = {1, 2, 3, 4, 5, 6}


def read_params(names, url='tcp:127.0.0.1:5760', timeout=2.0):
    """{name: value or None} over one MAVLink connection. Reads only.

    ⛔ Nothing in this toolchain writes a vehicle parameter. Routines SUGGEST a
    value and print it; applying it is the operator's decision, made explicitly.
    ⚠️ Parameters are not exposed over DDS at all, so MAVLink is the only route,
    and bulk PARAM_REQUEST_LIST was returning nothing on 2026-09-12 — named
    reads work, so this asks one at a time.
    """
    try:
        from pymavlink import mavutil
    except ImportError:
        return {n: None for n in names}

    out = {}
    try:
        link = mavutil.mavlink_connection(url)
        if not link.wait_heartbeat(timeout=15):
            return {n: None for n in names}
        for name in names:
            link.mav.param_request_read_send(
                link.target_system, link.target_component, name.encode(), -1)
            deadline, got = time.time() + timeout, None
            while time.time() < deadline:
                msg = link.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
                if msg and msg.param_id.strip('\x00') == name:
                    if msg.param_type in _TYPE_INT:
                        got = struct.unpack('<i', struct.pack('<f', msg.param_value))[0]
                    else:
                        got = round(msg.param_value, 6)
                    break
            out[name] = got
    except Exception:
        return {n: out.get(n) for n in names}
    return out


# --------------------------------------------------------------------------
# Analysis helpers
# --------------------------------------------------------------------------

def steady_segments(samples, key, tol, min_hold, settle=0.5):
    """Split samples into runs where `key` holds within +/- tol for min_hold [s].

    Returns the SETTLED tail of each run (last `settle` fraction). Samples need
    't' and the chosen key. ⚠️ A steady command is not a steady response — check
    the response separately, see is_settled().
    """
    segs, cur = [], []
    for s in samples:
        val = s.get(key)
        ok = val is not None
        if ok and cur and abs(val - cur[0][key]) <= tol:
            cur.append(s)
        else:
            if len(cur) > 1 and cur[-1]['t'] - cur[0]['t'] >= min_hold:
                segs.append(cur)
            cur = [s] if ok else []
    if len(cur) > 1 and cur[-1]['t'] - cur[0]['t'] >= min_hold:
        segs.append(cur)
    return [seg[int(len(seg) * (1 - settle)):] for seg in segs]


def is_settled(magnitudes, currents, drift_max=0.10):
    """True if the response held still and the motors were DRIVING.

    🔑 Negative mean current means the motor is regenerating — that is a
    spin-down, not a held level. Scoring those as rungs invents low-throttle
    points that were never commanded (seen 2026-09-12).
    """
    if not magnitudes:
        return False
    mean = sum(magnitudes) / len(magnitudes)
    if mean <= 1.0:
        return True                       # genuinely at rest: a valid data point
    drift = (max(magnitudes) - min(magnitudes)) / mean
    mean_amp = sum(currents) / len(currents) if currents else 0.0
    return drift <= drift_max and mean_amp >= 0.0


def linfit(points):
    """Least squares y = kx + c. Returns (k, c) or (None, None)."""
    n = len(points)
    if n < 2:
        return None, None
    sx = sum(p[0] for p in points)
    sy = sum(p[1] for p in points)
    sxx = sum(p[0] * p[0] for p in points)
    sxy = sum(p[0] * p[1] for p in points)
    den = n * sxx - sx * sx
    if abs(den) < 1e-12:
        return None, None
    k = (n * sxy - sx * sy) / den
    return k, (sy - k * sx) / n


def suggest(param, value, basis, caveats=(), px4_param=True):
    """Print a SUGGESTION. ⛔ Never writes. The operator applies it or does not.

    `px4_param=False` for anything that is NOT a writable FC parameter — a ROS
    node parameter, or a derived quantity like the friction deadband. Printing
    a `set_param.py` line for those invites someone to paste a command that
    cannot work.
    """
    print()
    print('=' * 70)
    print(f'  SUGGESTED: {param} = {value}')
    print(f'  BASIS    : {basis}')
    for c in caveats:
        print(f'  ⚠️  {c}')
    print('  ⛔ NOT APPLIED. Nothing here writes to the vehicle.')
    if px4_param:
        print(f'     To apply yourself: python3 tools/set_param.py {param} {value}')
        print('     (RAM-only; bldc_can/diag/param_save.py commits to flash,')
        print('      and INT32 params need bldc_can/diag/set_param_int.py)')
    else:
        print('     This is NOT an FC parameter — set_param.py cannot write it.')
    print('=' * 70)
