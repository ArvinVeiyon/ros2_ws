#!/usr/bin/env python3
"""Log per-wheel response while the vehicle is driven manually from the RC.

Read-only: never arms, never commands. Prints a line whenever the wheels or the
sticks change meaningfully, with the addr-10 ERPM sign inversion applied so the
numbers read as physical wheel direction.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import EscStatus, InputRc, VehicleStatus

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=5)

SIGN = {10: -1.0, 11: 1.0, 12: 1.0, 13: 1.0}   # addr 10 reports inverted ERPM
LEFT, RIGHT = {11, 13}, {10, 12}
ERPM_TO_MS = 0.000380
NAV = {0: 'Manual', 4: 'Hold', 23: 'AutoNav'}


class Log(Node):
    def __init__(self):
        super().__init__('manual_drive_log')
        self.addr, self.rpm = [], []
        self.ch = []
        self.nav = self.arm = None
        self.create_subscription(EscStatus, '/fmu/out/esc_status', self.esc, PX4_QOS)
        self.create_subscription(InputRc, '/fmu/out/input_rc', self.rc, PX4_QOS)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.st, PX4_QOS)

    def esc(self, m):
        n = m.esc_count
        self.addr = [m.esc[i].esc_address for i in range(n)]
        self.rpm = [m.esc[i].esc_rpm for i in range(n)]

    def rc(self, m):
        self.ch = list(m.values[:6])

    def st(self, m):
        self.nav, self.arm = m.nav_state, m.arming_state

    def phys(self):
        return {a: r * SIGN.get(a, 1.0) for a, r in zip(self.addr, self.rpm)}


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 180.0
    rclpy.init()
    n = Log()
    t0 = time.time()
    last_key = None
    peak = {}
    while time.time() - t0 < duration:
        rclpy.spin_once(n, timeout_sec=0.1)
        if not n.addr or not n.ch or n.nav is None:
            continue
        p = n.phys()
        for a, v in p.items():
            if abs(v) > abs(peak.get(a, 0)):
                peak[a] = v
        # quantise so a line is printed only on real change
        key = (tuple(int(v / 50) for v in p.values()),
               tuple(int(c / 25) for c in n.ch[:4]), n.arm)
        if key != last_key:
            last_key = key
            wheels = ' '.join(f'{a}:{p[a]:+6.0f}' for a in sorted(p))
            lv = [p[a] for a in p if a in LEFT]
            rv = [p[a] for a in p if a in RIGHT]
            lin = (sum(lv) / len(lv) + sum(rv) / len(rv)) / 2 * ERPM_TO_MS if lv and rv else 0.0
            print(f'[{time.time()-t0:6.1f}s] {NAV.get(n.nav, str(n.nav)):8s} '
                  f'{"ARMED " if n.arm == 2 else "disarm"} | ch1..4={n.ch[:4]} | '
                  f'{wheels} | v_lin={lin:+.2f} m/s', flush=True)

    print('\n=== peak physical ERPM per wheel ===')
    for a in sorted(peak):
        side = 'LEFT ' if a in LEFT else 'RIGHT'
        print(f'  addr {a} ({side}): {peak[a]:+.0f} ERPM = {peak[a]*ERPM_TO_MS:+.3f} m/s')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
