#!/usr/bin/env python3
"""Live view of rover state for the L2 bench test: ESC power/RPM + FC mode/arming.

Read-only. Run alongside l2_test.py, or on its own to see when the motor bus
comes up (esc_online_flags reaches 15 = all four VESCs).
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import EscStatus, VehicleStatus

NAV_STATES = {0: 'Manual', 3: 'Mission', 4: 'Hold', 23: 'AutoNav(EXTERNAL1)'}
ARM_STATES = {1: 'DISARMED', 2: 'ARMED'}

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=5)


class Watch(Node):
    def __init__(self):
        super().__init__('l2_watch')
        self.flags = None
        self.rpm = []
        self.volt = []
        self.nav = None
        self.arm = None
        self.create_subscription(EscStatus, '/fmu/out/esc_status', self.esc_cb, PX4_QOS)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.st_cb, PX4_QOS)

    def esc_cb(self, msg):
        self.flags = msg.esc_online_flags
        n = msg.esc_count
        self.rpm = [msg.esc[i].esc_rpm for i in range(n)]
        self.volt = [round(msg.esc[i].esc_voltage, 1) for i in range(n)]

    def st_cb(self, msg):
        self.nav, self.arm = msg.nav_state, msg.arming_state

    def line(self):
        online = f'{self.flags:04b}'[::-1] if self.flags is not None else '????'
        nav = NAV_STATES.get(self.nav, str(self.nav))
        arm = ARM_STATES.get(self.arm, str(self.arm))
        return (f'ESC online={self.flags} [{online}] rpm={self.rpm} V={self.volt} | '
                f'mode={nav} {arm}')


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 3600.0
    rclpy.init()
    n = Watch()
    t0 = time.time()
    last = ''
    while time.time() - t0 < duration:
        rclpy.spin_once(n, timeout_sec=0.2)
        cur = n.line()
        if cur != last:
            print(f'[{time.time()-t0:6.1f}s] {cur}', flush=True)
            last = cur
    rclpy.shutdown()


if __name__ == '__main__':
    main()
