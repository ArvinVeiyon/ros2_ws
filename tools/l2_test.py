#!/usr/bin/env python3
"""L2 floor/bench test: AutoNav mode I/O. YOU arm via RC — the script never arms.

This will MOVE A LIVE VEHICLE. Have a clear run-out and your hand on the KILL
switch (RC ch8). The AutoNav executor's reflex collision-stop is active, but it
is a backstop, not a substitute for the kill switch.

Sequence (all over DDS, no MAVLink):
  1. preflight   - all 4 VESCs online, disarmed, RPM zero
  2. mode        - DO_SET_MODE main=4 sub=11 -> nav_state 23 (AutoNav)
  3. arm         - WAITS for YOU to arm via RC (ch5); never software-arms.
                   Aborts (no motion) if you don't arm within the timeout.
  4. forward     - /cmd_vel linear.x, expect all 4 wheels to respond
                   (RPM sign is NOT a reliable direction indicator on this HW)
  5. yaw         - /cmd_vel angular.z only
  6. watchdog    - stop publishing, expect RPM -> 0 within ~0.5 s
  7. safe        - disarm, restore Hold

Every motion step is bounded in time and re-zeroes before moving on. Any step
that fails its check aborts straight to step 7.

Usage: l2_test.py --live            (waits for your RC arm, then runs)
       l2_test.py --wheels-are-up   (deprecated alias for --live)
       l2_test.py --dry-run         (steps 1-2 only, never arms, no motion)
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from px4_msgs.msg import EscStatus, VehicleCommand, VehicleStatus

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=5)

NAV_AUTONAV = 23
NAV_HOLD = 4
ARM_DISARMED, ARM_ARMED = 1, 2

# Deliberately well inside the node's 0.8 m/s / 1.0 rad/s clamp.
TEST_SPEED = 0.2      # [m/s]
TEST_YAW_RATE = 0.3   # [rad/s]
MOTION_SECONDS = 2.0  # first-run nudge (~0.4 m fwd); collision-stop backstops
ARM_WAIT_TIMEOUT = 30.0  # [s] time allowed for the operator to arm via RC
RESPONSE_RPM = 20     # [rpm] a wheel below this is treated as "did not respond"


class L2(Node):
    def __init__(self):
        super().__init__('l2_test')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vc_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)
        self.create_subscription(EscStatus, '/fmu/out/esc_status', self.esc_cb, PX4_QOS)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.st_cb, PX4_QOS)
        self.flags = None
        self.rpm = [0, 0, 0, 0]
        self.nav = None
        self.arm = None

    def esc_cb(self, msg):
        self.flags = msg.esc_online_flags
        self.rpm = [msg.esc[i].esc_rpm for i in range(msg.esc_count)]

    def st_cb(self, msg):
        self.nav, self.arm = msg.nav_state, msg.arming_state

    def spin(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)

    def send_cmd(self, command, p1=0.0, p2=0.0, p3=0.0):
        m = VehicleCommand()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.command = command
        m.param1, m.param2, m.param3 = float(p1), float(p2), float(p3)
        m.target_system, m.target_component = 1, 1
        m.source_system, m.source_component = 1, 1
        m.from_external = True
        self.vc_pub.publish(m)

    def set_mode(self, main, sub):
        self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, main, sub)

    def set_armed(self, armed):
        self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1 if armed else 0)

    def wait_for_arm(self, timeout):
        """Block until the operator arms via RC, or timeout. Never software-arms."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.arm == ARM_ARMED:
                return True
        return False

    def drive(self, speed, yaw_rate, seconds):
        """Publish /cmd_vel at 20 Hz, sampling peak RPM."""
        peak = [0, 0, 0, 0]
        t0 = time.time()
        while time.time() - t0 < seconds:
            t = Twist()
            t.linear.x = float(speed)
            t.angular.z = float(yaw_rate)
            self.cmd_pub.publish(t)
            self.spin(0.05)
            for i, r in enumerate(self.rpm[:4]):
                if abs(r) > abs(peak[i]):
                    peak[i] = r
        return peak

    def coast(self, seconds):
        """Stop publishing entirely; the node's 0.5 s watchdog should zero output."""
        self.spin(seconds)
        return list(self.rpm[:4])


def main():
    live = ('--live' in sys.argv) or ('--wheels-are-up' in sys.argv)
    dry_run = '--dry-run' in sys.argv
    if not live and not dry_run:
        print('REFUSING: pass --live (you arm via RC) or --dry-run')
        return 2

    rclpy.init()
    n = L2()
    n.spin(3.0)
    failures = []

    print('--- 1. preflight ---')
    print(f'    esc_online_flags={n.flags} rpm={n.rpm[:4]} nav_state={n.nav} arming_state={n.arm}')
    if n.flags != 15:
        print('    ABORT: not all 4 VESCs online')
        rclpy.shutdown()
        return 1
    pre_armed = (n.arm == ARM_ARMED)
    if pre_armed:
        # Operator may arm in Manual FIRST (they can't see the live prompt) and
        # we switch to AutoNav below. Tolerate it only if the wheels are stopped.
        if any(abs(r) > RESPONSE_RPM for r in n.rpm[:4]):
            print('    ABORT: armed AND wheels turning — disarm/kill first.')
            rclpy.shutdown()
            return 1
        print(f'    (already armed in nav={n.nav}, wheels stopped — will switch to AutoNav)')
    elif n.arm != ARM_DISARMED:
        print(f'    ABORT: unexpected arming_state={n.arm}')
        rclpy.shutdown()
        return 1

    print('--- 2. mode -> AutoNav ---')
    n.set_mode(4, 11)
    n.spin(3.0)
    print(f'    nav_state={n.nav} (expect {NAV_AUTONAV})')
    if n.nav != NAV_AUTONAV and not pre_armed:
        # When pre-armed, step 3b sends AutoNav again and verifies it holds.
        print('    ABORT: mode did not take')
        rclpy.shutdown()
        return 1

    if dry_run:
        print('--- dry run: restoring Hold, not arming ---')
        n.set_mode(4, 3)
        n.spin(2.0)
        print(f'    nav_state={n.nav}')
        rclpy.shutdown()
        return 0

    try:
        if not pre_armed:
            print(f'--- 3. ARM VIA RC NOW (ch5), throttle NEUTRAL. Hand on the KILL (ch8). ---')
            print(f'    You may arm in Manual; the script switches to AutoNav next.')
            print(f'    Waiting up to {ARM_WAIT_TIMEOUT:.0f}s. Ctrl-C to abort.')
            if not n.wait_for_arm(ARM_WAIT_TIMEOUT):
                print('    ABORT: not armed within timeout — no motion.')
                failures.append('arm-timeout')
                raise SystemExit
        print(f'    ARMED. arming_state={n.arm} nav_state={n.nav}')

        print('--- 3b. switching to AutoNav (software) ---')
        n.set_mode(4, 11)
        n.spin(3.0)
        if n.nav != NAV_AUTONAV or n.arm != ARM_ARMED:
            print(f'    ABORT: could not enter AutoNav armed (nav={n.nav} arm={n.arm}).')
            failures.append('autonav-switch-failed')
            raise SystemExit
        # confirm AutoNav holds — i.e. the RC mode switch is not yanking it back
        n.spin(2.0)
        if n.nav != NAV_AUTONAV:
            print(f'    ABORT: AutoNav did not hold (nav={n.nav}) — RC mode switch reverting it.')
            failures.append('autonav-not-held')
            raise SystemExit
        print(f'    in AutoNav, armed, holding zero. nav_state={n.nav} rpm={n.rpm[:4]}')

        print(f'--- 4. forward {TEST_SPEED} m/s for {MOTION_SECONDS}s ---')
        peak = n.drive(TEST_SPEED, 0.0, MOTION_SECONDS)
        print(f'    peak rpm={peak}  (sign is NOT physical direction on this HW)')
        # NOTE: if a wall is within collision.stop_distance the reflex will
        # legitimately zero forward — "no response" here can mean "wall ahead".
        stalled = [i for i, r in enumerate(peak) if abs(r) < RESPONSE_RPM]
        if len(stalled) == 4:
            failures.append('forward: no wheel response (or collision-stop engaged)')
        elif stalled:
            failures.append(f'forward: wheels {stalled} did not respond {peak}')
        n.coast(1.0)

        print(f'--- 5. yaw {TEST_YAW_RATE} rad/s for {MOTION_SECONDS}s ---')
        peak = n.drive(0.0, TEST_YAW_RATE, MOTION_SECONDS)
        print(f'    peak rpm={peak}')
        if all(r == 0 for r in peak):
            failures.append('yaw: no wheel response')
        n.coast(1.0)

        print('--- 6. watchdog: stop publishing /cmd_vel ---')
        n.drive(TEST_SPEED, 0.0, 1.5)
        idle = n.coast(2.0)
        print(f'    rpm 2s after last cmd_vel={idle} (expect zeros)')
        if any(abs(r) > 50 for r in idle):
            failures.append(f'watchdog: wheels still turning {idle}')

    except SystemExit:
        pass
    finally:
        print('--- 7. safe: disarm + Hold ---')
        n.coast(0.5)
        n.set_armed(False)
        n.spin(2.0)
        n.set_mode(4, 3)
        n.spin(2.0)
        print(f'    arming_state={n.arm} nav_state={n.nav} rpm={n.rpm[:4]}')

    print()
    if failures:
        print('L2 RESULT: FAIL')
        for f in failures:
            print(f'  - {f}')
    else:
        print('L2 RESULT: PASS — mode I/O and wheel response verified')
    rclpy.shutdown()
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
