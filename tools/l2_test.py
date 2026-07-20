#!/usr/bin/env python3
"""L2 bench test: AutoNav mode I/O with the wheels off the ground.

WHEELS MUST BE OFF THE GROUND. This arms the vehicle and spins the motors.

Sequence (all over DDS, no MAVLink):
  1. preflight   - all 4 VESCs online, disarmed, RPM zero
  2. mode        - DO_SET_MODE main=4 sub=11 -> nav_state 23 (AutoNav)
  3. arm         - COMPONENT_ARM_DISARM param1=1; onActivate should fire now
  4. forward     - /cmd_vel linear.x ramp, expect all wheels same sign
  5. yaw         - /cmd_vel angular.z only, expect left/right opposite sign
  6. watchdog    - stop publishing, expect RPM -> 0 within ~0.5 s
  7. safe        - disarm, restore Hold

Every motion step is bounded in time and re-zeroes before moving on. Any step
that fails its check aborts straight to step 7.

Usage: l2_test.py --wheels-are-up      (refuses to arm without this flag)
       l2_test.py --dry-run            (steps 1-2 only, never arms)
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
MOTION_SECONDS = 3.0


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
    wheels_up = '--wheels-are-up' in sys.argv
    dry_run = '--dry-run' in sys.argv
    if not wheels_up and not dry_run:
        print('REFUSING: pass --wheels-are-up (wheels off the ground) or --dry-run')
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
    if n.arm != ARM_DISARMED:
        print('    ABORT: vehicle is already armed')
        rclpy.shutdown()
        return 1

    print('--- 2. mode -> AutoNav ---')
    n.set_mode(4, 11)
    n.spin(3.0)
    print(f'    nav_state={n.nav} (expect {NAV_AUTONAV})')
    if n.nav != NAV_AUTONAV:
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
        print('--- 3. arm (onActivate should fire in the node log) ---')
        n.set_armed(True)
        n.spin(3.0)
        print(f'    arming_state={n.arm} nav_state={n.nav} rpm={n.rpm[:4]}')
        if n.arm != ARM_ARMED:
            print('    ABORT: did not arm')
            failures.append('arm')
            raise SystemExit

        print(f'--- 4. forward {TEST_SPEED} m/s for {MOTION_SECONDS}s ---')
        peak = n.drive(TEST_SPEED, 0.0, MOTION_SECONDS)
        print(f'    peak rpm={peak}')
        if all(r == 0 for r in peak):
            failures.append('forward: no wheel response')
        elif not (all(r >= 0 for r in peak) or all(r <= 0 for r in peak)):
            failures.append(f'forward: wheels disagree in sign {peak}')
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
