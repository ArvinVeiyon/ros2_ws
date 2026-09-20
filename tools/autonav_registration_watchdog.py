#!/usr/bin/env python3
"""Keep the AutoNav external mode REGISTERED with the FC, without operator action.

WHY THIS EXISTS
  PX4 wipes an external mode's registration when the FC reboots, and
  autonav_mode does not notice -- it keeps running, looks healthy, and the mode
  is simply refused when you arm. Every session so far has worked around that by
  hand: disarm, restart the node, check the log line, then arm.

WHAT IT DOES
  Watches /fmu/out/vehicle_status_v1. The timestamp is PX4 boot-relative, so a
  reboot makes it jump BACKWARDS. On that edge, and only while DISARMED, it
  restarts rover-autonav-mode so the mode re-registers.

THE ONE CONSTRAINT IT CANNOT REMOVE
  PX4 REFUSES TO REGISTER AN EXTERNAL MODE WHILE ARMED. So re-registration can
  only happen in a disarmed window. That is exactly what makes arming the only
  step left for the operator: by the time the switch goes up, registration is
  already in place.

SAFETY
  - NEVER restarts the node while armed. Restarting the mode executor under an
    armed rover would drop the active mode.
  - Rate-limited: at most one restart per RESTART_COOLDOWN seconds.
  - Commands nothing to the vehicle. It only reads status and calls systemctl.
"""
import subprocess, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleStatus

PX4Q = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                  history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.TRANSIENT_LOCAL)
BACKWARD_JUMP_S  = 5.0     # a drop this large in FC uptime means it rebooted
RESTART_COOLDOWN = 30.0
SETTLE_S         = 4.0     # let the FC finish booting before restarting the node

class Watchdog(Node):
    def __init__(s):
        super().__init__('autonav_registration_watchdog')
        s.last_ts = None
        s.last_restart = 0.0
        s.pending_since = None
        s.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', s.on_status, PX4Q)
        s.create_timer(1.0, s.tick)
        s.get_logger().info('watching for FC reboots; will re-register AutoNav while disarmed')

    def on_status(s, m):
        ts = m.timestamp / 1e6                      # PX4 boot-relative seconds
        if s.last_ts is not None and ts < s.last_ts - BACKWARD_JUMP_S:
            s.get_logger().warn(
                f'FC REBOOT detected (uptime {s.last_ts:.0f}s -> {ts:.0f}s). '
                'AutoNav registration is gone; will restart the mode node once disarmed.')
            s.pending_since = time.time()
        s.last_ts = ts
        s.armed = (m.arming_state == 2)

    def tick(s):
        if s.pending_since is None:
            return
        if time.time() - s.pending_since < SETTLE_S:
            return
        if getattr(s, 'armed', False):
            s.get_logger().warn('re-registration pending but the vehicle is ARMED -- waiting. '
                                'PX4 cannot register an external mode while armed.')
            return
        if time.time() - s.last_restart < RESTART_COOLDOWN:
            return
        s.get_logger().warn('restarting rover-autonav-mode to re-register AutoNav')
        r = subprocess.run(['sudo', '-n', 'systemctl', 'restart', 'rover-autonav-mode'],
                           capture_output=True, text=True)
        if r.returncode == 0:
            s.get_logger().info('rover-autonav-mode restarted; registration should be live')
            s.pending_since = None
        else:
            s.get_logger().error(f'restart FAILED rc={r.returncode}: {r.stderr.strip()}')
        s.last_restart = time.time()

def main():
    rclpy.init(); n = Watchdog()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__':
    main()
