#!/usr/bin/env python3
"""Trace the whole AutoNav chain from the companion: registration -> arming.

Watches every hop the px4_ros2 mode goes through and drives the two commands
(mode select, arm) over DDS, printing what was observed at each stage so the
chain can be audited rather than assumed.

Run `ros2 run autonav_mode autonav_mode` immediately AFTER starting this, so
stage 1 and 2 are captured live.

Usage: autonav_chain_check.py            observe + select mode only, never arms
       autonav_chain_check.py --arm      also arms (WHEELS MUST BE OFF GROUND)
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (ArmingCheckReply, ArmingCheckRequest, RegisterExtComponentReply,
                          RegisterExtComponentRequest, RoverRateSetpoint, RoverSpeedSetpoint,
                          VehicleCommand, VehicleCommandAck, VehicleStatus)

# /fmu/out/ comes from the uXRCE agent as TRANSIENT_LOCAL, but the px4_ros2 lib
# publishes the /fmu/in/ topics VOLATILE — subscribing TRANSIENT_LOCAL to those
# silently receives nothing.
PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=5)

PX4_IN_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST, depth=20)

NAV_AUTONAV, NAV_HOLD = 23, 4
ARM_DISARMED, ARM_ARMED = 1, 2
ACK = {0: 'ACCEPTED', 1: 'TEMPORARILY_REJECTED', 2: 'DENIED', 3: 'UNSUPPORTED', 4: 'FAILED'}


class Chain(Node):
    def __init__(self):
        super().__init__('autonav_chain_check')
        self.reg_req = []
        self.reg_reply = []
        self.ac_req = []
        self.ac_reply = []
        self.speed_sp = []
        self.rate_sp = []
        self.acks = []
        self.nav = self.arm = None
        self.t0 = time.time()

        sub = self.create_subscription
        sub(RegisterExtComponentRequest, '/fmu/in/register_ext_component_request',
            lambda m: self.reg_req.append((self.el(), m)), PX4_IN_QOS)
        sub(RegisterExtComponentReply, '/fmu/out/register_ext_component_reply',
            lambda m: self.reg_reply.append((self.el(), m)), PX4_QOS)
        sub(ArmingCheckRequest, '/fmu/out/arming_check_request_v1',
            lambda m: self.ac_req.append((self.el(), m)), PX4_QOS)
        sub(ArmingCheckReply, '/fmu/in/arming_check_reply_v1',
            lambda m: self.ac_reply.append((self.el(), m)), PX4_IN_QOS)
        sub(RoverSpeedSetpoint, '/fmu/in/rover_speed_setpoint',
            lambda m: self.speed_sp.append((self.el(), m)), PX4_IN_QOS)
        sub(RoverRateSetpoint, '/fmu/in/rover_rate_setpoint',
            lambda m: self.rate_sp.append((self.el(), m)), PX4_IN_QOS)
        sub(VehicleCommandAck, '/fmu/out/vehicle_command_ack',
            lambda m: self.acks.append((self.el(), m)), PX4_QOS)
        sub(VehicleStatus, '/fmu/out/vehicle_status_v1', self.st, PX4_QOS)

        self.vc = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

    def el(self):
        return time.time() - self.t0

    def st(self, m):
        self.nav, self.arm = m.nav_state, m.arming_state

    def spin(self, sec):
        t = time.time()
        while time.time() - t < sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def cmd(self, command, p1=0.0, p2=0.0, p3=0.0):
        m = VehicleCommand()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.command = command
        m.param1, m.param2, m.param3 = float(p1), float(p2), float(p3)
        m.target_system = m.target_component = 1
        m.source_system = m.source_component = 1
        m.from_external = True
        self.vc.publish(m)

    def ack_for(self, command, since):
        return [(t, a) for t, a in self.acks if a.command == command and t >= since]


def main():
    do_arm = '--arm' in sys.argv
    rclpy.init()
    n = Chain()

    print('Waiting up to 40 s for autonav_mode to register — start it now.\n')
    t_wait = time.time()
    while time.time() - t_wait < 40 and not n.reg_reply:
        rclpy.spin_once(n, timeout_sec=0.1)

    print('=' * 68)
    print('STAGE 1  companion -> FC   registration request')
    if n.reg_req:
        t, m = n.reg_req[-1]
        print(f'  t={t:5.1f}s  /fmu/in/register_ext_component_request')
        print(f'            name="{bytes(m.name).decode(errors="replace").strip(chr(0))}" '
              f'arming_check={m.register_arming_check} mode={m.register_mode}')
    else:
        print('  NOT SEEN (node may have registered before this script started)')

    print('\nSTAGE 2  FC -> companion   registration reply')
    if n.reg_reply:
        t, m = n.reg_reply[-1]
        print(f'  t={t:5.1f}s  /fmu/out/register_ext_component_reply')
        print(f'            success={m.success} arming_check_id={m.arming_check_id} '
              f'mode_id={m.mode_id}  <- mode_id is the External Mode slot')
    else:
        print('  NOT SEEN — mode never registered, chain stops here')
        rclpy.shutdown()
        return 1

    n.spin(6.0)
    print('\nSTAGE 3  arming-check handshake (continuous, keeps the mode alive)')
    print(f'  FC -> companion  arming_check_request_v1 : {len(n.ac_req)} in window')
    print(f'  companion -> FC  arming_check_reply_v1   : {len(n.ac_reply)} in window')
    if n.ac_reply:
        _, m = n.ac_reply[-1]
        print(f'  latest reply: can_arm_and_run={m.can_arm_and_run} '
              f'health_component_index={m.health_component_index} num_events={m.num_events}')
    print('  (the px4_ros2 lib aborts the node if a request does not arrive within 4 s)')

    print(f'\nSTAGE 4  mode is selectable   nav_state now={n.nav} arming_state={n.arm}')
    since = n.el()
    n.cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 4, 11)
    n.spin(4.0)
    acks = n.ack_for(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, since)
    for t, a in acks[-1:]:
        print(f'  DO_SET_MODE(main=4,sub=11) ack: {ACK.get(a.result, a.result)}')
    print(f'  nav_state -> {n.nav}  (expect {NAV_AUTONAV} = AutoNav / EXTERNAL1)')
    if n.nav != NAV_AUTONAV:
        print('  CHAIN BROKEN: mode did not take')
        rclpy.shutdown()
        return 1

    print('\nSTAGE 5  arming')
    if not do_arm:
        print('  skipped (no --arm). Restoring Hold.')
        n.cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 4, 3)
        n.spin(3.0)
        print(f'  nav_state -> {n.nav}')
        rclpy.shutdown()
        return 0

    n.speed_sp.clear()
    n.rate_sp.clear()
    since = n.el()
    n.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1)
    n.spin(5.0)
    for t, a in n.ack_for(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, since)[-1:]:
        print(f'  ARM ack: {ACK.get(a.result, a.result)}')
    print(f'  arming_state -> {n.arm} (expect {ARM_ARMED}), nav_state={n.nav}')

    print('\nSTAGE 6  onActivate fired -> mode is producing setpoints')
    print(f'  /fmu/in/rover_speed_setpoint : {len(n.speed_sp)} msgs since arm')
    print(f'  /fmu/in/rover_rate_setpoint  : {len(n.rate_sp)} msgs since arm')
    if n.speed_sp:
        _, m = n.speed_sp[-1]
        print(f'  latest speed setpoint: {m.speed_body_x:+.3f} m/s (expect 0.000, no /cmd_vel sent)')
    if n.rate_sp:
        _, m = n.rate_sp[-1]
        print(f'  latest rate setpoint : {m.yaw_rate_setpoint:+.3f} rad/s (expect 0.000)')

    print('\nSTAGE 7  safe teardown')
    n.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0)
    n.spin(3.0)
    n.cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 4, 3)
    n.spin(3.0)
    print(f'  arming_state={n.arm} nav_state={n.nav}')
    print('=' * 68)
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
