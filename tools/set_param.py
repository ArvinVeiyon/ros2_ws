#!/usr/bin/env python3
"""Set a PX4 parameter over MAVLink, with readback verification.

Parameters are NOT exposed over DDS at all, so this is the only route. Use
PARAM_SET/PARAM_REQUEST_READ via pymavlink -- NOT Tools/mavlink_shell.py, which
has wedged the FC's MAVLink link on this vehicle before.

Refuses to write while the vehicle is ARMED: changing a control gain under an
armed rover is how you get a surprise.

    python3 tools/set_param.py RO_YAW_RATE_P 0.05
    python3 tools/set_param.py RO_YAW_RATE_P            # read only

Values are written as float (MAV_PARAM_TYPE_REAL32). PX4 carries INT32 params as
a raw bit pattern in PARAM_VALUE's float field, so integer params read back as
nonsense floats -- this tool prints both interpretations and does not try to
guess which you meant.
"""
import argparse
import struct
import sys
import time

from pymavlink import mavutil

TYPE_INT = {1, 2, 3, 4, 5, 6}


def decode(msg):
    if msg.param_type in TYPE_INT:
        return struct.unpack('<i', struct.pack('<f', msg.param_value))[0]
    return msg.param_value


def read(m, name, tries=5):
    for _ in range(tries):
        m.mav.param_request_read_send(
            m.target_system, m.target_component, name.encode(), -1)
        deadline = time.time() + 1.5
        while time.time() < deadline:
            r = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.5)
            if r is None:
                break
            if r.param_id.strip('\x00') == name:
                return r
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('name')
    ap.add_argument('value', nargs='?', type=float, default=None)
    ap.add_argument('--url', default='tcp:127.0.0.1:5760')
    ap.add_argument('--force-armed', action='store_true',
                    help='allow writing while armed (do not)')
    args = ap.parse_args()
    name = args.name.upper()

    m = mavutil.mavlink_connection(args.url, source_system=250, source_component=190)
    print(f'connecting {args.url} ...', flush=True)
    if m.wait_heartbeat(timeout=15) is None:
        print('NO HEARTBEAT — MAVLink link down'); return 1

    before = read(m, name)
    if before is None:
        print(f'{name}: <no reply> — wrong name, or the link is busy. '
              f'Retry; do not assume it is unset.')
        return 1
    print(f'{name} currently = {decode(before)!r}  (type {before.param_type})')

    if args.value is None:
        return 0

    hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    armed = bool(hb and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    if armed and not args.force_armed:
        print('\nREFUSING: vehicle is ARMED. Disarm before changing a control gain.')
        return 2

    print(f'writing {name} = {args.value} ...')
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(),
                         float(args.value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.5)

    after = read(m, name)
    if after is None:
        print('!! no readback — CHANGE UNCONFIRMED. Re-read before flying.')
        return 1
    got = decode(after)
    print(f'{name} now = {got!r}')
    if abs(float(got) - args.value) > 1e-6:
        print(f'!! MISMATCH: asked {args.value}, got {got}. Not applied.')
        return 1
    print('verified.')
    print('NOTE: this is a RAM write. It does NOT survive an FC reboot unless '
          'saved (QGC, or `param save` in NuttShell).')
    return 0


if __name__ == '__main__':
    sys.exit(main())
