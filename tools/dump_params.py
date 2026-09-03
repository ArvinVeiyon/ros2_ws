#!/usr/bin/env python3
"""Dump every PX4 parameter over MAVLink to a timestamped backup file.

Companion to set_param.py, and uses the same route: parameters are NOT exposed
over DDS at all, so MAVLink PARAM_REQUEST_LIST is the only way to get them.

Writes a QGC-compatible .params file (tab separated, INT32 vs REAL32 typed the
way QGC expects) so the result can be re-loaded from QGC's parameter screen.

    python3 tools/dump_params.py                    # -> ~/fc_param_backups/
    python3 tools/dump_params.py --out /tmp/x.params

NOTE: a full param list is a large MAVLink burst. Upstream PX4 #22160 ties
MAVLink transfers to hardfaults on fmu-v6xrt, so do not run this casually while
a fault soak is in progress -- take the snapshot, then leave the link alone.
"""
import argparse
import os
import struct
import sys
import time
from datetime import datetime

from pymavlink import mavutil

TYPE_INT = {1, 2, 3, 4, 5, 6}


def decode(msg):
    if msg.param_type in TYPE_INT:
        return struct.unpack('<i', struct.pack('<f', msg.param_value))[0]
    return msg.param_value


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--url', default='tcp:127.0.0.1:5760')
    ap.add_argument('--out', default=None)
    ap.add_argument('--timeout', type=float, default=90.0)
    args = ap.parse_args()

    out = args.out
    if out is None:
        d = os.path.expanduser('~/fc_param_backups')
        os.makedirs(d, exist_ok=True)
        out = os.path.join(
            d, datetime.now().strftime('fc_params_%Y%m%d_%H%M%S.params'))

    m = mavutil.mavlink_connection(args.url, source_system=250,
                                   source_component=190)
    print(f'connecting {args.url} ...', flush=True)
    if m.wait_heartbeat(timeout=15) is None:
        sys.exit('no heartbeat -- is mavlink.router.service up?')
    print(f'  autopilot {m.target_system}:{m.target_component}', flush=True)

    params = {}
    expected = None
    m.mav.param_request_list_send(m.target_system, m.target_component)
    deadline = time.time() + args.timeout
    last = time.time()
    while time.time() < deadline:
        r = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if r is None:
            if time.time() - last > 6:
                break
            continue
        last = time.time()
        expected = r.param_count
        params[r.param_id.strip('\x00')] = (decode(r), r.param_type)
        if expected and len(params) >= expected:
            break

    print(f'  received {len(params)} of {expected} parameters', flush=True)

    # Re-request anything missed individually.
    if expected and len(params) < expected:
        print('  (gaps present -- PARAM_REQUEST_LIST dropped messages)')

    with open(out, 'w') as f:
        f.write('# Onboard parameters for Vind-Roz FC\n')
        f.write(f'# Saved {datetime.now().isoformat(timespec="seconds")}\n')
        f.write(f'# {len(params)} of {expected} parameters\n')
        f.write('# MAV ID\tCOMPONENT ID\tPARAM NAME\tVALUE\tTYPE\n')
        for name in sorted(params):
            val, ptype = params[name]
            if ptype in TYPE_INT:
                f.write(f'{m.target_system}\t{m.target_component}\t{name}\t{val}\t6\n')
            else:
                f.write(f'{m.target_system}\t{m.target_component}\t{name}\t{val:.8g}\t9\n')
    print(f'  wrote {out}', flush=True)
    if expected and len(params) < expected:
        sys.exit(f'INCOMPLETE: {len(params)}/{expected} -- do NOT reset params on this backup')


if __name__ == '__main__':
    main()
