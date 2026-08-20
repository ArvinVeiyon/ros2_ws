#!/usr/bin/env python3
"""Back up the FC's hardfault logs to the companion, then optionally delete them.

WHY THIS EXISTS
  A stored hardfault blocks arming, so provoking the fault again means clearing
  it first - and clearing it means destroying the only record of the last crash.
  On 2026-08-16 some fault logs were deleted before they were copied. This makes
  the safe order the DEFAULT order: copy, verify, and only then delete.

WHY NOT `cat` OVER THE MAVLINK CONSOLE
  Because it truncates. A fault log is ~1000 lines of stack dump and the NuttShell
  console cannot stream that fast enough - the 08-16 capture lost the tail of both
  files (no "End Fault Log" marker). MAVLink FTP transfers the whole file.

SAFETY
  --delete NEVER removes a file whose local copy failed verification. Verification
  is the "End Fault Log" trailer, which is the FC's own end-of-record marker, so a
  short transfer cannot pass. Read-only without --delete.

  python3 tools/fc_fault_backup.py                      # back up only
  python3 tools/fc_fault_backup.py --delete             # back up, verify, then delete
  python3 tools/fc_fault_backup.py --delete --rearm     # ...and clear the arming block
"""
import argparse
import os
import sys
import time

from pymavlink import mavutil, mavftp

FAULT_DIR = '/fs/microsd'
END_MARKER = b'End Fault Log'


def pump(m, ftp, op, timeout=25.0):
    """Drive the FTP session until the operation completes or times out.

    pymavlink's MAVFTP runs its own receive loop inside process_ftp_reply - the
    operation name it is given must match the FTP opcode in flight, or it waits
    for a reply that never comes.
    """
    try:
        return ftp.process_ftp_reply(op, timeout=timeout)
    except Exception as e:                      # noqa: BLE001 - report, don't mask
        print(f'[ftp {op}: {e}] ', end='', flush=True)
        return None


def connect(url):
    m = mavutil.mavlink_connection(url, source_system=253, source_component=190)
    if m.wait_heartbeat(timeout=20) is None:
        print(f'no heartbeat on {url}', file=sys.stderr)
        sys.exit(2)
    # The autopilot is 1:1. Never talk to the GCS at 255:190 - its faked base_mode
    # has broken tooling here before (see setup_manual A7 / MEMORY.md).
    ftp = mavftp.MAVFTP(m, target_system=1, target_component=1)
    return m, ftp


def list_faults(m, ftp):
    names = []
    ftp.cmd_list([FAULT_DIR])
    pump(m, ftp, 'ListDirectory', timeout=20)
    for item in getattr(ftp, 'dir_contents', None) or []:
        n = item if isinstance(item, str) else getattr(item, 'name', '')
        if n.startswith('fault_') and n.endswith('.log'):
            names.append(n)
    return names


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--url', default='tcp:127.0.0.1:5760')
    ap.add_argument('--dir', default=os.path.expanduser('~/fc_faults'),
                    help='local backup directory')
    ap.add_argument('--delete', action='store_true',
                    help='delete from the FC AFTER a verified local copy')
    ap.add_argument('--rearm', action='store_true',
                    help='also drop the uncommitted hardfault so the FC will arm')
    ap.add_argument('--files', nargs='*',
                    help='explicit filenames instead of listing the card')
    a = ap.parse_args()

    os.makedirs(a.dir, exist_ok=True)
    m, ftp = connect(a.url)
    print(f'connected {a.url} -> sys {m.target_system}')

    names = a.files if a.files else list_faults(m, ftp)
    if not names:
        print('no fault_*.log found on the FC.')
        print('If you expected some, list the card by hand:')
        print('  printf "ls /fs/microsd\\n\\n" | python3 '
              '~/PX4-Autopilot/Tools/mavlink_shell.py tcp:127.0.0.1:5760')
        return 0
    print(f'found {len(names)} fault log(s): {", ".join(names)}')

    verified, failed = [], []
    for n in names:
        remote, local = f'{FAULT_DIR}/{n}', os.path.join(a.dir, n)
        print(f'  downloading {n} ... ', end='', flush=True)
        ftp.cmd_get([remote, local])
        pump(m, ftp, 'OpenFileRO', timeout=120)
        if not os.path.exists(local) or os.path.getsize(local) == 0:
            print('FAILED (no local file)')
            failed.append(n)
            continue
        with open(local, 'rb') as f:
            body = f.read()
        if END_MARKER in body:
            print(f'ok, {len(body)} bytes, VERIFIED')
            verified.append(n)
        else:
            print(f'{len(body)} bytes but NO "End Fault Log" trailer - INCOMPLETE')
            failed.append(n)

    print(f'\nverified {len(verified)}, failed {len(failed)}')
    if failed:
        print('⚠️  incomplete copies are NOT safe to delete: ' + ', '.join(failed))
        print('    Pull the SD card for those rather than losing them.')

    if a.delete:
        if not verified:
            print('⛔ nothing verified - refusing to delete anything.')
            return 1
        for n in verified:
            print(f'  deleting {n} from the FC ... ', end='', flush=True)
            ftp.cmd_rm([f'{FAULT_DIR}/{n}'])
            pump(m, ftp, 'RemoveFile', timeout=20)
            print('done')
        print(f'deleted {len(verified)} of {len(names)}; kept anything unverified.')

    if a.rearm:
        # A COMMITTED fault on the card does not block arming; an UNCOMMITTED one
        # does. `rearm` drops it. Files are handled above - this is the latch.
        print('\nclearing the uncommitted-hardfault latch ...')
        print('  run this in the shell, it is one line:')
        print('    printf "hardfault_log rearm\\nhardfault_log reset\\n\\n" | \\')
        print('      python3 ~/PX4-Autopilot/Tools/mavlink_shell.py tcp:127.0.0.1:5760')

    print(f'\nlocal copies in {a.dir}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
