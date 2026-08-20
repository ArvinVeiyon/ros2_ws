#!/usr/bin/env python3
"""Pull fault_*.log off the FC per fc_hardfault_analysis.md §5 recipe.

Recipe compliance:
  1. OP_ResetSessions with EMPTY bytearray payload first, pump the reply.
  2. Fresh mavutil connection AND fresh MAVFTP object per file.
  3. callback= to cmd_get (buffers in memory; without it file lands 0 bytes).
  4. burst nack req_opcode:15 at ofs=filesize == EOF == success.
  5. Listing via ftp.list_result (NOT the nonexistent dir_contents).
"""
import os, sys, time
from pymavlink import mavutil, mavftp

URL = 'tcp:127.0.0.1:5760'
FAULT_DIR = '/fs/microsd'
OUT = os.path.expanduser('~/fc_faults')
END_MARKER = b'END Fault Log'   # uppercase END - see fc_hardfault_analysis.md 17.1


def fresh():
    m = mavutil.mavlink_connection(URL, source_system=253, source_component=190)
    hb = m.wait_heartbeat(timeout=20)
    if hb is None:
        print('no heartbeat', file=sys.stderr); sys.exit(2)
    ftp = mavftp.MAVFTP(m, target_system=1, target_component=1)
    # recipe step 1: reset leaked sessions, empty bytearray payload
    ftp._MAVFTP__send(mavftp.FTP_OP(ftp.seq, ftp.session, mavftp.OP_ResetSessions,
                                    0, 0, 0, 0, bytearray()))
    ftp.process_ftp_reply('ResetSessions', timeout=10)
    return m, ftp


def list_faults():
    m, ftp = fresh()
    ftp.cmd_list([FAULT_DIR])
    ftp.process_ftp_reply('ListDirectory', timeout=20)
    names = []
    for e in getattr(ftp, 'list_result', None) or []:
        n = getattr(e, 'name', '')
        if n.startswith('fault_') and n.endswith('.log'):
            names.append((n, getattr(e, 'size_b', 0)))
    m.close()
    return names


def get_file(name):
    m, ftp = fresh()
    buf = {}
    def cb(fh):
        buf['data'] = fh.read()
    ftp.cmd_get([f'{FAULT_DIR}/{name}'], callback=cb)
    ftp.process_ftp_reply('OpenFileRO', timeout=120)
    m.close()
    return buf.get('data', b'')


def rm_file(name):
    m, ftp = fresh()
    ftp.cmd_rm([f'{FAULT_DIR}/{name}'])
    ret = ftp.process_ftp_reply('RemoveFile', timeout=20)
    m.close()
    return ret


def verified(name):
    """True iff a complete backup of this log already sits in OUT."""
    local = os.path.join(OUT, name)
    if not (os.path.exists(local) and os.path.getsize(local) > 0):
        return False
    with open(local, 'rb') as f:
        return END_MARKER in f.read()


def pull(name):
    if verified(name):
        print(f'  {name}: already backed up + verified')
        return True
    print(f'  pulling {name} ...', flush=True)
    data = get_file(name)
    if not data:
        print('    FAILED (0 bytes)')
        return False
    with open(os.path.join(OUT, name), 'wb') as f:
        f.write(data)
    ok = END_MARKER in data
    print(f'    {len(data)} B  {"VERIFIED" if ok else "NO trailer - INCOMPLETE"}')
    time.sleep(1)
    return ok


def main():
    os.makedirs(OUT, exist_ok=True)
    delete = '--delete' in sys.argv

    if not delete:
        names = list_faults()
        print(f'fault logs on card: {len(names)}')
        for n, sz in names:
            print(f'  {n}  {sz} B')
        for n, _ in names:
            pull(n)
        return 0

    # 17.9: never delete from a stale listing. Re-list every round, and delete a
    # file only in the same pass that verified its backup, one file at a time.
    for rnd in range(1, 11):
        names = [n for n, _ in list_faults()]
        print(f'round {rnd}: {len(names)} fault log(s) on card')
        if not names:
            print('card is clean')
            return 0
        for n in names:
            if not pull(n):
                print(f'    KEEPING {n} on card (backup not verified)')
                continue
            print(f'    deleting {n} ...', end=' ', flush=True)
            print('ok' if rm_file(n) is None else 'see reply above')
            time.sleep(1)
    print('still not empty after 10 rounds - new faults arriving faster than the pass')
    return 1


if __name__ == '__main__':
    sys.exit(main())
