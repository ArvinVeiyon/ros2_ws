#!/usr/bin/env python3
"""Render an RTAB-Map database for manual verification by the operator.

A map can be internally consistent and still be a picture of nowhere. Only the
person who drove it can say whether it is their room, so this produces the views
that make that judgement possible - and strips the one thing that reliably
confuses it, the rover's own body.

    python3 tools/map_review.py ~/house_map_v4.db
    python3 tools/map_review.py ~/house_map_v4.db --out /tmp/review --keep-rover

Outputs, next to the database unless --out is given:
    <name>_review.png    three panels: top-down by height, oblique true colour,
                         and a 0.15-1.30 m slab (walls and furniture only)
    <name>_plan.png      top-down slab with the map axes and the driven path
    <name>_cloud.ply     the exported cloud, rover body removed

WHY THE ROVER BODY IS REMOVED: the mask that should crop the rover's top plate
out of each depth frame is silently discarded whenever the cropped image height
does not divide exactly by Grid/DepthDecimation. In house_map_v4 that left 11.8%
of the cloud made of the rover's own plate, drawn at every position it occupied
and clearly visible as a trail through the floor. Points within ROVER_RADIUS of
any camera pose and below the camera height are removed here regardless, so the
review is not contaminated by that bug when it recurs.
"""
import argparse
import math
import os
import subprocess
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

# Plate is 0.730 m long, base_link is 0.345 m from the front tip, camera sits
# above base_link at 0.305 m. 0.45 m horizontal covers the plate with margin.
ROVER_RADIUS = 0.45
ROVER_ZMAX = 0.34
SLAB = (0.15, 1.30)          # the band that holds walls and furniture
MAXPTS = 240000


def read_ply(path):
    with open(path, 'rb') as f:
        if f.readline().strip() != b'ply':
            raise SystemExit(f'{path} is not a PLY')
        count, props, in_vertex = 0, [], False
        while True:
            line = f.readline().decode('ascii', 'ignore').strip()
            if line.startswith('format'):
                pass
            elif line.startswith('element'):
                parts = line.split()
                in_vertex = parts[1] == 'vertex'
                if in_vertex:
                    count = int(parts[2])
            elif line.startswith('property') and in_vertex and not line.startswith('property list'):
                props.append(line.split()[1:])
            elif line == 'end_header':
                break
        names = [p[1] for p in props]
        tmap = {'float': 'f4', 'double': 'f8', 'uchar': 'u1', 'int': 'i4', 'ushort': 'u2'}
        dt = np.dtype([(n, '<' + tmap[p[0]]) for n, p in zip(names, props)])
        arr = np.frombuffer(f.read(dt.itemsize * count), dtype=dt, count=count)
        return {n: arr[n].astype(np.float64) for n in names}


def run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout[-1500:] or r.stderr[-1500:])
        raise SystemExit(f'FAILED: {" ".join(cmd[:3])} ...')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('db')
    ap.add_argument('--out', default=None, help='output directory (default: beside the db)')
    ap.add_argument('--voxel', type=float, default=0.02)
    ap.add_argument('--max-range', type=float, default=4.0)
    ap.add_argument('--keep-rover', action='store_true',
                    help='do NOT strip the rover body (to inspect the contamination itself)')
    a = ap.parse_args()

    db = os.path.abspath(a.db)
    if not os.path.exists(db):
        raise SystemExit(f'no such database: {db}')
    outdir = os.path.abspath(a.out) if a.out else os.path.dirname(db)
    os.makedirs(outdir, exist_ok=True)
    stem = os.path.splitext(os.path.basename(db))[0]

    print(f'database : {db}')
    print('exporting cloud and poses ...')
    run(['rtabmap-export', '--cloud', '--voxel', str(a.voxel),
         '--max_range', str(a.max_range),
         '--output', f'{stem}_rv', '--output_dir', outdir, db])
    run(['rtabmap-export', '--poses', '--output', f'{stem}_rv',
         '--output_dir', outdir, db])

    ply = os.path.join(outdir, f'{stem}_rv_cloud.ply')
    posef = os.path.join(outdir, f'{stem}_rv_poses.txt')
    c = read_ply(ply)
    x, y, z = c['x'], c['y'], c['z']
    has_rgb = all(k in c for k in ('red', 'green', 'blue'))
    rgb = np.stack([c['red'], c['green'], c['blue']], 1) / 255.0 if has_rgb else None

    ok = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    x, y, z = x[ok], y[ok], z[ok]
    if rgb is not None:
        rgb = rgb[ok]
    total = len(x)

    P = np.loadtxt(posef)
    px, py = (P[:, 1], P[:, 2]) if P.ndim > 1 else (np.array([P[1]]), np.array([P[2]]))

    if not a.keep_rover:
        near = np.full(len(x), np.inf)
        for i in range(0, len(x), 20000):
            dx = x[i:i + 20000, None] - px[None, :]
            dy = y[i:i + 20000, None] - py[None, :]
            near[i:i + 20000] = np.sqrt(dx * dx + dy * dy).min(axis=1)
        body = (near < ROVER_RADIUS) & (z < ROVER_ZMAX)
        keep = ~body
        print(f'rover body removed: {body.sum()} of {total} points '
              f'({100 * body.sum() / total:.1f}%)')
        x, y, z = x[keep], y[keep], z[keep]
        if rgb is not None:
            rgb = rgb[keep]

    def clip(v):
        lo, hi = np.percentile(v, [0.3, 99.7])
        return (v >= lo) & (v <= hi)
    m = clip(x) & clip(y) & clip(z)
    x, y, z = x[m], y[m], z[m]
    if rgb is not None:
        rgb = rgb[m]

    print(f'points   : {len(x)}')
    print(f'room     : {x.max()-x.min():.2f} x {y.max()-y.min():.2f} x {z.max()-z.min():.2f} m')
    print(f'driven   : {px.max()-px.min():.2f} x {py.max()-py.min():.2f} m  '
          f'({len(px)} keyframes)')

    if len(x) > MAXPTS:
        idx = np.random.default_rng(0).choice(len(x), MAXPTS, replace=False)
        x, y, z = x[idx], y[idx], z[idx]
        if rgb is not None:
            rgb = rgb[idx]
    slab = (z > SLAB[0]) & (z < SLAB[1])

    # ---- three-panel review ----
    fig = plt.figure(figsize=(19, 6.6), dpi=125)
    fig.patch.set_facecolor('#f2f3f5')
    ax1 = fig.add_subplot(131)
    s = ax1.scatter(x, y, c=z, s=.35, cmap='viridis', linewidths=0)
    ax1.set_title('top-down, coloured by height', fontsize=11)
    ax1.set_aspect('equal'); ax1.grid(alpha=.25, lw=.4)
    ax1.set_xlabel('x (m)'); ax1.set_ylabel('y (m)')
    fig.colorbar(s, ax=ax1, shrink=.8, label='z (m)')

    ax2 = fig.add_subplot(132, projection='3d')
    ax2.scatter(x, y, z, c=(rgb if rgb is not None else z), s=.25, linewidths=0,
                cmap=None if rgb is not None else 'viridis')
    ax2.set_title('oblique, true colour' if rgb is not None else 'oblique', fontsize=11)
    ax2.view_init(elev=32, azim=-58)
    try:
        ax2.set_box_aspect((x.ptp(), y.ptp(), max(z.ptp(), .1)))
    except Exception:
        pass

    ax3 = fig.add_subplot(133)
    ax3.scatter(x[slab], y[slab], c=z[slab], s=.35, cmap='plasma', linewidths=0)
    ax3.set_title(f'slice {SLAB[0]}-{SLAB[1]} m (walls & furniture)', fontsize=11)
    ax3.set_aspect('equal'); ax3.grid(alpha=.25, lw=.4)
    ax3.set_xlabel('x (m)'); ax3.set_ylabel('y (m)')
    plt.tight_layout()
    p1 = os.path.join(outdir, f'{stem}_review.png')
    plt.savefig(p1, facecolor=fig.get_facecolor()); plt.close(fig)

    # ---- plan view with axes and the driven path ----
    fig, ax = plt.subplots(figsize=(9.5, 10), dpi=125)
    fig.patch.set_facecolor('#f7f7f5')
    ax.scatter(x[slab], y[slab], c=z[slab], s=.6, cmap='plasma', linewidths=0, zorder=3)
    ax.plot(px, py, '-', color='#2f6fb0', lw=1.4, alpha=.75, zorder=5, label='where the rover drove')
    ax.plot(px, py, '.', color='#2f6fb0', ms=3, zorder=6)
    L = 0.8
    ax.annotate('', xy=(L, 0), xytext=(0, 0),
                arrowprops=dict(fc='#c23b3b', ec='white', lw=1.2, width=5, headwidth=15), zorder=9)
    ax.annotate('', xy=(0, L), xytext=(0, 0),
                arrowprops=dict(fc='#157f45', ec='white', lw=1.2, width=5, headwidth=15), zorder=9)
    ax.text(L * 1.05, .02, '+X', color='#8a1f1f', fontsize=13, fontweight='bold', va='center')
    ax.text(.03, L * 1.05, '+Y', color='#0d5e33', fontsize=13, fontweight='bold')
    ax.plot(0, 0, 'o', ms=15, mfc='#2f6fb0', mec='white', mew=2, zorder=10)
    ax.text(.07, -.24, 'START', color='#1c4a75', fontsize=10, fontweight='bold', zorder=10)
    ax.set_xlabel('x (m)    +x = rover forward at the start of the run', fontsize=11)
    ax.set_ylabel('y (m)    +y = rover left at the start', fontsize=11)
    ax.set_title(f'{stem} — slice {SLAB[0]}-{SLAB[1]} m'
                 f'{"" if a.keep_rover else " — rover body removed"}', fontsize=12.5, pad=12)
    ax.set_aspect('equal'); ax.grid(alpha=.3, lw=.5); ax.legend(loc='upper left', fontsize=9)
    plt.tight_layout()
    p2 = os.path.join(outdir, f'{stem}_plan.png')
    plt.savefig(p2, facecolor=fig.get_facecolor()); plt.close(fig)

    print(f'\nwrote {p1}\n      {p2}\n      {ply}')
    print("""
NOW HAVE THE OPERATOR ANSWER THESE. Q1 decides everything.
  Q1  Any wall drawn TWICE? Two parallel lines a few cm apart in the plan view,
      or a corridor at two slightly different angles. That is the drift
      signature. If none, the geometry is sound.
  Q2  Is the shape the room? Proportions, doorways, where the corners fall.
  Q3  Recognisable objects in the true-colour panel, in the right places
      relative to each other?
  Q4  Anything MISSING that was driven past? That is a coverage gap, not a
      geometry error - a different problem with a different fix.
  Q5  Is the floor flat? Orbit to eye level. A floor that slopes or bends means
      accumulated pitch error, which a wall test never checks.""")


if __name__ == '__main__':
    main()
