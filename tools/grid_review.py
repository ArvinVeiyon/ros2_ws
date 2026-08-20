#!/usr/bin/env python3
"""Assemble and inspect the 2D OCCUPANCY GRID inside an RTAB-Map database.

WHY THIS EXISTS
  One database, two products. `tools/map_review.py` renders the CLOUD - the layer
  a human recognises as their room, and the layer relocalization matches against.
  A PLANNER consumes neither of those: it consumes the 2D occupancy grid.

  For house_map_v4 the cloud was operator-verified and the scale was tape-verified,
  and on that basis the map was called "good". The grid was never examined - every
  other map version on this machine has a <name>_map.pgm export sitting beside it
  and v4 has none. autonav_reference.md section 13 records it as
  "NOT ESTABLISHED ... check the grid before blaming a planner". This is that check.

WHAT IT READS
  RTAB-Map stores per-node local grids in Data.{ground,obstacle,empty}_cells as
  zlib-compressed float32, FOUR floats per point (x, y, z, pad) in the node's local
  frame - NO header, despite these being cv::Mat elsewhere in the schema. Node.pose
  is 12 float32, a row-major 3x4 [R|t]. Points are transformed by their node pose
  and rasterized at Data.cell_size.

WHAT IT CHECKS
  * rover-plate contamination: obstacle points falling within ROVER_RADIUS of the
    pose that observed them. The ROI mask that should crop the plate is SILENTLY
    DISCARDED when the cropped depth height does not divide by Grid/DepthDecimation
    (v4 was built with decimation 4; the fix was 2), so this is a real risk and it
    would appear to a planner as an obstacle trail along the whole driven path.
  * free/occupied/unknown budget - a grid that is mostly unknown cannot be planned
    through however pretty the cloud looks.
  * enclosure - whether the free space is bounded by obstacles, or leaks to the edge.

    python3 tools/grid_review.py ~/house_map_v4.db
    python3 tools/grid_review.py ~/house_map_v4.db --out /tmp/gridcheck
"""
import argparse
import os
import sqlite3
import sys
import zlib

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Same figure map_review.py uses: plate is 0.730 m long, base_link 0.345 m from
# the front tip, so 0.45 m horizontal covers the plate with margin.
ROVER_RADIUS = 0.45


def load(dbpath):
    db = sqlite3.connect(dbpath)
    c = db.cursor()
    poses = {}
    for nid, blob in c.execute("SELECT id, pose FROM Node WHERE pose IS NOT NULL"):
        m = np.frombuffer(bytes(blob), dtype='<f4')
        if m.size != 12:
            continue
        poses[nid] = m.reshape(3, 4).astype(np.float64)

    layers = {}
    for name in ('obstacle_cells', 'ground_cells', 'empty_cells'):
        pts, owner = [], []
        q = f"SELECT id, {name}, cell_size FROM Data WHERE length({name})>0"
        for nid, blob, cs in c.execute(q):
            if nid not in poses:
                continue
            a = np.frombuffer(zlib.decompress(bytes(blob)), dtype='<f4')
            if a.size % 4:
                continue
            p = a.reshape(-1, 4)[:, :3].astype(np.float64)
            T = poses[nid]
            g = p @ T[:, :3].T + T[:, 3]
            pts.append(g)
            owner.append(np.full(len(g), nid))
        layers[name] = (np.vstack(pts) if pts else np.zeros((0, 3)),
                        np.concatenate(owner) if owner else np.zeros(0, int))
    cell = next(c.execute("SELECT cell_size FROM Data WHERE cell_size IS NOT NULL LIMIT 1"))[0]
    db.close()
    return poses, layers, float(cell)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('db')
    ap.add_argument('--out', help='output directory (default: beside the database)')
    ap.add_argument('--rover-radius', type=float, default=ROVER_RADIUS)
    a = ap.parse_args()

    if not os.path.exists(a.db):
        print(f'no such database: {a.db}', file=sys.stderr)
        return 2
    out = a.out or os.path.dirname(os.path.abspath(a.db))
    os.makedirs(out, exist_ok=True)
    base = os.path.splitext(os.path.basename(a.db))[0]

    poses, layers, cell = load(a.db)
    obs, obs_owner = layers['obstacle_cells']
    gnd, _ = layers['ground_cells']
    emp, _ = layers['empty_cells']
    print(f'{base}: {len(poses)} poses, cell {cell:.3f} m')
    print(f'  obstacle {len(obs):>8d}   ground {len(gnd):>8d}   empty {len(emp):>8d}')

    # --- rover-plate contamination -------------------------------------------
    # Distance from each obstacle point to the pose that observed it. Points
    # inside the rover's own footprint cannot be real obstacles.
    if len(obs):
        origins = np.array([poses[n][:, 3] for n in obs_owner])
        d = np.linalg.norm(obs[:, :2] - origins[:, :2], axis=1)
        near = d < a.rover_radius
        print(f'\nrover-plate check (obstacle points within {a.rover_radius:.2f} m '
              f'of their own pose):')
        print(f'  {near.sum()} of {len(obs)} = {100*near.mean():.2f}%')
        print(f'  closest obstacle point overall: {d.min():.3f} m')
        if near.mean() > 0.02:
            print('  🔴 CONTAMINATED - a planner sees an obstacle trail along the driven path.')
        else:
            print('  ✅ clean - the plate is NOT in the obstacle grid.')

    # --- occupancy budget ----------------------------------------------------
    allpts = np.vstack([p for p in (obs, gnd, emp) if len(p)])
    lo = allpts[:, :2].min(axis=0) - 0.5
    hi = allpts[:, :2].max(axis=0) + 0.5
    W = int(np.ceil((hi[0]-lo[0])/cell)); H = int(np.ceil((hi[1]-lo[1])/cell))
    grid = np.full((H, W), -1, dtype=np.int8)          # -1 unknown

    def stamp(pts, val):
        if not len(pts):
            return
        ix = ((pts[:, 0]-lo[0])/cell).astype(int)
        iy = ((pts[:, 1]-lo[1])/cell).astype(int)
        ok = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
        grid[iy[ok], ix[ok]] = val

    stamp(emp, 0); stamp(gnd, 0); stamp(obs, 100)      # obstacles win

    n = grid.size
    free = int((grid == 0).sum()); occ = int((grid == 100).sum())
    unk = int((grid == -1).sum())
    print(f'\ngrid {W} x {H} cells = {W*cell:.2f} x {H*cell:.2f} m')
    print(f'  free     {free:>8d}  {100*free/n:5.1f}%')
    print(f'  occupied {occ:>8d}  {100*occ/n:5.1f}%')
    print(f'  unknown  {unk:>8d}  {100*unk/n:5.1f}%')
    print(f'  free area {free*cell*cell:.2f} m^2')

    # --- enclosure: does free space touch the grid border? -------------------
    border = np.zeros_like(grid, dtype=bool)
    border[0, :] = border[-1, :] = True
    border[:, 0] = border[:, -1] = True
    leak = int(((grid == 0) & border).sum())
    print(f'\nenclosure: {leak} free cells on the border '
          f'{"🔴 free space LEAKS off the map" if leak else "✅ free space is bounded"}')

    # --- render ---------------------------------------------------------------
    img = np.full((H, W, 3), 0.5)
    img[grid == 0] = (1, 1, 1)
    img[grid == 100] = (0, 0, 0)
    fig, ax = plt.subplots(figsize=(10, 10*H/max(W, 1)))
    ax.imshow(img, origin='lower', extent=[lo[0], hi[0], lo[1], hi[1]])
    P = np.array([poses[n][:, 3] for n in sorted(poses)])
    ax.plot(P[:, 0], P[:, 1], '-', color='tab:red', lw=1.0, label='driven path')
    ax.set_title(f'{base} — 2D occupancy grid a planner would consume\n'
                 f'{W*cell:.1f} x {H*cell:.1f} m, {cell*100:.0f} cm cells, '
                 f'free {100*free/n:.1f}% / occ {100*occ/n:.1f}% / unknown {100*unk/n:.1f}%')
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.legend(loc='upper right')
    png = os.path.join(out, f'{base}_grid.png')
    fig.tight_layout(); fig.savefig(png, dpi=130); plt.close(fig)
    print(f'\nwrote {png}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
