# ros2_ws

ROS 2 workspace for rover/drone integration and collision-stop tuning.

## Branching
- Primary working branch: `main_dev`
- Stable reference branch: `main`
- Latest release branch: `release/2026-02-22`
- Latest release tag: `release-20260222`

## Current Update (2026-02-22)
- Collision-stop tuning updated in:
  - `src/rov_collision_stop/config/tune-20250910-104016.yaml`
- Parameter adjustments:
  - `pre_brake_level`: `0.9` -> `0.5`
  - `slow_zone_m`: `2.0` -> `1.5`

## Typical Workflow
```bash
git checkout main_dev
git pull
# make changes
git add <files>
git commit -m "your message"
git push origin main_dev
```

## Release Workflow
```bash
# from main_dev after validation
git checkout main
git merge --ff-only main_dev
git push origin main
```
