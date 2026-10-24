# Camera geometry check against ground truth (recorded Isaac mission)

Command (tm_dev):
```
python3 -m treemapper_tools.bag_frames --bag /data/isaac/cloud_bundle_isaac_2026-10-23-15-34-29.bag \
    --extracted /data/work/extracted/isaac_2026-10-23-15-34-29 --out /data/work/frames/isaac_full.bin
python3 -m treemapper_tools.geometry_check --frames /data/work/frames/isaac_full.bin \
    --gt /data/work/extracted/isaac_2026-10-23-15-34-29/ground_truth.json --out-json results/geometry_check.json
```
546 frames with detections and disparity, evenly sampled over the whole mission (102 SCAN, 444 HOPPING),
drone altitude 6.95-15.07 m; disparity sampled on a stride-4 grid (160 x 100 points per frame);
Z = fx * B / d with fx = 454.53 px, B = 0.160 m (right P[3] = -72724.8 = -fx*B in mm).

## Finding: `/tf_static` is the camera pose at spawn, not a mount

The bag's `/tf_static` parents `bottom_left_camera` to `world`. M1 guessed it is the body->camera mount.
It is not: `inv(world->body at spawn) * tf_static` is an **exact** nadir mount,
R_body_cam = [[0,-1,0],[-1,0,0],[0,0,-1]] (residual < 1e-6), lever arm (0.150, 0.080, 0.000) m. The
3.37 deg in the raw quaternion is the drone's resting pitch on the ground at spawn (world->body at
t = 10 s has q = (0, -0.0294, 0, 0.9996)), baked into a world-parented static transform. Using the raw
transform as the mount (M1's guess) therefore tilts the camera by 3.37 deg in every frame of the flight.
The reader now derives the mount (`bag_frames.static_to_body_cam`), image optical frame -> body -> world.

## Numbers per hypothesis

| Metric | **A derived (default)** | B derived, no lever arm | C M1 guess (raw tf_static as mount) | D C with inverted rotation |
|---|---|---|---|---|
| Ground-plane slope, median / p90 (flat sim ground: 0) | **0.01 / 0.06 deg** | 0.01 / 0.06 deg | 3.37 / 3.38 deg | 3.37 / 3.37 deg |
| Ground height median (p10..p90) | **0.013 m (0.010..0.018)** | 0.012 (0.006..0.034) | 0.131 (-0.028..0.281) | 0.140 (-0.229..0.345) |
| Canopy points (in a box, z > 2 m) inside a true canopy circle | **99.6 %** | 99.0 % | 95.5 % | 95.0 % |
| Per-tree accumulated canopy centroid - true centre, mean / max (9 trees) | **0.35 / 0.73 m**, mean vector (-0.02, -0.01) | 0.37 / 0.75 | 0.35 / 0.81 | 0.38 / 0.73 |
| Box top-30 % median xy -> nearest true centre, median / p90 | 0.75 / 1.94 m | 0.72 / 1.98 | 0.84 / 2.03 | 0.76 / 2.04 |
| In-canopy point height - truth height (range < 12 m), p50 / p90 / p99 | -0.16 / **+1.35 / +1.88 m** | same | -0.05 / +1.44 / +1.97 | -0.08 / +1.44 / +1.97 |

Reading the numbers:
- The ground-plane test is the sharpest discriminator: a rotation error shows up directly as a slope.
  Only the derived chain makes the ground flat (0.01 deg) and at height 0 to 1-2 cm, at every altitude
  (6.95-15 m), which also confirms the depth scale fx*B (a 1 % scale error would put the ground ~15 cm off at 15 m).
- 99.6 % of in-box points above 2 m land inside a true canopy circle; the aggregated canopy centroids sit
  0.35 m (mean) from the true centres with no systematic offset (mean vector 2 cm). The residual is the
  visibility bias (each tree is seen from one side at a time, edge trees from one side only) plus the
  circle approximation of the truth. The per-box metric (0.75 m) is larger because boxes clipped at the
  image border see only part of a tree; it is a weak test and is listed for completeness.

## Finding: ground-truth canopy height is ~1.4-1.9 m too low

With the verified chain, points inside the true canopy circles reach **+1.35 m (p90) and +1.88 m (p99)
above the ground-truth height (4.0 m for every tree)**, consistently from 7 m and 15 m altitude, with the
ground at 0.01 m. The simulated canopies therefore top out at about 5.4-5.9 m, not 4.0 m. The existing
system's recorded height bias of +1.41 m (DATA_README, baseline scorecard) is consistent with this: it is
mostly a ground-truth artefact, not an existing-system error. **Every height-error number scored against
this ground truth is biased by about +1.4 m** for any system that measures the canopy top correctly.
The scorer and ground truth are not changed here (they belong to the scorer fixer); recorded in DECISIONS
(D24) and reported to the coordinator.

## Other input facts (all frames of the mission)

- `/disparity_image_down_float`: 28874 messages = 14244 valid 32FC1 frames + 14630 **malformed** copies
  (step = width, 256000 bytes of garbage floats); no valid stamp appears twice. The reader rejects by size.
- 7414 detection messages: 6268 paired by exact stamp, 89 paired once a late disparity arrived, 1010 paired
  to the nearest valid disparity within 20 ms (pose taken at the disparity stamp), 47 without disparity.
- Detection boxes are (x, y) = top-left corner, width, height in pixels of the 640 x 400 left image.
