# Real-flight camera chain: verification without ground truth

Data: `/data/real/BB_treehopping_farm.bag` (7153 disparity frames, 511 s) and its companion bag (7157 detection
messages). Frames are read by `tools/treemapper_tools/real_frames.py` into `/data/work/extracted/real_farm/frames.bin`.
- 7164 frames: 7145 paired with disparity by exact stamp, 12 without disparity, 7 heartbeats.
- 0 frames with an invalid pose.

Commands (tm_dev, at most 2 cores):
```
python3 -m treemapper_tools.real_frames --bag /data/real/BB_treehopping_farm.bag \
    --companion /data/real/BB_treehopping_farm_companion_2026-10-23-18-01-20.bag --out /data/work/extracted/real_farm
taskset -c 0 python3 -m treemapper_tools.real_geometry --frames /data/work/extracted/real_farm/frames.bin \
    --every 2 --grid-every 20 --extra-pitch 78 80 84 --out-json results/real_flight_geometry.json
taskset -c 0 python3 -m treemapper_tools.real_stereo_check --bag /data/real/BB_treehopping_farm.bag \
    --dataset /data/work/extracted/real_farm --out-json results/real_flight_stereo_check.json
```
Raw numbers: `real_flight_geometry.json` and `real_flight_stereo_check.json`.

## The verified chain (reader defaults)

| Element | Value | How it was established |
|---|---|---|
| Pose | `/tf` `world_nwu -> body`, interpolated at the image header stamp (max gap 0.25 s) | Header stamps are flight time (Oct 2025); bag time is replay time (+6554596.89 s). One constant offset puts every stamp on the bag clock. |
| Lever arm body -> camera | (0.30, 0.08, -0.20) m, from tf_static `body -> sensor_down` | Removing it (D) changes nothing measurable, so it is neither confirmed nor contradicted. |
| Mount rotation | **82 deg about body +y (8 deg off nadir, looking forward)**, not tf_static's **75 deg** (15 deg off nadir) | Ground tilt and altitude drift, below |
| Axes | optical (x right, y down, z forward) -> ROS camera frame (x along the optical axis), same as the existing localizer's `sensor_optical_frame: false` | Without the swap (E), no ground plane exists. |
| Intrinsics | left rectified P: fx = fy = 477.15, cx = 322.72, cy = 200.06 (640 x 400) | |
| Baseline | right P[3] = -76633.363 = -fx*B in **millimetres** -> B = 0.1606 m (the same unit as Isaac) | `baseline_from_P` deduces the unit |
| Disparity | **Z = fx*B / (d - 4.30)**: the published disparity carries an offset. The calibration predicts cx_l - cx_r = 4.918 px; the data say **4.30 px**. | Motion test, ground vs altitude, below |
| Mission phase | constant `SCAN`, target -1: the real flight has none | Descent, lock and spray rules cannot be exercised. |

## Evidence: hypotheses compared (every 2nd frame above 8 m: 3397 frames analysed; ~880 give a ground plane; ~360 fully visible boxes)

| Test | **A: 82 deg, off 4.30 (reader)** | T: tf_static 75 deg | B: nadir 90 deg | C: 60 deg | F: 82 deg, calib off 4.918 | G: raw disparity (no offset) |
|---|---|---|---|---|---|---|
| Ground slope per frame, p50 / p90 | **3.9 / 10.0 deg** | 8.8 / 13.0 | 7.1 / 13.0 | 23.1 / 27.0 | 4.0 / 9.3 | 7.8 / 14.9 (*) |
| Mean ground tilt, heading frame (mount error is constant here) | **0.9 deg** | 7.3 deg | 6.9 deg | 22.0 deg | 1.4 deg | |
| Mean ground tilt, world frame (terrain slope is constant here) | **0.4 deg** | 6.5 deg | 7.0 deg | 20.6 deg | 0.8 deg | |
| Tree-position drift along the heading vs altitude, as a pitch error (a pitch error d shifts a crown by tan(d) x (altitude - crown z)) | **-0.8 deg** | +8.8 deg | -16.7 deg | +21.4 deg | -2.3 deg | |
| Ground z p10 / p50 / p90 (world_nwu; take-off body z = -0.21) | -1.7 / **+0.24** / +2.4 m | -1.6 / 0.09 / 2.2 | -1.9 / 0.22 / 2.2 | -3.0 / -0.76 / 1.4 | -3.3 / -1.19 / 0.87 | 4.3 / 6.3 / 9.5 (*) |
| d(ground z) / d(altitude): 0 = depth scale right | **-0.12** | -0.10 | -0.12 | -0.27 | -0.31 | **+0.65** (depth scale 0.35) (*) |
| Canopy top above the frame's ground (box p95), p10 / p50 / p90 | 3.0 / **4.6** / 6.4 m | 3.0 / 4.6 / 6.3 | 2.9 / 4.6 / 6.4 | 3.0 / 4.6 / 6.3 | 3.5 / 5.9 / 9.3 | 0.8 / 1.2 / 1.7 (*) |
| Crown radius (median distance of crown points from their centroid), p50 / p90 | 2.15 / 2.58 m | 2.12 / 2.57 | 2.14 / 2.56 | 2.06 / 2.50 | 2.42 / 3.03 | |
| Repeatability: pass mean -> tree median, p50 / p90 (13 trees, 6-7 seen in >= 2 passes) | **0.27 / 0.50 m** | 0.25 / 0.56 | 0.43 / 0.90 | 0.44 / 0.84 | 0.45 / 0.72 | |
| Repeatability: max pass-to-pass distance per tree, p50 / p90 | **0.65 / 1.15 m** | 0.68 / 1.20 | 0.99 / 1.48 | 1.14 / 1.42 | 1.16 / 1.25 | |
| Box crown centroids -> their tree, p50 / p90 | 0.31 / 0.60 m | 0.31 / 0.70 | 0.29 / 0.90 | 0.35 / 0.79 | 0.44 / 1.02 | |

(*) from the run made before the offset was fixed (`/data/work/eval/m4c/geo_quick.json`; the hypothesis was then called
"F_no_disparity_offset", 567 frames). In the final run, G yields no ground plane: its range gate rejects the
points.

Pitch sweep (ground-slope median, 340 frames, roll -2/0/+2):
- the minimum is broad, 3.5-4.0 deg, for pitch 82-86;
- 75 deg gives 8.8 deg.

The heading-frame tilt crosses zero at about 83 deg. The altitude-drift test gives 81-82 deg (78: +6.1, 80: +1.7,
84: -3.1 deg). **82 deg (+-1.5)** is the value both tests agree on.

Repeatability (0.25-0.7 m) does not separate 75 from 82 deg. With few trees seen at several altitudes it is
noise-limited. The two hypotheses 15 and 22 deg off (B and C) are clearly worse.

## Evidence: depth scale and disparity offset (`real_stereo_check.py`)

1. **The images are not rectified.**
   - Corners tracked from left to right show a vertical offset of 8.75 px as published (p10 to p90: 7.4 to 10.4).
   - After rectifying with the published K, D, R and P, the offset is 1.05 px (0.77 to 1.48).
   - So the calibration is close to the one the disparity node used, but not identical.
2. **The depth scale from motion does not depend on altitude or pose z.**
   - Method: features are tracked between left frames k and k+4 (0.28 s, 212 pairs, 8-19 m). Their position in
     k+4 is predicted from depth, the chain and the two `/tf` poses.
   - Free depth scale s at the reader's chain: **s = 0.99** (IQR 0.958-1.03). By altitude band (median depth 8, 10,
     13 and 16 m), s = 1.00 / 1.00 / 0.99 / 0.955.
   - Median prediction error: 2.8 px at the best s, 3.2 px at s = 1, 39 px if translation is ignored.
   - Joint grid over (fx*B, offset): fx*B and offset are correlated, with the best values at (78, 4.1) and (76, 4.3).
     **At the calibration's fx*B, the best offset is 4.3 px** (3.36 px error). The calibration's 4.9 gives 4.51 px.
   - With the calibration offset, depths come out 7 % too long at 9 m and 16 % too long at 18 m. That matches F's
     ground sinking with altitude (-0.31 m/m).
   - Without any offset (G), depth is 2.8x too short: the ground follows the drone (d ground z / d altitude = +0.65).

## Findings

- **tf_static's 75 deg mount is wrong by ~7 deg** for this flight's chain. The error could be in the mount itself or
  in a constant attitude bias: the body reads a pitch of +3.2 deg nose-down while hovering. The stereo data fit
  82 deg either way. With 75 deg, crowns move about 0.15 m per metre of altitude along the heading, for example
  about 1 m between the 11.5 m and 18.5 m passes.
- **The disparity needs an offset of 4.30 px, not cx_l - cx_r = 4.918.** The existing system's localizer uses the
  Q matrix from these camera_info messages, so it probably applies 4.918. Its depths are then 7-16 % long; this is
  an inference, not checked in its code.
- Canopy tops sit **4.6 m (p50; p10 3.0, p90 6.4) above the per-frame ground**. Crowns have a radius of about 2.2 m
  and sit about 8 m apart in rows. That is plausible for this orchard.

## Unverified / limits

- **Ground estimator.** Low envelope plus RANSAC works in only ~26 % of frames: the canopy covers most of the view.
  Per-frame ground z spreads over +-2 m. Some of that is terrain or pose-z drift (plain GNSS); the 17.5-25 m band
  reads -1.8 m against about +0.5 m elsewhere. The altitude-scale test therefore rests on the motion test.
- **Box pixels.** The detector ran on the raw (unrectified) left image, while the disparity lives in an unknown
  (rectified?) frame. Boxes are used as if both were rectified, as the existing pipeline does. The difference is
  up to ~13 px at the image edges (4 % scale, distortion k1 = 0.05). An edge-alignment test between image and
  disparity could not tell the two apart.
- **Roll.** Not estimated beyond +-2 deg: the ground-slope search is flat in roll, and the drift test is noisy
  (1-5 deg).
- **Crown tops vs altitude.** The crown-top world z falls by 0.30 m per metre of altitude (A). This is not
  explained by the depth scale, which the motion test puts at 0.99. Box clipping and visibility with altitude are
  the likely causes. Not resolved.
- **Lever arm.** Taken from tf_static, unverified: it is below the noise.
- **Pose z.** It comes from the flight controller (GNSS / baro). Its drift over the 8 min flight is unknown.
  Published heights are world z and inherit any such drift.
