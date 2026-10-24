# Synthetic generator: noise calibration against the recordings

Measured with `python3 -m treemapper_tools.synth_calibration --dataset /data/work/extracted/isaac_2026-10-23-15-34-29
--real-bag /data/real/BB_treehopping_farm.bag --real-start 1768476150` (raw numbers: `synthetic_calibration.json`)
plus a one-off probe of the real disparity (three 20-frame windows, numbers below). Generator parameters live
in `tools/treemapper_tools/synth.py` (`LEVELS`).

## What the recordings show

| Quantity | Isaac sim (recorded mission) | Real flight (`BB_treehopping_farm.bag`) |
|---|---|---|
| Disparity valid fraction | 1.000 | 0.967-0.984 |
| Disparity noise, robust 2nd-difference estimate (stride-4 grid) | 0.002 px (simulator depth, no stereo matching) | 0.04 px median (smooth regions) |
| Temporal frame-to-frame |Δd| at a fixed pixel, p50 / p90 / p99 | n/a | 0.06-0.13 / 0.4-1.0 / 0.9-3.4 px (includes motion) |
| Quantisation | float | 1/16 px (SGBM) |
| Disparity range seen | 4.3-8.2 px (15 m flight) | 3.5-16 px (lower flight) |
| Detector: fully visible trees detected (IoU >= 0.3 vs projected canopy) | 100 % (326 instances) | not measured (no truth) |
| Detector: false-positive boxes per frame | 0.07 | n/a |
| Detector: box centre error (std, fraction of box size, x / y) | 0.033 / 0.030 | n/a |
| Detector: box size ratio mean (std), w / h | 0.95 (0.05) / 0.96 (0.07) | n/a |
| Detector: boxes covering two trees | 0 % (sim trees 6 m apart, never touching) | n/a |
| Detector: scores TP p10/p50/p90; FP | 0.87/0.90/0.92; 0.81/0.83/0.86 | n/a |

Reading: the sim's disparity is essentially noise-free and its detector is near perfect on well-separated
trees. Real stereo has a per-pixel noise of roughly 0.1 px with heavy tails (p99 of the frame difference
1-3 px), ~2-3 % dropouts, and 1/16 px quantisation.

## Generator levels (harder than reality on purpose, brief section 8)

| Parameter | easy | medium | hard | Anchor |
|---|---|---|---|---|
| Disparity sigma (white), px | 0.15 | 0.30 | 0.50 | real ~0.1 px -> 1.5x / 3x / 5x |
| Correlated low-frequency disparity error, px | 0.05 | 0.10 | 0.20 | (none measured; pessimistic) |
| iid dropouts / blob dropouts | 3 % / 2 % | 8 % / 5 % | 15 % / 8 % | real 2-3 % |
| Uniform outliers (mismatches) | 0.5 % | 1.5 % | 3 % | real heavy tail p99 1-3 px |
| Foreground fattening at depth edges (probability per edge cell) | 0.3 | 0.6 | 0.9 | typical block-matching artefact |
| Noise multiplier in corridor turns (+1 s) | 2x | 3x | 4x | brief: blur + noise bursts at turns |
| Detector miss rate (sticky, x1.5 at the border) | 8 % | 15 % | 25 % | sim 0 % |
| Box centre jitter (fraction of size) | 4 % | 7 % | 10 % | sim 3 % |
| Box size sd | 6 % | 8 % | 10 % | sim 5-7 % |
| False positives per frame | 0.10 | 0.25 | 0.50 | sim 0.07 |
| Merged box over touching trees (per pair, sticky) | 15 % | 35 % | 60 % | sim 0 % (never touching) |
| Partial (half) boxes | 3 % | 7 % | 12 % | |
| Pose noise xy/z sd, bias random walk, attitude sd | 3 cm, 1 cm/sqrt(s), 0.1 deg | 5 cm, 2, 0.2 deg | 10 cm, 4, 0.4 deg | plain GNSS/INS |
| In-row spacing (mean, sd) / canopy radius | 6.0 m / 1.9-2.7 m | 5.0 m / 1.9-2.8 m | 4.5 m / 1.8-2.9 m | sim 6 m / 2.46-2.81 m |
| Resulting canopy gap | ~1.3 m | ~0.3 m, many touching | mostly touching/overlapping | sim 0.7-1.2 m |
| Canopy top above ground | 3.5-5.5 m | 3.0-6.0 m | 2.5-6.5 m | sim ~5.4-5.9 m (measured, GT says 4.0) |
| Missing trees / ground slope / clutter (bushes per m^2) | 5 % / 0 % / 0.010 | 10 % / 3 % / 0.025 | 15 % / 6 % / 0.045 | sim flat, no clutter |
| Scan speed | 4 m/s | 4 m/s | 5 m/s | sim 4 m/s |

Fixed for all levels: the recorded camera (640 x 400 half resolution, fx = 454.53 px, B = 0.16 m, exact nadir
mount with the recorded lever arm), scan at 15 m, hop transit 10.5 m, spray hover at 7 m, frames at 15 Hz while
awake (recording: ~27 Hz detections), detector asleep in IDLE / SPRAYING / RETURN (heartbeats at 5 Hz), pose
invalid for the first second of SCAN_APPROACH (start-up hold). Tree outlines are irregular (3 harmonics), not
circles; truth heights are the canopy-top world z of each tree.

Not modelled (known gaps): leaf-level occlusion holes inside canopies, real motion blur on the images (only its
effect: noise and detector degradation), wind motion of canopies, stereo calibration errors, time-sync error
between pose and image, a tilted real camera mount (the real bag's `/tf_static` has `body -> sensor_down` pitched
by ~75 deg about y, i.e. ~15 deg off nadir — the real-flight reader must use it; untested).
