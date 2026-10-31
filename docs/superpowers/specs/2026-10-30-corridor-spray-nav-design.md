# Corridor spray navigation and an Isaac-only closed loop: design

Date: 2026-10-30. Status: approved design, not implemented.

## 1. Goal

A new navigation node, `corridor_nav`, that sprays an orchard **corridor by corridor**: it scans one corridor, then
sprays that corridor's trees on the way back, flying terrain-following from tree to tree, then moves on to the next
corridor. It flies on TreeMapper's map (`cand_obj`, unchanged) and uses TreeMapper's tree IDs directly.

The whole system must run closed loop in **Isaac Sim alone**: no Orin, no SpotSpraying, no TreeHoppingNavigation, no
ArduPilot SITL. Isaac renders the orchard and the stereo pair, the real detector and a real stereo matcher run on the
workstation, TreeMapper builds the map, `corridor_nav` flies the drone and opens the nozzle, and Isaac measures where
the spray lands. One command runs a mission and prints a scorecard.

The design was made without reference to the existing spot-spray navigator.

### 1.1 Decisions taken with the user

| Question | Decision |
|---|---|
| What is corridor spraying | The field is divided into long corridors. Within a corridor: go to a tree, spray it, go to the next tree in the corridor. Then the next corridor. |
| Scanning vs spraying | **Scan out, spray back**: fly the corridor at scan altitude to map it, lock its tree list, spray its trees in reverse order on the way back. |
| Per-tree flight | **Terrain-following, no climb**: each tree sets its own spray altitude from its published canopy top; the drone glides directly from one tree's spray altitude to the next, climbing only when a canopy is in the way. |
| Isaac-only inputs | **Real detector + real stereo**: the YOLOX detector and an SGBM stereo matcher run on the rendered images. |
| Architecture | **ROS-free C++17 core + thin ROS 1 node**, Isaac (ROS 2) reached through `ros1_bridge`. |

### 1.2 Out of scope

- A flight backend for the real drone (MAVROS / ArduPilot). The node talks to the vehicle through a `FlightBackend`
  interface; only the Isaac backend is built.
- Getting the field polygon from GNSS on a real flight. In this spec the field polygon is given in the map frame.
- Wind and spray drift.
- Changes to TreeMapper. Its known low canopy-top bias (median -0.58 m, worst -1.9 m on
  `results/isaac_5x5_tropical.md`) is compensated by the spray clearance and an independent clearance guard (section
  4.6); fixing it in TreeMapper is a separate follow-up.

## 2. Architecture

```
┌──────────────── Isaac Sim 5.1 (ROS 2 Humble) ─ sim/isaac/orchard_sim.py ────────┐
│ seeded orchard from spec.json (rows, jitter, gaps, species, scale, slope)       │
│ kinematic drone: tracks setpoints, v/a limited, tilt = atan(a/g)                │
│ stereo pair (real half-res intrinsics + baseline + verified nadir mount)        │
│ nozzle: ray cone onto per-tree canopy colliders → deposition per tree / ground  │
│ ground truth from the placed meshes; /tf, /clock                                │
└──────────────┬────────────────────────────────────────────▲─────────────────────┘
   images, camera_info, tf, clock, truth, deposition     setpoint, spray command
              ros1_bridge parameter_bridge (fixed topic list, Foxy ↔ Noetic)
┌──────────────▼── ROS 1 Noetic ─────────────────────────────┴─────────────────────┐
│ stereo_sgbm_node  → /disparity_image_down_float (32FC1)                          │
│ detector_node     → /object_detection/detections (treemapper_msgs/Detection2DArray)
│ treemapper_node (treemapper_sim.launch, unchanged)                               │
│     → /treemapper/map, /treemapper/get_trees, /treemapper/set_tree_sprayed       │
│ corridor_nav_node ← /treemapper/map, disparity, camera_info, tf, field file      │
│     → /mission_state, /spray_target, /corridor_nav/setpoint, /corridor_nav/spray │
│ rosbag record → tools/treemapper_tools/mission_score.py → scorecard              │
└──────────────────────────────────────────────────────────────────────────────────┘
```

### 2.1 New code, by location

| Path | What | Language |
|---|---|---|
| `nav/corridor_nav/` | ROS-free navigator core: corridor planner, ordering, glide planner, state machine, guards, config | C++17, gtest |
| `ros/corridor_nav_msgs/` | `NavStatus.msg` | ROS 1 msgs |
| `ros/corridor_nav_ros/` | thin node, Isaac flight backend, launch, `nav_sim_lite` rostest | C++17 |
| `ros/sim_perception/` | `stereo_sgbm_node`, `detector_node` | Python 3.8 (Noetic) |
| `sim/isaac/` | `orchard_sim.py` (entry), `scene.py`, `drone.py`, `cameras.py`, `nozzle.py`, `truth.py` | Python 3.11 (Isaac) |
| `sim/bridge/` | `Dockerfile` (Ubuntu 20.04, Noetic + Foxy, `ros1_bridge`), `bridge_topics.yaml` | |
| `sim/specs/` | example scene specs (`3x3_smoke.json`, `5x5_tropical.json`, `rows_8x12.json`) | JSON |
| `config/corridor_nav.cfg` | every navigator parameter; built-in defaults equal this file (test-enforced) | |
| `tools/treemapper_tools/mission_score.py` | closed-loop mission scorer | Python 3.8 |
| `scripts/sim_mission.sh` | one command: start everything, record, score | bash |

The navigator core follows TreeMapper's conventions: plain inputs and outputs, decision logic in small pure functions,
one config file with clamping and logging in one place, every drop / hold / skip counted by reason.

## 3. Interfaces

### 3.1 With TreeMapper (no changes to TreeMapper)

| Direction | Name | Type | Use |
|---|---|---|---|
| in | `/treemapper/map` | `treemapper_msgs/TreeMap` (latched, 5 Hz) | the tree list; heights are world-frame canopy-top z |
| call | `/treemapper/set_tree_sprayed` | `treemapper_msgs/SetTreeSprayed` | after a valid dose |
| out | `/mission_state` | `std_msgs/String` | phase, table in section 4.4 |
| out | `/spray_target` | `std_msgs/Int32` | current target, TreeMapper ID, -1 = none |

### 3.2 With the vehicle (Isaac backend)

`geometry_msgs/PoseStamped` on `/corridor_nav/setpoint` at 20 Hz: `header.frame_id` = map frame (world in Isaac),
`pose.position` = the carrot point on the planned 3D path, `pose.orientation` = yaw only. The carrot advances at the
navigator's chosen speed, so no speed limits cross the interface, and only standard message types cross the bridge
(amended 2026-10-31 in the implementation plan; a custom message would have to be built for Isaac's internal ROS 2
Python).

`/corridor_nav/spray`: `std_msgs/Bool` (nozzle open), published at 20 Hz while the node runs and immediately on change.
The sim closes the nozzle when no spray message arrived for 0.5 s (fail closed on the vehicle side too).

### 3.3 Operator and diagnostics

- Field: `~field_file`, YAML with `polygon: [[x, y], ...]` in the map frame (>= 3 vertices) and optional
  `heading_rad`, `home: [x, y]`. In sim, `sim_mission.sh` writes it from the scene spec.
- `~start` (`std_srvs/Trigger`): begin the mission from IDLE. `~abort` (`std_srvs/Trigger`): close the nozzle and RETURN.
- `corridor_nav_msgs/NavStatus.msg` on `~status` at 2 Hz:

```
Header header
string state                  # nav state (section 4.4)
int32 corridor                # current corridor index, -1 = none
int32 corridors_total
int32 target_id               # TreeMapper ID, -1 = none
uint32 trees_sprayed
uint32 trees_skipped
uint32 trees_remaining        # in locked corridors + cleanup list
float64 clearance_m           # latest nadir canopy clearance from stereo, NaN if invalid
string[] reasons              # counter names (holds, guard triggers, climbs, retries, skips, re-resolved IDs, ...)
uint64[] counts               # cumulative, same length as reasons
string[] warnings             # active warnings
```

- `~plan_markers` (`visualization_msgs/MarkerArray`): corridors, order, planned glide paths, for RViz.

## 4. Navigator core (`nav/corridor_nav`)

All functions below are pure and individually tested.

### 4.1 Corridor planner: `planCorridors(field, heading, camera, cfg) -> [Corridor]`

- Heading: `field.heading_rad` if given, else the long axis of the field polygon's minimum-area bounding rectangle.
- Maximum width that keeps a whole crown in view when its centre is inside the strip:
  `w_max = 2 · (z_scan − z_top_max) · tan(fov_min / 2) − 2 · r_canopy_max − w_margin`, where
  `fov_min = min(hfov, vfov)` from the left `camera_info` (`fov = 2·atan(size / (2·f))`; the smaller one is used so
  the result does not depend on the camera's yaw on the body), `z_scan` = scan altitude, `z_top_max` =
  `expected_canopy_top_max`, `r_canopy_max` = `expected_canopy_radius_max`.
- **Wide-axis alignment (amended 2026-10-31, user decision).** When the camera mount is known, the drone flies corridor
  work with the image x-axis (the wide, 640 px axis) across track, and `w_max` uses `hfov` instead of `fov_min`. With the
  real half-res camera (640×400, fx ≈ 455: hfov 70.2°, vfov 47.5°) `fov_min` gives `w_max` ≈ 1.0 m at the defaults,
  below `corridor_width_min`, so planning would refuse; `hfov` gives ≈ 5.25 m, about one corridor per tree row. Yaw
  during corridor work = `heading + π/2 − φ`, where φ is the yaw, in the body frame, of the optical camera's x-axis
  (from the camera mount). Without a known mount the `fov_min` rule and yaw = heading stay.
- Width used: `w = w_max` when `corridor_width` is 0 (the default), else `w = min(corridor_width, w_max)`; if
  `corridor_width > w_max`, warn and count `corridor_width_clamped`. Example: `fov_min` 70°, `z_scan` 15 m,
  `z_top_max` 7 m, `r_canopy_max` 2.5 m, `w_margin` 1 m → `w_max` ≈ 5.2 m.
  If `w_max < corridor_width_min`, refuse to plan (fail closed, ERROR state, logged): the camera cannot cover a corridor.
- The field is cut into `n = ceil(field_extent_across / w)` strips of equal width, clipped to the polygon. Corridors
  shorter than `corridor_length_min` are dropped (counted).
- Order: every corridor is scanned in the same direction (`heading`), entry on the same side of the field. Corridor
  k's spray-back ends at its scan start (its entry), which is one corridor width from corridor k+1's entry, so the
  hop between corridors is short. (Corrected 2026-10-31: alternating directions would put k+1's entry at the far end,
  a full corridor length away. Boustrophedon applies only to `field_first`'s scan sequence, where the navigator
  scans odd corridors exit → entry.)
- Each corridor has a centre line (scan path, from entry point to exit point) and an axial coordinate `s` along it.

### 4.2 Tree assignment

- When corridor k locks, every published, unsprayed tree whose `centre` lies inside corridor k's strip and is not yet
  assigned is assigned to k. Assignment is permanent.
- A tree published later whose centre lies in an already locked corridor goes to the **cleanup list** (counted
  `late_trees`).
- A tree whose centre lies outside the field polygon is never targeted (counted `trees_outside_field`).

### 4.3 Spray order: `orderTargets(trees, start, end, cfg) -> [id]`

- Start: corridor k's exit point (where the scan ended). End: corridor k's entry point.
- Initial order: by axial coordinate `s`, descending (the return direction).
- Improvement: 2-opt on the open path start → trees → end with cost = planned glide time (section 4.5), stopped after
  `order_budget_ms` (5 ms) or when no improving swap remains. This handles corridors that hold two staggered rows.
- The cleanup list is ordered the same way, from the drone's position to home.

### 4.4 State machine: `step(State, Inputs, cfg) -> (State, Outputs)`

`Inputs` = time, drone pose, latest map snapshot, nadir clearance, service results. `Outputs` = setpoint, spray flag,
mission phase, target ID, service requests, counter increments.

| Nav state | Exit condition | `/mission_state` | `/spray_target` | Nozzle |
|---|---|---|---|---|
| IDLE | `~start` called, field valid, pose valid | IDLE | -1 | off |
| TAKEOFF | at `z_scan` above home | SCAN_APPROACH | -1 | off |
| TRANSIT(k) | at corridor k's entry point at `z_scan` | SCAN_APPROACH | -1 | off |
| SCAN_OUT(k) | at corridor k's exit point | SCAN | -1 | off |
| SCAN_SETTLE(k) | hovering at the exit point, no new tree published in corridor k for `settle_s` (2 s), or `settle_timeout_s` (8 s, counted) | SCAN | -1 | off |
| LOCK(k) | assignment (4.2) and order (4.3) done, held `lock_hold_s` (0.5 s) | PLANNING | -1 | off |
| GLIDE(id) | arrived at the target's spray point (4.5) | HOPPING | id | off |
| DOSE(id) | dose time elapsed (4.6) | SPRAYING | id | on while `xy_err <= xy_tol` |
| (dose valid) | `set_tree_sprayed(id, true)` succeeded → next target, or next corridor's TRANSIT | HOPPING | next id | off |
| CLEANUP | cleanup list empty | HOPPING / SPRAYING | id | as GLIDE / DOSE |
| RETURN | above home | RETURN | -1 | off |
| LAND | landed | RETURN | -1 | off |
| HOLD(reason) | reason cleared → previous state; `hold_timeout_s` (20 s) → RETURN | unchanged | unchanged | off |
| ABORT | entered on `~abort`, fatal planning error or repeated HOLD timeout → RETURN | ERROR | -1 | off |

Notes on how this interacts with TreeMapper's phase policy:
- New trees can only appear in SCAN. Settling happens **in SCAN** at the end of the corridor, so trees near the
  corridor end can still be confirmed before the list locks.
- TRANSIT uses SCAN_APPROACH; after the first PLANNING, TreeMapper's sticky lock keeps creation off until SCAN is
  entered again, so no trees appear while transiting between corridors.
- In GLIDE / DOSE only the target may change in TreeMapper, and after SPRAYING it holds non-target trees until the
  drone is 2 m away. Both are compatible with terrain-following glides.
- When corridor k+1 is scanned, TreeMapper may merge instances. Sprayed trees never merge. A planned, unsprayed ID that
  vanishes from the map is re-resolved: the tree whose outline contains the old centre, else the nearest centre within
  `reresolve_radius` (1.5 m), else the target is dropped. Both outcomes are counted (`ids_reresolved`,
  `ids_dropped_vanished`).

### 4.5 Glide planner: `planGlide(from, to, map, cfg) -> Path3D`

- Spray point of tree i: `p_i = (centre_i.x, centre_i.y, height_i + spray_clearance)`. `spray_clearance` defaults to
  2.5 m: TreeMapper's worst observed low bias (1.9 m) plus margin. The spray point is raised to `height_k +
  max(transit_clearance, spray_clearance − descent_slack)` when that is higher, for every other tree k whose outline lies within `drone_radius +
  lateral_margin` of the target centre (a tree the mapper split in two), counted `spray_point_raised`.
- Obstacles: every published tree `k ∉ {from, to}` whose outline comes within `drone_radius + lateral_margin` of the
  xy segment. Each obstacle requires `z ≥ height_k + transit_clearance` over the part of the segment within that
  distance of its outline. The `from` and `to` trees require `z ≥ height + spray_clearance − descent_slack` outside
  their own spray points (prevents cutting through their crown edge).
- If the straight 3D segment `p_from → p_to` satisfies every requirement, the path is that segment.
- Otherwise the path is up-over-down: climb vertically at `p_from` to `z_req = max` of the violated requirements, fly
  level to above `p_to`, descend to `p_to`. Counted `glide_climbs_inserted`.
- The path is never routed through a published canopy. Unmapped obstacles are handled by the clearance guard (4.6).
- Re-planning: if the target's or an obstacle's published height changes by more than `replan_dz` (0.3 m) during
  GLIDE, the remaining path is re-planned from the current position.
- Execution: the node streams the carrot point `carrot_lead_s` (0.5 s) ahead along the path at 20 Hz with
  `max_speed_xy` (4 m/s) and `max_speed_z` (1.5 m/s), so the path shape is decided by the navigator, not by the
  backend's controller.
- Arrival: `xy_err <= xy_tol` (0.3 m), `|z_err| <= z_tol` (0.3 m), speed < `arrive_speed` (0.3 m/s).

### 4.6 Spraying and safety rules

- **Dose time**: `dose_mode: area` (default) → `clamp(outline_area · dose_s_per_m2, dose_min_s, dose_max_s)` with
  defaults 0.3 s/m², 2 s, 6 s; `dose_mode: fixed` → `dose_fixed_s` (4 s, = 0.3 L at 0.075 L/s).
- **Nozzle gate**: the nozzle is open only in DOSE and only while `xy_err <= xy_tol`; it is forced off on every state
  exit, in HOLD, and when the node shuts down (a final `false` is published).
- **Dose validity**: a dose counts only if the nozzle was open for at least `dose_valid_frac` (0.8) of the dose time.
  Otherwise one retry (counted `dose_retries`); a second failure marks the tree **skipped** (counted `trees_skipped`),
  `set_tree_sprayed` is not called, and the tree goes to the cleanup list once. A tree skipped twice stays unsprayed and
  is reported.
- **Never spray twice**: a tree with `sprayed = true` in the map is never a target.
- **Clearance guard (fail closed)**: from `/disparity_image_down_float` and the stereo `camera_info`
  (`Z = f · B / d`), take the 10th percentile of valid ranges in the central `guard_window` (20 % × 20 %) of the image,
  corrected for the drone's tilt, as the nadir canopy clearance. If it is below `min_clearance` (1.5 m) in GLIDE or
  DOSE: stop descending, climb at `max_speed_z` until clearance ≥ `min_clearance + guard_hysteresis` (0.5 m), close the
  nozzle (counted `guard_triggers`, logged on the first trigger and every 10th). If fewer than `guard_min_valid` (30 %)
  of the window's pixels are valid, or the disparity is older than `guard_max_age_s` (0.5 s): no descent below the
  current altitude until valid again (counted `guard_blind`).
- **Missing inputs**: no map yet, or map older than `map_stale_s` (2 s) → HOLD(map_stale). No pose in TF, or older than
  `pose_max_age_s` (0.2 s) → HOLD(no_pose). `set_tree_sprayed` answering `success=false` (ID unknown, e.g. merged
  away) → re-resolve once and retry on the new ID, else count `spray_mark_failed` and continue; no answer within 2 s →
  retry up to `service_retries` (3), then HOLD(service). Every HOLD is counted by reason.
- **Comparison strategy**: `strategy: field_first` scans all corridors (SCAN_OUT/SCAN_SETTLE for each, no LOCK in
  between), locks once, then sprays corridor by corridor with the same ordering and glide planner. It exists only to
  measure what corridor mode saves.

### 4.7 Configuration

`config/corridor_nav.cfg`, same reader rules as `cand_obj.cfg`: unknown key → fatal at start; out-of-range values
clamped and logged; non-finite values rejected; built-in defaults equal the shipped file (test). Parameters:

| Group | Keys (default) |
|---|---|
| strategy | `strategy` (corridor); the heading comes from the field file (`heading_rad`, else auto) |
| corridors | `z_scan` (15 m above home), `scan_speed` (4 m/s), `corridor_width` (0 = `w_max`), `corridor_width_min` (3 m), `corridor_length_min` (5 m), `w_margin` (1 m), `expected_canopy_top_max` (7 m above home), `expected_canopy_radius_max` (2.5 m), `target_height_margin` (2 m: a target published above home + `expected_canopy_top_max` + this is implausible, never visited, counted `target_height_implausible`) |
| settle / lock | `settle_s` (2), `settle_timeout_s` (8), `lock_hold_s` (0.5) |
| ordering | `order_budget_ms` (5) |
| glide | `spray_clearance` (2.5 m), `transit_clearance` (2.0 m), `descent_slack` (0.5 m), `drone_radius` (0.8 m), `lateral_margin` (0.5 m), `replan_dz` (0.3 m), `carrot_lead_s` (0.5), `max_speed_xy` (4), `max_speed_z` (1.5), `xy_tol` (0.3), `z_tol` (0.3), `arrive_speed` (0.3) |
| dose | `dose_mode` (area), `dose_s_per_m2` (0.3), `dose_min_s` (2), `dose_max_s` (6), `dose_fixed_s` (4), `dose_valid_frac` (0.8) |
| guard | `min_clearance` (1.5 m), `guard_hysteresis` (0.5 m), `guard_window` (0.2), `guard_min_valid` (0.3), `guard_max_age_s` (0.5) |
| inputs | `map_stale_s` (2), `hold_timeout_s` (20), `reresolve_radius` (1.5 m), `pose_max_age_s` (0.2), `service_retries` (3) |

## 5. Isaac side (`sim/isaac`, Isaac Sim 5.1)

### 5.1 Runtime

- Image: the existing `treehoppingnavigation-treehoppingsim:latest`, used only as an Isaac Sim 5.1 runtime, in a
  container of our own (`tm_isaac`) with its entrypoint overridden: `./python.sh /ws/sim/isaac/orchard_sim.py --spec
  <spec.json> --out <run dir>`. **Stream mode** (headless crashes Isaac at start-up on this machine,
  `results/isaac_5x5_tropical.md`). The running `wasp_sim` container is never touched.
- Tree assets: `Orange_Tree.usd`, `Lemon_Tree.usd` from the existing tree-generator asset folder, mounted read-only;
  the path is a spec field (`assets_dir`).
- `ROS_DOMAIN_ID` is set to a dedicated value (`tm_domain_id`, default 77) for Isaac and the bridge, so no other sim on
  the host can see or inject topics.
- Physics 60 Hz, cameras rendered at 10 Hz. The sim runs free; every Noetic node uses `/clock` (`use_sim_time`). The
  real-time factor is logged and reported.

### 5.2 Scene (`scene.py`, `truth.py`)

Scene spec (`sim/specs/*.json`):

```json
{
  "seed": 1001,
  "assets_dir": "/assets/trees",
  "field": {"polygon": [[-5, -5], [60, -5], [60, 50], [-5, 50]]},
  "rows": {"count": 8, "row_spacing": 6.0, "tree_spacing": 4.5, "heading_rad": 0.0,
           "origin": [0, 0], "jitter": 0.5, "missing_fraction": 0.08},
  "species": {"orange": 0.5, "lemon": 0.5},
  "scale_range": [0.8, 1.2],
  "terrain": {"slope_rad": 0.03, "slope_dir_rad": 0.7},
  "clutter": {"grass": true, "rocks": 30},
  "drone": {"start": [-3, -3, 0], "home": [-3, -3]},
  "nozzle": {"flow_lps": 0.075, "cone_half_angle_deg": 20, "rays": 64, "offset_body": [0, 0, -0.3]}
}
```

- Trees are placed on the sloped ground plane; each instance gets a random yaw and a scale from `scale_range`.
- Ground truth per instance, computed from the **placed mesh** (world-space points of the canopy mesh): `id`,
  `species`, `centre` (xy of the canopy bounds centre), `outline` (convex hull of the canopy points projected to xy),
  `radius` (equivalent-area radius), `top_z` (max world z of the canopy), `ground_z` (terrain z at the trunk). Written to
  `<run dir>/truth.json` and published latched on `/sim/truth` (`std_msgs/String`, the same JSON). Truth heights
  therefore vary per instance.
- Each canopy gets an invisible convex-hull collider on a dedicated collision group, hit only by the nozzle rays.

### 5.3 Drone (`drone.py`)

- An Xform moved every physics step by a kinematic controller tracking the latest setpoint (PoseStamped carrot):
  velocity limited by `v_max` (6 / 2 m/s), acceleration limited to `a_max` (2 m/s²), yaw rate limited to 60 °/s.
- Roll and pitch follow the commanded horizontal acceleration (`atan(a/g)`, first-order lag 0.15 s), so the cameras see
  realistic tilt in turns and stops.
- No setpoint for 1 s → hover in place (vehicle-side fail safe).
- Publishes TF `world → body` at 60 Hz.

### 5.4 Cameras (`cameras.py`)

- Two pinhole cameras on the body: the real half-resolution intrinsics and stereo baseline, already rectified (ideal
  pinholes), and the mount verified in `results/geometry_check.md` (lever arm 0.15, 0.08, 0 m, nadir), so
  `treemapper_sim.launch`'s `camera_mount` parameter is correct as is.
- Topics (rgb8 at 10 Hz): `/cam/bottom/left/half/image_raw`, `/cam/bottom/right/half/image_raw`, and
  `/cam/bottom/{left,right}/half/camera_info` (same names as the drone, so TreeMapper's yaml needs no change).

### 5.5 Nozzle deposition (`nozzle.py`)

- While the last `/corridor_nav/spray` is `true` and younger than 0.5 s, every physics step casts `rays` rays in a
  uniform cone from the nozzle (body offset, pointing down in the body frame, so tilt moves the cone).
- Each ray carries `flow_lps · dt / rays` litres to the first canopy collider it hits, else to `ground` (off-target).
- Publishes `/sim/deposition` (`std_msgs/String` JSON: per-tree litres, ground litres, total emitted, nozzle-open time)
  at 2 Hz, cumulative, and writes the final state to `<run dir>/deposition.json`.

## 6. Bridge (`sim/bridge`)

- Container `tm_sim_bridge`: Ubuntu 20.04, ROS Noetic + ROS 2 Foxy, `ros1_bridge` built from source, running
  `parameter_bridge` with `bridge_topics.yaml`:
  - Isaac → Noetic: both `image_raw`, both `camera_info`, `/tf`, `/tf_static`, `/clock`, `/sim/truth`, `/sim/deposition`.
  - Noetic → Isaac: `/corridor_nav/setpoint` (`geometry_msgs/PoseStamped`), `/corridor_nav/spray` (`std_msgs/Bool`).
    Only standard types cross the bridge.
- The bridge and the whole Noetic sim graph (roscore, perception, `treemapper_node`, `corridor_nav_node`, `rosbag`)
  run in one container `tm_sim_ros` built from `sim/bridge/Dockerfile`, with GPU access for the detector; `tm_isaac`
  and `tm_sim_ros` share the docker network `tm_sim_net`. `tm_dev` (no GPU) stays the build and test container.
- Risk: Humble (Isaac) ↔ Foxy (bridge) DDS interop. Milestone M0 tests it first (section 9). Fallback, decided in M0
  if needed: a small `rclpy` relay inside the bridge container that converts the listed topics to ROS 1 with `rospy`
  (both available on 20.04).

## 7. Perception nodes (`ros/sim_perception`, Noetic)

- `stereo_sgbm_node`: subscribes to both `image_raw` (exact-stamp sync), converts to grey, runs `cv2.StereoSGBM`
  (params in `~sgbm`: `num_disparities` 64, `block_size` 5, `uniqueness` 10, `speckle_window` 100, `speckle_range` 2,
  `p1`/`p2` from block size, `mode` SGBM_3WAY), publishes 32FC1 disparity in pixels (invalid = NaN) on
  `/disparity_image_down_float` with the **left image's stamp and frame**. Counts dropped frames by reason
  (`unsynced`, `size_mismatch`) on `~counters`.
- `detector_node`: subscribes to the left `image_raw`, runs the YOLOX ONNX model with onnxruntime-gpu (CUDA, RTX 3070;
  `~device: cpu` fallback), publishes `treemapper_msgs/Detection2DArray` on `/object_detection/detections` with the
  image stamp. Params: `~model_path`, `~input_size`, `~input_channels` (grey replicated to 3 if needed),
  `~score_threshold`, `~nms_threshold`, `~class_ids` (tree classes). The model file is the one shipped with the
  existing detector; its path is given at launch. Per-frame inference time on `~timing`.

## 8. Running and scoring

### 8.1 One command

`scripts/sim_mission.sh <spec.json> [--strategy corridor|field_first] [--runs N] [--out <dir>]`:

1. Writes the field file from the spec; creates `<out>/run_<i>/`.
2. Starts `tm_isaac` and `tm_sim_bridge`, waits for `/clock` and both `camera_info` on the Noetic side.
3. Starts the Noetic graph in `tm_dev`: `sim_perception.launch`, `treemapper_sim.launch`, `corridor_nav.launch`, and
   `rosbag record` of every Noetic topic except raw images (left image kept at 1 Hz for review).
4. Calls `/corridor_nav/start`; waits for LAND or `mission_timeout_s` (default 1800 s sim time, then abort).
5. Stops its own containers and processes only, then runs `mission_score.py` on the run directory.

Repeated `--runs` use the same seed; the spread between them is the noise floor (rendering, SGBM and GPU inference are
not bit-deterministic). Different seeds are different spec files.

### 8.2 Scorecard (`mission_score.py`)

Inputs: the bag, `truth.json`, `deposition.json`. Truth trees are matched to TreeMapper trees with the existing scorer's
matching.

| Group | Metrics |
|---|---|
| Efficiency | mission time; path length; time split scan / settle / transit / glide / dose / hold; seconds per sprayed tree; glide climbs inserted; litres emitted |
| Spray | truth trees sprayed (deposit ≥ 50 % of the intended dose); missed; double-sprayed (≥ 20 % of a dose received in two separate dose events); on-target fraction (litres on the target / litres emitted while it was the target); off-target litres; skipped trees |
| Safety (must be 0 / pass) | minimum clearance between drone and any truth canopy (`drone z − top_z` for trees whose truth outline lies within `drone_radius` of the drone xy) must stay > 0; nozzle open outside DOSE; sprayed tree targeted again |
| Guards | clearance-guard triggers, guard-blind time, holds by reason, re-resolved and dropped IDs, late trees |
| Map | the existing scorer on `/treemapper/map` (detection, geometry, stability, safety, timeliness) |
| Cost | navigator step time per tick (mean, p99); detector and SGBM time per frame; real-time factor |

It prints a markdown scorecard and writes JSON. With `--compare` it puts a `corridor` and a `field_first` run directory
side by side, with mean and worst case over runs.

## 9. Testing and milestones

### 9.1 Tests

- **Core (gtest, `nav/corridor_nav/tests`)**, one test that would fail if each rule broke:
  corridor width respects `w_max` and refuses when coverage is impossible; corridors tile the field and all
  enter on the same side; assignment is permanent and late trees go to cleanup; ordering starts at the exit and ends at the entry, 2-opt
  never worsens cost and respects its budget; glide never passes below `height_k + transit_clearance` over any
  obstacle (property test over random maps); re-plan on height change; state machine phase strings per table 4.4;
  settle happens in SCAN; nozzle never open outside DOSE or with `xy_err > xy_tol`; dose validity, retry and skip;
  sprayed trees never targeted; vanished ID re-resolution; clearance guard climbs and blocks descent when blind; every
  HOLD reason; config defaults equal `config/corridor_nav.cfg`; clamps logged; unknown key fatal.
- **Node (gtest)**: message conversions; field file parsing and rejection.
- **`nav_sim_lite` (rostest, no Isaac)**: a scripted pose integrator stands in for the vehicle and a canned `TreeMap`
  publisher with a fake `set_tree_sprayed` stands in for TreeMapper; one 2-corridor mission runs to LAND in CI.
- **Isaac smoke**: `sim/specs/3x3_smoke.json`, end to end, every safety metric 0 / pass.

### 9.2 Milestones (each ends green and committed)

| # | Milestone | Done when |
|---|---|---|
| M0 | Bridge spike | Isaac 5.1 → bridge → Noetic delivers both images, `camera_info`, tf and `/clock` with correct stamps; Noetic → Isaac setpoint moves a cube; throughput and RTF measured and written to `results/sim_bridge_spike.md`; fallback chosen if needed |
| M1 | Scene, drone, cameras, truth | a spec renders; `truth.json` matches the placed meshes (spot-checked); the drone flies a scripted square; the camera mount checks against `results/geometry_check.md` |
| M2 | Perception front end | SGBM disparity and detections reach `treemapper_node`; a scripted scan pass produces a map; the existing scorer runs on it |
| M3 | Navigator core | every test in 9.1 "Core" passes; config file and defaults agree |
| M4 | Node and closed loop | `nav_sim_lite` passes; the 3×3 smoke mission flies to LAND in Isaac with TreeMapper live |
| M5 | Nozzle and scoring | `mission_score.py` scorecard; `corridor` vs `field_first` on the 5×5 and 8×12 specs, 3 runs each, written to `results/corridor_nav_isaac.md` |

## 10. Risks and open items

| Risk | Consequence | Mitigation |
|---|---|---|
| Humble ↔ Foxy DDS interop through `ros1_bridge` | no data on the Noetic side | M0 first; `rclpy`/`rospy` relay fallback |
| Isaac + YOLOX on one 8 GB RTX 3070 | out of GPU memory | half-res render, detector `~device: cpu` fallback (slower, sim time absorbs it) |
| TreeMapper's low canopy-top bias | spray altitude too low | `spray_clearance` 2.5 m and the stereo clearance guard; scorecard reports minimum true clearance; TreeMapper fix is a follow-up |
| Corridor width limited by camera footprint | many narrow corridors on a high canopy | `w_max` is computed and reported; `z_scan` is a parameter |
| The detector was trained on real images | detections on renders differ from flight | it already ran on Isaac renders in the recorded missions; reported, not claimed as flight performance |
| Non-deterministic loop | single-run differences are noise | repeated runs per seed; noise floor reported before any claim |

Nothing in this design is verified on the Orin or on a real flight; the real-drone flight backend is not built.
