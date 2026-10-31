# Corridor Spray Navigation + Isaac-only Closed Loop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A `corridor_nav` navigator (scan out, spray back, terrain-following glides) flying on TreeMapper's map, and a
closed loop that runs the whole system in Isaac Sim alone and prints a mission scorecard.

**Architecture:** A ROS-free C++17 core (`nav/corridor_nav`, built on `treemapper_core`'s geometry and `ConfigReader`)
holds every decision as tested pure functions plus one `Navigator` class; a thin Noetic node wraps it. Isaac Sim 5.1
(ROS 2) renders the orchard, flies a kinematic drone and measures spray deposition; `ros1_bridge` carries standard
message types only into a Noetic graph running SGBM stereo, the YOLOX detector, `treemapper_node` and `corridor_nav_node`.

**Tech Stack:** C++17 / CMake 3.16 / gtest (core), ROS 1 Noetic roscpp + yaml-cpp (node), Python 3.8 + rospy +
OpenCV + onnxruntime-gpu (perception, scorer), Isaac Sim 5.1 Python 3.11 (sim), ROS 2 Foxy `ros1_bridge`, Docker.

**Spec:** `docs/superpowers/specs/2026-10-30-corridor-spray-nav-design.md`

## Global Constraints

- Core: C++17, standard library + `treemapper_core` only; compile with `-Wall -Wextra -Wpedantic`, no warnings.
- Core decision logic lives in pure functions with one gtest each; `Navigator` is deterministic given its inputs.
- One config file `config/corridor_nav.cfg`; in-struct defaults equal the file (test); unknown key → `std::invalid_argument`; non-finite → rejected; clamps only in `Config::sanitized()`, each logged with the word `clamped`.
- Every hold, drop, skip, retry, climb, guard trigger is a named counter (`std::map<std::string, std::uint64_t>`), published.
- TreeMapper is not modified. The navigator talks to it only through `/treemapper/map`, `/treemapper/set_tree_sprayed`, `/mission_state` (std_msgs/String), `/spray_target` (std_msgs/Int32).
- Mission-state strings are exactly: `IDLE`, `SCAN_APPROACH`, `SCAN`, `PLANNING`, `HOPPING`, `SPRAYING`, `RETURN`, `ERROR`.
- Nozzle open only in DOSE with `xy_err <= xy_tol`; forced off on every state exit, in HOLD, on shutdown.
- Everything builds and is tested inside `tm_dev` (`docker exec tm_dev bash -lc 'source /opt/ros/noetic/setup.bash && cd /ws && ...'`); build output under `/data/work`, never in the repo.
- Never touch containers not named `tm_*`; Isaac and the sim graph run in new containers `tm_isaac` and `tm_sim_ros` on the docker network `tm_sim_net`, `ROS_DOMAIN_ID=77`. Never stop, restart or exec into `wasp_sim`, `tree_hopping_sim`, `bridge`, `ros2_docker`, `mlflow`.
- Data under `/media/alex/waspsim/cloud_build_bundle/{isaac,real}` is read-only; sim runs write to `/data/work/sim_runs/`; keep `/data/work` under 40 GB.
- Commits end with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`; work on branch `spec/corridor-spray-nav` (or a branch cut from it).

## Spec amendments made with this plan

Decided while planning and written back into the spec in the same commit:
1. **Setpoint is `geometry_msgs/PoseStamped`**, not a custom message (orientation = yaw only). The carrot point
   already moves at the navigator's chosen speed, so speed caps do not need to cross the bridge, and the Isaac side
   needs no custom ROS 2 message (building one for Isaac's internal Python 3.11 rclpy is the riskiest part of the
   bridge). `NavStatus.msg` stays, ROS 1 only.
2. **`heading_rad` comes from the field file only** (removed from the cfg table). Added cfg keys `pose_max_age_s`
   (0.2) and `service_retries` (3), which the spec used as literals.
3. **Spray point raised over an overlapping neighbour**: `z = max(height_to + spray_clearance, height_k +
   transit_clearance for every other tree k whose outline is within drone_radius + lateral_margin of the target
   centre)`, counted `spray_point_raised`. Without it a split tree makes the final descent impossible to plan.
4. **Run containers**: the Noetic sim graph and the bridge share one container `tm_sim_ros` (GPU, for the detector)
   built from `sim/bridge/Dockerfile`; `tm_dev` has no GPU and stays the build/test container.
5. **`set_tree_sprayed` failures split in two**: `success=false` (ID unknown, e.g. merged away) → re-resolve once and
   retry on the new ID, else count `spray_mark_failed` and continue; no response within 2 s → retry up to
   `service_retries`, then HOLD(service).

## Review Focus

1. **Non-convex (L-shaped) field** → a strip crossing the notch yields two pieces; each piece must become its own corridor so no scan line leaves the field (Task 2 test `CorridorPlan.LShapedFieldSplitsStripIntoTwoCorridors`).
2. **A corridor with no trees** → no LOCK hang: SCAN_SETTLE times out or settles, LOCK finds nothing, go straight to the next TRANSIT (Task 6 test `Navigator.EmptyCorridorGoesStraightToNext`).
3. **TreeMapper dies mid-mission** (map stops arriving; it is launched without respawn) → HOLD(map_stale), nozzle off, RETURN after `hold_timeout_s` (Task 7 test `Navigator.MapStaleHoldsThenReturns`).
4. **Overlapping neighbour at the target** (TreeMapper split a tree in two) → spray point raised, final descent still clears the neighbour (Task 4 test `Glide.SprayPointRaisedOverOverlappingNeighbour`).
5. **`set_tree_sprayed` returns `success=false`** because the ID was merged away → re-resolve once, retry on the new ID, else count `spray_mark_failed` and continue (Task 7 test `Navigator.SprayMarkFailureReresolvesThenContinues`).

---

## File structure

```
config/corridor_nav.cfg                         all navigator tunables (= built-in defaults)
nav/corridor_nav/CMakeLists.txt                 standalone project; add_subdirectory(core); lib corridor_nav_core; test_nav
nav/corridor_nav/include/corridor_nav/
  config.hpp        Config, CORRIDOR_NAV_FIELDS, configFrom, sanitized, enums Strategy/DoseMode
  types.hpp         Point3, Field, CameraFov, Corridor, Counters alias
  corridors.hpp     minAreaRectHeading, maxCorridorWidth, planCorridors
  assignment.hpp    Assignment, updateAssignment, reresolveId
  ordering.hpp      Stop, orderTargets
  glide.hpp         Path3, distanceToPolygon, sprayPoint, planGlide, pathTime, carrot, needsReplan
  spray.hpp         doseTime, DoseTracker
  guard.hpp         DisparityView, StereoIntrinsics, Clearance, nadirClearance, GuardAction, guardDecision
  navigator.hpp     NavState, Inputs, Outputs, Status, Navigator
nav/corridor_nav/src/{config,corridors,assignment,ordering,glide,spray,guard,navigator}.cpp
nav/corridor_nav/tests/{test_config,test_corridors,test_assignment,test_ordering,test_glide,test_spray,test_guard,test_navigator}.cpp
nav/corridor_nav/tests/fake_world.hpp           kinematic point + scripted map, for Navigator closed-loop tests
ros/corridor_nav_msgs/                          NavStatus.msg (ROS 1)
ros/corridor_nav_ros/                           node, conversions, field file, launch, nav_sim_lite rostest
ros/sim_perception/                             stereo_sgbm_node.py, detector_node.py, yolox.py, sgbm.py, tests, launch
sim/isaac/                                      layout.py, truth.py, controller.py, nozzle_model.py (pure, py3.8-compatible)
                                                scene.py, drone.py, cameras.py, nozzle.py, orchard_sim.py (Isaac wiring)
sim/tests/                                      pytest for the pure sim modules
sim/specs/{camera.json,3x3_smoke.json,5x5_tropical.json,rows_8x12.json}
sim/bridge/{Dockerfile,bridge_topics.yaml,spike_check.py}
tools/treemapper_tools/mission_score.py, tools/tests/test_mission_score.py
scripts/sim_mission.sh, scripts/run_tests.sh (modified)
results/sim_bridge_spike.md, results/corridor_nav_isaac.md
```

---

## Phase A — navigator core (no ROS, no Isaac)

### Task 1: Core skeleton and configuration

**Files:**
- Create: `nav/corridor_nav/CMakeLists.txt`, `nav/corridor_nav/include/corridor_nav/{config.hpp,types.hpp}`, `nav/corridor_nav/src/config.cpp`, `nav/corridor_nav/tests/test_config.cpp`, `config/corridor_nav.cfg`
- Modify: `scripts/run_tests.sh` (add a nav configure/build/ctest block after the core block, build dir `$BUILD/nav`)

**Interfaces:**
- Consumes: `treemapper::ConfigReader`, `treemapper::MapperConfig` (`core/include/treemapper/mapper.hpp`); `treemapper::Point2`, `Polygon2`, `TreeEntry`, `TreeId`, `kNoTree`, `DronePose` (`types.hpp`); `convexHull`, `polygonArea`, `pointInPolygon`, `convexIntersectionArea` (`geometry.hpp`).
- Produces (namespace `corridor_nav`):
  - `enum class Strategy { kCorridor, kFieldFirst }; enum class DoseMode { kArea, kFixed };`
  - `struct Config` with the fields below, `Config sanitized(const std::function<void(const std::string&)>& log) const;`
  - `Config configFrom(const treemapper::MapperConfig&);` `constexpr int kConfigKeyCount = 41;`
  - X-macro `CORRIDOR_NAV_FIELDS(X)` over every numeric field (enums are parsed from strings and checked separately).
  - `struct Point3 { double x{0}, y{0}, z{0}; };`
  - `struct Field { treemapper::Polygon2 polygon; std::optional<double> heading_rad; treemapper::Point2 home; };`
  - `struct CameraFov { double hfov_rad{0}, vfov_rad{0}; };`
  - `struct Corridor { int index; treemapper::Polygon2 strip; treemapper::Point2 entry, exit; double heading_rad; };`
  - `using Counters = std::map<std::string, std::uint64_t>;`

Config fields and defaults (cfg key = field name; units m, s, m/s):
`strategy`=corridor, `z_scan`=15, `scan_speed`=4, `corridor_width`=0, `corridor_width_min`=3, `corridor_length_min`=5,
`w_margin`=1, `expected_canopy_top_max`=7, `expected_canopy_radius_max`=2.5, `settle_s`=2, `settle_timeout_s`=8,
`lock_hold_s`=0.5, `order_budget_ms`=5, `spray_clearance`=2.5, `transit_clearance`=2.0, `descent_slack`=0.5,
`drone_radius`=0.8, `lateral_margin`=0.5, `replan_dz`=0.3, `carrot_lead_s`=0.5, `max_speed_xy`=4, `max_speed_z`=1.5,
`xy_tol`=0.3, `z_tol`=0.3, `arrive_speed`=0.3, `dose_mode`=area, `dose_s_per_m2`=0.3, `dose_min_s`=2, `dose_max_s`=6,
`dose_fixed_s`=4, `dose_valid_frac`=0.8, `min_clearance`=1.5, `guard_hysteresis`=0.5, `guard_window`=0.2,
`guard_min_valid`=0.3, `guard_max_age_s`=0.5, `map_stale_s`=2, `hold_timeout_s`=20, `reresolve_radius`=1.5,
`pose_max_age_s`=0.2, `service_retries`=3. (41 keys.)

Clamp ranges in `sanitized()`: every length/time/speed `> 0` except `corridor_width >= 0`; `dose_min_s <= dose_max_s`
(raise max); `dose_valid_frac`, `guard_window`, `guard_min_valid` in `(0, 1]`; `service_retries` in `[0, 10]`;
`spray_clearance >= min_clearance` (raise spray_clearance).

- [ ] **Step 1: Write the failing tests** in `test_config.cpp` (copy `repoRoot()`/`readCfgFile()` helpers from `core/tests/candidates/cand_obj/test_cand_obj.cpp:111-130`, adjusting the `__FILE__` depth to 4):
  - `NavConfig.ShippedFileEqualsBuiltInDefaults`: field count via `CORRIDOR_NAV_FIELDS` + 2 enums == `kConfigKeyCount`; file has 41 keys; every field equals the default; `strategy` and `dose_mode` parse to `kCorridor` / `kArea`; `configFrom({{"not_a_key","1"}})` and `configFrom({{"z_scan","nan"}})` throw `std::invalid_argument`; `configFrom({{"strategy","zigzag"}})` throws.
  - `NavConfig.SanitizedClampsAndLogsEveryClamp`: `xy_tol=0`, `dose_min_s=7` (> max 6), `spray_clearance=1.0` (< min_clearance 1.5) → three log lines containing `clamped`; results `xy_tol > 0`, `dose_max_s >= 7`, `spray_clearance == 1.5`.
- [ ] **Step 2: Run to verify they fail**: `docker exec tm_dev bash -lc 'cd /ws && ./scripts/run_tests.sh'` → nav build fails (no sources).
- [ ] **Step 3: Implement** `CMakeLists.txt` (project `corridor_nav`, `add_subdirectory(../../core ${CMAKE_BINARY_DIR}/treemapper_core)` with `TREEMAPPER_BUILD_TESTS OFF`, `add_library(corridor_nav_core ...)` linking `treemapper_core` PUBLIC, `test_nav` from `tests/*.cpp` with the same googletest fallback as `core/CMakeLists.txt:76-83`), `config.hpp/.cpp`, `config/corridor_nav.cfg` (header comment in the style of `config/cand_obj.cfg`, grouped as in spec 4.7).
- [ ] **Step 4: Run** `./scripts/run_tests.sh` → `ALL TESTS PASSED`.
- [ ] **Step 5: Commit** `feat(nav): corridor_nav core skeleton and config`.

### Task 2: Corridor planner

**Files:** Create `include/corridor_nav/corridors.hpp`, `src/corridors.cpp`, `tests/test_corridors.cpp`.

**Interfaces:**
- Consumes: Task 1 types and `Config`.
- Produces:
  - `double minAreaRectHeading(const treemapper::Polygon2& field);` — radians in `[0, π)`, direction of the long side of the minimum-area bounding rectangle (rotating calipers over the convex hull's edges).
  - `double maxCorridorWidth(const CameraFov&, const Config&);` — `2·(z_scan − expected_canopy_top_max)·tan(min(hfov,vfov)/2) − 2·expected_canopy_radius_max − w_margin`.
  - `struct CorridorPlan { std::vector<Corridor> corridors; double width{0}; double w_max{0}; bool ok{false}; std::string error; };`
  - `CorridorPlan planCorridors(const Field&, const CameraFov&, const Config&, Counters*);`

Algorithm: rotate the field into the heading frame (u along heading, v across); `w = corridor_width == 0 ? w_max : min(corridor_width, w_max)` (count `corridor_width_clamped` when clamped); `ok=false, error="coverage impossible"` when `w_max < corridor_width_min`; `ok=false, error="field invalid"` when the polygon has < 3 vertices or zero area. Cut v-range into `n = ceil(extent_v / w)` equal strips; for each strip, intersect with the field along the strip's centre line v = v_c: the u-intervals where the centre line is inside the polygon (even-odd crossings) — each interval longer than `corridor_length_min` is one corridor (else count `corridors_dropped_short`); `strip` = rectangle `[u0,u1]×[v_c−w/2, v_c+w/2]` rotated back. Number corridors in v order then u order; every corridor enters at u0 and exits at u1, `heading_rad = heading` (ruling 2026-10-31: same-direction corridors, so corridor k's spray-back end is adjacent to k+1's entry). A non-finite `w_max` fails closed like `w_max < corridor_width_min`.

- [ ] **Step 1: Write the failing tests:**
  - `CorridorPlan.MaxWidthMatchesFormula`: fov 70° both axes, defaults → `maxCorridorWidth` ≈ 5.2 (`EXPECT_NEAR(.., 2*8*tan(35°) - 5 - 1, 1e-9)`).
  - `CorridorPlan.HeadingIsLongAxis`: 60×20 rectangle rotated 30° → `minAreaRectHeading` ≈ 30° (mod π).
  - `CorridorPlan.TilesFieldAndAlternates`: 60×20 rectangle, heading 0, `corridor_width=5` (≤ w_max with fov 90°) → 4 corridors, each 5 m wide, union covers the field (sum of strip∩field areas ≈ 1200 within 1e-6); all corridors enter on the same side: `|entry_k − entry_{k−1}|` ≈ 5 and every `heading_rad` equals the field heading.
  - `CorridorPlan.ClampsWidthAndCounts`: `corridor_width=50` → `width == w_max`, counter `corridor_width_clamped == 1`.
  - `CorridorPlan.RefusesWhenCoverageImpossible`: `z_scan=8`, fov 40° → `ok == false`, error `coverage impossible`.
  - `CorridorPlan.LShapedFieldSplitsStripIntoTwoCorridors` (Review Focus 1): L-shape (0,0)(40,0)(40,10)(10,10)(10,30)(0,30), heading 90° (u along y), `corridor_width=5` → for a strip whose centre line x ∈ (10, 40) the corridor's u-range stays inside y ∈ [0,10]; no corridor's entry–exit segment midpoint lies outside the polygon.
- [ ] **Step 2: Run** → FAIL (undefined `planCorridors`).
- [ ] **Step 3: Implement** in `corridors.cpp`.
- [ ] **Step 4: Run** `./scripts/run_tests.sh` → PASS.
- [ ] **Step 5: Commit** `feat(nav): corridor planner`.

### Task 3: Tree assignment and spray ordering

**Files:** Create `assignment.hpp/.cpp`, `ordering.hpp/.cpp`, `tests/test_assignment.cpp`, `tests/test_ordering.cpp`.

**Interfaces:**
- Consumes: `Corridor`, `TreeEntry`, `pointInPolygon`.
- Produces:
  - `struct Assignment { std::map<treemapper::TreeId, int> corridor_of; std::vector<treemapper::TreeId> cleanup; std::set<treemapper::TreeId> outside_field; };`
  - `std::vector<treemapper::TreeId> updateAssignment(const std::vector<treemapper::TreeEntry>& map, const std::vector<Corridor>&, const treemapper::Polygon2& field, int locked_upto, bool lock_now, Assignment*, Counters*);` — corridors `0..locked_upto` are locked (`-1` = none). With `lock_now` (the LOCK event of corridor `locked_upto`): every unassigned, unsprayed tree whose corridor is `locked_upto` is assigned to it and returned. Every other unassigned, unsprayed tree whose corridor is `<= locked_upto` (or `< locked_upto` during the lock event) goes to `cleanup` once (count `late_trees`). Centres outside `field` go to `outside_field` once (count `trees_outside_field`). The navigator calls it with `lock_now=true` at each LOCK and with `lock_now=false` every other tick (ruling 2026-10-31: the old `just_locked` form could not route late trees in the latest-locked corridor).
  - `treemapper::TreeId reresolveId(const treemapper::Point2& old_centre, const std::vector<treemapper::TreeEntry>& map, double radius);` — tree whose outline contains the point, else nearest centre within `radius`, else `kNoTree`; sprayed trees excluded.
  - `struct Stop { treemapper::TreeId id; Point3 p; };`
  - `using CostFn = std::function<double(const Point3&, const Point3&)>;`
  - `std::vector<Stop> orderTargets(std::vector<Stop> stops, const Point3& start, const Point3& end, double heading_rad, const CostFn& cost, double budget_ms, const std::function<double()>& now_ms);` — initial order by axial coordinate along `heading_rad` descending, then 2-opt on the open path start→stops→end until no improving reversal or `now_ms() − t0 > budget_ms`.

- [ ] **Step 1: Write the failing tests:**
  - `Assignment.AssignsOnLockAndIsPermanent`: two corridors, trees at centres in each; lock 0 → returns corridor-0 IDs; moving a tree's centre into corridor 1 later does not change `corridor_of`.
  - `Assignment.LateTreeGoesToCleanup`: after lock 0 and lock 1, a new tree in corridor 0 → in `cleanup`, `late_trees == 1`.
  - `Assignment.SprayedAndOutsideFieldNeverAssigned`.
  - `Assignment.ReresolvePrefersContainingOutlineThenNearest`: point inside tree 7's outline → 7; point 1.0 m from tree 9's centre, inside nothing → 9; 2.0 m away with radius 1.5 → `kNoTree`; sprayed tree containing the point → skipped.
  - `Ordering.StartsAtExitEndsAtEntry`: 6 trees on a line u = 0..25, heading 0, start at u=30, end at u=−5, Euclidean cost → ids in descending u.
  - `Ordering.TwoOptFixesStaggeredRows`: two rows (v = ±1.5) staggered by 2 m; result cost ≤ initial axial-sort cost and ≤ the cost of a deliberately bad order; exact optimum for a 4-stop case equals brute force.
  - `Ordering.RespectsBudget`: `now_ms` stub that advances 10 ms per call, budget 5 → returns after at most 2 improvement passes (count calls); output is a permutation of the input.
- [ ] **Step 2: Run** → FAIL.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** → PASS.
- [ ] **Step 5: Commit** `feat(nav): tree assignment, ID re-resolution, spray ordering`.

### Task 4: Glide planner

**Files:** Create `glide.hpp`, `src/glide.cpp`, `tests/test_glide.cpp`.

**Interfaces:**
- Consumes: `Point3`, `TreeEntry`, `Config`.
- Produces:
  - `struct Path3 { std::vector<Point3> pts; bool climbed{false}; };`
  - `double distanceToPolygon(const treemapper::Polygon2&, const treemapper::Point2&);` — 0 inside.
  - `Point3 sprayPoint(const treemapper::TreeEntry& target, const std::vector<treemapper::TreeEntry>& map, const Config&, Counters*);` — spec amendment 3.
  - `Path3 planGlide(const Point3& from, treemapper::TreeId from_id, const treemapper::TreeEntry& to, const std::vector<treemapper::TreeEntry>& map, const Config&, Counters*);`
  - `double pathTime(const Path3&, const Config&);` — per segment `max(dxy / max_speed_xy, |dz| / max_speed_z)`.
  - `Point3 carrot(const Path3&, const Point3& pos, const Config&);` — project `pos` onto the path, advance `carrot_lead_s` of travel time along it, clamp at the end.
  - `bool needsReplan(const std::map<treemapper::TreeId, double>& planned_heights, const std::vector<treemapper::TreeEntry>& map, const Config&);` — any height moved by more than `replan_dz`.

Requirement check (the part the signature does not fix): sample the xy segment every 0.1 m; at each sample, for every tree k ≠ to.id with `distanceToPolygon(k.outline, xy) <= drone_radius + lateral_margin` the requirement is `height_k + transit_clearance`; for `to` and `from_id` it is `height + spray_clearance − descent_slack`. Straight 3D segment if `z(s) >= requirement` everywhere; otherwise `[from, (from.xy, z_up), (to.xy, z_up), p_to]` with `z_up = max(max violated requirement, from.z, p_to.z)`, dropping zero-length legs; count `glide_climbs_inserted`.

- [ ] **Step 1: Write the failing tests:**
  - `Glide.StraightWhenGapIsClear`: trees A (h=5) and B (h=5.5) 5 m apart, no other trees → 2-point path from `sprayPoint(A)` z=7.5 to z=8.0, `climbed == false`.
  - `Glide.ClimbsOverTallerMiddleTree`: A (h=4), C (h=7) in the middle, B (h=4) 10 m away → path climbs to ≥ 9.0 (7 + transit 2) before passing C's outline; `glide_climbs_inserted == 1`.
  - `Glide.NeverBelowAnyObstacleRequirement` (property test): 200 random maps (seeded `std::mt19937(42)`), 5–15 trees, heights 3–8 m → for every planned path, every 0.05 m sample satisfies the requirement rule above.
  - `Glide.SprayPointRaisedOverOverlappingNeighbour` (Review Focus 4): target T (h=4) and neighbour N (h=6) whose outline contains T's centre → `sprayPoint(T).z == 8.0` (6 + 2), counter `spray_point_raised == 1`; the planned glide's last point is that point.
  - `Glide.PathTimeAndCarrot`: 8 m level + 3 m down segment → `pathTime == 8/4 + 3/1.5`; carrot from the start advances 0.5 s of travel (2 m level).
  - `Glide.ReplanWhenHeightMoves`: planned {7: 5.0}, map height 5.4 → true; 5.2 → false.
- [ ] **Step 2: Run** → FAIL.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** → PASS.
- [ ] **Step 5: Commit** `feat(nav): terrain-following glide planner`.

### Task 5: Dose, nozzle gate, clearance guard

**Files:** Create `spray.hpp/.cpp`, `guard.hpp/.cpp`, `tests/test_spray.cpp`, `tests/test_guard.cpp`.

**Interfaces:**
- Produces:
  - `double doseTime(double outline_area_m2, const Config&);` — area mode `clamp(area·dose_s_per_m2, dose_min_s, dose_max_s)`; fixed mode `dose_fixed_s`.
  - `class DoseTracker { public: void start(double t, double dose_s); void update(double t, bool nozzle_open); bool done(double t) const; bool valid() const; double openFraction() const; };` — open time integrated between successive `update` calls using the previous `nozzle_open` value; `valid()` = `openFraction() >= dose_valid_frac` (fraction stored at construction: `explicit DoseTracker(double valid_frac)`).
  - `struct DisparityView { int width, height; const float* data; double stamp; };` (row-major, px, NaN/≤0 invalid)
  - `struct StereoIntrinsics { double fx, baseline; };`
  - `struct Clearance { bool valid{false}; double metres{NAN}; double stamp{0}; };`
  - `Clearance nadirClearance(const DisparityView&, const StereoIntrinsics&, double roll, double pitch, const Config&);` — central `guard_window` fraction of width and height; `Z = fx·B/d`; `valid` iff valid pixels ≥ `guard_min_valid` of the window; `metres` = 10th percentile of Z times `cos(roll)·cos(pitch)`.
  - `enum class GuardAction { kNone, kClimb, kNoDescent };`
  - `GuardAction guardDecision(const Clearance&, double now, bool climbing, const Config&);` — invalid or `now − stamp > guard_max_age_s` → `kNoDescent`; `metres < min_clearance` → `kClimb`; while `climbing` keep `kClimb` until `metres >= min_clearance + guard_hysteresis`; else `kNone`.

- [ ] **Step 1: Write the failing tests:**
  - `Spray.DoseTimeAreaAndFixed`: area 4 m² → 2.0 (min); 12 m² → 3.6; 40 m² → 6 (max); fixed → 4.
  - `Spray.DoseValidOnlyAboveFraction`: 4 s dose, open 3.4 s (85 %) → valid; open 3.0 s (75 %) → invalid; `done` at t0+4.
  - `Guard.ClearanceFromDisparity`: 100×100 view, centre window filled with d giving Z=3 m (fx=500, B=0.12 → d=20), level → 3.0; pitch 0.3 rad → 3·cos(0.3).
  - `Guard.PercentileIgnoresFewNearOutliers`: 5 % of window pixels at Z=0.5 m, rest at 3 m → ≈ 3.0.
  - `Guard.InvalidWhenTooFewPixels`: 25 % valid → `valid == false`.
  - `Guard.ClimbHysteresisAndBlind`: 1.2 m → `kClimb`; 1.8 m while climbing → `kClimb`; 2.1 m → `kNone`; stale (age 0.6 s) → `kNoDescent`.
- [ ] **Step 2: Run** → FAIL.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** → PASS.
- [ ] **Step 5: Commit** `feat(nav): dose timing, dose validity, stereo clearance guard`.

### Task 6: Navigator state machine — nominal mission

**Files:** Create `navigator.hpp`, `src/navigator.cpp`, `tests/fake_world.hpp`, `tests/test_navigator.cpp`.

**Interfaces:**
- Consumes: everything from Tasks 1–5.
- Produces:
  - `enum class NavState { kIdle, kTakeoff, kTransit, kScanOut, kScanSettle, kLock, kGlide, kDose, kReturn, kLand, kHold, kAbort, kDone };` `const char* toString(NavState);`
  - `struct MapSnapshot { double stamp; std::vector<treemapper::TreeEntry> trees; };`
  - `struct SprayResult { treemapper::TreeId id; bool ok; };`
  - `struct Inputs { double t; treemapper::DronePose pose; /* pose.t = its stamp, pose.valid */ std::optional<MapSnapshot> map; Clearance clearance; bool start_requested{false}; bool abort_requested{false}; std::optional<SprayResult> spray_result; };`
  - `struct Outputs { Point3 setpoint; double yaw; bool nozzle{false}; std::string phase; treemapper::TreeId target{treemapper::kNoTree}; std::optional<treemapper::TreeId> request_sprayed; };`
  - `struct Status { NavState state; bool cleanup; int corridor; int corridors_total; treemapper::TreeId target; unsigned sprayed, skipped, remaining; double clearance_m; std::vector<std::string> warnings; };`
  - `class Navigator { public: Navigator(Config, Field, CameraFov, std::function<void(const std::string&)> log = {}); Outputs step(const Inputs&); NavState state() const; Status status() const; const Counters& counters() const; const std::vector<Corridor>& corridors() const; std::vector<Path3> plannedPaths() const; };`
  - Phase strings per spec table 4.4; status string `CLEANUP` when gliding/dosing cleanup trees.

Behaviour fixed by this task: IDLE → (start, valid pose) plan corridors (on `!ok` → ABORT) → TAKEOFF to `home_z + z_scan` → TRANSIT → SCAN_OUT along the centre line at `scan_speed` → SCAN_SETTLE (settled when no new tree with centre in the corridor for `settle_s`, or `settle_timeout_s`, count `settle_timeouts`) → LOCK (`updateAssignment`, `orderTargets` with cost `pathTime(planGlide(...))`, hold `lock_hold_s`) → GLIDE/DOSE loop → after a valid dose emit `request_sprayed`; on `spray_result.ok` advance → next corridor TRANSIT, or CLEANUP when corridors are exhausted, then RETURN → LAND → DONE. `strategy=field_first`: SCAN_OUT/SCAN_SETTLE for every corridor (odd corridors scanned exit → entry, a boustrophedon scan), one LOCK over all, then spray corridor by corridor. Home z = pose z at start.

`fake_world.hpp`: `struct FakeWorld` with a point drone moving toward the setpoint at `max_speed_xy`/`max_speed_z`, a list of truth trees, a map that publishes a truth tree (as `TreeEntry`, given height, id = index+1) once the drone has been in SCAN within 6 m xy of it for 3 ticks, a `set_tree_sprayed` that marks it and returns ok, a clearance computed as `drone z − max truth height under the drone` (or 15 m over ground); `tick(Navigator&, double dt)`; records phase strings, nozzle samples, min clearance.

- [ ] **Step 1: Write the failing tests:**
  - `Navigator.FullCorridorMissionSpraysEveryTreeOnce`: 3 rows × 4 trees, 6 m spacing, field 30×24, fov 90°, dt 0.05 → reaches `kDone` within 2000 s sim; every tree sprayed exactly once; `trees_skipped == 0`; min clearance in the fake world `>= min_clearance`.
  - `Navigator.PhaseSequencePerCorridor`: recorded phase string sequence, collapsed, equals `IDLE, SCAN_APPROACH, SCAN, PLANNING, HOPPING, SPRAYING, HOPPING, ...` and contains no `SCAN` after a `PLANNING` without an intervening `SCAN_APPROACH`; `/spray_target` is −1 whenever phase ∉ {HOPPING, SPRAYING}.
  - `Navigator.NozzleOnlyInDoseWithinTolerance`: across the whole mission, every sample with `nozzle == true` has state `kDose` and xy error ≤ `xy_tol`.
  - `Navigator.SettleHappensInScan`: a tree that the fake map publishes only during the end-of-corridor hover is assigned to that corridor (not cleanup).
  - `Navigator.EmptyCorridorGoesStraightToNext` (Review Focus 2): field with an empty middle corridor → that corridor goes SCAN_SETTLE → LOCK → TRANSIT with no GLIDE; mission completes.
  - `Navigator.SprayedTreeNeverTargeted`: a tree pre-marked sprayed in the map is never `target`.
  - `Navigator.FieldFirstScansAllThenSprays`: `strategy=field_first` → all `SCAN` phases precede the first `PLANNING`; all trees sprayed once.
- [ ] **Step 2: Run** → FAIL.
- [ ] **Step 3: Implement** `Navigator` and `fake_world.hpp`.
- [ ] **Step 4: Run** → PASS.
- [ ] **Step 5: Commit** `feat(nav): navigator state machine, nominal corridor and field_first missions`.

### Task 7: Navigator — holds, guards, retries, re-resolution, cleanup

**Files:** Modify `src/navigator.cpp`; extend `tests/test_navigator.cpp`, `tests/fake_world.hpp` (fault injection switches).

**Interfaces:** unchanged from Task 6; adds counters `hold_map_stale`, `hold_no_pose`, `hold_service`, `hold_timeouts`, `guard_triggers`, `guard_blind`, `dose_retries`, `trees_skipped`, `ids_reresolved`, `ids_dropped_vanished`, `spray_mark_failed`, `glide_replans`.

- [ ] **Step 1: Write the failing tests:**
  - `Navigator.MapStaleHoldsThenReturns` (Review Focus 3): stop map updates mid-GLIDE → HOLD within `map_stale_s` + one tick, nozzle off, setpoint = hold position; after `hold_timeout_s` → RETURN, phase `RETURN`; `hold_timeouts == 1`.
  - `Navigator.PoseLossHolds`: `pose.t` older than `pose_max_age_s` (or `pose.valid == false`) → HOLD(no_pose); fresh again → resumes the previous state.
  - `Navigator.GuardClimbsAndBlocksDescent`: fake clearance forced to 1.0 m during GLIDE → setpoint z rises at `max_speed_z`, nozzle off, `guard_triggers >= 1`; clearance invalid → setpoint z never below current z.
  - `Navigator.DoseRetryThenSkip`: fake world pushes xy error to 0.5 m for the whole dose → one retry, then `trees_skipped == 1`, no `request_sprayed` for it, tree in cleanup once; skipped twice → stays unsprayed, mission completes.
  - `Navigator.VanishedIdIsReresolved`: before GLIDE to tree 5, the map replaces 5 with 9 (same outline) → target becomes 9, `ids_reresolved == 1`; replaced by nothing → `ids_dropped_vanished == 1`, next target.
  - `Navigator.SprayMarkFailureReresolvesThenContinues` (Review Focus 5): `set_tree_sprayed` returns `ok=false` → one re-resolve and one retry on the new ID; if that also fails → `spray_mark_failed == 1`, mission continues to the next tree; `service_retries` exhausted on a transport failure (no result for 2 s, treated as failure) → HOLD(service).
  - `Navigator.ReplanOnHeightChange`: target height raised 0.5 m mid-GLIDE → `glide_replans == 1`, final setpoint z = new height + clearance.
  - `Navigator.LateTreeSprayedInCleanup`: fake map publishes a new tree in corridor 0 while corridor 1 scans → sprayed during CLEANUP, status `cleanup == true` then.
  - `Navigator.AbortClosesNozzleAndReturns`: `abort_requested` during DOSE → nozzle false that tick, phase `ERROR` then RETURN.
- [ ] **Step 2: Run** → FAIL.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `./scripts/run_tests.sh` → PASS, no new warnings.
- [ ] **Step 5: Commit** `feat(nav): holds, clearance guard, dose retries, ID re-resolution, cleanup`.

---

## Phase B — ROS node

### Task 8: `corridor_nav_msgs` and `corridor_nav_ros` node

**Files:**
- Create: `ros/corridor_nav_msgs/{CMakeLists.txt,package.xml,msg/NavStatus.msg}` (fields exactly as spec 3.3)
- Create: `ros/corridor_nav_ros/{CMakeLists.txt,package.xml}`, `include/corridor_nav_ros/{conversions.hpp,field_file.hpp}`, `src/{conversions.cpp,field_file.cpp,corridor_nav_node.cpp}`, `launch/corridor_nav.launch`, `config/corridor_nav.yaml`, `test/{test_conversions.cpp,test_field_file.cpp}`
- Modify: `README.md` (catkin workspace section: also symlink `corridor_nav_msgs`, `corridor_nav_ros`)

**Interfaces:**
- Consumes: `corridor_nav::Navigator` and friends; `treemapper_msgs/TreeMap`, `SetTreeSprayed`.
- Produces:
  - `std::vector<treemapper::TreeEntry> fromMsg(const treemapper_msgs::TreeMap&);`
  - `corridor_nav_msgs::NavStatus toMsg(const corridor_nav::Status&, const corridor_nav::Counters&, const ros::Time&);`
  - `geometry_msgs::PoseStamped toSetpoint(const corridor_nav::Outputs&, const std::string& frame, const ros::Time&);` (orientation = yaw about z)
  - `corridor_nav::Field loadFieldFile(const std::string& path);` — YAML `polygon` (≥ 3 `[x, y]`), optional `heading_rad`, `home`; throws `std::invalid_argument` naming the problem.
  - Node `corridor_nav_node` params: `~config_file` (default `config/core/corridor_nav.cfg` via the same `config/core` symlink pattern as treemapper_ros), `~field_file` (required), `~world_frame` (world), `~body_frame` (body), `~map_topic` (/treemapper/map), `~set_sprayed_service` (/treemapper/set_tree_sprayed), `~disparity_topic` (/disparity_image_down_float), `~left_info_topic`/`~right_info_topic` (half camera_info), `~rate` (20).
  - Publishes `/mission_state`, `/spray_target`, `/corridor_nav/setpoint` (PoseStamped), `/corridor_nav/spray` (Bool, every tick and on change, `false` on shutdown), `~status` (2 Hz), `~plan_markers`, `~timing` (`treemapper_msgs/StageTiming`, stage `step`, ms of `Navigator::step` per tick); services `~start`, `~abort` (std_srvs/Trigger).
  - `set_tree_sprayed` calls run in a `std::async` worker; the finished future becomes `Inputs::spray_result`; a call pending > 2 s is a failure.
  - `CameraFov` from left `camera_info` (`hfov = 2·atan(width / (2·K[0]))`, vfov with `K[4]`); `StereoIntrinsics.baseline = −P_right[3] / P_right[0]`.

- [ ] **Step 1: Write the failing tests:** `Conversions.TreeMapRoundTrip` (outline, centre, height, sprayed preserved), `Conversions.SetpointYaw` (yaw 1.0 → quaternion z = sin 0.5, w = cos 0.5), `Conversions.StatusCarriesCounters` (reasons/counts same length, sorted by name), `FieldFile.ParsesAndRejects` (valid file → 4 vertices, heading set; 2 vertices → throws with "polygon"; missing file → throws).
- [ ] **Step 2: Build and run** in a scratch workspace `/data/work/catkin_nav` (symlink `treemapper_msgs`, `treemapper_ros`, `corridor_nav_msgs`, `corridor_nav_ros`): `catkin build corridor_nav_ros -j2 --make-args tests && catkin run_tests corridor_nav_ros` → FAIL.
- [ ] **Step 3: Implement** conversions, field file, node, launch (`corridor_nav.launch` args `field_file`, `config_file`, `use_sim_time`=true, `output`; `required="false"`, no respawn).
- [ ] **Step 4: Run** → PASS; `catkin_test_results` shows 0 failures.
- [ ] **Step 5: Commit** `feat(nav): corridor_nav ROS node and NavStatus message`.

### Task 9: `nav_sim_lite` rostest (closed loop without Isaac)

**Files:** Create `ros/corridor_nav_ros/test/{nav_sim_lite.py,nav_sim_lite.test,check_mission.py}`, `ros/corridor_nav_ros/test/field_lite.yaml`.

**Interfaces:**
- Consumes: the node from Task 8.
- `nav_sim_lite.py`: publishes TF `world→body` at 50 Hz moving toward `/corridor_nav/setpoint` at 4 / 1.5 m/s; publishes a latched `/treemapper/map` at 5 Hz from 8 scripted trees (2 corridors), each appearing once the pose has passed within 6 m during `SCAN`; serves `/treemapper/set_tree_sprayed`; publishes a constant-range synthetic disparity (Z = drone z − 4 m) and matching camera_info (640×400, fx=fy=400, baseline 0.12).
- `check_mission.py`: calls `~start`, waits for `/mission_state == RETURN` then landing (time limit 600 s sim at rate ×5 via `/clock` published by `nav_sim_lite.py`), asserts all 8 trees sprayed once, nozzle never true outside `SPRAYING`.

- [ ] **Step 1: Write** `check_mission.py` assertions and the `.test` file.
- [ ] **Step 2: Run** `rostest corridor_nav_ros nav_sim_lite.test` → FAIL (no sim script).
- [ ] **Step 3: Implement** `nav_sim_lite.py`.
- [ ] **Step 4: Run** → PASS within the time limit.
- [ ] **Step 5: Commit** `test(nav): nav_sim_lite closed-loop rostest`.

---

## Phase C — Isaac-only closed loop

### Task 10: M0 bridge spike

**Files:** Create `sim/bridge/Dockerfile`, `sim/bridge/bridge_topics.yaml`, `sim/bridge/spike_check.py`, `sim/isaac/spike_publish.py`, `results/sim_bridge_spike.md`.

**Interfaces:**
- Produces: image `tm_sim_ros:latest` (FROM `ros:noetic-ros-base-focal`; adds ROS 2 Foxy ros-base, `ros-noetic-desktop` subset needed by the nodes (`cv_bridge`, `tf2_ros`, `image_transport`, `rosbag`), `ros1_bridge` built from source against both, `python3-opencv`, `onnxruntime-gpu` (version compatible with the host CUDA driver), `yaml-cpp`); docker network `tm_sim_net`; topic list (Isaac → Noetic: `/cam/bottom/{left,right}/half/image_raw` sensor_msgs/Image, `.../camera_info`, `/tf`, `/tf_static`, `/clock`, `/sim/truth`, `/sim/deposition` std_msgs/String; Noetic → Isaac: `/corridor_nav/setpoint` geometry_msgs/PoseStamped, `/corridor_nav/spray` std_msgs/Bool).
- `spike_publish.py` (Isaac): empty stage, one cube, two cameras at 10 Hz, TF, clock; moves the cube toward received setpoints.

- [ ] **Step 1: Build** `docker build -t tm_sim_ros:latest sim/bridge` (log to `/data/work/logs`); host free disk on `/` must stay ≥ 8 GB (check `df -h /` before and after; Docker image layers live on `/`: if the image would breach that, stop and report).
- [ ] **Step 2: Run** `tm_isaac` (image `treehoppingnavigation-treehoppingsim:latest`, `--entrypoint bash`, `--gpus all`, `--network tm_sim_net`, `-e ROS_DOMAIN_ID=77`, repo mounted at `/ws:ro`, stream mode) with `spike_publish.py`, and `tm_sim_ros` with roscore + `parameter_bridge`.
- [ ] **Step 3: Run** `spike_check.py` in `tm_sim_ros`: for 60 s sim time measure per-topic rate and stamp lag (image stamp vs `/clock`), check TF `world→body` lookups, publish setpoints and confirm the cube's TF follows within 1 s.
  Expected: both images ≥ 9 Hz, camera_info present, `/clock` monotonic, setpoint round trip < 1 s sim.
- [ ] **Step 4: Write** `results/sim_bridge_spike.md` (rates, lags, RTF, decision). If the bridge fails Humble↔Foxy interop for any listed type, implement the fallback in the same task: `sim/bridge/relay.py` (rclpy subscriber → rospy publisher per topic, same names), and re-run Step 3 against it.
- [ ] **Step 5: Commit** `feat(sim): bridge container and M0 spike results`.

### Task 11: Scene layout and ground truth

**Files:** Create `sim/isaac/{layout.py,truth.py,scene.py}`, `sim/tests/{test_layout.py,test_truth.py}`, `sim/specs/{camera.json,3x3_smoke.json,5x5_tropical.json,rows_8x12.json}`; Modify `scripts/run_tests.sh` (add `python3 -m pytest -q sim/tests`).

**Interfaces:**
- Produces:
  - `layout.py`: `@dataclass Placement(id:int, species:str, x:float, y:float, z_ground:float, yaw:float, scale:float)`; `load_spec(path) -> dict` (validates required keys, raises `ValueError` naming the key); `plan_layout(spec: dict) -> List[Placement]` — seeded `numpy.random.default_rng(seed)`, rows along `heading_rad`, jitter uniform ±`jitter`, `missing_fraction` dropped, species by weight, scale uniform in `scale_range`, `z_ground` on the plane `z = tan(slope)·(x·cos(dir) + y·sin(dir))`; `ground_z(spec, x, y) -> float`.
  - `truth.py`: `tree_truth(placement: Placement, canopy_points_world: np.ndarray) -> dict` → `{id, species, centre, outline, radius, top_z, ground_z}` (outline = convex hull of xy, radius = sqrt(area/π), centre = hull centroid, top_z = max z); `write_truth(path, trees, frame="world")` also writes `ground_truth.json` in the scorer's contract (`contract.write_ground_truth` format, `height = top_z`, `"height_reliable": true`).
  - `scene.py` (Isaac only): `build_scene(stage, spec, assets_dir) -> List[dict]` — references `Orange_Tree.usd` / `Lemon_Tree.usd`, applies placement, extracts canopy mesh points in world space (`UsdGeom.Mesh` points × world transform, canopy = mesh prims whose names do not contain `trunk`, case-insensitive; if no such split exists, points above `z_ground + 0.25·(top − z_ground)`), adds an invisible convex-hull collider per canopy in collision group `canopy`, returns truth via `tree_truth`.
  - `sim/specs/camera.json`: copy `width`, `height`, `K`, `P` of left and right from the extracted Isaac recording's `camera_info.json` (`/data/work/extracted/isaac_2026-10-23-15-34-29/camera_info.json`), plus the mount from `ros/treemapper_ros/launch/treemapper_sim.launch` (`camera_mount`).
- Spec files: `3x3_smoke` (3×3, 6 m, seed 1), `5x5_tropical` (5×5, 7 m, jitter 1.5, seed 1001), `rows_8x12` (8 rows × 12, row 6 m, tree 4.5 m, missing 0.08, slope 0.03, seed 7), each with a `field.polygon` 5 m beyond the outer trees.

- [ ] **Step 1: Write failing pytest:** `test_layout.py::test_seeded_and_reproducible` (same spec → identical placements), `::test_counts_and_spacing` (8×12 with missing 0 → 96 trees, neighbour spacing 4.5 ± 2·jitter), `::test_missing_fraction` (8 % ± 4 % over seed range 0..9), `::test_bad_spec_names_key`; `test_truth.py::test_truth_from_points` (points of a sphere radius 2 at (3,4,5) → centre ≈ (3,4), radius ≈ 2 within 0.05, top_z ≈ 7), `::test_ground_truth_file_reads_with_contract` (written file loads with `treemapper_tools.contract.read_ground_truth`, `height_reliable is True`).
- [ ] **Step 2: Run** `docker exec tm_dev bash -lc 'cd /ws && PYTHONPATH=tools:sim python3 -m pytest -q sim/tests'` → FAIL.
- [ ] **Step 3: Implement** `layout.py`, `truth.py` (no Isaac imports), `scene.py`, specs, `camera.json`.
- [ ] **Step 4: Run** pytest → PASS. Isaac check: `orchard_sim.py --spec sim/specs/3x3_smoke.json --only-scene` (added in Task 12; until then a 10-line driver calling `build_scene`) writes `truth.json` with 9 trees whose `top_z` differ per instance.
- [ ] **Step 5: Commit** `feat(sim): seeded orchard layout and mesh ground truth`.

### Task 12: Drone, cameras and the sim entry point

**Files:** Create `sim/isaac/{controller.py,drone.py,cameras.py,orchard_sim.py}`, `sim/tests/test_controller.py`.

**Interfaces:**
- Produces:
  - `controller.py`: `class KinematicController: __init__(v_max_xy=6.0, v_max_z=2.0, a_max=2.0, yaw_rate_max=math.radians(60), tilt_tau=0.15, setpoint_timeout=1.0)`; `step(dt, now, setpoint: Optional[Tuple[x,y,z,yaw,stamp]]) -> State(x,y,z,vx,vy,vz,roll,pitch,yaw)`; no setpoint for `setpoint_timeout` → decelerate to hover; tilt target `atan(a/g)` with first-order lag.
  - `drone.py` (Isaac): Xform `/World/drone` driven by `KinematicController` each physics step; TF `world→body` publisher (OmniGraph `ROS2PublishTransformTree` or rclpy).
  - `cameras.py` (Isaac): two cameras from `camera.json` under `/World/drone/cam_left|cam_right` with the mount; `ROS2CameraHelper` rgb + camera_info at 10 Hz on the spec 5.4 topic names; frame ids matching camera_info's.
  - `orchard_sim.py`: CLI `--spec`, `--out`, `--only-scene`, `--physics-hz 60`, `--render-hz 10`; builds scene, drone, cameras, nozzle (Task 13), `/clock`; subscribes `/corridor_nav/setpoint`; publishes `/sim/truth` latched; logs RTF every 10 s; `--save-frame <sim s>` saves the left image at that time; exits on SIGTERM writing `deposition.json` and `rtf.json` (`{"sim_s", "wall_s", "rtf"}`).

- [ ] **Step 1: Write failing pytest** `test_controller.py`: `test_reaches_setpoint_with_limits` (10 m step → never exceeds 6 m/s or 2 m/s², arrives within 0.05 m), `test_vertical_limit`, `test_tilt_follows_acceleration` (steady 2 m/s² → pitch → atan(2/9.81) within 2 %), `test_hover_on_setpoint_timeout`.
- [ ] **Step 2: Run** → FAIL.
- [ ] **Step 3: Implement** controller, then Isaac modules.
- [ ] **Step 4: Run** pytest → PASS. Isaac check with `tm_isaac` + `tm_sim_ros`: publish a scripted square of setpoints from `tm_sim_ros` → `/tf` traces the square. Then hover at 15 m over the centre tree of `3x3_smoke`: its truth centre, projected with `camera.json` intrinsics and the body pose ∘ mount, lands within 10 px of the crown's centre in the saved left image (`--save-frame` flag of `orchard_sim.py`); report the pixel offset in the commit message.
- [ ] **Step 5: Commit** `feat(sim): kinematic drone, stereo cameras, sim entry point`.

### Task 13: Nozzle deposition

**Files:** Create `sim/isaac/{nozzle_model.py,nozzle.py}`, `sim/tests/test_nozzle_model.py`.

**Interfaces:**
- Produces:
  - `nozzle_model.py`: `cone_directions(n: int, half_angle_rad: float, seed: int = 0) -> np.ndarray (n×3, body frame, −z axis)`, uniform over the cone's solid angle; `class Deposition: __init__(flow_lps); add(hits: Sequence[Optional[int]], dt); open_time; to_json() -> {"trees": {id: litres}, "ground": l, "emitted": l, "open_time": s}`; `class NozzleGate: __init__(timeout=0.5); on_command(open: bool, now); is_open(now) -> bool`.
  - `nozzle.py` (Isaac): each physics step while `NozzleGate.is_open`, rotate `cone_directions(rays)` by the body pose, PhysX raycast against collision group `canopy` (max 30 m), map hit prim → tree id, `Deposition.add`; publish `/sim/deposition` at 2 Hz; `--out/deposition.json` on exit.

- [ ] **Step 1: Write failing pytest:** `test_cone_within_half_angle` (all directions within 20° of −z, mean ≈ −z), `test_deposition_conserves_volume` (64 rays, dt 1/60 for 4 s, half hit tree 3 → trees[3] + ground == emitted == 0.3 L within 1e-9), `test_gate_closes_after_timeout` (open at t=0, no message → closed at t=0.51).
- [ ] **Step 2: Run** → FAIL.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** pytest → PASS. Isaac check: hover 3 m above a 3×3 tree, open nozzle 4 s via `rostopic pub` → `deposition.json` shows ≥ 80 % of 0.3 L on that tree.
- [ ] **Step 5: Commit** `feat(sim): nozzle ray-cast deposition`.

### Task 14: Perception nodes

**Files:** Create `ros/sim_perception/{CMakeLists.txt,package.xml}`, `src/sim_perception/{__init__.py,sgbm.py,yolox.py}`, `scripts/{stereo_sgbm_node.py,detector_node.py}`, `launch/sim_perception.launch`, `test/{test_sgbm.py,test_yolox.py}`.

**Interfaces:**
- Produces:
  - `sgbm.py`: `make_matcher(params: dict) -> cv2.StereoSGBM` (defaults spec 7: num_disparities 64, block_size 5, uniqueness 10, speckle_window 100, speckle_range 2, `P1 = 8·block²`, `P2 = 32·block²`, mode `STEREO_SGBM_MODE_SGBM_3WAY`); `disparity(left_grey, right_grey, matcher) -> np.float32 array` (OpenCV fixed-point /16; values ≤ 0 → NaN).
  - `yolox.py`: `preprocess(img, input_size, channels) -> (tensor, ratio)` (YOLOX letterbox: resize by `min(h_in/h, w_in/w)`, pad 114, no normalisation, NCHW float32); `decode(raw, input_size, strides=(8,16,32)) -> np.ndarray` (YOLOX grid decode, used when `~decoded` is false); `postprocess(pred, ratio, score_thr, nms_thr, class_ids) -> List[(x, y, w, h, score, label)]` in input-image pixels.
  - `stereo_sgbm_node.py` (node name `stereo_sgbm`): exact-stamp sync of both `image_raw`; publishes `/disparity_image_down_float` (32FC1, left stamp + frame_id); `~counters` (DropCounters, stage `stereo`); `~timing` (StageTiming, stage `sgbm`).
  - `detector_node.py` runs as node name `detector`.
  - `detector_node.py`: `~model_path` (required), `~input_size` [416, 416] default, `~input_channels` 3, `~decoded` true, `~score_threshold` 0.3, `~nms_threshold` 0.45, `~class_ids` [0], `~device` cuda|cpu; publishes `treemapper_msgs/Detection2DArray` on `/object_detection/detections` with the image stamp; `~timing` (StageTiming).
  - `sim_perception.launch` args `detector_model` (no default: the user supplies the ONNX file shipped with the existing detector), `device`.

- [ ] **Step 1: Write failing pytest:** `test_sgbm.py::test_recovers_known_shift` (textured random image, right = left shifted 12 px → median disparity 12 ± 0.5 in the valid region; invalid → NaN), `test_yolox.py::test_letterbox_ratio_and_pad`, `::test_decode_grid` (single anchor at stride 8, grid cell x=2, y=3, raw offsets 0 → centre (16, 24), w = h = exp(0)·8 = 8, per YOLOX `(grid + xy)·stride`, `exp(wh)·stride`), `::test_nms_and_scaling` (two overlapping boxes IoU 0.8 → one; box scaled back by `1/ratio`; class filter drops label 1).
- [ ] **Step 2: Run** `python3 -m pytest -q ros/sim_perception/test` → FAIL.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** → PASS. Check against the recording: run `detector_node` on 20 left images extracted from the Isaac bag and compare with the recorded `/object_detection/detections` at the same stamps — mean matched-box IoU ≥ 0.8 (if the shipped model and threshold are right). Record the number in the commit message.
- [ ] **Step 5: Commit** `feat(sim): SGBM stereo and YOLOX detector nodes`.

### Task 15: Mission scorer

**Files:** Create `tools/treemapper_tools/mission_score.py`, `tools/tests/test_mission_score.py`.

**Interfaces:**
- Consumes: `contract.read_ground_truth`, `contract.Snapshot`, `score.score`, `score.to_markdown`; bag topics `/tf` (world→body), `/mission_state`, `/spray_target`, `/corridor_nav/spray`, `/corridor_nav/status`, `/treemapper/map`, `/treemapper/stage_timing`; files `truth.json`, `ground_truth.json`, `deposition.json`, and the `/sim/deposition` series in the bag.
- Produces:
  - Pure functions (tested without a bag): `dose_events(spray: List[Tuple[t, bool]]) -> List[Tuple[t0, t1]]`; `per_event_deposit(events, deposition_series) -> List[Dict[int, float]]` (difference of cumulative per-tree litres between `t0` and `t1 + 0.5`); `spray_metrics(events_deposits, truth, intended_l_per_event) -> dict` (sprayed ≥ 50 % intended, missed, double-sprayed = ≥ 20 % of a dose in two separate events, on-target fraction, off-target litres); `clearance_min(poses, truth, drone_radius) -> float`; `time_split(phases, status_states) -> dict`; `nozzle_outside_dose(spray, phases) -> int` (samples with nozzle true and phase ≠ SPRAYING).
  - CLI: `python3 -m treemapper_tools.mission_score --run <run dir> [--compare <run dir> ...] [--out <md>]` → markdown + `mission_score.json`; with several run dirs: mean and worst case per metric, grouped by strategy (read from `run.json` written by `sim_mission.sh`).
  - Cost section: `cost_metrics(timing_series) -> dict` — mean and p99 ms of `/corridor_nav/timing` (`step`), `/stereo_sgbm/timing` and `/detector/timing`; real-time factor from `run_i/rtf.json` (written by `orchard_sim.py` on exit: sim seconds / wall seconds). Tested by `test_cost_metrics_mean_p99`.
  - Map section: writes `map.jsonl` (via the same conversion as `treemapper_bag_to_jsonl.py`) and `mission.json` (phases, targets, pose from TF) into the run dir and calls `score.score(...)`.

- [ ] **Step 1: Write failing pytest:** `test_dose_events_pairs_open_close`, `test_double_spray_detected` (tree 2 gets 0.1 L in event 1 and 0.08 L in event 3 with 0.3 L intended → double), `test_missed_below_half`, `test_on_target_fraction`, `test_clearance_min_uses_truth_outline` (pose over tree top 6.0 at z 7.2 → 1.2; pose 3 m outside every outline ignored), `test_nozzle_outside_dose_counted`.
- [ ] **Step 2: Run** `cd tools && python3 -m pytest -q tests/test_mission_score.py` → FAIL.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** → PASS; `./scripts/run_tests.sh` → `ALL TESTS PASSED`.
- [ ] **Step 5: Commit** `feat(tools): closed-loop mission scorer`.

### Task 16: One-command mission, smoke and comparison

**Files:** Create `scripts/sim_mission.sh`, `ros/corridor_nav_ros/launch/sim_closed_loop.launch`, `results/corridor_nav_isaac.md`; Modify `README.md` (section "Isaac-only closed loop": build, run, score), `PROGRESS.md`.

**Interfaces:**
- Consumes: everything above.
- `sim_mission.sh <spec.json> [--strategy corridor|field_first] [--runs N] [--out DIR] [--detector-model PATH] [--timeout 1800]`:
  1. `run_i` dirs under `DIR` (default `/data/work/sim_runs/<spec-name>_<strategy>_<UTC time>`), `run.json` with spec, strategy, git commit, seed.
  2. Writes `field.yaml` (polygon, `heading_rad` from `rows.heading_rad`, home) from the spec.
  3. Starts `tm_isaac` (as Task 10 Step 2, `orchard_sim.py --spec ... --out run_i`) and `tm_sim_ros` (roscore, bridge, `sim_closed_loop.launch`: `sim_perception.launch` + `treemapper_sim.launch` + `corridor_nav.launch` with `strategy` override via a generated cfg copy in the run dir + `rosbag record -O run_i/mission.bag` of every topic except `image_raw`, plus `/cam/bottom/left/half/image_raw` throttled to 1 Hz on `/rec/left_1hz`).
  4. Waits for `/clock` and both camera_info; calls `/corridor_nav/start`; waits for state `DONE` on `~status` or `--timeout` sim seconds (then `~abort`, wait 60 s).
  5. Stops `tm_isaac` and `tm_sim_ros` only (`docker stop tm_isaac tm_sim_ros`), runs `mission_score.py --run run_i`; after all runs, `mission_score.py --compare` over them.
  - Refuses to start if a container named `tm_isaac` or `tm_sim_ros` already runs, or `/data/work` free space < 10 GB.

- [ ] **Step 1: Smoke**: `scripts/sim_mission.sh sim/specs/3x3_smoke.json --detector-model <path>` → mission reaches DONE; scorecard safety rows all 0 / pass (min truth clearance > 0, nozzle outside dose 0, sprayed tree re-targeted 0); 9/9 trees sprayed or each miss explained in the scorecard notes.
- [ ] **Step 2: Fix** anything the smoke exposes, each fix with a test in the owning task's test file, commit per fix.
- [ ] **Step 3: Comparison**: `5x5_tropical.json` and `rows_8x12.json`, `--strategy corridor` and `field_first`, `--runs 3` each.
- [ ] **Step 4: Write** `results/corridor_nav_isaac.md`: setup (commits, specs, detector model, RTF), per-strategy mean / worst / run-to-run spread for every scorecard group, corridor vs field_first differences with the noise floor stated, known weaknesses, and the line "Not verified on the Orin or on a real flight". Update README and PROGRESS.md.
- [ ] **Step 5: Commit** `results: corridor_nav closed loop in Isaac, corridor vs field_first`.

---

## Execution order and dependencies

Tasks 1 → 7 strictly in order (each consumes the previous interfaces). Task 8 needs 7; Task 9 needs 8. Task 10 is
independent of 1–9 and should start first in calendar time if two people work (it carries the main risk). Tasks 11–13
need 10; 14 needs 10; 15 is independent of 10–14 (pure functions + fixtures); 16 needs everything.
