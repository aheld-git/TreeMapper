# Project brief: on-board tree mapping for an autonomous spot-spraying drone

You will work on this autonomously, end to end, without a human to ask. Read the whole brief before you start. Section 11 explains how to work. Section 12 says what must be ready when I come back.

## 1. Mission

An agricultural drone flies over orchard rows and sprays individual trees instead of whole fields. It does this in two phases. First it scans a corridor and builds a map of the trees. Then it hops from tree to tree, descends over each one, sprays it, and moves on.

You are building the **perception system** that turns the drone's camera stream into that map. A separate navigation system decides where to fly. It treats your map as ground truth: it picks targets from it, computes its descent altitude from it, and records which trees are done. Navigation does no checking of its own, so every perception error turns into a flight error: a missed tree, a tree sprayed twice, spray landing between trees, or a descent to the wrong height.

A working system already exists. **Your job is not to recreate it but to beat it**, and the evaluation harness you build (section 9) is how that will be decided. You will not see the existing design, and that is deliberate. Reason from the problem.

## 2. Scope

**In scope:** everything from **2D tree detections onward**. That means associating detections over time, recovering each tree's 3D points from stereo, fusing them into map entries, deciding when entries are the same tree, behaving correctly in each mission phase, and producing the diagnostics.

**Out of scope:** the 2D tree detector itself. Treat it as an existing, pluggable stage that delivers per-frame bounding boxes with confidences. It is imperfect: it misses trees, produces false positives, jitters, and sometimes draws one box over two touching trees. Your system must be robust to all of that.

## 3. Target platform

The result must run on the drone. The drone runs **ROS 1 (Noetic) on an NVIDIA Jetson Orin Nano (arm64)**, with the system written in **C++17**. Other software runs alongside it, and the detector already uses much of the GPU, so assume your stages get a modest CPU budget and must keep up with the camera frame rate. Avoid dependencies that do not exist on, or are impractical for, an arm64 embedded Linux system.

**Make the core logic independent of ROS.** It should be a library with plain inputs and outputs, wrapped by thin ROS nodes. That lets you develop and test everything in the cloud without ROS or a GPU, and lets me drop the result onto the drone.

## 4. Output contract

This is what navigation consumes. For each tree, keep one map entry with:
- **A stable integer ID.** Navigation stores IDs and refers back to them later. An ID must never change meaning.
- **A ground footprint**: a 2D canopy outline seen from above, in a fixed world frame.
- **A centre point** to fly to.
- **A canopy height**, used as the descent reference.
- **A sprayed flag**, set by navigation.

The system must provide:
- A **query** that returns all trees, or one tree by ID.
- A **command** to mark a tree as sprayed.
- **Live visualisation** for an operator.
- **Health and diagnostics topics.**

The system must accept as inputs:
- **Downward-facing stereo**: left/right images or a disparity image, plus calibration.
- **The 2D detections.**
- **The drone's world pose**, including altitude.
- **The mission phase** from navigation: scanning, planning, hopping, spraying, returning, idle, error.
- **The current hop target's ID.**

Define these interfaces as ROS messages and services in a separate package, and document each field.

## 5. Operating conditions

- **Scanning.** The drone scans at cruise speed, a few metres per second, in a **single pass**. A tree must be captured reliably the first time it is seen, not only after hovering over it.
- **Stereo depth.** Depth gets noisier as distance grows. Errors are largest along the camera's line of sight and much smaller across it.
- **Clutter.** Trees in a row are close together and their canopies can touch. Gaps between trees can be smaller than the depth noise. Ground, grass and neighbouring trees are always somewhere in the image.
- **Motion blur and noise.** Fast turns at the ends of corridors blur the images and produce noise bursts.
- **No persistent map.** The world frame is reset every boot, and there is only plain GNSS, with metres of offset between sessions. No map carries over from one flight to the next.
- **Two kinds of input data.** The system must run on replayed real flight logs and on a physics simulator. The simulator uses different frame conventions and produces much denser, cleaner depth. Switching between them must be a configuration change only.

## 6. Behaviour across mission phases

- **Scanning.** Build the map. New trees can be created freely.
- **Planning and hopping.** The tree list is fixed and **no new standalone trees may appear**, because navigation has already planned against it. Observations may still grow and correct existing trees, since closer views make outlines more accurate.
- **Descending onto a target.** **Only the target tree may change.** Low-angle views during a descent can damage a neighbouring tree, so all other trees are frozen.
- **Spraying, idle and pre-mission.** Heavy processing may sleep to save compute. While asleep, keep sending a heartbeat that says "sleeping, not dead".
- **Unknown or missing phase information.** Behave as in scanning (fully awake, permissive) and log a warning. The system must work with no navigation running at all, for example during log replay.

## 7. Non-negotiable rules

1. **IDs are stable and never reused.** A tree's ID survives merges, track changes and track jumps. A freed ID never goes to a different tree.
2. **No silent damage to published geometry.** Navigation must never see a tree's outline suddenly shrink, jump or deform because of one bad frame. You decide how outlines evolve: grow-only, confidence-latched, or something better. But a single noisy frame, or a few, must not corrupt a published tree. Document your choice and why it is safe.
3. **Height is always a real number once a tree is published.** An undefined height reaches navigation as a descent altitude, which is a flight-safety bug. Do not publish a tree until it has a valid outline and height, and never publish an undefined height afterwards.
4. **One physical tree is one map entry.** Fragments of a tree must merge, including fragments that are no longer observed. Neighbouring trees must not merge. When entries merge, the better-established one keeps its identity and height.
5. **Sprayed trees are protected.** They stop changing. A detection that lands inside a sprayed tree must not create a new tree that would be sprayed again.
6. **Fail loudly and predictably.** For every gate, decide on purpose whether it fails open or fails closed when information is missing, document the choice, and log when it triggers.
7. **Every dropped data point is counted by reason** at every stage, and the counts are published. A silent drop cannot be debugged.

## 8. Known failure modes to design against

- **Detection box corners land on ground or on a neighbouring tree.** Those points must not end up in a tree's outline.
- **Points above the drone, or very far away,** are physically impossible or unreliable.
- **Start-up artefacts.** Processing before the drone pose is known freezes garbage into the map. Hold processing until the pose is valid.
- **A track jumping to another tree.** Deleting the old tree when its track jumps loses real trees. Only throw away brand-new, unconfirmed entries.
- **Several tracks on one tree.** All of their observations in a frame must count, not just the first track's.
- **Size-blind merge rules.** A small fragment inside a large tree should merge easily. Two large overlapping trees need far stronger evidence.
- **Hovering inflating confidence.** A tree seen for a long time must not end up with distorted shape or confidence compared with one seen once at cruise speed.
- **Simulation is too easy.** Anything tuned only on clean, dense synthetic depth will fail on real stereo. Make your synthetic data hard (section 9).

## 9. The evaluation harness, built first, because it decides "better"

Before building the system, build a **ground-truth world and a scorer**. They are the most important deliverable after the system itself.

**Synthetic orchard generator.** It should produce:
- Rows of trees with varied size and shape, touching canopies, gaps smaller than the depth noise, missing trees, ground clutter, and slopes.
- Scan trajectories at cruise speed, corridor turns with blur and noise bursts, and full missions (scan, plan, hop, descend, spray).
- Realistic stereo depth noise: range-dependent and stronger along the line of sight, plus dropouts and outliers.
- A detector simulator with misses, false positives, box jitter, and merged boxes over touching trees.

Make the noise harder than reality, not easier. Seed everything so runs can be reproduced.

**Scorer.** The scorer must work on **any system that emits the output contract**, so the existing system can be scored on the same data and real flight logs can be scored too once ground truth exists. It should report at least:
- **Detection:** each true tree appears exactly once. Report misses, duplicates, false trees, wrongly merged neighbours, and split trees.
- **Geometry:** outline overlap with the truth, area leaking outside the true canopy (especially into neighbours), centre error, and height error.
- **Stability:** ID switches, ID reuse, and published geometry that shrinks or jumps.
- **Safety:** frames with an undefined height published, new trees after scanning, and non-target changes during a descent. These must all be zero.
- **Timeliness:** how long until a tree is first published at cruise speed.
- **Cost:** processing time per frame, reported per stage.

**Aggregate across many seeds and scenarios.** Report means and worst cases, and measure the run-to-run noise floor before claiming any improvement.

**Recorded data provided** (download location given separately). These are ROS 1 bags. The message definitions are embedded in each bag, so they can be read without my packages.
- **Simulator missions with ground truth.** These contain:
  - stereo images, camera calibration, disparity, the transform tree and the sim clock;
  - the existing system's 2D detections, so you need no detector;
  - drone position, mission phase and hop target;
  - ground-truth trees;
  - the existing system's published map, which is **the baseline to beat**.

  Ground-truth outlines are approximations: a circle per tree with a per-tree radius, plus a per-tree canopy height. Score against them with that tolerance in mind, and do not tune your system to produce circles. Mark-as-sprayed is a service and is not in the bags; the sprayed state is visible in the recorded map.
- **One real flight.** It contains real stereo images, calibration, disparity and the transform tree, with 2D detections from the existing detector in a companion bag. It has no ground truth. Use it for robustness, stability and safety metrics, cost, and visual sanity checks, and to calibrate your synthetic noise against real stereo.

Use the recorded simulator missions as a **held-out test set**: tune on synthetic data, then report on the recordings. Report scores on recorded and synthetic data separately.

## 10. Design freedom and questions to answer

The structure is yours to choose. Explore at least two genuinely different architectures and compare them with the scorer before committing to one. For example, per-object tracking and fusion versus a spatial evidence map that confirms tree regions, or a hybrid. In your design document, answer these:
- How do you separate touching trees when the gap is below the depth noise?
- Where and how do you reject points from the ground and from neighbours, given rule 2?
- How do you keep IDs stable through track loss, jumps and merges?
- What is your confidence model for when a tree becomes publishable, and when its geometry may still change?
- How would a map from an earlier flight be reused, given frame resets, metres of GNSS offset and highly repetitive rows? A design is enough for this one; an implementation is optional.

## 11. How to work

- **Never stop to ask.** When something is ambiguous, choose the safest reasonable option, note it as an assumption in the decision log, and keep going.
- **Work in milestones**, each one ending green and committed:
  1. interfaces and contract;
  2. synthetic world and scorer;
  3. a deliberately simple baseline system, scored;
  4. candidate architectures, scored against each other;
  5. hardening of the chosen design;
  6. ROS wrappers and launch configuration;
  7. handoff.
- **Test every rule in section 7.** Each one needs a test that would fail if the rule were broken. Keep decision logic in small pure functions that can be tested on their own.
- **Keep all tunable parameters in one configuration file.** Built-in defaults must equal the shipped config, and a test must enforce that. Validation and clamping live in one place, and every clamp is logged.
- **Keep a decision log.** Record each meaningful choice, the alternatives you considered, the evidence from the scorer, and every assumption.
- **Write honest results.** If something is not verified, for example anything that needs the real Jetson or real flight logs, say so plainly. Never claim performance on hardware you did not run on.
- **Use the CPU budget as a design constraint.** Profile in the cloud and assume the embedded CPU is several times slower.

## 12. What must be ready when I come back

1. **The system**, as a buildable ROS 1 workspace: the core library, thin nodes, an interface package, and launch files for real-log and simulator modes.
2. **The synthetic world, the scorer** and a **one-command evaluation** that prints a scorecard.
3. **A results report**: the scorecard of the final design against the simple baseline, against each alternative you tried, and against **the existing system's recorded map** on the recorded simulator missions. Include worst cases, the noise floor, and known weaknesses.
4. **A design document** answering section 10.
5. **A handoff README** covering:
   - how to build on the Jetson;
   - how to run on a recorded flight log;
   - how to run the scorer against a different system that emits the same contract, so I can compare it with the existing one;
   - every tunable parameter and what it trades off;
   - what is unverified and what I should test first on real hardware.
6. **The decision log.**
