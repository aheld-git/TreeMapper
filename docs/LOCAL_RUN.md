# Overnight local run: environment, rules, time box

This file **overrides** the cloud assumptions in `PROJECT_BRIEF.md`. Everything else in the brief stands.

## Where you are

- **Machine:** the developer's workstation. Linux Mint 21.2 (Ubuntu 22.04 base), x86_64, 16 cores, 31 GB RAM, and an RTX 3070 you do not need. Other people's jobs run here too.
- **Project:** `/home/alex/SUIND/TreeMapper`, a local git repo on branch `main` with **no remote**. Commit often. Never add a remote or push.
- **Data:** `/media/alex/waspsim/cloud_build_bundle/`, an external SSD. See `docs/DATA_README.md`. Treat `isaac/` and `real/` as **read-only**. Never modify, move or delete them.
- **Scratch:** `/media/alex/waspsim/cloud_build_bundle/work/`. Synthetic datasets, extracted frames, caches, virtualenvs and anything large go here. Keep the total under **40 GB**.
- **Root disk:** about 20 GB free and shared. Keep everything big off it: no large files in the repo, no `docker build`, no new images.
- **ROS 1 Noetic:** the host has no ROS. Use the existing Docker image `treehoppingnavigation-bridge:latest`, which has ROS Noetic desktop-full, catkin and CGAL 5.5. **Always override its entrypoint**, because the default entrypoint starts someone else's node graph:
  ```
  docker run --rm --name tm_<purpose> --entrypoint bash --cpus 8 \
    -v /home/alex/SUIND/TreeMapper:/ws \
    -v /media/alex/waspsim/cloud_build_bundle/isaac:/data/isaac:ro \
    -v /media/alex/waspsim/cloud_build_bundle/real:/data/real:ro \
    -v /media/alex/waspsim/cloud_build_bundle/work:/data/work \
    treehoppingnavigation-bridge:latest -lc 'source /opt/ros/noetic/setup.bash && ...'
  ```
  Always mount the data read-only (`:ro`) exactly as shown: the container runs as root and would otherwise bypass file permissions. Files the container writes into `/ws` are owned by root, so pass `--user $(id -u):$(id -g)` when you can. Use it to build and test the ROS wrappers, and to read bags with the real `rosbag` API if you want that. Pure-Python bag reading on the host, for example `rosbags` in a venv under `work/`, is also fine.
- **Target hardware:** the Jetson Orin Nano is **not available tonight**. Do not ssh anywhere. Design for it as the brief says (C++17, arm64-friendly dependencies, a modest CPU budget), and list what needs on-device verification in the handoff.

## Hard rules: never break these

1. **Don't touch other work.** Do not stop, remove, restart or exec into containers you did not start. `mlflow` and `wasp_sim` belong to other work. Never run `docker compose`, `docker stop/kill/rm/rmi/prune` on anything but your own `tm_*` containers. Never touch `/home/alex/SUIND/*` other than `TreeMapper`, and never touch `/media/alex/waspsim/*` other than `cloud_build_bundle/work`.
2. **No network side effects.** No pushes, no uploads, no messages, no ssh. Downloading packages from pip or apt mirrors is fine.
3. **Share resources.** Limit parallel jobs to 8 cores (`-j8`, `--cpus 8`). Stop if host free disk on `/` drops below 8 GB.
4. **Never wait for a human.** Nobody will answer before morning. When something is ambiguous, choose the safest reasonable option, record it in `docs/DECISIONS.md` and continue.

## Time box

The run ends at **07:30 local time**. The driver will tell you when the wrap-up window starts. Plan so that **something usable exists early** and improves:

| By about | Milestone |
|---|---|
| 00:30 | Interfaces and contract; bag readers; ground-truth and baseline extraction from the recorded sim mission; the scorer working on the **existing system's** recorded map (the baseline scorecard) |
| 02:00 | Synthetic orchard generator and a simple baseline system, scored on synthetic and recorded data |
| 05:00 | At least two candidate architectures, compared with the scorer; pick one |
| 06:45 | Hardening, rule tests, ROS 1 wrappers built in the Noetic container, launch files for real-log and sim modes |
| 07:30 | Handoff: README, results report, design doc, decision log; everything committed |

If you fall behind, cut scope from the alternatives, never from the scorer, the safety-rule tests or the handoff.

## How to work

- **You are the orchestrator.** Delegate implementation to subagents with the Agent tool: `model: "sonnet"` for mechanical, well-specified work, `model: "opus"` for design-heavy work and for reviewing each milestone. Give every subagent a self-contained brief with exact paths. Tell subagents that need to produce a report to return it as text, or write it with `cat > file <<'EOF'`, because their Write tool may refuse report files.
- **Verify before claiming.** A milestone is done only when its tests pass and its scorecard is in the repo.
- **Keep `PROGRESS.md` current** at the repo root, updating it after every milestone and at least every hour. It should show: current milestone, what is done, the latest scorecard, blockers, and what comes next. The supervisor reads it hourly.
- **Supervisor guidance** arrives appended to the driver's resume prompt (and is archived under `overnight/inbox_archive/`). Follow it, and acknowledge it in `PROGRESS.md`.
- **When everything in brief section 12 is delivered and committed**, write `overnight/DONE` containing a one-line summary. The driver stops when it sees that file.
