# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & run environment (read first)

- **This repo is edited on a different machine than it is built/run.** The checkout you are editing has **no ROS 2 / colcon / Gazebo toolchain**. Actual `colcon build` / `colcon test` / simulation happen on a separate **Ubuntu 22.04 + ROS 2 Humble + Gazebo** machine, to which the developer **manually copies files** before building. So: make code changes here, then give the developer an explicit list of files to copy + commands to run, and treat their reported build/test output as the source of truth.
- Because Eigen/PCL/rclcpp/small_gicp headers aren't on this machine, **clang/clangd "file not found" / "undeclared identifier" diagnostics are false positives** — ignore them; trust the colcon build on the build machine.
- **The colcon workspace root is `src/`** (not the repo root). All build/run commands are run from `src/`: `cd src && colcon build ...`, then `source install/setup.bash`.
- Target platform is Linux-only. Frequencies measured with `ros2 topic hz` appear halved because Gazebo RTF ≈ 0.5 (correct in sim time).

## What this project is

A Gazebo simulation for LiDAR SLAM that builds toward **FAST-LIO2 + GICP relocalization in a LIO-SAM prior map**. Three stages, decoupled in development but stacked at runtime:

1. **LIO-SAM** (`src/LIO-SAM`, upstream) — builds & saves the prior-map PCD (`/lio_sam/save_map` → `~/result`).
2. **FAST-LIO2** (`src/FAST_LIO`, upstream) — LiDAR-inertial **odometry** (Stage 1, done). Publishes TF `camera_init→body`, `/Odometry`, `/cloud_registered`.
3. **`src/gicp_localization`** (this repo's package, Stage 2, done) — GICP scan-to-prior-map localization. Publishes the correction TF `map→camera_init` and `/localization`.
4. **Stage 3 (future)** — global relocalization front-end (Quatro) so localization can recover from arbitrary poses.

Runtime TF chain: `map →[gicp_localization]→ camera_init →[FAST-LIO]→ body`. The prior map is in LIO-SAM's `map` frame; FAST-LIO's `camera_init`/`body` are **not wired into the robot URDF TF tree** (RViz warnings about `base_footprint→map` are expected, not bugs).

See `docs/sim-dataflow-lio-sam.md` for the data flow and the README for full setup/run/acceptance per stage.

## Repository conventions

- **Upstream dependencies are NOT forked.** `FAST_LIO`, `LIO-SAM`, `velodyne_simulator`, `small_gicp`, `robot_gazebo` are **manually `git clone`d, pinned to a specific commit, and `.gitignore`d** (see README for exact clone commands + pinned SHAs). Local modifications to them are delivered as **patch files** under `src/<name>-patch/`, numbered with a comment header (e.g. `src/fast-lio2-patch/01-...patch`), applied via `git apply`. **Simulation-only** patches live in a `sim-only/` subdir and must never be applied to real hardware (they document why). `src/lio-sam.patch` is the LIO-SAM equivalent.
- **`docs/superpowers/` (spec & plan files) is `.gitignore`d and never committed** — process artifacts only; deliverables are patches + README + package code.
- Commit messages: Conventional Commits with a scope, written in Chinese (`feat(gicp): …`, `fix: …`, `docs: …`, `tune: …`, `chore: …`). Push only when explicitly asked.
- `src/livox_ros_driver2` is a **message-stub package** (only `CustomMsg`/`CustomPoint`), present solely to satisfy FAST-LIO's compile-time dependency in this velodyne-only setup — no Livox-SDK needed.

## Build, test, run

```bash
cd src
# Stage 1: FAST-LIO (livox stub builds first, then fast_lio)
colcon build --packages-up-to fast_lio
# Stage 2: GICP localization (small_gicp builds first; needs `sudo apt install libomp-dev`)
colcon build --packages-up-to gicp_localization
#   debug build adds the diagnostics msg/publisher (off by default):
colcon build --packages-up-to gicp_localization --cmake-args -DGICP_DIAGNOSTICS=ON

# Unit tests (gicp_localization)
colcon test --packages-select gicp_localization && colcon test-result --verbose
# A single test:
colcon test --packages-select gicp_localization --ctest-args -R test_pose_math

# Run (three terminals, each: cd src && source install/setup.bash)
ros2 launch robot_gazebo robot_sim.launch.py
ros2 launch fast_lio mapping.launch.py config_file:=gazebo_velodyne.yaml use_sim_time:=true
ros2 launch gicp_localization localization.launch.py prior_map_path:=~/result/GlobalMap.pcd
```

## gicp_localization architecture (Stage 2)

- **Single node, two timers** under a `MultiThreadedExecutor` + two callback groups: a slow timer (`localization_freq`, 0.5 Hz, MutuallyExclusive) runs GICP; a fast timer (`tf_pub_freq`, 50 Hz, Reentrant) broadcasts `map→camera_init` and publishes `/localization`. One `std::mutex` guards shared state; the GICP callback copies inputs under the lock, runs `align()` **without** the lock, then re-locks to commit — so the heavy registration never blocks the 50 Hz TF.
- Registration uses **koide3/small_gicp** (`preprocess_points` builds the target KdTree+covariances once; `align(target, source, target_tree, seed)` per scan). Because the source `/cloud_registered` is already in the `camera_init` frame, `result.T_target_source` **is** `T_map→camera_init` directly. The seed is the previous correction, or `/initialpose` (RViz "2D Pose Estimate") on demand.
- **Acceptance gate is fitness-only** (`fitness = num_inliers/num_source ≥ fitness_threshold`); `converged` is reported in diagnostics but does **not** gate (small_gicp often reports `converged=false` on dense scans even when correct). On reject, the last good correction is held.
- **Diagnostics are a build-time switch, with zero `#ifdef` in node logic.** `option(GICP_DIAGNOSTICS)` in CMake selects which `.cpp` is compiled: `diagnostics_real.cpp` (generates `GicpDiagnostics.msg`, publishes `~/diagnostics`) or `diagnostics_null.cpp` (no-op). The node always calls `diag_->report(m)`. `gicp_localization_core` and `_diag` are **static** libs linked into the executable (don't reintroduce shared-lib install targets — that caused a missing-`.so` bug).
- small_gicp has **no file IO**; the PCD prior map is loaded with PCL (`loadPriorMapPcd`) and re-published latched on `~/prior_map` (frame `map`) for RViz.

## Simulation gotchas (already known — don't rediscover)

- **Velodyne sim clouds lack per-point `time`.** FAST-LIO would fabricate timestamps and smear on rotation; the sim-only patch `disable-deskew-snapshot-lidar.patch` forces `given_offset_time=true`. LIO-SAM auto-disables deskew (its real-time `base_footprint` TF drifts, but the saved map is fine).
- **`src/robot_gazebo/.../robot.sdf` is physically ill-conditioned** (tiny steering-link inertia, light front wheels, mesh collision, free zero-damping steering joint) → vertical jitter that grows over ~1 min. Several mitigations tried and failed; drive gently. Don't relitigate without new ideas.
- **The sim does not drift.** In this feature-rich factory, FAST-LIO is fully constrained by LiDAR scan-matching, so IMU noise (even extreme) induces no drift — GICP's drift-correction benefit is not demonstrable here. The factory's **90° rotational pseudo-symmetry** creates a strong false GICP minimum at `fitness≈0.8`; the default `fitness_threshold=0.9` rejects it. GICP is local — large offsets / ~90° won't auto-recover (that's Stage 3); bootstrap needs a correct `/initialpose` near the robot's actual map pose.
