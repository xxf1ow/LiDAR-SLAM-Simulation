# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & run environment (read first)

- **This machine is the single dev/build/run host.** It runs **Ubuntu 22.04 + ROS 2 Humble + Gazebo Sim Harmonic** with the full toolchain on PATH (`colcon`, `ros2`, `gz`). Editing, `colcon build` / `colcon test`, and simulation all happen here — there is no longer a separate build machine and no file-copying step. Make changes, build, and verify the output directly.
- With the toolchain installed locally, clangd diagnostics are generally trustworthy. If a third-party header (Eigen/PCL/small_gicp/gz) still occasionally reports "not found", treat `colcon build` output as the source of truth rather than chasing the diagnostic.
- **Python uses the system `/usr/bin/python3` (3.10) directly — there is no `uv`/`.venv`.** It was removed to stop it polluting the build: a leaked `.venv` had made CMake cache `Python3_EXECUTABLE=.venv/bin/python3` and broke rosidl/ament with `No module named 'em'` / `'catkin_pkg'`. Run ad-hoc scripts with `python3 ...` (resolves to `/usr/bin/python3`) and install Python deps via `sudo apt` / system `pip`. With `uv`/`.venv` gone there is no longer anything to exclude from `PATH`.
- **The colcon workspace root is `core/`** (not the repo root, not `src/`). All build/run commands are run from `core/`: `cd core && colcon build ...`, then `source install/setup.bash`. `core/` is **self-contained** — upstream clones live under `core/<module>/`, not in a separate `src/`.
- Target platform is Linux-only. Frequencies measured with `ros2 topic hz` appear halved when RTF < 1 (correct in sim time).

## Running the sim stack — memory discipline (read before launching Gazebo)

This box has only **7.6 GB RAM**; one full sim stack (`robot_gz` + `fast_lio` + `gicp_localization` + `nav2`, plus Gazebo Harmonic's ogre2 renderer) uses **3–5 GB**. Two stacks at once, or one stack plus leaked orphan processes, **will OOM and freeze the machine** — this has already happened and required a hard reboot. Before launching Gazebo:

- **One stack at a time; check `free -h` first** — need ~2 GB available before starting Gazebo, otherwise stop and clean up.
- **`ros2 launch ... &` orphans its children when the launching shell exits or is killed.** The `gz sim` (ruby) process, `controller_manager`, `spawner`, `parameter_bridge`, `adapter_node`, `robot_state_publisher`, and every nav2 node keep running as orphans and pile up across runs — **this is the #1 cause of OOM**.
- **Tear down with the process group, never `pkill -f`.** Launch the stack with `setsid`; kill the whole tree with `kill -- -<PGID>`. `pkill -f` is unsafe here: it (a) leaks `adapter_node`/`robot_state_publisher`/`controller_manager`/`spawner`, and (b) an inline `bash -c '... pkill -f "gz sim" ...'` **matches its own command line and kills the shell running it**. Put cleanup commands in a script file (`bash /tmp/clean.sh` — its process name doesn't contain the pattern).
- **Verify cleanup before the next run:** `pgrep -af 'gz sim|fast_lio|gicp|nav2|controller_manager|adapter_node|robot_state_publisher'` must be empty **and** `free -h` must show memory reclaimed. No new stack until both hold.
- **Don't spin up redundant background stacks.** Reuse a running stack for the next check; tear it down only when truly done.

## What this project is

A Gazebo **Sim Harmonic** rebuild of a LiDAR-SLAM simulation, on **ROS 2 Humble**, built bottom-up in phases and organized as **six decoupled modules** under `core/` (five functional — robot/simulation/mapping/localization/navigation — plus `bringup` for one-entry full-stack launch and pre-launch consistency gating). The runtime goal: a differential-drive robot autonomously navigating a factory world using **FAST-LIO2 odometry + GICP localization in a LIO-SAM prior map + Nav2**.

This is a **rebuild** of an older Gazebo-Classic stack (the former `src/` workspace, now deleted). It follows official references (`ros2_control_demos` for the robot, Nav2 examples for navigation) and a clean ros2_control `sim/mock/real` switch rather than copying the old implementation.

Runtime TF chain (full stack): `map →[gicp_localization]→ camera_init →[FAST-LIO]→ body →[static weld]→ base_footprint →[URDF]→ base_link → velodyne/imu_link/wheels`. The prior map is in LIO-SAM's `map` frame; FAST-LIO's `camera_init`/`body` are wired into the robot URDF tree only at the navigation stage via the `body→base_footprint` weld.

## Repository structure (`core/` = six modules)

| Module | Contents | Role |
|---|---|---|
| `core/robot/` | `robot_description` (single `robot.urdf.xacro`, gz/mock/real tri-state), `robot_hardware` (C++ ros2_control `SystemInterface`, currently a loopback placeholder), `robot_bringup`, `drivers/chassis_8030d` (vendor `can_driver` + manual web acceptance tool); future real lidar/IMU drivers live under `drivers/lidar_<model>` | Differential-drive robot model + ros2_control + real device drivers |
| `core/simulation/` | `robot_gz_bringup` (`robot_gz.launch.py`, `worlds/factory.sdf`, `config/bridge.yaml`, `sticky_teleop.py`), `lidar_pointcloud_adapter` (Gz cloud → Velodyne-style `/points_raw`), `spike` (lidar smoke test) | Gz Harmonic world + sensor bridging |
| `core/mapping/` | `lio-sam.patch` (+ `LIO-SAM` clone, gitignored) | LIO-SAM builds & saves the prior map PCD (`~/result/GlobalMap.pcd`) |
| `core/localization/` | `gicp_localization` (in-repo package), `fast-lio2.patch` (+ `FAST_LIO` clone), `small_gicp` clone, `livox_ros_driver2` (msg stub, in-repo) | FAST-LIO2 odometry + GICP scan-to-prior-map localization |
| `core/navigation/` | `robot_navigation` (ament_python: `pcd_to_occupancy`, `twist_stamper`, `nav2_params.yaml`, `navigation.launch.py`) | Nav2 autonomous navigation (Smac Hybrid-A\* + MPPI) |
| `core/bringup/` | `system_bringup` (ament_python: `consistency_check.py`, `bringup.launch.py` + `config/bringup.yaml`, `slam_stack.launch.py`) | One-entry full-stack launch (sim/real × mapping/navigation) + pre-launch cross-module consistency gate |

Each functional module has its own `README.md` with detailed build + run + acceptance steps. The top-level `README.md` is an overview that points into them.

## Repository conventions

- **Upstream dependencies are NOT forked.** `FAST_LIO`, `LIO-SAM`, `small_gicp` are **manually `git clone`d into their owning module** (`core/localization/FAST_LIO`, `core/mapping/LIO-SAM`, `core/localization/small_gicp`), pinned to a specific commit, and **`.gitignore`d** (see each module README for exact clone commands + pinned SHAs). `core` is self-contained: colcon discovers the clones in-place, no separate workspace.
- **`gz_ros2_control` is the one upstream that lives outside `core/`.** Its apt package (`ros-humble-gz-ros2-control`) is incompatible with Gazebo Harmonic, so the `humble` branch is source-built in a separate workspace `~/res2_ws` and globally sourced via `.bashrc`. It provides sim's `controller_manager` (the `gz_ros2_control-system` URDF plugin) — do not delete `~/res2_ws`. Build steps: `core/simulation/robot_gz_bringup/README.md` §0.
- Local modifications to upstream are delivered as **patch files tracked in the owning module** (`core/localization/fast-lio2.patch`, `core/mapping/lio-sam.patch`), applied via `git apply`. **Simulation-only** patches live in a `sim-only/` subdir and must never be applied to real hardware (they document why). The correct way to edit upstream config: edit the clone's working tree → `git diff > ../<name>.patch` → commit the patch.
- `core/localization/livox_ros_driver2` is a **message-stub package** (only `CustomMsg`/`CustomPoint`), present solely to satisfy FAST-LIO's compile-time dependency in this velodyne-only setup — no Livox-SDK needed.
- The factory world `core/simulation/robot_gz_bringup/worlds/factory.sdf` is a **one-time** Classic→Harmonic conversion + hand-optimization, committed as the final artifact (no generator script). Its `model://` mesh visuals reference Classic asset libraries kept under `models/` (gitignored, must be present on this host under `models/` before running).
- Long-term direction lives in `docs/ROADMAP.md` (product vision + current progress + todo). Per-stage spec/plan artifacts are transient and not kept in-repo; deliverables are patches + READMEs + package code.
- Commit messages: Conventional Commits with a scope, written in Chinese (`feat(gicp): …`, `fix: …`, `docs: …`, `tune: …`, `chore: …`), ending with `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`. Push only when explicitly asked.

## Build, test, run (from `core/`)

```bash
cd core
# Robot + simulation (Gz Harmonic; controller_manager comes from the gz_ros2_control URDF plugin)
colcon build --packages-select robot_hardware robot_description robot_bringup lidar_pointcloud_adapter robot_gz_bringup
# Real 8030D device acceptance (aarch64 only)
colcon build --packages-select can_driver can_driver_web_control
# Mapping (LIO-SAM clone must be cloned+patched first)
colcon build --packages-up-to lio_sam
# Localization: FAST-LIO (livox stub builds first) then GICP (small_gicp first; needs `sudo apt install libomp-dev`)
colcon build --packages-up-to fast_lio
colcon build --packages-up-to gicp_localization
#   debug build adds the diagnostics msg/publisher (off by default):
colcon build --packages-up-to gicp_localization --cmake-args -DGICP_DIAGNOSTICS=ON
# Navigation
colcon build --packages-select robot_navigation

# Unit tests
colcon test --packages-select gicp_localization && colcon test-result --verbose
colcon test --packages-select robot_description robot_navigation lidar_pointcloud_adapter

# Run (each terminal: cd core && source install/setup.bash)
ros2 launch robot_gz_bringup robot_gz.launch.py                                            # sim
ros2 launch fast_lio mapping.launch.py config_file:=gazebo_velodyne.yaml use_sim_time:=true # odometry
ros2 launch gicp_localization localization.launch.py                                        # localization
ros2 launch robot_navigation navigation.launch.py                                           # nav2
```

## Robot, sensors & topology (new stack)

- Differential drive: box body ≈ 0.75×0.55×0.40 m, two drive wheels (`wheel_separation=0.55`, `wheel_radius=0.12`), two caster balls. Root link = `base_footprint`. Velodyne + IMU are **co-located** at `base_link + z=0.236` (lidar↔imu extrinsic is zero — matches both mapping and localization configs).
- `diff_drive_controller` is named **`base_controller`**; on Humble it subscribes **`TwistStamped`** (publishing plain `Twist` is silently ignored — the #1 "robot won't move" cause). `/cmd_vel` is remapped to `base_controller`. Wheel odom publishes `/base_controller/odom` (real twist) but **`enable_odom_tf:false`** — `odom→base_footprint` TF is owned by SLAM, not the wheels.
- Sim sensors are **Gz-native** (`gpu_lidar` VLP-16-style 16-line organized cloud ~10 Hz on `/lidar/points`; `imu` 200 Hz). `config/bridge.yaml` bridges `/clock`, `/lidar/points`, and `/imu→/imu_plugin/out`. `lidar_pointcloud_adapter` converts the organized cloud to a Velodyne-style `/points_raw` (frame `velodyne`, fields incl. `ring`+synthesized `time`).
- `controller_manager` is provided by the `gz_ros2_control` plugin inside the URDF — there is **no standalone `ros2_control_node`** in sim.
- Real device packages live under `core/robot/drivers/`. `can_driver_web_control` is a manual acceptance tool only and must never be included by production `system_bringup`. The current `robot_hardware` implementation is still a loopback placeholder; `use_mock_hardware:=false` is not real-hardware-ready until it is adapted to the 8030D driver.

## gicp_localization architecture (localization stage)

- **Single node, two timers** under a `MultiThreadedExecutor` + two callback groups: a slow timer (`localization_freq`, 0.5 Hz, MutuallyExclusive) runs GICP; a fast timer (`tf_pub_freq`, 50 Hz, Reentrant) broadcasts `map→camera_init` and publishes `/localization`. One `std::mutex` guards shared state; the GICP callback copies inputs under the lock, runs `align()` **without** the lock, then re-locks to commit — so the heavy registration never blocks the 50 Hz TF.
- Registration uses **koide3/small_gicp** (`preprocess_points` builds the target KdTree+covariances once; `align(target, source, target_tree, seed)` per scan). Because the source `/cloud_registered` is already in the `camera_init` frame, `result.T_target_source` **is** `T_map→camera_init` directly. The seed is the previous correction, or `/initialpose` (RViz "2D Pose Estimate") on demand.
- **Acceptance gate is fitness-only** (`fitness = num_inliers/num_source ≥ fitness_threshold`, default 0.9); `converged` is reported in diagnostics but does **not** gate (small_gicp often reports `converged=false` on dense scans even when correct). On reject, the last good correction is held.
- **Diagnostics are a build-time switch, with zero `#ifdef` in node logic.** `option(GICP_DIAGNOSTICS)` in CMake selects which `.cpp` is compiled: `diagnostics_real.cpp` (generates `GicpDiagnostics.msg`, publishes `~/diagnostics`) or `diagnostics_null.cpp` (no-op). The node always calls `diag_->report(m)`. `gicp_localization_core` and `_diag` are **static** libs linked into the executable (don't reintroduce shared-lib install targets — that caused a missing-`.so` bug).
- small_gicp has **no file IO**; the PCD prior map is loaded with PCL (`loadPriorMapPcd`) and re-published latched on `~/prior_map` (frame `map`) for RViz.
- The package was `git mv`'d intact from the old stack (architecture is clean) — reused, not rewritten.

## Nav2 architecture (navigation stage)

- Global planner **Smac Hybrid-A\*** (kinematics portable via `minimum_turning_radius` + `motion_model_for_search` — diff/Ackermann by params, not plugin swap); local controller **MPPI** (`motion_model: DiffDrive`). **Hard constraint: `1/controller_frequency ≤ model_dt`** (else controller_server fails to configure with "Controller period more then model dt").
- Double-frame: global costmap = `map` (GICP jumps OK, replans); local costmap + behavior_server = `camera_init` (FAST-LIO continuous, MPPI needs smooth high-rate). No AMCL — GICP provides `map→camera_init`.
- `body→base_footprint` static weld: **identity rotation** (NOT pitch=π), `z = -0.556`. cmd_vel: Nav2 publishes `Twist` on `/cmd_vel_nav` → `twist_stamper` → `/cmd_vel` (TwistStamped). `odom_topic = /base_controller/odom`. Local voxel layer source = `/cloud_registered` (sensor_frame `body`, `origin_z=-1.0`).

## Simulation gotchas (already known — don't rediscover)

- **Gz sim clouds have no per-point `time` natively.** `lidar_pointcloud_adapter` synthesizes per-point `time` (by column: `(i%width)/width*scan_period`) so FAST-LIO sets `given_offset_time=true` and deskews. This is **synthetic** azimuth time (Gz is an instantaneous snapshot, no real intra-frame distortion); if rotation tests smear, the fix is a constant `time` in the adapter (making deskew identity), not a FAST-LIO patch. LIO-SAM auto-disables deskew on time-less input; its saved map is unaffected.
- **The sim does not drift.** In this feature-rich factory, FAST-LIO is fully constrained by LiDAR scan-matching, so IMU noise (even extreme) induces no drift — GICP's drift-correction benefit is not demonstrable here. The factory's **90° rotational pseudo-symmetry** creates a strong false GICP minimum at `fitness≈0.8`; the default `fitness_threshold=0.9` rejects it. GICP is local — large offsets / ~90° won't auto-recover; bootstrap needs a correct `/initialpose` near the robot's actual map pose.
- **`enable_odom_tf:false`** — during mapping, `odom→base_footprint` is owned by LIO-SAM; in the localization/nav stack the wheel odom is a `/base_controller/odom` topic only (its twist is what Nav2/MPPI use, since FAST-LIO `/Odometry` twist is always zero).
- The old Gazebo-Classic robot model's pathological vertical jitter (ill-conditioned `robot.sdf`) does **not** apply to the new model — it is a clean ros2_control rewrite with primitive collisions. Don't carry over that gotcha.
