# CLAUDE.md

Repository guidance for coding agents and maintainers.

## Current execution environment

- The authoritative Git checkout is edited on Windows. The WSL2 Ubuntu 22.04 distribution named
  `slam` is a CPU-only build/test executor; do not install or launch Gazebo there and do not treat it
  as a full runtime-verification host.
- `/home/lxx/xxsim` is a Git checkout whose local `windows` remote points to the Windows repository.
  Before accepting WSL evidence, require a clean WSL worktree, fetch `windows`, fast-forward the
  current branch, and verify that both checkouts resolve to the same commit.
- ROS 2 Humble is installed under `/opt/ros/humble`; source it explicitly in non-interactive shells.
- The colcon workspace root is `core/`, not the repository root and not a nested `src/` directory.
- Python is the system `/usr/bin/python3` (3.10). Do not introduce a repository `.venv` or make CMake
  cache a virtual-environment interpreter.

Typical WSL command prefix:

```bash
cd /home/lxx/xxsim
git status --short                 # must be empty
git fetch windows
git merge --ff-only '@{u}'

cd /home/lxx/xxsim/core
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Build and test

```bash
cd core
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
colcon test
colcon test-result --all --verbose
```

`colcon_defaults.yaml` skips vendor/upstream packages during the default test run. Test them
explicitly only on a compatible platform; these package exclusions are not skipped test cases. The
latest verified WSL baseline is 768 tests with zero errors, failures, and skips.

The Web asset tests require a `node` executable. This machine currently exposes Node 22.21.0 to WSL
through `/home/lxx/.local/bin/node`, linked to the existing Windows Node installation. Verify
`command -v node` and `node --version` before accepting a run; missing Node is an environment failure,
not an allowed reason to skip those tests.

This workspace uses the existing copy-install build tree. Do not switch it to `--symlink-install`
without deliberately recreating the build/install trees; mixing modes leaves incompatible
`ament_cmake_python` directories and symlinks.

`xacro` is a runtime/test dependency. A successful build does not prove it is installed; verify
`command -v xacro` when robot description or launch tests fail before inspecting source code.

WSL may log `Could not enable FIFO RT scheduling policy: Operation not permitted`. This is an
expected warning in the current test environment, not a repository defect, when controllers and
tests otherwise pass.

## Runtime entry and configuration ownership

The following describes a deployment or GPU-capable runtime host. It is not an instruction to launch
the full stack in the current CPU-only WSL build/test environment.

The formal entry is:

```bash
ros2 launch system_bringup bringup.launch.py
```

`core/bringup/system_bringup/config/bringup.yaml` selects `platform: sim|real` and
`mode: mapping|navigation`. The runtime compiler combines the selected Profile with centralized
templates and writes generated YAML to a unique `/tmp/system_bringup-runtime-*` directory.

- `bringup.yaml`: run selection and resource paths.
- `config/profiles/{sim,real}.yaml`: platform facts, geometry, sensors, backends, and shared limits.
- `config/templates/*.yaml`: complete native configs owned by system_bringup, including FAST-LIO.
- Formal FAST-LIO parameters are rendered as `fast_lio.generated.yaml` and passed by absolute path
  through the manifest. GICP and LIO-SAM remain in their owning upstream/package configuration files.
- Generated `/tmp` files and effective reports are never source files and never enter Git.

The source config tree selected by `bringup.yaml` is the only active template source. Installed
templates are packaging/static-acceptance evidence only. Production runtime consistency checks
source/install byte freshness only for the launch/Python runtime files ROS actually loads.

Do not pass ad-hoc overrides around the compiler. Direct module launches are diagnostic paths and
must not be documented as equivalent to the formal full-stack entry.

## Repository structure

Six modules live under `core/`:

- `robot`: URDF, ros2_control, command gate, 8030D and Vanjee drivers.
- `simulation`: Gazebo Harmonic world, bridge, and lidar point-cloud adapter.
- `mapping`: LIO-SAM clone plus tracked patch and map-save helper.
- `localization`: FAST-LIO clone plus tracked patch, small_gicp, and in-repo GICP localization.
- `navigation`: PCD projection, Nav2 launch/config, and Twist conversion.
- `bringup`: runtime compiler, consistency/sensor gates, Web UI, and full-stack orchestration.

Upstream FAST-LIO, LIO-SAM, and small_gicp clones are gitignored and pinned by their module docs.
Local upstream changes belong in tracked patch files. Vendor documentation and SDK changelogs are
upstream records; do not rewrite them as project documentation.

## Runtime contracts

- Sensor topics: `/points_raw`, `/imu/data`; frames: `velodyne`, `imu_link`.
- Point fields: `x/y/z/intensity/ring/time`.
- Localization TF: `map -> camera_init -> body -> base_footprint -> base_link`.
- Wheel odometry topic: `/base_controller/odom`; wheel odom TF is disabled.
- Full bringup control: Nav2 `/cmd_vel_auto`, Web `/cmd_vel_manual`, `cmd_vel_gate` as the only
  `/cmd_vel` publisher.
- Sim uses generated lidar-adapter config; real uses generated Vanjee config. Both pass through the
  shared sensor contract gate before SLAM starts.

## Process and memory safety

Gazebo, FAST-LIO, GICP, Nav2, RViz, and rendering can consume several gigabytes. Run only one full
stack at a time and check memory before launch.

- Do not orphan `ros2 launch` children by killing only the parent shell.
- Prefer a dedicated process group and terminate the whole group.
- Do not use broad `pkill -f` patterns; they can match the cleanup shell and leave children behind.
- Before another run, verify Gazebo, controller manager, bridge, adapter, SLAM, and Nav2 processes are
  gone and memory has been reclaimed.

## Git and concurrent-session safety

- Never push, create/merge a PR, or otherwise change a remote without explicit human approval.
- Before broad edits, capture HEAD and check `git status`. If another session moves HEAD or changes an
  overlapping file, stop and reassess instead of overwriting it.
- Preserve unrelated user changes. Do not switch branches in a shared working directory.
- Keep Superpowers specs, plans, caches, and logs out of Git.

## Documentation ownership

- Root `README.md`: setup, architecture, build/test, supported full-stack workflow, module status,
  unfinished product direction, and milestones.
- Module README: module-specific integration, diagnostics, acceptance, and troubleshooting.
- `PROGRESS.md`: active profile-migration record; do not use it as a general runbook.
