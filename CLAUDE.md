# CLAUDE.md

Repository guidance for coding agents and maintainers.

## Authoritative documentation

- [`docs/architecture.md`](docs/architecture.md) owns current composition, runtime flow, module boundaries, and extension points.
- [`docs/development.md`](docs/development.md) owns contributor setup, build workflow, configuration edits, and documentation checks.
- [`docs/testing.md`](docs/testing.md) owns test tiers, commands, selection rules, and acceptance evidence.
- Module READMEs own package-level contracts, diagnostics, limitations, and visible effects.
- `docs/agent-notes/` owns durable rationale, alternatives, consequences, and active proposals.

Update the owning document with every changed fact. Do not restate current project facts or command catalogs in this file.

## Environment constraints

- Windows is the authoritative Git checkout. The WSL2 Ubuntu 22.04 distribution `slam` is a CPU-only build/test executor, not a Gazebo or full-runtime host.
- Before accepting WSL evidence, require a clean WSL worktree, fetch its local `windows` remote, fast-forward the branch, and verify both checkouts resolve to the same commit.
- The colcon workspace root is `core/`; ROS 2 Humble is under `/opt/ros/humble`; system Python is `/usr/bin/python3` 3.10. Do not create a repository `.venv`.
- Keep the existing copy-install build mode. Do not mix it with `--symlink-install` without deliberately recreating `core/build/` and `core/install/`.

## Repository constraints

- The formal runtime entry is `ros2 launch system_bringup bringup.launch.py`. Direct module launches are diagnostic paths and must not be described as equivalent full-stack entries.
- Upstream FAST-LIO, LIO-SAM, and small_gicp clones remain ignored and pinned by their module READMEs. Project changes to those sources use tracked patch files.
- Vendor documentation and SDK changelogs are upstream records. Do not rewrite them as project documentation.
- Generated runtime YAML, effective reports, maps, bags, logs, build products, and Superpowers artifacts never enter Git.
- One-time plans, migration logs, acceptance scripts, reports, and implementation progress do not enter the maintained documentation corpus. Keep reusable current commands in `docs/development.md`, `docs/testing.md`, or the owning module README; keep run evidence outside the repository.
- Every non-trivial change updates an owning Agent Note. Use the documentation authorities above for current facts and procedures.

## Runtime and process safety

- Run only one full stack at a time. Gazebo, SLAM, Nav2, RViz, and rendering can consume several gigabytes.
- Terminate the whole launch process group; do not kill only the parent shell or use broad `pkill -f` patterns.
- Before another run, verify Gazebo, controllers, bridges, adapters, SLAM, and Nav2 processes are gone and memory is reclaimed.
- Web stop and command timeouts do not disable hardware and never replace a physical emergency stop or power cutoff.

## Git safety

- Never push, create or merge a PR, or otherwise change a remote without explicit human approval in the current conversation.
- Before broad edits, record HEAD and inspect `git status`. Stop if another session changes HEAD or an overlapping file.
- Preserve unrelated user changes and do not switch branches in a shared working directory.
