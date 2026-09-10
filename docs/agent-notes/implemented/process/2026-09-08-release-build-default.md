# Agent Note: Release build default

Status: implemented

## Problem

The deployed ROS stack shares limited CPU with SLAM and navigation. Leaving CMake build type selection to each invocation can silently produce unoptimized binaries and invalidate runtime CPU comparisons.

## Decision

The workspace `core/colcon_defaults.yaml` passes `-DCMAKE_BUILD_TYPE=Release` to every CMake package by default. Callers that need assertions or debug symbols explicitly override the setting with `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug`. Python packages are unaffected.

## Alternatives considered

**Set Release only for Web UI and localization packages.** This leaves other deployed CMake nodes dependent on the caller's unstated build environment and makes whole-stack CPU measurements inconsistent.


**Require every build command to pass the build type.** Repetition makes omission likely on developer and deployment machines; a checked-in default makes the normal build deterministic while retaining an explicit override.

## Consequences

Normal workspace builds produce optimized CMake binaries without extra command-line flags, and CPU measurements share one build baseline. Debug builds require an explicit override, and a build directory should not mix configurations unintentionally.
