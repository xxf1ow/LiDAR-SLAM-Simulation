import os
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[4]
STICKY_TELEOP = (
    ROOT / "core/simulation/robot_gz_bringup/scripts/sticky_teleop.py"
)
LEGACY_WEB_CONTROL = (
    ROOT / "core/robot/drivers/chassis_8030d/can_driver_web_control"
)
ROBOT_GZ_CMAKE = ROOT / "core/simulation/robot_gz_bringup/CMakeLists.txt"
ROBOT_GZ_PACKAGE = ROOT / "core/simulation/robot_gz_bringup/package.xml"
ROBOT_WEB_UI = ROOT / "core/bringup/robot_web_ui/robot_web_ui"
ACTIVE_BRINGUP_DOCS_AND_LAUNCH = (
    ROOT / "CLAUDE.md",
    ROOT / "core/simulation/robot_gz_bringup/README.md",
    ROOT / "core/bringup/system_bringup/README.md",
    ROOT / "core/bringup/system_bringup/launch",
    ROOT / "core/robot/README.md",
    ROOT / "core/robot/drivers/README.md",
    ROOT / "core/robot/drivers/chassis_8030d/README.md",
    ROOT / "core/localization/README.md",
    ROOT / "core/mapping/README.md",
    ROOT / "core/navigation/README.md",
)


def _tracked_text(*paths: Path) -> str:
    relative_paths = [path.relative_to(ROOT).as_posix() for path in paths]
    try:
        result = subprocess.run(
            ["git", "ls-files", "-z", "--", *relative_paths],
            cwd=ROOT,
            check=True,
            capture_output=True,
            text=True,
            encoding="utf-8",
        )
        files = [ROOT / path for path in result.stdout.split("\0") if path]
    except (FileNotFoundError, subprocess.CalledProcessError):
        files = []
        for path in paths:
            if path.is_file():
                files.append(path)
                continue
            for root, directories, filenames in os.walk(path):
                directories[:] = [
                    name for name in directories
                    if name not in {
                        ".git", ".pytest_cache", "__pycache__", ".superpowers"
                    }
                ]
                files.extend(Path(root) / filename for filename in filenames)

    sources = []
    for source in dict.fromkeys(files):
        normalized = source.relative_to(ROOT).as_posix()
        if "docs/superpowers/" in normalized or "/.superpowers/" in normalized:
            continue
        if source.is_file():
            try:
                sources.append(source.read_text(encoding="utf-8"))
            except UnicodeDecodeError:
                continue
    return "\n".join(sources)


def test_tracked_text_falls_back_to_explicit_paths_when_git_is_unavailable(
    monkeypatch,
):
    def unavailable(*args, **kwargs):
        raise subprocess.CalledProcessError(128, args[0])

    monkeypatch.setattr(subprocess, "run", unavailable)

    source = _tracked_text(ROBOT_WEB_UI)

    assert "/cmd_vel_manual" in source


@pytest.mark.parametrize(
    "legacy_path",
    (STICKY_TELEOP, LEGACY_WEB_CONTROL),
)
def test_legacy_control_path_is_absent(legacy_path):
    assert not legacy_path.exists()


def test_robot_gz_bringup_does_not_install_or_depend_on_sticky_teleop():
    robot_gz_cmake = ROBOT_GZ_CMAKE.read_text(encoding="utf-8")
    robot_gz_package = ROBOT_GZ_PACKAGE.read_text(encoding="utf-8")

    assert "scripts/sticky_teleop.py" not in robot_gz_cmake
    assert "<exec_depend>rclpy</exec_depend>" not in robot_gz_package
    assert "<exec_depend>geometry_msgs</exec_depend>" not in robot_gz_package


def test_active_bringup_docs_and_launch_do_not_reference_legacy_control():
    active_bringup_docs_and_launch = _tracked_text(
        *ACTIVE_BRINGUP_DOCS_AND_LAUNCH
    )

    assert "sticky_teleop" not in active_bringup_docs_and_launch
    assert "can_driver_web_control" not in active_bringup_docs_and_launch


def test_robot_web_ui_does_not_bypass_the_controller_path():
    robot_web_ui_source = _tracked_text(ROBOT_WEB_UI)

    assert "/motor_speed" not in robot_web_ui_source
    assert "/driver" not in robot_web_ui_source
