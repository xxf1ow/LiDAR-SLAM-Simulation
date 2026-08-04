"""跨模块魔法值一致性检查(纯 Python,无 ROS 依赖,本机/构建机皆可跑)。

解析已跟踪源文件；仿真检查既有默认值，真机从 bringup.yaml 单一源生成后检查。
- run(repo_root) -> list[str]   失败描述(空=全过)
- main()        -> int          打印失败、返回退出码(供启动闸门/CLI)
- find_repo_root()              从 __file__ 上溯定位仓库根(pytest 本机用)

契约类(帧/话题)不纳入。patch 文件只信任 '+'(项目改值)行。
"""
import argparse
import ast
import math
import os
from pathlib import Path
import re
import sys
import tempfile

try:
    import yaml
except ImportError:  # pyyaml 是硬依赖
    yaml = None

# ---- 已跟踪源文件(仓库相对路径) ----
F_MACRO = "core/robot/robot_description/urdf/robot_macro.urdf.xacro"
F_ROBOT_XACRO = "core/robot/robot_description/urdf/robot.urdf.xacro"
F_GAZEBO = "core/robot/robot_description/gazebo/robot.gazebo.xacro"
F_CONTROLLERS = "core/robot/robot_bringup/config/robot_controllers.yaml"
F_NAV_PARAMS = "core/navigation/robot_navigation/config/nav2_params.yaml"
F_NAV_PARAMS_REAL = "core/navigation/robot_navigation/config/nav2_params_real.yaml"
F_NAV_LAUNCH = "core/navigation/robot_navigation/launch/navigation.launch.py"
F_GZ_LAUNCH = "core/simulation/robot_gz_bringup/launch/robot_gz.launch.py"
F_FASTLIO_PATCH = "core/localization/fast-lio2.patch"
F_LIOSAM_PATCH = "core/mapping/lio-sam.patch"
F_VANJEE_PARAMS = (
    "core/robot/drivers/lidar_vanjee_722/"
    "vanjee_lidar_ros/config/vanjee_722.yaml"
)

FASTLIO_CONFIG = {
    "sim": "config/gazebo_velodyne.yaml",
    "real": "config/vanjee_722.yaml",
}
LIOSAM_CONFIG = {
    "sim": "config/params.yaml",
    "real": "config/params_real.yaml",
}
_MARKER = os.path.join("core", "bringup", "system_bringup")


# ---- 仓库根定位 ----
def find_repo_root(start=None):
    here = os.path.abspath(start or __file__)
    d = here if os.path.isdir(here) else os.path.dirname(here)
    while True:
        if os.path.isdir(os.path.join(d, _MARKER)) or os.path.isdir(os.path.join(d, ".git")):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            raise RuntimeError("找不到仓库根(向上未见 %s 或 .git);start=%s" % (_MARKER, start))
        d = parent


def load_bringup_config(repo_root=None):
    """读源码 bringup config(launch 运行时调,不经 install —— 改 config 不用 rebuild)。"""
    repo_root = repo_root or find_repo_root()
    cfg_path = os.path.join(repo_root, "core", "bringup", "system_bringup", "config", "bringup.yaml")
    with open(cfg_path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def require_runtime_config_file(path, component):
    """在启动外部 SLAM 节点前确认所选安装态配置实际存在。"""
    resolved = os.fspath(path)
    if not os.path.isfile(resolved):
        raise RuntimeError(
            "%s 运行时配置不存在: %s；请 apply 对应 patch 并 rebuild。"
            % (component, resolved)
        )
    return resolved


def derive_real_geometry(config):
    """从 bringup.yaml 的实测值派生各模块需要的真机几何。"""
    measured = config["real_geometry"]
    body = measured["body"]
    wheel = measured["drive_wheel"]
    lidar = measured["lidar"]

    body_length = float(body["length"])
    body_width = float(body["width"])
    body_height = float(body["height"])
    ground_clearance = float(body["ground_clearance"])
    base_link_height = ground_clearance + body_height / 2.0
    wheel_diameter = float(wheel["diameter"])
    wheel_width = float(wheel["width"])
    wheel_separation = float(wheel["separation"])
    lidar_x = float(lidar["x"])
    lidar_y = float(lidar["y"])
    lidar_z = float(lidar["z"])
    roll = float(lidar["roll"])
    pitch = float(lidar["pitch"])
    yaw = float(lidar["yaw"])

    values = {
        "body.length": body_length,
        "body.width": body_width,
        "body.height": body_height,
        "body.ground_clearance": ground_clearance,
        "drive_wheel.diameter": wheel_diameter,
        "drive_wheel.width": wheel_width,
        "drive_wheel.separation": wheel_separation,
        "lidar.x": lidar_x,
        "lidar.y": lidar_y,
        "lidar.z": lidar_z,
        "lidar.roll": roll,
        "lidar.pitch": pitch,
        "lidar.yaw": yaw,
    }
    for name, value in values.items():
        if not math.isfinite(value):
            raise ValueError("real_geometry.%s 必须是有限数。" % name)
    for name in (
        "body.length", "body.width", "body.height",
        "drive_wheel.diameter", "drive_wheel.width",
        "drive_wheel.separation", "lidar.z",
    ):
        if values[name] <= 0.0:
            raise ValueError("real_geometry.%s 必须大于 0。" % name)
    if ground_clearance < 0.0:
        raise ValueError("real_geometry.body.ground_clearance 不能小于 0。")
    if wheel_separation + wheel_width > body_width + 1e-12:
        raise ValueError("real_geometry 轮子外缘宽度超过 body.width。")
    if any(abs(angle) > 1e-12 for angle in (roll, pitch, yaw)):
        raise ValueError("real_geometry.lidar 当前仅支持零安装角；非零角需同时实现刚体逆变换。")

    return {
        "body": {
            "length": body_length,
            "width": body_width,
            "height": body_height,
            "base_link_height": base_link_height,
        },
        "drive_wheel": {
            "radius": wheel_diameter / 2.0,
            "width": wheel_width,
            "separation": wheel_separation,
        },
        "sensor": {
            "x": lidar_x,
            "y": lidar_y,
            "z": lidar_z - base_link_height,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
        },
        "body_to_base_footprint": {
            "x": -lidar_x,
            "y": -lidar_y,
            "z": -lidar_z,
            "roll": -roll,
            "pitch": -pitch,
            "yaw": -yaw,
        },
        "footprint": [
            [body_length / 2.0, body_width / 2.0],
            [body_length / 2.0, -body_width / 2.0],
            [-body_length / 2.0, -body_width / 2.0],
            [-body_length / 2.0, body_width / 2.0],
        ],
    }


def _read(repo_root, relpath):
    with open(os.path.join(repo_root, *relpath.split("/")), encoding="utf-8") as f:
        return f.read()


def _yaml(text):
    return yaml.safe_load(text)


# ---- 解析器 ----
def _xacro_props(text):
    """字面量 xacro property -> float dict(跳过 ${...} 表达式 value)。"""
    out = {}
    for name, val in re.findall(r'<xacro:property\s+name="([^"]+)"\s+value="([^"]+)"\s*/>', text):
        try:
            out[name] = float(val)
        except ValueError:
            pass
    return out


def _xacro_args(text):
    """字面量 xacro arg default -> float dict。"""
    return {
        name: float(value)
        for name, value in re.findall(
            r'<xacro:arg\s+name="([^"]+)"\s+default="([-\d.]+)"\s*/>', text
        )
    }


def _xacro_joint_origin_xyz(text, joint_substr):
    """含 joint_substr 的 <joint> 块内第一个 <origin xyz="..."> 的 xyz 串。"""
    m = re.search(r'<joint[^>]*name="[^"]*' + re.escape(joint_substr) + r'[^"]*"[^>]*>(.*?)</joint>',
                  text, re.DOTALL)
    if not m:
        return None
    mo = re.search(r'<origin\s+xyz="([^"]+)"', m.group(1))
    return mo.group(1).strip() if mo else None


def _patch_file_section(text, relative_path):
    """从 unified diff 中取出指定文件的完整 diff 段。"""
    marker = "diff --git a/%s b/%s" % (relative_path, relative_path)
    start = text.find(marker)
    if start < 0:
        raise ValueError("patch 中找不到文件: %s" % relative_path)
    return text[start:].split("\ndiff --git ", 1)[0]


def _patch_added_file(text, relative_path):
    """从 unified diff 中重建指定新增文件；找不到或不是新增文件时明确失败。"""
    section = _patch_file_section(text, relative_path)
    if "--- /dev/null" not in section:
        raise ValueError("patch 目标不是新增文件: %s" % relative_path)
    return "\n".join(
        line[1:]
        for line in section.splitlines()
        if line.startswith("+") and not line.startswith("+++")
    )


def _patch_added_value(text, relative_path, key):
    """在指定文件 diff 的 '+' 行里读取 `key: value`。"""
    section = _patch_file_section(text, relative_path)
    for line in section.splitlines():
        if line.startswith("+++") or not line.startswith("+"):
            continue
        match = re.match(
            r"\s*" + re.escape(key) + r"\s*:\s*(.+?)\s*(#.*)?$",
            line[1:],
        )
        if match:
            return match.group(1).strip().strip('"')
    return None


def _gazebo_lidar(text):
    """gpu_lidar 传感器块 -> dict(h_samples,v_samples,update_rate,range_min)。"""
    blk = re.search(r'<sensor name="gpu_lidar".*?</sensor>', text, re.DOTALL).group(0)
    h = re.search(r'<horizontal>.*?<samples>(\d+)</samples>', blk, re.DOTALL).group(1)
    v = re.search(r'<vertical>.*?<samples>(\d+)</samples>', blk, re.DOTALL).group(1)
    rate = re.search(r'<update_rate>(\d+)</update_rate>', blk).group(1)
    rmin = re.search(r'<range>.*?<min>([\d.]+)</min>', blk, re.DOTALL).group(1)
    return {"h_samples": int(h), "v_samples": int(v),
            "update_rate": int(rate), "range_min": float(rmin)}


def _launch_floats(text, names):
    """从 launch py 文本取 `NAME = <float>` 字面量。"""
    out = {}
    for n in names:
        m = re.search(re.escape(n) + r'\s*=\s*([-\d.]+)', text)
        if m:
            out[n] = float(m.group(1))
    return out


def _adapter_scan_period(text):
    return float(re.search(r'"scan_period"\s*:\s*([\d.]+)', text).group(1))


def _parse_footprint(s):
    return [(float(x), float(y)) for x, y in ast.literal_eval(s)]


def _format_footprint(points):
    return "[ " + ", ".join("[%.3f, %.3f]" % (x, y) for x, y in points) + " ]"


def build_real_runtime_configs(repo_root, config):
    """以既有 YAML 为模板，只注入 bringup 中的真机几何派生值。"""
    geometry = derive_real_geometry(config)
    controllers = _yaml(_read(repo_root, F_CONTROLLERS))
    controller = controllers["base_controller"]["ros__parameters"]
    controller["wheel_radius"] = geometry["drive_wheel"]["radius"]
    controller["wheel_separation"] = geometry["drive_wheel"]["separation"]

    nav2 = _yaml(_read(repo_root, F_NAV_PARAMS_REAL))
    footprint = _format_footprint(geometry["footprint"])
    for scope in ("global_costmap", "local_costmap"):
        nav2[scope][scope]["ros__parameters"]["footprint"] = footprint

    return {
        "geometry": geometry,
        "controllers": controllers,
        "nav2": nav2,
    }


def real_geometry_launch_arguments(geometry):
    """把派生几何转换为 robot/navigation launch 的字符串参数。"""
    def as_text(value):
        return str(0.0 if abs(value) < 1e-12 else value)

    body = geometry["body"]
    wheel = geometry["drive_wheel"]
    sensor = geometry["sensor"]
    weld = geometry["body_to_base_footprint"]
    return {
        "robot": {
            "base_length": as_text(body["length"]),
            "base_width": as_text(body["width"]),
            "base_height": as_text(body["height"]),
            "base_link_height": as_text(body["base_link_height"]),
            "wheel_radius": as_text(wheel["radius"]),
            "wheel_width": as_text(wheel["width"]),
            "wheel_separation": as_text(wheel["separation"]),
            "sensor_x": as_text(sensor["x"]),
            "sensor_y": as_text(sensor["y"]),
            "sensor_z": as_text(sensor["z"]),
            "sensor_roll": as_text(sensor["roll"]),
            "sensor_pitch": as_text(sensor["pitch"]),
            "sensor_yaw": as_text(sensor["yaw"]),
        },
        "navigation": {
            "weld_x": as_text(weld["x"]),
            "weld_y": as_text(weld["y"]),
            "weld_z": as_text(weld["z"]),
            "weld_roll": as_text(weld["roll"]),
            "weld_pitch": as_text(weld["pitch"]),
            "weld_yaw": as_text(weld["yaw"]),
        },
    }


def write_real_runtime_configs(repo_root, config, output_dir=None):
    """把派生 YAML 写到临时目录；不改源码，也不改 install。"""
    runtime = build_real_runtime_configs(repo_root, config)
    if output_dir is None:
        directory = Path(tempfile.mkdtemp(prefix="system_bringup-"))
    else:
        directory = Path(output_dir)
        directory.mkdir(parents=True, exist_ok=True)
    paths = {
        "controllers": directory / "robot_controllers_real.generated.yaml",
        "nav2": directory / "nav2_params_real.generated.yaml",
    }
    for name, path in paths.items():
        path.write_text(
            yaml.safe_dump(runtime[name], sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )
    return paths


def check_geometry(repo_root, platform="sim"):
    """G1–G5:几何派生值在 xacro / controllers / nav2 / launch 间自洽。"""
    if platform not in ("sim", "real"):
        return ["未知 platform=%r(应为 sim|real)。" % platform]
    fails = []
    macro = _read(repo_root, F_MACRO)
    if platform == "real":
        runtime = build_real_runtime_configs(
            repo_root, load_bringup_config(repo_root)
        )
        geometry = runtime["geometry"]
        base_l = geometry["body"]["length"]
        base_w = geometry["body"]["width"]
        base_h = geometry["body"]["height"]
        wheel_r = geometry["drive_wheel"]["radius"]
        wheel_separation = geometry["drive_wheel"]["separation"]
        nav = runtime["nav2"]
        ctrl = runtime["controllers"]
    else:
        defaults = _xacro_args(_read(repo_root, F_ROBOT_XACRO))
        base_l = defaults["base_length"]
        base_w = defaults["base_width"]
        base_h = defaults["base_height"]
        wheel_r = defaults["wheel_radius"]
        wheel_separation = defaults["wheel_separation"]
        nav = _yaml(_read(repo_root, F_NAV_PARAMS))
        ctrl = _yaml(_read(repo_root, F_CONTROLLERS))
    cp = ctrl["base_controller"]["ros__parameters"]

    # G1 footprint(global + local 两处)半长/半宽 == 车体半长/半宽
    for scope in ("global_costmap", "local_costmap"):
        fp = nav[scope][scope]["ros__parameters"]["footprint"]
        pts = _parse_footprint(fp)
        hx = max(abs(x) for x, _ in pts)
        hy = max(abs(y) for _, y in pts)
        if abs(hx - base_l / 2) > 1e-6:
            fails.append("[G1] %s footprint 半长 %.3f != base_length/2 %.3f(源 xacro)。改 nav2_params.yaml 或核对 xacro。"
                         % (scope, hx, base_l / 2))
        if abs(hy - base_w / 2) > 1e-6:
            fails.append("[G1] %s footprint 半宽 %.3f != base_width/2 %.3f。" % (scope, hy, base_w / 2))

    # G2 controllers 轮参 == 当前 platform 的几何源
    if abs(cp["wheel_radius"] - wheel_r) > 1e-9:
        fails.append("[G2] wheel_radius 不一致: geometry=%.4f vs controllers=%.4f。"
                     % (wheel_r, cp["wheel_radius"]))
    if abs(cp["wheel_separation"] - wheel_separation) > 1e-9:
        fails.append("[G2] wheel_separation controllers=%.4f != geometry %.4f。" %
                     (cp["wheel_separation"], wheel_separation))

    # G3 仿真 launch 默认焊接继续与仿真 xacro 默认值一致；真机由同一份派生值下发。
    if platform == "sim":
        lidar_h = _xacro_props(macro)["lidar_height"]
        lc = _launch_floats(
            _read(repo_root, F_NAV_LAUNCH),
            ["_BASE_HEIGHT", "_WHEEL_RADIUS", "_LIDAR_HEIGHT"],
        )
        for cname, xval in [
            ("_BASE_HEIGHT", base_h),
            ("_WHEEL_RADIUS", wheel_r),
            ("_LIDAR_HEIGHT", lidar_h),
        ]:
            if cname not in lc:
                fails.append("[G3] navigation.launch.py 缺仿真几何常量 %s。" % cname)
            elif abs(lc[cname] - xval) > 1e-9:
                fails.append("[G3] navigation.launch.py %s=%.4f != 仿真 xacro %.4f。"
                             % (cname, lc[cname], xval))

    # G4 共位 -> 外参零
    vxyz = _xacro_joint_origin_xyz(macro, "velodyne_joint")
    ixyz = _xacro_joint_origin_xyz(macro, "imu_joint")
    if vxyz != ixyz:
        fails.append("[G4] velodyne_joint/imu_joint origin 不同(应共位): '%s' vs '%s'。" % (vxyz, ixyz))
    fastlio = _yaml(_patch_added_file(
        _read(repo_root, F_FASTLIO_PATCH),
        FASTLIO_CONFIG[platform],
    ))["/**"]["ros__parameters"]
    flm = fastlio["mapping"]
    if [float(v) for v in flm["extrinsic_T"]] != [0.0, 0.0, 0.0]:
        fails.append("[G4] fast-lio extrinsic_T 非零: %s(velodyne/imu 共位应为 [0,0,0])。" % flm["extrinsic_T"])
    if [float(v) for v in flm["extrinsic_R"]] != [1, 0, 0, 0, 1, 0, 0, 0, 1]:
        fails.append("[G4] fast-lio extrinsic_R 非单位阵: %s。" % flm["extrinsic_R"])

    # G5 nav2 限速 <= 底盘限速
    fpp = nav["controller_server"]["ros__parameters"]["FollowPath"]
    if fpp["vx_max"] > cp["linear.x.max_velocity"] + 1e-9:
        fails.append("[G5] nav2 vx_max %.2f > 底盘 linear.x.max_velocity %.2f。" % (fpp["vx_max"], cp["linear.x.max_velocity"]))
    if fpp["wz_max"] > cp["angular.z.max_velocity"] + 1e-9:
        fails.append("[G5] nav2 wz_max %.2f > 底盘 angular.z.max_velocity %.2f。" % (fpp["wz_max"], cp["angular.z.max_velocity"]))

    return fails


def check_lidar(repo_root, platform="sim"):
    """sim 检查 L1–L4，real 检查 R1–R9。"""
    if platform not in ("sim", "real"):
        return ["未知 platform=%r(应为 sim|real)。" % platform]
    fails = []
    fastlio = _yaml(_patch_added_file(
        _read(repo_root, F_FASTLIO_PATCH),
        FASTLIO_CONFIG[platform],
    ))["/**"]["ros__parameters"]

    if platform == "real":
        lio = _yaml(_patch_added_file(
            _read(repo_root, F_LIOSAM_PATCH),
            LIOSAM_CONFIG[platform],
        ))["/**"]["ros__parameters"]
        driver = _yaml(_read(repo_root, F_VANJEE_PARAMS))["vanjee_lidar"]["ros__parameters"]
        fl_pre = fastlio["preprocess"]
        fl_common = fastlio["common"]
        fl_lidar_type = int(fl_pre["lidar_type"])
        fl_scan_line = int(fl_pre["scan_line"])
        fl_scan_rate = int(fl_pre["scan_rate"])
        fl_timestamp_unit = int(fl_pre["timestamp_unit"])
        fl_blind = float(fl_pre["blind"])
        driver_min_distance = float(driver["min_distance"])
        lio_n_scan = int(lio["N_SCAN"])
        lio_horizon_scan = int(lio["Horizon_SCAN"])

        if not (driver["lidar_type"] == "vanjee_722" and fl_lidar_type == 2):
            fails.append("[R1] lidar_type 不一致: driver=%r, fast-lio=%r(应为 vanjee_722/2)。"
                         % (driver["lidar_type"], fl_lidar_type))
        if not (fl_scan_line == lio_n_scan == 32):
            fails.append("[R2] 线数不一致: fast-lio=%d, lio-sam=%d(应均为 32)。"
                         % (fl_scan_line, lio_n_scan))
        if fl_scan_rate != 10:
            fails.append("[R3] fast-lio scan_rate=%d(应为 10)。" % fl_scan_rate)
        if fl_timestamp_unit != 0:
            fails.append("[R4] fast-lio timestamp_unit=%d(应为 0)。" % fl_timestamp_unit)
        if not (fl_blind == 0.3 and fl_blind >= driver_min_distance):
            fails.append("[R5] fast-lio blind=%.2f(应为 0.30且不小于 driver min_distance=%.2f)。"
                         % (fl_blind, driver_min_distance))
        if not (driver["point_cloud_topic"] == fl_common["lid_topic"] == lio["pointCloudTopic"] == "/points_raw"):
            fails.append("[R6] 点云话题不一致: driver=%r, fast-lio=%r, lio-sam=%r(应均为 /points_raw)。"
                         % (driver["point_cloud_topic"], fl_common["lid_topic"], lio["pointCloudTopic"]))
        if not (driver["imu_topic"] == fl_common["imu_topic"] == lio["imuTopic"] == "/imu/data"):
            fails.append("[R7] IMU 话题不一致: driver=%r, fast-lio=%r, lio-sam=%r(应均为 /imu/data)。"
                         % (driver["imu_topic"], fl_common["imu_topic"], lio["imuTopic"]))
        if (driver["lidar_frame"], driver["imu_frame"]) != ("velodyne", "imu_link"):
            fails.append("[R8] driver frame 不一致: lidar=%r, imu=%r(应为 velodyne/imu_link)。"
                         % (driver["lidar_frame"], driver["imu_frame"]))
        if lio["lidarFrame"] != driver["lidar_frame"]:
            fails.append("[R8] lio-sam lidarFrame=%r != driver lidar_frame=%r。"
                         % (lio["lidarFrame"], driver["lidar_frame"]))
        if not (lio_horizon_scan == 1200 and lio["use_sim_time"] is False):
            fails.append("[R9] lio-sam Horizon_SCAN=%d/use_sim_time=%r(应为 1200/false)。"
                         % (lio_horizon_scan, lio["use_sim_time"]))
        return fails

    gz = _gazebo_lidar(_read(repo_root, F_GAZEBO))
    fl_pre = fastlio["preprocess"]
    n_scan = int(_patch_added_value(
        _read(repo_root, F_LIOSAM_PATCH),
        LIOSAM_CONFIG[platform],
        "N_SCAN",
    ))
    horizon = int(_patch_added_value(
        _read(repo_root, F_LIOSAM_PATCH),
        LIOSAM_CONFIG[platform],
        "Horizon_SCAN",
    ))
    adapter_rate = round(1.0 / _adapter_scan_period(_read(repo_root, F_GZ_LAUNCH)))

    # L1 线数
    if not (gz["v_samples"] == n_scan == fl_pre["scan_line"]):
        fails.append("[L1] 线数不一致: gazebo=%d, lio-sam N_SCAN=%d, fast-lio scan_line=%d。"
                     % (gz["v_samples"], n_scan, fl_pre["scan_line"]))
    # L2 水平
    if gz["h_samples"] != horizon:
        fails.append("[L2] 水平点数不一致: gazebo=%d, lio-sam Horizon_SCAN=%d。" % (gz["h_samples"], horizon))
    # L3 频率
    if not (gz["update_rate"] == fl_pre["scan_rate"] == adapter_rate):
        fails.append("[L3] 频率不一致: gazebo update_rate=%d, fast-lio scan_rate=%d, adapter 1/scan_period=%d。"
                     % (gz["update_rate"], fl_pre["scan_rate"], adapter_rate))
    # L4 近距(不等式:盲区 >= 传感器最小距)
    if fl_pre["blind"] < gz["range_min"] - 1e-9:
        fails.append("[L4] fast-lio blind %.2f < gazebo range.min %.2f(盲区应 >= 传感器最小距)。"
                     % (fl_pre["blind"], gz["range_min"]))
    return fails


def run(repo_root=None):
    """跑全部检查,返回失败描述列表(空=全过)。repo_root 为空则从 __file__ 上溯。"""
    if yaml is None:
        return ["缺少 pyyaml(pip install pyyaml / apt install python3-yaml)。"]
    if repo_root is None:
        repo_root = find_repo_root()
    platform = load_bringup_config(repo_root)["platform"]
    fails = []
    fails += check_geometry(repo_root, platform)
    fails += check_lidar(repo_root, platform)
    return fails


def main(argv=None):
    ap = argparse.ArgumentParser(description="跨模块魔法值一致性检查")
    ap.add_argument("--repo-root", default=None, help="仓库根(默认从 __file__ 上溯)")
    ns = ap.parse_args(argv)
    root = os.path.expanduser(ns.repo_root) if ns.repo_root else None
    fails = run(root)
    if fails:
        print("跨模块一致性检查 未通过:")
        for f in fails:
            print("  - " + f)
        return 1
    print("跨模块一致性检查 通过。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
