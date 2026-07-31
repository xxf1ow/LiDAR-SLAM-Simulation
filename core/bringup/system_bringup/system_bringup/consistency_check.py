"""跨模块魔法值一致性检查(纯 Python,无 ROS 依赖,本机/构建机皆可跑)。

解析已跟踪源文件,断言被复述在多处的几何/雷达值彼此自洽。
- run(repo_root) -> list[str]   失败描述(空=全过)
- main()        -> int          打印失败、返回退出码(供启动闸门/CLI)
- find_repo_root()              从 __file__ 上溯定位仓库根(pytest 本机用)

契约类(帧/话题)不纳入。patch 文件只信任 '+'(项目改值)行。
"""
import argparse
import ast
import os
import re
import sys

try:
    import yaml
except ImportError:  # pyyaml 是硬依赖
    yaml = None

# ---- 已跟踪源文件(仓库相对路径) ----
F_MACRO = "core/robot/robot_description/urdf/robot_macro.urdf.xacro"
F_GAZEBO = "core/robot/robot_description/gazebo/robot.gazebo.xacro"
F_CONTROLLERS = "core/robot/robot_bringup/config/robot_controllers.yaml"
F_NAV_PARAMS = "core/navigation/robot_navigation/config/nav2_params.yaml"
F_NAV_LAUNCH = "core/navigation/robot_navigation/launch/navigation.launch.py"
F_GZ_LAUNCH = "core/simulation/robot_gz_bringup/launch/robot_gz.launch.py"
F_FASTLIO_PATCH = "core/localization/fast-lio2.patch"
F_LIOSAM_PATCH = "core/mapping/lio-sam.patch"

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


def check_geometry(repo_root):
    """G1–G5:几何派生值在 xacro / controllers / nav2 / launch 间自洽。"""
    fails = []
    macro = _read(repo_root, F_MACRO)
    props = _xacro_props(macro)
    base_l, base_w, base_h = props["base_length"], props["base_width"], props["base_height"]
    wheel_r, lidar_h = props["wheel_radius"], props["lidar_height"]

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

    # G2 controllers 轮参 == xacro
    if abs(cp["wheel_radius"] - wheel_r) > 1e-9:
        fails.append("[G2] wheel_radius 不一致: xacro=%.4f vs controllers=%.4f。改 robot_controllers.yaml 或核对 xacro。"
                     % (wheel_r, cp["wheel_radius"]))
    if abs(cp["wheel_separation"] - base_w) > 1e-9:
        fails.append("[G2] wheel_separation controllers=%.4f != xacro base_width=%.4f。" % (cp["wheel_separation"], base_w))

    # G3 navigation.launch.py 几何常量 == xacro(weld_z 由它派生)
    lc = _launch_floats(_read(repo_root, F_NAV_LAUNCH), ["_BASE_HEIGHT", "_WHEEL_RADIUS", "_LIDAR_HEIGHT"])
    for cname, xval in [("_BASE_HEIGHT", base_h), ("_WHEEL_RADIUS", wheel_r), ("_LIDAR_HEIGHT", lidar_h)]:
        if cname not in lc:
            fails.append("[G3] navigation.launch.py 缺几何常量 %s(weld_z 应由它派生)。" % cname)
        elif abs(lc[cname] - xval) > 1e-9:
            fails.append("[G3] navigation.launch.py %s=%.4f != xacro %.4f。改 navigation.launch.py 或核对 xacro。"
                         % (cname, lc[cname], xval))

    # G4 共位 -> 外参零
    vxyz = _xacro_joint_origin_xyz(macro, "velodyne_joint")
    ixyz = _xacro_joint_origin_xyz(macro, "imu_joint")
    if vxyz != ixyz:
        fails.append("[G4] velodyne_joint/imu_joint origin 不同(应共位): '%s' vs '%s'。" % (vxyz, ixyz))
    fastlio = _yaml(_patch_added_file(
        _read(repo_root, F_FASTLIO_PATCH),
        "config/gazebo_velodyne.yaml",
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


def check_lidar(repo_root):
    """L1–L4:雷达规格在 gazebo.xacro / lio-sam.patch / fast-lio2.patch / adapter 间自洽。"""
    fails = []
    gz = _gazebo_lidar(_read(repo_root, F_GAZEBO))
    fastlio = _yaml(_patch_added_file(
        _read(repo_root, F_FASTLIO_PATCH),
        "config/gazebo_velodyne.yaml",
    ))["/**"]["ros__parameters"]
    fl_pre = fastlio["preprocess"]
    n_scan = int(_patch_added_value(
        _read(repo_root, F_LIOSAM_PATCH),
        "config/params.yaml",
        "N_SCAN",
    ))
    horizon = int(_patch_added_value(
        _read(repo_root, F_LIOSAM_PATCH),
        "config/params.yaml",
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
    fails = []
    fails += check_geometry(repo_root)
    fails += check_lidar(repo_root)
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
