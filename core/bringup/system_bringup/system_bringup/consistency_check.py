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


def _patch_added_text(text):
    """patch 的 '+'(新增)行内容(去前导 '+',跳过 '+++' 头),拼回文本。"""
    out = []
    for ln in text.splitlines():
        if ln.startswith("+++"):
            continue
        if ln.startswith("+"):
            out.append(ln[1:])
    return "\n".join(out)


def _patch_added_value(text, key):
    """在 patch '+' 行里找 `key: value`,返回去注释/去引号的 value 串。"""
    for ln in text.splitlines():
        if ln.startswith("+++") or not ln.startswith("+"):
            continue
        m = re.match(r'\s*' + re.escape(key) + r'\s*:\s*(.+?)\s*(#.*)?$', ln[1:])
        if m:
            return m.group(1).strip().strip('"')
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
