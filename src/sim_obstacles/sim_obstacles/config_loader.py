"""障碍清单 YAML 加载 + 校验（纯逻辑，不依赖 ROS，本机可单测）。"""
import yaml

REQUIRED_KEYS = {"name", "x", "y", "yaw", "speed", "period"}


def load_obstacles(path: str) -> list:
    """读取并校验障碍清单；非法配置抛 ValueError（fail fast，spec §3）。

    返回的每项保证含 REQUIRED_KEYS 及 phase（缺省补 0.0）。
    """
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    obstacles = cfg.get("obstacles") if isinstance(cfg, dict) else None
    if not obstacles:
        raise ValueError(f"{path}: 缺少非空 obstacles 列表")

    names = set()
    out = []
    for i, ob in enumerate(obstacles):
        missing = REQUIRED_KEYS - set(ob)
        if missing:
            raise ValueError(f"obstacles[{i}] 缺少字段: {sorted(missing)}")
        if ob["speed"] <= 0:
            raise ValueError(f"obstacles[{i}] ({ob['name']}): speed 必须 > 0")
        if ob["period"] <= 0:
            raise ValueError(f"obstacles[{i}] ({ob['name']}): period 必须 > 0")
        if ob["name"] in names:
            raise ValueError(f"obstacles[{i}]: name 重复: {ob['name']}")
        names.add(ob["name"])
        ob.setdefault("phase", 0.0)
        out.append(ob)
    return out
