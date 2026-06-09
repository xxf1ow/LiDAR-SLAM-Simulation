#!/usr/bin/env python3
"""离线把 3D 先验地图 PCD 按高度带投影成 2D 占据栅格 (PGM + YAML)，供 nav2 map_server 加载。

纯逻辑（filter_height_band / points_to_occupancy / occupancy_to_pgm_bytes / make_map_yaml）
只依赖 numpy，可在编辑机用 pytest 测；read_pcd_xyz 用 open3d，仅在 build 机运行。

用法（build 机）：
  pip install open3d
  python pcd_to_occupancy.py --pcd ~/result/GlobalMap.pcd --out ~/result/map.yaml \\
      --resolution 0.05 --z-min 0.2 --z-max 1.5 --min-pts 2
输出 map.yaml + 同目录同名 map.pgm。
"""
import argparse
import os

import numpy as np


def filter_height_band(points, z_min, z_max):
    """保留 z 落在 [z_min, z_max] 的点。points: (N,3) -> (M,3)。"""
    z = points[:, 2]
    mask = (z >= z_min) & (z <= z_max)
    return points[mask]


def points_to_occupancy(xy, resolution, min_pts):
    """(N,2) 点 -> 占据栅格。

    返回 (grid, origin_xy)：
      grid: (H,W) uint8，占据=100、空闲=0；行号随 +y 增大（未翻转）。
      origin_xy: (2,) = (min_x, min_y)，即 map.yaml 的 origin（左下角像元世界坐标）。
    占据判据：像元内点数 >= min_pts。
    """
    if xy.shape[0] == 0:
        raise ValueError("empty point set after height-band filter")
    min_x = float(np.min(xy[:, 0]))
    min_y = float(np.min(xy[:, 1]))
    max_x = float(np.max(xy[:, 0]))
    max_y = float(np.max(xy[:, 1]))
    width = int(np.ceil((max_x - min_x) / resolution)) + 1
    height = int(np.ceil((max_y - min_y) / resolution)) + 1
    counts = np.zeros((height, width), dtype=np.int32)
    cols = ((xy[:, 0] - min_x) / resolution).astype(np.int32)
    rows = ((xy[:, 1] - min_y) / resolution).astype(np.int32)
    np.clip(cols, 0, width - 1, out=cols)
    np.clip(rows, 0, height - 1, out=rows)
    np.add.at(counts, (rows, cols), 1)
    grid = np.where(counts >= min_pts, np.uint8(100), np.uint8(0)).astype(np.uint8)
    return grid, np.array([min_x, min_y], dtype=np.float64)


def occupancy_to_pgm_bytes(grid):
    """grid {0 空闲,100 占据} -> 二进制 P5 PGM 字节。

    按 map_server 约定上下翻转（图像第一行 = 最大 y），占据->0(黑)、空闲->254(白)。
    """
    pix = np.where(grid >= 100, np.uint8(0), np.uint8(254)).astype(np.uint8)
    pix = np.flipud(pix)
    h, w = pix.shape
    header = ("P5\n%d %d\n255\n" % (w, h)).encode("ascii")
    return header + pix.tobytes()


def make_map_yaml(image_name, resolution, origin_xy):
    """生成 map_server 的 .yaml 文本。"""
    return (
        "image: %s\n"
        "resolution: %s\n"
        "origin: [%s, %s, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
        "mode: trinary\n"
    ) % (image_name, repr(float(resolution)),
         repr(float(origin_xy[0])), repr(float(origin_xy[1])))


def read_pcd_xyz(path):  # pragma: no cover - 仅 build 机（需 open3d）
    """用 open3d 读 PCD，返回 (N,3) float64。"""
    import open3d as o3d
    pcd = o3d.io.read_point_cloud(path)
    return np.asarray(pcd.points, dtype=np.float64)


def main(argv=None):  # pragma: no cover - CLI，仅 build 机
    ap = argparse.ArgumentParser(description="PCD -> 2D occupancy grid (PGM+YAML)")
    ap.add_argument("--pcd", required=True, help="输入先验地图 .pcd")
    ap.add_argument("--out", required=True, help="输出 .yaml 路径（同目录写同名 .pgm）")
    ap.add_argument("--resolution", type=float, default=0.05)
    ap.add_argument("--z-min", type=float, default=0.2)
    ap.add_argument("--z-max", type=float, default=1.5)
    ap.add_argument("--min-pts", type=int, default=2)
    args = ap.parse_args(argv)

    pts = read_pcd_xyz(args.pcd)
    band = filter_height_band(pts, args.z_min, args.z_max)
    grid, origin = points_to_occupancy(band[:, :2], args.resolution, args.min_pts)

    out_yaml = os.path.abspath(args.out)
    base = os.path.splitext(out_yaml)[0]
    pgm_path = base + ".pgm"
    image_name = os.path.basename(pgm_path)

    with open(pgm_path, "wb") as f:
        f.write(occupancy_to_pgm_bytes(grid))
    with open(out_yaml, "w") as f:
        f.write(make_map_yaml(image_name, args.resolution, origin))

    print("wrote %s (%dx%d) + %s ; origin=%s"
          % (pgm_path, grid.shape[1], grid.shape[0], out_yaml, origin.tolist()))


if __name__ == "__main__":  # pragma: no cover
    main()
