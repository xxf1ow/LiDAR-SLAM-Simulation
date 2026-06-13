"""把 Gz 组织化点云的点序号映射到 Velodyne 风格的 ring/time。

组织化点云:height=垂直线(ring),width=水平采样,行主序。
ring = index // width;time = (index % width) / width * scan_period。
对标量与 numpy 数组同时成立(整除/取余/乘除都是逐元素)。
"""


def compute_ring_time(index, width, scan_period):
    ring = index // width
    time = (index % width) / width * scan_period
    return ring, time
