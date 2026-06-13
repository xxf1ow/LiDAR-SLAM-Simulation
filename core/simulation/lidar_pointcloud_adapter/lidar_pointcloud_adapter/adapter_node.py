"""订阅 Gz 桥来的组织化 PointCloud2,补 ring(uint16)+time(float32),
以 Velodyne 风格重发。数学见 ring_time.compute_ring_time。

参数:
  input_topic  (默认 /gz_lidar/points):Gz 桥输出的原始点云
  output_topic (默认 /points_raw):FAST-LIO 订阅的话题
  output_frame (默认 velodyne)
  scan_period  (默认 0.1 = 1/雷达update_rate)
注意:依赖输入为组织化点云(height=ring 数, width=水平采样, 行主序);
若 build 机实测 height==1(非组织化),需按实际 ring 来源调整(见 spike README)。
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

from lidar_pointcloud_adapter.ring_time import compute_ring_time

_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
    PointField(name='time', offset=18, datatype=PointField.FLOAT32, count=1),
]


class AdapterNode(Node):
    def __init__(self):
        super().__init__('lidar_pointcloud_adapter')
        self.declare_parameter('input_topic', '/gz_lidar/points')
        self.declare_parameter('output_topic', '/points_raw')
        self.declare_parameter('output_frame', 'velodyne')
        self.declare_parameter('scan_period', 0.1)
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.scan_period = float(self.get_parameter('scan_period').value)

        self.pub = self.create_publisher(PointCloud2, self.output_topic, 5)
        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self.cb, 5)
        self.get_logger().info(
            f'adapter: {self.input_topic} -> {self.output_topic} '
            f'(frame={self.output_frame}, scan_period={self.scan_period})')

    def cb(self, msg: PointCloud2):
        width = msg.width if msg.width > 0 else 1
        pts = point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z', 'intensity'),
            skip_nans=False, reshape_organized_cloud=False)
        n = pts.shape[0]
        if n == 0:
            return
        idx = np.arange(n, dtype=np.int64)
        ring, t = compute_ring_time(idx, width, self.scan_period)

        out = [(float(p['x']), float(p['y']), float(p['z']),
                float(p['intensity']), int(ring[i]), float(t[i]))
               for i, p in enumerate(pts)]

        header = msg.header
        header.frame_id = self.output_frame
        cloud = point_cloud2.create_cloud(header, _FIELDS, out)
        self.pub.publish(cloud)


def main():
    rclpy.init()
    node = None
    try:
        node = AdapterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
