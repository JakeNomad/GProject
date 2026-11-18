"""LiDAR obstacle extraction.

Pipeline: range crop -> ground removal (height threshold) -> voxel-grid
clustering -> one DetectedObject per cluster (centroid + axis-aligned size).

Subscribed:  /carla/ego/lidar/points        sensor_msgs/PointCloud2
Published:   /perception/lidar/obstacles    autonomous_interfaces/DetectedObjectArray
             /perception/lidar/nonground    sensor_msgs/PointCloud2 (for Nav2 costmap)
"""
from autonomous_interfaces.msg import DetectedObject, DetectedObjectArray
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def cluster_points(xy, cell_size):
    """Group 2D points into connected occupied grid cells (8-neighbourhood)."""
    if len(xy) == 0:
        return []
    cells = np.floor(xy / cell_size).astype(np.int64)
    cell_to_idx = {}
    for i, c in enumerate(map(tuple, cells)):
        cell_to_idx.setdefault(c, []).append(i)

    clusters, visited = [], set()
    for start in cell_to_idx:
        if start in visited:
            continue
        stack, members = [start], []
        visited.add(start)
        while stack:
            cx, cy = stack.pop()
            members.extend(cell_to_idx[(cx, cy)])
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (cx + dx, cy + dy)
                    if n in cell_to_idx and n not in visited:
                        visited.add(n)
                        stack.append(n)
        clusters.append(np.array(members))
    return clusters


class LidarProcessorNode(Node):

    def __init__(self):
        super().__init__('lidar_processor_node')
        self.declare_parameter('sensor_height', 2.4)  # lidar mount height above ground [m]
        self.declare_parameter('ground_threshold', 0.3)
        self.declare_parameter('max_range', 40.0)
        self.declare_parameter('cluster_cell_size', 0.5)
        self.declare_parameter('min_cluster_points', 10)

        self.obs_pub = self.create_publisher(
            DetectedObjectArray, '/perception/lidar/obstacles', 10)
        self.nonground_pub = self.create_publisher(
            PointCloud2, '/perception/lidar/nonground', 10)
        self.create_subscription(PointCloud2, '/carla/ego/lidar/points', self.on_cloud, 10)

    def on_cloud(self, msg):
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        # convert to base_link (ground-level origin)
        pts = pts + np.array([0.0, 0.0, p('sensor_height')], dtype=np.float32)
        rng = np.linalg.norm(pts[:, :2], axis=1)
        keep = (rng < p('max_range')) & (pts[:, 2] > p('ground_threshold')) & (rng > 2.5)
        pts = pts[keep]

        cloud = point_cloud2.create_cloud_xyz32(msg.header, pts.tolist())
        cloud.header.frame_id = 'base_link'
        self.nonground_pub.publish(cloud)

        out = DetectedObjectArray(header=cloud.header)
        for idx in cluster_points(pts[:, :2], p('cluster_cell_size')):
            if len(idx) < p('min_cluster_points'):
                continue
            c = pts[idx]
            lo, hi = c.min(axis=0), c.max(axis=0)
            obj = DetectedObject(source=DetectedObject.SOURCE_LIDAR, class_label='unknown',
                                 confidence=min(1.0, len(idx) / 100.0))
            obj.position.x, obj.position.y, obj.position.z = (float(v) for v in (lo + hi) / 2)
            obj.size.x, obj.size.y, obj.size.z = (float(v) for v in hi - lo)
            out.objects.append(obj)
        self.obs_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
