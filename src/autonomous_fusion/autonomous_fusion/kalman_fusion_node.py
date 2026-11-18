"""Sensor fusion node: LiDAR + radar + camera -> tracked objects.

LiDAR clusters create and update tracks (position), radar refines position
and velocity (Doppler), camera detections attach class labels by bearing.

Subscribed:  /perception/lidar/obstacles   autonomous_interfaces/DetectedObjectArray
             /perception/radar/objects     autonomous_interfaces/DetectedObjectArray
             /perception/camera/detections autonomous_interfaces/DetectedObjectArray
Published:   /fusion/tracked_objects       autonomous_interfaces/TrackedObjectArray

Note: tracks are kept in base_link (ego-relative). Moving to the `map` frame
via tf2 is a planned improvement once localisation is in place.
"""
import math

from autonomous_interfaces.msg import DetectedObjectArray, TrackedObject, TrackedObjectArray
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from .kalman_filter import MultiObjectTracker


class KalmanFusionNode(Node):

    def __init__(self):
        super().__init__('kalman_fusion_node')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('lidar_var', 0.2)
        self.declare_parameter('radar_pos_var', 1.0)
        self.declare_parameter('radar_vel_var', 0.5)
        self.declare_parameter('max_misses', 5)
        self.declare_parameter('min_hits', 3)
        p = lambda n: self.get_parameter(n).value  # noqa: E731

        self.tracker = MultiObjectTracker(
            p('lidar_var'), p('radar_pos_var'), p('radar_vel_var'), p('max_misses'))
        self.dt = 1.0 / p('rate_hz')

        self.pub = self.create_publisher(TrackedObjectArray, '/fusion/tracked_objects', 10)
        self.create_subscription(
            DetectedObjectArray, '/perception/lidar/obstacles', self.on_lidar, 10)
        self.create_subscription(
            DetectedObjectArray, '/perception/radar/objects', self.on_radar, 10)
        self.create_subscription(
            DetectedObjectArray, '/perception/camera/detections', self.on_camera, 10)
        self.create_timer(self.dt, self.step)

    def on_lidar(self, msg):
        self.tracker.update_lidar([
            (o.position.x, o.position.y, (o.size.x, o.size.y, o.size.z)) for o in msg.objects])

    def on_radar(self, msg):
        self.tracker.update_radar([
            (o.position.x, o.position.y, o.radial_velocity) for o in msg.objects])

    def on_camera(self, msg):
        self.tracker.update_camera([
            (math.atan2(o.position.y, o.position.x), o.class_label, o.confidence)
            for o in msg.objects])

    def step(self):
        self.tracker.predict(self.dt)
        out = TrackedObjectArray(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id='base_link'))
        for t in self.tracker.confirmed(self.get_parameter('min_hits').value):
            o = TrackedObject(id=t.id, class_label=t.label, confidence=float(t.confidence),
                              age=t.hits)
            o.position.x, o.position.y = float(t.kf.x[0]), float(t.kf.x[1])
            o.velocity.x, o.velocity.y = float(t.kf.x[2]), float(t.kf.x[3])
            o.size.x, o.size.y, o.size.z = (float(s) for s in t.size)
            o.covariance = [float(v) for v in t.kf.P.flatten()]
            out.objects.append(o)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = KalmanFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
