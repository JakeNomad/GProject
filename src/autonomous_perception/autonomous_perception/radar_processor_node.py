"""Radar object extraction.

Converts raw polar radar returns into Cartesian objects in base_link and
keeps the radial velocity, which the fusion layer uses as a velocity cue.

Subscribed:  /carla/ego/radar/front       autonomous_interfaces/RadarDetectionArray
Published:   /perception/radar/objects    autonomous_interfaces/DetectedObjectArray
"""
import math

from autonomous_interfaces.msg import DetectedObject, DetectedObjectArray, RadarDetectionArray
import rclpy
from rclpy.node import Node


class RadarProcessorNode(Node):

    def __init__(self):
        super().__init__('radar_processor_node')
        self.declare_parameter('mount_x', 2.0)
        self.declare_parameter('mount_z', 1.0)
        self.declare_parameter('min_abs_velocity', 0.5)  # drop static clutter [m/s]
        self.declare_parameter('max_altitude', 0.2)      # [rad]

        self.pub = self.create_publisher(DetectedObjectArray, '/perception/radar/objects', 10)
        self.create_subscription(
            RadarDetectionArray, '/carla/ego/radar/front', self.on_radar, 10)

    def on_radar(self, msg):
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        out = DetectedObjectArray(header=msg.header)
        out.header.frame_id = 'base_link'
        for d in msg.detections:
            if abs(d.velocity) < p('min_abs_velocity') or abs(d.altitude) > p('max_altitude'):
                continue
            obj = DetectedObject(source=DetectedObject.SOURCE_RADAR, class_label='unknown',
                                 confidence=0.5)
            ground = d.depth * math.cos(d.altitude)
            obj.position.x = p('mount_x') + ground * math.cos(d.azimuth)
            obj.position.y = ground * math.sin(d.azimuth)
            obj.position.z = p('mount_z') + d.depth * math.sin(d.altitude)
            obj.radial_velocity = float(d.velocity)
            out.objects.append(obj)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RadarProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
