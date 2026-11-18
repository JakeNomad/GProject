"""CARLA <-> ROS 2 bridge.

Connects to a running CARLA server, spawns the ego vehicle with a front
camera, a roof LiDAR and a front radar, publishes their data on ROS 2 topics
and applies incoming VehicleControlCmd messages to the ego vehicle.

Published:
    /carla/ego/camera/front/image_raw    sensor_msgs/Image
    /carla/ego/camera/front/camera_info  sensor_msgs/CameraInfo
    /carla/ego/lidar/points              sensor_msgs/PointCloud2
    /carla/ego/radar/front               autonomous_interfaces/RadarDetectionArray
    /carla/ego/odometry                  nav_msgs/Odometry
    /tf                                  map -> base_link
Subscribed:
    /control/vehicle_cmd                 autonomous_interfaces/VehicleControlCmd
"""
import math

from autonomous_interfaces.msg import RadarDetection, RadarDetectionArray, VehicleControlCmd
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

try:
    import carla  # CARLA PythonAPI: pip install carla==<server version>
except ImportError:  # allows building/linting without CARLA installed
    carla = None


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class CarlaBridgeNode(Node):

    def __init__(self):
        super().__init__('carla_bridge_node')
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 2000)
        self.declare_parameter('town', 'Town03')
        self.declare_parameter('fixed_delta_seconds', 0.05)
        self.declare_parameter('vehicle_filter', 'vehicle.tesla.model3')
        self.declare_parameter('camera.width', 800)
        self.declare_parameter('camera.height', 600)
        self.declare_parameter('camera.fov', 90.0)
        self.declare_parameter('lidar.range', 50.0)
        self.declare_parameter('lidar.channels', 32)
        self.declare_parameter('radar.range', 100.0)

        self.image_pub = self.create_publisher(Image, '/carla/ego/camera/front/image_raw', 10)
        self.cam_info_pub = self.create_publisher(
            CameraInfo, '/carla/ego/camera/front/camera_info', 10)
        self.lidar_pub = self.create_publisher(PointCloud2, '/carla/ego/lidar/points', 10)
        self.radar_pub = self.create_publisher(RadarDetectionArray, '/carla/ego/radar/front', 10)
        self.odom_pub = self.create_publisher(Odometry, '/carla/ego/odometry', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(VehicleControlCmd, '/control/vehicle_cmd', self.on_control, 10)

        self.world = None
        self.ego = None
        self.actors = []
        if carla is None:
            self.get_logger().error('CARLA PythonAPI not found; bridge is idle.')
            return
        self._setup_world()
        self.create_timer(self.get_parameter('fixed_delta_seconds').value, self.tick)

    # ------------------------------------------------------------------ setup
    def _setup_world(self):
        p = self.get_parameter
        client = carla.Client(p('host').value, p('port').value)
        client.set_timeout(10.0)
        self.world = client.load_world(p('town').value)
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = p('fixed_delta_seconds').value
        self.world.apply_settings(settings)

        lib = self.world.get_blueprint_library()
        spawn = self.world.get_map().get_spawn_points()[0]
        self.ego = self.world.spawn_actor(lib.filter(p('vehicle_filter').value)[0], spawn)
        self.actors.append(self.ego)

        cam_bp = lib.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', str(p('camera.width').value))
        cam_bp.set_attribute('image_size_y', str(p('camera.height').value))
        cam_bp.set_attribute('fov', str(p('camera.fov').value))
        self._attach(cam_bp, carla.Transform(carla.Location(x=1.5, z=1.6)), self.on_image)

        lidar_bp = lib.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('range', str(p('lidar.range').value))
        lidar_bp.set_attribute('channels', str(p('lidar.channels').value))
        lidar_bp.set_attribute('rotation_frequency', str(1.0 / p('fixed_delta_seconds').value))
        self._attach(lidar_bp, carla.Transform(carla.Location(z=2.4)), self.on_lidar)

        radar_bp = lib.find('sensor.other.radar')
        radar_bp.set_attribute('range', str(p('radar.range').value))
        self._attach(radar_bp, carla.Transform(carla.Location(x=2.0, z=1.0)), self.on_radar)
        self.get_logger().info(f'Ego vehicle spawned: {self.ego.type_id}')

    def _attach(self, bp, transform, callback):
        sensor = self.world.spawn_actor(bp, transform, attach_to=self.ego)
        sensor.listen(callback)
        self.actors.append(sensor)

    def _header(self, frame_id):
        return Header(stamp=self.get_clock().now().to_msg(), frame_id=frame_id)

    # -------------------------------------------------------------- callbacks
    def tick(self):
        self.world.tick()
        self._publish_odometry()

    def on_image(self, image):
        msg = Image(header=self._header('camera_front'))
        msg.height, msg.width = image.height, image.width
        msg.encoding = 'bgra8'
        msg.step = image.width * 4
        msg.data = bytes(image.raw_data)
        self.image_pub.publish(msg)

        f = image.width / (2.0 * math.tan(math.radians(float(image.fov)) / 2.0))
        info = CameraInfo(header=msg.header, height=image.height, width=image.width)
        info.k = [f, 0.0, image.width / 2.0, 0.0, f, image.height / 2.0, 0.0, 0.0, 1.0]
        self.cam_info_pub.publish(info)

    def on_lidar(self, scan):
        points = np.frombuffer(scan.raw_data, dtype=np.float32).reshape(-1, 4).copy()
        points[:, 1] *= -1.0  # CARLA is left-handed (y right) -> ROS REP-103 (y left)
        msg = PointCloud2(header=self._header('lidar'))
        msg.height, msg.width = 1, points.shape[0]
        msg.fields = [
            PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
            for i, n in enumerate(('x', 'y', 'z', 'intensity'))
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * points.shape[0]
        msg.is_dense = True
        msg.data = points.tobytes()
        self.lidar_pub.publish(msg)

    def on_radar(self, measurement):
        msg = RadarDetectionArray(header=self._header('radar_front'))
        for d in measurement:
            msg.detections.append(RadarDetection(
                depth=d.depth, azimuth=-d.azimuth, altitude=d.altitude, velocity=d.velocity))
        self.radar_pub.publish(msg)

    def _publish_odometry(self):
        t = self.ego.get_transform()
        v = self.ego.get_velocity()
        yaw = -math.radians(t.rotation.yaw)
        header = self._header('map')

        odom = Odometry(header=header, child_frame_id='base_link')
        pose = odom.pose.pose
        pose.position.x, pose.position.y, pose.position.z = (
            t.location.x, -t.location.y, t.location.z)
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = yaw_to_quaternion(yaw)
        vx, vy = v.x, -v.y  # world frame (ROS convention)
        odom.twist.twist.linear.x = math.cos(yaw) * vx + math.sin(yaw) * vy
        odom.twist.twist.linear.y = -math.sin(yaw) * vx + math.cos(yaw) * vy
        self.odom_pub.publish(odom)

        tf = TransformStamped(header=header, child_frame_id='base_link')
        tf.transform.translation.x = pose.position.x
        tf.transform.translation.y = pose.position.y
        tf.transform.translation.z = pose.position.z
        tf.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(tf)

    def on_control(self, cmd):
        if self.ego is None:
            return
        self.ego.apply_control(carla.VehicleControl(
            throttle=float(cmd.throttle), steer=float(cmd.steer), brake=float(cmd.brake),
            hand_brake=cmd.hand_brake, reverse=cmd.reverse))

    def destroy_node(self):
        if self.world is not None:
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            self.world.apply_settings(settings)
        for actor in reversed(self.actors):
            if hasattr(actor, 'stop'):
                actor.stop()
            actor.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CarlaBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
