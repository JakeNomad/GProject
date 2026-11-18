"""Vehicle controller: follows the planned path at the behavior target speed.

Subscribed:  /planning/global_path     nav_msgs/Path
             /planning/behavior_state  autonomous_interfaces/BehaviorState
             /carla/ego/odometry       nav_msgs/Odometry
Published:   /control/vehicle_cmd      autonomous_interfaces/VehicleControlCmd
"""
import math

from autonomous_interfaces.msg import BehaviorState, VehicleControlCmd
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from .controllers import PID, pure_pursuit_steer


class VehicleControllerNode(Node):

    def __init__(self):
        super().__init__('vehicle_controller_node')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('wheelbase', 2.875)          # Tesla Model 3 [m]
        self.declare_parameter('max_steer_angle', 1.22)     # [rad] maps to steer = 1.0
        self.declare_parameter('lookahead_gain', 0.8)
        self.declare_parameter('min_lookahead', 4.0)
        self.declare_parameter('speed_kp', 0.5)
        self.declare_parameter('speed_ki', 0.05)
        self.declare_parameter('speed_kd', 0.02)
        p = lambda n: self.get_parameter(n).value  # noqa: E731

        self.pid = PID(p('speed_kp'), p('speed_ki'), p('speed_kd'))
        self.dt = 1.0 / p('rate_hz')
        self.path = []
        self.odom = None
        self.behavior = None

        self.pub = self.create_publisher(VehicleControlCmd, '/control/vehicle_cmd', 10)
        self.create_subscription(Path, '/planning/global_path', self.on_path, 10)
        self.create_subscription(BehaviorState, '/planning/behavior_state', self.on_behavior, 10)
        self.create_subscription(Odometry, '/carla/ego/odometry', self.on_odom, 10)
        self.create_timer(self.dt, self.step)

    def on_path(self, msg):
        self.path = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]

    def on_behavior(self, msg):
        self.behavior = msg

    def on_odom(self, msg):
        self.odom = msg

    def step(self):
        cmd = VehicleControlCmd(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id='base_link'))
        if self.odom is None or self.behavior is None or not self.path:
            cmd.brake = 1.0
            self.pub.publish(cmd)
            return

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        pose = self.odom.pose.pose
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        speed = self.odom.twist.twist.linear.x

        steer = pure_pursuit_steer(pose.position.x, pose.position.y, yaw, self.path, speed,
                                   p('wheelbase'), p('lookahead_gain'), p('min_lookahead'))
        if steer is None or self.behavior.state == BehaviorState.EMERGENCY_BRAKE:
            self.pid.reset()
            cmd.brake = 1.0
            self.pub.publish(cmd)
            return

        # CARLA steer is in [-1, 1] with positive = right; ROS angle positive = left
        cmd.steer = float(max(-1.0, min(1.0, -steer / p('max_steer_angle'))))
        u = self.pid.step(self.behavior.target_speed - speed, self.dt)
        cmd.throttle = float(max(0.0, min(1.0, u)))
        cmd.brake = float(max(0.0, min(1.0, -u)))
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
