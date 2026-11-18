"""Rule-based behavior planner (finite state machine).

Looks at the fused object list for anything in the ego corridor ahead and
decides between LANE_FOLLOW, FOLLOW_VEHICLE, STOP and EMERGENCY_BRAKE, plus a
target speed for the longitudinal controller.

Subscribed:  /fusion/tracked_objects   autonomous_interfaces/TrackedObjectArray
             /carla/ego/odometry       nav_msgs/Odometry
             /planning/global_path     nav_msgs/Path
Published:   /planning/behavior_state  autonomous_interfaces/BehaviorState
"""
from autonomous_interfaces.msg import BehaviorState, TrackedObjectArray
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

STOP_CLASSES = {'pedestrian', 'cyclist', 'stop_sign'}


def decide(objects, ego_speed, cruise_speed, corridor_half_width=1.8,
           safe_time_gap=2.0, min_gap=5.0, ttc_emergency=1.5):
    """Pure decision function. objects: iterable of (id, x, y, vx, label). Returns
    (state, target_speed, lead_id)."""
    lead = None
    for oid, x, y, vx, label in objects:
        if x > 0.0 and abs(y) < corridor_half_width and (lead is None or x < lead[1]):
            lead = (oid, x, vx, label)
    if lead is None:
        return BehaviorState.LANE_FOLLOW, cruise_speed, 0

    oid, dist, rel_vx, label = lead
    closing = -rel_vx  # relative velocity in base_link: negative = approaching
    if closing > 0.1 and dist / closing < ttc_emergency:
        return BehaviorState.EMERGENCY_BRAKE, 0.0, oid
    if label in STOP_CLASSES and dist < min_gap + ego_speed * safe_time_gap:
        return BehaviorState.STOP, 0.0, oid
    desired_gap = min_gap + ego_speed * safe_time_gap
    if dist < desired_gap * 2.0:
        lead_speed = max(0.0, ego_speed + rel_vx)
        speed = lead_speed * min(1.0, dist / desired_gap)
        return BehaviorState.FOLLOW_VEHICLE, min(cruise_speed, speed), oid
    return BehaviorState.LANE_FOLLOW, cruise_speed, 0


class BehaviorPlannerNode(Node):

    def __init__(self):
        super().__init__('behavior_planner_node')
        self.declare_parameter('cruise_speed', 8.0)  # [m/s]
        self.declare_parameter('corridor_half_width', 1.8)
        self.declare_parameter('safe_time_gap', 2.0)
        self.declare_parameter('min_gap', 5.0)
        self.declare_parameter('ttc_emergency', 1.5)

        self.ego_speed = 0.0
        self.has_path = False
        self.objects = []
        self.pub = self.create_publisher(BehaviorState, '/planning/behavior_state', 10)
        self.create_subscription(
            TrackedObjectArray, '/fusion/tracked_objects', self.on_objects, 10)
        self.create_subscription(Odometry, '/carla/ego/odometry', self.on_odom, 10)
        self.create_subscription(Path, '/planning/global_path', self.on_path, 10)
        self.create_timer(0.1, self.step)

    def on_objects(self, msg):
        self.objects = [(o.id, o.position.x, o.position.y, o.velocity.x, o.class_label)
                        for o in msg.objects]

    def on_odom(self, msg):
        self.ego_speed = msg.twist.twist.linear.x

    def on_path(self, msg):
        self.has_path = bool(msg.poses)

    def step(self):
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        if not self.has_path:
            state, speed, lead = BehaviorState.STOP, 0.0, 0
        else:
            state, speed, lead = decide(
                self.objects, self.ego_speed, p('cruise_speed'), p('corridor_half_width'),
                p('safe_time_gap'), p('min_gap'), p('ttc_emergency'))
        self.pub.publish(BehaviorState(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id='base_link'),
            state=state, target_speed=float(speed), lead_object_id=lead))


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
