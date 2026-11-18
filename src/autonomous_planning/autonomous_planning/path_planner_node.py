"""Global path planning through Nav2.

Takes a goal (e.g. from RViz "2D Goal Pose"), asks the Nav2 planner_server
for a path via the ComputePathToPose action and republishes it for the
controller. Replans periodically so new obstacles in the costmap are taken
into account.

Subscribed:  /goal_pose               geometry_msgs/PoseStamped
Action:      /compute_path_to_pose    nav2_msgs/action/ComputePathToPose
Published:   /planning/global_path    nav_msgs/Path
"""
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class PathPlannerNode(Node):

    def __init__(self):
        super().__init__('path_planner_node')
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('replan_period', 1.0)

        self.goal = None
        self.pending = False
        self.client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.path_pub = self.create_publisher(Path, '/planning/global_path', 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_timer(self.get_parameter('replan_period').value, self.request_path)

    def on_goal(self, msg):
        self.get_logger().info(
            f'New goal: ({msg.pose.position.x:.1f}, {msg.pose.position.y:.1f})')
        self.goal = msg
        self.request_path()

    def request_path(self):
        if self.goal is None or self.pending:
            return
        if not self.client.server_is_ready():
            self.get_logger().warn('Nav2 planner_server not available yet', once=True)
            return
        req = ComputePathToPose.Goal(
            goal=self.goal, planner_id=self.get_parameter('planner_id').value, use_start=False)
        self.pending = True
        self.client.send_goal_async(req).add_done_callback(self._on_accepted)

    def _on_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.pending = False
            return
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        self.pending = False
        path = future.result().result.path
        if path.poses:
            self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
