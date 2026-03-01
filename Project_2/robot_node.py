import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from sensor_msgs.msg import PointCloud
from std_srvs.srv import Empty

class RobotNode(Node):
    __PUBLISHER_TOPIC = 'cmd_vel'
    __POSE_TOPIC = 'pose'
    __TARGETS_TOPIC = 'unvisited_targets'
    __RESET_SERVICE = 'reset'
    __TIMER_PERIOD = 0.1  # seconds

    # Control constants
    __LINEAR_GAIN = 2.0
    __ANGULAR_GAIN = 6.0
    __MAX_LINEAR_SPEED = 4.0
    __MAX_ANGULAR_SPEED = 6.0
    __ARRIVAL_THRESHOLD = 0.3
    __ANGLE_THRESHOLD = 0.1

    def __init__(self):
        super().__init__('robot_node')

        # reset
        self.reset_client = self.create_client(Empty, self.__RESET_SERVICE)
        self.reset_client.wait_for_service()
        self.reset_client.call_async(Empty.Request())

        # pub
        self.cmd_pub = self.create_publisher(Twist, self.__PUBLISHER_TOPIC, 10)

        # sub
        self.create_subscription(Pose, self.__POSE_TOPIC, self.pose_callback, 10)
        self.create_subscription(
            PointCloud, self.__TARGETS_TOPIC, self.targets_callback, 10
        )

        # states
        self.pose = None
        self.segments = []
        self.waypoints = []
        self.current_index = 0
        self.planned = False

        # Timer at 10 Hz
        self.create_timer(self.__TIMER_PERIOD, self.control_loop)

    def pose_callback(self, msg):
        self.pose = msg

    def targets_callback(self, msg):
        if self.planned:
            return
        self.segments = []
        for i in range(0, len(msg.points), 2):
            p1 = (msg.points[i].x, msg.points[i].y)
            p2 = (msg.points[i + 1].x, msg.points[i + 1].y)
            self.segments.append((p1, p2))

    def _closest_point_on_segment(self, px, py, ax, ay, bx, by):
        dx, dy = bx - ax, by - ay
        length_sq = dx * dx + dy * dy
        if length_sq == 0.0:
            return ax, ay
        t = ((px - ax) * dx + (py - ay) * dy) / length_sq
        t = max(0.0, min(1.0, t))
        return ax + t * dx, ay + t * dy

    def _plan_order(self):
        """
        greedy nearest-neighbor that always visits the closest segment next.
        For each remaining segment, the target point is the closest
        point on that segment to the robot's current position.
        """
        if self.pose is None or not self.segments:
            return

        remaining = list(self.segments)
        cx = self.pose.x
        cy = self.pose.y
        self.waypoints = []

        while remaining:
            best_dist = float('inf')
            best_idx = 0
            best_point = remaining[0][0]

            for i, (p1, p2) in enumerate(remaining):
                nx, ny = self._closest_point_on_segment(
                    cx, cy, p1[0], p1[1], p2[0], p2[1]
                )
                d = math.hypot(nx - cx, ny - cy)
                if d < best_dist:
                    best_dist = d
                    best_idx = i
                    best_point = (nx, ny)

            self.waypoints.append(best_point)
            cx, cy = best_point
            remaining.pop(best_idx)

        self.current_index = 0
        self.planned = True
        self.get_logger().info(f'Planned {len(self.waypoints)} waypoints')

    def _normalize_angle(self, angle):
        """[-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        """Turn toward target, then drive."""
        if self.pose is None or not self.segments:
            return

        if not self.planned:
            self._plan_order()
            return

        if self.current_index >= len(self.waypoints):
            self.cmd_pub.publish(Twist())
            return

        tx, ty = self.waypoints[self.current_index]
        dx = tx - self.pose.x
        dy = ty - self.pose.y
        dist = math.hypot(dx, dy)

        if dist < self.__ARRIVAL_THRESHOLD:
            self.get_logger().info(
                f'Target {self.current_index + 1}/{len(self.waypoints)} reached'
            )
            self.current_index += 1
            return

        #handles error
        desired = math.atan2(dy, dx)
        error = self._normalize_angle(desired - self.pose.theta)

        cmd = Twist()

        if abs(error) < self.__ANGLE_THRESHOLD:
            # Robot is aligned
            cmd.linear.x = min(
                self.__LINEAR_GAIN * dist, self.__MAX_LINEAR_SPEED
            )
            cmd.angular.z = self.__ANGULAR_GAIN * error
        else:
            # Rotate in place first
            cmd.angular.z = max(
                -self.__MAX_ANGULAR_SPEED,
                min(self.__MAX_ANGULAR_SPEED, self.__ANGULAR_GAIN * error),
            )

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = RobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
