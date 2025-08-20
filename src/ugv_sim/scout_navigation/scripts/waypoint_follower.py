#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robot_navigator import BasicNavigator, TaskResult

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower_client')
        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()
        self.auto = False
        self.running = False
        self.have_pose = False
        self.current_pose = None
        self.goals = self._build_goals()
        self.sub_mode = self.create_subscription(Bool, '/auto_mode', self.cb_mode, 10)
        self.sub_amcl = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.cb_amcl, 10)
        self.timer = self.create_timer(0.1, self.tick)

    def cb_amcl(self, msg: PoseWithCovarianceStamped):
        self.current_pose = PoseStamped()
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose
        self.have_pose = True

    def cb_mode(self, msg: Bool):
        self.auto = bool(msg.data)
        if not self.auto and self.running:
            self.nav.cancelTask()
            self.running = False

    def _build_goals(self):
        def mk(x, y, oz, ow):
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.header.stamp = self.nav.get_clock().now().to_msg()
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.z = oz
            p.pose.orientation.w = ow
            return p
        return [
            mk(4.35,  3.7, 0.277,  0.99),
            mk(9.3,  4.58, 0.168,  0.98),
            mk(16.7, 14.4, 0.0, 0.99),
            mk(-3.15, 18.5, -0.95, 0.3),
            mk(-5.35,  3.0, 0.92,  0.38),
        ]

    def _dist2(self, a: PoseStamped, b: PoseStamped):
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        return dx*dx + dy*dy

    def _ordered_from_nearest(self):
        goals = self._build_goals()
        if not self.have_pose:
            return goals
        i0 = min(range(len(goals)), key=lambda i: self._dist2(goals[i], self.current_pose))
        return goals[i0:] + goals[:i0]

    def _start_follow(self):
        if not self.have_pose:
            return
        ordered = self._ordered_from_nearest()
        if not self.nav.goThroughPoses(ordered):
            return
        self.running = True

    def tick(self):
        if self.auto and not self.running:
            self._start_follow()
            return
        if not self.running:
            return
        if self.nav.isTaskComplete():
            res = self.nav.getResult()
            self.running = False
            if self.auto and res == TaskResult.SUCCEEDED:
                self._start_follow()

def main():
    rclpy.init()
    n = WaypointFollower()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
