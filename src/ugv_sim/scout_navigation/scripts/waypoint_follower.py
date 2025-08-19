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
        self.amcl_ok = False
        self.goal_poses = self._build_goals()
        self.reset_pose = self._mk_pose_xy_q(0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
        self.sub_mode = self.create_subscription(Bool, '/auto_mode', self.cb_mode, 10)
        self.sub_amcl = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.cb_amcl, 10)
        self.timer = self.create_timer(0.1, self.tick)

    def cb_amcl(self, _msg: PoseWithCovarianceStamped):
        self.amcl_ok = True

    def _mk_pose_xy_q(self, x, y, qx, qy, qz, qw, frame_id='map'):
        p = PoseStamped()
        p.header.frame_id = frame_id
        p.header.stamp = self.nav.get_clock().now().to_msg()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = 0.0
        p.pose.orientation.x = qx
        p.pose.orientation.y = qy
        p.pose.orientation.z = qz
        p.pose.orientation.w = qw
        return p

    def _build_goals(self):
        mk = self._mk_pose_xy_q
        return [
            mk(9.8, 4.3, 0.0, 0.0, 0.0, 0.99),
            mk(12.0, 5.97, 0.0, 0.0, 0.707, 0.707),
            mk(18.6, 13.7, 0.0, 0.0, 0.707, 0.707),
            mk(-3.8, 18.1, 0.0, 0.0, 0.99, 0.01),
            mk(-5.2, 3.8, 0.0, 0.0, -0.63, 0.77),
        ]

    def cb_mode(self, msg: Bool):
        self.auto = bool(msg.data)
        if not self.auto and self.running:
            self.nav.cancelTask()
            self.running = False

    def _go_to_pose_blocking(self, pose: PoseStamped):
        if not self.nav.goToPose(pose):
            return TaskResult.FAILED
        while not self.nav.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.nav.getResult()

    def _start_follow(self):
        if not self.amcl_ok:
            return
        r = self._go_to_pose_blocking(self.reset_pose)
        if r != TaskResult.SUCCEEDED:
            return
        self.goal_poses = self._build_goals()
        if not self.nav.goThroughPoses(self.goal_poses):
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
                r = self._go_to_pose_blocking(self.reset_pose)
                if self.auto and r == TaskResult.SUCCEEDED:
                    self._start_follow()

def main():
    rclpy.init()
    n = WaypointFollower()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
