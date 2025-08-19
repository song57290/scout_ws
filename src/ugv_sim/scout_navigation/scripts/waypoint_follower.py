#! /usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robot_navigator import BasicNavigator ,TaskResult 

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower_client')

        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()

        self.auto = False
        self.running = False
        self.amcl_ok = False
        self.goal_poses = self._build_goals()
        self.nav_start = None
        self.i = 0

        self.sub_mode = self.create_subscription(Bool, '/auto_mode', self.cb_mode, 10)
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.cb_amcl, 10
        )
        self.timer = self.create_timer(0.1, self.tick)

    def cb_amcl(self, _msg: PoseWithCovarianceStamped):
        self.amcl_ok = True

    def _build_goals(self):
        def mk(x,y,oz,ow):
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.header.stamp = self.nav.get_clock().now().to_msg()
            p.pose.position.x = x; p.pose.position.y = y; p.pose.position.z = 0.0
            p.pose.orientation.x = 0.0; p.pose.orientation.y = 0.0
            p.pose.orientation.z = oz; p.pose.orientation.w = ow
            return p
        return [
            mk( 10.0,  4.60, 0.23,  0.97),
            mk( 14.0, 14.20, 0.707, 1.5),
            mk( -5.45, 18.0, 0.92, -0.38),
            mk( -5.35, 3.0, 0.92,  0.38),
            mk( 0.0,  1.0, 0.0,   1.0),
        ]

    def cb_mode(self, msg: Bool):
        self.auto = bool(msg.data)
        if not self.auto and self.running:
            self.nav.cancelTask()
            self.running = False

    def _start_follow(self):
        if not self.amcl_ok:
            return
        self.goal_poses = self._build_goals()
        self.nav.goThroughPoses(self.goal_poses) # self.nav.followWaypoints(self.goal_poses)에서 연속성을 위해 변경

        self.running = True
        self.i = 0
        self.nav_start = self.nav.get_clock().now()

    def tick(self):
        if self.auto and not self.running:
            self._start_follow()
            return
        if not self.running:
            return

        if self.nav.isTaskComplete():
            res = self.nav.getResult()
            self.running = False
            # 성공하면 즉시 다음 라운드 시작 (자동 순환)
            if self.auto and res == TaskResult.SUCCEEDED:
                self._start_follow()
            return

        self.i += 1
        # (피드백/타임아웃 로직은 필요시 추가 유지)
        
def main():
    rclpy.init()
    n = WaypointFollower()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
