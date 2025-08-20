#! /usr/bin/env python3
import rclpy, math
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
        pts = [
            (4.35,  3.7),
            (9.3,   4.58),
            (16.7, 14.4),
            (-3.15, 18.5),
            (-5.35, 3.0),
            (-1.46, 1.35),
        ]
        goals = []
        n = len(pts)
        now = self.nav.get_clock().now().to_msg()
        for i, (x, y) in enumerate(pts):
            nx, ny = pts[(i + 1) % n]
            yaw = math.atan2(ny - y, nx - x)
            oz = math.sin(yaw / 2.0)
            ow = math.cos(yaw / 2.0)
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.header.stamp = now
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.z = oz
            p.pose.orientation.w = ow
            goals.append(p)
        return goals

    def _dist2(self, a: PoseStamped, b: PoseStamped):
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        return dx*dx + dy*dy

    def _ordered_skip_current(self, tol=0.5):
        goals = self._build_goals()
        n = len(goals)
        if not self.have_pose or n == 0:
            return goals
        # 현재와 가장 가까운 인덱스
        i_cur = min(range(n), key=lambda i: self._dist2(goals[i], self.current_pose))
        # 현재에 충분히 가까우면 그 포인트는 제외하고, 다음 포인트부터 n-1개만 사용
        if self._dist2(goals[i_cur], self.current_pose) <= tol * tol:
            return [goals[(i_cur + k) % n] for k in range(1, n)]  # 현재(i_cur)는 제외
        # 멀리 있으면 전체를 i_cur부터 시작 (마지막이 현재가 아님)
        return goals[i_cur:] + goals[:i_cur]

    def _start_follow(self):
        if not self.have_pose:
            return
        ordered = self._ordered_skip_current(tol=0.5)
        if not ordered:  # 안전장치
            return
        if not self.nav.goThroughPoses(ordered):
            return
        self.running = True


    # waypoint를 순서대로 사용하고 싶으면 이걸 사용
    # def _start_follow(self):
    #     if not self.have_pose:
    #         return
    #     ordered = self._ordered_from_nearest()
    #     if not self.nav.goThroughPoses(ordered):
    #         return
    #     self.running = True

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
