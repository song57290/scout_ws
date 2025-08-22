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

        # 상태
        self.auto = False               # /auto_mode
        self.recall_active = False      # /recall_active (리콜 중엔 어떤 경우에도 출발 금지)
        self.running = False
        self.have_pose = False
        self.current_pose = None
        self.goals = self._build_goals()

        # 구독
        self.sub_mode   = self.create_subscription(Bool, '/auto_mode', self.cb_mode, 10)
        self.sub_recall = self.create_subscription(Bool, '/recall_active', self.cb_recall, 10)
        self.sub_amcl   = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.cb_amcl, 10)

        # 주기 실행
        self.timer = self.create_timer(0.1, self.tick)

    # ----- 콜백 -----
    def cb_amcl(self, msg: PoseWithCovarianceStamped):
        self.current_pose = PoseStamped()
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose
        self.have_pose = True

    def cb_mode(self, msg: Bool):
        self.auto = bool(msg.data)
        # auto가 꺼지면 즉시 정지
        if (not self.auto) and self.running:
            self.nav.cancelTask()
            self.running = False

    def cb_recall(self, msg: Bool):
        prev = self.recall_active
        self.recall_active = bool(msg.data)
        # 리콜이 시작되면 즉시 정지
        if self.recall_active and self.running:
            self.get_logger().info('[WF] recall_active -> True, cancel current waypoint task')
            self.nav.cancelTask()
            self.running = False
        # 리콜이 끝났고(auto가 켜져 있으면) 자동 재출발은 tick()에서 처리

    # ----- 웨이포인트 구성 -----
    def _build_goals(self):
        pts = [
            (4.35,  3.7),
            (9.3,   4.58),
            (16.7, 14.4),
            (-2.88, 18.46),
            (-5.1, 3.5),
            (-2.54, 1.74),
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

    # ----- 유틸 -----
    def _dist2(self, a: PoseStamped, b: PoseStamped):
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        return dx*dx + dy*dy

    def _ordered_skip_current(self, tol=0.5):
        goals = self._build_goals()
        n = len(goals)
        if not self.have_pose or n == 0:
            return goals
        i_cur = min(range(n), key=lambda i: self._dist2(goals[i], self.current_pose))
        if self._dist2(goals[i_cur], self.current_pose) <= tol * tol:
            return [goals[(i_cur + k) % n] for k in range(1, n)]  # 현재 포인트 제외
        return goals[i_cur:] + goals[:i_cur]

    def _start_follow(self):
        if not self.have_pose:
            return
        ordered = self._ordered_skip_current(tol=0.5)
        if not ordered:
            return
        if not self.nav.goThroughPoses(ordered):
            return
        self.running = True
        self.get_logger().info('[WF] goThroughPoses started.')

    # ----- 메인 루프 -----
    def tick(self):
        # 리콜 중이면 어떤 경우에도 출발/유지하지 않음
        if self.recall_active:
            if self.running:
                self.nav.cancelTask()
                self.running = False
            return

        # 순찰 조건: auto==True AND recall_active==False
        if not self.auto:
            if self.running:
                self.nav.cancelTask()
                self.running = False
            return

        # auto=True, 리콜 아님 → 순찰 시작/유지
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
