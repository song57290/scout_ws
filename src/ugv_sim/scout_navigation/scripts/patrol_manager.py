#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64MultiArray
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import math
import time


def q_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class PatrolManager(Node):
    def __init__(self):
        super().__init__("patrol_manager")

        # Nav2 helper
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        # Fire detection subscriber
        self.fire_detected = False
        self.create_subscription(Bool, "/fire_detected", self.fire_cb, 10)

        # Gripper publisher
        self.grip_pub = self.create_publisher(Float64MultiArray, "/gripper_controller/commands", 10)

        # Waypoints (waypoint_follower.py 내용 가져옴)
        self.goal_poses = []
        self._init_waypoints()

        # Extinguisher location
        self.extinguisher_pose = self.make_pose(-2.0, 2.0, 0.0)  # 원하는 좌표(yaml로 뺄 수 있음)

        # 실행 상태
        self.state = "PATROL"
        self.current_index = 0

        self.get_logger().info("Patrol Manager started. Beginning patrol...")
        self.start_patrol()

    def _init_waypoints(self):
        # === waypoint_follower.py 의 goal_poses 그대로 복사 ===
        goals = [
            (1.3, 6.0, 0.23, 0.97),
            (2.0, -3.5, 0.707, -0.707),
            (1.5, -7.7, 0.92, -0.38),
            (-1.4, -7.8, 0.92, 0.38),
            (-2.6, -4.5, 0.38, 0.92),
            (0.0, 0.0, 0.0, 1.0),
        ]
        for g in goals:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            pose.pose.position.x = g[0]
            pose.pose.position.y = g[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = g[2]
            pose.pose.orientation.w = g[3]
            self.goal_poses.append(pose)

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        _, _, z, w = q_from_yaw(yaw)
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        return pose

    def fire_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().warn("Fire detected! Switching to extinguisher mission.")
            self.fire_detected = True
            self.navigator.cancelTask()

    def start_patrol(self):
        self.state = "PATROL"
        self.navigator.followWaypoints(self.goal_poses)

    def patrol_loop(self):
        if self.state == "PATROL":
            if not self.navigator.isTaskComplete():
                return
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("route completed, restarting...")
                self.start_patrol()

        elif self.state == "FIRE":
            self.get_logger().info("Navigating to extinguisher...")
            self.navigator.goToPose(self.extinguisher_pose)
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.1)

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("Arrived at extinguisher. Grabbing...")
                self.use_gripper()
                self.fire_detected = False
                self.start_patrol()

    def use_gripper(self):
        # 예시: open → close
        msg = Float64MultiArray()
        msg.data = [0.10, 0.0]  # 열기
        self.grip_pub.publish(msg)
        time.sleep(1.0)
        msg.data = [0.10, 0.8]  # 닫기
        self.grip_pub.publish(msg)
        self.get_logger().info("Gripper closed to grab extinguisher.")

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.fire_detected and self.state != "FIRE":
                self.state = "FIRE"
            self.patrol_loop()


def main():
    rclpy.init()
    node = PatrolManager()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
