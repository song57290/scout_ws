#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

class CmdVelSwitch(Node):
    def __init__(self):
        super().__init__('cmd_vel_switch')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('timeout_sec', 0.5)
        self.declare_parameter('joy_topic', '/cmd_vel_joy')
        self.declare_parameter('nav_topic', '/cmd_vel_nav')
        self.declare_parameter('out_topic', '/cmd_vel')
        rate = float(self.get_parameter('rate_hz').value)
        self.timeout = float(self.get_parameter('timeout_sec').value)
        self.joy_topic = self.get_parameter('joy_topic').value
        self.nav_topic = self.get_parameter('nav_topic').value
        self.out_topic = self.get_parameter('out_topic').value
        qos = QoSProfile(depth=10)
        self.auto_mode = False
        self.last_joy = None
        self.last_nav = None
        self.last_joy_t = self.get_clock().now()
        self.last_nav_t = self.get_clock().now()
        self.pub = self.create_publisher(Twist, self.out_topic, 10)
        self.sub_auto = self.create_subscription(Bool, '/auto_mode', self.on_mode, qos)
        self.sub_joy = self.create_subscription(Twist, self.joy_topic, self.on_joy, qos)
        self.sub_nav = self.create_subscription(Twist, self.nav_topic, self.on_nav, qos)
        self.timer = self.create_timer(1.0/max(1.0, rate), self.tick)

    def on_mode(self, msg: Bool):
        self.auto_mode = bool(msg.data)

    def on_joy(self, msg: Twist):
        self.last_joy = msg
        self.last_joy_t = self.get_clock().now()

    def on_nav(self, msg: Twist):
        self.last_nav = msg
        self.last_nav_t = self.get_clock().now()

    def tick(self):
        now = self.get_clock().now()
        zero = Twist()
        if self.auto_mode:
            if self.last_nav and (now - self.last_nav_t).nanoseconds/1e9 <= self.timeout:
                self.pub.publish(self.last_nav)
            else:
                self.pub.publish(zero)
        else:
            if self.last_joy and (now - self.last_joy_t).nanoseconds/1e9 <= self.timeout:
                self.pub.publish(self.last_joy)
            else:
                self.pub.publish(zero)

def main():
    rclpy.init()
    n = CmdVelSwitch()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
