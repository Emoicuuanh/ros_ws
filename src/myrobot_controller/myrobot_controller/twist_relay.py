#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistRelay(Node):
    def __init__(self):
        super().__init__("twist_relay")
        self.controller_sub = self.create_subscription(
            Twist,
            "my_robot_controller/cmd_vel_unstamped",
            self.controller_twist_callback,
            10
            )
        self.controller_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
            )
        self.key_sub = self.create_subscription(
            TwistStamped,
            "/input_key/cmd_vel_stamped",
            self.joy_twist_callback,
            10
            )
        self.key_pub = self.create_publisher(
            Twist,
            "/input_key/cmd_vel",
            10
            )
    def controller_twist_callback(self, msg: Twist):
        twist_ = Twist()
        twist_ = msg
        self.controller_pub.publish(twist_)
    def joy_twist_callback(self, msg: TwistStamped):
        twist = Twist()
        twist = msg.twist
        self.key_pub.publish(twist)


def main():
    rclpy.init()
    node = TwistRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()