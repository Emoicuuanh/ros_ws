#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math
from rclpy.action import ActionClient
from enum import Enum
from twist_mux_msgs.action import JoyTurbo

class State(Enum):
    FREE = 0
    DANGER = 2
    WARNING = 1

class SafetyStop(Node):
    def __init__(self):
        super().__init__("safety_stop_node")
        self.declare_parameter("danger_distance", 1.0)
        self.declare_parameter("warning_distance", 2.0)
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("safety_stop_topic", "safety_stop")
        self.danger_distance = self.get_parameter("danger_distance").get_parameter_value().double_value
        self.warning_distance = self.get_parameter("warning_distance").get_parameter_value().double_value
        self.scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        self.safety_stop_topic = self.get_parameter("safety_stop_topic").get_parameter_value().string_value

        self.laser_sub =self.create_subscription(LaserScan, self.scan_topic, self.laser_callback,10)
        self.safety_stop_pub = self.create_publisher(Bool,self.safety_stop_topic,10)

        self.decrease_speed_client = ActionClient(self, JoyTurbo, "/joy_turbo_decrease")
        self.increase_speed_client = ActionClient(self, JoyTurbo, "/joy_turbo_increase")

        while not self.decrease_speed_client.wait_for_server(timeout_sec=1.0) and rclpy.ok():
            self.get_logger().warn("Action /joy_turbo_decrease not available! Waiting..")

        while not self.increase_speed_client.wait_for_server(timeout_sec=1.0) and rclpy.ok():
            self.get_logger().warn("Action /joy_turbo_increase not available! Waiting..")

        self.state = State.FREE
        self.prev_state = State.FREE
    def laser_callback(self, msg: LaserScan):
        self.state = State.FREE
        valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        min_range = min(valid_ranges) if valid_ranges else float('inf')

    # In khoảng cách nhỏ nhất ra log
        #self.get_logger().info(f"Min range: {min_range:.3f} m | Danger distance: {self.danger_distance:.3f} m")
        for range_value in msg.ranges:
            if not math.isinf(range_value) and range_value <= self.warning_distance:
                self.state = State.WARNING
                if range_value <= self.danger_distance:
                    self.state = State.DANGER
                    break
        is_safety_stop = Bool()
        is_safety_stop.data = (self.state == State.DANGER)

        if self.state != self.prev_state:   # chỉ xử lý khi state thay đổi
            if self.state == State.WARNING:
                is_safety_stop.data = False
                self.decrease_speed_client.send_goal_async(JoyTurbo.Goal())   # Giảm tốc
            elif self.state == State.DANGER:
                is_safety_stop.data = True
                # Dừng hẳn (Safety Stop = True)
            elif self.state == State.FREE:
                is_safety_stop.data = False
                self.increase_speed_client.send_goal_async(JoyTurbo.Goal())   # Tăng tốc
            self.safety_stop_pub.publish(is_safety_stop)   # Publish trạng thái stop

        self.prev_state = self.state
        self.safety_stop_pub.publish(is_safety_stop)


def main():
    rclpy.init()
    node = SafetyStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()
