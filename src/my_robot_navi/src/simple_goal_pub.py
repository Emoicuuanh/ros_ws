import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
import json
import math
import time
import os

FILE_PATH = '/home/dung/waypoints.json'  # phải trùng với file mà node waypoint_recorder lưu

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')

        # Danh sách waypoint
        self.waypoints = self.load_waypoints()
        self.current_index = 0

        # Action client để gửi goal cho Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publisher để xóa marker trên RViz
        self.marker_pub = self.create_publisher(Marker, '/waypoint_markers', 10)

        if not self.waypoints:
            self.get_logger().warn('⚠️ Không có waypoint nào trong file!')
        else:
            self.get_logger().info(f'✅ Đã tải {len(self.waypoints)} waypoint từ file.')

    # ======================================================================
    # 📂 Đọc file JSON
    # ======================================================================
    def load_waypoints(self):
        if not os.path.exists(FILE_PATH):
            self.get_logger().error(f'❌ Không tìm thấy file {FILE_PATH}')
            return []
        with open(FILE_PATH, 'r') as f:
            data = json.load(f)
        return data

    # ======================================================================
    # 🚀 Gửi waypoint kế tiếp
    # ======================================================================
    def send_next_goal(self):
        if self.current_index >= len(self.waypoints):
            self.get_logger().info('🎯 Hoàn thành tất cả waypoint!')
            self.clear_all_waypoints()  # 🧹 Xóa marker + file sau khi xong
            return

        # Lấy điểm hiện tại
        x, y, z = self.waypoints[self.current_index]

        # Tính hướng yaw nếu có điểm kế tiếp
        if self.current_index < len(self.waypoints) - 1:
            next_x, next_y, _ = self.waypoints[self.current_index + 1]
            yaw = math.atan2(next_y - y, next_x - x)
        else:
            yaw = 0.0

        qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)

        # Tạo goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = z
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(f'🚀 Gửi waypoint {self.current_index + 1}/{len(self.waypoints)}: ({x:.2f}, {y:.2f})')

        self.nav_client.wait_for_server()
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_callback)

    # ======================================================================
    # 🎯 Xử lý phản hồi goal
    # ======================================================================
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('❌ Goal bị từ chối!')
            return

        self.get_logger().info('🟢 Goal đã được chấp nhận.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    # ======================================================================
    # ✅ Khi goal hoàn thành
    # ======================================================================
    def result_callback(self, future):
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'✅ Waypoint {self.current_index + 1} hoàn thành!')
            self.current_index += 1
            time.sleep(1.0)  # nghỉ nhẹ 1s trước khi gửi tiếp
            self.send_next_goal()
        else:
            self.get_logger().warn(f'⚠️ Waypoint {self.current_index + 1} thất bại (status={status})')

    # ======================================================================
    # 🧹 Xóa marker + file JSON sau khi hoàn thành
    # ======================================================================
    def clear_all_waypoints(self):
        # 1️⃣ Gửi marker DELETEALL để xóa hết marker khỏi RViz
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header.frame_id = 'map'
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        self.marker_pub.publish(delete_marker)
        self.get_logger().info('🧹 Đã xóa toàn bộ marker khỏi RViz.')

        # 2️⃣ Xóa file JSON
        if os.path.exists(FILE_PATH):
            os.remove(FILE_PATH)
            self.get_logger().info(f'🗑️ Đã xóa file waypoint: {FILE_PATH}')
        else:
            self.get_logger().warn('⚠️ Không tìm thấy file waypoint để xóa.')

# ======================================================================
# 🔧 Main
# ======================================================================
def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()

    if node.waypoints:
        input('Nhấn ENTER để bắt đầu gửi waypoint...\n')
        node.send_next_goal()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
