import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import json
import os

FILE_PATH = '/home/dung/waypoints.json'  # thay đường dẫn nếu cần

class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')

        # --- Danh sách waypoint ---
        self.waypoints = []

        # --- Subscriber: nhận click từ RViz ---
        self.sub_point = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.point_callback,
            10
        )

        # --- Publisher: hiển thị marker lên RViz ---
        self.marker_pub = self.create_publisher(Marker, '/waypoint_markers', 10)

        self.get_logger().info('✅ Click điểm trong RViz (tool "Publish Point") để lưu waypoint!')

    # ========================= =============================================
    # 📍 Khi người dùng click điểm trên RViz
    # ======================================================================
    def point_callback(self, msg):
        p = [msg.point.x, msg.point.y, msg.point.z]
        self.waypoints.append(p)
        self.save_to_file()
        self.get_logger().info(f'📍 Đã lưu waypoint: {p}')
        self.publish_markers()

    # ======================================================================
    # 💾 Lưu waypoint vào file JSON
    # ======================================================================
    def save_to_file(self):
        with open(FILE_PATH, 'w') as f:
            json.dump(self.waypoints, f, indent=2)
        self.get_logger().info(f'💾 Đã lưu {len(self.waypoints)} waypoint vào {FILE_PATH}')

    # ======================================================================
    # 🎨 Hiển thị marker trên RViz
    # ======================================================================
    def publish_markers(self):
        # Marker: các điểm (SPHERE_LIST)
        marker_points = Marker()
        marker_points.header.frame_id = 'map'
        marker_points.header.stamp = self.get_clock().now().to_msg()
        marker_points.ns = 'waypoints'
        marker_points.id = 0
        marker_points.type = Marker.SPHERE_LIST
        marker_points.action = Marker.ADD
        marker_points.scale.x = 0.2
        marker_points.scale.y = 0.2
        marker_points.scale.z = 0.2

        # Marker: đường nối giữa các điểm (LINE_STRIP)
        marker_line = Marker()
        marker_line.header.frame_id = 'map'
        marker_line.header.stamp = self.get_clock().now().to_msg()
        marker_line.ns = 'path'
        marker_line.id = 1
        marker_line.type = Marker.LINE_STRIP
        marker_line.action = Marker.ADD
        marker_line.scale.x = 0.05  # độ dày đường
        marker_line.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # vàng

        for (x, y, z) in self.waypoints:
            pt = Point(x=x, y=y, z=z)
            marker_points.points.append(pt)
            marker_line.points.append(pt)
            marker_points.colors.append(ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0))  # đỏ

        # Publish cả 2 marker
        self.marker_pub.publish(marker_points)
        self.marker_pub.publish(marker_line)

# ======================================================================
# 🔧 Main
# ======================================================================
def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
