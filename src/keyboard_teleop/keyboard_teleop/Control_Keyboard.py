import sys
import termios
import tty
import rclpy
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Time

# Thiết lập tốc độ cố định
LINEAR_SPEED = 1    # m/s
ANGULAR_SPEED = 0.5  # rad/s

# Điều khiển theo phím
moveBindings = {
    'w': (LINEAR_SPEED, 0.0),    # Tiến
    's': (-LINEAR_SPEED, 0.0),   # Lùi
    'a': (0.0, ANGULAR_SPEED),   # Rẽ trái
    'd': (0.0, -ANGULAR_SPEED),  # Rẽ phải
}

def get_key():
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = rclpy.create_node('simple_teleop')
    pub = node.create_publisher(Twist, "/cmd_vel", 20)

    print("Điều khiển robot: w (tiến), s (lùi), a (trái), d (phải), x (dừng). Nhấn Ctrl+C để thoát.")

    try:
        while True:
            key = get_key()
            twist_ = Twist()
            if key in moveBindings:
                twist_.linear.x = float(moveBindings[key][0])
                twist_.angular.z = float(moveBindings[key][1])
            elif key == 'x':  # Dừng robot
                twist_.linear.x = 0.0
                twist_.angular.z = 0.0
                print("Dừng robot")
            elif key == '\x03':  # Ctrl+C
                break
            else:
                continue  # bỏ qua các phím khác

            pub.publish(twist_)
            print(f"Gửi: linear.x={twist_.linear.x}, angular.z={twist_.angular.z}")

    except Exception as e:
        print(f"Lỗi: {e}")

    finally:
        twist_ = Twist()  # Dừng robot khi thoát
        twist_.linear.x = 0.0
        twist_.angular.z = 0.0
        pub.publish(twist_)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
