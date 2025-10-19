from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    # 1️⃣ Tạo tham số lựa chọn ngôn ngữ (Python hoặc C++)
    use_python_arg = DeclareLaunchArgument(
        "use_python",                # tên tham số
        default_value="True",       # mặc định dùng C++
    )

    # 2️⃣ Lấy giá trị tham số khi chạy launch
    use_python = LaunchConfiguration("use_python")

    # 3️⃣ Node static_transform_publisher để phát TF tĩnh
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0.103",
            "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
            "--frame-id", "base_footprint_ekf",
            "--child-frame-id", "imu_link_ekf"
        ]
    )

    # 4️⃣ Node robot_localization chạy EKF
    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("my_robot_localization"),
                "config",
                "ekf.yaml"
            )
        ]
    )

    # 5️⃣ Node imu_republisher viết bằng Python, chỉ chạy nếu use_python=True
    imu_republisher_py = Node(
        package="my_robot_localization",
        executable="imu_republisher.py"
    )
    # 7️⃣ Trả về danh sách các node trong LaunchDescription
    return LaunchDescription([
        use_python_arg,               # khai báo tham số
        static_transform_publisher,   # node TF tĩnh
        robot_localization,           # node EKF
        imu_republisher_py,           # node Python (nếu True)
    ])
