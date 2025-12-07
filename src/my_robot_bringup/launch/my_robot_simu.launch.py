from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
import os
from launch_ros.actions import Node
from launch.conditions import UnlessCondition,IfCondition

def generate_launch_description():
    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value= "false"
    )
    map_yaml_arg = DeclareLaunchArgument(
    "map_yaml",
    default_value="ttt.yaml"
    )
    map_yaml = LaunchConfiguration("map_yaml")

    use_slam = LaunchConfiguration("use_slam")
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("my_robot_description"),
            "launch",
            "gazebo.launch.py"
        )
    )
    controller = IncludeLaunchDescription(
    os.path.join(
        get_package_share_directory("myrobot_controller"),
        "launch",
        "my_robot_controller.launch.py"
    ), 
    )
    navigation = IncludeLaunchDescription(
    os.path.join(
        get_package_share_directory("my_robot_navi"),
        "launch",
        "navigation.launch.py"
    ), 
    )
    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("my_robot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition = UnlessCondition(use_slam),
        launch_arguments={
        "map_yaml_name": map_yaml
        }.items()
    )
    key_board = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("myrobot_controller"),
            "launch",
            "keyboard.launch.py"
        ),
    )
    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("my_robot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition = IfCondition(use_slam)
    )
    safety_stop = Node(
        package="my_robot_utils",
        executable="safety_stop.py",
        output = "screen"
    )
    goal_get = Node(
        package="my_robot_navi",
        executable="simple_goal_get.py",
        output = "screen"
    )
    goal_pub = Node(
        package="my_robot_navi",
        executable="simple_goal_pub.py",
        output = "screen"
    )
    rviz_config_path = os.path.join(
        get_package_share_directory("my_robot_bringup"),
        "rviz",
        "urdf_config.rviz"  # Đường dẫn đến file .rviz của bạn
    )
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen"
    )
    return LaunchDescription([
        use_slam_arg,
        map_yaml_arg,
        gazebo,
        controller,
        localization,
        slam,
        key_board,
        safety_stop,
        navigation,
        rviz2,
        goal_get,
        goal_pub
    ])



