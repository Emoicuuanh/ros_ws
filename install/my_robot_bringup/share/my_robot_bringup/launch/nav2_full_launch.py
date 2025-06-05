from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Đường dẫn tới các file
    map_path = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_path = LaunchConfiguration('world')

    pkg_gazebo = os.path.join(
        get_package_share_directory('my_robot_bringup')
    )
    pkg_nav2 = os.path.join(
        get_package_share_directory('my_robot_bringup')
    )
    pkg_desc = os.path.join(
        get_package_share_directory('my_robot_description')
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='/home/dung/maps/my_map.yaml',
            description='Full path to map yaml file to load'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_gazebo, 'worlds', 'empty.world'),
            description='Gazebo world file'
        ),

        # Start Gazebo with your world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path}.items()
        ),

        # Start robot state publisher with your URDF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_desc, 'launch', 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # Start Nav2 bringup with AMCL
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_path,
                'use_sim_time': use_sim_time,
                'autostart': 'true'
            }.items()
        )
    ])
