from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import DeclareLaunchArgument,OpaqueFunction,GroupAction,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition,UnlessCondition
from ament_index_python.packages import get_package_share_directory

# def noisy_controller(context , *args ,**kwargs):
#     wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
#     wheel_serapation = float(LaunchConfiguration("wheel_serapation").perform(context))
#     wheel_radius_error = float(LaunchConfiguration("wheel_radius_error").perform(context))
#     wheel_serapation_error = float(LaunchConfiguration("wheel_serapation_error").perform(context))

#     noisy_controller_py = Node(
#         package="myrobot_controller",
#         executable= "noisy_controller.py",
#         parameters= [{
#             "wheel_radius": wheel_radius+ wheel_radius_error,
#             "wheel_serapation" : wheel_serapation + wheel_serapation_error

#         }]
#     )
#     return[
#         noisy_controller_py
#     ]
def generate_launch_description():

    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.04"
    )

    wheel_serapation_arg = DeclareLaunchArgument(
        "wheel_serapation",
        default_value="0.37"
    )

    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.0005"
    )

    wheel_serapation_error_arg = DeclareLaunchArgument(
        "wheel_serapation_error",
        default_value="0.0005"
    )

    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="true"
    )
    my_robot_controller_pkg = get_package_share_directory("myrobot_controller")

    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_serapation = LaunchConfiguration("wheel_serapation")
    use_simple_controller =LaunchConfiguration("use_simple_controller")
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )
    
    wheel_controller_spwaner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "my_robot_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        condition=UnlessCondition(use_simple_controller)
    )

    simple_controller = GroupAction(
        condition = IfCondition(use_simple_controller),
        actions=[    
            Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "simple_velocity_controller",
                "--controller-manager",
                "/controller_manager"
            ]
            ),
            Node(
            package="myrobot_controller",
            executable="simple_controller.py",
            parameters=[{
                "wheel_radius": wheel_radius,
                "wheel_serapation": wheel_serapation
            }]
            )
    ]
    )

    # twist_mux_launch = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("twist_mux"),
    #         "launch",
    #         "twist_mux_launch.py"),
    #     launch_arguments = {
    #         "cmd_vel_out" :"/my_robot_controller/cmd_vel_unstamped",
    #         "config_topics":os.path.join(my_robot_controller_pkg,"config","twist_mux_topics.yaml"),
    #         "config_locks":os.path.join(my_robot_controller_pkg,"config","twist_mux_locks.yaml"),
    #         "config_key":os.path.join(my_robot_controller_pkg,"config","twist_mux_key.yaml"),
    #         "use_sim_time": LaunchConfiguration("use_sim_time")
    #     }.items()
    #     )
    # twist_relay_node = Node(
    #     package="myrobot_controller",
    #     executable="twist_relay.py",
    #     name="twist_relay",
    #     parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    # )

    # noisy_controller_launch= OpaqueFunction(function= noisy_controller)
    return LaunchDescription([
        wheel_radius_arg,
        wheel_serapation_arg,
        wheel_serapation_error_arg,
        wheel_radius_error_arg,
        use_simple_controller_arg,
        joint_state_broadcaster_spawner,
        simple_controller,
        wheel_controller_spwaner,
        # noisy_controller_launch,
        # twist_mux_launch,
        # twist_relay_node
    ])
