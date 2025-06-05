from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration


def noisy_controller(context , *args ,**kwargs):
    wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
    wheel_serapation = float(LaunchConfiguration("wheel_serapation").perform(context))
    wheel_radius_error = float(LaunchConfiguration("wheel_radius_error").perform(context))
    wheel_serapation_error = float(LaunchConfiguration("wheel_serapation_error").perform(context))

    noisy_controller_py = Node(
        package="myrobot_controller",
        executable= "noisy_controller.py",
        parameters= [{
            "wheel_radius": wheel_radius+ wheel_radius_error,
            "wheel_serapation" : wheel_serapation +wheel_radius_error

        }]
    )
    return[
        noisy_controller_py
    ]
def generate_launch_description():

    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.1"
    )

    wheel_serapation_arg = DeclareLaunchArgument(
        "wheel_serapation",
        default_value="0.4"
    )

    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.005"
    )

    wheel_serapation_error_arg = DeclareLaunchArgument(
        "wheel_serapation_error",
        default_value="0.01"
    )

    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_serapation = LaunchConfiguration("wheel_serapation")

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    simple_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "simple_velocity_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    simple_controller_py = Node(
        package="myrobot_controller",
        executable="simple_controller.py",
        parameters=[{
            "wheel_radius": wheel_radius,
            "wheel_serapation": wheel_serapation
        }]
    )

    noisy_controller_launch= OpaqueFunction(function= noisy_controller)
    return LaunchDescription([
        wheel_radius_arg,
        wheel_serapation_arg,
        wheel_serapation_error_arg,
        wheel_radius_error_arg,
        joint_state_broadcaster_spawner,
        simple_controller,
        simple_controller_py,
        noisy_controller_launch
    ])
