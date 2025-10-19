from launch import LaunchDescription 
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription 
from launch.substitutions import LaunchConfiguration 
from launch_ros.actions import Node 
import os 
from ament_index_python.packages import get_package_share_directory 
def generate_launch_description(): 
    use_sim_time_arg = DeclareLaunchArgument( 
        "use_sim_time", 
        default_value="true" 
        )
    my_robot_controller_pkg = get_package_share_directory("myrobot_controller") 

    twist_mux_launch = IncludeLaunchDescription( 
        os.path.join( 
            get_package_share_directory("twist_mux"),
            "launch", 
            "twist_mux_launch.py"), 
        launch_arguments = { 
            "cmd_vel_out" :"/my_robot_controller/cmd_vel_unstamped", 
            "config_topics":os.path.join(my_robot_controller_pkg,"config","twist_mux_topics.yaml"), 
            "config_locks":os.path.join(my_robot_controller_pkg,"config","twist_mux_locks.yaml"), 
            "config_key":os.path.join(my_robot_controller_pkg,"config","twist_mux_key.yaml"), 
            "use_sim_time": LaunchConfiguration("use_sim_time") 
            }.items() 
        ) 
    twist_relay_node = Node( 
        package="myrobot_controller", 
        executable="twist_relay.py", 
        name="twist_relay", 
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}] 
    ) 
    return LaunchDescription([ 
        use_sim_time_arg, 
        twist_mux_launch, 
        twist_relay_node 
    ])