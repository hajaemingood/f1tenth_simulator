# ferrari.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument,RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    package_name = "ferrari"
    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path, "urdf", "ferrari.xacro")
    robot_description = xacro.process_file(xacro_file)

    params = {
        "robot_description": robot_description.toxml(),
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool)
    }

    jount_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[params],
    )
    
    robot_state = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="use sim time"
            ),
            jount_pub,
            robot_state,
        ]
    )
