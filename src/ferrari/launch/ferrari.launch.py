import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_path = os.path.join(get_package_share_directory("ferrari"))
    xacro_file = os.path.join(pkg_path, "urdf", "ferrari.xacro")
    robot_description = xacro.process_file(xacro_file)

    params = {
        "robot_description": robot_description.toxml(),
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool)
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="use sim time"
            ),

            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen"
            ),
            
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[params],
            ),
            # # temp
            # Node(
            #     package='tf2_ros',
            #     executable='static_transform_publisher',
            #     name='static_tf_pub_map_to_base',
            #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
            #     output='screen'
            # ),
        ]
    )
