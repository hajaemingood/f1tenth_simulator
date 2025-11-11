import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_name = "ferrari"
    use_sim_time = LaunchConfiguration("use_sim_time")
    frame_suffix = LaunchConfiguration("frame_suffix")
    robot_ros_namespace = LaunchConfiguration("robot_ros_namespace")

    pkg_path = get_package_share_directory(package_name)
    xacro_file = os.path.join(pkg_path, "urdf", "ferrari_op.xacro")

    robot_description_cmd = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
            " ",
            "frame_suffix:=",
            frame_suffix,
            " ",
            "ros_namespace:=",
            robot_ros_namespace,
        ]
    )

    params = {
        "robot_description": ParameterValue(robot_description_cmd, value_type=str),
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
    }

    joint_pub = Node(
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
            DeclareLaunchArgument(
                "frame_suffix", default_value="_op", description="Suffix applied to joint/link names."
            ),
            DeclareLaunchArgument(
                "robot_ros_namespace",
                default_value="",
                description="Namespace applied to this robot instance.",
            ),
            joint_pub,
            robot_state,
        ]
    )
