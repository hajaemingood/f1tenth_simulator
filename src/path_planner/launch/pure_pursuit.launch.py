import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share_dir = get_package_share_directory("path_planner")
    waypoints_csv = os.path.join(share_dir, "data", "waypoints.csv")

    pure_pursuit_node = Node(
        package="path_planner",
        executable="pure_pursuit",
        name="pure_pursuit",
        output="screen",
        parameters=[{"waypoints_csv": waypoints_csv}],
    )

    base_link_pub_node = Node(
        package="path_planner",
        executable="base_link_pub",
        name="base_link_pub",
        output="screen",
    )

    waypoint_viz_node = Node(
        package="path_planner",
        executable="waypoint_visualization",
        name="waypoint_visualization",
        output="screen",
        parameters=[{"waypoints_csv": waypoints_csv}],
    )

    return LaunchDescription(
        [
            pure_pursuit_node,
            TimerAction(period=2.0, actions=[base_link_pub_node]),
            TimerAction(period=4.0, actions=[waypoint_viz_node]),
        ]
    )
