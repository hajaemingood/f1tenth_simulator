import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ferrari_share = get_package_share_directory("ferrari")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    base_frame = LaunchConfiguration("base_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    scan_topic = LaunchConfiguration("scan_topic")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    slam_params_file = LaunchConfiguration("slam_params_file")

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    default_map_yaml = os.path.join(ferrari_share, "maps", "iccas_track.yaml")
    default_slam_params = os.path.join(
        get_package_share_directory("slam_toolbox"), "config", "mapper_params_localization.yaml"
    )

    declare_map = DeclareLaunchArgument(
        "map",
        default_value=default_map_yaml,
        description="posegraph과 map_server에서 사용할 지도 yaml 파일",
    )
    declare_base_frame = DeclareLaunchArgument("base_frame", default_value="base_link")
    declare_odom_frame = DeclareLaunchArgument("odom_frame", default_value="odom")
    declare_scan_topic = DeclareLaunchArgument("scan_topic", default_value="/scan")
    declare_initial_pose_x = DeclareLaunchArgument("initial_pose_x", default_value="-4.771")
    declare_initial_pose_y = DeclareLaunchArgument("initial_pose_y", default_value="-0.639")
    declare_initial_pose_yaw = DeclareLaunchArgument("initial_pose_yaw", default_value="-1.584")
    declare_slam_params = DeclareLaunchArgument(
        "slam_params_file",
        default_value=default_slam_params,
        description="slam_toolbox localization parameter file",
    )


    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"yaml_filename": map_yaml},
        ],
    )

    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="localization_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params_file,
            {"use_sim_time": use_sim_time},
            {"odom_frame": odom_frame},
            {"base_frame": base_frame},
            {"scan_topic": scan_topic},
            {"map_start_pose": [initial_pose_x, initial_pose_y, initial_pose_yaw]},
        ],
    )

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )


    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_map,
            declare_base_frame,
            declare_odom_frame,
            declare_scan_topic,
            declare_initial_pose_x,
            declare_initial_pose_y,
            declare_initial_pose_yaw,
            declare_slam_params,
            map_server_node,
            slam_toolbox_node,
            lifecycle_manager_node,
        ]
    )
