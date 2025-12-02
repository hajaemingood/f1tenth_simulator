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

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    default_map_yaml = os.path.join(ferrari_share, "maps", "iccas_track.yaml")

    declare_map = DeclareLaunchArgument(
        "map",
        default_value=default_map_yaml,
        description="AMCL과 map_server에서 사용할 지도 yaml 파일",
    )
    declare_base_frame = DeclareLaunchArgument("base_frame", default_value="base_link")
    declare_odom_frame = DeclareLaunchArgument("odom_frame", default_value="odom")
    declare_scan_topic = DeclareLaunchArgument("scan_topic", default_value="/scan")
    declare_initial_pose_x = DeclareLaunchArgument("initial_pose_x", default_value="-4.771")
    declare_initial_pose_y = DeclareLaunchArgument("initial_pose_y", default_value="-0.639")
    declare_initial_pose_yaw = DeclareLaunchArgument("initial_pose_yaw", default_value="-1.584")


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

    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"base_frame_id": base_frame},
            {"odom_frame_id": odom_frame},
            {"global_frame_id": "map"},
            {"scan_topic": scan_topic},
            {"tf_broadcast": True},
            {"set_initial_pose": True},
            {"alpha1": 0.1},
            {"alpha2": 0.1},
            {"alpha3": 0.1},
            {"alpha4": 0.1},
            # Keep legacy PoseArray particle cloud so RViz PoseArray display works.
            {"use_particle_cloud_message_type": True},
            {"initial_pose": {
                "x": initial_pose_x,
                "y": initial_pose_y,
                "z": 0.0,
                "yaw": initial_pose_yaw,
            }},
        ],
        remappings=[
            ("tf", "tf"),
            ("tf_static", "tf_static"),
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
            {"node_names": ["map_server", "amcl"]},
        ],
    )

    base_link_pub_node = Node(
        package="path_planner",
        executable="base_link_pub",
        name="base_link_pub",
        output="screen",
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
            map_server_node,
            amcl_node,
            lifecycle_manager_node,
            base_link_pub_node,
        ]
    )
