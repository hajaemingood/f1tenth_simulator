import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _amcl_node(
    *,
    name: str,
    use_sim_time,
    map_yaml,
    base_frame,
    odom_frame,
    scan_topic,
    initial_pose_x,
    initial_pose_y,
    initial_pose_yaw,
) -> Node:
    return Node(
        package="nav2_amcl",
        executable="amcl",
        name=name,
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"base_frame_id": base_frame},
            {"odom_frame_id": odom_frame},
            {"global_frame_id": "map"},
            {"scan_topic": scan_topic},
            {"tf_broadcast": True},
            {"set_initial_pose": True},
            {"alpha1": 0.2},
            {"alpha2": 0.2},
            {"alpha3": 0.2},
            {"alpha4": 0.2},
            {"use_particle_cloud_message_type": True},
            {
                "initial_pose": {
                    "x": initial_pose_x,
                    "y": initial_pose_y,
                    "z": 0.0,
                    "yaw": initial_pose_yaw,
                }
            },
        ],
        remappings=[
            ("tf", "tf"),
            ("tf_static", "tf_static"),
        ],
    )


def generate_launch_description() -> LaunchDescription:
    ferrari_share = get_package_share_directory("ferrari")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")

    ferrari_base_frame = LaunchConfiguration("ferrari_base_frame")
    ferrari_odom_frame = LaunchConfiguration("ferrari_odom_frame")
    ferrari_scan_topic = LaunchConfiguration("ferrari_scan_topic")
    ferrari_init_x = LaunchConfiguration("ferrari_initial_pose_x")
    ferrari_init_y = LaunchConfiguration("ferrari_initial_pose_y")
    ferrari_init_yaw = LaunchConfiguration("ferrari_initial_pose_yaw")

    ferrari_op_base_frame = LaunchConfiguration("ferrari_op_base_frame")
    ferrari_op_odom_frame = LaunchConfiguration("ferrari_op_odom_frame")
    ferrari_op_scan_topic = LaunchConfiguration("ferrari_op_scan_topic")
    ferrari_op_init_x = LaunchConfiguration("ferrari_op_initial_pose_x")
    ferrari_op_init_y = LaunchConfiguration("ferrari_op_initial_pose_y")
    ferrari_op_init_yaw = LaunchConfiguration("ferrari_op_initial_pose_yaw")

    default_map_yaml = os.path.join(ferrari_share, "maps", "iccas_track.yaml")

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    declare_map = DeclareLaunchArgument(
        "map",
        default_value=default_map_yaml,
        description="AMCL과 map_server에서 사용할 지도 yaml 파일",
    )

    declare_ferrari_base_frame = DeclareLaunchArgument(
        "ferrari_base_frame", default_value="base_link"
    )
    declare_ferrari_odom_frame = DeclareLaunchArgument(
        "ferrari_odom_frame", default_value="odom"
    )
    declare_ferrari_scan_topic = DeclareLaunchArgument(
        "ferrari_scan_topic", default_value="/ferrari/scan"
    )
    declare_ferrari_init_x = DeclareLaunchArgument(
        "ferrari_initial_pose_x", default_value="-4.771"
    )
    declare_ferrari_init_y = DeclareLaunchArgument(
        "ferrari_initial_pose_y", default_value="-0.639"
    )
    declare_ferrari_init_yaw = DeclareLaunchArgument(
        "ferrari_initial_pose_yaw", default_value="-1.584"
    )

    declare_ferrari_op_base_frame = DeclareLaunchArgument(
        "ferrari_op_base_frame", default_value="base_link_op"
    )
    declare_ferrari_op_odom_frame = DeclareLaunchArgument(
        "ferrari_op_odom_frame", default_value="odom_op"
    )
    declare_ferrari_op_scan_topic = DeclareLaunchArgument(
        "ferrari_op_scan_topic", default_value="/ferrari_op/scan"
    )
    declare_ferrari_op_init_x = DeclareLaunchArgument(
        "ferrari_op_initial_pose_x", default_value="-0.623"
    )
    declare_ferrari_op_init_y = DeclareLaunchArgument(
        "ferrari_op_initial_pose_y", default_value="-0.058"
    )
    declare_ferrari_op_init_yaw = DeclareLaunchArgument(
        "ferrari_op_initial_pose_yaw", default_value="1.565"
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

    amcl_ferrari = _amcl_node(
        name="amcl_ferrari",
        use_sim_time=use_sim_time,
        map_yaml=map_yaml,
        base_frame=ferrari_base_frame,
        odom_frame=ferrari_odom_frame,
        scan_topic=ferrari_scan_topic,
        initial_pose_x=ferrari_init_x,
        initial_pose_y=ferrari_init_y,
        initial_pose_yaw=ferrari_init_yaw,
    )

    amcl_ferrari_op = _amcl_node(
        name="amcl_ferrari_op",
        use_sim_time=use_sim_time,
        map_yaml=map_yaml,
        base_frame=ferrari_op_base_frame,
        odom_frame=ferrari_op_odom_frame,
        scan_topic=ferrari_op_scan_topic,
        initial_pose_x=ferrari_op_init_x,
        initial_pose_y=ferrari_op_init_y,
        initial_pose_yaw=ferrari_op_init_yaw,
    )

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["map_server", "amcl_ferrari", "amcl_ferrari_op"]},
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_map,
            declare_ferrari_base_frame,
            declare_ferrari_odom_frame,
            declare_ferrari_scan_topic,
            declare_ferrari_init_x,
            declare_ferrari_init_y,
            declare_ferrari_init_yaw,
            declare_ferrari_op_base_frame,
            declare_ferrari_op_odom_frame,
            declare_ferrari_op_scan_topic,
            declare_ferrari_op_init_x,
            declare_ferrari_op_init_y,
            declare_ferrari_op_init_yaw,
            map_server_node,
            amcl_ferrari,
            amcl_ferrari_op,
            lifecycle_manager_node,
        ]
    )
