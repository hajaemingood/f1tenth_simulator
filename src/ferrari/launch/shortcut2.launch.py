import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    package_name = "ferrari"

    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_enable  = LaunchConfiguration("rviz")

    # ▶ 스폰 포즈/이름 인자 (기본값 = 네가 준 포즈)
    spawn_x     = LaunchConfiguration("spawn_x")
    spawn_y     = LaunchConfiguration("spawn_y")
    spawn_z     = LaunchConfiguration("spawn_z")
    spawn_roll  = LaunchConfiguration("spawn_roll")
    spawn_pitch = LaunchConfiguration("spawn_pitch")
    spawn_yaw   = LaunchConfiguration("spawn_yaw")

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), "launch", "ferrari.launch.py")]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    world_path = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "shortcut2.world"  
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")]
        ),
        launch_arguments={"world": world_path}.items(),
    )

    # ▶ 스폰: 포즈 + 자동 리네이밍
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", 'with_robot',
            "-x", spawn_x, "-y", spawn_y, "-z", spawn_z,
            "-R", spawn_roll, "-P", spawn_pitch, "-Y", spawn_yaw,
        ],
        output="screen",
    )
    
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(get_package_share_directory(package_name), "config", "drive.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(rviz_enable),
    )

    controllers_yaml = os.path.join(get_package_share_directory(package_name), "config", "controllers.yaml")
    spawner_exec = 'spawner.py' if os.environ.get('ROS_DISTRO','') == 'foxy' else 'spawner'

    jsb_spawner = Node(
        package="controller_manager",
        executable=spawner_exec,
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    f1_spawner = Node(
        package="controller_manager",
        executable=spawner_exec,
        arguments=["f1tenth_commands_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    spawn_controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[jsb_spawner, f1_spawner],
        )
    )

    return LaunchDescription(
        [
            # 기본 인자
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),

            # ▶ 스폰 포즈 기본값: 네가 보내준 값
            DeclareLaunchArgument("entity",      default_value="with_robot"),
            DeclareLaunchArgument("spawn_x",     default_value="15.183058"),
            DeclareLaunchArgument("spawn_y",     default_value="-4.169753"),
            DeclareLaunchArgument("spawn_z",     default_value="0.03"),
            DeclareLaunchArgument("spawn_roll",  default_value="0.00"),
            DeclareLaunchArgument("spawn_pitch", default_value="0.000003"),
            DeclareLaunchArgument("spawn_yaw",   default_value="1.665918"),

            rsp,
            gazebo,
            spawn_entity,
            spawn_controllers_after_spawn,
            rviz2,
        ]
    )
