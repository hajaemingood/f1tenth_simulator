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

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), "launch", "ferrari.launch.py")]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    world_path = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "with_robot.world"
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")]
        ),
        launch_arguments={"world": world_path}.items(),
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "with_robot"],
        output="screen",
    )
    
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(get_package_share_directory(package_name), 
        "config", "drive.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(rviz_enable),
    )

    # === 컨트롤러 스포너 추가 ===
    controllers_yaml = os.path.join(get_package_share_directory(package_name), "config", "controllers.yaml")
    spawner_exec = 'spawner.py' if os.environ.get('ROS_DISTRO','') == 'foxy' else 'spawner'

    # joint_state_broadcaster (권장: 맨 먼저)
    jsb_spawner = Node(
        package="controller_manager",
        executable=spawner_exec,
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    # 앞바퀴 조향(포지션)
    front_steer_spawner = Node(
        package="controller_manager",
        executable=spawner_exec,
        arguments=[
            "front_steer_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    # 앞바퀴 구동(속도)
    # front_drive_spawner = Node(
    #     package="controller_manager",
    #     executable=spawner_exec,
    #     arguments=[
    #         "front_drive_controller",
    #         "--controller-manager", "/controller_manager",
    #     ],
    #     output="screen",
    # )

    # 뒷바퀴 구동(속도)
    rear_drive_spawner = Node(
        package="controller_manager",
        executable=spawner_exec,
        arguments=[
            "rear_drive_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    # 로봇이 스폰된 뒤 컨트롤러 스폰 (spawn_entity 종료 이벤트에 연결)
    spawn_controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                jsb_spawner,
                front_steer_spawner,
                # front_drive_spawner,
                rear_drive_spawner,
            ],
        )
    )

    # Launch them all!
    return LaunchDescription(
        [
            # 런치 인자 선언
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"), 
            
            rsp,
            gazebo,
            spawn_entity,
            spawn_controllers_after_spawn,
            rviz2,
        ]
    )