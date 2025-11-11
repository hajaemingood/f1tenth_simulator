import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def _robot_group(
    *,
    namespace: str,
    description_launch: IncludeLaunchDescription,
    spawn_arguments,
    controllers_file: str,
    use_sim_time,
):
    spawner_exec = "spawner.py" if os.environ.get("ROS_DISTRO", "") == "foxy" else "spawner"

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=spawn_arguments,
        output="screen",
    )

    controller_manager_path = f"/{namespace}/controller_manager"

    jsb_spawner = Node(
        package="controller_manager",
        executable=spawner_exec,
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager_path,
            "-p",
            controllers_file,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    f1_spawner = Node(
        package="controller_manager",
        executable=spawner_exec,
        arguments=[
            "f1tenth_commands_controller",
            "--controller-manager",
            controller_manager_path,
            "-p",
            controllers_file,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    spawn_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[jsb_spawner, f1_spawner],
        )
    )

    return GroupAction(
        [
            PushRosNamespace(namespace),
            description_launch,
            spawn_entity,
            spawn_controllers,
        ]
    )


def generate_launch_description():
    ferrari_pkg_path = get_package_share_directory("ferrari")

    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_enable = LaunchConfiguration("rviz")

    world_path = os.path.join(ferrari_pkg_path, "config", "with_robot.world")
    gazebo_pkg_path = get_package_share_directory("gazebo_ros")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(gazebo_pkg_path, "launch", "gazebo.launch.py")]
        ),
        launch_arguments={"world": world_path}.items(),
    )

    ferrari_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(ferrari_pkg_path, "launch", "ferrari.launch.py")]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "ros_namespace": "",
        }.items(),
    )

    ferrari_op_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(ferrari_pkg_path, "launch", "ferrari_op.launch.py")]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "ros_namespace": "",
            "frame_suffix": "_op",
        }.items(),
    )

    ferrari_group = _robot_group(
        namespace="ferrari",
        description_launch=ferrari_description,
        spawn_arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "ferrari",
            "-robot_namespace",
            "ferrari",
            "-x",
            LaunchConfiguration("ferrari_spawn_x"),
            "-y",
            LaunchConfiguration("ferrari_spawn_y"),
            "-z",
            LaunchConfiguration("ferrari_spawn_z"),
            "-R",
            LaunchConfiguration("ferrari_spawn_roll"),
            "-P",
            LaunchConfiguration("ferrari_spawn_pitch"),
            "-Y",
            LaunchConfiguration("ferrari_spawn_yaw"),
        ],
        controllers_file=os.path.join(ferrari_pkg_path, "config", "controllers.yaml"),
        use_sim_time=use_sim_time,
    )

    ferrari_op_group = _robot_group(
        namespace="ferrari_op",
        description_launch=ferrari_op_description,
        spawn_arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "ferrari_op",
            "-robot_namespace",
            "ferrari_op",
            "-x",
            LaunchConfiguration("ferrari_op_spawn_x"),
            "-y",
            LaunchConfiguration("ferrari_op_spawn_y"),
            "-z",
            LaunchConfiguration("ferrari_op_spawn_z"),
            "-R",
            LaunchConfiguration("ferrari_op_spawn_roll"),
            "-P",
            LaunchConfiguration("ferrari_op_spawn_pitch"),
            "-Y",
            LaunchConfiguration("ferrari_op_spawn_yaw"),
        ],
        controllers_file=os.path.join(ferrari_pkg_path, "config", "controllers_op.yaml"),
        use_sim_time=use_sim_time,
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            os.path.join(ferrari_pkg_path, "config", "drive.rviz"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(rviz_enable),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("ferrari_spawn_x", default_value="0.0"),
            DeclareLaunchArgument("ferrari_spawn_y", default_value="0.5"),
            DeclareLaunchArgument("ferrari_spawn_z", default_value="0.05"),
            DeclareLaunchArgument("ferrari_spawn_roll", default_value="0.0"),
            DeclareLaunchArgument("ferrari_spawn_pitch", default_value="0.0"),
            DeclareLaunchArgument("ferrari_spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("ferrari_op_spawn_x", default_value="1.0"),
            DeclareLaunchArgument("ferrari_op_spawn_y", default_value="-0.5"),
            DeclareLaunchArgument("ferrari_op_spawn_z", default_value="0.05"),
            DeclareLaunchArgument("ferrari_op_spawn_roll", default_value="0.0"),
            DeclareLaunchArgument("ferrari_op_spawn_pitch", default_value="0.0"),
            DeclareLaunchArgument("ferrari_op_spawn_yaw", default_value="0.0"),
            gazebo,
            ferrari_group,
            ferrari_op_group,
            rviz2,
        ]
    )
