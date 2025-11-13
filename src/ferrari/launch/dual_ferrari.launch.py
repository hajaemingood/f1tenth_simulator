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

# 1
def _robot_group(
    *,
    namespace: str,
    launch_path: str,
    extra_launch_args: dict,
    spawn_pose_args: dict,
    use_sim_time,
):
    spawner_exec = "spawner.py" if os.environ.get("ROS_DISTRO", "") == "foxy" else "spawner"

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_path]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot_ros_namespace": namespace,
            **extra_launch_args,
        }.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            namespace,
            "-robot_namespace",
            namespace,
            "-x",
            spawn_pose_args["x"],
            "-y",
            spawn_pose_args["y"],
            "-z",
            spawn_pose_args["z"],
            "-R",
            spawn_pose_args["roll"],
            "-P",
            spawn_pose_args["pitch"],
            "-Y",
            spawn_pose_args["yaw"],
        ],
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

    world_path = os.path.join(ferrari_pkg_path, "config", "iccas_track.world")
    gazebo_pkg_path = get_package_share_directory("gazebo_ros")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(gazebo_pkg_path, "launch", "gazebo.launch.py")]
        ),
        launch_arguments={"world": world_path}.items(),
    )

    ferrari_group = _robot_group(
        namespace="ferrari",
        launch_path=os.path.join(ferrari_pkg_path, "launch", "ferrari.launch.py"),
        extra_launch_args={},
        spawn_pose_args={
            "x": LaunchConfiguration("ferrari_spawn_x"),
            "y": LaunchConfiguration("ferrari_spawn_y"),
            "z": LaunchConfiguration("ferrari_spawn_z"),
            "roll": LaunchConfiguration("ferrari_spawn_roll"),
            "pitch": LaunchConfiguration("ferrari_spawn_pitch"),
            "yaw": LaunchConfiguration("ferrari_spawn_yaw"),
        },
        use_sim_time=use_sim_time,
    )

    ferrari_op_group = _robot_group(
        namespace="ferrari_op",
        launch_path=os.path.join(ferrari_pkg_path, "launch", "ferrari_op.launch.py"),
        extra_launch_args={"frame_suffix": "_op"},
        spawn_pose_args={
            "x": LaunchConfiguration("ferrari_op_spawn_x"),
            "y": LaunchConfiguration("ferrari_op_spawn_y"),
            "z": LaunchConfiguration("ferrari_op_spawn_z"),
            "roll": LaunchConfiguration("ferrari_op_spawn_roll"),
            "pitch": LaunchConfiguration("ferrari_op_spawn_pitch"),
            "yaw": LaunchConfiguration("ferrari_op_spawn_yaw"),
        },
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
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("ferrari_spawn_x", default_value="-5.195865"),
            DeclareLaunchArgument("ferrari_spawn_y", default_value="0.883153"),
            DeclareLaunchArgument("ferrari_spawn_z", default_value="0.05"),
            DeclareLaunchArgument("ferrari_spawn_roll", default_value="0.0"),
            DeclareLaunchArgument("ferrari_spawn_pitch", default_value="0.000003"),
            DeclareLaunchArgument("ferrari_spawn_yaw", default_value="-1.600342"),
            DeclareLaunchArgument("ferrari_op_spawn_x", default_value="-1.038894"),
            DeclareLaunchArgument("ferrari_op_spawn_y", default_value="1.353274"),
            DeclareLaunchArgument("ferrari_op_spawn_z", default_value="0.05"),
            DeclareLaunchArgument("ferrari_op_spawn_roll", default_value="0.0"),
            DeclareLaunchArgument("ferrari_op_spawn_pitch", default_value="0.000003"),
            DeclareLaunchArgument("ferrari_op_spawn_yaw", default_value="1.541203"),
            gazebo,
            ferrari_group,
            ferrari_op_group,
            rviz2,
        ]
    )
