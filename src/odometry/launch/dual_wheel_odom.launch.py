from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('odometry')
    wheel_odom_launch = os.path.join(pkg_share, 'launch', 'wheel_odom.launch.py')
    wheel_odom_op_launch = os.path.join(pkg_share, 'launch', 'wheel_odom_op.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(wheel_odom_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(wheel_odom_op_launch),
        ),
    ])
