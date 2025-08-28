from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('odometry')
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_se_odom',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'ekf_wheel_imu.yaml')],
        ),
    ])
