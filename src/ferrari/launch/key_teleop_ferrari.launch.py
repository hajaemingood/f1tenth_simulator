# ferrari/launch/key_teleop.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # === 런치 인자 선언 ===
        # 내장 키보드 이름으로 고정 (필요시 바꿔서 사용)
        DeclareLaunchArgument('device_name_regex', default_value='AT Translated Set 2 keyboard'),
        # 명시 경로 강제용(보통 빈 문자열 유지)
        DeclareLaunchArgument('device', default_value=''),
        # by-id가 없는 도커에서도 fallback로 쓰기 좋음
        DeclareLaunchArgument('device_glob', default_value='/dev/input/*'),
        # 입력 독점 (다른 앱으로 키 전달 방지), 필요시 false
        DeclareLaunchArgument('grab', default_value='false'),
        # 퍼블리시 주기/스케일
        DeclareLaunchArgument('publish_rate_hz', default_value='50.0'),
        DeclareLaunchArgument('max_erpm', default_value='5000.0'),
        DeclareLaunchArgument('steer_left', default_value='0.0'),
        DeclareLaunchArgument('steer_center', default_value='0.5'),
        DeclareLaunchArgument('steer_right', default_value='1.0'),

        Node(
            package='ferrari',
            executable='key_teleop',
            name='key_teleop',
            namespace='ferrari',
            output='screen',
            parameters=[{
                'device_name_regex': LaunchConfiguration('device_name_regex'),
                'device':            LaunchConfiguration('device'),
                'device_glob':       LaunchConfiguration('device_glob'),
                'grab':              LaunchConfiguration('grab'),
                'publish_rate_hz':   LaunchConfiguration('publish_rate_hz'),
                'max_erpm':          LaunchConfiguration('max_erpm'),
                'steer_left':        LaunchConfiguration('steer_left'),
                'steer_center':      LaunchConfiguration('steer_center'),
                'steer_right':       LaunchConfiguration('steer_right'),
            }],
        )
    ])
