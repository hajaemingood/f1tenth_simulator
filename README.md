ros2 launch ferrari drive.launch.py use_sim_time:=true
ㄴ 시뮬레이터 런치 파일 실행

ros2 launch ferrari joy_teleop.launch.py use_sim_time:=true
ㄴ 키보드 조작 런치 파일 실행

ros2 launch odometry wheel_odom.launch.py use_sim_time:=true

ros2 launch odometry ekf_wheel_imu.launch.py
ㄴ 휠, IMU, EKF 기반 odom 생성

ros2 launch slam_toolbox online_sync_launch.py use_sim_time:=true
