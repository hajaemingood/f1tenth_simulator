#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wheel_odom_node.py

/commands/motor/speed(ERPM)와 /commands/servo/position(0~1)을 직접 구독하여
Ackermann(자전거) 모델로 오도메트리를 적분해 /wheel_odom(nav_msgs/Odometry) 퍼블리시.

핵심 파라미터(ros2 params 또는 YAML로 주입):
  - wheelbase [m], wheel_radius [m]
  - odom_frame, base_frame, odom_topic
  - body_forward_sign (전진 +X 정합)
  - use_imu_heading, imu_topic
  - erpm_per_radps (ERPM → rad/s 변환 스케일)
  - servo_center (서보 중앙값), max_steer_rad (최대 조향각, 라디안)

메모:
  - 시작 직후에도 안전하게 돌도록 delta(조향각)를 0.0으로 초기화
  - IMU yaw가 오면 그 값을 우선 채택(드리프트 억제). 없으면 운동학 적분
  - 공분산 기본값 포함(필요시 조정)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


def q_to_yaw(z: float, w: float) -> float:
    """2D 가정(roll/pitch ≈ 0)에서 yaw = 2*atan2(z, w)"""
    return 2.0 * math.atan2(z, w)


class WheelOdomNode(Node):
    def __init__(self) -> None:
        super().__init__('wheel_odom')

        # 차량 기하/프레임/토픽
        self.declare_parameter('wheelbase', 1.55)                # [m]
        self.declare_parameter('wheel_radius', 0.05)             # [m]
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_topic', '/wheel_odom')
        self.declare_parameter('publish_tf', False)              # 현재 노드는 TF 발행 안 함(옵션 자리)
        self.declare_parameter('body_forward_sign', 1.0)         # 전진이 +X가 되도록 필요시 -1.0

        # IMU
        self.declare_parameter('use_imu_heading', True)
        self.declare_parameter('imu_topic', '/ferrari/imu/data')

        # 모터/서보 스케일
        self.declare_parameter('erpm_per_radps', 300.0)          # 예: 300 ERPM = 1 rad/s
        self.declare_parameter('servo_center', 0.5)              # 서보 중앙(직진) 위치
        self.declare_parameter('max_steer_rad', 0.5)             # 최대 조향각(라디안)

        # 파라미터 로드
        self.L = float(self.get_parameter('wheelbase').value)
        self.R = float(self.get_parameter('wheel_radius').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.body_forward_sign = float(self.get_parameter('body_forward_sign').value)

        self.use_imu_heading = bool(self.get_parameter('use_imu_heading').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)

        self.erpm_per_radps = float(self.get_parameter('erpm_per_radps').value)
        self.servo_center = float(self.get_parameter('servo_center').value)
        self.max_steer_rad = float(self.get_parameter('max_steer_rad').value)

        # 상태 변수 초기화
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0                  # 내부 yaw (IMU 또는 적분)
        self.yaw_imu = None             # 최신 IMU yaw
        self.wz_imu = 0.0               # IMU z각속도

        self.motor_speed = 0.0          # 바퀴 각속도 [rad/s], ERPM 수신 후 변환하여 저장
        self.servo_pos = self.servo_center
        self.delta = 0.0                # 조향각 [rad], 서보 콜백 전에도 안전

        self.last_time_ros: Time = None  # 타이머에서 dt 계산용

        # 퍼블리셔/서브스크라이버
        self.pub_odom = self.create_publisher(Odometry, self.odom_topic, 20)

        # 실차 입력
        self.sub_motor = self.create_subscription(
            Float64, '/ferrari/commands/motor/speed', self.cb_motor, 20
        )
        self.sub_servo = self.create_subscription(
            Float64, '/ferrari/commands/servo/position', self.cb_servo, 20
        )

        if self.use_imu_heading:
            self.sub_imu = self.create_subscription(Imu, self.imu_topic, self.cb_imu, 50)

        # 50Hz 타이머(주기적 적분/퍼블리시)
        self.timer = self.create_timer(1.0 / 50.0, self.on_timer)

        self.get_logger().info('wheel_odom node started (motor/servo inputs)')

    # ---------------------- 콜백 ---------------------- #

    def cb_motor(self, msg: Float64) -> None:
        """ERPM → rad/s 변환 후 내부 각속도 저장"""
        erpm = float(msg.data)
        scale = self.erpm_per_radps if abs(self.erpm_per_radps) > 1e-9 else 1.0
        self.motor_speed = erpm / scale  # [rad/s]

    def cb_servo(self, msg: Float64) -> None:
        """서보 0~1 → 조향각 라디안 변환"""
        self.servo_pos = float(msg.data)
        # 중앙 기준 대칭 스케일: (pos - center) * 2 * max
        self.delta = (self.servo_pos - self.servo_center) * (2.0 * self.max_steer_rad)

    def cb_imu(self, msg: Imu) -> None:
        """IMU yaw/wz 수신"""
        self.yaw_imu = q_to_yaw(float(msg.orientation.z), float(msg.orientation.w))
        self.wz_imu = float(msg.angular_velocity.z)

    # ---------------------- 타이머(핵심 적분 루프) ---------------------- #

    def on_timer(self) -> None:
        now = self.get_clock().now()
        if self.last_time_ros is None:
            self.last_time_ros = now
            return

        dt = (now - self.last_time_ros).nanoseconds * 1e-9
        # 센서 타임스탬프 이상/런치 초기 불안정 구간 방어
        if dt <= 0.0 or dt > 1.0:
            dt = 0.0
        self.last_time_ros = now

        # 운동학 계산 & 퍼블리시
        self.update_odom(dt, now)

    # ---------------------- 운동학/퍼블리시 ---------------------- #

    def update_odom(self, dt: float, now_ros_time: Time) -> None:
        # 선속도 v [m/s] = (바퀴 각속도[rad/s]) * R
        v_body = self.body_forward_sign * self.motor_speed * self.R

        # 요레이트 omega [rad/s] = v/L * tan(delta)
        delta = self.delta  # 방어 초기화 되어있음
        omega_kin = (v_body * math.tan(delta) / self.L) if abs(self.L) > 1e-6 else 0.0

        # yaw 업데이트: IMU가 있으면 우선 적용, 없으면 적분
        if dt > 0.0:
            if self.use_imu_heading and (self.yaw_imu is not None):
                self.yaw = self.yaw_imu
            else:
                self.yaw += omega_kin * dt

            # yaw 정규화(±pi)
            self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

            # 바디 전진을 월드(odom)로 투영
            self.x += v_body * math.cos(self.yaw) * dt
            self.y += v_body * math.sin(self.yaw) * dt

        # Odometry 메시지 구성
        odom = Odometry()
        odom.header.stamp = now_ros_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(self.yaw * 0.5)

        # Twist
        odom.twist.twist.linear.x = v_body
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.wz_imu if (self.use_imu_heading and self.yaw_imu is not None) else omega_kin

        # 공분산(예시값: 상황에 맞게 조정 가능)
        odom.pose.covariance = [
            0.05, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.05, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e6, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e6, 0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e6, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.1,
        ]
        odom.twist.covariance = [
            0.05, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.05, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e6, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e6, 0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e6, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.1,
        ]

        # 퍼블리시
        self.pub_odom.publish(odom)


def main() -> None:
    rclpy.init()
    node = WheelOdomNode()
    try:
        rclpy.spin(node)  # 타이머/구독 콜백 모두 여기서 처리
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
