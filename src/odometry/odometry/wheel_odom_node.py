#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry

def q_to_yaw(z, w):
    # 2D 가정(roll/pitch≈0): yaw = 2*atan2(z,w)
    return 2.0 * math.atan2(z, w)

class WheelOdomNode(Node):
    def __init__(self):
        super().__init__('wheel_odom')

        # 기본 파라미터
        self.declare_parameter('wheelbase', 1.55)                     # [m]
        self.declare_parameter('wheel_radius', 0.03)                  # [m]
        self.declare_parameter('steer_joint', 'front_steer_joint')
        self.declare_parameter('drive_joints', ['rear_left_wheel_joint','rear_right_wheel_joint'])
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_topic', '/wheel_odom')
        self.declare_parameter('publish_tf', False)                   # EKF가 TF 퍼블리시
        self.declare_parameter('steer_gain', 1.0)                     # rad = gain*pos + offset
        self.declare_parameter('steer_offset', 0.0)
        self.declare_parameter('drive_joint_signs', [1.0, 1.0])       # drive_joints와 길이 동일
        self.declare_parameter('steer_sign', 1.0)                     # 좌회전(+yaw)시 +가 되도록
        self.declare_parameter('body_forward_sign', 1.0)              # 전진이 +X가 되도록(필요시 -1)
        # IMU 헤딩 사용 옵션
        self.declare_parameter('use_imu_heading', True)
        self.declare_parameter('imu_topic', '/imu/data')

        # 파라미터 로드
        self.L = float(self.get_parameter('wheelbase').value)
        self.R = float(self.get_parameter('wheel_radius').value)
        self.steer_joint = self.get_parameter('steer_joint').value
        self.drive_joints = list(self.get_parameter('drive_joints').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.k_steer = float(self.get_parameter('steer_gain').value)
        self.b_steer = float(self.get_parameter('steer_offset').value)
        self.drive_joint_signs = list(self.get_parameter('drive_joint_signs').value)
        self.steer_sign = float(self.get_parameter('steer_sign').value)
        self.body_forward_sign = float(self.get_parameter('body_forward_sign').value)
        self.use_imu_heading = bool(self.get_parameter('use_imu_heading').value)
        self.imu_topic = self.get_parameter('imu_topic').value

        # 상태
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0                # 내부 yaw (IMU 또는 적분)
        self.yaw_imu = None           # IMU yaw 최신값
        self.wz_imu = 0.0
        self.joint_pos = {}
        self.joint_vel = {}
        self.last_stamp = None

        # pub/sub
        self.pub_odom = self.create_publisher(Odometry, self.odom_topic, 20)
        self.sub_js = self.create_subscription(JointState, '/joint_states', self.cb_js, 50)
        if self.use_imu_heading:
            self.sub_imu = self.create_subscription(Imu, self.imu_topic, self.cb_imu, 50)

        self.get_logger().info('wheel_odom node started')

    def cb_imu(self, msg: Imu):
        # IMU yaw/wz 저장
        self.yaw_imu = q_to_yaw(msg.orientation.z, msg.orientation.w)
        self.wz_imu = float(msg.angular_velocity.z)

    def cb_js(self, msg: JointState):
        # 시간
        stamp = msg.header.stamp
        if self.last_stamp is None:
            self.last_stamp = stamp

        dt = (stamp.sec - self.last_stamp.sec) + 1e-9*(stamp.nanosec - self.last_stamp.nanosec)
        if dt <= 0.0 or dt > 1.0:
            dt = 0.0
        self.last_stamp = stamp

        # 조인트 맵/갱신
        name_to_idx = {n:i for i,n in enumerate(msg.name)}
        for n, i in name_to_idx.items():
            if i < len(msg.position):
                self.joint_pos[n] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_vel[n] = msg.velocity[i]

        # 조향각(부호 포함)
        delta = 0.0
        if self.steer_joint in self.joint_pos:
            raw_delta = self.k_steer * float(self.joint_pos[self.steer_joint]) + self.b_steer
            delta = self.steer_sign * raw_delta

        # 구동 각속도 평균(rad/s) — 부호 적용
        w_vals = []
        for idx, j in enumerate(self.drive_joints):
            if j in self.joint_vel:
                sign = self.drive_joint_signs[idx] if idx < len(self.drive_joint_signs) else 1.0
                w_vals.append(sign * float(self.joint_vel[j]))
        if not w_vals:
            return
        w_mean = sum(w_vals) / len(w_vals)

        # 선속도, 요레이트(기구학)
        v_body = self.body_forward_sign * w_mean * self.R
        omega_kin = v_body * math.tan(delta) / self.L if abs(self.L) > 1e-6 else 0.0

        # yaw 업데이트: IMU 우선 사용
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

        # 메시지
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.yaw*0.5)
        odom.pose.pose.orientation.w = math.cos(self.yaw*0.5)
        odom.twist.twist.linear.x = v_body
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.wz_imu if (self.use_imu_heading and self.yaw_imu is not None) else omega_kin

        # 공분산(float 36개)
        odom.pose.covariance = [
            0.05,0.0,0.0,0.0,0.0,0.0,
            0.0,0.05,0.0,0.0,0.0,0.0,
            0.0,0.0,1e6,0.0,0.0,0.0,
            0.0,0.0,0.0,1e6,0.0,0.0,
            0.0,0.0,0.0,0.0,1e6,0.0,
            0.0,0.0,0.0,0.0,0.0,0.1
        ]
        odom.twist.covariance = [
            0.05,0.0,0.0,0.0,0.0,0.0,
            0.0,0.05,0.0,0.0,0.0,0.0,
            0.0,0.0,1e6,0.0,0.0,0.0,
            0.0,0.0,0.0,1e6,0.0,0.0,
            0.0,0.0,0.0,0.0,1e6,0.0,
            0.0,0.0,0.0,0.0,0.0,0.1
        ]

        self.pub_odom.publish(odom)

def main():
    rclpy.init()
    node = WheelOdomNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
