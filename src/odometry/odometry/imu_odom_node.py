# ferrari_imu_odom/imu_odom_node.py
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time

def q_to_yaw(z, w):
    # roll, pitch 무시. 2D 가정: yaw만 사용
    # z,w만으로 yaw 복원(assuming x=y=0) → yaw = 2*atan2(z,w)
    return 2.0 * math.atan2(z, w)

def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)

class ImuOdomNode(Node):
    def __init__(self):
        super().__init__('imu_odom')

        # 파라미터
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_topic', '/imu_odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('mode', 'yaw_only')  # yaw_only or integrate_accel
        self.declare_parameter('two_d_mode', True)

        self.declare_parameter('accel_use', True)
        self.declare_parameter('remove_gravity', True)
        self.declare_parameter('accel_bias_init_samples', 200)
        self.declare_parameter('accel_noise_sigma', 0.6)
        self.declare_parameter('gyro_bias_correction', True)
        self.declare_parameter('accel_lpf_alpha', 0.3)

        self.imu_topic = self.get_parameter('imu_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.mode = self.get_parameter('mode').value
        self.two_d_mode = bool(self.get_parameter('two_d_mode').value)

        self.accel_use = bool(self.get_parameter('accel_use').value)
        self.remove_gravity = bool(self.get_parameter('remove_gravity').value)
        self.accel_bias_init_samples = int(self.get_parameter('accel_bias_init_samples').value)
        self.accel_noise_sigma = float(self.get_parameter('accel_noise_sigma').value)
        self.gyro_bias_correction = bool(self.get_parameter('gyro_bias_correction').value)
        self.accel_lpf_alpha = float(self.get_parameter('accel_lpf_alpha').value)

        # 상태 변수
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.yaw = 0.0
        self.last_time = None

        # 바이어스 및 필터
        self.acc_bias_x = 0.0
        self.acc_bias_y = 0.0
        self.bias_count = 0
        self.lpf_ax = 0.0
        self.lpf_ay = 0.0
        self.gyro_bias_z = 0.0
        self.gyro_bias_initialized = False

        # 퍼블리셔/브로드캐스터
        self.pub_odom = self.create_publisher(Odometry, self.odom_topic, 20)
        self.br = TransformBroadcaster(self)

        # 구독
        self.sub_imu = self.create_subscription(Imu, self.imu_topic, self.cb_imu, 100)

        self.get_logger().info(f'IMU odom started: mode={self.mode}, imu_topic={self.imu_topic}')

    def cb_imu(self, msg: Imu):
        now = msg.header.stamp
        if self.last_time is None:
            self.last_time = now
            # 초기 yaw 설정
            self.yaw = q_to_yaw(msg.orientation.z, msg.orientation.w)
            # 가속도 초기화
            self.lpf_ax = msg.linear_acceleration.x
            self.lpf_ay = msg.linear_acceleration.y
            return

        dt = self.duration_sec(self.last_time, now)
        if dt <= 0.0 or dt > 1.0:
            self.last_time = now
            return
        self.last_time = now

        # 1) Yaw 갱신(자이로 적분 + 절대 orientation 보정 혼합 가능)
        yaw_meas = q_to_yaw(msg.orientation.z, msg.orientation.w)
        wz = msg.angular_velocity.z

        if self.gyro_bias_correction and not self.gyro_bias_initialized:
            # 정지 상태 초기에 약간의 샘플로 gyro 바이어스 추정(매우 단순)
            # 필요시 별도 조건(속도 매우 작음) 넣어서 안정화 가능
            self.gyro_bias_z = 0.0
            self.gyro_bias_initialized = True

        wz -= self.gyro_bias_z

        # 보수적: 자이로 적분과 절대 yaw를 혼합(알파=0.02 같은)
        alpha_yaw = 0.02
        yaw_from_gyro = self.yaw + wz * dt
        self.yaw = (1.0 - alpha_yaw) * yaw_from_gyro + alpha_yaw * yaw_meas

        # 2) 모드별 위치/속도 갱신
        if self.mode == 'yaw_only':
            # 위치/속도는 업데이트하지 않음(0 유지). 회전만 추적.
            pass

        elif self.mode == 'integrate_accel' and self.accel_use:
            # 가속도 전처리: 중력 제거 옵션, 저역통과, 바이어스 추정
            ax = msg.linear_acceleration.x
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z

            if self.remove_gravity:
                # 2D 가정: 중력은 z에만. 시뮬/실차 드라이버에 따라 이미 제거되어 있을 수도 있음.
                az -= 9.80665

            # 간단 LPF
            self.lpf_ax = self.accel_lpf_alpha * ax + (1 - self.accel_lpf_alpha) * self.lpf_ax
            self.lpf_ay = self.accel_lpf_alpha * ay + (1 - self.accel_lpf_alpha) * self.lpf_ay

            # 초기 바이어스 추정(정지 가정) → 앞 N샘플 평균
            if self.bias_count < self.accel_bias_init_samples:
                self.acc_bias_x = (self.acc_bias_x * self.bias_count + self.lpf_ax) / (self.bias_count + 1)
                self.acc_bias_y = (self.acc_bias_y * self.bias_count + self.lpf_ay) / (self.bias_count + 1)
                self.bias_count += 1

            axc = self.lpf_ax - self.acc_bias_x
            ayc = self.lpf_ay - self.acc_bias_y

            # 바디좌표계(x=전진, y=좌측)를 odom좌표계(x,y)로 회전
            cos_y = math.cos(self.yaw)
            sin_y = math.sin(self.yaw)
            ax_odom = axc * cos_y - ayc * sin_y
            ay_odom = axc * sin_y + ayc * cos_y

            # 속도 적분
            self.vx += ax_odom * dt
            self.vy += ay_odom * dt

            # 매우 단순한 속도 클램프/드리프트 완화(옵션)
            vmax = 25.0
            self.vx = max(min(self.vx, vmax), -vmax)
            self.vy = max(min(self.vy, vmax), -vmax)

            # 위치 적분
            self.x += self.vx * dt
            self.y += self.vy * dt

        # 3) 오도메트리 메시지 퍼블리시
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        qx, qy, qz, qw = yaw_to_quat(self.yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = wz

        # 대략적 공분산(튜닝 필요)
        if self.mode == 'yaw_only':
            odom.pose.covariance = [
                1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 5e-3
            ]
            odom.twist.covariance = [
                1e3, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1e3, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 5e-3
            ]
        else:
            odom.pose.covariance = [
                0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.05
            ]
            odom.twist.covariance = [
                0.2, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.2, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.05
            ]

        self.pub_odom.publish(odom)

        # TF 퍼블리시
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.br.sendTransform(t)

    @staticmethod
    def duration_sec(t0: Time, t1: Time) -> float:
        return (t1.sec - t0.sec) + 1e-9 * (t1.nanosec - t0.nanosec)

def main():
    rclpy.init()
    node = ImuOdomNode()
    rclpy.spin(node)
    rclpy.shutdown()
