#!/usr/bin/env python3
import rclpy # ROS2 Python 클라이언트 라이브러리, ROS2 프로그램을 작성할 때 기본적으로 사용
from rclpy.node import Node # ROS2에서 모든 노드는 이 클래스를 상속받아서 생성된다
from geometry_msgs.msg import Twist # 선속도 (Linear) 와 각속도 (angular)를 담는 메시지 타입
from std_msgs.msg import Float64 # 단일 부동 소수점 값을 퍼블리시 할 때 사용하는 메시지 타입

class TwistToVESC(Node):
    def __init__(self): # 생성자 _init_에서 부모 클래스인 Node의 생성자를 호출하면서 노드 이름을 지정
        super().__init__('twist_to_vesc_converter')

        # /key_vel 토픽을 읽어들여서 Twist 타입의 메시지가 들어올때마다 listener_callback()호출
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)

        # VESC에 보내기 위한 2개의 퍼블리셔 생성, Float64 메시지 타입
        self.motor_pub = self.create_publisher(Float64, '/commands/motor/duty_cycle', 10)
        self.servo_pub = self.create_publisher(Float64, '/commands/servo/position', 10)

    def listener_callback(self, msg):
        motor_msg = Float64()
        servo_msg = Float64()

        # 제한된 범위의 duty cycle (e.g. max 0.1)
        motor_duty = max(min(msg.linear.x, 1.0), -1.0) * 0.1

        # mapping parameter
        steer_deg_max = 20.0
        center = 0.5
        use_normalized_input = True

        if use_normalized_input:
            z = max(min(msg.angular.z, 1.0), -1.0) # [-1 1] 
            steer_deg = z * steer_deg_max # [-20 +20]
        else:
            steer_deg = max(min(msg.angular.z,steer_deg_max), -steer_deg_max)

        # 조향은 angular.z 값을 VESC servo position 범위 [0.0, 1.0]으로 매핑
        servo_pos = center - 0.5 * (steer_deg/steer_deg_max)
        servo_pos = max(min(servo_pos, 1.0), 0.0)

        # 클리핑 (안정성용), servo position을 0.0 ~ 1.0 범위로 제한
        servo_pos = max(min(servo_pos, 1.0), 0.0)

        # 계산한 값을 각각의 메시지에 할당
        motor_msg.data = motor_duty
        servo_msg.data = servo_pos

        # 퍼블리시하여 VESC 드라이버에 모터와 조향 명령 전달
        self.motor_pub.publish(motor_msg)
        self.servo_pub.publish(servo_msg)

        # 노드 실행 중 터미널에 duty와 servo position 값을 실시간으로 출력
        self.get_logger().info(f'Motor: {motor_duty:.2f}, Servo: {servo_pos:.2f}')

def main(args=None):
    rclpy.init(args=args) # ROS 노드 시스템 초기화
    node = TwistToVESC() # 노드 인스턴스 생성
    rclpy.spin(node) # 이벤트 루프 시작
    node.destroy_node() # Ctrl + C 등으로 종료되면 destroy_node()후 종료
    rclpy.shutdown()