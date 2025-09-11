#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock

class ScanAndClockPrinter(Node):
    def __init__(self):
        super().__init__('scan_and_clock_printer')

        # /scan 구독
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        # /clock 구독
        self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10)

        self.latest_clock = None

    def scan_callback(self, msg: LaserScan):
        sec = msg.header.stamp.sec
        nsec = msg.header.stamp.nanosec
        if self.latest_clock is not None:
            c_sec = self.latest_clock.sec
            c_nsec = self.latest_clock.nanosec
            self.get_logger().info(
                f'/scan.header.stamp = {sec}.{nsec:09d}  |  /clock = {c_sec}.{c_nsec:09d}'
            )
        else:
            self.get_logger().info(
                f'/scan.header.stamp = {sec}.{nsec:09d}  |  /clock = (아직 수신 안 됨)'
            )

    def clock_callback(self, msg: Clock):
        self.latest_clock = msg.clock

def main(args=None):
    rclpy.init(args=args)
    node = ScanAndClockPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
