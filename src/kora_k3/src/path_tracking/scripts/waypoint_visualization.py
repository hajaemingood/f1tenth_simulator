#!/usr/bin/env python3
import rospy
import csv
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# CSV 파일의 경로
csv_file = '/home/tony/f1tenth_ws/src/pure_pursuit/waypoints/waypoints.csv'  # 파일 경로를 맞춰 설정하세요.

class WaypointVisualizer:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node("waypoint_visualizer")

        # Marker 퍼블리셔 초기화
        self.marker_pub = rospy.Publisher("/waypoint_marker", Marker, queue_size=10)

        # Marker 메시지 설정
        self.marker = Marker()
        self.marker.header.frame_id = "map"  # RViz에서 사용할 기준 프레임 설정
        self.marker.ns = "waypoints"
        self.marker.id = 0
        self.marker.type = Marker.POINTS
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.1  # 점 크기 설정
        self.marker.scale.y = 0.1
        self.marker.color.a = 1.0  # 투명도
        self.marker.color.r = 0.0  # 빨간색
        self.marker.color.g = 0.0  # 초록색
        self.marker.color.b = 1.0  # 파란색

        # CSV 파일에서 경로점 로드 및 마커 포인트 추가
        self.load_waypoints()

    def load_waypoints(self):
        # CSV 파일 읽기
        with open(csv_file, 'r') as file:
            reader = csv.reader(file)
            next(reader)  # 헤더 건너뛰기 (필요 시)
            for row in reader:
                # x, y, z 좌표 추출
                x = float(row[0])
                y = float(row[1])
                # z = float(row[2]) if len(row) > 2 else 0.0

                # 포인트 생성 및 추가
                point = Point()
                point.x = x
                point.y = y
                point.z = 0
                self.marker.points.append(point)

    def publish_marker(self):
        # Marker 퍼블리시
        while not rospy.is_shutdown():
            self.marker.header.stamp = rospy.Time.now()
            self.marker_pub.publish(self.marker)
            rospy.sleep(0.1)  # 0.1초 간격으로 퍼블리시

if __name__ == "__main__":
    visualizer = WaypointVisualizer()
    visualizer.publish_marker()
