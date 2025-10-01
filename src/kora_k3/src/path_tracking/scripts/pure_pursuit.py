#!/usr/bin/env python3
import rospy
import numpy as np
import csv
import sys
from math import *
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class Pure_pursuit:
    def __init__(self):
        rospy.init_node("pure_pursuit_node", anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.pose_callback)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=10)
        self.drive_msg = AckermannDriveStamped()

        self.csv_file = '/home/tony/f1tenth_ws/src/pure_pursuit/waypoints/waypoints.csv'
        self.waypoints = self.load_waypoints()

        # Parameters
        self.lookahead_distance = 0.7  # Lookahead distance for Pure Pursuit


    def pose_callback(self, pose_msg):
        # 1. Find the current waypoint to track
        # 2. Transform the goal point to the vehicle frame
        goal_point = self.find_goal_point(pose_msg)
        print(goal_point)
        # 3. Calculate curvature (steering angle)
        steering_angle = self.calculate_steering_angle(goal_point)
        # print(steering_angle)
        # 4. Publish the drive message
        self.publish_drive_message(steering_angle)

    def find_goal_point(self, pose_msg):
        # 현재 차량 위치
        car_x = pose_msg.pose.pose.position.x
        car_y = pose_msg.pose.pose.position.y
        yaw = self.get_yaw_from_pose(pose_msg)

        # 목표 경로점 리스트 초기화
        max_distance = -1
        goal_point = []
        
        for x, y in self.waypoints:
            # 차량과 경로점 간의 거리 계산
            dx = x - car_x
            dy = y - car_y
            distance = sqrt(dx**2 + dy**2)

            # 거리 조건을 먼저 확인 (0.7 미터 이내, 차량 앞쪽)
            if distance <= self.lookahead_distance:
                # 차량 프레임에서 x축이 양수인 경우만 앞쪽으로 간주
                rotated_x = cos(-yaw) * dx - sin(-yaw) * dy
                rotated_y = sin(-yaw) * dx + cos(-yaw) * dy

                if rotated_x > 0:
                    # 가장 먼 점을 실시간으로 찾기
                    if distance > max_distance:
                        max_distance = distance
                        goal_point = (x, y, rotated_x, rotated_y, distance)

        return goal_point

    def calculate_steering_angle(self, goal_point):
        # Calculate the curvature using Pure Pursuit formula
        L = self.lookahead_distance
        y = goal_point[3]
        curvature = 2 * y / (L ** 2)  # Curvature formula

        # Limit the steering angle within the car's steering limits (example: +/- 0.5 rad)
        if curvature < -0.5:
            curvature = -0.5
        elif curvature > 0.5:
            curvature = 0.5

        return curvature

    def publish_drive_message(self, steering_angle):
        # Create and publish the Ackermann drive message

        if abs(steering_angle) > 0.35:
            velocity = 1.0
        elif abs(steering_angle) > 0.175:
            velocity = 1.5
        else:
            velocity = 3.0

        self.drive_msg.drive.steering_angle = steering_angle
        self.drive_msg.drive.speed = velocity 
        self.drive_pub.publish(self.drive_msg)

    def get_yaw_from_pose(self, pose_msg):
        # Extract yaw from the quaternion orientation
        orientation_q = pose_msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y ** 2 + orientation_q.z ** 2)
        return np.arctan2(siny_cosp, cosy_cosp)
    
    def load_waypoints(self):
        """CSV 파일에서 웨이포인트를 읽어 리스트로 반환"""
        waypoints = []
        with open(self.csv_file, 'r') as file:
            reader = csv.reader(file)
            next(reader)  # 헤더 건너뛰기 (필요 시)
            for row in reader:
                # x, y 좌표 추출
                x = float(row[0])
                y = float(row[1])
                waypoints.append((x, y))
        return waypoints

def main():
    try:
        pure_pursuit = Pure_pursuit()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
