#!/usr/bin/env python3
#-*- coding: utf-8 -*-
import os
import select
import subprocess
import sys
import termios
import threading
import tty

# Ensure local packages resolve when launched via the catkin relay script.
PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

import rospy 
from math import *
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64
from obstacle.follow_the_gap import Follow_the_gap
from path_tracking.pure_pursuit import Pure_pursuit
from path_tracking.PID_control import PIDController
import numpy as np

class Controller:
    def __init__(self):
        rospy.init_node("main_controller")
        self.motor_pub = rospy.Publisher('/ferrari_op/commands/motor/speed', Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('/ferrari_op/commands/servo/position', Float64, queue_size=1)
        self.follow_gap = Follow_the_gap()
        self.pure_pursuit = Pure_pursuit()
        self.gap_threshold = 1.2 # lidar detection range (m)

        self.scan_msg = None
        self.pose_msg = None

        # PID controllers (통합 제어)
        default_steer_gains = {
            "pure_pursuit": {"Kp": 0.3, "Ki": 0.0, "Kd": 0.5},
            "gap_follow": {"Kp": 0.8, "Ki": 0.0, "Kd": 0.5},
        }
        steer_pid_params = rospy.get_param("~steer_pid_gains", default_steer_gains)
        self.steer_pid_by_mode = {}
        available_modes = set(default_steer_gains.keys()) | set(steer_pid_params.keys())
        for mode_name in available_modes:
            base = default_steer_gains.get(mode_name, default_steer_gains["pure_pursuit"])
            custom = steer_pid_params.get(mode_name, {})
            self.steer_pid_by_mode[mode_name] = PIDController(
                Kp=custom.get("Kp", base["Kp"]),
                Ki=custom.get("Ki", base["Ki"]),
                Kd=custom.get("Kd", base["Kd"]),
            )

        self.speed_pid = PIDController(Kp=5.0, Ki=5.0, Kd=20.0)
        self.current_mode = "pure_pursuit"
        self.active_steer_pid = self._get_steer_pid(self.current_mode)

        # 속도 변환 파라미터 (명령→속도 스케일 B 기반)
        self.rpm_per_data = 0.025
        self.wheel_radius = 0.05
        # self.speed_weight = 0.5
        self.speed_gain = (
            self.rpm_per_data * (2.0 * np.pi / 60.0) * self.wheel_radius 
        )
        if self.speed_gain <= 1e-6:
            rospy.logwarn("speed_gain is too small; defaulting speed command limits")
            self.speed_cmd_limits = (0.0, 0.0)
        else:
            min_cmd = 0.0
            if self.pure_pursuit.min_speed > 0.0:
                min_cmd = self.pure_pursuit.min_speed / self.speed_gain
            max_speed_ref = max(self.pure_pursuit.max_speed, self.pure_pursuit.min_speed)
            max_cmd = max_speed_ref / self.speed_gain
            self.speed_cmd_limits = (float(min_cmd), float(max_cmd))
        self.speed_alpha = 0.3
        self.speed_est = 0.0
        self.last_speed_cmd = self.speed_cmd_limits[0]

        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber("/base_link_pose", Pose2D, self.pose_callback, queue_size=1)

        self.control_timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)
        self._start_keyboard_listener()

    def scan_callback(self, scan_msg):
        self.scan_msg = scan_msg

    def pose_callback(self, pose_msg):
        self.pose_msg = pose_msg

    def pub_motor(self, speed, steer):
        speed_msg = Float64()
        steer_msg = Float64()
        speed_msg.data = speed
        steer_msg.data = steer
        self.motor_pub.publish(speed_msg)
        self.servo_pub.publish(steer_msg)

    def compute_min_distance(self, scan_msg):
        if scan_msg is None:
            return float('inf'), None, None
        angle_ranges, dist_ranges = self.follow_gap.preprocess_lidar(scan_msg)
        valid_ranges = dist_ranges[np.isfinite(dist_ranges)]
        if valid_ranges.size:
            return float(valid_ranges.min()), angle_ranges, dist_ranges
        else:
            return float('inf'), angle_ranges, dist_ranges

    def command_to_mps(self, command):
        return command * self.speed_gain

    def mps_to_command(self, speed_mps):
        if self.speed_gain <= 1e-6:
            return self.speed_cmd_limits[0]
        return speed_mps / self.speed_gain

    def to_servo_angle(self, steer_rad):
        return -(steer_rad - 0.5)

    def _get_steer_pid(self, mode):
        fallback = self.steer_pid_by_mode.get("pure_pursuit")
        if fallback is None and self.steer_pid_by_mode:
            fallback = next(iter(self.steer_pid_by_mode.values()))
        return self.steer_pid_by_mode.get(mode, fallback)

    def control_loop(self, _event):
        if self.scan_msg is None or self.pose_msg is None:
            return

        min_distance, angle_ranges, dist_ranges = self.compute_min_distance(self.scan_msg)
        steer_err = 0.0
        target_speed = self.pure_pursuit.target_speed
        mode = "pure_pursuit"

        if min_distance <= self.gap_threshold and angle_ranges is not None:
            theta_err, safe_speed = self.follow_gap.compute_errors(
                angle_ranges, dist_ranges, self.gap_threshold
            )
            steer_err = theta_err if theta_err is not None else 0.0
            target_speed = min(target_speed, safe_speed)
            mode = "gap_follow"
        else:
            steer_err, cruise_speed = self.pure_pursuit.compute_errors(
                self.pose_msg, current_speed=self.speed_est
            )
            if steer_err is None:
                return
            target_speed = cruise_speed

        if mode != self.current_mode:
            self.speed_pid.reset()
            self.current_mode = mode
            self.active_steer_pid = self._get_steer_pid(mode)
            self.active_steer_pid.reset()
            rospy.loginfo(f"Controller mode: {mode}")

        steer_cmd = float(np.clip(self.active_steer_pid.compute(steer_err), -0.5, 0.5))
        #steer_cmd = float(np.clip(steer_err, -0.5, 0.5))
        # 0.85
        servo_position = self.to_servo_angle(steer_cmd)

        current_speed = self.speed_est
        speed_error = target_speed - current_speed
        delta_command = self.speed_pid.compute(speed_error)
        base_command = self.mps_to_command(target_speed)
        speed_command = base_command + delta_command
        speed_command = float(np.clip(speed_command, *self.speed_cmd_limits))

        commanded_speed_mps = self.command_to_mps(speed_command)
        self.speed_est = (
            (1.0 - self.speed_alpha) * self.speed_est + self.speed_alpha * commanded_speed_mps
        )
        self.last_speed_cmd = speed_command

        # print(
        #     f"mode={mode}, min_dist={min_distance:.2f}m, steer_err={steer_err:.3f}rad, "
        #     f"target_speed={target_speed:.2f}m/s, cmd_speed={speed_command:.0f}"
        # )

        self.pub_motor(speed_command, servo_position)

    def _start_keyboard_listener(self):
        thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        thread.start()
        self._keyboard_thread = thread

    def _keyboard_listener(self):
        if not sys.stdin.isatty():
            rospy.logwarn("stdin is not a TTY; 'q' shortcut disabled.")
            return

        fd = sys.stdin.fileno()
        old_settings = None

        shutdown_requested = False

        try:
            old_settings = termios.tcgetattr(fd)
            tty.setcbreak(fd)
            rospy.loginfo("Press 'q' to shut down all ROS nodes.")

            while not rospy.is_shutdown():
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not rlist:
                    continue

                ch = sys.stdin.read(1)
                if not ch:
                    rospy.logdebug("stdin closed; stopping keyboard listener.")
                    return

                if ch.lower() == 'q':
                    rospy.logwarn("Detected 'q'. Killing /vesc_driver_node.")
                    shutdown_requested = True
                    break

        except Exception as exc:
            rospy.logwarn(f"Keyboard listener stopped: {exc}")
        finally:
            if old_settings is not None:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        if shutdown_requested:
            self._kill_vesc_node()

    def _kill_vesc_node(self):
        try:
            subprocess.check_call(["rosnode", "kill", "/vesc_driver_node"])
        except (subprocess.CalledProcessError, OSError) as exc:
            rospy.logerr(f"Failed to kill /vesc_driver_node: {exc}")
        finally:
            rospy.signal_shutdown("Shutdown requested by keyboard input.")

def main():
    try:
        controller = Controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass    
  
if __name__ == '__main__':
    main()