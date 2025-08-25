#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import curses
import os
import signal
import time

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import Header


class TextWindow():
    def __init__(self, stdscr, lines=10):
        self._screen = stdscr
        self._screen.nodelay(True)
        self._screen.keypad(True)  # 화살표 인식 안정화
        curses.curs_set(0)
        self._num_lines = lines

    def read_key(self):
        keycode = self._screen.getch()
        return keycode if keycode != -1 else None

    def clear(self):
        self._screen.clear()

    def write_line(self, lineno, message):
        if lineno < 0 or lineno >= self._num_lines:
            raise ValueError('lineno out of bounds')
        height, width = self._screen.getmaxyx()
        y = (height / self._num_lines) * lineno
        x = 10
        for text in message.split('\n'):
            text = text.ljust(width)
            self._screen.addstr(int(y), int(x), text)
            y += 1

    def refresh(self):
        self._screen.refresh()

    def beep(self):
        curses.flash()


class SimpleKeyTeleop(Node):
    def __init__(self, interface):
        super().__init__('key_teleop')
        self._interface = interface

        self._publish_stamped_twist = self.declare_parameter('twist_stamped_enabled', False).value
        if self._publish_stamped_twist:
            self._pub_cmd = self.create_publisher(TwistStamped, 'cmd_vel', qos_profile_system_default)
        else:
            self._pub_cmd = self.create_publisher(Twist, 'cmd_vel', qos_profile_system_default)

        self._hz = self.declare_parameter('hz', 20).value  # 입력 감도 ↑

        # 동작 파라미터
        self._acceleration = 2.5     # m/s^2, 가감속률
        self._rotation_rate = 1.0    # rad/s, 좌우 회전 각속도
        self._max_speed = 5.0
        self._min_speed = -1.0

        self._linear_speed = 0.0
        self._angular_speed = 0.0

        # 키 상태 관리
        self._last_pressed = {}                 # {keycode: time}
        self._hold_window = Duration(seconds=0.5)  # 최근 이 시간 내 관찰되면 "눌림"으로 간주

        # 퍼블리시 값
        self._linear = 0.0
        self._angular = 0.0

        self._last_update_time = self.get_clock().now()

    # 키 그룹 묶음
    KEY_UPS = (curses.KEY_UP, ord('w'), ord('W'), ord('q'), ord('Q'))     # q를 가속으로
    KEY_DOWNS = (curses.KEY_DOWN, ord('s'), ord('S'), ord('z'), ord('Z')) # z를 감속으로
    KEY_LEFTS = (curses.KEY_LEFT, ord('a'), ord('A'))
    KEY_RIGHTS = (curses.KEY_RIGHT, ord('d'), ord('D'))

    def run(self):
        self._running = True
        while self._running:
            # 입력 모두 소진
            while True:
                keycode = self._interface.read_key()
                if keycode is None:
                    break
                self._key_pressed(keycode)

            self._set_velocity()
            self._publish()
            time.sleep(1.0 / self._hz)

    def _key_pressed(self, keycode):
        # 스페이스바: 정지
        if keycode == ord(' '):
            self._linear_speed = 0.0
            self._angular_speed = 0.0
            return

        # 종료 단축키 제거(q는 더 이상 종료 아님)
        # 방향/가감속 관련 키는 전부 "최근 눌림"으로 기록
        if keycode in self.KEY_UPS + self.KEY_DOWNS + self.KEY_LEFTS + self.KEY_RIGHTS:
            self._last_pressed[keycode] = self.get_clock().now()

        # 원하면 다른 종료키를 따로 둘 수 있음. 예: ESC
        if keycode == 27:  # ESC
            self._running = False
            os.kill(os.getpid(), signal.SIGINT)

    def _is_held(self, group_keys, now):
        for k in group_keys:
            t = self._last_pressed.get(k)
            if t is not None and (now - t) < self._hold_window:
                return True
        return False

    def _set_velocity(self):
        now = self.get_clock().now()
        dt = (now - self._last_update_time).nanoseconds / 1e9
        if dt <= 0:
            dt = 1.0 / max(self._hz, 1.0)
        self._last_update_time = now

        held_up = self._is_held(self.KEY_UPS, now)
        held_down = self._is_held(self.KEY_DOWNS, now)
        held_left = self._is_held(self.KEY_LEFTS, now)
        held_right = self._is_held(self.KEY_RIGHTS, now)

        # 선속도: 시간기반 누적 가감속
        if held_up and not held_down:
            self._linear_speed = min(self._linear_speed + self._acceleration * dt, self._max_speed)
        elif held_down and not held_up:
            self._linear_speed = max(self._linear_speed - self._acceleration * dt, self._min_speed)
        # 둘 다 누르면 상쇄 → 속도 유지

        # 각속도: 좌우 동시 누르면 0이 되도록 처리
        if held_left and not held_right:
            self._angular_speed = +self._rotation_rate
        elif held_right and not held_left:
            self._angular_speed = -self._rotation_rate
        else:
            self._angular_speed = 0.0

        self._linear = self._linear_speed
        self._angular = self._angular_speed

    def _make_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        return twist

    def _make_twist_stamped(self, linear, angular):
        twist_stamped = TwistStamped()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'key_teleop'
        twist_stamped.header = header
        twist_stamped.twist.linear.x = linear
        twist_stamped.twist.angular.z = angular
        return twist_stamped

    def _publish(self):
        self._interface.clear()
        self._interface.write_line(2, f'Linear: {self._linear:.2f}, Angular: {self._angular:.2f}')
        self._interface.write_line(4, 'Hold ↑/W/Q to accelerate, ↓/S/Z to decelerate')
        self._interface.write_line(5, 'Hold ←/A or →/D to turn, Space to stop, ESC to quit')
        self._interface.refresh()

        twist = (self._make_twist_stamped if self._publish_stamped_twist else self._make_twist)(
            self._linear, self._angular
        )
        self._pub_cmd.publish(twist)


def execute(stdscr):
    rclpy.init()
    app = SimpleKeyTeleop(TextWindow(stdscr))
    app.run()
    app.destroy_node()
    rclpy.shutdown()


def main():
    try:
        curses.wrapper(execute)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
