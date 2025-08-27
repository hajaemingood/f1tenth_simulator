#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import math, time, threading, select, glob, os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from evdev import InputDevice, list_devices, ecodes

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
def slew(cur, tgt, rate, dt):
    if rate <= 0.0: return tgt
    d = tgt - cur; m = rate * dt
    return tgt if abs(d) <= m else cur + (m if d>0 else -m)

def _pick_keyboard_device():
    """우선순위: (1) by-id *-event-kbd -> (2) W/A/S/D/SPACE 지원 장치 -> (3) 이름 휴리스틱"""
    # 1) by-id 우선 (가장 신뢰도 높음)
    try:
        for path in sorted(glob.glob('/dev/input/by-id/*-event-kbd')):
            if os.path.exists(path):
                return path
    except Exception:
        pass

    # 2) capabilities로 실제 키 지원 확인
    needed = {ecodes.KEY_W, ecodes.KEY_A, ecodes.KEY_S, ecodes.KEY_D, ecodes.KEY_SPACE}
    for p in list_devices():
        try:
            d = InputDevice(p)
            caps = d.capabilities().get(ecodes.EV_KEY, [])
            keys = set(caps)
            if needed.issubset(keys):
                return p
        except Exception:
            continue

    # 3) 이름 휴리스틱 (가짜 키보드 제외)
    banned = ('wmi', 'video', 'power', 'lid', 'headphone', 'hdmi', 'touchpad', 'mouse')
    for p in list_devices():
        try:
            name = InputDevice(p).name.lower()
            if ('keyboard' in name or 'kbd' in name or 'key' in name) and not any(b in name for b in banned):
                return p
        except Exception:
            continue
    return ''

class KbAckermann(Node):
    def __init__(self):
        super().__init__('kb_evdev_ackermann')

        # 파라미터
        self.declare_parameter('device_path', '')         # 비우면 자동탐색
        self.declare_parameter('grab', False)             # True면 키 입력을 터미널로 전달 안 함
        self.declare_parameter('rate_hz', 80.0)
        self.declare_parameter('cruise_speed', 12.0)      # w/s 홀드시 목표 각속도(rad/s)
        self.declare_parameter('max_speed', 30.0)
        self.declare_parameter('max_steer_rad', 0.6)
        self.declare_parameter('steer_rate_limit', 10.0)  # rad/s
        self.declare_parameter('accel_limit', 60.0)       # rad/s^2
        self.declare_parameter('decel_limit', 90.0)       # rad/s^2
        self.declare_parameter('step_steer', 0.05)        # a/d 홀드 중 프레임당 누적량
        self.declare_parameter('use_ackermann_split', True)
        self.declare_parameter('wheelbase', 0.30)
        self.declare_parameter('track', 0.35)

        g = lambda k:self.get_parameter(k).value
        self.rate = float(g('rate_hz'))
        self.cruise = float(g('cruise_speed'))
        self.max_speed = float(g('max_speed'))
        self.max_delta = float(g('max_steer_rad'))
        self.drate_lim = float(g('steer_rate_limit'))
        self.acc_lim = float(g('accel_limit'))
        self.dec_lim = float(g('decel_limit'))
        self.step_d = float(g('step_steer'))
        self.use_ack = bool(g('use_ackermann_split'))
        self.L = float(g('wheelbase')); self.W = float(g('track'))
        want_grab = bool(g('grab'))

        # 키보드 장치 선택(자동 + 견고)
        path = str(g('device_path')).strip()
        if not path:
            path = _pick_keyboard_device()
        if not path:
            # 진단을 위해 목록을 같이 보여줌
            found = []
            for p in list_devices():
                try:
                    d = InputDevice(p)
                    found.append(f"{p}  {d.name}")
                except Exception:
                    pass
            raise RuntimeError("keyboard device not found; set -p device_path:=/dev/input/eventX\n"
                               "found devices:\n  " + "\n  ".join(found))

        try:
            self.dev = InputDevice(path)
        except PermissionError as e:
            raise PermissionError(f"cannot open {path}: {e}\n"
                                  "도커 실행 시 --device /dev/input:/dev/input:ro 또는 --privileged 옵션 확인") from e

        self._grabbed = False

        # 필요시 grab: 키가 터미널로 전달되지 않게 함
        if want_grab:
            try:
                self.dev.grab()
            except Exception as e:
                self.get_logger().warn(f'grab failed on {self.dev.path}: {e}')

        # 상태
        self.down = set()      # 현재 눌린 키 코드 집합
        self.v_cmd = 0.0; self.v_tgt = 0.0
        self.d_cmd = 0.0; self.d_tgt = 0.0
        self.dt = 1.0/self.rate

        # ROS pub
        self.pub_steer = self.create_publisher(Float64MultiArray, '/front_steer_controller/commands', 10)
        self.pub_drive = self.create_publisher(Float64MultiArray, '/rear_drive_controller/commands', 10)

        # 이벤트 읽기 스레드
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()
        # 제어 타이머
        self.timer = self.create_timer(self.dt, self._tick)

        self.get_logger().info(f'Using keyboard device: {self.dev.path} ({self.dev.name})')
        self.get_logger().info('Controls: hold W/S=forward/reverse, hold A/D=steer, SPACE=stop')

    def _read_loop(self):
        fd = self.dev.fd
        while rclpy.ok():
            r,_,_ = select.select([fd], [], [], 0.1)
            if not r: continue
            try:
                for e in self.dev.read():
                    if e.type == ecodes.EV_KEY:
                        code, val = e.code, e.value
                        if val == 1:   self.down.add(code)      # keydown
                        elif val == 0: self.down.discard(code)  # keyup
                        # val == 2 (autorepeat)는 무시
            except OSError:
                # 장치가 잠시 끊기는 경우 방어
                time.sleep(0.01)

    def _tick(self):
        # 1) 눌림 상태 해석(동시 입력 완전 지원)
        w = ecodes.KEY_W in self.down
        s = ecodes.KEY_S in self.down
        a = ecodes.KEY_A in self.down
        d = ecodes.KEY_D in self.down
        sp= ecodes.KEY_SPACE in self.down

        if sp:
            self.v_tgt = 0.0; self.d_tgt = 0.0
        else:
            # 속도: 홀드 동안 유지, 둘 다/둘 다 아님이면 0
            if w and not s:   self.v_tgt = +self.cruise
            elif s and not w: self.v_tgt = -self.cruise
            else:             self.v_tgt = 0.0

            # 조향: 홀드 동안만 누적, 없으면 즉시 0
            if a and not d:   self.d_tgt = clamp(self.d_tgt + self.step_d, -self.max_delta, self.max_delta)
            elif d and not a: self.d_tgt = clamp(self.d_tgt - self.step_d, -self.max_delta, self.max_delta)
            else:             self.d_tgt = 0.0

        # 2) 레이트 제한(부드럽게)
        rate = self.acc_lim if abs(self.v_tgt) > abs(self.v_cmd) else self.dec_lim
        self.v_cmd = slew(self.v_cmd, self.v_tgt, rate, self.dt)
        self.d_cmd = slew(self.d_cmd, self.d_tgt, self.drate_lim, self.dt)

        # 3) 퍼블리시
        if self.use_ack and abs(self.d_cmd) > 1e-6 and self.L>0 and self.W>0:
            R = self.L / math.tan(abs(self.d_cmd))
            ins = math.atan(self.L / max(1e-6, (R - self.W/2)))
            ous = math.atan(self.L / (R + self.W/2))
            if self.d_cmd > 0: dl, dr = ins, ous
            else:              dl, dr = -ous, -ins
        else:
            dl = dr = self.d_cmd

        self.pub_steer.publish(Float64MultiArray(data=[dl, dr]))
        self.pub_drive.publish(Float64MultiArray(data=[self.v_cmd, self.v_cmd]))

def main():
    rclpy.init()
    node = KbAckermann()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # (C) 종료 시 grab 해제 (grab=False면 그냥 지나감)
        try:
            if getattr(node, '_grabbed', False):
                node.dev.ungrab()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
