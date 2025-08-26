#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import os, sys, time, math, select, termios, tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
def slew(cur, tgt, rate, dt):
    if rate <= 0.0: return tgt
    d = tgt - cur; m = rate * dt
    return tgt if abs(d) <= m else cur + math.copysign(m, d)
def lp1(y, x, tau, dt):
    if tau <= 0.0: return x
    a = dt / (tau + dt); return y + a*(x - y)

def ackermann_split(delta, L, W):
    if abs(delta) < 1e-6 or L <= 0.0 or W <= 0.0: return delta, delta
    s = 1.0 if delta > 0 else -1.0
    d = abs(delta); R = L / math.tan(d)
    din  = math.atan(L / max(1e-9, (R - W/2)))
    dout = math.atan(L / (R + W/2))
    return ( din,  dout) if s>0 else (-dout, -din)

HELP = "홀드 방식: w/s 누르는 동안만 전/후진, a/d 누르는 동안만 조향(떼면 정렬), Space 즉시 정지"

class KeyboardAckermann(Node):
    def __init__(self):
        super().__init__('keyboard_ackermann')

        # 주행/조향 파라미터
        self.declare_parameter('rate_hz',            80.0)
        self.declare_parameter('cruise_speed',       12.0)   # w/s 홀드시 목표 각속도(rad/s)
        self.declare_parameter('max_speed',          30.0)
        self.declare_parameter('max_steer_rad',       0.6)
        self.declare_parameter('step_steer',          0.05)  # 홀드 중 매 틱당 누적량

        # 레이트 제한/필터(빠른 반응)
        self.declare_parameter('accel_limit',        50.0)   # rad/s^2
        self.declare_parameter('decel_limit',        80.0)
        self.declare_parameter('steer_rate_limit',   10.0)   # rad/s
        self.declare_parameter('speed_tau',           0.02)
        self.declare_parameter('steer_tau',           0.02)

        # 속도에 따른 조향 축소
        self.declare_parameter('steer_min_frac',      0.5)
        self.declare_parameter('steer_fade_speed',   15.0)   # rad/s

        # 아커만
        self.declare_parameter('wheelbase',           0.30)
        self.declare_parameter('track',               0.35)
        self.declare_parameter('use_ackermann_split', True)

        # 키 홀드 판정 창(중요!)
        self.declare_parameter('speed_hold_window_sec', 0.40)  # w/s: 넉넉하게
        self.declare_parameter('steer_hold_window_sec', 0.20)  # a/d: 짧게(떼면 바로 정렬)

        g = lambda k: self.get_parameter(k).value
        self.rate_hz   = float(g('rate_hz'))
        self.cruise    = float(g('cruise_speed'))
        self.max_speed = float(g('max_speed'))
        self.max_delta = float(g('max_steer_rad'))
        self.step_d    = float(g('step_steer'))
        self.acc_lim   = float(g('accel_limit'))
        self.dec_lim   = float(g('decel_limit'))
        self.drate_lim = float(g('steer_rate_limit'))
        self.sp_tau    = float(g('speed_tau'))
        self.st_tau    = float(g('steer_tau'))
        self.min_frac  = float(g('steer_min_frac'))
        self.fade_spd  = float(g('steer_fade_speed'))
        self.L         = float(g('wheelbase'))
        self.W         = float(g('track'))
        self.use_ack   = bool(g('use_ackermann_split'))
        self.win_w     = float(g('speed_hold_window_sec'))
        self.win_a     = float(g('steer_hold_window_sec'))

        # 상태
        self.v_cmd = 0.0
        self.d_cmd = 0.0
        self.v_tgt = 0.0
        self.d_tgt = 0.0

        # 마지막 입력 시각(키별)
        now = time.time()
        self.last = {'w': 0.0, 's': 0.0, 'a': 0.0, 'd': 0.0}

        # 퍼블리셔
        self.pub_steer = self.create_publisher(Float64MultiArray, '/front_steer_controller/commands', 10)
        self.pub_drive = self.create_publisher(Float64MultiArray, '/rear_drive_controller/commands', 10)

        # /dev/tty
        self.fd = os.open('/dev/tty', os.O_RDONLY)
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

        self.get_logger().info(HELP)

    def close_tty(self):
        try:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
            os.close(self.fd)
        except Exception:
            pass

    def process_keys(self):
        """버퍼에 쌓인 모든 키를 한 프레임에 처리."""
        any_key = False
        while True:
            r, _, _ = select.select([self.fd], [], [], 0.0)
            if not r: break
            ch = os.read(self.fd, 1).decode('utf-8', 'ignore')
            any_key = True
            t = time.time()
            if ch in ('w','W'): self.last['w'] = t
            elif ch in ('s','S'): self.last['s'] = t
            elif ch in ('a','A'): self.last['a'] = t
            elif ch in ('d','D'): self.last['d'] = t
            elif ch == ' ':
                # 즉시 정지/정렬
                self.v_tgt = 0.0
                self.d_tgt = 0.0
                self.last = {'w':0.0,'s':0.0,'a':0.0,'d':0.0}
        return any_key

    def held(self, key, window):
        return (time.time() - self.last[key]) <= window

    def update_targets_from_holds(self):
        """홀드 상태를 읽어 목표값 결정."""
        w_on = self.held('w', self.win_w)
        s_on = self.held('s', self.win_w)
        a_on = self.held('a', self.win_a)
        d_on = self.held('d', self.win_a)

        # 속도: w/s 둘 다 아니면 정지. 둘 다면 0(상쇄).
        if w_on and not s_on:
            self.v_tgt = +self.cruise
        elif s_on and not w_on:
            self.v_tgt = -self.cruise
        else:
            self.v_tgt = 0.0

        # 조향: a/d 둘 다 아니면 0으로 복귀. 홀드 중에는 누적.
        if a_on and not d_on:
            self.d_tgt = clamp(self.d_tgt + self.step_d, -self.max_delta, self.max_delta)
        elif d_on and not a_on:
            self.d_tgt = clamp(self.d_tgt - self.step_d, -self.max_delta, self.max_delta)
        else:
            self.d_tgt = 0.0

    def step_control(self, dt):
        # 속도 슬루/LPF
        rate = self.acc_lim if abs(self.v_tgt) > abs(self.v_cmd) else self.dec_lim
        self.v_cmd = lp1(self.v_cmd, slew(self.v_cmd, self.v_tgt, rate, dt), self.sp_tau, dt)

        # 속도에 따른 조향 한계 축소
        vabs = abs(self.v_cmd)
        min_ang = self.max_delta * clamp(self.min_frac, 0.1, 1.0)
        if self.fade_spd > 1e-6:
            k = clamp(vabs / self.fade_spd, 0.0, 1.0)
            max_eff = min_ang + (self.max_delta - min_ang) * (1.0 - k)
        else:
            max_eff = self.max_delta
        self.d_tgt = clamp(self.d_tgt, -max_eff, +max_eff)

        # 조향 슬루/LPF
        self.d_cmd = lp1(self.d_cmd, slew(self.d_cmd, self.d_tgt, self.drate_lim, dt), self.st_tau, dt)

    def publish(self):
        if self.get_parameter('use_ackermann_split').value:
            dl, dr = ackermann_split(self.d_cmd, float(self.get_parameter('wheelbase').value),
                                     float(self.get_parameter('track').value))
        else:
            dl = dr = self.d_cmd
        self.pub_steer.publish(Float64MultiArray(data=[dl, dr]))
        self.pub_drive.publish(Float64MultiArray(data=[self.v_cmd, self.v_cmd]))

def main():
    rclpy.init()
    node = KeyboardAckermann()
    period = 1.0 / max(10.0, node.get_parameter('rate_hz').value)
    last = time.time(); last_print = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)

            node.process_keys()
            node.update_targets_from_holds()

            now = time.time(); dt = now - last
            if dt < 0.0 or dt > 0.2: dt = period
            last = now

            node.step_control(dt)
            node.publish()

            if now - last_print > 0.25:
                dir_str = '전진' if node.v_tgt>0 else '후진' if node.v_tgt<0 else '정지'
                sys.stdout.write(f'\r[{dir_str}] ω_tgt {node.v_tgt:+5.2f}  ω_cmd {node.v_cmd:+5.2f}  δ {node.d_cmd:+5.3f}   ')
                sys.stdout.flush(); last_print = now

            time.sleep(max(0.0, period - (time.time() - now)))
    except KeyboardInterrupt:
        pass
    finally:
        node.close_tty()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
