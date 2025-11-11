#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import os, glob, re, time, threading, select, asyncio
from evdev import InputDevice, list_devices, ecodes
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

KEY_W, KEY_A, KEY_S, KEY_D, KEY_SPACE = ecodes.KEY_W, ecodes.KEY_A, ecodes.KEY_S, ecodes.KEY_D, ecodes.KEY_SPACE

def clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)

class KeyTeleop(Node):
    def __init__(self):
        super().__init__('key_teleop')

        # === 파라미터 ===
        self.declare_parameter('device', '')                         # 명시 경로 (예: /dev/input/event6)
        self.declare_parameter('device_glob', '/dev/input/by-id/*-event-kbd')
        self.declare_parameter('device_name_regex', '')              # 예: "Keychron|Logitech"
        self.declare_parameter('grab', True)                         # True면 터미널로 키 전달 차단
        self.declare_parameter('rate_hz', 50.0)                      # 퍼블리시 주기
        self.declare_parameter('max_erpm', 1000.0)                   # W/S 때 출력 ERPM
        self.declare_parameter('steer_left', 0.0)                    # A → 0.0
        self.declare_parameter('steer_center', 0.5)                  # 중립
        self.declare_parameter('steer_right', 1.0)                   # D → 1.0
        self.declare_parameter('debug', False)

        g = lambda k: self.get_parameter(k).value
        self.dev_path_cfg = str(g('device')).strip()
        self.dev_glob     = str(g('device_glob')).strip()
        self.name_regex   = str(g('device_name_regex')).strip()
        self.name_re      = re.compile(self.name_regex) if self.name_regex else None
        self.grab         = bool(g('grab'))
        self.rate_hz      = float(g('rate_hz'))
        self.max_erpm     = float(g('max_erpm'))
        self.steer_left   = float(g('steer_left'))
        self.steer_center = float(g('steer_center'))
        self.steer_right  = float(g('steer_right'))
        self.debug        = bool(g('debug'))

        # === 퍼블리셔 ===
        self.pub_speed = self.create_publisher(Float64, 'commands/motor/speed', 10)
        self.pub_steer = self.create_publisher(Float64, 'commands/servo/position', 10)

        # === 상태 ===
        self.down = set()  # 눌린 키 집합
        self.current_erpm  = 0.0
        self.current_steer = self.steer_center

        # === 디바이스 열기 ===
        self.dev = self._open_keyboard_device()
        if self.grab:
            try:
                self.dev.grab()
            except Exception as e:
                self.get_logger().warning(f'grab failed: {e}')

        # === 이벤트 리더 스레드 ===
        self._stop_evt = threading.Event()
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()

        # === 퍼블리시 타이머 ===
        period = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(period, self._publish_cmd)

        self.get_logger().info("Controls: W/S=Fwd/Rev, A/D=Left/Right, SPACE=Stop(reset to center)")
        self.get_logger().info(f"Using input device: {self.dev.path} ({self.dev.name})")

    # ------------------ 장치 탐색 ------------------
    def _open_keyboard_device(self) -> InputDevice:
        # 1) 명시 경로
        if self.dev_path_cfg and os.path.exists(self.dev_path_cfg):
            return InputDevice(self.dev_path_cfg)

        # 2) by-id 우선
        if self.dev_glob:
            for p in sorted(glob.glob(self.dev_glob)):
                try:
                    d = InputDevice(p)
                    if self._is_keyboard_like(d) and self._name_ok(d):
                        return d
                except Exception:
                    pass

        # 3) 전체 스캔
        for p in list_devices():
            try:
                d = InputDevice(p)
                if self._is_keyboard_like(d) and self._name_ok(d):
                    return d
            except Exception:
                continue

        # 실패 시 디버깅 도움
        found = []
        for p in list_devices():
            try:
                d = InputDevice(p); found.append(f"{p}  {d.name}")
            except Exception:
                pass
        raise RuntimeError("keyboard device not found; set -p device:=/dev/input/eventX\n"
                           "found devices:\n  " + "\n  ".join(found))

    def _name_ok(self, dev: InputDevice) -> bool:
        if not self.name_re:
            return True
        try:
            return bool(self.name_re.search(dev.name or ''))
        except Exception:
            return False

    def _is_keyboard_like(self, dev: InputDevice) -> bool:
        try:
            caps = dev.capabilities(verbose=False)
            if ecodes.EV_KEY not in caps:
                return False
            keys = set(caps.get(ecodes.EV_KEY, []))
            needed = {KEY_W, KEY_A, KEY_S, KEY_D, KEY_SPACE}
            return needed.issubset(keys)
        except Exception:
            return False

    # ------------------ 이벤트 루프 ------------------
    def _read_loop(self):
        # evdev의 close가 asyncio loop를 요구 → 스레드용 루프 생성
        try:
            asyncio.get_event_loop()
        except RuntimeError:
            asyncio.set_event_loop(asyncio.new_event_loop())

        fd = self.dev.fd
        while not self._stop_evt.is_set():
            try:
                r, _, _ = select.select([fd], [], [], 0.1)
                if not r:
                    continue
                for e in self.dev.read():
                    if e.type != ecodes.EV_KEY:
                        continue
                    if e.value not in (0, 1):  # auto-repeat(2) 무시
                        continue
                    code, val = e.code, e.value
                    if val == 1:
                        self.down.add(code)
                    else:
                        self.down.discard(code)
                    if self.debug:
                        self.get_logger().info(f'key {code} {"DOWN" if val==1 else "UP"}')
                    self._recompute_command()
            except OSError:
                time.sleep(0.01)
            except Exception as e:
                self.get_logger().warning(f"read error: {e}")
                time.sleep(0.1)

    # ------------------ 명령 계산 ------------------
    def _recompute_command(self):
        w = KEY_W in self.down
        s = KEY_S in self.down
        a = KEY_A in self.down
        d = KEY_D in self.down
        sp= KEY_SPACE in self.down

        if sp:
            self.current_erpm  = 0.0
            self.current_steer = self.steer_center
            self._publish_cmd()  # 즉시 반영
            return

        if   w and not s: self.current_erpm = +self.max_erpm
        elif s and not w: self.current_erpm = -self.max_erpm
        else:             self.current_erpm = 0.0

        if   a and not d: self.current_steer = self.steer_left
        elif d and not a: self.current_steer = self.steer_right
        else:             self.current_steer = self.steer_center

    # ------------------ 퍼블리시 ------------------
    def _publish_cmd(self):
        self.pub_speed.publish(Float64(data=float(self.current_erpm)))
        self.pub_steer.publish(Float64(data=clamp01(float(self.current_steer))))

    # ------------------ 종료 ------------------
    def destroy_node(self):
        self._stop_evt.set()
        try:
            if self.dev:
                if self.grab:
                    try: self.dev.ungrab()
                    except Exception: pass
                self.dev.close()
        except Exception:
            pass
        return super().destroy_node()

def main():
    rclpy.init()
    node = None
    try:
        node = KeyTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
