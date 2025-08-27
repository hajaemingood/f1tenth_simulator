from launch import LaunchDescription
from launch.actions import OpaqueFunction, LogInfo
from launch_ros.actions import Node
import os, glob, re

def _pick_keyboard_device():
    """
    키보드 자동 선택(안전판):
      1) by-id *-event-kbd 중에서 W/A/S/D/SPACE 지원 장치
      2) evdev capabilities로 W/A/S/D/SPACE 지원하는 모든 event 중 우선순위 선택
      3) by-path *-kbd
      4) 최종 폴백: /dev/input/event2
    """
    # evdev가 있으면 capabilities 기반으로 엄격 선택
    try:
        from evdev import list_devices, InputDevice, ecodes

        needed = {ecodes.KEY_W, ecodes.KEY_A, ecodes.KEY_S, ecodes.KEY_D, ecodes.KEY_SPACE}

        def supports_wasd(path):
            try:
                d = InputDevice(path)
                keys = set(d.capabilities().get(ecodes.EV_KEY, []))
                return needed.issubset(keys), d.name
            except Exception:
                return False, ""

        # (1) by-id 우선
        for p in sorted(glob.glob('/dev/input/by-id/*-event-kbd')):
            ok, name = supports_wasd(p)
            if ok:
                return p, name

        # (2) 모든 event 중 capabilities 만족 장치 수집
        candidates = []
        for p in list_devices():
            ok, name = supports_wasd(p)
            if ok:
                candidates.append((p, name))

        # 우선순위: 내장 AT 키보드 > 일반 "keyboard" > 리시버류,
        #          그리고 전형적 노이즈( power/wmi/video/lid/headphone/hdmi/touchpad/mouse )는 역가중
        if candidates:
            def score(item):
                path, name = item
                nl = (name or "").lower()
                s = 0
                if 'translated set 2' in nl: s += 5
                if 'keyboard' in nl:         s += 3
                if 'receiver' in nl or 'wireless' in nl or '2.4g' in nl: s += 1
                if any(bad in nl for bad in ('power', 'wmi', 'video', 'lid', 'headphone', 'hdmi', 'touchpad', 'mouse')):
                    s -= 10
                return s
            candidates.sort(key=score, reverse=True)
            p, name = candidates[0]
            return p, name
    except Exception:
        pass

    # (3) by-path 백업
    bypath = sorted(glob.glob('/dev/input/by-path/*-kbd'))
    if bypath:
        return bypath[0], '(by-path)'

    # (4) 최후 폴백
    return '/dev/input/event2', '(fallback)'

def _make_nodes(context):
    dev_path, dev_name = _pick_keyboard_device()

    return [
        LogInfo(msg=f'[key_teleop] Using device: {dev_path} ({dev_name})'),
        Node(
            package='ferrari',
            executable='key_ackermann',   # setup.py console_scripts 에 등록한 이름
            name='key_ack',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'device_path': dev_path,  # 자동 선택 결과
                'grab': False,            # 기본 끔: 다른 터미널 입력 막지 않음
                'rate_hz': 80.0,
                'cruise_speed': 12.0,
                'max_steer_rad': 0.6,
                'max_speed': 20.0,
                'use_ackermann_split': True,
                'wheelbase': 0.30,
                'track': 0.35,
            }]
        )
    ]

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_make_nodes)])