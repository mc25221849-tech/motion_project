"""모터 연결·Torque·프로파일 관리 및 위치/오류 감시 공통 코드."""

from dataclasses import dataclass
import math
import time
from types import SimpleNamespace
from dynamixel_control import MotorError
from motor_preflight import prepare


@dataclass(frozen=True)
class JogSettings:
    safe_min: int
    safe_max: int
    step_degrees: float = 1.0
    timeout_s: float = 5.0

    def validate(self):
        if not 0 <= self.safe_min < self.safe_max <= 4095:
            raise ValueError("실측 안전 범위는 0~4095 안의 하한 < 상한이어야 합니다")
        if not math.isfinite(self.step_degrees) or not 0.5 <= self.step_degrees <= 5:
            raise ValueError("키 한 번 이동량은 0.5~5°여야 합니다")
        if not math.isfinite(self.timeout_s) or not 0 < self.timeout_s <= 30:
            raise ValueError("목표 도달 제한 시간은 0 초과 30초 이하여야 합니다")
        return self

    @property
    def step_ticks(self):
        return round(self.step_degrees * 4096 / 360)


class JogController:
    def __init__(self, device, log, clock=time.perf_counter):
        self.device, self.log, self.clock = device, log, clock
        self.connected = False
        self.armed = False
        self.moving = False
        self.torque_owned = False
        self.settings = None
        self.profiles = []
        self.position = self.goal = self.anchor = None
        self.mode = None
        self.goal_started = None
        self.hits = 0
        self.torque_off_confirmed = None

    def connect(self):
        if self.torque_owned or self.armed:
            raise MotorError("먼저 Space로 Torque를 끈 뒤 재연결하세요")
        self.device.close()
        self.device.open()
        info = self.device.verify()
        self.position = self.device.position()
        self.mode = info["mode"]
        self.connected = True
        self.log.emit("keyboard_connected", position=self.position, **info)
        return info

    def _check_cancel(self, cancelled):
        if cancelled():
            raise MotorError("조작 준비 중 중단 요청을 받았습니다")

    def arm(self, settings, cancelled=lambda: False):
        if not self.connected or self.armed or self.torque_owned or self.profiles:
            raise MotorError("연결 및 Torque OFF 상태에서 조작을 시작하세요")
        settings.validate()
        self.settings = settings
        options = SimpleNamespace(safe_min=settings.safe_min, safe_max=settings.safe_max,
                                  delta_ticks=0, allow_current_position_mode=True)
        # EEPROM/모드/전류 설정을 쓰지 않고, 기존 모드 3/5의 조건을 읽어 검사한다.
        plan = prepare(self.device, options)
        self._check_cancel(cancelled)
        if self.device.position() != plan.anchor:
            raise MotorError("준비 중 위치가 바뀌었습니다. 다시 확인하세요")
        self.device.allow_writes = True
        try:
            for address, value in ((108, 1), (112, 2)):
                old = self.device.read(address, 4, "keyboard_original_profile")
                self.profiles.append((address, old))
                self.device.write(address, 4, value, "keyboard_profile")
                self._check_cancel(cancelled)
            self.device.write(116, 4, plan.anchor, "keyboard_goal_preload")
            self._check_cancel(cancelled)
            self.torque_owned = True  # ON 응답 유실 시에도 OFF를 시도해야 함
            self.torque_off_confirmed = False
            self.device.torque(True)
            self._check_cancel(cancelled)
            self.position = self.device.position()
            if (abs(self.position - plan.anchor) > 2 or
                    not settings.safe_min <= self.position <= settings.safe_max):
                raise MotorError("Torque ON 후 위치가 예상과 다릅니다")
            self.anchor = self.goal = plan.anchor
            self.armed, self.moving = True, False
            self.log.emit("keyboard_armed", anchor=self.anchor, safe_min=settings.safe_min,
                          safe_max=settings.safe_max, step_ticks=settings.step_ticks)
        except BaseException:
            self.disarm()
            raise

    def poll(self):
        self.position = self.device.position()
        if not self.armed:
            return
        if not self.settings.safe_min <= self.position <= self.settings.safe_max:
            raise MotorError("안전 범위 이탈. Torque OFF를 시도합니다")
        if self.device.read(70, 1, "keyboard_hardware_error"):
            raise MotorError("모터 하드웨어 오류. Torque OFF를 시도합니다")
        if self.device.read(64, 1, "keyboard_torque_state") != 1:
            raise MotorError("예상하지 못한 Torque OFF. 조작을 중단합니다")
        self.log.emit("keyboard_position", position=self.position, goal=self.goal,
                      moving=self.moving)
        if self.moving:
            self.hits = self.hits + 1 if abs(self.position - self.goal) <= 2 else 0
            elapsed = self.clock() - self.goal_started
            if self.hits >= 3:
                self.moving = False
                self.log.emit("keyboard_goal_reached", goal=self.goal,
                              position=self.position, elapsed_ms=elapsed * 1000)
            elif elapsed >= self.settings.timeout_s:
                raise MotorError(f"목표 {self.goal} / 실제 {self.position}tick: "
                                 "제한 시간 내 도달 실패. 기구 하중/마찰과 내부 제어 상태를 확인하세요")

    def disarm(self, force=False):
        self.armed, self.moving = False, False
        failures = []
        off = not self.torque_owned
        if self.torque_owned or (force and self.connected):
            self.device.allow_writes = True
            off = False
            for _ in range(2):
                try:
                    self.device.torque(False)
                    off = True
                    break
                except (MotorError, OSError) as exc:
                    failures.append(str(exc))
            self.torque_off_confirmed = off
        if off:
            self.torque_owned = False
            while self.profiles:
                address, old = self.profiles[-1]
                try:
                    self.device.write(address, 4, old, "keyboard_profile_restore")
                    self.profiles.pop()
                except (MotorError, OSError) as exc:
                    failures.append(f"프로파일 복원 실패: {exc}")
                    break
        self.log.emit("keyboard_disarmed", torque_off_confirmed=self.torque_off_confirmed,
                      cleanup_errors=failures)
        if not off:
            raise MotorError("Torque OFF 확인 불가. 외부 전원 정지 수단을 사용하세요. " + "; ".join(failures))
        if self.profiles:
            raise MotorError("Torque OFF 확인, 원래 프로파일 복원 실패: " + "; ".join(failures))

    def close(self):
        try:
            self.disarm()
        finally:
            self.connected = False
            self.device.close()
