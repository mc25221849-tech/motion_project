"""PID에 필요한 출력 범위 설정, 센서/모터 실행 및 정리 공통 코드."""

from dataclasses import dataclass
import math
import threading
import time

import dynamixel_sdk as sdk

import config
from arduino_serial import ArduinoReader, distance_is_fresh, format_diagnostic
from dynamixel_control import Dynamixel, MotorError
from integration_test import normalized_port, session_log, save_summary, metadata
from motor_session import JogController, JogSettings
from telemetry import Metrics, SharedState


@dataclass(frozen=True)
class PSettings:
    kp: float = 0.05  # 모터 각도 °/거리 mm. 미튜닝 초기값, 빔각 이득이 아님
    max_offset_deg: float = 2.0
    slew_deg_s: float = 2.0

    def validate(self):
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.kp, self.max_offset_deg, self.slew_deg_s)):
            raise ValueError("Kp/출력 제한/명령 변화율은 양의 유한한 값이어야 합니다")
        if self.max_offset_deg > 5 or self.slew_deg_s > 5:
            raise ValueError("초기 시험 출력/변화율 제한은 최대 5° 및 5°/s입니다")
        return self

    @property
    def bounds(self):
        low = max(config.MOTOR_MEASURED_MIN_TICK,
                  math.ceil((config.BEAM_LEVEL_MOTOR_DEG - self.max_offset_deg) * 4096 / 360))
        high = min(config.MOTOR_MEASURED_MAX_TICK,
                   math.floor((config.BEAM_LEVEL_MOTOR_DEG + self.max_offset_deg) * 4096 / 360))
        if low >= high:
            raise ValueError("출력 범위가 모터 해상도보다 작습니다")
        return low, high


def calculate(settings, distance_mm, previous_deg, dt):
    if not all(math.isfinite(v) for v in (distance_mm, previous_deg, dt)) or dt < 0:
        raise ValueError("거리/이전 명령/시간이 유효하지 않습니다")
    error = config.TARGET_CART_DISTANCE_MM - distance_mm
    # 사용자 지정 규칙: 멀면 빔 위쪽(각도 감소), 가까우면 아래쪽(각도 증가).
    # 매 프레임 현재 각도에 더하는 방식이 아니라 수평 기준의 비례 목표를 사용.
    raw = config.BEAM_LEVEL_MOTOR_DEG + settings.kp * error
    beam_action = "UP" if error < 0 else ("DOWN" if error > 0 else "LEVEL")
    low, high = settings.bounds
    limited = min(high * 360 / 4096, max(low * 360 / 4096, raw))
    step = settings.slew_deg_s * dt
    requested = min(previous_deg + step, max(previous_deg - step, limited))
    goal = min(high, max(low, round(requested * 4096 / 360)))
    return {"distance_mm": distance_mm, "error_mm": error, "raw_motor_deg": raw,
            "beam_action": beam_action,
            "goal_tick": goal, "goal_deg": goal * 360 / 4096,
            "output_limited": raw != limited, "slew_limited": requested != limited}


class PWorker(threading.Thread):
    def __init__(self, sensor, device, settings, drive, stop, metrics, log, duration_s=30, algorithm=None):
        super().__init__(name="cart-p-control", daemon=True)
        self.sensor, self.device, self.settings = sensor, device, settings
        self.drive, self.stop, self.metrics, self.log = drive, stop, metrics, log
        self.state = SharedState()
        self.controller = JogController(device, log)
        self.failed = False
        self.duration_s = duration_s
        self.algorithm = algorithm
        self.kind = "PID" if algorithm is not None else "P"
        self.name = f"cart-{self.kind.lower()}-control"

    def poll_motor(self):
        """읽기 응답 누락만 한 번 재시도. 목표 쓰기/구동 시작은 재실행하지 않음."""
        try:
            self.controller.poll()
        except MotorError as exc:
            if exc.comm_result not in (sdk.COMM_RX_TIMEOUT, sdk.COMM_RX_CORRUPT):
                raise
            if (self.stop.is_set() or not distance_is_fresh(
                    self.sensor.state.snapshot(), time.perf_counter(), stale_s=0.18)):
                raise
            self.metrics.count(f"{self.kind.lower()}.motor_read_retries")
            self.log.emit(f"{self.kind.lower()}_motor_read_retry", reason=str(exc), attempts_remaining=1)
            if self.stop.wait(0.005):
                raise MotorError("읽기 재시도 전 종료 요청") from exc
            self.controller.poll()  # 두 번째 실패는 기존 정리 경로로 전달

    def run(self):
        run_deadline = time.perf_counter() + self.duration_s
        try:
            self.controller.connect()
            self.state.update(connected=True, error="센서 프레임 대기")
            deadline = time.perf_counter() + config.SENSOR_STARTUP_S
            while not self.stop.is_set() and time.perf_counter() < run_deadline:
                if distance_is_fresh(self.sensor.state.snapshot(), time.perf_counter()):
                    break
                sensor_state = self.sensor.state.snapshot()
                waiting_sample = sensor_state.get("sample")
                if waiting_sample is not None and not waiting_sample.valid:
                    waiting_reason = ("Arduino 프레임 수신 중 / 거리 측정 실패. "
                                      "카트 반사판 방향·위치와 20~400mm 수집 범위를 확인하세요. "
                                      + format_diagnostic(sensor_state))
                elif not sensor_state.get("port_open") and sensor_state.get("error"):
                    waiting_reason = f"Arduino 연결 확인: {sensor_state['error']}"
                else:
                    waiting_reason = "Arduino 센서 프레임 대기. COM4/60ms 펌웨어 확인"
                self.state.update(error=waiting_reason)
                if time.perf_counter() >= deadline:
                    raise MotorError(f"{self.kind} 구동 시작 안 함: {waiting_reason}")
                self.stop.wait(0.01)
            if self.stop.is_set():
                return
            if (time.perf_counter() >= run_deadline or
                    not distance_is_fresh(self.sensor.state.snapshot(), time.perf_counter(), stale_s=0.18)):
                raise MotorError("센서 준비 전 실행 시간 종료 또는 최신 데이터 없음")
            low, high = self.settings.bounds
            position = self.controller.position
            if self.drive:
                # 시작 위치가 수평 근처 제한 안에 있을 때만 ON. 자동 원점 복귀 없음.
                self.controller.arm(JogSettings(low, high), lambda:
                    self.stop.is_set() or time.perf_counter() >= run_deadline or
                    not distance_is_fresh(self.sensor.state.snapshot(), time.perf_counter(), stale_s=0.18))
                position = self.controller.position
                previous = position * 360 / 4096
            else:
                # 미리보기는 실제 모터 위치가 시험 범위 밖이어도 쓰지 않고 계산만 표시.
                previous = min(high, max(low, position)) * 360 / 4096
            last_time = time.perf_counter()
            if self.algorithm is not None:
                self.algorithm.reset(previous)
            last_sample = None
            last_goal = position
            follow_bad_since = None
            while not self.stop.is_set() and time.perf_counter() < run_deadline:
                now = time.perf_counter()
                snapshot = self.sensor.state.snapshot()
                if not distance_is_fresh(snapshot, now, stale_s=0.18):
                    raise MotorError("센서 무효/180ms 이상 오래된 데이터. 제어 중단. "
                                     + format_diagnostic(snapshot))
                sample = snapshot["sample"]
                if sample is last_sample:
                    self.stop.wait(0.005)
                    continue
                cycle_started = time.perf_counter()
                self.poll_motor()
                position = self.controller.position
                if self.drive:
                    if abs(position - last_goal) * 360 / 4096 > 1.0:
                        follow_bad_since = follow_bad_since or now
                        if now - follow_bad_since > 2:
                            raise MotorError("모터 추종 오차가 1°를 넘어 2초 지속. 하중/마찰 확인")
                    else:
                        follow_bad_since = None
                output = (calculate(self.settings, sample.distance_mm, previous, now - last_time)
                          if self.algorithm is None else self.algorithm.update(sample, now - last_time))
                if self.stop.is_set():
                    break
                checked_at = time.perf_counter()
                if (checked_at - sample.received_at > 0.18 or
                        not distance_is_fresh(self.sensor.state.snapshot(), checked_at, stale_s=0.18)):
                    raise MotorError("명령 전 센서 최신성 확인 실패. 제어 중단")
                if self.drive and output["goal_tick"] != last_goal:
                    self.device.write(116, 4, output["goal_tick"], f"{self.kind.lower()}_goal")
                    last_goal = output["goal_tick"]
                    self.controller.goal = last_goal
                # 연속 각도 상태를 유지해 작은 dt의 tick 반올림 정체를 방지.
                if self.algorithm is None:
                    wanted = min(high * 360 / 4096, max(low * 360 / 4096, output["raw_motor_deg"]))
                    delta = self.settings.slew_deg_s * (now - last_time)
                    previous = min(previous + delta, max(previous - delta, wanted))
                else:
                    previous = output["command_state_deg"]
                last_time, last_sample = now, sample
                self.state.update(error="", position=position, **output)
                self.log.emit(f"{self.kind.lower()}_sample", sequence=sample.sequence, received_at=sample.received_at,
                              motor_position=position, drive=self.drive, **output)
                self.metrics.observe(f"{self.kind.lower()}.cycle_processing_ms", (time.perf_counter() - cycle_started) * 1000)
        except (MotorError, OSError, ValueError) as exc:
            self.failed = True
            self.state.update(error=str(exc))
            self.log.emit(f"{self.kind.lower()}_error", reason=str(exc))
        except Exception as exc:
            # 작업 스레드의 예외가 failed=False/상태OK로 기록되지 않도록 한다.
            self.failed = True
            reason = f"예상하지 못한 제어 오류 ({type(exc).__name__}): {exc}"
            self.state.update(error=reason)
            self.log.emit(f"{self.kind.lower()}_error", reason=reason)
        finally:
            try:
                self.controller.close()
            except (MotorError, OSError) as exc:
                self.failed = True
                self.state.update(error=f"정리 실패: {exc}")
            self.state.update(connected=False, torque_off_confirmed=self.controller.torque_off_confirmed)
            self.stop.set()


def run_session(args, settings, algorithm=None):
    kind = "PID" if algorithm is not None else "P"
    metrics, stop = Metrics(), threading.Event()
    log = session_log(args.log_dir)
    environment = metadata()
    environment["pid_enabled"] = algorithm is not None
    log.emit("session", controller=kind, drive=args.drive, settings=vars(args), **environment)
    sensor = ArduinoReader(args.arduino_port, config.ARDUINO_BAUD, stop, metrics, log)
    device = Dynamixel(args.u2d2_port, config.U2D2_BAUD, config.MOTOR_ID, metrics, log)
    worker = PWorker(sensor, device, settings, args.drive, stop, metrics, log, args.duration, algorithm)
    print(f"{kind} {'실제 구동' if args.drive else '미리보기 / 모터 명령 없음'} | 목표108mm / 수평267.28°")
    print("P 복귀 방향: 거리>108mm → 빔 위쪽 / 거리<108mm → 빔 아래쪽")
    print(f"Kp={settings.kp} 모터°/mm (미튜닝), 수평±{settings.max_offset_deg}°, Ctrl+C 종료")
    if algorithm is not None:
        print(f"Ki={settings.ki}, Kd={settings.kd} (미튜닝); I 잔류 오차 보정 / D 제동")
    started = time.perf_counter()
    sensor.start()
    worker.start()
    try:
        while not stop.is_set() and time.perf_counter() - started < args.duration:
            data = worker.state.snapshot()
            print(f"거리={data.get('distance_mm', '--')}mm | 오차={data.get('error_mm', '--')}mm | "
                  f"목표={data.get('goal_tick', '--')}tick | 실제={data.get('position', '--')}tick | "
                  f"보정방향={data.get('beam_action', '--')} | "
                  f"상태={data['error'] or 'OK'}", flush=True)
            print(format_diagnostic(sensor.state.snapshot()), flush=True)
            stop.wait(0.5)
    except KeyboardInterrupt:
        print("종료 요청: 구동 중이면 Torque OFF를 시도합니다")
    finally:
        stop.set()
        worker.join(3)
        sensor.join(2)
        final = worker.state.snapshot()
        print("최종 상태:", final)
        sensor_final = sensor.state.snapshot()
        print(format_diagnostic(sensor_final), flush=True)
        save_summary(log, metrics, controller=kind, pid_enabled=algorithm is not None,
                     drive=args.drive, settings=vars(args),
                     final=final, sensor_diagnostic=sensor_final.get("diagnostic"),
                     failed=worker.failed, worker_finished=not worker.is_alive())
    return 1 if worker.failed or worker.is_alive() or sensor.is_alive() else 0
