"""카트 중앙108mm PID. 기본 미리보기, --drive로 실제 구동."""

import argparse
from dataclasses import dataclass
import math

import config
from dynamixel_control import MotorError
from integration_test import normalized_port
from control_runtime import PSettings, run_session


def clamp(value, low, high):
    return min(high, max(low, value))


@dataclass(frozen=True)
class PIDSettings(PSettings):
    ki: float = 0.003  # 모터°/(mm*s), 미튜닝 초기값
    kd: float = 0.02   # 모터°*s/mm, 미튜닝 초기값
    position_tau: float = 0.06
    derivative_tau: float = 0.12
    integral_limit_deg: float = 1.0

    def validate(self):
        super().validate()
        for name in ("ki", "kd", "position_tau", "derivative_tau", "integral_limit_deg"):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name}은 0 이상의 유한한 값이어야 합니다")
        if self.integral_limit_deg > self.max_offset_deg:
            raise ValueError("적분 출력 한도는 전체 출력 한도를 넘을 수 없습니다")
        return self


class PIDAlgorithm:
    def __init__(self, settings):
        self.settings = settings.validate()
        self.reset(config.BEAM_LEVEL_MOTOR_DEG)

    def reset(self, previous_command_deg):
        self.command = previous_command_deg
        self.filtered = None
        self.velocity = 0.0
        self.integral_deg = 0.0
        self.last_ms = None
        self.last_sequence = None

    def update(self, sample, command_dt):
        s = self.settings
        if (not sample.valid or sample.distance_mm is None or not math.isfinite(sample.distance_mm)
                or not config.MIN_DISTANCE_MM <= sample.distance_mm <= config.MAX_DISTANCE_MM
                or not math.isfinite(command_dt) or command_dt < 0):
            raise MotorError("PID 입력 프레임/시간이 유효하지 않습니다")
        distance = sample.distance_mm
        first = self.last_ms is None
        if first:
            dt = 0.0
            filtered = distance
            velocity = 0.0  # 첫 프레임 D kick 방지
        else:
            seq_delta = (sample.sequence - self.last_sequence) & 0xFFFFFFFF
            dt_ms = (sample.arduino_ms - self.last_ms) & 0xFFFFFFFF
            if not 0 < seq_delta < 0x80000000 or not 0 < dt_ms <= 180:
                raise MotorError("PID 측정 시간/순번 중복·리셋·과도한 간격. 재시작 필요")
            dt = dt_ms / 1000
            alpha = dt / (s.position_tau + dt)
            filtered = self.filtered + alpha * (distance - self.filtered)
            raw_velocity = (filtered - self.filtered) / dt
            beta = dt / (s.derivative_tau + dt)
            velocity = self.velocity + beta * (raw_velocity - self.velocity)
        error = config.TARGET_CART_DISTANCE_MM - filtered
        p_deg = s.kp * error
        d_deg = -s.kd * velocity  # 측정값 미분: 목표 변화에 의한 D kick 방지
        low, high = s.bounds
        lo_deg, hi_deg = low*360/4096, high*360/4096
        # 출력 제한과 명령 변화율 제한을 모두 anti-windup 판단에 포함.
        delta = s.slew_deg_s * command_dt
        reachable_low = max(lo_deg, self.command - delta)
        reachable_high = min(hi_deg, self.command + delta)
        if reachable_low > reachable_high:
            raise MotorError("PID 이전 명령이 허용 범위를 벗어났습니다")
        candidate_i = clamp(self.integral_deg + s.ki * error * dt,
                            -s.integral_limit_deg, s.integral_limit_deg) if s.ki else 0.0
        candidate_raw = config.BEAM_LEVEL_MOTOR_DEG + p_deg + candidate_i + d_deg
        integration_step = candidate_i - self.integral_deg
        blocked = ((candidate_raw > reachable_high and integration_step > 0) or
                   (candidate_raw < reachable_low and integration_step < 0))
        if not blocked:
            self.integral_deg = candidate_i
        raw = config.BEAM_LEVEL_MOTOR_DEG + p_deg + self.integral_deg + d_deg
        limited = clamp(raw, lo_deg, hi_deg)
        self.command = clamp(limited, reachable_low, reachable_high)
        goal = clamp(round(self.command * 4096 / 360), low, high)
        self.filtered, self.velocity = filtered, velocity
        self.last_ms, self.last_sequence = sample.arduino_ms, sample.sequence
        offset = raw - config.BEAM_LEVEL_MOTOR_DEG
        return {"distance_mm": distance, "filtered_distance_mm": filtered,
                "error_mm": error, "p_deg": p_deg, "i_deg": self.integral_deg,
                "d_deg": d_deg, "velocity_mm_s": velocity, "sample_dt_s": dt,
                "raw_motor_deg": raw, "goal_tick": goal, "goal_deg": goal*360/4096,
                "command_state_deg": self.command,
                "beam_action": "UP" if offset < 0 else ("DOWN" if offset > 0 else "LEVEL"),
                "output_limited": raw != limited, "slew_limited": self.command != limited,
                "integration_blocked": blocked}


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drive", action="store_true")
    parser.add_argument("--kp", type=float, default=0.05)
    parser.add_argument("--ki", type=float, default=0.003)
    parser.add_argument("--kd", type=float, default=0.02)
    parser.add_argument("--max-offset-deg", type=float, default=2)
    parser.add_argument("--slew-deg-s", type=float, default=2)
    parser.add_argument("--position-tau", type=float, default=0.06)
    parser.add_argument("--derivative-tau", type=float, default=0.12)
    parser.add_argument("--integral-limit-deg", type=float, default=1)
    parser.add_argument("--duration", type=float, default=30)
    parser.add_argument("--arduino-port", default=config.ARDUINO_PORT)
    parser.add_argument("--u2d2-port", default=config.U2D2_PORT)
    parser.add_argument("--log-dir", default="logs")
    args = parser.parse_args(argv)
    try:
        settings = PIDSettings(kp=args.kp, ki=args.ki, kd=args.kd,
                               max_offset_deg=args.max_offset_deg, slew_deg_s=args.slew_deg_s,
                               position_tau=args.position_tau, derivative_tau=args.derivative_tau,
                               integral_limit_deg=args.integral_limit_deg).validate()
        settings.bounds
        if not math.isfinite(args.duration) or args.duration <= 0:
            raise ValueError("실행 시간은 양의 유한한 값이어야 합니다")
        if not args.arduino_port or not args.u2d2_port or normalized_port(args.arduino_port) == normalized_port(args.u2d2_port):
            raise ValueError("서로 다른 Arduino/U2D2 포트가 필요합니다")
    except ValueError as exc:
        parser.error(str(exc))
    return args, settings


def main(argv=None):
    args, settings = arguments(argv)
    return run_session(args, settings, PIDAlgorithm(settings))


if __name__ == "__main__":
    raise SystemExit(main())
