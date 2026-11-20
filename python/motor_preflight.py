"""모터 구동 전 읽기 전용 준비 검사."""

from dataclasses import dataclass
from dynamixel_control import MotorError


@dataclass(frozen=True)
class MotionPlan:
    anchor: int
    goal: int
    safe_min: int
    safe_max: int


def prepare(device, options):
    """확인 전에는 읽기만 한다. 운전 모드/ID/baud/EEPROM은 변경하지 않는다."""
    info = device.verify()
    allowed_modes = (3, 5) if getattr(options, "allow_current_position_mode", False) else (3,)
    if info["mode"] not in allowed_modes:
        raise MotorError("Position Control Mode(3) required; mode was NOT changed")
    if info["mode"] == 5:
        # 기존 모드/전류 설정은 유지한다. 목표 전류가 0이면 움직일 수 없다.
        goal_current = device.read(102, 2, "goal_current")
        current_limit = device.read(38, 2, "current_limit")
        if not 0 < goal_current <= current_limit <= 1193:
            raise MotorError("Current-based position test requires a positive configured Goal Current within Current Limit")
    if info["torque"] != 0:
        raise MotorError("Torque must already be OFF before this test")
    drive = device.read(10, 1, "drive_mode")
    if drive & 0x0C:
        raise MotorError("Velocity-based profile and Torque-On-by-Goal OFF required")
    if device.read(20, 4, "homing_offset") != 0:
        raise MotorError("This first test requires Homing Offset=0; offset was NOT changed")
    if device.read(68, 1, "status_return") != 2:
        raise MotorError("Status Return Level=2 required for write acknowledgement; not changed")
    if device.read(70, 1, "hardware_error") != 0:
        raise MotorError("Hardware Error Status is nonzero")
    motor_min = device.read(52, 4, "min_position")
    motor_max = device.read(48, 4, "max_position")
    if not motor_min <= options.safe_min < options.safe_max <= motor_max:
        raise MotorError("Measured safety bounds must also fit motor position limits")
    anchor = device.position()
    goal = anchor + options.delta_ticks
    # OFF→ON 시 다회전 좌표가 리셋되므로 modulo 변환해서 임의로 목표를 만들지 않음.
    if not 0 <= anchor <= 4095:
        raise MotorError("Present Position outside single-turn range; torque-enable reset risk")
    if not options.safe_min <= min(anchor, goal) <= max(anchor, goal) <= options.safe_max:
        raise MotorError(f"Current {anchor} / goal {goal} outside measured safety bounds")
    return MotionPlan(anchor, goal, options.safe_min, options.safe_max)
