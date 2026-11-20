"""센서 없이 사용자 실측 양끝을 5회 왕복. --run 지정 시 실제 구동."""

import argparse
import math
import threading
import time

import config
from dynamixel_control import Dynamixel, MotorError
from integration_test import metadata, session_log, save_summary
from motor_session import JogController, JogSettings
from telemetry import Metrics


def move_to(controller, target, stop, log, timeout_s=60, clock=time.perf_counter):
    if not controller.armed:
        raise MotorError("Torque ON 준비가 되지 않았습니다")
    if not controller.settings.safe_min <= target <= controller.settings.safe_max:
        raise MotorError("실측 범위를 벗어난 목표를 거부했습니다")
    if stop.is_set():
        raise MotorError("종료 요청")
    controller.goal = target
    controller.device.write(116, 4, target, "sweep_goal")
    started = last_motion = last_print = clock()
    motion_position = controller.position
    hits = 0
    while not stop.is_set():
        controller.poll()  # 위치/하드웨어 오류/Torque/실측 범위 확인
        now = clock()
        position = controller.position
        log.emit("sweep_position", goal=target, position=position, elapsed_ms=(now-started)*1000)
        if now - last_print >= 0.5:
            print(f"목표 {target} / 실제 {position}tick ({position*360/4096:.2f}°)", flush=True)
            last_print = now
        if abs(position - target) <= 2:
            hits += 1
            if hits >= 3:
                return
        else:
            hits = 0
        if abs(position - motion_position) >= 2:
            motion_position, last_motion = position, now
        if hits == 0 and now - last_motion >= 3:
            raise MotorError(f"3초간 유의한 위치 변화 없음: 목표{target}, 실제{position}. "
                             "축/링크 움직임, 걸림과 하중을 확인하세요")
        if now - started >= timeout_s:
            raise MotorError(f"목표 도달 시간 초과: 목표{target}, 실제{position}")
        stop.wait(0.03)
    raise MotorError("종료 요청")


def run_sweep(controller, cycles, stop, log, result, timeout_s=60):
    low, high = config.MOTOR_MEASURED_MIN_TICK, config.MOTOR_MEASURED_MAX_TICK
    controller.connect()
    # 기존 모드3/5 유지, 현재 위치를 먼저 Goal에 기록하고 저속 프로파일 적용.
    controller.arm(JogSettings(low, high), stop.is_set)
    print(f"초기 위치 정렬: 위쪽 끝 {low}tick으로 이동", flush=True)
    move_to(controller, low, stop, log, timeout_s)
    for cycle in range(1, cycles + 1):
        print(f"[{cycle}/{cycles}] 아래쪽 끝으로 이동", flush=True)
        move_to(controller, high, stop, log, timeout_s)
        if stop.wait(0.3):
            raise MotorError("종료 요청")
        print(f"[{cycle}/{cycles}] 위쪽 끝으로 복귀", flush=True)
        move_to(controller, low, stop, log, timeout_s)
        result["completed_cycles"] = cycle
        log.emit("sweep_cycle_completed", cycle=cycle)
        print(f"왕복 {cycle}회 완료", flush=True)
        if stop.wait(0.3):
            raise MotorError("종료 요청")


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true", help="실제 모터 왕복 실행")
    parser.add_argument("--cycles", type=int, default=5)
    parser.add_argument("--port", default=config.U2D2_PORT)
    parser.add_argument("--id", dest="motor_id", type=int, default=config.MOTOR_ID)
    parser.add_argument("--baud", type=int, default=config.U2D2_BAUD)
    parser.add_argument("--leg-timeout", type=float, default=60, help="편도 도달 제한 시간(초)")
    parser.add_argument("--log-dir", default="logs")
    args = parser.parse_args(argv)
    if not 1 <= args.cycles <= 20:
        parser.error("왕복 횟수는 1~20이어야 합니다")
    if not math.isfinite(args.leg_timeout) or not 5 <= args.leg_timeout <= 120:
        parser.error("편도 제한 시간은 5~120초의 유한한 값이어야 합니다")
    if not args.port or not 0 <= args.motor_id <= 252 or args.baud <= 0:
        parser.error("포트/ID/통신속도 확인 필요")
    return args


def main(argv=None):
    args = arguments(argv)
    low, high = config.MOTOR_MEASURED_MIN_TICK, config.MOTOR_MEASURED_MAX_TICK
    print(f"모터 전용: {args.port} / ID{args.motor_id} / {args.baud}bps")
    print(f"위쪽 {low*360/4096:.3f}° ↔ 아래쪽 {high*360/4096:.3f}°, {args.cycles}왕복")
    print("초기 위쪽 이동 후 아래쪽→위쪽을 1회로 계산. 저속 구동으로 5왕복은 대략 6~7분 예상.")
    if not args.run:
        print("실제 실행은 --run을 지정하세요. 아직 모터를 연결하거나 움직이지 않았습니다.")
        return 0
    print("실제 구동 시작. Ctrl+C로 중단합니다. 종료 시 Torque OFF되므로 빔을 지지하세요.", flush=True)
    metrics, stop = Metrics(), threading.Event()
    log = session_log(args.log_dir)
    log.emit("session", purpose="motor_sweep", settings=vars(args), **metadata())
    device = Dynamixel(args.port, args.baud, args.motor_id, metrics, log)
    controller = JogController(device, log)
    result = {"completed_cycles": 0, "requested_cycles": args.cycles, "success": False}
    try:
        run_sweep(controller, args.cycles, stop, log, result, args.leg_timeout)
        result["success"] = True
    except KeyboardInterrupt:
        result["error"] = "사용자 Ctrl+C 중단"
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        stop.set()
        try:
            controller.close()  # OFF 확인 후 원래 프로파일 복원, 통신 포트 해제
        except (MotorError, OSError) as exc:
            result["success"] = False
            result["cleanup_error"] = str(exc)
        result["torque_off_confirmed"] = controller.torque_off_confirmed
        log.emit("sweep_result", **result)
        print("최종 결과:", result)
        if controller.torque_off_confirmed is False:
            print("Torque OFF 확인 불가: 외부 전원 정지 수단을 사용하세요.", flush=True)
        save_summary(log, metrics, motor_sweep=result)
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
