"""센서/모터 개별 및 통합 통신 검사. 모터 회전 명령은 보내지 않는다."""

import argparse
from datetime import datetime
from importlib.metadata import version
import json
import math
from pathlib import Path
import sys
import threading
import time

from serial.tools import list_ports

import config
from arduino_serial import ArduinoReader, distance_is_fresh, format_diagnostic
from dynamixel_control import MotorReader
from telemetry import EventLog, Metrics, print_metrics


def session_log(root):
    # 실행 호스트의 로컬 시간은 파일명 구분에만 사용. 사건 분석은 단조 시계 사용.
    directory = Path(root) / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    return EventLog(directory)


def metadata():
    return {"python": sys.version, "pyserial": version("pyserial"),
            "dynamixel-sdk": version("dynamixel-sdk"),
            "project": "BALL BEAM 300 / BB300-R3", "pid_enabled": False}


def save_summary(log, metrics, **extra):
    log.close()
    report = {**metadata(), **metrics.summary(), "log_dropped": log.dropped,
              "log_error": log.error, **extra}
    (log.directory / "summary.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print_metrics(report)
    print(f"Log directory: {log.directory.resolve()}")
    if log.dropped or log.error:
        print(f"Log warning: dropped={log.dropped}, error={log.error}")
    return report


def normalized_port(port):
    return port.strip().upper().removeprefix("\\\\.\\")


def motor_is_fresh(snapshot, now):
    return bool(snapshot.get("connected") and
                now - snapshot.get("updated_at", -math.inf) <= config.MOTOR_STALE_S)


def display(sensor, motor, log):
    now = time.perf_counter()
    lines = ["\n--- READ ONLY / PID OFF ---"]
    if sensor is not None:
        state = sensor.state.snapshot()
        sample = state.get("sample")
        connected = bool(state.get("connected") and sample and
                         now - sample.received_at <= config.SENSOR_STALE_S)
        valid = distance_is_fresh(state, now)
        lines += [f"Arduino Connected : {'YES' if connected else 'NO'} "
                  f"(port {'OPEN' if state['port_open'] else 'CLOSED'})",
                  f"Distance          : {f'{sample.distance_mm:.1f} mm' if valid else 'Unavailable'}",
                  f"Distance Valid    : {'YES' if valid else 'NO'}"]
        if sample:
            lines.append(f"Sensor Sequence   : {sample.sequence}; "
                         f"received {(now - sample.received_at) * 1000:.1f} ms ago")
        if not valid and state.get("last_valid"):
            old = state["last_valid"]
            lines.append(f"Last Valid (past) : {old.distance_mm:.1f} mm; "
                         f"{(now - old.received_at) * 1000:.1f} ms ago")
        interval = state.get("sensor_interval_ms")
        lines.append(f"Sensor Interval   : {str(interval) + ' ms' if interval is not None else 'N/A'}")
        lines.append(f"Arduino Status    : {state['error'] or 'OK'}")
        lines.append(format_diagnostic(state))
    if motor is not None:
        state = motor.state.snapshot()
        fresh = motor_is_fresh(state, now)
        lines += [f"U2D2 Connected    : {'YES' if fresh else 'NO'} "
                  f"(port {'OPEN' if state['port_open'] else 'CLOSED'})",
                  f"XM430 ID / Baud   : {state.get('motor_id', 'unverified')} / "
                  f"{state.get('baud', 'unverified')}",
                  f"Motor Position    : {state.get('position', 'N/A')}"
                  f"{' (past / invalid)' if not fresh else ''}",
                  "Motor Goal        : No movement command",
                  f"Mode / Torque     : {state.get('mode', 'N/A')} / "
                  f"{state.get('torque', 'N/A')} (startup readings)",
                  f"Position Read     : {state.get('position_read_ms', 0):.3f} ms",
                  f"U2D2 Status       : {state['error'] or ('OK' if fresh else 'STALE')}"]
    lines.append(f"Log dropped       : {log.dropped}; Log error: {log.error or 'none'}")
    print("\n".join(lines), flush=True)


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list-ports", action="store_true", help="포트 열지 않고 목록 표시")
    parser.add_argument("--mode", choices=("sensor", "motor", "integrated"), default="integrated")
    parser.add_argument("--arduino-port", default=config.ARDUINO_PORT)
    parser.add_argument("--u2d2-port", default=config.U2D2_PORT)
    parser.add_argument("--arduino-baud", type=int, default=config.ARDUINO_BAUD)
    parser.add_argument("--u2d2-baud", type=int, default=config.U2D2_BAUD)
    parser.add_argument("--motor-id", type=int, default=config.MOTOR_ID)
    parser.add_argument("--duration", type=float, default=0, help="초 단위, 0이면 Ctrl+C까지")
    parser.add_argument("--log-dir", default="logs")
    args = parser.parse_args(argv)
    if args.list_ports:
        return args
    if args.mode != "motor" and not args.arduino_port:
        parser.error("--arduino-port 또는 config.ARDUINO_PORT가 필요합니다")
    if args.mode != "sensor" and not args.u2d2_port:
        parser.error("--u2d2-port 또는 config.U2D2_PORT가 필요합니다")
    if args.mode == "integrated" and normalized_port(args.arduino_port) == normalized_port(args.u2d2_port):
        parser.error("Arduino와 U2D2는 서로 다른 COM 포트여야 합니다")
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("--duration은 0 이상의 유한한 숫자여야 합니다")
    if min(args.arduino_baud, args.u2d2_baud) <= 0 or not 0 <= args.motor_id <= 252:
        parser.error("통신속도는 양수, 모터 ID는 0~252여야 합니다")
    return args


def main(argv=None):
    args = arguments(argv)
    if args.list_ports:
        ports = list(list_ports.comports())
        for port in ports:
            print(f"{port.device:8} {port.description} | {port.hwid}")
        if not ports:
            print("No serial ports found.")
        return 0
    metrics = Metrics()
    stop = threading.Event()
    try:
        log = session_log(args.log_dir)
    except OSError as exc:
        print(f"Cannot create log: {exc}", file=sys.stderr)
        return 1
    log.emit("session", mode=args.mode, read_only=True, settings=vars(args), **metadata())
    sensor = (ArduinoReader(args.arduino_port, args.arduino_baud, stop, metrics, log)
              if args.mode != "motor" else None)
    motor = (MotorReader(args.u2d2_port, args.u2d2_baud, args.motor_id, stop, metrics, log)
             if args.mode != "sensor" else None)
    workers = [worker for worker in (sensor, motor) if worker is not None]
    started = time.perf_counter()
    interrupted = False
    healthy = False
    try:
        for worker in workers:
            worker.start()
        while not args.duration or time.perf_counter() - started < args.duration:
            display(sensor, motor, log)
            remaining = args.duration - (time.perf_counter() - started) if args.duration else config.DISPLAY_PERIOD_S
            stop.wait(min(config.DISPLAY_PERIOD_S, max(0, remaining)))
    except KeyboardInterrupt:
        interrupted = True
        print("\nStopping; closing both ports...")
    finally:
        now = time.perf_counter()
        healthy = ((sensor is None or distance_is_fresh(sensor.state.snapshot(), now)) and
                   (motor is None or motor_is_fresh(motor.state.snapshot(), now)))
        stop.set()
        for worker in workers:
            if worker.ident is not None:
                worker.join(timeout=2)
                if worker.is_alive():
                    metrics.count("shutdown.worker_timeout")
                    healthy = False
                    print(f"Warning: {worker.name} did not finish", file=sys.stderr)
        save_summary(log, metrics, read_only=True, healthy_at_end=healthy,
                     interrupted=interrupted, requested_duration_s=args.duration,
                     elapsed_s=time.perf_counter() - started)
    print(f"Communication / valid readings at end: {'OK' if healthy else 'NOT VERIFIED'}")
    return 0 if healthy else 1


if __name__ == "__main__":
    raise SystemExit(main())
