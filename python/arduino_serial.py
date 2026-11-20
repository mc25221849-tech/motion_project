"""Arduino 전용 수신기: 부분 프레임 처리, 최신값 보관, 끊김/지연 계측."""

from dataclasses import dataclass
import math
import threading
import time

import serial

import config
from telemetry import SharedState

UINT32_MASK = 0xFFFFFFFF


@dataclass(frozen=True)
class Sample:
    sequence: int
    arduino_ms: int
    distance_mm: float | None
    valid: bool
    received_at: float
    status: str = "LEGACY"
    echo_us: int | None = None
    raw_distance_mm: float | None = None
    timeout_us: int | None = None


def parse_sample(raw, received_at):
    fields = raw.decode("ascii").strip().split(",")
    if len(fields) not in (5, 9) or fields[0] != "S" or fields[4] not in ("0", "1"):
        raise ValueError("Expected 5-field legacy or 9-field diagnostic S frame")
    sequence, arduino_ms = int(fields[1]), int(fields[2])
    distance = float(fields[3])
    if not (0 <= sequence <= UINT32_MASK and 0 <= arduino_ms <= UINT32_MASK):
        raise ValueError("Sequence/time must be uint32")
    valid = fields[4] == "1"
    if not math.isfinite(distance):
        raise ValueError("Non-finite distance")
    if valid and not config.MIN_DISTANCE_MM <= distance <= config.MAX_DISTANCE_MM:
        raise ValueError("Distance outside configured acquisition window")
    if not valid and distance != -1.0:
        raise ValueError("Invalid measurement must use -1.0 sentinel")
    status = "OK" if valid else "UNKNOWN_INVALID"
    echo_us = raw_distance = timeout_us = None
    if len(fields) == 9:
        status, echo_text, raw_text, timeout_text = fields[5:]
        echo_us, timeout_us = int(echo_text), int(timeout_text)
        raw_distance = float(raw_text)
        if (status not in ("OK", "TIMEOUT", "TOO_NEAR", "TOO_FAR") or
                not 0 <= echo_us <= UINT32_MASK or not 0 < timeout_us <= UINT32_MASK or
                not math.isfinite(raw_distance) or (status == "OK") != valid):
            raise ValueError("Invalid diagnostic fields")
        if status == "TIMEOUT":
            if echo_us != 0 or raw_distance != -1:
                raise ValueError("TIMEOUT requires echo=0 and raw distance=-1")
            raw_distance = None
        else:
            if echo_us == 0 or raw_distance < 0 or abs(raw_distance - echo_us * 0.343 / 2) > 0.11:
                raise ValueError("Echo and raw distance disagree")
            if valid and abs(raw_distance - distance) > 0.11:
                raise ValueError("Valid and raw distance disagree")
            if status == "TOO_NEAR" and raw_distance >= config.MIN_DISTANCE_MM:
                raise ValueError("TOO_NEAR conflicts with configured range")
            if status == "TOO_FAR" and raw_distance <= config.MAX_DISTANCE_MM:
                raise ValueError("TOO_FAR conflicts with configured range")
    return Sample(sequence, arduino_ms, distance if valid else None, valid, received_at,
                  status, echo_us, raw_distance, timeout_us)


def diagnostic_fields(sample):
    return {"status": sample.status, "echo_us": sample.echo_us,
            "raw_distance_mm": sample.raw_distance_mm, "timeout_us": sample.timeout_us,
            "firmware_diagnostics": sample.echo_us is not None}


def format_diagnostic(snapshot):
    sample = snapshot.get("sample")
    if sample is None:
        return "[센서진단] 아직 프레임 없음 | " + snapshot.get("error", "대기 중")
    labels = {"OK": "정상", "TIMEOUT": "에코 없음/타임아웃",
              "TOO_NEAR": "너무 가까움", "TOO_FAR": "수집 범위 초과",
              "UNKNOWN_INVALID": "무효 원인 미포함", "LEGACY": "진단 미포함"}
    detail = diagnostic_fields(sample)
    suffix = "" if detail["firmware_diagnostics"] else " | 상세 진단은 새 펌웨어 업로드 필요"
    raw = f"{sample.raw_distance_mm:.1f}mm" if sample.raw_distance_mm is not None else "--"
    return (f"[센서진단] {sample.status} ({labels.get(sample.status, sample.status)}) | "
            f"echo={sample.echo_us if sample.echo_us is not None else '--'}us | raw={raw} | "
            f"timeout={sample.timeout_us if sample.timeout_us is not None else '--'}us | "
            f"seq={sample.sequence}{suffix}")


class LineFramer:
    """개행 전 버퍼를 128바이트로 제한하고 비정상 행의 나머지도 버린다."""
    def __init__(self):
        self.buffer = bytearray()
        self.discarding = False

    def feed(self, chunk):
        lines = []
        dropped = 0
        for byte in chunk:
            if byte == 10:
                if not self.discarding and self.buffer:
                    lines.append(bytes(self.buffer))
                self.buffer.clear()
                self.discarding = False
            elif not self.discarding:
                self.buffer.append(byte)
                if len(self.buffer) > 128:
                    self.buffer.clear()
                    self.discarding = True
                    dropped += 1
        return lines, dropped


def distance_is_fresh(snapshot, now, stale_s=config.SENSOR_STALE_S):
    sample = snapshot.get("sample")
    return bool(snapshot.get("connected") and sample and sample.valid and
                now - sample.received_at <= stale_s)


class ArduinoReader(threading.Thread):
    def __init__(self, port, baud, stop, metrics, log):
        super().__init__(name="arduino-reader", daemon=True)
        self.port, self.baud = port, baud
        self.stop, self.metrics, self.log = stop, metrics, log
        self.state = SharedState()
        self._previous = None

    def _accept(self, sample):
        previous = self._previous
        interval = None
        if previous is not None:
            seq_delta = (sample.sequence - previous.sequence) & UINT32_MASK
            time_delta = (sample.arduino_ms - previous.arduino_ms) & UINT32_MASK
            if seq_delta == 0:
                self.metrics.count("sensor.duplicate")
                return
            if seq_delta >= 0x80000000 or time_delta >= 0x80000000:
                if (sample.sequence < previous.sequence and
                        sample.arduino_ms < previous.arduino_ms):
                    self.metrics.count("sensor.restart")
                    self.log.emit("sensor_restart")
                else:
                    self.metrics.count("sensor.out_of_order")
                    return
            else:
                arrival_ms = (sample.received_at - previous.received_at) * 1000
                self.metrics.observe("sensor.arrival_interval_ms", arrival_ms)
                if seq_delta == 1:
                    interval = time_delta
                    self.metrics.observe("sensor.trigger_interval_ms", time_delta)
                    # 시계 동기화 없이 비교 가능한 간격 차이. 절대 편도 지연이 아님.
                    self.metrics.observe("sensor.interval_difference_ms", arrival_ms - time_delta)
                else:
                    self.metrics.count("sensor.sequence_gaps", seq_delta - 1)
        self._previous = sample
        self.metrics.count("sensor.frames")
        updates = {"sample": sample, "connected": True,
                   "sensor_interval_ms": interval,
                   "diagnostic": diagnostic_fields(sample),
                   "error": "" if sample.valid else f"거리 측정 실패: {sample.status}"}
        if sample.valid:
            updates["last_valid"] = sample
        else:
            self.metrics.count("sensor.invalid_echo")
            self.metrics.count(f"sensor.invalid_reason.{sample.status}")
        self.state.update(**updates)
        self.log.emit("sensor", sequence=sample.sequence, arduino_ms=sample.arduino_ms,
                      distance_mm=sample.distance_mm, valid=sample.valid,
                      received_at=sample.received_at, trigger_interval_ms=interval,
                      **diagnostic_fields(sample))

    def run(self):
        connection = None
        framer = LineFramer()
        try:
            connection = serial.Serial(self.port, self.baud, timeout=config.SENSOR_TIMEOUT_S)
            # 연결 전 누적값은 버린다. 보드가 리셋되어도 이 스레드에서만 대기한다.
            connection.reset_input_buffer()
            self.state.update(port_open=True, error="Port open; waiting for sensor frames")
            self.log.emit("sensor_port_open", port=self.port, baud=self.baud)
            opened_at = time.perf_counter()
            while not self.stop.is_set():
                if connection.in_waiting > 4096:
                    connection.reset_input_buffer()
                    framer = LineFramer()
                    self.metrics.count("sensor.backlog_resets")
                    self.state.update(connected=False, error="Discarded serial backlog")
                    self.log.emit("sensor_backlog_discarded")
                started = time.perf_counter()
                chunk = connection.read(min(max(connection.in_waiting, 1), 4096))
                arrived = time.perf_counter()
                self.metrics.observe("sensor.read_call_ms", (arrived - started) * 1000)
                if chunk:
                    parsing_started = time.perf_counter()
                    lines, dropped = framer.feed(chunk)
                    self.metrics.count("sensor.overlong_lines", dropped)
                    if len(lines) > 1:
                        self.metrics.count("sensor.burst_batches")
                    for line in lines:
                        parse_started = time.perf_counter()
                        try:
                            sample = parse_sample(line, arrived)
                        except (ValueError, UnicodeError) as exc:
                            self.metrics.count("sensor.malformed")
                            self.state.update(error=f"Malformed sensor frame: {exc}")
                            self.log.emit("sensor_parse_error", reason=str(exc))
                            continue
                        self.metrics.observe("sensor.parse_ms", (time.perf_counter() - parse_started) * 1000)
                        self._accept(sample)
                    self.metrics.observe("sensor.batch_processing_ms",
                                         (time.perf_counter() - parsing_started) * 1000)
                latest = self.state.snapshot().get("sample")
                if latest and time.perf_counter() - latest.received_at > config.SENSOR_STALE_S:
                    self.state.update(connected=False, error="Sensor stream stale / disconnected")
                elif latest is None and time.perf_counter() - opened_at > config.SENSOR_STARTUP_S:
                    self.state.update(connected=False, error="No sensor frames; check sketch / baud / port")
        except (serial.SerialException, OSError, ValueError) as exc:
            self.state.update(connected=False, error=f"Arduino {self.port}: {exc}")
            self.metrics.count("sensor.connection_errors")
            self.log.emit("sensor_error", reason=str(exc))
        finally:
            if connection is not None:
                try:
                    connection.close()
                except (serial.SerialException, OSError) as exc:
                    self.log.emit("sensor_close_error", reason=str(exc))
            self.state.update(port_open=False, connected=False)
