"""XM430-W350-T / Protocol 2.0. 기본 경로에는 제어 테이블 쓰기가 없다."""

import threading
import time

import dynamixel_sdk as sdk

import config
from telemetry import SharedState

MODEL_NUMBER = 1020
BAUDRATES = {0: 9600, 1: 57600, 2: 115200, 3: 1000000, 4: 2000000,
             5: 3000000, 6: 4000000, 7: 4500000}


class MotorError(RuntimeError):
    def __init__(self, message, *, operation=None, comm_result=None, device_error=None):
        super().__init__(message)
        self.operation = operation
        self.comm_result = comm_result
        self.device_error = device_error


class Dynamixel:
    """하나의 스레드에서만 사용. 쓰기는 별도 승인한 동작 프로그램에서만 허용."""
    def __init__(self, port, baud, motor_id, metrics, log, allow_writes=False):
        self.port_name, self.baud, self.motor_id = port, baud, motor_id
        self.metrics, self.log, self.allow_writes = metrics, log, allow_writes
        self.port = sdk.PortHandler(port)
        self.packet = sdk.PacketHandler(2.0)

    def open(self):
        # SDK setBaudRate는 PC 측 포트만 설정하고 연다. 모터 Baud 레지스터는 안 쓴다.
        if not self.port.setBaudRate(self.baud):
            raise MotorError(f"Cannot open U2D2 {self.port_name} at {self.baud} bps")

    def close(self):
        if self.port.is_open:
            self.port.closePort()

    def _request(self, name, function, *args):
        started = time.perf_counter()
        try:
            result = function(self.port, self.motor_id, *args)
        except IndexError as exc:
            # 응답 지연 후 다른 길이의 패킷을 받으면 SDK 내부 인덱싱이 실패할 수 있음.
            elapsed = (time.perf_counter() - started) * 1000
            self.metrics.observe("motor.request_ms", elapsed)
            self.log.emit("motor_packet_error", operation=name, duration_ms=elapsed,
                          reason="SDK short response", comm_result=sdk.COMM_RX_CORRUPT)
            raise MotorError(f"{name}: SDK received a short/mismatched response packet",
                             operation=name, comm_result=sdk.COMM_RX_CORRUPT) from exc
        elapsed = (time.perf_counter() - started) * 1000
        self.metrics.observe("motor.request_ms", elapsed)
        value, comm, device_error = result
        self.log.emit("motor_transaction", operation=name, duration_ms=elapsed,
                      comm_result=comm, device_error=device_error, value=value)
        if comm != sdk.COMM_SUCCESS:
            raise MotorError(f"{name}: {self.packet.getTxRxResult(comm)}",
                             operation=name, comm_result=comm)
        if device_error:
            raise MotorError(f"{name}: {self.packet.getRxPacketError(device_error)}",
                             operation=name, device_error=device_error)
        self.metrics.observe(f"motor.{name}_ms", elapsed)
        return value

    def ping(self):
        model = self._request("ping", self.packet.ping)
        if model != MODEL_NUMBER:
            raise MotorError(f"Expected XM430-W350 model 1020, received {model}; stopped")
        return model

    def read(self, address, size, name="register"):
        function = getattr(self.packet, f"read{size}ByteTxRx")
        return self._request(name, function, address)

    def position(self):
        raw = self.read(132, 4, "position")
        # Torque OFF에서는 Present Position이 음수/다회전 값일 수도 있다.
        return raw - (1 << 32) if raw & (1 << 31) else raw

    def verify(self):
        model = self.ping()
        actual_id = self.read(7, 1, "id")
        baud_code = self.read(8, 1, "baud")
        if actual_id != self.motor_id or BAUDRATES.get(baud_code) != self.baud:
            raise MotorError(f"ID/baud mismatch: ID={actual_id}, baud code={baud_code}")
        return {"model": model, "motor_id": actual_id, "baud": self.baud,
                "mode": self.read(11, 1, "mode"),
                "torque": self.read(64, 1, "torque")}

    def write(self, address, size, value, name="write"):
        if not self.allow_writes:
            raise MotorError("Writes disabled in read-only communication test")
        function = getattr(self.packet, f"write{size}ByteTxRx")
        started = time.perf_counter()
        comm, device_error = function(self.port, self.motor_id, address, value)
        elapsed = (time.perf_counter() - started) * 1000
        self.metrics.observe("motor.request_ms", elapsed)
        self.log.emit("motor_write", operation=name, address=address, value=value,
                      duration_ms=elapsed, comm_result=comm, device_error=device_error)
        if comm != sdk.COMM_SUCCESS:
            raise MotorError(f"{name}: {self.packet.getTxRxResult(comm)}",
                             operation=name, comm_result=comm)
        if device_error:
            raise MotorError(f"{name}: {self.packet.getRxPacketError(device_error)}",
                             operation=name, device_error=device_error)
        self.metrics.observe(f"motor.{name}_ms", elapsed)
        return elapsed

    def torque(self, enable):
        self.write(64, 1, int(enable), "torque_on" if enable else "torque_off")
        if self.read(64, 1, "torque_verify") != int(enable):
            raise MotorError("Torque state readback mismatch")


class MotorReader(threading.Thread):
    def __init__(self, port, baud, motor_id, stop, metrics, log):
        super().__init__(name="motor-reader", daemon=True)
        self.stop, self.metrics, self.log = stop, metrics, log
        self.state = SharedState()
        self.device = Dynamixel(port, baud, motor_id, metrics, log)

    def run(self):
        try:
            self.device.open()
            self.state.update(port_open=True)
            # 초기 레지스터 조회 중 USB 응답이 일시 누락되면 읽기만 최대 3회 시도.
            # 모델/ID/설정 불일치 및 모터 자체 오류는 재시도로 숨기지 않는다.
            for attempt in range(3):
                try:
                    info = self.device.verify()
                    break
                except MotorError as exc:
                    if (exc.comm_result not in (sdk.COMM_RX_TIMEOUT, sdk.COMM_RX_CORRUPT)
                            or attempt == 2):
                        raise
                    self.metrics.count("motor.startup_retries")
                    self.log.emit("motor_startup_retry", attempt=attempt + 1, reason=str(exc))
                    if self.stop.wait(0.1):
                        raise MotorError("Motor startup cancelled") from exc
            self.state.update(**info)
            self.log.emit("motor_verified", **info)
            while not self.stop.is_set():
                cycle_started = time.perf_counter()
                try:
                    position = self.device.position()
                    finished = time.perf_counter()
                    self.state.update(connected=True, error="", position=position,
                                      updated_at=finished,
                                      position_read_ms=(finished - cycle_started) * 1000)
                    self.log.emit("motor_position", position=position)
                except (MotorError, OSError) as exc:
                    self.state.update(connected=False, error=f"U2D2: {exc}")
                    self.metrics.count("motor.read_errors")
                    self.log.emit("motor_error", reason=str(exc))
                # 밀린 주기를 만회하려고 요청을 몰아서 보내지 않는다.
                self.stop.wait(max(0, config.MOTOR_POLL_S - (time.perf_counter() - cycle_started)))
        except (MotorError, OSError, ValueError) as exc:
            self.state.update(connected=False, error=f"U2D2: {exc}")
            self.metrics.count("motor.connection_errors")
            self.log.emit("motor_error", reason=str(exc))
        finally:
            try:
                self.device.close()  # 기본 조회는 종료 때도 Torque를 변경하지 않음
            except OSError as exc:
                self.log.emit("motor_close_error", reason=str(exc))
            self.state.update(port_open=False, connected=False)
