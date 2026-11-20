"""BB300-R3 통신 설정과 사용자 실측 모터/카트 중앙 기준."""

import math

ARDUINO_PORT = "COM4"  # 2026-11-13 60ms 센서 프레임 동시 수신 확인. 재연결 후 번호 확인
U2D2_PORT = "COM3"   # 2026-11-13 실제 XM430 응답 확인. 재연결 후 번호 확인
ARDUINO_BAUD = 115200
U2D2_BAUD = 1000000  # 2026-11-13 읽기 전용 검색 및 Baud 레지스터 검증
MOTOR_ID = 15        # 2026-11-13 Ping 및 ID 레지스터 검증. 모터 내부 설정 변경 없음
SENSOR_TIMEOUT_S = 0.05
SENSOR_STALE_S = 0.3
SENSOR_STARTUP_S = 3.0
MOTOR_POLL_S = 0.06
MOTOR_STALE_S = 0.5
DISPLAY_PERIOD_S = 0.5

# 약 300 mm 빔과 장착 오프셋 여유를 둔 초기 '수집 창'이며 실측 범위가 아님.
# 펌웨어의 MIN/MAX_DISTANCE_MM과 함께 변경할 것. 기구 안전 범위와 무관.
MIN_DISTANCE_MM = 20.0
MAX_DISTANCE_MM = 400.0

# 2026-11-13 사용자 실측. 단위는 DYNAMIXEL Wizard의 모터 위치 각도(°).
# 빔 자체의 기울기 각도가 아니다. 모터 각도 감소 -> 빔 위쪽, 증가 -> 아래쪽.
BEAM_UP_MOTOR_DEG = 209.97
BEAM_LEVEL_MOTOR_DEG = 267.28
BEAM_DOWN_MOTOR_DEG = 311.57

# XM430: 1회전=4096tick. 한계는 반올림으로 실측 범위를 넘지 않게 안쪽 정수화.
MOTOR_MEASURED_MIN_TICK = math.ceil(BEAM_UP_MOTOR_DEG * 4096 / 360)    # 2389
MOTOR_LEVEL_TICK = round(BEAM_LEVEL_MOTOR_DEG * 4096 / 360)            # 3041
MOTOR_MEASURED_MAX_TICK = math.floor(BEAM_DOWN_MOTOR_DEG * 4096 / 360)  # 3544
TARGET_CART_DISTANCE_MM = 108.0  # 사용자 실측 카트 중앙: 초음파 센서 거리 기준
# 향후 제어용 기준 데이터. 기존 키보드 프로그램의 동작 범위를 자동 변경하지 않는다.
# 자동 제어 여유 범위와 모터각->빔각 변환은 별도로 결정해야 한다.
