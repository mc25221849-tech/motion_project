# Ball & Beam — 통합 수신·모터 왕복·PID

## 팀원 개발 환경 설정

초대 수락 후 비공개 저장소를 복제한다. GitHub 로그인과 저장소 접근 권한이 필요하다.

```powershell
git clone https://github.com/mc25221849-tech/motion_project.git
cd motion_project
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r python\requirements.txt
```

Python 3.13.7에서 실행을 확인했다. Arduino 개발 PC는 Arduino IDE와 **Arduino UNO R4 Boards** 패키지를 설치하고 UNO R4 WiFi를 선택한다. COM 번호는 PC마다 다르므로 `python/config.py` 또는 명령 인수로 지정한다. VS Code 업로드 작업의 Arduino CLI 경로도 설치 위치가 다르면 `.vscode/tasks.json`에서 수정한다. 연결된 기구 없이 `--drive`/`--run`을 실행하지 않는다.

소스·펌웨어·설정·문서를 Git으로 공유하며 `.venv`, `.build`, `logs`, Python 캐시와 인증 정보 파일은 제외한다. 기존 실험 로그는 각 PC에 보존되고 새 로그도 로컬에 생성된다.

팀원은 작업 전 `git pull --ff-only` 후 새 브랜치에서 수정하고 Pull Request로 합친다.

```powershell
git switch -c feature/my-change
# 수정 후
git add <수정한파일>
git commit -m "Describe the change"
git push -u origin feature/my-change
```

실행 프로그램은 다음 세 개다. 나머지 Python 파일은 이 프로그램들이 사용하는 공통 코드다.

| 실행 파일 | 기능 |
|---|---|
| `python/integration_test.py` | 센서/모터 개별 또는 동시 읽기, 모터 명령 없음 |
| `python/motor_sweep.py` | 실측 범위 내 모터 5회 왕복 |
| `python/pid_control.py` | 중앙 108mm 목표 PID 계산 및 실제 구동 |

## 실행

프로젝트 폴더의 PowerShell에서 실행한다. Arduino 시리얼 모니터, DYNAMIXEL Wizard, 다른 제어 프로그램이 같은 COM 포트를 점유하지 않도록 종료한다.

```powershell
# 센서만 10초 수신 (모터 사용 안 함)
.\.venv\Scripts\python.exe python\integration_test.py --mode sensor --duration 10

# 센서와 모터 위치 동시 수신 (회전 명령 없음)
.\.venv\Scripts\python.exe python\integration_test.py --duration 30

# 모터 실제 5왕복. 위쪽 초기 정렬 후 아래쪽→위쪽을 1회로 계산
.\.venv\Scripts\python.exe python\motor_sweep.py --run --cycles 5

# PID 계산 미리보기 (회전 명령 없음)
.\.venv\Scripts\python.exe python\pid_control.py --duration 10

# PID 실제 구동
.\.venv\Scripts\python.exe python\pid_control.py --drive --duration 30
```

Ctrl+C로 중단한다. 왕복은 저속으로 약 6~7분 예상되며, 목표 미달·정체·통신/장치 오류는 중단한다. 모터 단독 왕복은 사용자 확인상 정상이다. PID 실물 중앙 유지와 외란 복귀 성능은 아직 검증 완료되지 않았다.

## 연결 및 실측 기준

| 항목 | 설정 |
|---|---|
| Arduino UNO R4 WiFi / HC-SR04 | COM4, 115200bps, TRIG D2 / ECHO D3, VCC 5V / GND |
| U2D2 / XM430-W350-T | COM3, ID 15, 1000000bps, Hub 외부 12V |
| 카트 중앙 | 초음파 거리 108mm |
| 빔 위쪽 한계 / 수평 / 아래쪽 한계 | 모터 위치 209.97° / 267.28° / 311.57° |
| 안쪽 정수화한 한계 / 수평 | 2389~3544tick / 3041tick |

모터 위치 각도와 빔 자체 기울기는 다르다. 기준값과 포트는 `python/config.py`에 있다. 재연결 후 COM 번호가 바뀌면 실행 인수 또는 설정 파일을 수정한다.

## Arduino 펌웨어

`arduino/ultrasonic_sensor/ultrasonic_sensor.ino`를 UNO R4 WiFi에 업로드한다. VS Code에서 Ctrl+Shift+P → 작업 실행 → **통합: 1. 60ms 센서 펌웨어 업로드**를 선택할 수 있다. 빌드 도구 설정은 `.vscode/arduino-cli.yaml`에 있으며 현재 PC의 Arduino IDE 설치 경로를 사용한다. `.build`는 필요 시 다시 생성되는 캐시다.

출력: `S,순번,측정시각ms,유효거리mm,유효성,상태,에코폭us,원시거리mm,타임아웃us`.

- 약 60ms 간격, 유효 거리 20~400mm, 에코 대기 약 3332μs.
- `OK`: 정상, `TIMEOUT`: 에코 측정 미완료, `TOO_NEAR`: 20mm 미만, `TOO_FAR`: 400mm 초과.
- 원시 거리는 진단용이다. 무효값은 PID에 사용하지 않는다.
- 이전 5필드 프레임도 수신 가능하지만 상세 무효 원인은 알 수 없다.
- 최근 진단에서는 초반 연속 TIMEOUT 후 정상 거리 수신을 확인했다. 안정적인 반사판/카트 측정은 여전히 확인할 항목이다.

## PID 사용 조건

시작은 Torque OFF, 수평 부근 3019~3063tick에서 준비한다. 자동으로 큰 각도를 수평으로 복귀시키지는 않는다. 시작 위치를 Goal에 미리 기록한 뒤 Torque ON한다.

P 복귀 규칙: 거리>108mm이면 빔 위쪽(모터 각도 감소), 거리<108mm이면 빔 아래쪽(모터 각도 증가). I는 잔류 오차를 보정하고 D는 제동하므로 전체 출력은 순간적으로 P 방향과 반대일 수 있다.

미튜닝 초기값: Kp 0.05 / Ki 0.003 / Kd 0.02, 수평 ±2°, 명령 변화율 2°/s, 적분 출력 ±1°. 위치/미분 필터와 과누적 방지가 포함돼 있다. `--kp`, `--ki`, `--kd`, `--max-offset-deg` 등은 `--help`로 확인한다.

센서 무효·180ms 이상 오래된 데이터·범위 이탈·모터 추종 실패·통신/예외 발생 시 중단한다. 종료 시 Torque OFF와 임시 프로파일 복원을 시도한다. Torque OFF 시 빔이 자중으로 움직일 수 있으므로 지지한다. 통신 단절 시 소프트웨어 정지는 보장할 수 없어 외부 전원 정지 수단이 필요하다.

## 공통 코드

- `config.py`: 통신 및 실측 기준
- `arduino_serial.py`: 센서 수신/진단
- `dynamixel_control.py`: SDK 통신
- `motor_preflight.py`: 구동 전 읽기 전용 조건 검사
- `motor_session.py`: Torque·프로파일·현재 위치/오류 관리
- `control_runtime.py`: PID의 센서/모터 실행 루프와 정리
- `telemetry.py`: 통계 및 비동기 로그

가상환경 `.venv`는 유지한다. 새 환경에서는 `python -m venv .venv` 후 `.\.venv\Scripts\python.exe -m pip install -r python\requirements.txt`를 실행한다. 실행 로그는 `logs/`에 새로 생성된다.
