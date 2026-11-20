> **보관 문서:** 테스트 코드 정리 이전의 기록입니다. 삭제된 프로그램의 명령/경로가 포함될 수 있습니다. 현재 실행 방법과 파일 구성은 프로젝트 루트 README.md를 참조하세요.

# 초음파 센서 상세 진단

`arduino/ultrasonic_sensor/ultrasonic_sensor.ino`와 Python 수신/PID 화면에 진단 정보를 추가했다. 측정 주기60ms, 유효거리20~400mm, 타임아웃 약3332μs는 그대로다. 원인을 확인하기 위한 변경이며 유효성 조건을 느슨하게 바꾸지 않았다.

## 새 펌웨어 업로드

시리얼 모니터/PID의 COM4 연결을 닫고 VS Code 작업 **통합: 1. 60ms 센서 펌웨어 업로드**를 실행한다. 포트는COM4, 보드는UNO R4 WiFi다. 1초 진단 스케치가 아니라 기존 통합 스케치를 업데이트해야 한다.

업로드 후 모터를 움직이지 않고 센서만 확인:

VS Code 작업 **센서: 상세 진단 (모터 미사용, 10초)**로도 실행한다.

```powershell
.\.venv\Scripts\python.exe python\integration_test.py --mode sensor --duration 10
```

PID 계산 미리보기에서도 같은 진단을 볼 수 있다.

```powershell
.\.venv\Scripts\python.exe python\pid_control.py --duration 10
```

## 화면/로그

```text
[센서진단] TIMEOUT (에코 없음/타임아웃) | echo=0us | raw=-- | timeout=3332us | seq=52
[센서진단] TOO_NEAR (너무 가까움) | echo=58us | raw=9.9mm | timeout=3332us | seq=53
[센서진단] TOO_FAR (수집 범위 초과) | echo=2624us | raw=450.0mm | timeout=3332us | seq=54
[센서진단] OK (정상) | echo=847us | raw=145.3mm | timeout=3332us | seq=55
```

위 숫자는 형식 예시이며 실측값이 아니다.

- **TIMEOUT**: `pulseIn`이0을 반환함. 반사가 없거나, ECHO가 정상 펄스를 만들지 못했거나, 대기 시간 내 측정을 끝내지 못한 경우다. 이것만으로 배선 고장이나 먼 거리를 확정하지 않는다. 거리 계산 자체가 안 됐으므로 raw는`--`다.
- **TOO_NEAR/TOO_FAR**: 에코를 받았으나 환산 거리가 설정 범위 밖. raw에 원시 환산 거리를 표시한다. 타임아웃을 넘는 긴 에코는 TOO_FAR 대신 TIMEOUT이 될 수 있다.
- **OK**: 에코가 있고 수집 범위 안. 정확한 카트 반사판을 보고 있는지까지 보장하지는 않는다.
- **UNKNOWN_INVALID**: 옛5필드 프레임. 원인 정보를 안 보내므로 추정하지 않고 새 펌웨어 업로드를 안내한다.

PID 무효값/180ms 최신성 보호는 그대로 유지한다. 진단용 raw값이 있어도 valid=false이면 제어 거리로 사용하지 않는다. 마지막 정상값도 과거값으로만 보관한다.

`events.jsonl`의 센서 이벤트에 `status`, `echo_us`, `raw_distance_mm`, `timeout_us`, `firmware_diagnostics`를 기록한다. `summary.json`은 무효 원인별 횟수(`sensor.invalid_reason.TIMEOUT` 등)를 저장하며 P/PID 요약에는 마지막`sensor_diagnostic`도 포함한다.

## 시리얼 프로토콜

```text
S,순번,측정시각ms,유효거리mm,유효성,상태,에코폭us,원시거리mm,타임아웃us
S,52,3120,-1.0,0,TIMEOUT,0,-1.0,3332
```

앞5필드는 기존 형식과 같다. Python은5필드/9필드를 모두 지원한다. 새 펌웨어의9필드 프레임을 읽으려면 업데이트한 Python 수신기를 사용한다. 파서는 상태/유효성/에코/거리의 일치도 검사한다.

## 이번 적용 결과

진단 펌웨어를 COM4에 실제 업로드했다. 센서만5초 읽은 결과83프레임 모두 TIMEOUT, echo=0, timeout=3332μs로 확인했다. 이는 에코 측정 미완료를 뜻하며 실제 거리가400mm를 넘었다고 판정한 것은 아니다. 로그 `logs/20261113_180524_487188/`. 모터 접근/구동은 수행하지 않았다.
