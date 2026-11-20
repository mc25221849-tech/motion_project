> **보관 문서:** 테스트 코드 정리 이전의 기록입니다. 삭제된 프로그램의 명령/경로가 포함될 수 있습니다. 현재 실행 방법과 파일 구성은 프로젝트 루트 README.md를 참조하세요.

# 초음파 센서 단독 확인 — 1초 간격

## VS Code에서 업로드하고 보기

이 PC에 설치된 Arduino CLI를 사용하는 `.vscode/tasks.json`을 추가했다. 별도 VS Code 확장 설치 없이 실행한다.

1. Arduino IDE 시리얼 모니터와 COM4를 사용하는 Python 프로그램을 종료한다.
2. VS Code에서 프로젝트 폴더를 연 상태로 **Ctrl+Shift+P → Tasks: Run Task(작업 실행)**를 선택한다.
3. **초음파: 업로드 (UNO R4 WiFi)** → 포트 **COM4** 확인 → Enter. 아래 터미널에서 업로드 완료를 기다린다.
4. 다시 작업 실행에서 **초음파: 시리얼 모니터 (115200)** → **COM4**를 선택한다.
5. VS Code 아래 터미널에서 약 1초마다 거리값을 확인한다. 종료는 해당 터미널에서 **Ctrl+C**다.

다시 업로드하기 전에 실행 중인 시리얼 모니터를 Ctrl+C로 종료한다. COM 번호가 바뀌었으면 입력창에서 실제 포트를 지정한다. 이 설정은 현재 PC의 Arduino IDE 설치 경로와 `.build/arduino-cli.yaml`을 사용한다. 업로드/모니터 작업은 사용자가 실행할 때 수행되며, 설정 추가만으로 보드에 업로드하지 않는다.

## Arduino IDE에서 사용

1. Arduino IDE에서 `ultrasonic_check_1s.ino`를 연다.
2. 보드는 **Arduino UNO R4 WiFi**, 포트는 Arduino의 실제 포트(이전 확인 당시 COM4)를 선택한다.
3. 업로드한 뒤 시리얼 모니터를 **115200 baud**로 연다.
4. 센서 앞 반사판/물체를 이동시키며 1초마다 거리값이 바뀌는지 확인한다.

배선: HC-SR04 VCC → 5V, GND → GND, TRIG → D2, ECHO → D3.

출력 예시(실측값 아님):

```text
#2 | Time: 1000 ms | Interval: 1000 ms | Distance: 145.2 mm (14.52 cm)
#3 | Time: 2000 ms | Interval: 1000 ms | Distance: 180.1 mm (18.01 cm)
#4 | Time: 3000 ms | Interval: 1000 ms | NO ECHO (timeout): check target / wiring
```

`Interval`은 실제 측정 시작 간격이다. 첫 줄의 간격은 초기화를 위한 값이므로 주기 확인은 두 번째 출력부터 한다. 실패 시 0mm 대신 `NO ECHO`를 표시한다. 물체 방향/거리, 반사판, D2/D3 배선 및 전원을 확인한다.

센서 자체 확인을 위해 기존 통합 프로그램의 20~400mm 수집 창은 적용하지 않는다. 측정 에코를 그대로 거리로 환산하며 무응답 대기는 30ms로 제한한다. 온도 보정 및 정확도 교정은 하지 않았다.

이 코드는 사람이 읽는 시리얼 모니터용이다. 기존 Python 통합 수신기의 `S,...` 형식과 다르다. 통합 수신을 다시 사용할 때는 `../ultrasonic_sensor/ultrasonic_sensor.ino`를 업로드한다. Arduino 포트를 사용하는 Python 프로그램은 업로드/시리얼 모니터 사용 전에 종료한다.
