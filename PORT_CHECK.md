> **보관 문서:** 테스트 코드 정리 이전의 기록입니다. 삭제된 프로그램의 명령/경로가 포함될 수 있습니다. 현재 실행 방법과 파일 구성은 프로젝트 루트 README.md를 참조하세요.

# COM3/COM4 점유 확인

장치 관리자에 포트가 표시되는 것은 장치 인식 여부이며, 프로그램의 점유 여부는 별도로 확인한다.

프로젝트 루트에서 실행:

```powershell
.\.venv\Scripts\python.exe python\check_ports.py
```

현재 설정은 COM3=U2D2(1000000bps), COM4=Arduino(115200bps). 데이터는 보내지 않고 포트를 열고 즉시 닫는다. Arduino는 열기 과정에서 리셋될 수 있다.

- `OPEN OK`: 확인 순간 해당 포트를 이 프로그램에서 열 수 있었다.
- `OPEN FAILED`: 다른 프로그램 점유 또는 권한/드라이버 문제. 오류 내용을 확인한다.
- `포트 없음`: 연결 해제/COM 번호 변경 등을 먼저 확인한다.

PID가 실행 중이면 COM3/COM4는 해당 PID프로그램이 사용하므로 점유로 나오는 것이 정상이다. PID 종료 후 두 포트가 OPEN OK로 돌아오는지 확인한다. OPEN OK는 모터 전원/응답 정상 판정이나 과거 점유 없음의 증거가 아니다.

## 먼저 닫을 프로그램

1. Arduino IDE/VS Code의 시리얼 모니터를 종료한다.
2. DYNAMIXEL Wizard의 Disconnect로 포트 연결을 해제한다.
3. PID/P/키보드/통합 프로그램이 실행 중인 터미널에서 Ctrl+C로 종료한다.
4. 점유 확인을 다시 실행한다.

## 실제 점유 프로세스 찾기 (막힐 때)

[Microsoft Process Explorer](https://learn.microsoft.com/en-us/sysinternals/downloads/process-explorer)를 관리자 권한으로 실행하고 Find → Find Handle or DLL(Ctrl+F)로 장치 경로를 검색한다. 이 PC에서 조회한 매핑:

| 포트 | 검색할 장치 경로 |
|---|---|
| COM3 모터 | `\Device\VCP0` 또는 `VCP0` |
| COM4 Arduino | `\Device\USBSER000` 또는 `USBSER000` |

검색된 프로세스/PID를 확인하고 해당 프로그램에서 연결을 정상 해제한다. COM 번호 문자만 검색하면 내부 핸들 이름이 달라 안 나올 수 있다. 재연결 후 매핑이 바뀌면 PowerShell에서 다음으로 재확인한다.

```powershell
Get-ItemProperty 'HKLM:\HARDWARE\DEVICEMAP\SERIALCOMM'
```

## 이번 실제 확인

COM3/COM4 모두 OPEN OK. 모터/센서 데이터 및 구동 명령은 보내지 않았다. 이는 점검 시점의 포트 접근 확인이며, PID 중 발생하는 응답 타임아웃 원인까지 해결됐다는 뜻은 아니다.
