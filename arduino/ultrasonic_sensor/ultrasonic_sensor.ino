// Arduino IDE: Arduino UNO R4 Boards > Arduino UNO R4 WiFi 선택
// HC-SR04: VCC=5V, GND=GND, TRIG=D2, ECHO=D3
#include <Arduino.h>

constexpr uint8_t TRIG_PIN = 2;
constexpr uint8_t ECHO_PIN = 3;
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t PERIOD_US = 60000;  // 초기 측정 시작 간격: 약 60 ms
constexpr float SOUND_MM_PER_US = 0.343f;  // 약 20도 기준, 온도 보정 미적용
constexpr float MIN_DISTANCE_MM = 20.0f;
// 실측 전 임시 수집 창: 300 mm 빔 + 센서 장착 여유. 안전/중앙 거리 아님.
constexpr float MAX_DISTANCE_MM = 400.0f;
constexpr uint32_t ECHO_TIMEOUT_US =
    static_cast<uint32_t>(MAX_DISTANCE_MM * 2.0f / SOUND_MM_PER_US) + 1000;

uint32_t previousStartUs = 0;
uint32_t sequence = 0;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  Serial.begin(SERIAL_BAUD);
  // USB 연결을 무한 대기하지 않는다. 재연결 시 PC에서 새 스트림으로 처리.
  previousStartUs = micros() - PERIOD_US;
}

void loop() {
  const uint32_t nowUs = micros();
  // 부호 없는 차감으로 micros() 오버플로 처리. 지연 후 몰아서 측정하지 않음.
  if (static_cast<uint32_t>(nowUs - previousStartUs) < PERIOD_US) return;
  previousStartUs = nowUs;
  const uint32_t measuredAtMs = millis();  // 에코 완료가 아닌 트리거 시작 시각
  ++sequence;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  const unsigned long echoUs = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  const float distanceMm = echoUs * SOUND_MM_PER_US / 2.0f;
  const bool valid = echoUs != 0 && distanceMm >= MIN_DISTANCE_MM &&
                     distanceMm <= MAX_DISTANCE_MM;
  const char* status = echoUs == 0 ? "TIMEOUT" :
      (distanceMm < MIN_DISTANCE_MM ? "TOO_NEAR" :
      (distanceMm > MAX_DISTANCE_MM ? "TOO_FAR" : "OK"));

  if (Serial) {
    // 앞 5필드는 기존과 동일. 뒤에 원인/에코폭/원시거리/타임아웃을 추가.
    // 무효 원시거리는 진단 전용이며 PID 측정값으로 사용하지 않는다.
    Serial.print("S,");
    Serial.print(sequence);
    Serial.print(',');
    Serial.print(measuredAtMs);
    Serial.print(',');
    Serial.print(valid ? distanceMm : -1.0f, 1);
    Serial.print(',');
    Serial.print(valid ? 1 : 0);
    Serial.print(',');
    Serial.print(status);
    Serial.print(',');
    Serial.print(echoUs);
    Serial.print(',');
    Serial.print(echoUs == 0 ? -1.0f : distanceMm, 1);
    Serial.print(',');
    Serial.println(ECHO_TIMEOUT_US);
  }
}
