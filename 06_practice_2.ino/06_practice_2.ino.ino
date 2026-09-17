// 과제코드: 06P13
// analogWrite()를 이용한 LED 밝기 제어

const int LED_PIN = 9;  // PWM 핀 사용
int brightness = 0;     // LED 밝기: 0 ~ 255
int fadeAmount = 5;     // 밝기 변화량

void setup() {
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // PWM 신호로 LED 밝기 설정
  analogWrite(LED_PIN, brightness);

  // 밝기 변경
  brightness += fadeAmount;

  // 0 또는 255에 도달하면 밝아짐/어두워짐 방향 전환
  if (brightness <= 0 || brightness >= 255) {
    fadeAmount = -fadeAmount;
  }

  delay(30);  // 밝기 변화 속도
}
