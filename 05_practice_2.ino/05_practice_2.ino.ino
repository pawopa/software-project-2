const int LED_PIN = 7;

// Active-Low LED 모듈용 설정
const int LED_ON  = LOW;
const int LED_OFF = HIGH;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);

  // 처음 1초 동안 켜기
  Serial.println("LED ON: 1초 유지");
  digitalWrite(LED_PIN, LED_ON);
  delay(1000);

  // 1초 동안 5회 깜빡이기
  for (int i = 1; i <= 5; i++) {
    Serial.print("Blink ");
    Serial.print(i);
    Serial.println(": ON");

    digitalWrite(LED_PIN, LED_ON);
    delay(100);

    Serial.print("Blink ");
    Serial.print(i);
    Serial.println(": OFF");

    digitalWrite(LED_PIN, LED_OFF);
    delay(100);
  }

  // 마지막에 LED 끄기
  digitalWrite(LED_PIN, LED_OFF);
  Serial.println("완료: LED OFF");
}

void loop() {
  // 종료 후 LED는 계속 꺼진 상태
  digitalWrite(LED_PIN, LED_OFF);
}
