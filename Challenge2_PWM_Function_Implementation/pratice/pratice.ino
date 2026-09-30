const byte LED_PIN = 7;

// period 범위: 100~10000 μs
const int MIN_PERIOD_US = 100;
const int MAX_PERIOD_US = 10000;

// duty 범위: 0~100%
const int MIN_DUTY = 0;
const int MAX_DUTY = 100;

//int periodUs = 10000; // 주기: 10 ms
//int periodUs = 1000;  // 주기: 1 ms
int periodUs = 100;   // 주기: 0.1 ms
int dutyPercent = 0;

// PWM 주기를 마이크로초 단위로 설정
void set_period(int period) {
  periodUs = constrain(period, MIN_PERIOD_US, MAX_PERIOD_US);
}

// PWM duty를 0~100%로 설정
void set_duty(int duty) {
  dutyPercent = constrain(duty, MIN_DUTY, MAX_DUTY);
}

// digitalWrite()와 delayMicroseconds()를 이용해
// 설정된 주기와 duty의 PWM 신호 한 주기를 출력
void writePWMcycle() {
  int highUs = (long)periodUs * dutyPercent / 100;
  int lowUs = periodUs - highUs;

  if (highUs > 0) {
    digitalWrite(LED_PIN, HIGH);
    delayMicroseconds(highUs);
  }

  if (lowUs > 0) {
    digitalWrite(LED_PIN, LOW);
    delayMicroseconds(lowUs);
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);  // 시리얼 플로터 출력 속도
  set_period(1000);      // 기본 주기: 1 ms
  set_duty(0);
}

void loop() {
  // duty 0% → 100%
  for (int duty = 0; duty <= 100; duty++) {
    set_duty(duty);
    unsigned long startMs = millis();

    while (millis() - startMs < 10) {
      writePWMcycle();
    }

    Serial.println(duty);  // 시리얼 플로터에 duty 출력
  }

  // duty 99% → 0%
  for (int duty = 99; duty >= 0; duty--) {
    set_duty(duty);
    unsigned long startMs = millis();

    while (millis() - startMs < 10) {
      writePWMcycle();
    }

    Serial.println(duty);  // 시리얼 플로터에 duty 출력
  }
}
