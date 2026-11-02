#include <Servo.h>

// ===== 실험 설정 (여기만 바꿔서 두 번 실험) =====
#define _SERVO_SPEED   3      // 단위: 도/초 (3 → 60초에 180°, 0.3 → 300초에 90°)
#define _TARGET_ANGLE  180    // 실험1: 180, 실험2: 90
// ================================================

#define PIN_SERVO   10
#define INTERVAL    20        // 제어 주기(ms)
#define _DUTY_MIN   520       // 0°   펄스폭(us)
#define _DUTY_NEU   1500      // 90°  펄스폭(us)
#define _DUTY_MAX   2500      // 180° 펄스폭(us)

Servo myservo;

float start_angle = 0.0;
unsigned long t_start;
unsigned long t_last;
bool finished = false;

// 각도(0~180) -> 펄스폭(us): MIN-NEU-MAX 두 구간 선형 보간
int angleToUs(float angle) {
  if (angle <= 90.0) {
    return _DUTY_MIN + (angle / 90.0) * (_DUTY_NEU - _DUTY_MIN);
  } else {
    return _DUTY_NEU + ((angle - 90.0) / 90.0) * (_DUTY_MAX - _DUTY_NEU);
  }
}

void setServoAngle(float angle) {
  myservo.writeMicroseconds(angleToUs(angle));
}

void setup() {
  Serial.begin(57600);
  myservo.attach(PIN_SERVO);

  setServoAngle(start_angle);      // 시작 위치 0°
  delay(1000);                     // 시작 위치로 이동할 시간

  Serial.print("SERVO_SPEED = ");
  Serial.print(_SERVO_SPEED);
  Serial.print(" deg/s, target = ");
  Serial.println(_TARGET_ANGLE);

  t_start = millis();
  t_last = t_start;
}

void loop() {
  if (finished) return;

  unsigned long now = millis();
  if (now - t_last < INTERVAL) return;
  t_last += INTERVAL;

  // 경과 시간으로부터 각도를 계산 -> 누적 오차 없이 등속 이동
  float elapsed = (now - t_start) / 1000.0;
  float angle = start_angle + _SERVO_SPEED * elapsed;

  if (angle >= _TARGET_ANGLE) {
    angle = _TARGET_ANGLE;
    finished = true;
  }
  setServoAngle(angle);

  Serial.print("time(s):");
  Serial.print(elapsed, 1);
  Serial.print(", angle(deg):");
  Serial.println(angle, 1);

  if (finished) {
    Serial.println("DONE");
  }
}
