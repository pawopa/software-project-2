/*
 * 도전과제 1: 주차장 차단기 구현 (과제코드 10C16)
 * 기반 코드: 10_example_3.ino (millis() 기반 서보 속도 제어)
 *
 * 동작
 *  - 초음파센서로 차량 접근 감지 (OPEN_DIST 이하가 연속 몇 번 측정되면 감지)
 *  - 감지되면 서보(차단기 바)를 천천히 부드럽게 올림 (CLOSED -> OPEN)
 *  - 차량이 지나가서 일정 시간 동안 감지되지 않으면 부드럽게 내림 (OPEN -> CLOSED)
 *  - 서보 부드러운 제어 함수 2종 비교: (1) sigmoid  (2) minimum-jerk (smootherstep)
 *    + 비교용 linear (원본 예제와 동일한 등속 이동)
 *
 * 시리얼 모니터(115200bps) 명령으로 실시간 전환
 *   '1' : sigmoid    '2' : smootherstep    '3' : linear
 * 출력 형식 "angle,distance" -> 시리얼 플로터로 곡선 비교 가능
 */

#include <Servo.h>

// ---------------- 핀 설정 ----------------
#define PIN_SERVO 10   // 서보 (Uno에서 9, 10번은 Servo 라이브러리 사용 시 PWM 불가)
#define PIN_TRIG  12   // 초음파 Trig
#define PIN_ECHO  13   // 초음파 Echo

// ---------------- 차단기 설정 ----------------
const int CLOSED_ANGLE = 0;     // 바가 내려온 각도 (실제 조립에 맞게 조정)
const int OPEN_ANGLE   = 90;    // 바가 올라간 각도

const float OPEN_DIST    = 15.0;  // [cm] 이 거리 이내로 들어오면 차량 접근으로 판단
const float RELEASE_DIST = 20.0;  // [cm] 이 거리보다 멀어져야 차량이 지나간 것으로 판단 (히스테리시스)
const int   DETECT_COUNT = 3;     // 연속 N회 감지되어야 열림 (노이즈 방지)

const unsigned long MOVE_TIME  = 3000;  // [ms] 0~90도 전체 이동에 걸리는 시간
const unsigned long CLOSE_WAIT = 2000;  // [ms] 차량이 사라진 뒤 바를 내리기까지 대기 시간
const unsigned long SENSE_INTERVAL = 50; // [ms] 초음파 측정 주기

// ---------------- 부드러운 제어 함수 ----------------
enum EaseMode { EASE_SIGMOID = 1, EASE_SMOOTHER = 2, EASE_LINEAR = 3 };
EaseMode easeMode = EASE_SIGMOID;

const float SIGMOID_K = 12.0;   // sigmoid 기울기 (클수록 중앙이 가파르고 양 끝이 느림)

// (1) sigmoid: a = 1 / (1 + exp(-z)),  z = K*(t - 0.5)
//     t=0, t=1 에서 정확히 0, 1이 되도록 정규화함
float easeSigmoid(float t) {
  float s  = 1.0 / (1.0 + exp(-SIGMOID_K * (t - 0.5)));
  float s0 = 1.0 / (1.0 + exp( SIGMOID_K * 0.5));
  float s1 = 1.0 / (1.0 + exp(-SIGMOID_K * 0.5));
  return (s - s0) / (s1 - s0);
}

// (2) smootherstep (minimum-jerk): 6t^5 - 15t^4 + 10t^3
//     시작/끝에서 속도와 가속도가 모두 0 -> 서보에 충격이 거의 없음
float easeSmoother(float t) {
  return t * t * t * (t * (6.0 * t - 15.0) + 10.0);
}

float ease(float t) {
  if (t <= 0.0) return 0.0;
  if (t >= 1.0) return 1.0;
  switch (easeMode) {
    case EASE_SIGMOID:  return easeSigmoid(t);
    case EASE_SMOOTHER: return easeSmoother(t);
    default:            return t;   // linear
  }
}

// ---------------- 상태 머신 ----------------
enum GateState { GATE_CLOSED, GATE_OPENING, GATE_OPEN, GATE_CLOSING };
GateState state = GATE_CLOSED;

Servo myServo;

float currentAngle = CLOSED_ANGLE;  // 현재 서보 각도
float moveFrom, moveTo;             // 이동 시작/목표 각도
unsigned long moveStart;            // 이동 시작 시각
unsigned long moveDuration;         // 이번 이동 소요 시간

unsigned long lastSense = 0;
unsigned long clearSince = 0;       // 차량이 안 보이기 시작한 시각
int detectCnt = 0;
float distance = 999;

// ---------------- 초음파 거리 측정 ----------------
float readDistanceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long dur = pulseIn(PIN_ECHO, HIGH, 25000); // 약 4m 타임아웃
  if (dur == 0) return 999;                           // 측정 실패 = 물체 없음
  return dur * 0.0343 / 2.0;
}

// ---------------- 서보 이동 시작 ----------------
// 이동 도중 방향이 바뀌어도 현재 각도에서 이어서 움직이도록 시간을 거리에 비례해 계산
void startMove(float target) {
  moveFrom = currentAngle;
  moveTo   = target;
  float ratio = fabs(moveTo - moveFrom) / (OPEN_ANGLE - CLOSED_ANGLE);
  moveDuration = (unsigned long)(MOVE_TIME * ratio);
  if (moveDuration < 50) moveDuration = 50;
  moveStart = millis();
}

// 반환값: 이동이 끝났으면 true
bool updateMove() {
  unsigned long elapsed = millis() - moveStart;
  float t = (float)elapsed / (float)moveDuration;
  currentAngle = moveFrom + (moveTo - moveFrom) * ease(t);
  myServo.write((int)(currentAngle + 0.5));
  return elapsed >= moveDuration;
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  myServo.attach(PIN_SERVO);
  myServo.write(CLOSED_ANGLE);   // 시작은 바가 내려온 상태
  delay(500);
}

void loop() {
  // --- 시리얼 명령으로 제어 함수 전환 ---
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '1') easeMode = EASE_SIGMOID;
    else if (c == '2') easeMode = EASE_SMOOTHER;
    else if (c == '3') easeMode = EASE_LINEAR;
  }

  // --- 주기적으로 초음파 측정 + 차량 감지 판단 ---
  bool carNear = false;   // 이번 측정에서 연속 감지 조건을 만족했는가
  bool carGone = false;   // RELEASE_DIST 보다 멀어졌는가
  if (millis() - lastSense >= SENSE_INTERVAL) {
    lastSense = millis();
    distance = readDistanceCm();

    if (distance <= OPEN_DIST) detectCnt++;
    else detectCnt = 0;

    carNear = (detectCnt >= DETECT_COUNT);
    carGone = (distance > RELEASE_DIST);

    Serial.print(currentAngle);
    Serial.print(",");
    Serial.println(distance > 100 ? 100 : distance);
  }

  // --- 상태 머신 ---
  switch (state) {
    case GATE_CLOSED:
      if (carNear) {
        startMove(OPEN_ANGLE);
        state = GATE_OPENING;
      }
      break;

    case GATE_OPENING:
      if (updateMove()) {
        state = GATE_OPEN;
        clearSince = 0;
      }
      break;

    case GATE_OPEN:
      // 차량이 완전히 지나가 일정 시간 동안 감지되지 않으면 내림
      if (distance <= RELEASE_DIST) {
        clearSince = 0;                       // 아직 차량이 있음 -> 타이머 리셋
      } else if (carGone) {
        if (clearSince == 0) clearSince = millis();
        if (millis() - clearSince >= CLOSE_WAIT) {
          startMove(CLOSED_ANGLE);
          state = GATE_CLOSING;
        }
      }
      break;

    case GATE_CLOSING:
      if (carNear) {                          // 내려가는 중 차량 재접근 -> 다시 올림
        startMove(OPEN_ANGLE);
        state = GATE_OPENING;
      } else if (updateMove()) {
        state = GATE_CLOSED;
        detectCnt = 0;
      }
      break;
  }
}
