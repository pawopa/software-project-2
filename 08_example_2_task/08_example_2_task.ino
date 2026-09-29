// Arduino pin assignment
#define PIN_LED  9
#define PIN_TRIG 12   // sonar sensor TRIGGER
#define PIN_ECHO 13   // sonar sensor ECHO

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300.0   // maximum distance to be measured (unit: mm)
#define DIST_PEAK 200.0   // distance at maximum LED brightness (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000UL) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficient to convert duration to distance

unsigned long last_sampling_time = 0; // unit: msec

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  Serial.begin(57600);
}

void loop() {
  unsigned long now = millis();
  if (now - last_sampling_time < INTERVAL)
    return;

  // Advance by one interval to avoid adding processing time to each period.
  last_sampling_time += INTERVAL;

  float distance = USS_measure(PIN_TRIG, PIN_ECHO);
  int led_pwm;

  // Active-low LED: PWM 0 is brightest and 255 is off.
  // Brightness peaks at 200 mm and decreases linearly toward 100/300 mm.
  if (distance <= _DIST_MIN || distance >= _DIST_MAX) {
    led_pwm = 255;
  } else if (distance <= DIST_PEAK) {
    led_pwm = (int)(255.0 * (DIST_PEAK - distance) / (DIST_PEAK - _DIST_MIN) + 0.5);
  } else {
    led_pwm = (int)(255.0 * (distance - DIST_PEAK) / (_DIST_MAX - DIST_PEAK) + 0.5);
  }

  analogWrite(PIN_LED, led_pwm);

  Serial.print("Min:");      Serial.print(_DIST_MIN);
  Serial.print(",distance:"); Serial.print(distance);
  Serial.print(",Max:");     Serial.print(_DIST_MAX);
  Serial.print(",LED_PWM:"); Serial.println(led_pwm);
}

// Get a distance reading from USS. Return value is in millimeter.
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);

  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE;
}
