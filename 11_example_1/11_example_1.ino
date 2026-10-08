#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9   // LED active-low
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10  // servo motor

// configurable parameters for sonar
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25      // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 180.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficent to convert duration to distance

#define _EMA_ALPHA 0.3    // EMA weight of new sample (range: 0 to 1)
                          // Setting EMA to 1 effectively disables EMA filter.

// duty duration for myservo.writeMicroseconds()
// Calibrate _DUTY_MIN and _DUTY_MAX for your servo, then upload again.
#define _DUTY_MIN 500  // expanded pulse width (0 degree; tune for your servo)
#define _DUTY_MAX 2500 // expanded pulse width (180 degree; tune for your servo)
#define _DUTY_NEU ((_DUTY_MIN + _DUTY_MAX) / 2) // midpoint (90 degree)

// global variables
float  dist_ema, dist_prev = _DIST_MAX; // unit: mm
unsigned long last_sampling_time;       // unit: ms

Servo myservo;

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);    // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);     // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar 

  myservo.attach(PIN_SERVO, _DUTY_MIN, _DUTY_MAX); 
  myservo.writeMicroseconds(_DUTY_NEU);

  // initialize USS related variables
  dist_prev = _DIST_MIN; // raw distance output from USS (unit: mm)
  dist_ema = dist_prev;

  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  float dist_raw, dist_filtered;
  bool range_valid;
  
  // wait until next sampling time.
  // millis() returns the number of milliseconds since the program started. 
  // will overflow after 50 days.
  if (millis() < last_sampling_time + INTERVAL)
    return;

  // get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // Range filter: reject invalid/out-of-range samples and hold the last valid distance.
  range_valid = (dist_raw >= _DIST_MIN && dist_raw <= _DIST_MAX);
  if (range_valid) {
    dist_filtered = dist_raw;
    dist_prev = dist_raw;
    digitalWrite(PIN_LED, LOW);  // LED on (active-low) while measurement is in range
  } else {
    dist_filtered = dist_prev;
    digitalWrite(PIN_LED, HIGH); // LED off (active-low) for invalid/out-of-range reading
  }

  // EMA reduces noise while following changes in distance.
  dist_ema = _EMA_ALPHA * dist_filtered + (1.0 - _EMA_ALPHA) * dist_ema;

  // Map 18-36 cm continuously to 0-180 degrees using calibrated pulse limits.
  float distance_for_servo = dist_ema;
  if (distance_for_servo < _DIST_MIN) distance_for_servo = _DIST_MIN;
  if (distance_for_servo > _DIST_MAX) distance_for_servo = _DIST_MAX;
  float servo_angle = (distance_for_servo - _DIST_MIN) * 180.0 / (_DIST_MAX - _DIST_MIN);
  unsigned int servo_pulse = _DUTY_MIN + (unsigned int)(servo_angle * (_DUTY_MAX - _DUTY_MIN) / 180.0 + 0.5);
  myservo.writeMicroseconds(servo_pulse);

  // Output fields in a format suitable for the Serial Plotter.
  // output the distance to the serial port
  Serial.print("Min:");   Serial.print(_DIST_MIN);
  Serial.print(",dist:"); Serial.print(dist_raw);
  Serial.print(",ema:");  Serial.print(dist_ema);
  Serial.print(",Servo:"); Serial.print(myservo.read());
  Serial.print(",Max:");   Serial.print(_DIST_MAX);
  Serial.println("");
 
  // update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}
