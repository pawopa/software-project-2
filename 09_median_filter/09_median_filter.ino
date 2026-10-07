// Arduino pin assignment
#define PIN_LED  9
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100     // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300     // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)     // coefficent to convert duration to distance

#define _EMA_ALPHA 0.5    // EMA weight of new sample (range: 0 to 1)

// median filter window size: change to 3, 10, 30 for the experiments
#define N 3

// global variables
unsigned long last_sampling_time;   // unit: msec
float samples[N];                   // circular buffer holding the latest N samples
int sample_idx = 0;                 // next position to overwrite
int sample_cnt = 0;                 // number of valid samples stored (<= N)
float dist_ema = _DIST_MAX;         // EMA distance (initial value)

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  Serial.begin(57600);
}

void loop() {
  float dist_raw, dist_median;

  // wait until next sampling time
  if (millis() < last_sampling_time + INTERVAL)
    return;

  // get a distance reading from the USS (timeout gives 0.0)
  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // store the raw sample as is (no "previous valid value" substitution)
  add_sample(dist_raw);

  // median of the latest samples
  dist_median = get_median();

  // EMA of the raw value (for comparison with the median filter)
  dist_ema = _EMA_ALPHA * dist_raw + (1.0 - _EMA_ALPHA) * dist_ema;

  // output the read value to the serial port
  Serial.print("Min:");     Serial.print(_DIST_MIN);
  Serial.print(",raw:");    Serial.print(dist_raw);
  Serial.print(",ema:");    Serial.print(dist_ema);
  Serial.print(",median:"); Serial.print(dist_median);
  Serial.print(",Max:");    Serial.print(_DIST_MAX);
  Serial.println("");

  // LED on when the median is inside the range
  if ((dist_median < _DIST_MIN) || (dist_median > _DIST_MAX))
    digitalWrite(PIN_LED, 1);       // LED OFF
  else
    digitalWrite(PIN_LED, 0);       // LED ON

  // update last sampling time
  last_sampling_time += INTERVAL;
}

// put a new sample into the circular buffer (oldest one is overwritten)
void add_sample(float value)
{
  samples[sample_idx] = value;
  sample_idx = (sample_idx + 1) % N;
  if (sample_cnt < N)
    sample_cnt++;
}

// return the median of the stored samples
float get_median()
{
  float sorted[N];

  // copy so that the circular buffer keeps its time order
  for (int i = 0; i < sample_cnt; i++)
    sorted[i] = samples[i];

  // insertion sort (N is small)
  for (int i = 1; i < sample_cnt; i++) {
    float key = sorted[i];
    int j = i - 1;
    while (j >= 0 && sorted[j] > key) {
      sorted[j + 1] = sorted[j];
      j--;
    }
    sorted[j + 1] = key;
  }

  if (sample_cnt % 2 == 1)
    return sorted[sample_cnt / 2];
  else
    return (sorted[sample_cnt / 2 - 1] + sorted[sample_cnt / 2]) / 2.0;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);

  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}
