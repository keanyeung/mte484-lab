#include <Arduino.h>
#include "geeWhiz.h"

// =====================================================================
// Lab 2 (e) - Ball position sensor calibration
// Reads the ball sensor on A1 with the ball held still at known positions
// so analyze_ball_scaling.py can fit  y [m] = BALL_M * reading + BALL_OFFSET.
//
// The motor is held at 0 V on every cycle; this sketch never drives it.
// The gearbox holds the beam where it was turned by hand (shown in Lab 2 (d)).
//
// Commands (type in CoolTerm):
//   y<metres><Enter>  label the next point with the ball centre position,
//                     e.g. y0.1627  (stop face on the ruler + 0.0127 m)
//   m                 measure: 500 readings at 1 ms, then print one # point line
//   v                 toggle the live stream
//
// Live stream line (every STREAM_EVERY samples):  t_ms,ball_raw,theta
// Point line:
//   # point <n> y=<m> raw_median=<counts> raw_mean=<counts> raw_sd=<counts> n=<N> theta=<rad>
// Lines starting with '#' are messages
// =====================================================================

// ================== Pins ==================
int MOT_PIN = A0;   // motor angle sensor
int BAL_PIN = A1;   // ball position sensor

// ================== Sensor Scaling ==================
// From Lab 1 (c): theta [rad] = THETA_M * reading [counts] + THETA_OFFSET
const float THETA_M      = -3.645179e-4f;   // rad per ADC count
const float THETA_OFFSET =  3.949147f;      // rad

// ================== Test Settings ==================
constexpr uint16_t SAMPLE_MS     = 1;       // sampling interval [ms]
constexpr int      POINT_SAMPLES = 500;     // readings per point, 0.5 s at 1 ms
constexpr int      STREAM_EVERY  = 20;      // stream decimation [samples], 20 ms
constexpr float    Y_MIN         = 0.0f;    // accepted label range [m]
constexpr float    Y_MAX         = 0.45f;   // a little past the 0.417 m beam

// ================== State ==================
uint16_t          samples[POINT_SAMPLES];   // A1 readings for the current point
volatile int      sampleIndex = 0;
volatile bool     measuring   = false;      // set by loop(), cleared by the ISR when full
volatile bool     pointReady  = false;      // set by the ISR, printed by loop()
volatile float    thetaSum    = 0.0f;       // to report the beam angle during the point [rad]

float yLabel     = 0.0f;                    // ball centre for the next point [m]
float yMeasured  = 0.0f;                    // label frozen when m starts, printed with the point [m]
bool  hasLabel   = false;                   // each point needs its own y, so none is reused
int   pointCount = 0;

char yBuf[16];
int  yLen = -1;                             // -1 when not typing a y<value> command

bool              streaming   = true;
int               streamCount = 0;
volatile bool     streamReady = false;
volatile uint32_t streamMs    = 0;
volatile uint16_t streamBall  = 0;
volatile float    streamTheta = 0.0f;


// ================== Setup ==================
void setup() {

  analogReadResolution(14);
  Serial.begin(115200);
  delay(300);

  Serial.println("# geeWhiz Started - ball sensor calibration (motor held at 0 V)");
  Serial.println("# commands: y<metres><Enter> m v");
  Serial.println("# t_ms,ball_raw,theta");

  geeWhizBegin();
  setMotorVoltage(0.0f);
  set_control_interval_ms(SAMPLE_MS);
}

// ================== Loop ==================
// Handles commands and all printing; the ISR never prints
void loop() {
  readCommands();
  if (pointReady) printPoint();
  printStream();
}

// ================== Commands ==================
void readCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (yLen >= 0) readLabelChar(c);
    else           handleCommand(c);
  }
}

void handleCommand(char c) {
  switch (c) {
    case 'y': yLen = 0;           break;
    case 'm': startMeasurement(); break;
    case 'v': toggleStream();     break;
    default:                      break;   // ignore line endings and unknown keys
  }
}

// Collects the digits after 'y' until Enter
void readLabelChar(char c) {
  if (c == '\n' || c == '\r') {
    yBuf[yLen] = '\0';
    yLen = -1;
    setLabel(atof(yBuf));
    return;
  }
  if (yLen < (int)sizeof(yBuf) - 1) yBuf[yLen++] = c;
}

// True while a point is being recorded or waiting to print; labels and new
// measurements wait until it is out, so a point can never get the wrong y
bool pointBusy() {
  if (!measuring && !pointReady) return false;
  Serial.println("# refused: wait for the # point line before the next y or m");
  return true;
}

void setLabel(float y) {
  if (pointBusy()) return;
  // Also rejects NaN and the 0 that atof returns for bad input
  if (!(y > Y_MIN && y < Y_MAX)) {
    Serial.print("# refused: y must be between ");
    Serial.print(Y_MIN, 2);
    Serial.print(" and ");
    Serial.print(Y_MAX, 2);
    Serial.println(" m, e.g. y0.1627");
    return;
  }
  yLabel   = y;
  hasLabel = true;
  Serial.print("# label y=");
  Serial.print(yLabel, 4);
  Serial.println(" m, press m to measure");
}

void startMeasurement() {
  if (pointBusy()) return;
  if (!hasLabel) {
    Serial.println("# refused: type y<metres> and Enter first, e.g. y0.1627");
    return;
  }
  yMeasured = yLabel;
  noInterrupts();
  sampleIndex = 0;
  thetaSum    = 0.0f;
  measuring   = true;
  interrupts();
  Serial.print("# measuring y=");
  Serial.print(yMeasured, 4);
  Serial.println(" m, keep the ball still for 0.5 s");
}

void toggleStream() {
  streaming = !streaming;
  Serial.println(streaming ? "# stream on" : "# stream off");
}

// ================== Statistics (loop context) ==================
// Insertion sort, enough for 500 values once per point
void sortSamples(uint16_t* v, int n) {
  for (int i = 1; i < n; i++) {
    uint16_t key = v[i];
    int j = i - 1;
    while (j >= 0 && v[j] > key) {
      v[j + 1] = v[j];
      j--;
    }
    v[j + 1] = key;
  }
}

void printPoint() {
  // The ISR has stopped writing samples[] (measuring is false), so it is safe to read
  static uint16_t sorted[POINT_SAMPLES];
  noInterrupts();
  memcpy(sorted, samples, sizeof(sorted));
  float thetaMean = thetaSum / POINT_SAMPLES;
  pointReady = false;
  interrupts();

  double sum = 0.0;
  for (int i = 0; i < POINT_SAMPLES; i++) sum += sorted[i];
  double mean = sum / POINT_SAMPLES;
  double sq = 0.0;
  for (int i = 0; i < POINT_SAMPLES; i++) sq += (sorted[i] - mean) * (sorted[i] - mean);
  double sd = sqrt(sq / (POINT_SAMPLES - 1));

  sortSamples(sorted, POINT_SAMPLES);
  float median = (sorted[POINT_SAMPLES / 2 - 1] + sorted[POINT_SAMPLES / 2]) / 2.0f;

  pointCount++;
  Serial.print("# point ");       Serial.print(pointCount);
  Serial.print(" y=");            Serial.print(yMeasured, 4);
  Serial.print(" raw_median=");   Serial.print(median, 1);
  Serial.print(" raw_mean=");     Serial.print(mean, 2);
  Serial.print(" raw_sd=");       Serial.print(sd, 2);
  Serial.print(" n=");            Serial.print(POINT_SAMPLES);
  Serial.print(" theta=");        Serial.println(thetaMean, 4);

  hasLabel = false;   // the next point needs its own y
}

// ================== Output (loop context) ==================
void printStream() {
  if (!streaming || !streamReady) return;

  noInterrupts();
  uint32_t t = streamMs;
  uint16_t b = streamBall;
  float th = streamTheta;
  streamReady = false;
  interrupts();

  Serial.print(t);       Serial.print(",");
  Serial.print(b);       Serial.print(",");
  Serial.println(th, 4);
}

// ================== Sampling ISR ==================
void interval_control_code(void) {
  setMotorVoltage(0.0f);   // never drive the motor in this sketch

  int   motor = analogRead(MOT_PIN);
  int   ball  = analogRead(BAL_PIN);
  float theta = THETA_M * motor + THETA_OFFSET;   // gear angle [rad]

  if (measuring) {
    samples[sampleIndex] = (uint16_t)ball;
    thetaSum += theta;
    if (++sampleIndex >= POINT_SAMPLES) {
      measuring  = false;
      pointReady = true;
    }
  }

  if (++streamCount >= STREAM_EVERY) {
    streamCount = 0;
    streamMs    = millis();
    streamBall  = (uint16_t)ball;
    streamTheta = theta;
    streamReady = true;
  }
}
