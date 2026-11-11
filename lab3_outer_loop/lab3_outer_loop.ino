#include <Arduino.h>
#include "geeWhiz.h"

// =====================================================================
// Lab 3 (b) - Outer loop: ball position with D2[z] around the Lab 2 inner loop
// D2[z] is the Tustin discretization (Ts = 1 ms) of the given lead controller
// C2(s) = -7 (s + 0.35) / (s + 2.5)   [m in, rad out]
//
//   r_ball -> e2 -> D2[z] -> thRef -> saturator +-0.7 -> thSat -> e1 -> D1[z]
//          -> u -> stiction -> V -> motor -> theta -> beam -> ball y
//
// Both loops run in the same 1 ms ISR. The outer history keeps the
// UNSATURATED thRef and the inner history keeps the LINEAR u, so no
// nonlinear block output is fed back (manual Lab 3 (b) caution).
//
// Kept from Labs 1-2: the +-0.7 rad saturator, the stiction offsets, the 0 V
// cutoff beyond +-0.8 rad (2 consecutive readings), NO software voltage limiter.
//
// Commands (type in CoolTerm):
//   g  start             s  stop immediately (0 V)
//   h<m><Enter>  hold a fixed ball reference (bring-up), e.g. h0.175
//   a  back to the 0.10 <-> 0.25 m square wave (while stopped)
//   v  toggle the live stream          d  print ISR timing and reset it
//
// Stream line, every 10 ms, also while stopped:
//   t_ms,r,th_ref,th_sat,theta,V,y
//   r = ball reference [m], th_ref = D2 output [rad], th_sat = saturator
//   output [rad], theta = gear angle [rad], V = motor command [V], y = ball [m]
// Lines starting with '#' are messages
// =====================================================================

// PINS
int MOT_PIN = A0;   // motor angle sensor
int BAL_PIN = A1;   // ball position sensor

// SENSOR SCALING
// Lab 1 (c): theta [rad] = THETA_M * reading + THETA_OFFSET
const float THETA_M      = -3.645179e-4f;   // rad per ADC count
const float THETA_OFFSET =  3.949147f;      // rad
// Lab 2 (e): ball centre y [m] = BALL_M * reading + BALL_OFFSET, y = 0 at the
// left stopper (away from the motor), as C2 assumes. Range 0.0127 to 0.3693 m.
const float BALL_M      = 2.454620e-05f;    // m per ADC count
const float BALL_OFFSET = -0.004055f;       // m

// STICTION COMPENSATION (Lab 2 (d), beam on)
// +V turns the large gear CW (theta decreases), -V turns it CCW (theta increases)
constexpr float STICTION_POS = 0.134f;   // added when u > 0 [V]
constexpr float STICTION_NEG = 0.723f;   // subtracted when u < 0 [V]

// INNER CONTROLLER D1[Z] (Lab 2 (b)), u = -D1_B1 u1 + D1_A0 e1 + D1_A1 e1_1
constexpr double D1_A0 = -2.454674639351807;   // V/rad
constexpr double D1_A1 =  2.284293610869540;   // V/rad
constexpr double D1_B1 = -0.967822458684839;

// OUTER CONTROLLER D2[Z]
// D2(z) = (D2_A0 z + D2_A1) / (z + D2_B1) = c2d(C2, 0.001, 'tustin'),
// check against Matlab_file/Lab 3/mte484lab3partbsetup.m (format long g).
// thRef = -D2_B1 thRef1 + D2_A0 e2 + D2_A1 e2_1
constexpr double D2_A0 = -6.992484394506866;   // rad/m
constexpr double D2_A1 =  6.990037453183521;   // rad/m
constexpr double D2_B1 = -0.997503121098627;   // controller pole at z = 0.9975 (s = -2.5)

// TEST SETTINGS
constexpr uint16_t SAMPLE_MS      = 1;        // both loops [ms]
constexpr uint32_t HALF_PERIOD_MS = 10000;    // 20 s period, 50 % duty [ms]
constexpr float    Y_LOW          = 0.10f;    // square wave low level [m]
constexpr float    Y_HIGH         = 0.25f;    // square wave high level [m]
constexpr float    HOLD_MIN       = 0.05f;    // hold references allowed [m]
constexpr float    HOLD_MAX       = 0.33f;
constexpr float    REF_SAT        = 0.7f;     // gear-angle saturator [rad]
constexpr float    START_LIMIT    = 0.5f;     // refuse to start if |theta| is above this [rad]
constexpr float    THETA_LIMIT    = 0.8f;     // motor to 0 V beyond this [rad]
constexpr int      LIMIT_SAMPLES  = 2;        // consecutive readings beyond THETA_LIMIT that stop the loop
constexpr float    V_NOMINAL_MAX  = 6.0f;     // D/A range; counted, not clipped [V]
constexpr int      STREAM_EVERY   = 10;       // 10 ms stream at SAMPLE_MS = 1

constexpr uint32_t HALF_PERIOD_SAMPLES = HALF_PERIOD_MS / SAMPLE_MS;

static_assert(Y_LOW < Y_HIGH, "square wave levels out of order");
static_assert(HOLD_MIN > 0.0127f && HOLD_MAX < 0.3693f, "hold range must stay inside the ball travel");

// STATE
enum RunState { STOPPED, RUNNING };

volatile RunState runState     = STOPPED;
volatile bool     limitTripped = false;
volatile float    tripTheta    = 0.0f;
int               limitCount   = 0;        // ISR only
volatile float    lastTheta    = 0.0f;
volatile uint32_t overVoltage  = 0;        // samples beyond the D/A range since start
volatile uint32_t satSamples   = 0;        // samples with the saturator clipping since start
volatile uint32_t runSamples   = 0;

bool   holdMode = false;    // h sets it, a clears it
float  holdRef  = 0.175f;   // [m]
char   holdBuf[16];
int    holdLen  = -1;       // -1 when not typing an h<value> command

volatile float rBall = Y_LOW;   // ball reference [m]
bool     squareRunning = false; // ISR flips rBall only in square-wave runs
uint32_t halfCount = 0;

// Outer loop D2
double thRef  = 0.0;    // D2 output, UNSATURATED [rad]
double thRef1 = 0.0;    // thRef[k-1], unsaturated
double e2_1   = 0.0;    // e2[k-1] [m]
float  thSat  = 0.0f;   // saturator output, what the inner loop tracks [rad]

// Inner loop D1
double ctrlU  = 0.0;    // linear output u[k] [V]
double ctrlU1 = 0.0;    // u[k-1], LINEAR, never the stiction one
double e1_1   = 0.0;    // e1[k-1] [rad]
float  motorVoltage = 0.0f;

bool              streaming   = true;
int               streamCount = 0;
volatile bool     streamReady = false;
volatile uint32_t streamMs    = 0;
volatile float    sR = 0, sThRef = 0, sThSat = 0, sTheta = 0, sV = 0, sY = 0;

volatile uint32_t isrSumUs = 0, isrMaxUs = 0, isrCount = 0;

// SETUP
void setup() {
  analogReadResolution(14);
  pinMode(A5, OUTPUT);   // scope on A5: pulse width = ISR execution time
  Serial.begin(115200);
  delay(300);

  Serial.println("# geeWhiz Started - outer loop D2[z] + inner loop D1[z] (Lab 3 b)");
  Serial.println("# commands: g s h<m> a v d");
  Serial.print("# D2: a0=");  Serial.print(D2_A0, 9);
  Serial.print(" a1=");       Serial.print(D2_A1, 9);
  Serial.print(" b1=");       Serial.println(D2_B1, 9);
  Serial.print("# D1: a0=");  Serial.print(D1_A0, 9);
  Serial.print(" a1=");       Serial.print(D1_A1, 9);
  Serial.print(" b1=");       Serial.println(D1_B1, 9);
  Serial.println("# t_ms,r,th_ref,th_sat,theta,V,y");

  geeWhizBegin();
  setMotorVoltage(0.0f);
  set_control_interval_ms(SAMPLE_MS);
}

// LOOP, commands and printing only; the ISR never prints
void loop() {
  readCommands();
  reportLimitTrip();
  printStream();
}

void reportLimitTrip() {
  if (!limitTripped) return;
  limitTripped = false;
  Serial.print("# stop LIMIT: |theta| above ");  Serial.print(THETA_LIMIT, 2);
  Serial.print(" rad (last ");                   Serial.print(tripTheta, 4);
  Serial.println(" rad), motor set to 0 V");
}

// NONLINEAR BLOCKS
float applyStiction(float v) {
  if (v > 0.0f) return v + STICTION_POS;
  if (v < 0.0f) return v - STICTION_NEG;
  return 0.0f;
}

float saturate(float value) {
  if (value >  REF_SAT) return  REF_SAT;
  if (value < -REF_SAT) return -REF_SAT;
  return value;
}

// COMMANDS
void readCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (holdLen >= 0) { readHoldChar(c); continue; }
    switch (c) {
      case 'g': startControl();   break;
      case 's': stopFromUser();   break;
      case 'h': beginHoldEntry(); break;
      case 'a': leaveHold();      break;
      case 'v': streaming = !streaming;
                Serial.println(streaming ? "# stream on" : "# stream off"); break;
      case 'd': printIsrTiming(); break;
      default:                    break;
    }
  }
}

void startControl() {
  if (runState == RUNNING) return;
  if (fabsf(lastTheta) > START_LIMIT) {
    Serial.println("# refused: |theta| above START_LIMIT, reposition the gear");
    return;
  }
  noInterrupts();
  rBall         = holdMode ? holdRef : Y_LOW;
  squareRunning = !holdMode;
  halfCount     = 0;
  streamCount   = 0;
  overVoltage   = satSamples = runSamples = 0;
  limitCount    = 0;
  resetControllers();
  runState      = RUNNING;
  interrupts();

  Serial.print(holdMode ? "# start hold r=" : "# start square r=");
  Serial.print(holdMode ? holdRef : Y_LOW, 3);
  if (!holdMode) { Serial.print("<->"); Serial.print(Y_HIGH, 3);
                   Serial.print(" half_period_ms="); Serial.print(HALF_PERIOD_MS); }
  Serial.print(" ctrl=D2+D1 dt_ms=");  Serial.print(SAMPLE_MS);
  Serial.print(" sat=");               Serial.print(REF_SAT, 2);
  Serial.print(" stiction_pos=");      Serial.print(STICTION_POS, 3);
  Serial.print(" stiction_neg=");      Serial.println(STICTION_NEG, 3);
}

void stopFromUser() {
  noInterrupts();
  stopControl();
  uint32_t over = overVoltage, sat = satSamples, n = runSamples;
  interrupts();
  Serial.print("# stop, samples=");   Serial.print(n);
  Serial.print(" saturated=");        Serial.print(sat);
  Serial.print(" beyond_DA=");        Serial.println(over);
}

void beginHoldEntry() {
  if (runState == RUNNING) { Serial.println("# refused: stop with s first"); return; }
  holdLen = 0;
}

void readHoldChar(char c) {
  if (c != '\n' && c != '\r') {
    if (holdLen < (int)sizeof(holdBuf) - 1) holdBuf[holdLen++] = c;
    return;
  }
  holdBuf[holdLen] = '\0';
  bool typed = holdLen > 0;
  holdLen = -1;

  char* end = nullptr;
  float value = strtof(holdBuf, &end);
  if (!typed || end == holdBuf || *end != '\0' || !(value >= HOLD_MIN && value <= HOLD_MAX)) {
    Serial.print("# refused: type h then a ball position from ");
    Serial.print(HOLD_MIN, 2); Serial.print(" to "); Serial.print(HOLD_MAX, 2);
    Serial.println(" m and Enter, e.g. h0.175");
    return;
  }
  holdRef  = value;
  holdMode = true;
  Serial.print("# hold set: r=");  Serial.print(holdRef, 3);
  Serial.println(" m. g holds it, s stops, a returns to the square wave");
}

void leaveHold() {
  if (runState == RUNNING) { Serial.println("# refused: stop with s first"); return; }
  holdMode = false;
  Serial.println("# square wave 0.10 <-> 0.25 m, 20 s period");
}

void printIsrTiming() {
  noInterrupts();
  uint32_t sum = isrSumUs, worst = isrMaxUs, n = isrCount;
  isrSumUs = isrMaxUs = isrCount = 0;
  interrupts();
  if (n == 0) { Serial.println("# isr timing: no samples yet"); return; }
  Serial.print("# isr_us mean=");  Serial.print(sum / n);
  Serial.print(" max=");           Serial.print(worst);
  Serial.print(" of ");            Serial.print((uint32_t)SAMPLE_MS * 1000UL);
  Serial.print(" n=");             Serial.println(n);
}

// CONTROL HELPERS (ISR context, or loop() with interrupts off)
void resetControllers() {
  thRef = thRef1 = e2_1 = 0.0;
  thSat = 0.0f;
  ctrlU = ctrlU1 = e1_1 = 0.0;
}

void stopControl() {
  runState     = STOPPED;
  motorVoltage = 0.0f;
  resetControllers();
}

// Flips the ball reference every HALF_PERIOD_SAMPLES in square-wave runs
void updateBallReference() {
  if (!squareRunning) return;
  if (++halfCount < HALF_PERIOD_SAMPLES) return;
  halfCount = 0;
  rBall = (rBall == Y_LOW) ? Y_HIGH : Y_LOW;
}

// Outer loop: D2 on the ball error, then the saturator. thRef1 stores the
// UNSATURATED output; only the inner loop sees thSat.
void updateOuter(float y) {
  double e2 = (double)rBall - (double)y;
  thRef  = -D2_B1 * thRef1 + D2_A0 * e2 + D2_A1 * e2_1;
  thRef1 = thRef;
  e2_1   = e2;
  thSat  = saturate((float)thRef);
  if (thSat != (float)thRef) satSamples++;
}

// Inner loop: D1 on the gear error, stiction added after, no voltage limiter
void updateInner(float theta) {
  double e1 = (double)thSat - (double)theta;
  ctrlU  = -D1_B1 * ctrlU1 + D1_A0 * e1 + D1_A1 * e1_1;
  ctrlU1 = ctrlU;
  e1_1   = e1;
  motorVoltage = applyStiction((float)ctrlU);
  if (fabsf(motorVoltage) > V_NOMINAL_MAX) overVoltage++;
}

void updateStream(float theta, float y) {
  if (++streamCount < STREAM_EVERY) return;
  streamCount = 0;
  streamMs = millis();
  sR = rBall;  sThRef = (float)thRef;  sThSat = thSat;
  sTheta = theta;  sV = motorVoltage;  sY = y;
  streamReady = true;
}

// OUTPUT (loop context)
void printStream() {
  if (!streaming || !streamReady) return;
  noInterrupts();
  uint32_t t = streamMs;
  float r = sR, tr = sThRef, ts = sThSat, th = sTheta, v = sV, y = sY;
  streamReady = false;
  interrupts();

  Serial.print(t);       Serial.print(",");
  Serial.print(r, 3);    Serial.print(",");
  Serial.print(tr, 4);   Serial.print(",");
  Serial.print(ts, 4);   Serial.print(",");
  Serial.print(th, 4);   Serial.print(",");
  Serial.print(v, 3);    Serial.print(",");
  Serial.println(y, 4);
}

// CONTROL ISR, RUNS EVERY 1 MS
void interval_control_code(void) {
  digitalWrite(A5, HIGH);
  uint32_t tUs = micros();

  // Both sensors first: the ball position is now a control input
  float theta = THETA_M * analogRead(MOT_PIN) + THETA_OFFSET;   // gear angle [rad]
  float y     = BALL_M  * analogRead(BAL_PIN) + BALL_OFFSET;    // ball centre [m]
  lastTheta = theta;

  if (runState == RUNNING) {
    bool over = fabsf(theta) > THETA_LIMIT;
    limitCount = over ? limitCount + 1 : 0;
    if (limitCount >= LIMIT_SAMPLES) {
      tripTheta = theta;
      stopControl();
      limitTripped = true;
    } else {
      runSamples++;
      updateBallReference();
      updateOuter(y);
      // A single reading beyond the limit is not trusted: 0 V for this
      // sample, inner history held, the next reading decides
      if (over) motorVoltage = 0.0f;
      else      updateInner(theta);
    }
  }

  setMotorVoltage(motorVoltage);
  updateStream(theta, y);

  uint32_t d = micros() - tUs;
  isrSumUs += d;  isrCount++;
  if (d > isrMaxUs) isrMaxUs = d;
  digitalWrite(A5, LOW);
}
