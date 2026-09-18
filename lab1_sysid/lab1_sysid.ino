#include <Arduino.h>
#include "geeWhiz.h"

// =====================================================================
// Lab 1 (e) - Plant identification with a proportional controller
// Closes a P loop around the motor and drives it with a square-wave
// reference. Each captured step is stored in RAM at the full control
// rate and printed afterwards, so serial speed does not limit sampling.
//
// Commands (type in CoolTerm, Kp can only change while stopped):
//   g  start the square wave          s  stop immediately (0 V)
//   c  capture the next 2 steps (one up, one down)
//   1 2 3  select a preset Kp         k<value><Enter>  set Kp, e.g. k-18.5
//   v  toggle the live stream
//
// Live stream line:  ref,theta,V   (every STREAM_EVERY samples)
// Capture block, one per step:
//   # capture <id> kp=<Kp> dir=<+|-> dt_us=<interval> samples=<N> sat=<count>
//   i,t_us,ref,V,motor,theta        (i = 0 is the reference edge)
//   # end capture <id>
// Lines starting with '#' are messages
// =====================================================================

// ================== Pins ==================
int MOT_PIN = A0;   // motor angle sensor

// ================== Sensor Scaling ==================
// From part (c): theta [rad] = THETA_M * reading [counts] + THETA_OFFSET
const float THETA_M      = -3.645179e-4f;   // rad per ADC count
const float THETA_OFFSET =  3.949147f;      // rad

// ================== Stiction Compensation ==================
// Part (d): mean + 2 sd of breakaway voltage over 54 ramp runs
// +V turns the large gear CW (theta decreases), -V turns it CCW (theta increases)
constexpr float STICTION_POS = 0.277f;   // added when V > 0 (CW)  [V]
constexpr float STICTION_NEG = 0.259f;   // subtracted when V < 0 (CCW) [V]

// ================== Test Settings ==================
constexpr uint16_t SAMPLE_MS      = 1;      // control interval [ms], confirm with the A5 scope check
constexpr uint32_t HALF_PERIOD_MS = 1000;   // square wave half period [ms]
constexpr float    REF_AMP        = 0.1f;   // reference switches between -REF_AMP and +REF_AMP [rad]
constexpr float    V_SAT          = 5.9f;   // command clamp, keeps |V| below 6 V [V]
constexpr float    KP_MAX_MAG     = 28.0f;  // largest allowed |Kp| [V/rad]
constexpr float    START_LIMIT    = 0.3f;   // refuse to start if |theta| is above this [rad]
constexpr float    THETA_LIMIT    = 0.6f;   // safety stop if |theta| exceeds this [rad]
constexpr uint32_t CAPTURE_MS     = 600;    // recording window after each edge [ms]
constexpr int      CAPTURE_EDGES  = 2;      // steps per capture (one up, one down)
constexpr int      STREAM_EVERY   = 20;     // live stream decimation [samples]
const float        KP_PRESETS[3]  = { -15.0f, -20.0f, -25.0f };   // keys 1 2 3 [V/rad]

constexpr uint32_t HALF_PERIOD_SAMPLES = HALF_PERIOD_MS / SAMPLE_MS;
constexpr int      CAPTURE_SAMPLES     = CAPTURE_MS / SAMPLE_MS;

// A full 0.2 rad step at the largest gain must stay inside the clamp
static_assert(KP_MAX_MAG * 2.0f * REF_AMP + STICTION_POS <= V_SAT, "KP_MAX_MAG saturates on a full step");
static_assert(CAPTURE_MS < HALF_PERIOD_MS, "capture window must end before the next edge");

// ================== Capture Buffer ==================
struct Sample {
  uint32_t tUs;     // micros() when the sample was taken
  uint16_t motor;   // raw sensor reading [counts]
  int16_t  vmV;     // command sent to the motor [mV]
};

struct Step {
  int      id;
  int8_t   dir;        // +1 reference went up, -1 reference went down
  float    kp;
  uint16_t satCount;   // samples where the command hit V_SAT
  Sample   samples[CAPTURE_SAMPLES];
};

Step steps[CAPTURE_EDGES];   // 2 x 600 x 8 B = 9.6 kB at 1 ms

// ================== State ==================
enum RunState     { STOPPED, RUNNING };
enum CaptureState { CAP_IDLE, CAP_ARMED, CAP_RECORDING, CAP_BETWEEN, CAP_READY };

volatile RunState     runState     = STOPPED;
volatile CaptureState captureState = CAP_IDLE;
volatile bool         limitTripped = false;           // set by the ISR, reported by loop()
volatile float        kp           = KP_PRESETS[1];   // [V/rad]
volatile float        lastTheta    = 0.0f;            // latest angle, so loop() never uses the ADC

float    ref          = -REF_AMP;   // reference angle [rad]
uint32_t halfCount    = 0;          // samples since the last edge
float    motorVoltage = 0.0f;       // command sent to the motor [V]
int      stepSlot     = 0;          // steps[] entry being recorded
int      sampleIndex  = 0;
int      nextStepId   = 1;

bool           streaming    = true;
int            streamCount  = 0;
volatile bool  streamReady  = false;
volatile float streamRef    = 0.0f;
volatile float streamTheta  = 0.0f;
volatile float streamV      = 0.0f;

char gainBuf[12];
int  gainLen = -1;   // -1 when not typing a k<value> command


// ================== Setup ==================
void setup() {

  analogReadResolution(14);
  pinMode(A5, OUTPUT);   // scope on A5: pulse width = ISR execution time
  Serial.begin(115200);
  delay(300);

  // Print the header before the timer starts
  Serial.println("# geeWhiz Started - system identification");
  Serial.println("# commands: g s c 1 2 3 k<value> v");
  Serial.print("# presets Kp: ");
  Serial.print(KP_PRESETS[0], 1); Serial.print(" ");
  Serial.print(KP_PRESETS[1], 1); Serial.print(" ");
  Serial.println(KP_PRESETS[2], 1);
  printGain();

  geeWhizBegin();
  setMotorVoltage(0.0f);
  set_control_interval_ms(SAMPLE_MS);
}

// ================== Loop ==================
// Handles commands and all printing; the ISR never prints
void loop() {
  readCommands();

  if (limitTripped) {
    limitTripped = false;
    Serial.println("# stop LIMIT: |theta| above THETA_LIMIT, capture discarded");
  }

  if (captureState == CAP_READY) dumpCapture();
  else                           printStream();
}

// ================== Stiction Compensation ==================
// Pushes a nonzero command past the dead zone in its direction; zero stays zero
float applyStiction(float v) {
  if (v > 0.0f) return v + STICTION_POS;
  if (v < 0.0f) return v - STICTION_NEG;
  return 0.0f;
}

// ================== Commands ==================
void readCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (gainLen >= 0) readGainChar(c);
    else              handleCommand(c);
  }
}

void handleCommand(char c) {
  switch (c) {
    case 'g': startControl();  break;
    case 's': stopFromUser();  break;
    case 'c': armCapture();    break;
    case 'v': toggleStream();  break;
    case '1': case '2': case '3': setGain(KP_PRESETS[c - '1']); break;
    case 'k': gainLen = 0;     break;
    default:                   break;   // ignore line endings and unknown keys
  }
}

// Collects the digits after 'k' until Enter
void readGainChar(char c) {
  if (c == '\n' || c == '\r') {
    gainBuf[gainLen] = '\0';
    gainLen = -1;
    setGain(atof(gainBuf));
    return;
  }
  if (gainLen < (int)sizeof(gainBuf) - 1) gainBuf[gainLen++] = c;
}

void setGain(float value) {
  if (runState == RUNNING) {
    Serial.println("# refused: stop with s before changing Kp");
    return;
  }
  // Also rejects NaN and the 0 that atof returns for bad input
  if (!(value < 0.0f && value >= -KP_MAX_MAG)) {
    Serial.print("# refused: Kp must be between -");
    Serial.print(KP_MAX_MAG, 1);
    Serial.println(" and 0");
    return;
  }
  kp = value;
  printGain();
}

void startControl() {
  if (runState == RUNNING) return;
  if (fabsf(lastTheta) > START_LIMIT) {
    Serial.println("# refused: |theta| above START_LIMIT, reposition the gear");
    return;
  }
  noInterrupts();
  ref         = -REF_AMP;
  halfCount   = 0;
  streamCount = 0;
  runState    = RUNNING;
  interrupts();

  Serial.print("# start kp=");         Serial.print(kp, 2);
  Serial.print(" dt_ms=");             Serial.print(SAMPLE_MS);
  Serial.print(" half_period_ms=");    Serial.print(HALF_PERIOD_MS);
  Serial.print(" ref_amp=");           Serial.print(REF_AMP, 3);
  Serial.print(" stiction_pos=");      Serial.print(STICTION_POS, 3);
  Serial.print(" stiction_neg=");      Serial.println(STICTION_NEG, 3);
}

void stopFromUser() {
  noInterrupts();
  bool cancelled = (captureState == CAP_ARMED || captureState == CAP_RECORDING ||
                    captureState == CAP_BETWEEN);
  stopControl();
  interrupts();
  Serial.println(cancelled ? "# stop (capture cancelled)" : "# stop");
}

void armCapture() {
  noInterrupts();
  bool running = (runState == RUNNING);
  bool idle    = (captureState == CAP_IDLE);
  if (running && idle) captureState = CAP_ARMED;
  interrupts();

  if (!running)   Serial.println("# refused: start with g first");
  else if (!idle) Serial.println("# refused: capture already in progress");
  else            Serial.println("# capture armed, recording the next 2 steps");
}

void toggleStream() {
  streaming = !streaming;
  Serial.println(streaming ? "# stream on" : "# stream off");
}

void printGain() {
  Serial.print("# kp=");
  Serial.println(kp, 2);
}

// ================== Control Helpers (ISR context) ==================
// Called from the ISR, or from loop() with interrupts off
void stopControl() {
  runState     = STOPPED;
  motorVoltage = 0.0f;
  if (captureState != CAP_READY) captureState = CAP_IDLE;
}

// Flips the reference every HALF_PERIOD_SAMPLES; returns true on the flip
bool updateReference() {
  halfCount++;
  if (halfCount < HALF_PERIOD_SAMPLES) return false;
  halfCount = 0;
  ref = -ref;
  return true;
}

// P control with stiction compensation; returns true if the clamp was active
bool updateController(float theta) {
  float v = applyStiction(kp * (ref - theta));
  bool saturated = fabsf(v) > V_SAT;
  motorVoltage = constrain(v, -V_SAT, V_SAT);
  return saturated;
}

// Starts recording a step at a reference edge if a capture is waiting
void startStepAtEdge() {
  if (captureState == CAP_ARMED)        stepSlot = 0;
  else if (captureState == CAP_BETWEEN) stepSlot++;
  else return;

  Step& s = steps[stepSlot];
  s.id       = nextStepId++;
  s.dir      = (ref > 0.0f) ? 1 : -1;
  s.kp       = kp;
  s.satCount = 0;
  sampleIndex  = 0;
  captureState = CAP_RECORDING;
}

void recordSample(uint32_t tUs, int motor, bool saturated) {
  if (captureState != CAP_RECORDING) return;

  Step& s = steps[stepSlot];
  s.samples[sampleIndex] = { tUs, (uint16_t)motor, (int16_t)lroundf(motorVoltage * 1000.0f) };
  if (saturated) s.satCount++;
  sampleIndex++;

  if (sampleIndex < CAPTURE_SAMPLES) return;
  captureState = (stepSlot == CAPTURE_EDGES - 1) ? CAP_READY : CAP_BETWEEN;
}

void updateStream(float theta) {
  if (++streamCount < STREAM_EVERY) return;
  streamCount = 0;
  streamRef   = ref;
  streamTheta = theta;
  streamV     = motorVoltage;
  streamReady = true;
}

// ================== Output (loop context) ==================
void printStream() {
  if (!streaming || !streamReady || captureState != CAP_IDLE) return;

  noInterrupts();
  float r = streamRef, th = streamTheta, v = streamV;
  streamReady = false;
  interrupts();

  Serial.print(r, 3);  Serial.print(",");
  Serial.print(th, 4); Serial.print(",");
  Serial.println(v, 3);
}

void dumpCapture() {
  for (int k = 0; k < CAPTURE_EDGES; k++) printStep(k);
  captureState = CAP_IDLE;
}

// Takes an index, not a Step&, so the IDE's auto-generated prototype
// does not need the Step type
void printStep(int slot) {
  const Step& s = steps[slot];
  Serial.print("# capture "); Serial.print(s.id);
  Serial.print(" kp=");       Serial.print(s.kp, 2);
  Serial.print(" dir=");      Serial.print(s.dir > 0 ? '+' : '-');
  Serial.print(" dt_us=");    Serial.print((uint32_t)SAMPLE_MS * 1000UL);
  Serial.print(" samples=");  Serial.print(CAPTURE_SAMPLES);
  Serial.print(" sat=");      Serial.println(s.satCount);

  float r = s.dir * REF_AMP;
  for (int i = 0; i < CAPTURE_SAMPLES; i++) {
    const Sample& p = s.samples[i];
    Serial.print(i);                               Serial.print(",");
    Serial.print(p.tUs - s.samples[0].tUs);        Serial.print(",");
    Serial.print(r, 3);                            Serial.print(",");
    Serial.print(p.vmV / 1000.0f, 3);              Serial.print(",");
    Serial.print(p.motor);                         Serial.print(",");
    Serial.println(THETA_M * p.motor + THETA_OFFSET, 4);
  }

  Serial.print("# end capture "); Serial.println(s.id);
}

// ================== Control ISR ==================
void interval_control_code(void) {
  digitalWrite(A5, HIGH);   // scope: pulse width = ISR execution time

  // ---- Read sensor ----
  uint32_t tUs   = micros();
  int      motor = analogRead(MOT_PIN);
  float    theta = THETA_M * motor + THETA_OFFSET;   // gear angle [rad]
  lastTheta = theta;

  // ---- Control, capture and stream ----
  if (runState == RUNNING) {
    if (fabsf(theta) > THETA_LIMIT) {
      stopControl();
      limitTripped = true;
    } else {
      bool edge      = updateReference();
      bool saturated = updateController(theta);
      if (edge) startStepAtEdge();
      recordSample(tUs, motor, saturated);
      updateStream(theta);
    }
  }

  // ---- Apply voltage ----
  setMotorVoltage(motorVoltage);
  digitalWrite(A5, LOW);
}
