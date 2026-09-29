#include <Arduino.h>
#include "geeWhiz.h"

// =====================================================================
// Lab 1 (f) - Inner loop with a reference saturator
// Proportional control of the motor gear angle, with theta_ref passed
// through a +-REF_SAT saturator before the summing junction, as in
// Figure 7(b). The square wave swings wider than the saturator limit so
// the clipping is visible in both directions.
//
// This sketch carries forward into Labs 2 and 3: the saturator and the
// THETA_LIMIT cutoff are required to stay in place there.
//
// Commands (type in CoolTerm, Kp can only change while stopped):
//   g  start the square wave          s  stop immediately (0 V)
//   1 2 3  select a preset Kp         k<value><Enter>  set Kp, e.g. k-3.5
//   d  print ISR timing and reset it
//
// Sample line (every STREAM_EVERY samples):
//   t_ms,ref_raw,ref_sat,theta,V
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

// ================== Plant Model ==================
// Part (e): theta(s)/V(s) = PLANT_K1 / (s (PLANT_TAU s + 1)), from 30 steps
// at Kp = -15, -20 and -25 V/rad. Kept here for reference and for Lab 2.
constexpr float PLANT_K1  = -1.883f;    // rad/(V s), +- 0.118
constexpr float PLANT_TAU =  0.0216f;   // s, i.e. 21.6 +- 1.3 ms

// ================== Test Settings ==================
constexpr uint16_t SAMPLE_MS      = 1;      // control interval [ms]
constexpr uint32_t HALF_PERIOD_MS = 2000;   // square wave half period [ms]
constexpr float    REF_AMP        = 1.0f;   // raw reference swings +-REF_AMP [rad]
constexpr float    REF_SAT        = 0.7f;   // saturator limit, Figure 7(b) [rad]
constexpr float    START_LIMIT    = 0.5f;   // refuse to start if |theta| is above this [rad]
constexpr float    THETA_LIMIT    = 0.8f;   // motor to 0 V beyond this [rad], required from Lab 2 (d) on
constexpr float    V_NOMINAL_MAX  = 6.0f;   // D/A range; commands past this are counted, not clipped [V]
constexpr float    KP_MAX_MAG     = 10.0f;  // largest |Kp| this sketch accepts [V/rad]
constexpr int      STREAM_EVERY   = 10;     // stream decimation [samples], 10 ms at SAMPLE_MS = 1
const float        KP_PRESETS[3]  = { -3.0f, -3.5f, -4.0f };   // keys 1 2 3 [V/rad]

constexpr uint32_t HALF_PERIOD_SAMPLES = HALF_PERIOD_MS / SAMPLE_MS;

// A full saturated step must stay inside the D/A range at the largest preset
static_assert(4.0f * 2.0f * REF_SAT + STICTION_POS <= V_NOMINAL_MAX,
              "largest preset gain exceeds 6 V on a full saturated step");
static_assert(REF_AMP > REF_SAT, "reference must exceed the saturator limit or nothing clips");

// ================== State ==================
enum RunState { STOPPED, RUNNING };

volatile RunState runState     = STOPPED;
volatile bool     limitTripped = false;           // set by the ISR, reported by loop()
volatile float    kp           = KP_PRESETS[1];   // [V/rad]
volatile float    lastTheta    = 0.0f;            // latest angle, so loop() never uses the ADC
volatile uint32_t overVoltage  = 0;               // samples whose command exceeded the D/A range

float    refRaw       = -REF_AMP;   // square wave, saturator input [rad]
float    refSat       = -REF_SAT;   // saturator output, what the loop tracks [rad]
uint32_t halfCount    = 0;          // samples since the last edge
float    motorVoltage = 0.0f;       // command sent to the motor [V]
int      streamCount  = 0;

volatile bool     streamReady = false;
volatile uint32_t streamMs    = 0;
volatile float    streamRaw   = 0.0f;
volatile float    streamSat   = 0.0f;
volatile float    streamTheta = 0.0f;
volatile float    streamV     = 0.0f;

char gainBuf[12];
int  gainLen = -1;   // -1 when not typing a k<value> command

// ISR timing, the software version of the A5 scope check
volatile uint32_t isrSumUs = 0;
volatile uint32_t isrMaxUs = 0;
volatile uint32_t isrCount = 0;
volatile uint32_t gapMinUs = 0xFFFFFFFF;
volatile uint32_t gapMaxUs = 0;
uint32_t lastIsrUs = 0;


// ================== Setup ==================
void setup() {

  analogReadResolution(14);
  pinMode(A5, OUTPUT);   // scope on A5: pulse width = ISR execution time
  Serial.begin(115200);
  delay(300);

  Serial.println("# geeWhiz Started - inner loop with reference saturator");
  Serial.println("# commands: g s 1 2 3 k<value> d");
  Serial.print("# ref +-");             Serial.print(REF_AMP, 2);
  Serial.print(" rad, saturator +-");   Serial.print(REF_SAT, 2);
  Serial.print(" rad, half period ");   Serial.print(HALF_PERIOD_MS);
  Serial.println(" ms");
  Serial.println("# t_ms,ref_raw,ref_sat,theta,V");
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
    Serial.print("# stop LIMIT: |theta| above ");
    Serial.print(THETA_LIMIT, 2);
    Serial.println(" rad, motor set to 0 V");
  }

  printStream();
}

// ================== Stiction Compensation ==================
// Pushes a nonzero command past the dead zone in its direction; zero stays zero
float applyStiction(float v) {
  if (v > 0.0f) return v + STICTION_POS;
  if (v < 0.0f) return v - STICTION_NEG;
  return 0.0f;
}

// ================== Saturator (Figure 7(b)) ==================
// Clips the reference before the summing junction so the gear angle stays
// inside +-pi/4 rad even with the overshoot of a Lab 2 controller
float saturate(float value) {
  if (value >  REF_SAT) return  REF_SAT;
  if (value < -REF_SAT) return -REF_SAT;
  return value;
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
    case 'g': startControl();   break;
    case 's': stopFromUser();   break;
    case 'd': printIsrTiming(); break;
    case '1': case '2': case '3': setGain(KP_PRESETS[c - '1']); break;
    case 'k': gainLen = 0;      break;
    default:                    break;   // ignore line endings and unknown keys
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
  warnIfCommandExceedsRange();
}

// A full saturated step is 2 * REF_SAT of error; say so if that overruns the D/A
void warnIfCommandExceedsRange() {
  float peak = fabsf(kp) * 2.0f * REF_SAT + STICTION_POS;
  if (peak <= V_NOMINAL_MAX) return;
  Serial.print("# note: a full step commands ");
  Serial.print(peak, 2);
  Serial.print(" V, beyond the ");
  Serial.print(V_NOMINAL_MAX, 1);
  Serial.println(" V range; the drive will limit it");
}

void startControl() {
  if (runState == RUNNING) return;
  if (fabsf(lastTheta) > START_LIMIT) {
    Serial.println("# refused: |theta| above START_LIMIT, reposition the gear");
    return;
  }
  noInterrupts();
  refRaw      = -REF_AMP;
  refSat      = saturate(refRaw);
  halfCount   = 0;
  streamCount = 0;
  overVoltage = 0;
  runState    = RUNNING;
  interrupts();

  Serial.print("# start kp=");   Serial.print(kp, 2);
  Serial.print(" ref_amp=");     Serial.print(REF_AMP, 2);
  Serial.print(" ref_sat=");     Serial.print(REF_SAT, 2);
  Serial.print(" dt_ms=");       Serial.print(SAMPLE_MS);
  Serial.print(" stream_ms=");   Serial.println((uint32_t)SAMPLE_MS * STREAM_EVERY);
}

void stopFromUser() {
  noInterrupts();
  stopControl();
  uint32_t over = overVoltage;
  interrupts();

  Serial.print("# stop, samples beyond the D/A range: ");
  Serial.println(over);
}

void printGain() {
  Serial.print("# kp=");
  Serial.println(kp, 2);
}

// Software version of the A5 scope check; printing resets the stats
void printIsrTiming() {
  noInterrupts();
  uint32_t sum = isrSumUs, worst = isrMaxUs, n = isrCount;
  uint32_t gapLo = gapMinUs, gapHi = gapMaxUs;
  isrSumUs = isrMaxUs = isrCount = gapMaxUs = 0;
  gapMinUs = 0xFFFFFFFF;
  interrupts();

  if (n == 0) {
    Serial.println("# isr timing: no samples yet");
    return;
  }
  Serial.print("# isr_us mean=");   Serial.print(sum / n);
  Serial.print(" max=");            Serial.print(worst);
  Serial.print(" of ");             Serial.print((uint32_t)SAMPLE_MS * 1000UL);
  Serial.print("  gap_us min=");    Serial.print(gapLo);
  Serial.print(" max=");            Serial.print(gapHi);
  Serial.print("  n=");             Serial.println(n);
}

// ================== Control Helpers (ISR context) ==================
// Called from the ISR, or from loop() with interrupts off
void stopControl() {
  runState     = STOPPED;
  motorVoltage = 0.0f;
}

// Flips the raw reference every HALF_PERIOD_SAMPLES
void updateReference() {
  halfCount++;
  if (halfCount >= HALF_PERIOD_SAMPLES) {
    halfCount = 0;
    refRaw = -refRaw;
  }
  refSat = saturate(refRaw);   // Figure 7(b): the loop only ever sees refSat
}

// P control on the saturated reference. The command is NOT limited in software:
// the drive's own D/A range is the only limit, as the lab manual requires.
void updateController(float theta) {
  motorVoltage = applyStiction(kp * (refSat - theta));
  if (fabsf(motorVoltage) > V_NOMINAL_MAX) overVoltage++;
}

void updateStream(uint32_t nowMs, float theta) {
  if (++streamCount < STREAM_EVERY) return;
  streamCount = 0;
  streamMs    = nowMs;
  streamRaw   = refRaw;
  streamSat   = refSat;
  streamTheta = theta;
  streamV     = motorVoltage;
  streamReady = true;
}

// Called at the end of every control cycle
void recordIsrTiming(uint32_t startUs) {
  uint32_t duration = micros() - startUs;
  isrSumUs += duration;
  isrCount++;
  if (duration > isrMaxUs) isrMaxUs = duration;

  if (lastIsrUs != 0) {
    uint32_t gap = startUs - lastIsrUs;
    if (gap < gapMinUs) gapMinUs = gap;
    if (gap > gapMaxUs) gapMaxUs = gap;
  }
  lastIsrUs = startUs;
}

// ================== Output (loop context) ==================
void printStream() {
  if (!streamReady) return;

  noInterrupts();
  uint32_t t = streamMs;
  float raw = streamRaw, sat = streamSat, th = streamTheta, v = streamV;
  streamReady = false;
  interrupts();

  Serial.print(t);        Serial.print(",");
  Serial.print(raw, 3);   Serial.print(",");
  Serial.print(sat, 3);   Serial.print(",");
  Serial.print(th, 4);    Serial.print(",");
  Serial.println(v, 3);
}

// ================== Control ISR ==================
void interval_control_code(void) {
  digitalWrite(A5, HIGH);   // scope: pulse width = ISR execution time

  // ---- Read sensor ----
  uint32_t tUs   = micros();
  int      motor = analogRead(MOT_PIN);
  float    theta = THETA_M * motor + THETA_OFFSET;   // gear angle [rad]
  lastTheta = theta;

  // ---- Reference, saturator, controller ----
  if (runState == RUNNING) {
    if (fabsf(theta) > THETA_LIMIT) {
      stopControl();
      limitTripped = true;
    } else {
      updateReference();
      updateController(theta);
    }
  }

  // Streams while stopped too (V = 0), so the gear can be centred by hand
  updateStream(millis(), theta);

  // ---- Apply voltage ----
  setMotorVoltage(motorVoltage);
  recordIsrTiming(tUs);
  digitalWrite(A5, LOW);
}
