#include <Arduino.h>
#include "geeWhiz.h"

// =====================================================================
// Lab 1 (d) - Motor stiction test
// Slowly ramps the motor voltage in one direction and cuts it to 0 V
// as soon as the gear starts to move.
//
// Commands (type in CoolTerm):
//   +  ramp positive voltage          -  ramp negative voltage
//   s  stop immediately (0 V)
//   p  +V_TEST with stiction comp     n  -V_TEST with stiction comp
//   P  +V_TEST without comp           N  -V_TEST without comp
//
// Sample line:  t_ms,run,state,V,motor,theta
// Lines starting with '#' are messages and run summaries
// =====================================================================

// ================== Pins ==================
int MOT_PIN = A0;   // motor angle sensor

// ================== Sensor Scaling ==================
// From part (c): theta [rad] = THETA_M * reading [counts] + THETA_OFFSET
const float THETA_M      = -3.645179e-4f;   // rad per ADC count
const float THETA_OFFSET =  3.949147f;      // rad

// ================== Test Settings ==================
const uint16_t SAMPLE_MS        = 20;       // control interval [ms]
const int      BASELINE_SAMPLES = 10;       // samples held at 0 V before the ramp (0.2 s)
const float    RAMP_STEP        = 0.001f;   // voltage increase per sample [V] (0.05 V/s, 20 s per volt)
const float    V_MAX            = 4.0f;     // stop the ramp if |V| reaches this [V]
const int      AVG_SAMPLES      = 5;        // moving average length for motion detection
const float    MOTION_THRESHOLD = 0.03f;    // averaged theta change that counts as motion [rad]
const float    THETA_LIMIT      = 1.2f;     // safety stop if |theta| exceeds this [rad]
const float    V_TEST           = 0.1f;     // small command for the dead zone check [V]
const int      PULSE_SAMPLES    = 50;       // dead zone check duration (1 s)

// ================== Stiction Compensation ==================
// Magnitudes added to the command in each direction
// Task 5: mean + 2 sd of breakaway voltage over 54 ramp runs (theta = -0.5, 0, +0.5 rad)
const float STICTION_POS = 0.277f;   // added when V > 0 (CW)  [V]
const float STICTION_NEG = 0.259f;   // subtracted when V < 0 (CCW) [V]

// ================== Test State ==================
enum TestState { IDLE = 0, BASELINE = 1, RAMP = 2, PULSE = 3 };

volatile char pendingCommand = 0;   // set in loop(), used by the control ISR

TestState state        = IDLE;
int   runNumber        = 0;
int   direction        = 0;      // +1 or -1
int   sampleCount      = 0;
float rampVoltage      = 0.0f;   // ramp magnitude [V]
float motorVoltage     = 0.0f;   // voltage sent to the motor [V]
float thetaStart       = 0.0f;   // average theta before the ramp [rad]
float thetaSum         = 0.0f;
float avgBuffer[AVG_SAMPLES];
int   avgIndex         = 0;
int   avgCount         = 0;


// ================== Setup ==================
void setup() {

  analogReadResolution(14);
  Serial.begin(115200);
  delay(300);

  // Print the header before the timer starts so lines don't get mixed
  Serial.println("# geeWhiz Started - stiction test");
  Serial.println("# commands: + - s p n P N");
  Serial.println("# t_ms,run,state,V,motor,theta");

  geeWhizBegin();
  setMotorVoltage(0.0f);
  set_control_interval_ms(SAMPLE_MS);
}

// ================== Loop ==================
// Only listens for commands; all motor and sensor work happens in the ISR
void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '+' || c == '-' || c == 's' || c == 'p' || c == 'n' || c == 'P' || c == 'N') {
      pendingCommand = c;
    }
  }
}

// ================== Stiction Compensation ==================
// Pushes a nonzero command past the dead zone in its direction
float applyStiction(float v) {
  if (v > 0.0f) return v + STICTION_POS;
  if (v < 0.0f) return v - STICTION_NEG;
  return 0.0f;
}

// ================== Run Control ==================
void startRun(char cmd, float theta) {
  if (fabsf(theta) > THETA_LIMIT) {
    Serial.println("# refused: |theta| above THETA_LIMIT, reposition the gear");
    return;
  }
  runNumber++;
  direction    = (cmd == '+') ? 1 : -1;
  sampleCount  = 0;
  thetaSum     = 0.0f;
  rampVoltage  = 0.0f;
  motorVoltage = 0.0f;
  avgIndex     = 0;
  avgCount     = 0;
  state        = BASELINE;
}

void startPulse(char cmd, float theta) {
  float v = (cmd == 'p' || cmd == 'P') ? V_TEST : -V_TEST;
  bool compensate = (cmd == 'p' || cmd == 'n');

  runNumber++;
  motorVoltage = compensate ? applyStiction(v) : v;
  thetaStart   = theta;
  sampleCount  = 0;
  state        = PULSE;

  Serial.print("# run ");  Serial.print(runNumber);
  Serial.print(" pulse "); Serial.print(cmd);
  Serial.print(" V=");     Serial.println(motorVoltage, 3);
}

void stopRun(const char* reason, float theta) {
  if (state == IDLE) {
    motorVoltage = 0.0f;
    return;
  }
  Serial.print("# run ");        Serial.print(runNumber);
  Serial.print(" stop ");        Serial.print(reason);
  Serial.print(" V=");           Serial.print(motorVoltage, 3);
  Serial.print(" theta_start="); Serial.print(thetaStart, 4);
  Serial.print(" theta_end=");   Serial.println(theta, 4);

  motorVoltage = 0.0f;
  state        = IDLE;
}

void handleCommand(char cmd, float theta) {
  if (cmd == 0) return;
  if (cmd == 's') {
    stopRun("STOP", theta);
    return;
  }
  if (state != IDLE) return;   // ignore new runs until the current one ends
  if (cmd == '+' || cmd == '-') startRun(cmd, theta);
  else                          startPulse(cmd, theta);
}

// ================== State Updates ==================
// Hold 0 V and record the starting angle
void updateBaseline(float theta) {
  thetaSum += theta;
  sampleCount++;
  if (sampleCount >= BASELINE_SAMPLES) {
    thetaStart  = thetaSum / BASELINE_SAMPLES;
    sampleCount = 0;
    state       = RAMP;
  }
}

// Raise the voltage one step and check whether the gear has moved
void updateRamp(float theta) {
  rampVoltage += RAMP_STEP;
  motorVoltage = direction * rampVoltage;

  // Moving average of the last AVG_SAMPLES angles to ignore sensor noise
  avgBuffer[avgIndex] = theta;
  avgIndex = (avgIndex + 1) % AVG_SAMPLES;
  if (avgCount < AVG_SAMPLES) avgCount++;
  float sum = 0.0f;
  for (int i = 0; i < avgCount; i++) sum += avgBuffer[i];
  float thetaAvg = sum / avgCount;

  if (fabsf(theta) > THETA_LIMIT)                          stopRun("LIMIT", theta);
  else if (fabsf(thetaAvg - thetaStart) > MOTION_THRESHOLD) stopRun("MOTION", theta);
  else if (rampVoltage >= V_MAX)                            stopRun("VMAX", theta);
}

// Hold the small test voltage for PULSE_SAMPLES samples
void updatePulse(float theta) {
  sampleCount++;
  if (fabsf(theta) > THETA_LIMIT)       stopRun("LIMIT", theta);
  else if (sampleCount >= PULSE_SAMPLES) stopRun("PULSE_END", theta);
}

// ================== Logging ==================
void printSample(int motor, float theta) {
  Serial.print(millis());        Serial.print(",");
  Serial.print(runNumber);       Serial.print(",");
  Serial.print((int)state);      Serial.print(",");
  Serial.print(motorVoltage, 3); Serial.print(",");
  Serial.print(motor);           Serial.print(",");
  Serial.println(theta, 4);
}

// ================== Control ISR ==================
void interval_control_code(void) {
  // ---- Read sensor ----
  int   motor = analogRead(MOT_PIN);
  float theta = THETA_M * motor + THETA_OFFSET;   // gear angle [rad]

  // ---- Take the latest command from loop() ----
  char cmd = pendingCommand;
  pendingCommand = 0;
  handleCommand(cmd, theta);

  // ---- Update the test ----
  switch (state) {
    case BASELINE: updateBaseline(theta); break;
    case RAMP:     updateRamp(theta);     break;
    case PULSE:    updatePulse(theta);    break;
    default:       motorVoltage = 0.0f;   break;
  }

  // ---- Apply voltage and log ----
  setMotorVoltage(motorVoltage);
  printSample(motor, theta);
}
