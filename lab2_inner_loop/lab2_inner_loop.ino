#include <Arduino.h>
#include "geeWhiz.h"

// =====================================================================
// Lab 2 (c) - Inner loop with the discrete controller D1[z]
// D1[z] is the Tustin discretization (Ts = 1 ms) of the part (a) lag
// controller C1(s) = -2.408 (s + 71.906) / (s + 32.704). It runs as a
// difference equation on the saturated reference, as in Figure 7(c):
//
//   theta_ref -> saturator +-0.7 -> e -> D1[z] -> u -> stiction -> V -> motor
//
// Kept from Lab 1, required for the rest of the project: the +-0.7 rad
// reference saturator, the stiction offsets, the 0 V cutoff beyond
// +-0.8 rad (on 2 consecutive readings, so one sensor spike cannot trip it),
// and NO software voltage limiter (only the drive limits V).
//
// Commands (type in CoolTerm):
//   g  start the square wave          s  stop immediately (0 V)
//   a  toggle amplitude (bring-up +-0.1 rad / full +-1.0 rad -> +-0.7), while stopped
//   c  capture the next 2 steps (one up, one down), full amplitude only
//   C  capture a whole data set: BATCH_CAPTURES captures back to back
//   v  toggle the live stream          d  print ISR timing and reset it
//
// Live stream line (every STREAM_EVERY samples, paused during a capture):
//   t_ms,ref_raw,ref_sat,theta,V
// Capture block, one per step, 1 ms samples:
//   # capture <id> ctrl=D1 dir=<+|-> dt_us=<interval> samples=<N> over6=<count>
//   i,t_us,ref,V,u,motor,theta        (i = 0 is the reference edge)
//   # end capture <id>
// V includes the stiction offset; u is the linear controller output.
// Lines starting with '#' are messages
// =====================================================================

// ================== Pins ==================
int MOT_PIN = A0;   // motor angle sensor

// ================== Sensor Scaling ==================
// From Lab 1 (c): theta [rad] = THETA_M * reading [counts] + THETA_OFFSET
const float THETA_M      = -3.645179e-4f;   // rad per ADC count
const float THETA_OFFSET =  3.949147f;      // rad

// ================== Stiction Compensation ==================
// Lab 1 (d): mean + 2 sd of breakaway voltage over 54 ramp runs
// +V turns the large gear CW (theta decreases), -V turns it CCW (theta increases)
constexpr float STICTION_POS = 0.277f;   // added when u > 0 (CW)  [V]
constexpr float STICTION_NEG = 0.259f;   // subtracted when u < 0 (CCW) [V]

// ================== Controller D1[z] ==================
// D1(z) = (D1_A0 z + D1_A1) / (z + D1_B1) = c2d(C1, 0.001, 'tustin'),
// coefficients from Matlab_file/Lab 2/mte484lab2partbsetup.m (format long g).
// Difference equation (manual Appendix 2):  u = -D1_B1 u1 + D1_A0 e + D1_A1 e1
// Double precision keeps every digit the manual asks for.
constexpr double D1_A0 = -2.454674639351807;   // V/rad
constexpr double D1_A1 =  2.284293610869540;   // V/rad
constexpr double D1_B1 = -0.967822458684839;   // controller pole at z = 0.9678

// ================== Test Settings ==================
constexpr uint16_t SAMPLE_MS       = 1;      // control interval, same Ts as part (b) [ms]
constexpr uint32_t HALF_PERIOD_MS  = 1000;   // 2 s period, 50 % duty, as in part (a) [ms]
constexpr float    REF_AMP_FULL    = 1.0f;   // raw square wave, clipped to +-REF_SAT [rad]
constexpr float    REF_AMP_BRINGUP = 0.1f;   // small first test, never clipped [rad]
constexpr float    REF_SAT         = 0.7f;   // saturator limit, Figure 7(c) [rad]
constexpr float    START_LIMIT     = 0.5f;   // refuse to start if |theta| is above this [rad]
constexpr float    THETA_LIMIT     = 0.8f;   // motor to 0 V beyond this [rad], Lab 2 (d) on
constexpr int      LIMIT_SAMPLES   = 2;      // consecutive readings beyond THETA_LIMIT that stop the loop;
                                             // a single-sample sensor spike tripped it at a normal 0.74 rad peak
constexpr float    V_NOMINAL_MAX   = 6.0f;   // D/A range; commands past this are counted, not clipped [V]
constexpr int      STREAM_EVERY    = 10;     // stream decimation [samples], 10 ms at SAMPLE_MS = 1

constexpr uint32_t CAPTURE_MS     = 800;    // recording window after each edge [ms]
constexpr int      CAPTURE_EDGES  = 2;      // steps per capture (one up, one down)
constexpr int      BATCH_CAPTURES = 5;      // captures in one 'C' batch (10 steps)

constexpr uint32_t HALF_PERIOD_SAMPLES = HALF_PERIOD_MS / SAMPLE_MS;
constexpr int      CAPTURE_SAMPLES     = CAPTURE_MS / SAMPLE_MS;

static_assert(REF_AMP_FULL > REF_SAT, "full reference must exceed the saturator limit or nothing clips");
static_assert(REF_AMP_BRINGUP < REF_SAT, "bring-up reference must stay inside the saturator");
static_assert(CAPTURE_MS < HALF_PERIOD_MS, "capture window must end before the next edge");

// ================== Capture Buffer ==================
// Packed: 10 B per sample instead of 12, so 2 steps x 800 samples = 16 kB
struct __attribute__((packed)) Sample {
  uint32_t tUs;     // micros() when the sample was taken
  uint16_t motor;   // raw sensor reading [counts]
  int16_t  vmV;     // command sent to the motor, stiction included [mV]
  int16_t  umV;     // linear controller output u [mV]
};

struct Step {
  int      id;
  int8_t   dir;          // +1 reference went up, -1 reference went down
  uint16_t overCount;    // samples whose command exceeded the D/A range
  Sample   samples[CAPTURE_SAMPLES];
};

Step steps[CAPTURE_EDGES];

static_assert(sizeof(steps) <= 16500, "capture buffer too large for the R4's 32 kB RAM");

// ================== State ==================
enum RunState     { STOPPED, RUNNING };
enum CaptureState { CAP_IDLE, CAP_ARMED, CAP_RECORDING, CAP_BETWEEN, CAP_READY };

volatile RunState     runState     = STOPPED;
volatile CaptureState captureState = CAP_IDLE;
volatile bool         limitTripped = false;   // set by the ISR, reported by loop()
volatile float        tripTheta    = 0.0f;    // the reading that completed the trip [rad]
int                   limitCount   = 0;       // ISR only: consecutive readings beyond THETA_LIMIT
volatile float        lastTheta    = 0.0f;    // latest angle, so loop() never uses the ADC
volatile uint32_t     overVoltage  = 0;       // samples beyond the D/A range since start

float    refAmp       = REF_AMP_BRINGUP;   // raw square wave amplitude [rad]
float    refRaw       = -REF_AMP_BRINGUP;  // saturator input [rad]
float    refSat       = -REF_AMP_BRINGUP;  // saturator output, what the loop tracks [rad]
uint32_t halfCount    = 0;                 // samples since the last edge
double   ctrlU        = 0.0;               // linear controller output u[k] [V]
double   ctrlU1       = 0.0;               // u[k-1], the LINEAR output, never the stiction one [V]
double   ctrlE1       = 0.0;               // e[k-1] [rad]
float    motorVoltage = 0.0f;              // command sent to the motor [V]
int      stepSlot     = 0;                 // steps[] entry being recorded
int      sampleIndex  = 0;
int      nextStepId   = 1;
int      batchLeft    = 0;                 // captures still to record in the current 'C' batch

bool              streaming   = true;
int               streamCount = 0;
volatile bool     streamReady = false;
volatile uint32_t streamMs    = 0;
volatile float    streamRaw   = 0.0f;
volatile float    streamSat   = 0.0f;
volatile float    streamTheta = 0.0f;
volatile float    streamV     = 0.0f;

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

  Serial.println("# geeWhiz Started - inner loop with D1[z] (Lab 2 c)");
  Serial.println("# commands: g s a c C v d");
  Serial.print("# D1: a0=");   Serial.print(D1_A0, 9);
  Serial.print(" a1=");        Serial.print(D1_A1, 9);
  Serial.print(" b1=");        Serial.print(D1_B1, 9);
  Serial.print(" Ts_ms=");     Serial.println(SAMPLE_MS);
  Serial.println("# t_ms,ref_raw,ref_sat,theta,V");
  printAmplitude();

  geeWhizBegin();
  setMotorVoltage(0.0f);
  set_control_interval_ms(SAMPLE_MS);
}

// ================== Loop ==================
// Handles commands and all printing; the ISR never prints
void loop() {
  readCommands();
  reportLimitTrip();

  if (captureState == CAP_READY) {
    dumpCapture();
    reportLimitTrip();   // a trip during the multi-second dump is reported before the batch moves on
    advanceBatch();
  }

  printStream();
}

void reportLimitTrip() {
  if (!limitTripped) return;
  limitTripped = false;
  batchLeft = 0;
  Serial.print("# stop LIMIT: |theta| above ");
  Serial.print(THETA_LIMIT, 2);
  Serial.print(" rad for ");
  Serial.print(LIMIT_SAMPLES);
  Serial.print(" samples (last ");
  Serial.print(tripTheta, 4);
  Serial.println(" rad), motor set to 0 V");
}

// ================== Stiction Compensation ==================
// Pushes a nonzero command past the dead zone in its direction; zero stays zero
float applyStiction(float v) {
  if (v > 0.0f) return v + STICTION_POS;
  if (v < 0.0f) return v - STICTION_NEG;
  return 0.0f;
}

// ================== Saturator (Figure 7(c)) ==================
// Clips the reference before the summing junction so the gear angle stays
// inside +-pi/4 rad even with the controller's overshoot
float saturate(float value) {
  if (value >  REF_SAT) return  REF_SAT;
  if (value < -REF_SAT) return -REF_SAT;
  return value;
}

// ================== Commands ==================
void readCommands() {
  while (Serial.available() > 0) {
    switch (Serial.read()) {
      case 'g': startControl();    break;
      case 's': stopFromUser();    break;
      case 'a': toggleAmplitude(); break;
      case 'c': armCapture();      break;
      case 'C': armBatch();        break;
      case 'v': toggleStream();    break;
      case 'd': printIsrTiming();  break;
      default:                     break;   // ignore line endings and unknown keys
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
  refRaw      = -refAmp;
  refSat      = saturate(refRaw);
  halfCount   = 0;
  streamCount = 0;
  overVoltage = 0;
  limitCount  = 0;
  resetController();
  runState    = RUNNING;
  interrupts();

  Serial.print("# start ctrl=D1 ref_amp=");  Serial.print(refAmp, 2);
  Serial.print(" ref_sat=");                 Serial.print(REF_SAT, 2);
  Serial.print(" dt_ms=");                   Serial.print(SAMPLE_MS);
  Serial.print(" half_period_ms=");          Serial.print(HALF_PERIOD_MS);
  Serial.print(" stiction_pos=");            Serial.print(STICTION_POS, 3);
  Serial.print(" stiction_neg=");            Serial.println(STICTION_NEG, 3);
}

void stopFromUser() {
  noInterrupts();
  bool cancelled = (captureState == CAP_ARMED || captureState == CAP_RECORDING ||
                    captureState == CAP_BETWEEN);
  stopControl();
  uint32_t over = overVoltage;
  interrupts();
  batchLeft = 0;

  Serial.print(cancelled ? "# stop (capture cancelled)" : "# stop");
  Serial.print(", samples beyond the D/A range: ");
  Serial.println(over);
}

void toggleAmplitude() {
  if (runState == RUNNING) {
    Serial.println("# refused: stop with s before changing the amplitude");
    return;
  }
  refAmp = (refAmp == REF_AMP_FULL) ? REF_AMP_BRINGUP : REF_AMP_FULL;
  printAmplitude();
}

void printAmplitude() {
  Serial.print(refAmp == REF_AMP_FULL ? "# amplitude FULL: raw +-" : "# amplitude BRING-UP: +-");
  Serial.print(refAmp, 2);
  Serial.print(" rad, saturated to +-");
  Serial.print(saturate(refAmp), 2);
  Serial.println(" rad");
}

// Arms one capture; returns false (with a reason) if the loop is not ready
bool armNextCapture() {
  noInterrupts();
  bool running = (runState == RUNNING);
  bool idle    = (captureState == CAP_IDLE);
  bool full    = (refAmp == REF_AMP_FULL);
  if (running && idle && full) captureState = CAP_ARMED;
  interrupts();

  if (!running)    Serial.println("# refused: start with g first");
  else if (!full)  Serial.println("# refused: captures need the full amplitude (s, a, g)");
  else if (!idle)  Serial.println("# refused: capture already in progress");
  return running && idle && full;
}

void armCapture() {
  if (armNextCapture()) Serial.println("# capture armed, recording the next 2 steps");
}

void armBatch() {
  if (!armNextCapture()) return;
  batchLeft = BATCH_CAPTURES;
  printBatchProgress();
}

// Called after each dump: arms the next capture of the batch, or reports the total
void advanceBatch() {
  if (batchLeft == 0) return;
  batchLeft--;
  if (batchLeft > 0 && armNextCapture()) printBatchProgress();
  else                                   printBatchDone();
}

void printBatchProgress() {
  Serial.print("# batch capture ");
  Serial.print(BATCH_CAPTURES - batchLeft + 1);
  Serial.print("/");
  Serial.println(BATCH_CAPTURES);
}

void printBatchDone() {
  Serial.print("# batch done: ");
  Serial.print((BATCH_CAPTURES - batchLeft) * CAPTURE_EDGES);
  Serial.println(" steps");
  batchLeft = 0;
  printIsrTiming();
}

void toggleStream() {
  streaming = !streaming;
  Serial.println(streaming ? "# stream on" : "# stream off");
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
// Clears the difference-equation history so a restart never inherits old state
void resetController() {
  ctrlU = ctrlU1 = ctrlE1 = 0.0;
}

// Called from the ISR, or from loop() with interrupts off
void stopControl() {
  runState     = STOPPED;
  motorVoltage = 0.0f;
  resetController();
  if (captureState != CAP_READY) captureState = CAP_IDLE;
}

// Flips the raw reference every HALF_PERIOD_SAMPLES; returns true on the flip
bool updateReference() {
  halfCount++;
  bool edge = (halfCount >= HALF_PERIOD_SAMPLES);
  if (edge) {
    halfCount = 0;
    refRaw = -refRaw;
  }
  refSat = saturate(refRaw);   // Figure 7(c): the loop only ever sees refSat
  return edge;
}

// D1[z] as a difference equation. The stiction offset is added AFTER the
// controller, and the history keeps the linear u, so no nonlinear value is
// fed back. The command is NOT limited in software, as the manual requires.
void updateController(float theta) {
  double e = (double)refSat - (double)theta;
  ctrlU  = -D1_B1 * ctrlU1 + D1_A0 * e + D1_A1 * ctrlE1;
  ctrlU1 = ctrlU;
  ctrlE1 = e;

  motorVoltage = applyStiction((float)ctrlU);
  if (fabsf(motorVoltage) > V_NOMINAL_MAX) overVoltage++;
}

// Starts recording a step at a reference edge if a capture is waiting
void startStepAtEdge() {
  if (captureState == CAP_ARMED)        stepSlot = 0;
  else if (captureState == CAP_BETWEEN) stepSlot++;
  else return;

  Step& s = steps[stepSlot];
  s.id        = nextStepId++;
  s.dir       = (refSat > 0.0f) ? 1 : -1;
  s.overCount = 0;
  sampleIndex  = 0;
  captureState = CAP_RECORDING;
}

void recordSample(uint32_t tUs, int motor) {
  if (captureState != CAP_RECORDING) return;

  Step& s = steps[stepSlot];
  s.samples[sampleIndex] = { tUs, (uint16_t)motor,
                             (int16_t)lroundf(motorVoltage * 1000.0f),
                             (int16_t)lround(ctrlU * 1000.0) };
  if (fabsf(motorVoltage) > V_NOMINAL_MAX) s.overCount++;
  sampleIndex++;

  if (sampleIndex < CAPTURE_SAMPLES) return;
  captureState = (stepSlot == CAPTURE_EDGES - 1) ? CAP_READY : CAP_BETWEEN;
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
  if (!streaming || !streamReady || captureState != CAP_IDLE) return;

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

void dumpCapture() {
  for (int k = 0; k < CAPTURE_EDGES; k++) printStep(k);
  captureState = CAP_IDLE;
}

// Takes an index, not a Step&, so the IDE's auto-generated prototype
// does not need the Step type
void printStep(int slot) {
  const Step& s = steps[slot];
  Serial.print("# capture ");     Serial.print(s.id);
  Serial.print(" ctrl=D1 dir=");  Serial.print(s.dir > 0 ? '+' : '-');
  Serial.print(" dt_us=");        Serial.print((uint32_t)SAMPLE_MS * 1000UL);
  Serial.print(" samples=");      Serial.print(CAPTURE_SAMPLES);
  Serial.print(" over6=");        Serial.println(s.overCount);

  float r = s.dir * REF_SAT;
  uint32_t t0 = s.samples[0].tUs;
  for (int i = 0; i < CAPTURE_SAMPLES; i++) {
    const Sample& p = s.samples[i];
    Serial.print(i);                      Serial.print(",");
    Serial.print(p.tUs - t0);             Serial.print(",");
    Serial.print(r, 3);                   Serial.print(",");
    Serial.print(p.vmV / 1000.0f, 3);     Serial.print(",");
    Serial.print(p.umV / 1000.0f, 3);     Serial.print(",");
    Serial.print(p.motor);                Serial.print(",");
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

  // ---- Reference, saturator, controller, capture ----
  if (runState == RUNNING) {
    bool over = fabsf(theta) > THETA_LIMIT;
    limitCount = over ? limitCount + 1 : 0;
    if (limitCount >= LIMIT_SAMPLES) {
      tripTheta = theta;
      stopControl();
      limitTripped = true;
    } else {
      bool edge = updateReference();
      // A single reading beyond the limit is not trusted: 0 V for this sample,
      // controller history held, and the next reading decides
      if (over) motorVoltage = 0.0f;
      else      updateController(theta);
      if (edge) startStepAtEdge();
      recordSample(tUs, motor);
    }
  }

  // Streams while stopped too (V = 0), so the gear can be centred by hand
  updateStream(millis(), theta);

  // ---- Apply voltage ----
  setMotorVoltage(motorVoltage);
  recordIsrTiming(tUs);
  digitalWrite(A5, LOW);
}
