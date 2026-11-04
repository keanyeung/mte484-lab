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
//   h<rad><Enter>  hold mode (Lab 2 f): g then holds that gear angle (within
//          +-0.7, saturator applies) instead of the square wave; a leaves it.
//          Start line: # start hold ref=<rad> ctrl=D1 ... stiction_pos= stiction_neg=
//   x / X  arm the 0.8 rad cutoff test toward +0.95 / -0.95 rad, BEAM OFF ONLY:
//          the next g holds that reference with the saturator bypassed until
//          the cutoff stops the loop; any stop or trip disarms it
//
// Every LIMIT trip also prints a 1 ms trip log once the gear has coasted:
//   # trip log ...   then   trip,i,theta,V   (i = 0 is the sample that stopped
//   the loop)   then   # end trip log
// Live stream line (every STREAM_EVERY samples, paused during a capture):
//   t_ms,ref_raw,ref_sat,theta,V,y_m        (y_m = ball centre, Lab 2 (e))
// Capture block, one per step, 1 ms samples:
//   # capture <id> ctrl=D1 dir=<+|-> dt_us=<interval> samples=<N> over6=<count>
//   i,t_us,ref,V,u,motor,theta        (i = 0 is the reference edge)
//   # end capture <id>
// V includes the stiction offset; u is the linear controller output.
// Lines starting with '#' are messages
// =====================================================================

// ================== Pins ==================
int MOT_PIN = A0;   // motor angle sensor
int BAL_PIN = A1;   // ball position sensor

// ================== Sensor Scaling ==================
// From Lab 1 (c): theta [rad] = THETA_M * reading [counts] + THETA_OFFSET
const float THETA_M      = -3.645179e-4f;   // rad per ADC count
const float THETA_OFFSET =  3.949147f;      // rad

// From Lab 2 (e): ball centre y [m] = BALL_M * reading [counts] + BALL_OFFSET,
// y = 0 at the left stopper's inner face. Least-squares fit over 16 points at
// 8 positions (analyze_ball_scaling.py): R^2 = 0.99995, worst residual 1.4 mm.
// The ball centre can reach 0.0127 m (left stopper) to 0.3693 m (right stopper).
const float BALL_M      = 2.454620e-05f;    // m per ADC count
const float BALL_OFFSET = -0.004055f;       // m

// ================== Stiction Compensation ==================
// Lab 2 (d), beam attached, ball off: mean + 2 sd of breakaway voltage over
// 30 ramp runs at theta = -0.5, 0, +0.5 rad (lab2_stiction_test, analyze_stiction_beam.py).
// The beam's weight helps CW and resists CCW, so the two moved apart
// (Lab 1 (d), beam off: STICTION_POS 0.277 V, STICTION_NEG 0.259 V).
// +V turns the large gear CW (theta decreases), -V turns it CCW (theta increases)
constexpr float STICTION_POS = 0.134f;   // added when u > 0 (CW)  [V]
constexpr float STICTION_NEG = 0.723f;   // subtracted when u < 0 (CCW) [V]

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

constexpr float    CUTOFF_TEST_REF = 0.95f;  // reference the x / X test holds, past THETA_LIMIT [rad]
constexpr int      TRIP_LOG_N      = 64;     // samples kept around a LIMIT trip, 1 ms each
constexpr int      TRIP_POST       = 32;     // of those, samples recorded after the trip (coasting)

constexpr uint32_t CAPTURE_MS     = 800;    // recording window after each edge [ms]
constexpr int      CAPTURE_EDGES  = 2;      // steps per capture (one up, one down)
constexpr int      BATCH_CAPTURES = 5;      // captures in one 'C' batch (10 steps)

constexpr uint32_t HALF_PERIOD_SAMPLES = HALF_PERIOD_MS / SAMPLE_MS;
constexpr int      CAPTURE_SAMPLES     = CAPTURE_MS / SAMPLE_MS;

static_assert(REF_AMP_FULL > REF_SAT, "full reference must exceed the saturator limit or nothing clips");
static_assert(REF_AMP_BRINGUP < REF_SAT, "bring-up reference must stay inside the saturator");
static_assert(CAPTURE_MS < HALF_PERIOD_MS, "capture window must end before the next edge");
static_assert(CUTOFF_TEST_REF > THETA_LIMIT, "the cutoff test must aim past the limit it tests");
static_assert(CUTOFF_TEST_REF < 1.2f, "the cutoff test must stay inside the beam-off travel used in Lab 1");
static_assert(TRIP_POST < TRIP_LOG_N, "the trip log must keep samples from before the trip");

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

// Cutoff test (x / X): armed while stopped, running until the next stop or trip
volatile int8_t       cutoffArmed   = 0;      // +1 / -1 armed toward +-CUTOFF_TEST_REF, 0 off
volatile bool         cutoffRunning = false;

// Hold mode (h): a fixed gear-angle reference instead of the square wave, Lab 2 (f)
bool                  holdMode      = false;  // set by h, cleared by a
float                 holdRef       = 0.0f;   // [rad], within +-REF_SAT
volatile bool         holdRunning   = false;  // the current run is a hold
char                  holdBuf[16];
int                   holdLen       = -1;     // -1 when not typing an h<value> command

// Trip log: ring buffer of the last TRIP_LOG_N samples, frozen TRIP_POST samples after a trip
float         tripLogTheta[TRIP_LOG_N];       // [rad]
float         tripLogV[TRIP_LOG_N];           // command sent that sample [V]
int           tripLogHead  = 0;               // next slot to write
int           tripLogCount = 0;               // filled slots
volatile int  tripPostLeft = 0;               // samples still to record after a trip
volatile bool tripLogReady = false;           // set by the ISR, printed by loop()

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
volatile float    streamBallY = 0.0f;   // ball centre [m]

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
  Serial.println("# commands: g s a c C v d x X h<rad>");
  Serial.print("# D1: a0=");   Serial.print(D1_A0, 9);
  Serial.print(" a1=");        Serial.print(D1_A1, 9);
  Serial.print(" b1=");        Serial.print(D1_B1, 9);
  Serial.print(" Ts_ms=");     Serial.println(SAMPLE_MS);
  Serial.println("# t_ms,ref_raw,ref_sat,theta,V,y_m");
  printAmplitude();

  geeWhizBegin();
  setMotorVoltage(0.0f);
  set_control_interval_ms(SAMPLE_MS);
}

// ================== Loop ==================
// Handles commands and all printing; the ISR never prints
void loop() {
  printTripLog();   // before commands, so a quick restart cannot overwrite an unprinted log
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
    char c = Serial.read();
    if (holdLen >= 0) {   // typing the angle after h
      readHoldChar(c);
      continue;
    }
    switch (c) {
      case 'h': beginHoldEntry();  break;
      case 'g': startControl();    break;
      case 's': stopFromUser();    break;
      case 'a': toggleAmplitude(); break;
      case 'c': armCapture();      break;
      case 'C': armBatch();        break;
      case 'v': toggleStream();    break;
      case 'd': printIsrTiming();  break;
      case 'x': armCutoffTest(+1); break;
      case 'X': armCutoffTest(-1); break;
      default:                     break;   // ignore line endings and unknown keys
    }
  }
}

void startControl() {
  if (runState == RUNNING) return;
  if (fabsf(lastTheta) > START_LIMIT) {
    Serial.println("# refused: |theta| above START_LIMIT, reposition the gear");
    if (cutoffArmed != 0) {
      cutoffArmed = 0;   // never leave the saturator bypass waiting for a later g
      Serial.println("# cutoff test disarmed, press x or X again once repositioned");
    }
    return;
  }
  noInterrupts();
  int8_t test = cutoffArmed;
  if (test != 0) {
    refRaw = refSat = test * CUTOFF_TEST_REF;   // saturator bypassed for this run only
  } else if (holdMode) {
    refRaw = holdRef;
    refSat = saturate(refRaw);                  // the saturator still applies to a hold
  } else {
    refRaw = -refAmp;
    refSat = saturate(refRaw);
  }
  cutoffRunning = (test != 0);
  holdRunning   = (test == 0) && holdMode;
  halfCount    = 0;
  streamCount  = 0;
  overVoltage  = 0;
  limitCount   = 0;
  tripLogHead  = 0;
  tripLogCount = 0;
  tripPostLeft = 0;
  tripLogReady = false;
  resetController();
  runState     = RUNNING;
  interrupts();

  if (test != 0) {
    Serial.print("# start cutoff_test ref=");  Serial.print(test * CUTOFF_TEST_REF, 3);
    Serial.print(" limit=");                   Serial.print(THETA_LIMIT, 2);
    Serial.print(" limit_samples=");           Serial.println(LIMIT_SAMPLES);
    return;
  }
  if (holdRunning) {
    Serial.print("# start hold ref=");         Serial.print(refSat, 3);
    Serial.print(" ctrl=D1 dt_ms=");           Serial.print(SAMPLE_MS);
    Serial.print(" stiction_pos=");            Serial.print(STICTION_POS, 3);
    Serial.print(" stiction_neg=");            Serial.println(STICTION_NEG, 3);
    return;
  }
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
  if (holdMode) {          // a leaves hold mode and keeps the current amplitude
    holdMode = false;
    Serial.print("# hold off, back to the square wave. ");
    printAmplitude();
    return;
  }
  refAmp = (refAmp == REF_AMP_FULL) ? REF_AMP_BRINGUP : REF_AMP_FULL;
  printAmplitude();
}

// ================== Hold mode (Lab 2 (f)) ==================
// h<rad><Enter> sets a fixed gear-angle reference; g then holds it instead of
// running the square wave, so the beam stays at one angle while the ball rolls.
void beginHoldEntry() {
  if (runState == RUNNING) {
    Serial.println("# refused: stop with s before setting a hold angle");
    return;
  }
  holdLen = 0;
}

// Collects the characters after h until Enter, then validates them
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
  if (!typed || end == holdBuf || *end != '\0' || !(fabsf(value) <= REF_SAT)) {
    Serial.print("# refused: type h then an angle within +-");
    Serial.print(REF_SAT, 2);
    Serial.println(" rad and Enter, e.g. h0.2 or h-0.1");
    return;
  }
  holdRef  = value;
  holdMode = true;
  Serial.print("# hold set: ref=");
  Serial.print(holdRef, 3);
  Serial.println(" rad. g holds it, s stops, a returns to the square wave");
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
  bool full    = (refAmp == REF_AMP_FULL) && !cutoffRunning && !holdRunning;
  if (running && idle && full) captureState = CAP_ARMED;
  interrupts();

  if (!running)    Serial.println("# refused: start with g first");
  else if (!full)  Serial.println("# refused: captures need the full amplitude (s, a, g), not the cutoff test");
  else if (!idle)  Serial.println("# refused: capture already in progress");
  return running && idle && full;
}

// x / X: the next g drives the gear toward +-CUTOFF_TEST_REF so the cutoff must fire
void armCutoffTest(int8_t sign) {
  if (runState == RUNNING) {
    Serial.println("# refused: stop with s before arming the cutoff test");
    return;
  }
  cutoffArmed = sign;
  Serial.print("# CUTOFF TEST armed toward ");
  Serial.print(sign * CUTOFF_TEST_REF, 2);
  Serial.println(" rad. THE BEAM MUST BE DISCONNECTED. g runs it, s cancels");
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
  runState      = STOPPED;
  motorVoltage  = 0.0f;
  cutoffRunning = false;
  cutoffArmed   = 0;      // the cutoff test is one-shot: every stop disarms it
  holdRunning   = false;  // hold mode itself stays set, so g holds the same angle again
  resetController();
  if (captureState != CAP_READY) captureState = CAP_IDLE;
}

// Flips the raw reference every HALF_PERIOD_SAMPLES; returns true on the flip
bool updateReference() {
  if (cutoffRunning || holdRunning) return false;   // a fixed reference, no edges
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

// Keeps the last TRIP_LOG_N samples while running; after a trip, records
// TRIP_POST more (the gear coasting at 0 V) and then freezes for printing
void logTripSample(float theta) {
  bool running = (runState == RUNNING);
  if (!running && tripPostLeft == 0) return;

  tripLogTheta[tripLogHead] = theta;
  tripLogV[tripLogHead]     = motorVoltage;
  tripLogHead = (tripLogHead + 1) % TRIP_LOG_N;
  if (tripLogCount < TRIP_LOG_N) tripLogCount++;

  if (!running && --tripPostLeft == 0) tripLogReady = true;
}

void updateStream(uint32_t nowMs, float theta, float ballY) {
  if (++streamCount < STREAM_EVERY) return;
  streamCount = 0;
  streamMs    = nowMs;
  streamRaw   = refRaw;
  streamSat   = refSat;
  streamTheta = theta;
  streamV     = motorVoltage;
  streamBallY = ballY;
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
  float raw = streamRaw, sat = streamSat, th = streamTheta, v = streamV, y = streamBallY;
  streamReady = false;
  interrupts();

  Serial.print(t);        Serial.print(",");
  Serial.print(raw, 3);   Serial.print(",");
  Serial.print(sat, 3);   Serial.print(",");
  Serial.print(th, 4);    Serial.print(",");
  Serial.print(v, 3);     Serial.print(",");
  Serial.println(y, 4);
}

// Prints the frozen trip log, oldest sample first; i = 0 is the sample that stopped the loop
void printTripLog() {
  if (!tripLogReady) return;

  float theta[TRIP_LOG_N], volts[TRIP_LOG_N];
  noInterrupts();
  int n = tripLogCount;
  int first = (tripLogHead - n + TRIP_LOG_N) % TRIP_LOG_N;
  for (int k = 0; k < n; k++) {
    theta[k] = tripLogTheta[(first + k) % TRIP_LOG_N];
    volts[k] = tripLogV[(first + k) % TRIP_LOG_N];
  }
  tripLogReady = false;
  interrupts();

  int tripAt = n - 1 - TRIP_POST;
  Serial.print("# trip log: ");     Serial.print(n);
  Serial.print(" samples at 1 ms, limit "); Serial.print(THETA_LIMIT, 2);
  Serial.println(" rad; trip,i,theta,V with i = 0 the sample that stopped the loop");
  for (int k = 0; k < n; k++) {
    Serial.print("trip,");          Serial.print(k - tripAt);
    Serial.print(",");              Serial.print(theta[k], 4);
    Serial.print(",");              Serial.println(volts[k], 3);
  }
  Serial.println("# end trip log");
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
      tripPostLeft = TRIP_POST + 1;   // this sample, then TRIP_POST more while the gear coasts
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

  logTripSample(theta);

  // ---- Apply voltage ----
  setMotorVoltage(motorVoltage);

  // ---- Ball position, read after the motor update so it adds no control delay ----
  float ballY = BALL_M * analogRead(BAL_PIN) + BALL_OFFSET;   // ball centre [m]

  // Streams while stopped too (V = 0), so the gear can be centred by hand
  updateStream(millis(), theta, ballY);
  recordIsrTiming(tUs);
  digitalWrite(A5, LOW);
}
