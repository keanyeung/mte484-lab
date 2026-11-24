#include <Arduino.h>
#include <FspTimer.h>
#include "geeWhiz.h"

// =====================================================================
// Lab 3 (e) - FINAL / DEMO CODE. Same controller as Lab 3 (d) (lab3d_outer_loop),
// which already meets the (e) specs. Changes for demo reliability only:
//   1. The control timer interrupt is raised to priority 8. geeWhiz starts it at
//      12, the same as USB serial, so a long USB handler could hold it off (one
//      1.55 s pause seen in Lab 3 (d)). Lower number = higher priority; 8 is the
//      core's own TIMER_PRIORITY. The course library is not modified.
//   2. Anti-windup ON by default (w still toggles it).
//   3. On-board (e) monitor: at every reference edge the board prints whether
//      the half period that just ended met the (e) specs, plus a streak count:
//        # e step <n> dir=<+|-> settle_s=<s> os_pct=<%> PASS|FAIL streak=<steps> periods=<p>
//      PASS = the ball entered the band within 8.5 s and stayed until the edge,
//      and overshoot < 45 %. Bands: 0.094-0.106 m and 0.240-0.260 m (manual (e)).
//
// Lab 3 (d) description, unchanged:
// Part (c) design (pole placement, poles -0.9 +- 0.6j, -2, -4, inner loop as unity):
//   C2(s) = -(72.4513 s^2 + 77.7119 s + 33.9581) / (s (s + 7.8))     [m in, rad out]
// run in SPLIT form, each part Tustin at Ts = 1 ms (the sum is the same D2[z]):
//   C2(s) = -[ KP  +  KI / s  +  R / (s + 7.8) ]
//
//   r_ball -> e2 -> D2[z] -> thRef -> saturator +-0.7 -> thSat -> e1 -> D1[z]
//          -> u -> stiction -> V -> motor -> theta -> beam -> ball y
//
// Anti-windup (command w, ON at reset): conditional integration. The integrator
// holds its value while thRef is beyond the saturator AND integrating would push
// it further out. Only the saturation flag is used; the controller states always
// hold their own linear values, never the saturated thSat (manual Lab 3 (b)).
// With anti-windup OFF this is exactly the groupmate's D2[z] (mte484lab3partcsetup.m).
//
// Kept from Labs 1-3(b): the +-0.7 rad saturator, the stiction offsets, the 0 V
// cutoff beyond +-0.8 rad (2 consecutive readings), NO software voltage limiter.
//
// Commands (type in CoolTerm):
//   g  start             s  stop immediately (0 V)
//   h<m><Enter>  hold a fixed ball reference, e.g. h0.175
//   a  back to the 0.10 <-> 0.25 m square wave (while stopped)
//   w  toggle anti-windup on / off (while stopped)
//   v  toggle the live stream          d  print ISR timing and reset it
//
// Stream line, every 10 ms, also while stopped (same as lab3_outer_loop):
//   t_ms,r,th_ref,th_sat,theta,V,y
// Lines starting with '#' are messages. The '# start' line records aw=on|off.
// =====================================================================

// PINS
int MOT_PIN = A0;   // motor angle sensor
int BAL_PIN = A1;   // ball position sensor

// SENSOR SCALING
// Lab 1 (c): theta [rad] = THETA_M * reading + THETA_OFFSET
const float THETA_M      = -3.645179e-4f;   // rad per ADC count
const float THETA_OFFSET =  3.949147f;      // rad
// Lab 2 (e) slope; y = 0 at the LEFT END OF THE BEAM (away from the motor), as
// the manual defines it. Lab 2 (e) put y = 0 at the left stopper's inner face
// (offset -0.004055). Lab 2 TA feedback: the ball centre can't get below ~3 cm.
// Measured 2026-11-24: ball against the left stopper is centred at 0.030 m from the
// beam end, so the offset rises by 0.030 - 0.0127 = 0.0173 m. Ball centre range
// is now 0.030 to 0.3866 m.
const float BALL_M      = 2.454620e-05f;    // m per ADC count
const float BALL_OFFSET = 0.013245f;        // m (= -0.004055 + 0.0173)
constexpr float Y_MIN_BALL = 0.030f;        // ball centre against the left stopper [m]
constexpr float Y_MAX_BALL = 0.3866f;       // ball centre against the right stopper [m]

// STICTION COMPENSATION (Lab 2 (d), beam on)
// +V turns the large gear CW (theta decreases), -V turns it CCW (theta increases)
constexpr float STICTION_POS = 0.134f;   // added when u > 0 [V]
constexpr float STICTION_NEG = 0.723f;   // subtracted when u < 0 [V]

// INNER CONTROLLER D1[Z] (Lab 2 (b)), u = -D1_B1 u1 + D1_A0 e1 + D1_A1 e1_1
constexpr double D1_A0 = -2.454674639351807;   // V/rad
constexpr double D1_A1 =  2.284293610869540;   // V/rad
constexpr double D1_B1 = -0.967822458684839;

// OUTER CONTROLLER D2[Z], PART (C), SPLIT FORM
// From 3c design review/design.py (split_coefficients), Ts = 1 ms, e2 = r - y [m]:
//   i     = i1 + D2_KI*Ts/2*(e2 + e2_1)      integrator          [rad]
//   x     = -D2_LA*x1 + D2_LB*(e2 + e2_1)    lead R/(s + 7.8)    [rad]
//   thRef = -(D2_KP*e2 + i + x)                                  [rad]
constexpr double D2_KP = 72.4513066994058;     // rad/m, proportional (= a2)
constexpr double D2_KI = 4.35360881518713;     // rad/(m s), integrator (= a0/p)
constexpr double D2_LB = -0.244925731506854;   // rad/m, lead numerator R/(2/Ts + p)
constexpr double D2_LA = -0.992230301822891;   // lead pole at z = 0.99223 (s = -7.8)
constexpr double TS_S  = 0.001;                // sampling period [s]

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

// (E) ASSESSMENT, manual Lab 3 (e)
constexpr float E_LOW_MIN   = 0.094f;   // band around Y_LOW [m]
constexpr float E_LOW_MAX   = 0.106f;
constexpr float E_HIGH_MIN  = 0.240f;   // band around Y_HIGH [m]
constexpr float E_HIGH_MAX  = 0.260f;
constexpr float E_SETTLE_S  = 8.5f;     // enter the band by this time after the edge, then stay [s]
constexpr float E_OS_PCT    = 45.0f;    // overshoot limit [% of the 0.15 m step]
constexpr int   E_PERIODS   = 10;       // consecutive periods assessed in the demo

// CONTROL INTERRUPT PRIORITY (lower number = higher priority)
constexpr uint8_t CONTROL_IRQ_PRIORITY = 8;   // USB serial is 12, millis (AGT) 14

static_assert(E_LOW_MIN < Y_LOW && Y_LOW < E_LOW_MAX, "low band must contain Y_LOW");
static_assert(E_HIGH_MIN < Y_HIGH && Y_HIGH < E_HIGH_MAX, "high band must contain Y_HIGH");

static_assert(Y_LOW < Y_HIGH, "square wave levels out of order");
static_assert(HOLD_MIN > Y_MIN_BALL && HOLD_MAX < Y_MAX_BALL, "hold range must stay inside the ball travel");
static_assert(Y_LOW > Y_MIN_BALL && Y_HIGH < Y_MAX_BALL, "square wave must stay inside the ball travel");
static_assert(SAMPLE_MS == 1, "D2 split coefficients are for Ts = 1 ms");

// STATE
enum RunState { STOPPED, RUNNING };

volatile RunState runState     = STOPPED;
volatile bool     limitTripped = false;
volatile float    tripTheta    = 0.0f;
int               limitCount   = 0;        // ISR only
volatile float    lastTheta    = 0.0f;
volatile uint32_t overVoltage  = 0;        // samples beyond the D/A range since start
volatile uint32_t satSamples   = 0;        // samples with the saturator clipping since start
volatile uint32_t awHeld       = 0;        // samples the anti-windup held the integrator
volatile uint32_t runSamples   = 0;

bool   awOn     = true;     // w toggles it while stopped
bool   holdMode = false;    // h sets it, a clears it
float  holdRef  = 0.175f;   // [m]
char   holdBuf[16];
int    holdLen  = -1;       // -1 when not typing an h<value> command

volatile float rBall = Y_LOW;   // ball reference [m]
bool     squareRunning = false; // ISR flips rBall only in square-wave runs
uint32_t halfCount = 0;

// Outer loop D2 (all linear states, never the saturated value)
double i2    = 0.0;     // integrator state [rad]
double x2    = 0.0;     // lead state [rad]
double e2_1  = 0.0;     // e2[k-1] [m]
double thRef = 0.0;     // D2 output, UNSATURATED [rad]
float  thSat = 0.0f;    // saturator output, what the inner loop tracks [rad]

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
// Longest time between two ISR starts during the current run. 2026-11-24: one run
// lost ~1.55 s of ISR calls (stream gap, late edges); this shows if it happens again.
volatile uint32_t gapMaxUs  = 0;
volatile uint32_t gapAtMs   = 0;      // millis() when that longest gap ended
uint32_t          lastIsrUs = 0;      // ISR only

// (E) MONITOR. The ISR tracks the current half period; at each edge it hands the
// finished step to loop() for printing. The start-up half (from g to the first
// edge) is not a step and is not scored, as in the analysis scripts.
struct EStep {
  int8_t  dir;        // +1 step up to Y_HIGH, -1 step down to Y_LOW
  int32_t enterMs;    // time after the edge the ball last entered the band, -1 = ended outside
  float   peakM;      // largest excursion past the target in the step direction [m]
};
bool              eActive   = false;  // ISR only: a step is being tracked
EStep             eCur;               // ISR only
uint32_t          eSamples  = 0;      // ISR only: samples since the edge
volatile bool     eReady    = false;  // a finished step waits for loop()
EStep             eDone;              // written by the ISR before eReady = true
int               eSteps    = 0;      // loop only: scored steps this run
int               ePasses   = 0;
int               eStreak   = 0;      // consecutive passing steps
int               eBest     = 0;

int8_t controlIrqSlot = -1;           // ICU slot of the control timer, -1 = not found

// SETUP
void setup() {
  analogReadResolution(14);
  pinMode(A5, OUTPUT);   // scope on A5: pulse width = ISR execution time
  Serial.begin(115200);
  delay(300);

  Serial.println("# geeWhiz Started - Lab 3 (e) FINAL: part (c) D2[z] split form + D1[z], (e) monitor");
  Serial.println("# commands: g s h<m> a w v d");
  Serial.print("# D2: kp=");  Serial.print(D2_KP, 9);
  Serial.print(" ki=");       Serial.print(D2_KI, 9);
  Serial.print(" lb=");       Serial.print(D2_LB, 9);
  Serial.print(" la=");       Serial.println(D2_LA, 9);
  Serial.print("# D1: a0=");  Serial.print(D1_A0, 9);
  Serial.print(" a1=");       Serial.print(D1_A1, 9);
  Serial.print(" b1=");       Serial.println(D1_B1, 9);
  Serial.print("# ball: m=");  Serial.print(BALL_M, 9);
  Serial.print(" offset=");    Serial.print(BALL_OFFSET, 6);
  Serial.println(" (y = 0 at the left end of the beam)");
  printAw();

  geeWhizBegin();
  setMotorVoltage(0.0f);
  set_control_interval_ms(SAMPLE_MS);
  raiseControlIrqPriority();
  Serial.println("# t_ms,r,th_ref,th_sat,theta,V,y");
}

// The control timer is a GPT overflow interrupt that geeWhiz registers at
// priority 12. Find its ICU slot by event number and raise it, without
// touching the course library.
void raiseControlIrqPriority() {
  static const uint16_t GPT_OVF[] = {
    ELC_EVENT_GPT0_COUNTER_OVERFLOW, ELC_EVENT_GPT1_COUNTER_OVERFLOW,
    ELC_EVENT_GPT2_COUNTER_OVERFLOW, ELC_EVENT_GPT3_COUNTER_OVERFLOW,
    ELC_EVENT_GPT4_COUNTER_OVERFLOW, ELC_EVENT_GPT5_COUNTER_OVERFLOW,
    ELC_EVENT_GPT6_COUNTER_OVERFLOW, ELC_EVENT_GPT7_COUNTER_OVERFLOW };
  const int slots = sizeof(R_ICU->IELSR) / sizeof(R_ICU->IELSR[0]);
  int found = 0;
  for (int i = 0; i < slots; i++) {
    uint32_t ev = R_ICU->IELSR[i] & R_ICU_IELSR_IELS_Msk;
    for (uint16_t g : GPT_OVF) {
      if (ev != g) continue;
      NVIC_SetPriority((IRQn_Type)i, CONTROL_IRQ_PRIORITY);
      controlIrqSlot = (int8_t)i;
      found++;
    }
  }
  if (found == 0) {
    Serial.println("# WARNING: control timer interrupt not found, priority left at 12");
    return;
  }
  Serial.print("# control irq slot=");  Serial.print(controlIrqSlot);
  Serial.print(" priority=");           Serial.print(NVIC_GetPriority((IRQn_Type)controlIrqSlot));
  Serial.print(" (usb=12, millis=14) gpt_overflow_irqs=");  Serial.println(found);
}

// LOOP, commands and printing only; the ISR never prints
void loop() {
  readCommands();
  reportLimitTrip();
  printEStep();
  printStream();
}

// Prints the step the ISR just finished and updates the streak
void printEStep() {
  if (!eReady) return;
  noInterrupts();
  EStep s = eDone;
  eReady = false;
  interrupts();

  float settle = (s.enterMs < 0) ? -1.0f : s.enterMs / 1000.0f;
  float osPct  = (s.peakM > 0.0f ? s.peakM : 0.0f) / (Y_HIGH - Y_LOW) * 100.0f;
  bool  pass   = settle >= 0.0f && settle <= E_SETTLE_S && osPct < E_OS_PCT;
  eSteps++;
  if (pass) { ePasses++; eStreak++; } else { eStreak = 0; }
  if (eStreak > eBest) eBest = eStreak;

  Serial.print("# e step ");   Serial.print(eSteps);
  Serial.print(" dir=");       Serial.print(s.dir > 0 ? '+' : '-');
  Serial.print(" settle_s=");
  if (settle < 0.0f) Serial.print("never"); else Serial.print(settle, 2);
  Serial.print(" os_pct=");    Serial.print(osPct, 1);
  Serial.print(pass ? " PASS" : " FAIL");
  Serial.print(" streak=");    Serial.print(eStreak);
  Serial.print(" periods=");   Serial.print(eStreak / 2.0f, 1);
  if (eStreak == 2 * E_PERIODS) Serial.print("  <-- 10 consecutive periods passed");
  Serial.println();
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
      case 'w': toggleAw();       break;
      case 'v': streaming = !streaming;
                Serial.println(streaming ? "# stream on" : "# stream off"); break;
      case 'd': printIsrTiming(); break;
      default:                    break;
    }
  }
}

void printAw() {
  Serial.println(awOn ? "# anti-windup ON (conditional integration)"
                      : "# anti-windup OFF (plain part (c) D2[z])");
}

void toggleAw() {
  if (runState == RUNNING) { Serial.println("# refused: stop with s first"); return; }
  awOn = !awOn;
  printAw();
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
  overVoltage   = satSamples = awHeld = runSamples = 0;
  gapMaxUs      = gapAtMs = 0;
  lastIsrUs     = 0;
  eActive       = false;
  eReady        = false;
  eSteps = ePasses = eStreak = eBest = 0;
  limitCount    = 0;
  resetControllers();
  runState      = RUNNING;
  interrupts();

  Serial.print(holdMode ? "# start hold r=" : "# start square r=");
  Serial.print(holdMode ? holdRef : Y_LOW, 3);
  if (!holdMode) { Serial.print("<->"); Serial.print(Y_HIGH, 3);
                   Serial.print(" half_period_ms="); Serial.print(HALF_PERIOD_MS); }
  Serial.print(" ctrl=D2c+D1 aw=");    Serial.print(awOn ? "on" : "off");
  Serial.print(" dt_ms=");             Serial.print(SAMPLE_MS);
  Serial.print(" sat=");               Serial.print(REF_SAT, 2);
  Serial.print(" stiction_pos=");      Serial.print(STICTION_POS, 3);
  Serial.print(" stiction_neg=");      Serial.println(STICTION_NEG, 3);
}

void stopFromUser() {
  noInterrupts();
  stopControl();
  uint32_t over = overVoltage, sat = satSamples, held = awHeld, n = runSamples;
  uint32_t gap = gapMaxUs, gapAt = gapAtMs;
  interrupts();
  Serial.print("# stop, samples=");   Serial.print(n);
  Serial.print(" saturated=");        Serial.print(sat);
  Serial.print(" aw_held=");          Serial.print(held);
  Serial.print(" beyond_DA=");        Serial.print(over);
  Serial.print(" isr_gap_max_us=");   Serial.print(gap);
  Serial.print(" at_ms=");            Serial.println(gapAt);
  if (gap > 5000UL) Serial.println("# WARNING: the control ISR paused for more than 5 ms during this run");
  printEStep();   // a step that finished just before s
  if (eSteps > 0) {
    Serial.print("# e summary: steps=");  Serial.print(eSteps);
    Serial.print(" passed=");             Serial.print(ePasses);
    Serial.print(" best_streak=");        Serial.print(eBest);
    Serial.print(" (");                   Serial.print(eBest / 2.0f, 1);
    Serial.print(" periods; demo needs ");Serial.print(E_PERIODS);
    Serial.println(")");
  }
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
  i2 = x2 = e2_1 = thRef = 0.0;
  thSat = 0.0f;
  ctrlU = ctrlU1 = e1_1 = 0.0;
}

void stopControl() {
  runState     = STOPPED;
  motorVoltage = 0.0f;
  resetControllers();
}

// Flips the ball reference every HALF_PERIOD_SAMPLES in square-wave runs;
// returns true on the sample where it flips
bool updateBallReference() {
  if (!squareRunning) return false;
  if (++halfCount < HALF_PERIOD_SAMPLES) return false;
  halfCount = 0;
  rBall = (rBall == Y_LOW) ? Y_HIGH : Y_LOW;
  return true;
}

// (e) monitor, ISR context. At an edge, hands the finished step to loop() and
// starts tracking the new one; every sample, updates band entry and peak.
void updateEMonitor(bool edge, float y) {
  if (!squareRunning) return;
  if (edge) {
    if (eActive) { eDone = eCur; eReady = true; }
    eActive     = true;
    eCur.dir    = (rBall == Y_HIGH) ? 1 : -1;
    eCur.enterMs = -1;
    eCur.peakM  = 0.0f;
    eSamples    = 0;
  }
  if (!eActive) return;
  bool high = (eCur.dir > 0);
  bool in   = high ? (y >= E_HIGH_MIN && y <= E_HIGH_MAX) : (y >= E_LOW_MIN && y <= E_LOW_MAX);
  if (!in)                     eCur.enterMs = -1;                     // left the band: not settled
  else if (eCur.enterMs < 0)   eCur.enterMs = (int32_t)(eSamples * SAMPLE_MS);
  float past = high ? (y - Y_HIGH) : (Y_LOW - y);
  if (past > eCur.peakM) eCur.peakM = past;
  eSamples++;
}

// Outer loop: split-form D2 on the ball error, then the saturator.
// Anti-windup: if thRef is past the saturator and the new integrator value
// would push it further out, keep the old integrator value instead.
void updateOuter(float y) {
  double e2   = (double)rBall - (double)y;
  double iNew = i2 + D2_KI * TS_S / 2.0 * (e2 + e2_1);
  double xNew = -D2_LA * x2 + D2_LB * (e2 + e2_1);
  double u    = -(D2_KP * e2 + iNew + xNew);

  if (awOn && fabs(u) > REF_SAT) {
    bool pushesOut = (u > 0.0) ? (iNew < i2) : (iNew > i2);   // u = -(... + i)
    if (pushesOut) {
      iNew = i2;
      u    = -(D2_KP * e2 + iNew + xNew);
      awHeld++;
    }
  }

  i2 = iNew;  x2 = xNew;  e2_1 = e2;
  thRef = u;
  thSat = saturate((float)thRef);
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

  // Time since the previous ISR start, tracked while running (expected 1000 us)
  if (runState == RUNNING) {
    if (lastIsrUs != 0 && tUs - lastIsrUs > gapMaxUs) {
      gapMaxUs = tUs - lastIsrUs;
      gapAtMs  = millis();
    }
    lastIsrUs = tUs;
  }

  // Both sensors first: the ball position is a control input
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
      bool edge = updateBallReference();
      updateEMonitor(edge, y);
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
