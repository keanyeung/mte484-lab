#include "geeWhiz.h"
#include <pwm.h>        // UNO R4 PWM helper
#include <FspTimer.h>   // UNO R4 timer helper

// ---- Pins & constants ----
static constexpr uint8_t PWM_PIN  = D9;       // motor PWM
static constexpr uint8_t DIR_PIN  = D8;       // direction
static constexpr float    VMAX     = 6.0f;    // ±6 V saturator
static constexpr float    PWM_HZ   = 24000.0f;
static constexpr float    DUTY_MAX = 99.2f;   // avoid true 100%
static constexpr float    DUTY_MIN = 0.0f;

// ---- Objects ----
static PwmOut   motorPWM(PWM_PIN);  // GPT-backed PWM for D9
static FspTimer controlTimer;       // separate GPT for the control ISR

// ---- ISR adapter ----
static void timer_cb_adapter(timer_callback_args_t *) {
  if (interval_control_code) { interval_control_code(); return; }
  if (intervalControlCode)   { intervalControlCode();   return; }
  // neither defined -> do nothing
}

// ---- API ----
void geeWhizBegin() {
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);

  // Start 24 kHz PWM on D9 at 0% duty
  motorPWM.begin(PWM_HZ, 0.0f);
}

void setMotorVoltage(float volts) {
  // Saturate to ±6 V
  if (volts >  VMAX) volts =  VMAX;
  if (volts < -VMAX) volts = -VMAX;

  // Direction and duty
  digitalWrite(DIR_PIN, (volts >= 0.0f) ? HIGH : LOW);
  float duty = (fabsf(volts) / VMAX) * 100.0f;
  if (duty > DUTY_MAX) duty = DUTY_MAX;
  if (duty < DUTY_MIN) duty = DUTY_MIN;

  motorPWM.pulse_perc(duty);
}

void set_control_interval_ms(uint16_t interval_ms) {
  if (interval_ms == 0) interval_ms = 1;
  const float freq_hz = 1000.0f / (float)interval_ms;

  // Use a GPT timer (don’t touch AGT: it’s used by Arduino timebase)
  uint8_t timer_type = GPT_TIMER;
  int8_t  tindex     = FspTimer::get_available_timer(timer_type); // prefer non-PWM GPT

  bool forced = false;
  if (tindex < 0) {
    // As a fallback, allow using a PWM-reserved GPT if needed
    tindex = FspTimer::get_available_timer(timer_type, true);
    forced = (tindex >= 0);
  }
  if (tindex < 0) return;           // no timer available

  if (forced) {
    // Permit using a PWM-reserved GPT *for the timer only*.
    // PwmOut(D9) will keep its own GPT channel; FspTimer will pick a different one.
    FspTimer::force_use_of_pwm_reserved_timer();
  }

  // Begin periodic timer -> IRQ -> open -> start (open is REQUIRED)
  if (!controlTimer.begin(TIMER_MODE_PERIODIC, timer_type, tindex, freq_hz, 0.0f, timer_cb_adapter)) return;
  if (!controlTimer.setup_overflow_irq()) return;
  if (!controlTimer.open()) return;
  controlTimer.start();
}
