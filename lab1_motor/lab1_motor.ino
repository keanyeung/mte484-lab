#include <Arduino.h>
#include "geeWhiz.h"

// ================== Pins ==================
int MOT_PIN = A0;   // motor angle sensor
int BAL_PIN = A1;   // ball position sensor

// ================== Sensor Scaling ==================
// Gear angle: theta [rad] = THETA_M * reading [counts] + THETA_OFFSET
// Part (c) 3-point least-squares fit (+pi/4, 0, -pi/4), R^2 = 0.999995
// Valid for readings 0-16383 (theta = -2.02 to +3.95 rad); reading rolls over outside that range
const float THETA_M      = -3.645179e-4f;   // rad per ADC count
const float THETA_OFFSET =  3.949147f;      // rad

// ================== Stiction Compensation ==================
// Part (d): mean + 2 sd of breakaway voltage over 54 ramp runs (theta = -0.5, 0, +0.5 rad)
// +V turns the large gear CW (theta decreases), -V turns it CCW (theta increases)
const float STICTION_POS = 0.277f;   // added when V > 0 (CW)  [V]
const float STICTION_NEG = 0.259f;   // subtracted when V < 0 (CCW) [V]

// ================== Plant Model ==================
// theta(s) / V(s) = PLANT_K1 / (s (PLANT_TAU s + 1))
// Part (e): %OS and Tp of 30 closed-loop steps of 0.2 rad (Kp = -15, -20 and
// -25 V/rad, 10 steps each) converted to zeta and wn, then to K1 and tau.
// Spread is one standard deviation over those 30 steps.
// Per gain: K1 = -1.929, -1.814, -1.906 and tau = 20.8, 22.2, 21.9 ms, i.e. no
// trend with Kp. Simulating the steps from these values matches the measured
// angle to 0.0025 rad rms, which is 1.3 % of the step (validate_model.py).
// K1 < 0 because +V turns the large gear CW, which makes theta decrease.
const float PLANT_K1  = -1.883f;    // rad/(V s), +- 0.118
const float PLANT_TAU =  0.0216f;   // s, i.e. 21.6 +- 1.3 ms


// ================== Setup ==================
void setup() {

  analogReadResolution(14);
  pinMode(A5, OUTPUT);   // A5 can be used to measure cycle time using an oscilloscope by connecting the scope to the Arduino Box Motor Leads
  Serial.begin(115200);
  delay(300);

  geeWhizBegin();
  set_control_interval_ms(100); // 100 ms loop
  setMotorVoltage(0.0f);

  Serial.println("geeWhiz Started");
}

// ================== Loop ==================
void loop() {

}

// ================== Stiction Compensation ==================
// Pushes a nonzero command past the dead zone in its direction; zero stays zero
float applyStiction(float v) {
  if (v > 0.0f) return v + STICTION_POS;
  if (v < 0.0f) return v - STICTION_NEG;
  return 0.0f;
}

// ================== Control ISR ==================
void interval_control_code(void) {
  // ---- For Serial Plotter Scaling
  int maxy = 17000;
  int miny = 0;
  // ---- Read sensors ----
  int motor = analogRead(MOT_PIN);
  int ball  = analogRead(BAL_PIN);

  // ---- Sensor scaling ----
  float theta = THETA_M * motor + THETA_OFFSET;   // gear angle [rad]

  digitalWrite(A5,HIGH);   // A5 can be used to measure cycle time using an oscilloscope by connecting the scope to the Arduino Box Motor Leads
  Serial.print(maxy);
    Serial.print(",");
  Serial.print(miny);
    Serial.print(",");
  Serial.print(ball);
  Serial.print(",");
  Serial.print(motor);
  Serial.print(",");
  Serial.println(theta, 4);   // gear angle [rad], 4 decimals
  digitalWrite(A5,LOW);   // A5 can be used to measure cycle time using an oscilloscope by connecting the scope to the Arduino Box Motor Leads

}
