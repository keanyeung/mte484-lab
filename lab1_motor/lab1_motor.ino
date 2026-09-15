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
