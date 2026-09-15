#pragma once
#include <Arduino.h>

// Init motor outputs: D9 PWM @ 24 kHz, D8 DIR
void geeWhizBegin();

// Drive motor with voltage command in ±6 V (saturated)
void setMotorVoltage(float volts);

// Start a periodic ISR at 'interval_ms' using a GPT timer (no PWM conflict)
void set_control_interval_ms(uint16_t interval_ms);

// Your sketch must define ONE of these names (either is accepted)
extern "C" void interval_control_code(void) __attribute__((weak));
extern "C" void intervalControlCode(void) __attribute__((weak));
