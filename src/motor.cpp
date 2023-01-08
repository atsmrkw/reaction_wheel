#include "motor.h"

#include <Arduino.h>

constexpr int PWM_BASE_FREQ = 20000;
constexpr int PWM_TIMER_BIT = 8;

Motor::Motor(int pin_dir, int pin_pwm, int pwm_ch)
  : pin_dir_(pin_dir), pin_pwm_(pin_pwm), pwm_ch_(pwm_ch) {
  pinMode(pin_dir_, OUTPUT);
  ledcSetup(pwm_ch_, PWM_BASE_FREQ, PWM_TIMER_BIT);
  ledcAttachPin(pin_pwm_, pwm_ch_);
  digitalWrite(pin_dir_, HIGH);
  Drive(0);
}

Motor::~Motor() {}

void Motor::Drive(int speed) {
  ledcWrite(pwm_ch_, speed > 255 ? 255 : 255 - speed);
}