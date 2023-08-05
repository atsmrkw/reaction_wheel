#include "motor.h"

constexpr int PWM_BASE_FREQ = 20000;
constexpr int PWM_TIMER_BIT = 8;

constexpr int CW = HIGH;
constexpr int CCW = LOW;

volatile uint8_t enc_val_x_ = 0;
volatile uint8_t enc_val_y_ = 0;
volatile uint8_t enc_val_z_ = 0;
volatile int8_t enc_cnt_x_ = 0;
volatile int8_t enc_cnt_y_ = 0;
volatile int8_t enc_cnt_z_ = 0;

Motor::Motor(Axis axis, int pin_dir, int pin_pwm, int pwm_ch)
  : axis_(axis), pin_dir_(pin_dir), pin_pwm_(pin_pwm), pwm_ch_(pwm_ch) {
  pinMode(pin_dir_, OUTPUT);
  ledcSetup(pwm_ch_, PWM_BASE_FREQ, PWM_TIMER_BIT);
  ledcAttachPin(pin_pwm_, pwm_ch_);
  digitalWrite(pin_dir_, CW);
  Drive(0);
}

Motor::~Motor() {}

void Motor::SetupEncoderInterrupt(int pin_x_enc_a, int pin_x_enc_b,
                                  int pin_y_enc_a, int pin_y_enc_b,
                                  int pin_z_enc_a, int pin_z_enc_b) {
  attachInterrupt(pin_x_enc_a, UpdateRotationX, CHANGE);
  attachInterrupt(pin_x_enc_b, UpdateRotationX, CHANGE);
  attachInterrupt(pin_y_enc_a, UpdateRotationY, CHANGE);
  attachInterrupt(pin_y_enc_b, UpdateRotationY, CHANGE);
  attachInterrupt(pin_z_enc_a, UpdateRotationZ, CHANGE);
  attachInterrupt(pin_z_enc_b, UpdateRotationZ, CHANGE);
}

void Motor::Drive(int speed) {
  if (speed >= 0) {
    digitalWrite(pin_dir_, CW);
  } else {
    digitalWrite(pin_dir_, CCW);
    speed *= -1;
  }
  ledcWrite(pwm_ch_, speed > 255 ? 255 : 255 - speed);
}

int8_t Motor::ReadEncoder() {
  int8_t cnt;
  switch (axis_) {
    case Axis::X:
      cnt = enc_cnt_x_;
      enc_cnt_x_ = 0;
      break;
    case Axis::Y:
      cnt = enc_cnt_y_;
      enc_cnt_y_ = 0;
      break;
    default:
      cnt = enc_cnt_z_;
      enc_cnt_z_ = 0;
      break;
  }
  return cnt;
}

void Motor::UpdateRotationX() {
  UpdateEncoder(pin::MOT_X_ENC_A, pin::MOT_X_ENC_B, enc_val_x_, enc_cnt_x_);
}
void Motor::UpdateRotationY() {
  UpdateEncoder(pin::MOT_Y_ENC_A, pin::MOT_Y_ENC_B, enc_val_y_, enc_cnt_y_);
}
void Motor::UpdateRotationZ() {
  UpdateEncoder(pin::MOT_Z_ENC_A, pin::MOT_Z_ENC_B, enc_val_z_, enc_cnt_z_);
}

void Motor::UpdateEncoder(int pin_enc_a, int pin_enc_b,
                          volatile uint8_t &enc_val, volatile int8_t &enc_cnt) {
  /*
  | B | A | enc |
  | - | - | --- |
  | 0 | 0 | 0   |
  | 0 | 1 | 1   |
  | 1 | 1 | 2   |
  | 1 | 0 | 3   |
  */
  uint8_t cur_enc_val = (digitalRead(pin_enc_b) << 1) + digitalRead(pin_enc_a);
  if (cur_enc_val == 3)
    cur_enc_val = 2;
  else if (cur_enc_val == 2)
    cur_enc_val = 3;

  if (cur_enc_val == enc_val + 1)
    enc_cnt++;
  else if (cur_enc_val + 1 == enc_val)
    enc_cnt--;
  else if (enc_val == 3 && cur_enc_val == 0)
    enc_cnt++;
  else if (enc_val == 0 && cur_enc_val == 3)
    enc_cnt--;

  enc_val = cur_enc_val;
}
