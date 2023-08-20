#pragma once

#include <Arduino.h>

#include "constants.h"
#include "pin.h"

static const float ENC_RES = 2.0 * PI / 100;  // Encoder resolution[rad/pulse]

/**
 * @brief Motor class
 * @note Nidec 24H
 */
class Motor {
 public:
  static const int MAX_TORQUE = 255;

  /**
   * @brief Construct a new Motor object
   *
   * @param axis Axis
   * @param pin_dir Pin number of direction
   * @param pin_pwm Pin number of PWM
   * @param pwm_ch PWM channel
   */
  Motor(Axis axis, int pin_dir, int pin_pwm, int pwm_ch);

  /**
   * @brief Destroy the Motor object
   *
   */
  ~Motor();

  /**
   * @brief Setup encoder interrupt
   *
   * @param pin_x_enc_a Pin number of X encoder signal A
   * @param pin_x_enc_b Pin number of X encoder signal B
   * @param pin_y_enc_a Pin number of Y encoder signal A
   * @param pin_y_enc_b Pin number of Y encoder signal B
   * @param pin_z_enc_a Pin number of Z encoder signal A
   * @param pin_z_enc_b Pin number of Z encoder signal B
   */
  static void SetupEncoderInterrupt(int pin_x_enc_a, int pin_x_enc_b,
                                    int pin_y_enc_a, int pin_y_enc_b,
                                    int pin_z_enc_a, int pin_z_enc_b);

  /**
   * @brief Drive motor
   * @param torque Torque (-255 ~ 255)
   */
  void Drive(int torque);

  /**
   * @brief Read encoder
   *
   * @return int8_t Encoder count value since last read
   */
  int8_t ReadEncoder();

 private:
  static void UpdateRotationX();
  static void UpdateRotationY();
  static void UpdateRotationZ();
  static void UpdateEncoder(int pin_enc_a, int pin_enc_b,
                            volatile uint8_t &enc_val,
                            volatile int8_t &enc_cnt);

  Axis axis_;    // Axis
  int pin_dir_;  // Pin number of direction
  int pin_pwm_;  // Pin number of PWM
  int pwm_ch_;   // PWM channel
};