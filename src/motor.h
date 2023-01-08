#pragma once

/**
 * @brief Motor class
 * @note Nidec 24H
 */
class Motor {
 public:
  /**
   * @brief Construct a new Motor object
   * @param pin_dir Pin number of direction
   * @param pin_pwm Pin number of PWM
   * @param pwm_ch PWM channel
   */
  Motor(int pin_dir, int pin_pwm, int pwm_ch);

  /**
   * @brief Destroy the Motor object
   *
   */
  ~Motor();

  /**
   * @brief Drive motor
   * @param speed Speed (0 - 255)
   */
  void Drive(int speed);

 private:
  int pin_dir_;  //!< Pin number of direction
  int pin_pwm_;  //!< Pin number of PWM
  int pwm_ch_;   //!< PWM channel
};