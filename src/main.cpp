#include <Arduino.h>

#include "imu.h"
#include "motor.h"
#include "pin.h"

constexpr int SERIAL_BAUD_RATE = 115200;  //!< Baud rate of serial comm [bps]

constexpr int PWM_CH_X = 0;  //!< PWM channel of X motor
constexpr int PWM_CH_Y = 1;  //!< PWM channel of Y motor
constexpr int PWM_CH_Z = 2;  //!< PWM channel of Z motor

constexpr int CYCLE_TIME = 10;  //!< Loop cycle time [ms]

Motor motor_x_(pin::MOT_X_DIR, pin::MOT_X_PWM, PWM_CH_X);  //!< X motor
Motor motor_y_(pin::MOT_Y_DIR, pin::MOT_Y_PWM, PWM_CH_Y);  //!< Y motor
Motor motor_z_(pin::MOT_Z_DIR, pin::MOT_Z_PWM, PWM_CH_Z);  //!< Z motor

IMU imu_;  //!< IMU

unsigned long time_ = 0;       //!< Current time [ms]
unsigned long prev_time_ = 0;  //!< Time of previous cycle [ms]

int16_t acc_x_;
int16_t acc_y_;
int16_t acc_z_;
int16_t gyro_x_;
int16_t gyro_y_;
int16_t gyro_z_;

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  imu_.Initialize();

  pinMode(pin::BUZZER, OUTPUT);
  pinMode(pin::BRAKE, OUTPUT);
  digitalWrite(pin::BRAKE, HIGH);

  digitalWrite(pin::BUZZER, HIGH);
  delay(50);
  digitalWrite(pin::BUZZER, LOW);
}

void loop() {
  time_ = millis();

  if (time_ - prev_time_ >= CYCLE_TIME) {
    prev_time_ = time_;

    imu_.GetData(&acc_x_, &acc_y_, &acc_z_, &gyro_x_, &gyro_y_, &gyro_z_);
    Serial.print("acc: (");
    Serial.print(acc_x_);
    Serial.print(", ");
    Serial.print(acc_y_);
    Serial.print(", ");
    Serial.print(acc_z_);
    Serial.print(") gyro: (");
    Serial.print(gyro_x_);
    Serial.print(", ");
    Serial.print(gyro_y_);
    Serial.print(", ");
    Serial.print(gyro_z_);
    Serial.println(")");
  }
}