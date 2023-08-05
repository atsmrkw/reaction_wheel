#include <Arduino.h>

#include "imu.h"
#include "motor.h"
#include "pin.h"

constexpr int SERIAL_BAUD_RATE = 115200;  // Baud rate of serial comm [bps]

constexpr int PWM_CH_X = 0;  // PWM channel of X motor
constexpr int PWM_CH_Y = 1;  // PWM channel of Y motor
constexpr int PWM_CH_Z = 2;  // PWM channel of Z motor

constexpr int CYCLE_TIME = 10;  // Loop cycle time [ms]

static const float INST_ANGLE =
  -atan(2.0 / sqrt(2.0));  // IMU installation angle

Motor motor_x_(Motor::Axis::X, pin::MOT_X_DIR, pin::MOT_X_PWM,
               PWM_CH_X);  // X motor
Motor motor_y_(Motor::Axis::Y, pin::MOT_Y_DIR, pin::MOT_Y_PWM,
               PWM_CH_Y);  // Y motor
Motor motor_z_(Motor::Axis::Z, pin::MOT_Z_DIR, pin::MOT_Z_PWM,
               PWM_CH_Z);  // Z motor

IMU imu_;  // IMU
Quaternion IMU_ROT(cos(INST_ANGLE / 2), -1 / sqrt(2.0) * sin(INST_ANGLE / 2),
                   1 / sqrt(2.0) * sin(INST_ANGLE / 2), 0);

unsigned long time_ = 0;       // Current time [ms]
unsigned long prev_time_ = 0;  // Time of previous cycle [ms]
unsigned long count_ = 0;      // Cycle count

VectorFloat accel_(0.0, 0.0, 0.0);         // Acceleration vector
VectorFloat gyro_(0.0, 0.0, 0.0);          // Gyro data
VectorFloat tgt_pose_(0.58, 0.56, -0.58);  // Target pose
VectorFloat speed_(0.0, 0.0, 0.0);         // Motor speed

// Gain
float kp_x_ = 500.0;  // Proportional gain
float kp_y_ = 500.0;  // Proportional gain
float kp_z_ = 500.0;  // Proportional gain
float kd_x_ = 20.0;   // Differential gain
float kd_y_ = 20.0;   // Differential gain
float kd_z_ = 20.0;   // Differential gain
float kw_x_ = 0.0;    // Integral gain
float kw_y_ = 0.0;    // Integral gain
float kw_z_ = 0.0;    // Integral gain
float kp2_x_ = 0.0;
float kp2_y_ = 0.0;
float kp2_z_ = 0.0;

VectorFloat mot_ang_vel_(0.0, 0.0,
                         0.0);  // Motor angular velocity @todo local val
VectorFloat mot_ang_vel_i_(0.0, 0.0, 0.0);

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  imu_.Initialize();

  pinMode(pin::BUZZER, OUTPUT);
  pinMode(pin::MOT_X_BRK, OUTPUT);
  digitalWrite(pin::MOT_X_BRK, HIGH);
  pinMode(pin::MOT_Y_BRK, OUTPUT);
  digitalWrite(pin::MOT_Y_BRK, HIGH);
  pinMode(pin::MOT_Z_BRK, OUTPUT);
  digitalWrite(pin::MOT_Z_BRK, HIGH);

  pinMode(pin::MOT_X_ENC_A, INPUT);
  pinMode(pin::MOT_X_ENC_B, INPUT);
  pinMode(pin::MOT_Y_ENC_A, INPUT);
  pinMode(pin::MOT_Y_ENC_B, INPUT);
  pinMode(pin::MOT_Z_ENC_A, INPUT);
  pinMode(pin::MOT_Z_ENC_B, INPUT);
  Motor::SetupEncoderInterrupt(pin::MOT_X_ENC_A, pin::MOT_X_ENC_B,
                               pin::MOT_Y_ENC_A, pin::MOT_Y_ENC_B,
                               pin::MOT_Z_ENC_A, pin::MOT_Z_ENC_B);

  digitalWrite(pin::BUZZER, HIGH);
  delay(50);
  digitalWrite(pin::BUZZER, LOW);
}

void loop() {
  time_ = millis();

  if (time_ - prev_time_ >= CYCLE_TIME) {
    VectorFloat tmp_accel;
    VectorFloat tmp_gyro;

    // Get input data
    imu_.GetData(tmp_accel.x, tmp_accel.y, tmp_accel.z, tmp_gyro.x, tmp_gyro.y,
                 tmp_gyro.z);
    tmp_accel.rotate(&IMU_ROT);
    tmp_gyro.rotate(&IMU_ROT);

    constexpr float ALPHA_ACCEL = 0.5;
    constexpr float ALPHA_GYRO = 0.5;
    accel_.x = (1.0 - ALPHA_ACCEL) * accel_.x + ALPHA_ACCEL * tmp_accel.x;
    accel_.y = (1.0 - ALPHA_ACCEL) * accel_.y + ALPHA_ACCEL * tmp_accel.y;
    accel_.z = (1.0 - ALPHA_ACCEL) * accel_.z + ALPHA_ACCEL * tmp_accel.z;
    gyro_.x = (1.0 - ALPHA_GYRO) * gyro_.x + ALPHA_GYRO * tmp_gyro.x;
    gyro_.y = (1.0 - ALPHA_GYRO) * gyro_.y + ALPHA_GYRO * tmp_gyro.y;
    gyro_.z = (1.0 - ALPHA_GYRO) * gyro_.z + ALPHA_GYRO * tmp_gyro.z;

    float ang_diff_x =
      -std::atan2(tgt_pose_.y * accel_.z - tgt_pose_.z * accel_.y,
                  tgt_pose_.y * accel_.y + tgt_pose_.z * accel_.z);
    float ang_diff_y =
      -std::atan2((-tgt_pose_.x) * accel_.z - tgt_pose_.z * (-accel_.x),
                  (-tgt_pose_.x) * (-accel_.x) + tgt_pose_.z * accel_.z);
    float ang_diff_z =
      -std::atan2(tgt_pose_.x * accel_.y - tgt_pose_.y * accel_.x,
                  tgt_pose_.x * accel_.x + tgt_pose_.y * accel_.y);

    // mot_ang_vel_.x += 0.01 * motor_x_.ReadEncoder();
    // mot_ang_vel_.y += 0.01 * motor_y_.ReadEncoder();
    // mot_ang_vel_.z += 0.01 * motor_z_.ReadEncoder();
    mot_ang_vel_.x = motor_x_.ReadEncoder();  // @todo anguler velocity
    mot_ang_vel_.y = motor_y_.ReadEncoder();  // @todo anguler velocity
    mot_ang_vel_.z = motor_z_.ReadEncoder();  // @todo anguler velocity

    // Calculate motor speed
    if ((fabs(ang_diff_x) < 15.0 * DEG_TO_RAD) &&
        (fabs(ang_diff_y) < 15.0 * DEG_TO_RAD) &&
        (fabs(ang_diff_z) < 15.0 * DEG_TO_RAD)) {
      VectorFloat tmp_speed(
        constrain(-kp_x_ * (ang_diff_x + kp2_x_ * mot_ang_vel_i_.x) -
                    kd_x_ * gyro_.x + kw_x_ * mot_ang_vel_.x,
                  -255, 255),
        constrain(-kp_y_ * (ang_diff_y + kp2_y_ * mot_ang_vel_i_.y) -
                    kd_y_ * gyro_.y + kw_y_ * mot_ang_vel_.y,
                  -255, 255),
        constrain(kp_z_ * (ang_diff_z - kp2_z_ * mot_ang_vel_i_.z) +
                    kd_z_ * gyro_.z - kw_z_ * mot_ang_vel_.z,
                  -255, 255));

      static const float ALPHA_SPEED = 0.5;
      speed_.x = (1.0 - ALPHA_SPEED) * speed_.x + ALPHA_SPEED * tmp_speed.x;
      speed_.y = (1.0 - ALPHA_SPEED) * speed_.y + ALPHA_SPEED * tmp_speed.y;
      speed_.z = (1.0 - ALPHA_SPEED) * speed_.z + ALPHA_SPEED * tmp_speed.z;

      motor_x_.Drive(speed_.x);
      motor_y_.Drive(speed_.y);
      motor_z_.Drive(speed_.z);
    } else {
      speed_.x = 0;
      speed_.y = 0;
      speed_.z = 0;
      motor_x_.Drive(0);
      motor_y_.Drive(0);
      motor_z_.Drive(0);
    }

    // Update parameters
    mot_ang_vel_i_.x += 0.0001 * mot_ang_vel_.x;
    mot_ang_vel_i_.y += 0.0001 * mot_ang_vel_.y;
    mot_ang_vel_i_.z += 0.0001 * mot_ang_vel_.z;

    // Output log
    if (count_ % 200 == 0) {
      Serial.print(time_);
      Serial.print(":  acc: (");
      Serial.print(accel_.x);
      Serial.print(", ");
      Serial.print(accel_.y);
      Serial.print(", ");
      Serial.print(accel_.z);
      Serial.print(") gyro: (");
      Serial.print(gyro_.x);
      Serial.print(", ");
      Serial.print(gyro_.y);
      Serial.print(", ");
      Serial.print(gyro_.z);
      Serial.println(")");
      Serial.print("spd_x: ");
      Serial.println(speed_.x);
    }

    if (Serial.available()) {
      char cmd = Serial.read();
      switch (cmd) {
        case 'q':
          kp_x_ += 30;
          kp_y_ += 30;
          kp_z_ += 30;
          break;
        case 'a':
          kp_x_ -= 30;
          kp_y_ -= 30;
          kp_z_ -= 30;
          break;
        case 'w':
          kd_x_ += 0.1;
          kd_y_ += 0.1;
          kd_z_ += 0.1;
          break;
        case 's':
          kd_x_ -= 0.1;
          kd_y_ -= 0.1;
          kd_z_ -= 0.1;
          break;
        case 'e':
          kw_x_ += 0.1;
          kw_y_ += 0.1;
          kw_z_ += 0.1;
          break;
        case 'd':
          kw_x_ -= 0.1;
          kw_y_ -= 0.1;
          kw_z_ -= 0.1;
          break;
        case 'r':
          kp2_x_ += 0.001;
          kp2_y_ += 0.001;
          kp2_z_ += 0.001;
          break;
        case 'f':
          kp2_x_ -= 0.001;
          kp2_y_ -= 0.001;
          kp2_z_ -= 0.001;
          break;
        default:
          break;
      }
      Serial.print("[rcv] kp: ");
      Serial.print(kp_x_);
      Serial.print(", kd: ");
      Serial.print(kd_x_);
      Serial.print(", kw: ");
      Serial.print(kw_x_);
      Serial.print(", kp2: ");
      Serial.println(kp2_x_);
    }

    prev_time_ = time_;
    count_++;
  }
}