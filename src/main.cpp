#include <Arduino.h>

#include "constants.h"
#include "imu.h"
#include "motor.h"
#include "pin.h"

/**
 * State
 */
enum class State { Idle, BalancingOnEdge, BalancingAtCorner };

constexpr int SERIAL_BAUD_RATE = 115200;  // Baud rate of serial comm [bps]

constexpr int PWM_CH_X = 0;               // PWM channel of X motor
constexpr int PWM_CH_Y = 1;               // PWM channel of Y motor
constexpr int PWM_CH_Z = 2;               // PWM channel of Z motor

constexpr int CYCLE_TIME = 10;            // Loop cycle time [ms]
constexpr int TIME_TO_START = 1000;       // Time to start control [ms]

constexpr float INIT_KP = -12300.0;       // Initial value of Kp
constexpr float INIT_KP2 = 0.1;           // Initial value of Kp2
constexpr float INIT_KD = -420.2;         // Initial value of Kd
constexpr float INIT_KW = 0.45;           // Initial value of Kw

State state_ = State::Idle;               // State
State next_state_ = State::Idle;          // Next state candidate

// Motors
std::array<Motor, AXIS_NUM> motors_ = {
  Motor(Axis::X, pin::MOT_X_DIR, pin::MOT_X_PWM, PWM_CH_X),
  Motor(Axis::Y, pin::MOT_Y_DIR, pin::MOT_Y_PWM, PWM_CH_Y),
  Motor(Axis::Z, pin::MOT_Z_DIR, pin::MOT_Z_PWM, PWM_CH_Z)};

IMU imu_;                            // IMU

unsigned long time_ = 0;             // Current time [ms]
unsigned long prev_time_ = 0;        // Time of previous cycle [ms]
unsigned long time_at_sta_dec_ = 0;  // Time at control start decision [ms]
unsigned long count_ = 0;            // Cycle count

std::array<float, AXIS_NUM> accel_ = {0.0, 0.0, 0.0};  // Acceleration [G]
std::array<float, AXIS_NUM> gyro_ = {0.0, 0.0, 0.0};   // Gyro [rad/sec]
// VectorFloat tgt_pose_(0.58, 0.56, -0.58);  // Target pose
std::array<float, AXIS_NUM> tgt_pose_ = {0.0, -0.73, -0.66};  // Target pose [G]
std::array<int, AXIS_NUM> torque_ = {0, 0, 0};  // Motor torque @todo unit?

// Angular difference [rad]
std::array<float, AXIS_NUM> ang_diff_ = {PI / 4, PI / 4, PI / 4};
// Previous angular difference [rad]
std::array<float, AXIS_NUM> prev_ang_diff_ = {0.0, 0.0, 0.0};
// Estimated angle ofset [rad]
std::array<float, AXIS_NUM> est_ang_ofst_ = {0.0, 0.0, 0.0};

// Gain
// Proportional gain
std::array<float, AXIS_NUM> kp_ = {INIT_KP, INIT_KP, INIT_KP};
// Differential gain
std::array<float, AXIS_NUM> kd_ = {INIT_KD, INIT_KD, INIT_KD};
// Integral gain
std::array<float, AXIS_NUM> kw_ = {INIT_KW, INIT_KW, INIT_KW};
std::array<float, AXIS_NUM> kp2_ = {0.0, 0.0, 0.0};

// Motor angular velocity [rad/s]
std::array<float, AXIS_NUM> mot_ang_vel_ = {0.0, 0.0, 0.0};

/**
 * @brief Calculate angular difference using complementary filter
 * @param prev_ang_diff Previous angular difference [rad]
 * @param ang_diff_accel Angular difference calculated from acceleration sensor
 * [rad]
 * @param ang_vel Angular velocity aquired from gyro sensor [rad/s]
 * @return Angular difference [rad]
 */
float CalculateAngularDifference(float prev_ang_diff, float ang_diff_accel,
                                 float ang_vel);

/**
 * @brief Get motor angular velocity
 * @param enc_val Encoder value
 * @return Angular velocity [rad/s]
 */
float GetMotorAngularVelocity(int8_t enc_val);

/**
 * @brief Processing in Idle state
 */
void Idle();

/**
 * @brief Processing in BalancingOnEdge state
 */
void BalanceOnEdge();

/**
 * @brief Update feedback gain
 * @details Read characters from serial port.
 */
void UpdateFeedbackGain();

/**
 * @brief Plot parameters with Teleplot
 * @note VSCode extension Teleplot must be installed.
 */
void PlotWithTeleplot();

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
    std::array<float, AXIS_NUM> tmp_accel = {0.0, 0.0, 0.0};
    std::array<float, AXIS_NUM> tmp_gyro = {0.0, 0.0, 0.0};

    // Get input data
    imu_.GetData(tmp_accel[Axis::X], tmp_accel[Axis::Y], tmp_accel[Axis::Z],
                 tmp_gyro[Axis::X], tmp_gyro[Axis::Y], tmp_gyro[Axis::Z]);

    constexpr float ALPHA_ACCEL = 0.8;
    constexpr float ALPHA_GYRO = 1.0;
    for (int i = 0; i < AXIS_NUM; i++) {
      accel_[i] = (1.0 - ALPHA_ACCEL) * accel_[i] + ALPHA_ACCEL * tmp_accel[i];
      gyro_[i] = (1.0 - ALPHA_GYRO) * gyro_[i] + ALPHA_GYRO * tmp_gyro[i];
    }

    std::array<float, AXIS_NUM> ang_diff_accel = {
      -std::atan2(tgt_pose_[Axis::Y] * accel_[Axis::Z] -
                    tgt_pose_[Axis::Z] * accel_[Axis::Y],
                  tgt_pose_[Axis::Y] * accel_[Axis::Y] +
                    tgt_pose_[Axis::Z] * accel_[Axis::Z]),
      -std::atan2((-tgt_pose_[Axis::X]) * accel_[Axis::Z] -
                    tgt_pose_[Axis::Z] * (-accel_[Axis::X]),
                  (-tgt_pose_[Axis::X]) * (-accel_[Axis::X]) +
                    tgt_pose_[Axis::Z] * accel_[Axis::Z]),
      -std::atan2(tgt_pose_[Axis::X] * accel_[Axis::Y] -
                    tgt_pose_[Axis::Y] * accel_[Axis::X],
                  tgt_pose_[Axis::X] * accel_[Axis::X] +
                    tgt_pose_[Axis::Y] * accel_[Axis::Y])};

    for (int i = 0; i < AXIS_NUM; i++) {
      ang_diff_[i] = CalculateAngularDifference(prev_ang_diff_[i],
                                                ang_diff_accel[i], gyro_[i]);
      prev_ang_diff_[i] = ang_diff_[i];
      mot_ang_vel_[i] = GetMotorAngularVelocity(motors_[i].ReadEncoder());
    }

    if ((fabs(ang_diff_[Axis::X]) < 15.0 * DEG_TO_RAD)) {
      if (state_ == State::Idle) {
        if (next_state_ == State::Idle) {
          next_state_ = State::BalancingOnEdge;
          time_at_sta_dec_ = time_;
        } else if ((next_state_ == State::BalancingOnEdge) &&
                   (time_ - time_at_sta_dec_ >= TIME_TO_START)) {
          state_ = State::BalancingOnEdge;
        }
      }
    } else {
      state_ = State::Idle;
      next_state_ = State::Idle;
    }

    // Calculate motor torque
    switch (state_) {
      case State::BalancingOnEdge:
        BalanceOnEdge();
        break;
      case State::Idle:
      default:
        Idle();
        break;
    }

    for (int i = 0; i < AXIS_NUM; i++) {
      motors_[i].Drive(torque_[i]);
    }

    UpdateFeedbackGain();

    if (count_ % 2 == 0) {
      PlotWithTeleplot();
    }

    prev_time_ = time_;
    count_++;
  }
}

float CalculateAngularDifference(float prev_ang_diff, float ang_diff_accel,
                                 float ang_vel) {
  constexpr float COMP_FILT_COEF = 0.05;
  return COMP_FILT_COEF * ang_diff_accel +
         (1.0 - COMP_FILT_COEF) *
           (prev_ang_diff + ang_vel * ((time_ - prev_time_) * MS_TO_S));
}

float GetMotorAngularVelocity(int8_t enc_val) {
  return static_cast<float>(enc_val) * ENC_RES /
         ((time_ - prev_time_) * MS_TO_S);
}

void Idle() {
  for (int i = 0; i < AXIS_NUM; i++) {
    est_ang_ofst_[i] = 0.0;
    torque_[i] = 0;
  }
}

void BalanceOnEdge() {
  std::array<float, AXIS_NUM> tmp_torque = {
    constrain(
      -kp_[Axis::X] *
          (ang_diff_[Axis::X] - kp2_[Axis::X] * est_ang_ofst_[Axis::X]) -
        kd_[Axis::X] * gyro_[Axis::X] - kw_[Axis::X] * mot_ang_vel_[Axis::X],
      -Motor::MAX_TORQUE, Motor::MAX_TORQUE),
    constrain(
      -kp_[Axis::Y] *
          (ang_diff_[Axis::Y] - kp2_[Axis::Y] * est_ang_ofst_[Axis::Y]) -
        kd_[Axis::Y] * gyro_[Axis::Y] - kw_[Axis::Y] * mot_ang_vel_[Axis::Y],
      -Motor::MAX_TORQUE, Motor::MAX_TORQUE),
    constrain(
      kp_[Axis::Z] *
          (ang_diff_[Axis::Z] - kp2_[Axis::Z] * est_ang_ofst_[Axis::Z]) +
        kd_[Axis::Z] * gyro_[Axis::Z] + kw_[Axis::Z] * mot_ang_vel_[Axis::Z],
      -Motor::MAX_TORQUE, Motor::MAX_TORQUE)};

  static const float ALPHA_TORQUE = 1.0;
  torque_[Axis::X] = static_cast<int>((1.0 - ALPHA_TORQUE) * torque_[Axis::X] +
                                      ALPHA_TORQUE * tmp_torque[Axis::X]);
  torque_[Axis::Y] = 0;
  torque_[Axis::Z] = 0;
  // torque_.y = (1.0 - ALPHA_TORQUE) * torque_.y + ALPHA_TORQUE * tmp_torque.y;
  // torque_.z = (1.0 - ALPHA_TORQUE) * torque_.z + ALPHA_TORQUE * tmp_torque.z;

  static const float ALPHA_OFST = 0.000001;
  est_ang_ofst_[Axis::X] =
    constrain((1.0 - ALPHA_OFST) * est_ang_ofst_[Axis::X] +
                ALPHA_OFST * mot_ang_vel_[Axis::X],
              -10.0 * DEG_TO_RAD, 10.0 * DEG_TO_RAD);
  est_ang_ofst_[Axis::Y] = 0.0;
  est_ang_ofst_[Axis::Z] = 0.0;
}

void UpdateFeedbackGain() {
  if (Serial.available()) {
    /**
     * @brief Add values to all array elements
     * @param k_array Gain array
     * @param val Value to add
     */
    auto add_values_to_all = [](std::array<float, AXIS_NUM>& k_array,
                                float val) {
      // for (auto k : k_array) {
      //   k += val;
      // }
      k_array[Axis::X] += val;
      k_array[Axis::Y] += val;
      k_array[Axis::Z] += val;
    };

    char cmd = Serial.read();
    switch (cmd) {
      case 'q':
        add_values_to_all(kp_, 30.0);
        break;
      case 'a':
        add_values_to_all(kp_, -30.0);
        break;
      case 'w':
        add_values_to_all(kd_, 0.1);
        break;
      case 's':
        add_values_to_all(kd_, -0.1);
        break;
      case 'e':
        add_values_to_all(kw_, 0.01);
        break;
      case 'd':
        add_values_to_all(kw_, -0.01);
        break;
      case 'r':
        add_values_to_all(kp2_, 0.01);
        break;
      case 'f':
        add_values_to_all(kp2_, -0.01);
        break;
      default:
        break;
    }

    Serial.print("[rcv] kp: ");
    Serial.print(kp_[Axis::X]);
    Serial.print(", kd: ");
    Serial.print(kd_[Axis::X]);
    Serial.print(", kw: ");
    Serial.print(kw_[Axis::X]);
    Serial.print(", kp2: ");
    Serial.println(kp2_[Axis::X]);
  }
}

void PlotWithTeleplot() {
  // Serial.println(">accel_.x:" + String(accel_.x));
  // Serial.println(">accel_.y:" + String(accel_.y));
  // Serial.println(">accel_.z:" + String(accel_.z));
  Serial.println(">ang_diff_x:" + String(ang_diff_[Axis::X] * RAD_TO_DEG));
  // Serial.println(">ang_diff_y:" + String(ang_diff_y_ * RAD_TO_DEG));
  // Serial.println(">ang_diff_z:" + String(ang_diff_z_ * RAD_TO_DEG));
  Serial.println(">mot_ang_vel:" + String(mot_ang_vel_[Axis::X]));
  Serial.println(">gyro:" + String(gyro_[Axis::X]));
  Serial.println(">torque:" + String(torque_[Axis::X]));
  Serial.println(">est_ang_ofst:" +
                 String(est_ang_ofst_[Axis::X] * RAD_TO_DEG));
}