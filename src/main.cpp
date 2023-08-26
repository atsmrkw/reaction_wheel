#include <Arduino.h>

#include "constants.h"
#include "imu.h"
#include "math.h"
#include "motor.h"
#include "pin.h"

/**
 * State
 */
enum class State {
  Idle,
  BalancingOnEdgeX,
  BalancingOnEdgeY,
  BalancingOnEdgeZ,
  BalancingAtCorner
};

constexpr int SERIAL_BAUD_RATE = 115200;  // Baud rate of serial comm [bps]

constexpr int PWM_CH_X = 0;               // PWM channel of X motor
constexpr int PWM_CH_Y = 1;               // PWM channel of Y motor
constexpr int PWM_CH_Z = 2;               // PWM channel of Z motor

constexpr int CYCLE_TIME = 10;            // Loop cycle time [ms]
constexpr int TIME_TO_START = 1000;       // Time to start control [ms]

constexpr float INIT_KP = -12990.0;       // Initial value of Kp
constexpr float INIT_KP2 = 0.1;           // Initial value of Kp2
constexpr float INIT_KD = -488.6;         // Initial value of Kd
constexpr float INIT_KW = 0.51;           // Initial value of Kw

// Target pose (X edge) [G]
static const Vector3 TGT_POSE_X = {0.0, -0.73, -0.66};
// Target pose (Y edge) [G]
static const Vector3 TGT_POSE_Y = {-0.74, 0.0, -0.66};
// Target pose (Z edge) [G]
static const Vector3 TGT_POSE_Z = {-0.74, -0.67, 0.0};

State state_ = State::Idle;       // State
State next_state_ = State::Idle;  // Next state candidate

// Motors
std::array<Motor, AXIS_NUM> motors_ = {
  Motor(Axis::X, pin::MOT_X_DIR, pin::MOT_X_PWM, PWM_CH_X),
  Motor(Axis::Y, pin::MOT_Y_DIR, pin::MOT_Y_PWM, PWM_CH_Y),
  Motor(Axis::Z, pin::MOT_Z_DIR, pin::MOT_Z_PWM, PWM_CH_Z)};

IMU imu_;                             // IMU

unsigned long time_ = 0;              // Current time [ms]
unsigned long prev_time_ = 0;         // Time of previous cycle [ms]
unsigned long time_at_sta_dec_ = 0;   // Time at control start decision [ms]
unsigned long count_ = 0;             // Cycle count

Vector3 accel_ = {0.0, 0.0, 0.0};     // Acceleration [G]
Vector3 gyro_ = {0.0, 0.0, 0.0};      // Gyro [rad/sec]
Vector3 tgt_pose_ = {0.0, 0.0, 0.0};  // Target pose [G]
std::array<int, AXIS_NUM> torque_ = {0, 0, 0};  // Motor torque @todo unit?

// Angular difference [rad]
Vector3 ang_diff_ = {PI / 4, PI / 4, PI / 4};
// Previous angular difference [rad]
Vector3 prev_ang_diff_ = {0.0, 0.0, 0.0};
// Estimated angle ofset [rad]
Vector3 est_ang_ofst_ = {0.0, 0.0, 0.0};

// Gain
Vector3 kp_ = {INIT_KP, INIT_KP, INIT_KP};  // Proportional gain
Vector3 kp2_ = {INIT_KP2, INIT_KP2, INIT_KP2};
Vector3 kd_ = {INIT_KD, INIT_KD, INIT_KD};  // Differential gain
Vector3 kw_ = {INIT_KW, INIT_KW, INIT_KW};  // Integral gain

// Motor angular velocity [rad/s]
Vector3 mot_ang_vel_ = {0.0, 0.0, 0.0};

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
 * @param axis Balancing axis
 */
void BalanceOnEdge(Axis axis);

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
    Vector3 tmp_accel = {0.0, 0.0, 0.0};
    Vector3 tmp_gyro = {0.0, 0.0, 0.0};

    // Get input data
    imu_.GetData(tmp_accel[Axis::X], tmp_accel[Axis::Y], tmp_accel[Axis::Z],
                 tmp_gyro[Axis::X], tmp_gyro[Axis::Y], tmp_gyro[Axis::Z]);

    constexpr float ALPHA_ACCEL = 0.8;
    constexpr float ALPHA_GYRO = 1.0;
    for (int i = 0; i < AXIS_NUM; i++) {
      accel_[i] = (1.0 - ALPHA_ACCEL) * accel_[i] + ALPHA_ACCEL * tmp_accel[i];
      gyro_[i] = (1.0 - ALPHA_GYRO) * gyro_[i] + ALPHA_GYRO * tmp_gyro[i];
    }

    if (state_ == State::Idle) {
      /**
       * @brief Judge and transition state
       * @param cand_state Candidate state
       * @param tgt_pose Target pose in candidate state
       */
      auto judge_and_transition_state = [&](State cand_state,
                                            Vector3 tgt_pose) {
        if (next_state_ != cand_state) {
          next_state_ = cand_state;
          time_at_sta_dec_ = time_;
        } else if (time_ - time_at_sta_dec_ >= TIME_TO_START) {
          state_ = cand_state;
          next_state_ = State::Idle;
          tgt_pose_ = tgt_pose;
        }
      };

      // Control start decision
      if (GetAngle(accel_, TGT_POSE_X) < 15.0 * DEG_TO_RAD) {
        judge_and_transition_state(State::BalancingOnEdgeX, TGT_POSE_X);
      } else if (GetAngle(accel_, TGT_POSE_Y) < 15.0 * DEG_TO_RAD) {
        judge_and_transition_state(State::BalancingOnEdgeY, TGT_POSE_Y);
      } else if (GetAngle(accel_, TGT_POSE_Z) < 15.0 * DEG_TO_RAD) {
        judge_and_transition_state(State::BalancingOnEdgeZ, TGT_POSE_Z);
      } else {
        next_state_ = State::Idle;
      }
    } else if (GetAngle(accel_, tgt_pose_) < 15.0 * DEG_TO_RAD) {
      Vector3 ang_diff_accel = {
        -std::atan2(tgt_pose_[Axis::Y] * accel_[Axis::Z] -
                      tgt_pose_[Axis::Z] * accel_[Axis::Y],
                    tgt_pose_[Axis::Y] * accel_[Axis::Y] +
                      tgt_pose_[Axis::Z] * accel_[Axis::Z]),
        -std::atan2(tgt_pose_[Axis::Z] * accel_[Axis::X] -
                      tgt_pose_[Axis::X] * accel_[Axis::Z],
                    tgt_pose_[Axis::Z] * accel_[Axis::Z] +
                      tgt_pose_[Axis::X] * accel_[Axis::X]),
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
    } else {
      state_ = State::Idle;
      next_state_ = State::Idle;
    }

    // Calculate motor torque
    switch (state_) {
      case State::BalancingOnEdgeX:
        BalanceOnEdge(Axis::X);
        break;
      case State::BalancingOnEdgeY:
        BalanceOnEdge(Axis::Y);
        break;
      case State::BalancingOnEdgeZ:
        BalanceOnEdge(Axis::Z);
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

void BalanceOnEdge(Axis axis) {
  // Update torque
  torque_ = std::array<int, 3>{0, 0, 0};
  torque_[axis] = constrain(
    -kp_[axis] * (ang_diff_[axis] - kp2_[axis] * est_ang_ofst_[axis]) -
      kd_[axis] * gyro_[axis] - kw_[axis] * mot_ang_vel_[axis],
    -Motor::MAX_TORQUE, Motor::MAX_TORQUE);

  // Update angle offset
  static const float ALPHA_OFST = 0.000001;
  Vector3 tmp_est_ang_ofst = {0.0, 0.0, 0.0};
  tmp_est_ang_ofst[axis] = constrain(
    (1.0 - ALPHA_OFST) * est_ang_ofst_[axis] + ALPHA_OFST * mot_ang_vel_[axis],
    -10.0 * DEG_TO_RAD, 10.0 * DEG_TO_RAD);
  est_ang_ofst_ = tmp_est_ang_ofst;
}

void UpdateFeedbackGain() {
  if (Serial.available()) {
    /**
     * @brief Add values to all array elements
     * @param k_array Gain array
     * @param val Value to add
     */
    auto add_values_to_all = [](Vector3& k_array, float val) {
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
  // Serial.println(">accel_[X]:" + String(accel_[Axis::X]));
  // Serial.println(">accel_[Y]:" + String(accel_[Axis::Y]));
  // Serial.println(">accel_[Z]:" + String(accel_[Axis::Z]));
  Serial.println(">ang_diff_x:" + String(ang_diff_[Axis::X] * RAD_TO_DEG));
  // Serial.println(">ang_diff_y:" + String(ang_diff_y_ * RAD_TO_DEG));
  // Serial.println(">ang_diff_z:" + String(ang_diff_z_ * RAD_TO_DEG));
  Serial.println(">mot_ang_vel:" + String(mot_ang_vel_[Axis::X]));
  Serial.println(">gyro:" + String(gyro_[Axis::X]));
  Serial.println(">torque:" + String(torque_[Axis::X]));
  // Serial.println(">est_ang_ofst:" +
  //                String(est_ang_ofst_[Axis::X] * RAD_TO_DEG));
  // Serial.println(">ang_pose_x:" +
  //                String(GetAngle(accel_, TGT_POSE_X) * RAD_TO_DEG));
  // Serial.println(">ang_pose_y:" +
  //                String(GetAngle(accel_, TGT_POSE_Y) * RAD_TO_DEG));
  // Serial.println(">ang_pose_z:" +
  //                String(GetAngle(accel_, TGT_POSE_Z) * RAD_TO_DEG));
}