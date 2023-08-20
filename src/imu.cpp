#include "imu.h"

#include <Wire.h>

constexpr float DATA_TO_G =
  1.0 / 16384.0;                 // Convert to gravitational acceleration [G]
constexpr float DATA_TO_RAD_PER_SEC =
  250.0 / 32768.0 * DEG_TO_RAD;  // Convert to angular acceleration [rad/sec]

constexpr int16_t OFST_X_ACCEL = 1073;
constexpr int16_t OFST_Y_ACCEL = 2013;
constexpr int16_t OFST_Z_ACCEL = 907;
constexpr int16_t OFST_X_GYRO = 55;
constexpr int16_t OFST_Y_GYRO = -5;
constexpr int16_t OFST_Z_GYRO = -27;

IMU::IMU() {}

IMU::~IMU() {}

void IMU::Initialize() {
  Wire.begin();
  mpu6050_.initialize();

  mpu6050_.setXAccelOffset(OFST_X_ACCEL);
  mpu6050_.setYAccelOffset(OFST_Y_ACCEL);
  mpu6050_.setZAccelOffset(OFST_Z_ACCEL);
  mpu6050_.setXGyroOffset(OFST_X_GYRO);
  mpu6050_.setYGyroOffset(OFST_Y_GYRO);
  mpu6050_.setZGyroOffset(OFST_Z_GYRO);
}

void IMU::GetData(float& ax, float& ay, float& az, float& gx, float& gy,
                  float& gz) {
  int16_t raw_ax;
  int16_t raw_ay;
  int16_t raw_az;
  int16_t raw_gx;
  int16_t raw_gy;
  int16_t raw_gz;

  mpu6050_.getMotion6(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);

  ax = static_cast<float>(-raw_ax) * DATA_TO_G;
  ay = static_cast<float>(-raw_ay) * DATA_TO_G;
  az = static_cast<float>(-raw_az) * DATA_TO_G;
  gx = static_cast<float>(raw_gx) * DATA_TO_RAD_PER_SEC;
  gy = static_cast<float>(raw_gy) * DATA_TO_RAD_PER_SEC;
  gz = static_cast<float>(raw_gz) * DATA_TO_RAD_PER_SEC;
}