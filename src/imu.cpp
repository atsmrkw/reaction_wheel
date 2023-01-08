#include "imu.h"

#include <Wire.h>

IMU::IMU() {}

IMU::~IMU() {}

void IMU::Initialize() {
  Wire.begin();
  mpu6050_.initialize();
}

void IMU::GetData(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx,
                  int16_t* gy, int16_t* gz) {
  mpu6050_.getMotion6(ax, ay, az, gx, gy, gz);
}