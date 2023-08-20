#pragma once

#include "../lib/MPU6050/MPU6050.h"

/**
 * @brief IMU class
 *
 * @note MPU 6050
 */
class IMU {
 public:
  /**
   * @brief Construct a new IMU object
   */
  IMU();

  /**
   * @brief Destroy the IMU object
   */
  ~IMU();

  /**
   * @brief Initialize IMU
   */
  void Initialize();

  /**
   * @brief Get the Data object
   *
   * @param ax X axis acceleration [G]
   * @param ay Y axis acceleration [G]
   * @param az Z axis acceleration [G]
   * @param gx Angular velocity around X axis [rad/sec]
   * @param gy Angular velocity around Y axis [rad/sec]
   * @param gz Angular velocity around Z axis [rad/sec]
   */
  void GetData(float& ax, float& ay, float& az, float& gx, float& gy,
               float& gz);

 private:
  MPU6050 mpu6050_;  // MPU6050
};