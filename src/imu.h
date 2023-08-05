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
   *
   */
  IMU();

  /**
   * @brief Destroy the IMU object
   *
   */
  ~IMU();

  /**
   * @brief Initialize IMU
   *
   */
  void Initialize();

  /**
   * @brief Get the Data object
   *
   * @param ax
   * @param ay
   * @param az
   * @param gx
   * @param gy
   * @param gz
   */
  void GetData(float& ax, float& ay, float& az, float& gx, float& gy,
               float& gz);

 private:
  MPU6050 mpu6050_;  // MPU6050
};