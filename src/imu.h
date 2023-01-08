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
  void GetData(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy,
               int16_t* gz);

 private:
  MPU6050 mpu6050_;  //!< MPU6050
};