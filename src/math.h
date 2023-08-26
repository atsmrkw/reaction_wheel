#pragma once

#include <array>

constexpr int AXIS_NUM = 3;  // Number of axes

using Vector3 = std::array<float, AXIS_NUM>;

// Axis index
enum Axis { X, Y, Z };

/**
 * @brief Get inner product
 * @param vec1 First vector
 * @param vec2 Second vector
 * @return Inner product
 */
float GetInnerProduct(Vector3 vec1, Vector3 vec2);

/**
 * @brief Get norm
 * @param vec Vector
 * @return Norm
 */
float GetNorm(Vector3 vec);

/**
 * @brief Get angle between two vectors
 * @param vec1 First vector
 * @param vec2 Second vector
 * @return Angle between two vectors
 */
float GetAngle(Vector3 vec1, Vector3 vec2);