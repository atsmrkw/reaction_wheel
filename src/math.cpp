#include "math.h"

#include <cmath>

float GetInnerProduct(Vector3 vec1, Vector3 vec2) {
  float ret = 0.0;
  for (int i = 0; i < AXIS_NUM; i++) {
    ret += vec1[i] * vec2[i];
  }
  return ret;
}

float GetNorm(Vector3 vec) {
  float ret = 0.0;
  for (int i = 0; i < AXIS_NUM; i++) {
    ret += vec[i] * vec[i];
  }
  return sqrtf(ret);
}

float GetAngle(Vector3 vec1, Vector3 vec2) {
  return acos(GetInnerProduct(vec1, vec2) / (GetNorm(vec1) * GetNorm(vec2)));
}