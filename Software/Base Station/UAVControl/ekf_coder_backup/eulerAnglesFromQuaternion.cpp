/*
 * eulerAnglesFromQuaternion.cpp
 *
 * Code generation for function 'eulerAnglesFromQuaternion'
 *
 * C source code generated on: Mon Jun  9 07:29:50 2014
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "Reb.h"
#include "correctStateAndCov.h"
#include "eulerAnglesFromQuaternion.h"
#include "projectStateAndCov.h"
#include "quaternionRotation.h"
#include "solvePoseEst.h"

/* Function Declarations */
static real_T rt_atan2d_snf(real_T u0, real_T u1);

/* Function Definitions */
static real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T b_u0;
  int32_T b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2((real_T)b_u0, (real_T)b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

void eulerAnglesFromQuaternion(const real_T x[4], real_T *roll, real_T *pitch,
  real_T *yaw)
{
  *roll = rt_atan2d_snf(2.0 * (x[0] * x[1] + x[2] * x[3]), 1.0 - 2.0 * (x[1] *
    x[1] + x[2] * x[2])) * 180.0 / 3.1415926535897931;
  *pitch = asin(2.0 * (x[0] * x[2] - x[3] * x[1])) * 180.0 / 3.1415926535897931;
  *yaw = rt_atan2d_snf(2.0 * (x[0] * x[3] + x[1] * x[2]), 1.0 - 2.0 * (x[2] * x
    [2] + x[3] * x[3])) * 180.0 / 3.1415926535897931;
}

/* End of code generation (eulerAnglesFromQuaternion.cpp) */
