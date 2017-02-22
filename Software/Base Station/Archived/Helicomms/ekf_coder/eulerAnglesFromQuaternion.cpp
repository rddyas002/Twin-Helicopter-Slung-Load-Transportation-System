/*
 * eulerAnglesFromQuaternion.cpp
 *
 * Code generation for function 'eulerAnglesFromQuaternion'
 *
 * C source code generated on: Wed Jan 08 14:21:44 2014
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "Reb.h"
#include "correctStateAndCov.h"
#include "eulerAnglesFromQuaternion.h"
#include "projectStateAndCov.h"
#include "quaternionRotation.h"
#include "ekf_coder_rtwutil.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
static real_T rt_atan2d_snf(real_T u0, real_T u1);

/* Function Definitions */
static real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T i13;
  int32_T i14;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u1 > 0.0) {
      i13 = 1;
    } else {
      i13 = -1;
    }

    if (u0 > 0.0) {
      i14 = 1;
    } else {
      i14 = -1;
    }

    y = atan2((real_T)i14, (real_T)i13);
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
  *roll = rt_atan2d_snf(2.0 * (x[0] * x[1] + x[2] * x[3]), 1.0 - 2.0 *
                        (rt_powd_snf(x[1], 2.0) + rt_powd_snf(x[2], 2.0))) *
    180.0 / 3.1415926535897931;
  *pitch = asin(2.0 * (x[0] * x[2] - x[3] * x[1])) * 180.0 / 3.1415926535897931;
  *yaw = rt_atan2d_snf(2.0 * (x[0] * x[3] + x[1] * x[2]), 1.0 - 2.0 *
                       (rt_powd_snf(x[2], 2.0) + rt_powd_snf(x[3], 2.0))) *
    180.0 / 3.1415926535897931;
}

/* End of code generation (eulerAnglesFromQuaternion.cpp) */
