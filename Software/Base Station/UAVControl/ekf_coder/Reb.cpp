/*
 * Reb.cpp
 *
 * Code generation for function 'Reb'
 *
 * C source code generated on: Sun Aug 17 12:20:38 2014
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

/* Function Definitions */
void Reb(const real_T q[4], const real_T x[3], real_T y[3])
{
  real_T b_q[9];
  int32_T i12;
  int32_T i13;
  b_q[0] = ((q[0] * q[0] + q[1] * q[1]) - q[2] * q[2]) - q[3] * q[3];
  b_q[3] = 2.0 * (q[1] * q[2] - q[0] * q[3]);
  b_q[6] = 2.0 * (q[1] * q[3] + q[0] * q[2]);
  b_q[1] = 2.0 * (q[1] * q[2] + q[0] * q[3]);
  b_q[4] = ((q[0] * q[0] - q[1] * q[1]) + q[2] * q[2]) - q[3] * q[3];
  b_q[7] = 2.0 * (q[2] * q[3] - q[0] * q[1]);
  b_q[2] = 2.0 * (q[1] * q[3] - q[0] * q[2]);
  b_q[5] = 2.0 * (q[2] * q[3] + q[0] * q[1]);
  b_q[8] = ((q[0] * q[0] - q[1] * q[1]) - q[2] * q[2]) + q[3] * q[3];
  for (i12 = 0; i12 < 3; i12++) {
    y[i12] = 0.0;
    for (i13 = 0; i13 < 3; i13++) {
      y[i12] += b_q[i12 + 3 * i13] * x[i13];
    }
  }
}

/* End of code generation (Reb.cpp) */
