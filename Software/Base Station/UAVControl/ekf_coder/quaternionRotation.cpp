/*
 * quaternionRotation.cpp
 *
 * Code generation for function 'quaternionRotation'
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
void quaternionRotation(const real_T q1[4], const real_T q2[4], real_T q[4])
{
  real_T b_q2[16];
  int32_T i10;
  int32_T i11;

  /*  rotation q1 by q2 */
  b_q2[0] = q2[0];
  b_q2[4] = -q2[1];
  b_q2[8] = -q2[2];
  b_q2[12] = -q2[3];
  b_q2[1] = q2[1];
  b_q2[5] = q2[0];
  b_q2[9] = q2[3];
  b_q2[13] = q2[2];
  b_q2[2] = q2[2];
  b_q2[6] = q2[3];
  b_q2[10] = q2[0];
  b_q2[14] = -q2[1];
  b_q2[3] = q2[3];
  b_q2[7] = -q2[2];
  b_q2[11] = q2[1];
  b_q2[15] = q2[0];
  for (i10 = 0; i10 < 4; i10++) {
    q[i10] = 0.0;
    for (i11 = 0; i11 < 4; i11++) {
      q[i10] += b_q2[i10 + (i11 << 2)] * q1[i11];
    }
  }
}

/* End of code generation (quaternionRotation.cpp) */
