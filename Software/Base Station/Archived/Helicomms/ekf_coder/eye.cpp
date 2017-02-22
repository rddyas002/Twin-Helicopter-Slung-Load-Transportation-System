/*
 * eye.cpp
 *
 * Code generation for function 'eye'
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
#include "eye.h"
#include "ekf_coder_emxutil.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
static real_T rt_roundd_snf(real_T u);

/* Function Definitions */
static real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

void b_eye(real_T I[9])
{
  int32_T i;
  memset(&I[0], 0, 9U * sizeof(real_T));
  for (i = 0; i < 3; i++) {
    I[i + 3 * i] = 1.0;
  }
}

void c_eye(real_T I[9])
{
  int32_T i;
  memset(&I[0], 0, 9U * sizeof(real_T));
  for (i = 0; i < 3; i++) {
    I[i + 3 * i] = 1.0;
  }
}

void eye(real_T n, emxArray_real_T *I)
{
  int32_T q;
  int32_T loop_ub;
  real_T d1;
  q = I->size[0] * I->size[1];
  I->size[0] = (int32_T)n;
  I->size[1] = (int32_T)n;
  emxEnsureCapacity((emxArray__common *)I, q, (int32_T)sizeof(real_T));
  loop_ub = (int32_T)n * (int32_T)n - 1;
  for (q = 0; q <= loop_ub; q++) {
    I->data[q] = 0.0;
  }

  d1 = rt_roundd_snf(n);
  if (d1 < 2.147483648E+9) {
    if (d1 >= -2.147483648E+9) {
      q = (int32_T)d1;
    } else {
      q = MIN_int32_T;
    }
  } else if (d1 >= 2.147483648E+9) {
    q = MAX_int32_T;
  } else {
    q = 0;
  }

  if (q > 0) {
    for (loop_ub = 0; loop_ub + 1 <= q; loop_ub++) {
      I->data[loop_ub + I->size[0] * loop_ub] = 1.0;
    }
  }
}

/* End of code generation (eye.cpp) */
