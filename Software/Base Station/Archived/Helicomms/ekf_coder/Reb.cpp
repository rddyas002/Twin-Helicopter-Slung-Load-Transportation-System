/*
 * Reb.cpp
 *
 * Code generation for function 'Reb'
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

/* Function Definitions */
void Reb(const real_T q[4], const real_T x[3], real_T y[3])
{
  real_T dv15[9];
  int32_T i11;
  int32_T i12;
  dv15[0] = ((rt_powd_snf(q[0], 2.0) + rt_powd_snf(q[1], 2.0)) - rt_powd_snf(q[2],
              2.0)) - rt_powd_snf(q[3], 2.0);
  dv15[3] = 2.0 * (q[1] * q[2] - q[0] * q[3]);
  dv15[6] = 2.0 * (q[1] * q[3] + q[0] * q[2]);
  dv15[1] = 2.0 * (q[1] * q[2] + q[0] * q[3]);
  dv15[4] = ((rt_powd_snf(q[0], 2.0) - rt_powd_snf(q[1], 2.0)) + rt_powd_snf(q[2],
              2.0)) - rt_powd_snf(q[3], 2.0);
  dv15[7] = 2.0 * (q[2] * q[3] - q[0] * q[1]);
  dv15[2] = 2.0 * (q[1] * q[3] - q[0] * q[2]);
  dv15[5] = 2.0 * (q[2] * q[3] + q[0] * q[1]);
  dv15[8] = ((rt_powd_snf(q[0], 2.0) - rt_powd_snf(q[1], 2.0)) - rt_powd_snf(q[2],
              2.0)) + rt_powd_snf(q[3], 2.0);
  for (i11 = 0; i11 < 3; i11++) {
    y[i11] = 0.0;
    for (i12 = 0; i12 < 3; i12++) {
      y[i11] += dv15[i11 + 3 * i12] * x[i12];
    }
  }
}

/* End of code generation (Reb.cpp) */
