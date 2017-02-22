/*
 * eye.cpp
 *
 * Code generation for function 'eye'
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
#include "eye.h"
#include "ekf_coder_emxutil.h"

/* Function Definitions */
void b_eye(real_T I[9])
{
  int32_T i;
  memset(&I[0], 0, 9U * sizeof(real_T));
  for (i = 0; i < 3; i++) {
    I[i + 3 * i] = 1.0;
  }
}

void eye(real_T n, emxArray_real_T *I)
{
  int32_T i;
  int32_T loop_ub;
  real_T minval;
  i = I->size[0] * I->size[1];
  I->size[0] = (int32_T)n;
  I->size[1] = (int32_T)n;
  emxEnsureCapacity((emxArray__common *)I, i, (int32_T)sizeof(real_T));
  loop_ub = (int32_T)n * (int32_T)n;
  for (i = 0; i < loop_ub; i++) {
    I->data[i] = 0.0;
  }

  if ((n <= n) || rtIsNaN(n)) {
    minval = n;
  } else {
    minval = n;
  }

  if ((int32_T)minval > 0) {
    for (i = 0; i + 1 <= (int32_T)minval; i++) {
      I->data[i + I->size[0] * i] = 1.0;
    }
  }
}

/* End of code generation (eye.cpp) */
