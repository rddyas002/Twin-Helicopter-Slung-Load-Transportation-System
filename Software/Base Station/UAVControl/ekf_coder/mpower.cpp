/*
 * mpower.cpp
 *
 * Code generation for function 'mpower'
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
#include "mpower.h"
#include "ekf_coder_emxutil.h"
#include "inv.h"

/* Function Definitions */
void mpower(const emxArray_real_T *a, emxArray_real_T *c)
{
  uint32_T uv0[2];
  int32_T i4;
  int32_T loop_ub;
  emxArray_real_T *x;
  emxArray_real_T *b_c;
  for (i4 = 0; i4 < 2; i4++) {
    uv0[i4] = (uint32_T)a->size[i4];
  }

  i4 = c->size[0] * c->size[1];
  c->size[0] = (int32_T)uv0[0];
  emxEnsureCapacity((emxArray__common *)c, i4, (int32_T)sizeof(real_T));
  i4 = c->size[0] * c->size[1];
  c->size[1] = (int32_T)uv0[1];
  emxEnsureCapacity((emxArray__common *)c, i4, (int32_T)sizeof(real_T));
  loop_ub = (int32_T)uv0[0] * (int32_T)uv0[1];
  for (i4 = 0; i4 < loop_ub; i4++) {
    c->data[i4] = a->data[i4];
  }

  emxInit_real_T(&x, 2);
  i4 = x->size[0] * x->size[1];
  x->size[0] = c->size[0];
  x->size[1] = c->size[1];
  emxEnsureCapacity((emxArray__common *)x, i4, (int32_T)sizeof(real_T));
  loop_ub = c->size[0] * c->size[1];
  for (i4 = 0; i4 < loop_ub; i4++) {
    x->data[i4] = c->data[i4];
  }

  emxInit_real_T(&b_c, 2);
  i4 = b_c->size[0] * b_c->size[1];
  b_c->size[0] = c->size[0];
  b_c->size[1] = c->size[1];
  emxEnsureCapacity((emxArray__common *)b_c, i4, (int32_T)sizeof(real_T));
  loop_ub = c->size[0] * c->size[1];
  for (i4 = 0; i4 < loop_ub; i4++) {
    b_c->data[i4] = c->data[i4];
  }

  invNxN(b_c, c);
  emxFree_real_T(&b_c);
  emxFree_real_T(&x);
}

/* End of code generation (mpower.cpp) */
