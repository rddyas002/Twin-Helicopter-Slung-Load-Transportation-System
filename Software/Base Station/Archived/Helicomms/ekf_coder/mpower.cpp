/*
 * mpower.cpp
 *
 * Code generation for function 'mpower'
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
#include "mpower.h"
#include "ekf_coder_emxutil.h"
#include "inv.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void mpower(const emxArray_real_T *a, emxArray_real_T *c)
{
  uint32_T uv0[2];
  int32_T i3;
  emxArray_real_T *b_a;
  int32_T a_idx_0;
  int32_T b_c[2];
  int32_T loop_ub;
  int32_T i4;
  emxArray_real_T *x;
  emxArray_real_T *c_c;
  for (i3 = 0; i3 < 2; i3++) {
    uv0[i3] = (uint32_T)a->size[i3];
  }

  b_emxInit_real_T(&b_a, 1);
  i3 = c->size[0] * c->size[1];
  c->size[0] = (int32_T)uv0[0];
  c->size[1] = (int32_T)uv0[1];
  emxEnsureCapacity((emxArray__common *)c, i3, (int32_T)sizeof(real_T));
  a_idx_0 = a->size[0] * a->size[1];
  i3 = b_a->size[0];
  b_a->size[0] = a_idx_0;
  emxEnsureCapacity((emxArray__common *)b_a, i3, (int32_T)sizeof(real_T));
  a_idx_0--;
  for (i3 = 0; i3 <= a_idx_0; i3++) {
    b_a->data[i3] = a->data[i3];
  }

  for (i3 = 0; i3 < 2; i3++) {
    b_c[i3] = c->size[i3];
  }

  i3 = c->size[0] * c->size[1];
  c->size[0] = b_c[0];
  c->size[1] = b_c[1];
  emxEnsureCapacity((emxArray__common *)c, i3, (int32_T)sizeof(real_T));
  a_idx_0 = b_c[1] - 1;
  for (i3 = 0; i3 <= a_idx_0; i3++) {
    loop_ub = b_c[0] - 1;
    for (i4 = 0; i4 <= loop_ub; i4++) {
      c->data[i4 + c->size[0] * i3] = b_a->data[i4 + b_c[0] * i3];
    }
  }

  emxFree_real_T(&b_a);
  emxInit_real_T(&x, 2);
  i3 = x->size[0] * x->size[1];
  x->size[0] = c->size[0];
  x->size[1] = c->size[1];
  emxEnsureCapacity((emxArray__common *)x, i3, (int32_T)sizeof(real_T));
  a_idx_0 = c->size[0] * c->size[1] - 1;
  for (i3 = 0; i3 <= a_idx_0; i3++) {
    x->data[i3] = c->data[i3];
  }

  emxInit_real_T(&c_c, 2);
  i3 = c_c->size[0] * c_c->size[1];
  c_c->size[0] = c->size[0];
  c_c->size[1] = c->size[1];
  emxEnsureCapacity((emxArray__common *)c_c, i3, (int32_T)sizeof(real_T));
  a_idx_0 = c->size[0] * c->size[1] - 1;
  for (i3 = 0; i3 <= a_idx_0; i3++) {
    c_c->data[i3] = c->data[i3];
  }

  invNxN(c_c, c);
  emxFree_real_T(&c_c);
  emxFree_real_T(&x);
}

/* End of code generation (mpower.cpp) */
