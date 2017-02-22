/*
 * inv.cpp
 *
 * Code generation for function 'inv'
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
#include "inv.h"
#include "ekf_coder_emxutil.h"

/* Function Definitions */
void inv(const real_T x[49], real_T y[49])
{
  real_T A[49];
  int32_T i21;
  int8_T ipiv[7];
  int32_T j;
  int32_T c;
  int32_T pipk;
  int32_T ix;
  real_T smax;
  int32_T k;
  real_T s;
  int32_T jy;
  int32_T ijA;
  int8_T p[7];
  for (i21 = 0; i21 < 49; i21++) {
    y[i21] = 0.0;
    A[i21] = x[i21];
  }

  for (i21 = 0; i21 < 7; i21++) {
    ipiv[i21] = (int8_T)(1 + i21);
  }

  for (j = 0; j < 6; j++) {
    c = j << 3;
    pipk = 0;
    ix = c;
    smax = fabs(A[c]);
    for (k = 2; k <= 7 - j; k++) {
      ix++;
      s = fabs(A[ix]);
      if (s > smax) {
        pipk = k - 1;
        smax = s;
      }
    }

    if (A[c + pipk] != 0.0) {
      if (pipk != 0) {
        ipiv[j] = (int8_T)((j + pipk) + 1);
        ix = j;
        pipk += j;
        for (k = 0; k < 7; k++) {
          smax = A[ix];
          A[ix] = A[pipk];
          A[pipk] = smax;
          ix += 7;
          pipk += 7;
        }
      }

      i21 = (c - j) + 7;
      for (jy = c + 1; jy + 1 <= i21; jy++) {
        A[jy] /= A[c];
      }
    }

    pipk = c;
    jy = c + 7;
    for (k = 1; k <= 6 - j; k++) {
      smax = A[jy];
      if (A[jy] != 0.0) {
        ix = c + 1;
        i21 = (pipk - j) + 14;
        for (ijA = 8 + pipk; ijA + 1 <= i21; ijA++) {
          A[ijA] += A[ix] * -smax;
          ix++;
        }
      }

      jy += 7;
      pipk += 7;
    }
  }

  for (i21 = 0; i21 < 7; i21++) {
    p[i21] = (int8_T)(1 + i21);
  }

  for (k = 0; k < 6; k++) {
    if (ipiv[k] > 1 + k) {
      pipk = p[ipiv[k] - 1];
      p[ipiv[k] - 1] = p[k];
      p[k] = (int8_T)pipk;
    }
  }

  for (k = 0; k < 7; k++) {
    y[k + 7 * (p[k] - 1)] = 1.0;
    for (j = k; j + 1 < 8; j++) {
      if (y[j + 7 * (p[k] - 1)] != 0.0) {
        for (jy = j + 1; jy + 1 < 8; jy++) {
          y[jy + 7 * (p[k] - 1)] -= y[j + 7 * (p[k] - 1)] * A[jy + 7 * j];
        }
      }
    }
  }

  for (j = 0; j < 7; j++) {
    c = 7 * j;
    for (k = 6; k > -1; k += -1) {
      pipk = 7 * k;
      if (y[k + c] != 0.0) {
        y[k + c] /= A[k + pipk];
        for (jy = 0; jy + 1 <= k; jy++) {
          y[jy + c] -= y[k + c] * A[jy + pipk];
        }
      }
    }
  }
}

void invNxN(const emxArray_real_T *x, emxArray_real_T *y)
{
  int32_T i5;
  int32_T n;
  emxArray_real_T *A;
  int32_T yk;
  emxArray_int32_T *ipiv;
  int32_T k;
  int32_T j;
  int32_T mmj;
  int32_T c;
  int32_T ix;
  real_T smax;
  real_T s;
  int32_T i6;
  int32_T b_c;
  int32_T ijA;
  emxArray_int32_T *p;
  i5 = y->size[0] * y->size[1];
  y->size[0] = x->size[0];
  y->size[1] = x->size[1];
  emxEnsureCapacity((emxArray__common *)y, i5, (int32_T)sizeof(real_T));
  n = x->size[0] * x->size[1];
  for (i5 = 0; i5 < n; i5++) {
    y->data[i5] = 0.0;
  }

  emxInit_real_T(&A, 2);
  i5 = A->size[0] * A->size[1];
  A->size[0] = x->size[0];
  A->size[1] = x->size[1];
  emxEnsureCapacity((emxArray__common *)A, i5, (int32_T)sizeof(real_T));
  n = x->size[0] * x->size[1];
  for (i5 = 0; i5 < n; i5++) {
    A->data[i5] = x->data[i5];
  }

  n = x->size[0];
  yk = x->size[0];
  if (n <= yk) {
  } else {
    n = yk;
  }

  if (n < 1) {
    n = 0;
  }

  emxInit_int32_T(&ipiv, 2);
  i5 = ipiv->size[0] * ipiv->size[1];
  ipiv->size[0] = 1;
  ipiv->size[1] = n;
  emxEnsureCapacity((emxArray__common *)ipiv, i5, (int32_T)sizeof(int32_T));
  if (n > 0) {
    ipiv->data[0] = 1;
    yk = 1;
    for (k = 2; k <= n; k++) {
      yk++;
      ipiv->data[k - 1] = yk;
    }
  }

  if ((x->size[0] < 1) || (x->size[0] < 1)) {
  } else {
    n = x->size[0] - 1;
    yk = x->size[0];
    if (n <= yk) {
      i5 = n;
    } else {
      i5 = yk;
    }

    for (j = 0; j + 1 <= i5; j++) {
      mmj = x->size[0] - j;
      c = j * (x->size[0] + 1);
      if (mmj < 1) {
        yk = -1;
      } else {
        yk = 0;
        if (mmj > 1) {
          ix = c;
          smax = fabs(A->data[c]);
          for (k = 2; k <= mmj; k++) {
            ix++;
            s = fabs(A->data[ix]);
            if (s > smax) {
              yk = k - 1;
              smax = s;
            }
          }
        }
      }

      if (A->data[c + yk] != 0.0) {
        if (yk != 0) {
          ipiv->data[j] = (j + yk) + 1;
          ix = j;
          yk += j;
          for (k = 1; k <= x->size[0]; k++) {
            smax = A->data[ix];
            A->data[ix] = A->data[yk];
            A->data[yk] = smax;
            ix += x->size[0];
            yk += x->size[0];
          }
        }

        i6 = c + mmj;
        for (yk = c + 1; yk + 1 <= i6; yk++) {
          A->data[yk] /= A->data[c];
        }
      }

      b_c = x->size[0] - j;
      yk = (c + x->size[0]) + 1;
      n = c + x->size[0];
      for (k = 1; k < b_c; k++) {
        smax = A->data[n];
        if (A->data[n] != 0.0) {
          ix = c + 1;
          i6 = mmj + yk;
          for (ijA = yk; ijA + 1 < i6; ijA++) {
            A->data[ijA] += A->data[ix] * -smax;
            ix++;
          }
        }

        n += x->size[0];
        yk += x->size[0];
      }
    }
  }

  if (x->size[0] < 1) {
    n = 0;
  } else {
    n = x->size[0];
  }

  emxInit_int32_T(&p, 2);
  i5 = p->size[0] * p->size[1];
  p->size[0] = 1;
  p->size[1] = n;
  emxEnsureCapacity((emxArray__common *)p, i5, (int32_T)sizeof(int32_T));
  if (n > 0) {
    p->data[0] = 1;
    yk = 1;
    for (k = 2; k <= n; k++) {
      yk++;
      p->data[k - 1] = yk;
    }
  }

  for (k = 0; k < ipiv->size[1]; k++) {
    if (ipiv->data[k] > 1 + k) {
      n = p->data[ipiv->data[k] - 1];
      p->data[ipiv->data[k] - 1] = p->data[k];
      p->data[k] = n;
    }
  }

  emxFree_int32_T(&ipiv);
  for (k = 0; k + 1 <= x->size[0]; k++) {
    y->data[k + y->size[0] * (p->data[k] - 1)] = 1.0;
    for (j = k; j + 1 <= x->size[0]; j++) {
      if (y->data[j + y->size[0] * (p->data[k] - 1)] != 0.0) {
        for (yk = j + 1; yk + 1 <= x->size[0]; yk++) {
          y->data[yk + y->size[0] * (p->data[k] - 1)] -= y->data[j + y->size[0] *
            (p->data[k] - 1)] * A->data[yk + A->size[0] * j];
        }
      }
    }
  }

  emxFree_int32_T(&p);
  if ((x->size[0] == 0) || ((y->size[0] == 0) || (y->size[1] == 0))) {
  } else {
    for (j = 1; j <= x->size[0]; j++) {
      c = x->size[0] * (j - 1);
      for (k = x->size[0] - 1; k + 1 > 0; k--) {
        b_c = x->size[0] * k;
        if (y->data[k + c] != 0.0) {
          y->data[k + c] /= A->data[k + b_c];
          for (yk = 1; yk <= k; yk++) {
            y->data[(yk + c) - 1] -= y->data[k + c] * A->data[(yk + b_c) - 1];
          }
        }
      }
    }
  }

  emxFree_real_T(&A);
}

/* End of code generation (inv.cpp) */
