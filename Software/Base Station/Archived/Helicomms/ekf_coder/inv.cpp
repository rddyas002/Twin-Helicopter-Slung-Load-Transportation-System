/*
 * inv.cpp
 *
 * Code generation for function 'inv'
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
#include "inv.h"
#include "ekf_coder_emxutil.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void invNxN(const emxArray_real_T *x, emxArray_real_T *y)
{
  int32_T n;
  int32_T i5;
  int32_T jj;
  int32_T yk;
  int32_T b_n;
  emxArray_real_T *A;
  emxArray_int32_T *ipiv;
  int32_T jA;
  int32_T u0;
  int32_T j;
  int32_T mmj;
  int32_T jp1j;
  int32_T c;
  int32_T ix;
  real_T smax;
  real_T s;
  emxArray_int32_T *p;
  n = x->size[0];
  for (i5 = 0; i5 < 2; i5++) {
    jj = y->size[0] * y->size[1];
    y->size[i5] = x->size[i5];
    emxEnsureCapacity((emxArray__common *)y, jj, (int32_T)sizeof(real_T));
  }

  i5 = y->size[0] * y->size[1];
  y->size[0] = y->size[0];
  y->size[1] = y->size[1];
  emxEnsureCapacity((emxArray__common *)y, i5, (int32_T)sizeof(real_T));
  yk = y->size[1] - 1;
  for (i5 = 0; i5 <= yk; i5++) {
    b_n = y->size[0] - 1;
    for (jj = 0; jj <= b_n; jj++) {
      y->data[jj + y->size[0] * i5] = 0.0;
    }
  }

  emxInit_real_T(&A, 2);
  i5 = A->size[0] * A->size[1];
  A->size[0] = x->size[0];
  A->size[1] = x->size[1];
  emxEnsureCapacity((emxArray__common *)A, i5, (int32_T)sizeof(real_T));
  yk = x->size[0] * x->size[1] - 1;
  for (i5 = 0; i5 <= yk; i5++) {
    A->data[i5] = x->data[i5];
  }

  if (n < 1) {
    b_n = 0;
  } else {
    b_n = n;
  }

  emxInit_int32_T(&ipiv, 2);
  i5 = ipiv->size[0] * ipiv->size[1];
  ipiv->size[0] = 1;
  ipiv->size[1] = b_n;
  emxEnsureCapacity((emxArray__common *)ipiv, i5, (int32_T)sizeof(int32_T));
  if (b_n > 0) {
    ipiv->data[0] = 1;
    yk = 1;
    for (jA = 2; jA <= b_n; jA++) {
      yk++;
      ipiv->data[jA - 1] = yk;
    }
  }

  if (n < 1) {
  } else {
    u0 = n - 1;
    if (u0 <= n) {
    } else {
      u0 = n;
    }

    for (j = 1; j <= u0; j++) {
      mmj = n - j;
      jj = (j - 1) * (n + 1);
      jp1j = jj + 2;
      c = mmj + 1;
      if (c < 1) {
        yk = -1;
      } else {
        yk = 0;
        if (c > 1) {
          ix = jj;
          smax = fabs(A->data[jj]);
          for (jA = 1; jA + 1 <= c; jA++) {
            ix++;
            s = fabs(A->data[ix]);
            if (s > smax) {
              yk = jA;
              smax = s;
            }
          }
        }
      }

      if (A->data[jj + yk] != 0.0) {
        if (yk != 0) {
          ipiv->data[j - 1] = j + yk;
          ix = j - 1;
          yk = (j + yk) - 1;
          for (jA = 1; jA <= n; jA++) {
            smax = A->data[ix];
            A->data[ix] = A->data[yk];
            A->data[yk] = smax;
            ix += n;
            yk += n;
          }
        }

        i5 = (jp1j + mmj) - 1;
        for (b_n = jp1j; b_n <= i5; b_n++) {
          A->data[b_n - 1] /= A->data[jj];
        }
      }

      c = n - j;
      jA = (jj + n) + 1;
      yk = jj + n;
      for (b_n = 1; b_n <= c; b_n++) {
        smax = A->data[yk];
        if (A->data[yk] != 0.0) {
          ix = jp1j;
          i5 = mmj + jA;
          for (jj = jA; jj + 1 <= i5; jj++) {
            A->data[jj] += A->data[ix - 1] * -smax;
            ix++;
          }
        }

        yk += n;
        jA += n;
      }
    }
  }

  if (n < 1) {
    b_n = 0;
  } else {
    b_n = n;
  }

  emxInit_int32_T(&p, 2);
  i5 = p->size[0] * p->size[1];
  p->size[0] = 1;
  p->size[1] = b_n;
  emxEnsureCapacity((emxArray__common *)p, i5, (int32_T)sizeof(int32_T));
  if (b_n > 0) {
    p->data[0] = 1;
    yk = 1;
    for (jA = 2; jA <= b_n; jA++) {
      yk++;
      p->data[jA - 1] = yk;
    }
  }

  for (jA = 0; jA <= ipiv->size[1] - 1; jA++) {
    if (ipiv->data[jA] > 1 + jA) {
      yk = p->data[ipiv->data[jA] - 1];
      p->data[ipiv->data[jA] - 1] = p->data[jA];
      p->data[jA] = yk;
    }
  }

  emxFree_int32_T(&ipiv);
  for (jA = 0; jA + 1 <= n; jA++) {
    y->data[jA + y->size[0] * (p->data[jA] - 1)] = 1.0;
    for (j = jA; j + 1 <= n; j++) {
      if (y->data[j + y->size[0] * (p->data[jA] - 1)] != 0.0) {
        for (b_n = j + 1; b_n + 1 <= n; b_n++) {
          y->data[b_n + y->size[0] * (p->data[jA] - 1)] -= y->data[j + y->size[0]
            * (p->data[jA] - 1)] * A->data[b_n + A->size[0] * j];
        }
      }
    }
  }

  emxFree_int32_T(&p);
  if ((n == 0) || ((y->size[0] == 0) || (y->size[1] == 0))) {
  } else {
    for (j = 1; j <= n; j++) {
      c = n * (j - 1) - 1;
      for (jA = n; jA > 0; jA--) {
        yk = n * (jA - 1) - 1;
        if (y->data[jA + c] != 0.0) {
          y->data[jA + c] /= A->data[jA + yk];
          for (b_n = 1; b_n <= jA - 1; b_n++) {
            y->data[b_n + c] -= y->data[jA + c] * A->data[b_n + yk];
          }
        }
      }
    }
  }

  emxFree_real_T(&A);
}

/* End of code generation (inv.cpp) */
