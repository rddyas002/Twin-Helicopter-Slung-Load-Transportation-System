/*
 * solvePoseEst.cpp
 *
 * Code generation for function 'solvePoseEst'
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
#include "ekf_coder_emxutil.h"
#include "inv.h"

/* Function Declarations */
static void b_project2_2D(const real_T A[9], const real_T Rcw[9], const real_T
  Tcw[3], const real_T Twp[3], const real_T qwp[4], const real_T Po[3], real_T
  x[2]);
static void solveLSQ(const emxArray_real_T *A, const emxArray_real_T *b, const
                     real_T x0[7], real_T x[7], real_T *norm_err, real_T
                     delta_x_plus_x[7]);

/* Function Definitions */
static void b_project2_2D(const real_T A[9], const real_T Rcw[9], const real_T
  Tcw[3], const real_T Twp[3], const real_T qwp[4], const real_T Po[3], real_T
  x[2])
{
  real_T b_qwp[9];
  real_T b_A[9];
  int32_T i17;
  int32_T i18;
  int32_T i19;
  real_T c_qwp[3];
  real_T d1;
  real_T image_homog[3];
  b_qwp[0] = ((qwp[0] * qwp[0] + qwp[1] * qwp[1]) - qwp[2] * qwp[2]) - qwp[3] *
    qwp[3];
  b_qwp[3] = 2.0 * (qwp[1] * qwp[2] - qwp[0] * qwp[3]);
  b_qwp[6] = 2.0 * (qwp[1] * qwp[3] + qwp[0] * qwp[2]);
  b_qwp[1] = 2.0 * (qwp[1] * qwp[2] + qwp[0] * qwp[3]);
  b_qwp[4] = ((qwp[0] * qwp[0] - qwp[1] * qwp[1]) + qwp[2] * qwp[2]) - qwp[3] *
    qwp[3];
  b_qwp[7] = 2.0 * (qwp[2] * qwp[3] - qwp[0] * qwp[1]);
  b_qwp[2] = 2.0 * (qwp[1] * qwp[3] - qwp[0] * qwp[2]);
  b_qwp[5] = 2.0 * (qwp[2] * qwp[3] + qwp[0] * qwp[1]);
  b_qwp[8] = ((qwp[0] * qwp[0] - qwp[1] * qwp[1]) - qwp[2] * qwp[2]) + qwp[3] *
    qwp[3];
  for (i17 = 0; i17 < 3; i17++) {
    for (i18 = 0; i18 < 3; i18++) {
      b_A[i17 + 3 * i18] = 0.0;
      for (i19 = 0; i19 < 3; i19++) {
        b_A[i17 + 3 * i18] += A[i17 + 3 * i19] * Rcw[i19 + 3 * i18];
      }
    }
  }

  for (i17 = 0; i17 < 3; i17++) {
    d1 = 0.0;
    for (i18 = 0; i18 < 3; i18++) {
      d1 += b_qwp[i17 + 3 * i18] * Po[i18];
    }

    c_qwp[i17] = (d1 + Twp[i17]) - Tcw[i17];
  }

  for (i17 = 0; i17 < 3; i17++) {
    image_homog[i17] = 0.0;
    for (i18 = 0; i18 < 3; i18++) {
      image_homog[i17] += b_A[i17 + 3 * i18] * c_qwp[i18];
    }
  }

  x[0] = image_homog[0] / image_homog[2];
  x[1] = image_homog[1] / image_homog[2];
}

static void solveLSQ(const emxArray_real_T *A, const emxArray_real_T *b, const
                     real_T x0[7], real_T x[7], real_T *norm_err, real_T
                     delta_x_plus_x[7])
{
  emxArray_real_T *a;
  int32_T i20;
  int32_T i;
  int32_T cr;
  real_T y[49];
  int32_T br;
  int32_T ic;
  int32_T ar;
  int32_T ib;
  int32_T ia;
  real_T c[49];
  real_T b_c[49];
  emxArray_real_T *b_y;
  uint32_T unnamed_idx_1;
  real_T b_x;
  emxArray_real_T *c_y;
  emxInit_real_T(&a, 2);
  i20 = a->size[0] * a->size[1];
  a->size[0] = 7;
  a->size[1] = A->size[0];
  emxEnsureCapacity((emxArray__common *)a, i20, (int32_T)sizeof(real_T));
  i = A->size[0];
  for (i20 = 0; i20 < i; i20++) {
    for (cr = 0; cr < 7; cr++) {
      a->data[cr + a->size[0] * i20] = A->data[i20 + A->size[0] * cr];
    }
  }

  if ((a->size[1] == 1) || (A->size[0] == 1)) {
    for (i20 = 0; i20 < 7; i20++) {
      for (cr = 0; cr < 7; cr++) {
        y[i20 + 7 * cr] = 0.0;
        i = a->size[1];
        for (br = 0; br < i; br++) {
          y[i20 + 7 * cr] += a->data[i20 + a->size[0] * br] * A->data[br +
            A->size[0] * cr];
        }
      }
    }
  } else {
    memset(&y[0], 0, 49U * sizeof(real_T));
    for (cr = 0; cr < 44; cr += 7) {
      for (ic = cr; ic + 1 <= cr + 7; ic++) {
        y[ic] = 0.0;
      }
    }

    br = 0;
    for (cr = 0; cr < 44; cr += 7) {
      ar = 0;
      i20 = br + a->size[1];
      for (ib = br; ib + 1 <= i20; ib++) {
        if (A->data[ib] != 0.0) {
          ia = ar;
          for (ic = cr; ic + 1 <= cr + 7; ic++) {
            ia++;
            y[ic] += A->data[ib] * a->data[ia - 1];
          }
        }

        ar += 7;
      }

      br += a->size[1];
    }
  }

  memcpy(&c[0], &y[0], 49U * sizeof(real_T));
  memcpy(&b_c[0], &c[0], 49U * sizeof(real_T));
  inv(b_c, c);
  i20 = a->size[0] * a->size[1];
  a->size[0] = 7;
  a->size[1] = A->size[0];
  emxEnsureCapacity((emxArray__common *)a, i20, (int32_T)sizeof(real_T));
  i = A->size[0];
  for (i20 = 0; i20 < i; i20++) {
    for (cr = 0; cr < 7; cr++) {
      a->data[cr + a->size[0] * i20] = A->data[i20 + A->size[0] * cr];
    }
  }

  emxInit_real_T(&b_y, 2);
  unnamed_idx_1 = (uint32_T)a->size[1];
  i20 = b_y->size[0] * b_y->size[1];
  b_y->size[0] = 7;
  emxEnsureCapacity((emxArray__common *)b_y, i20, (int32_T)sizeof(real_T));
  i20 = b_y->size[0] * b_y->size[1];
  b_y->size[1] = (int32_T)unnamed_idx_1;
  emxEnsureCapacity((emxArray__common *)b_y, i20, (int32_T)sizeof(real_T));
  i = 7 * (int32_T)unnamed_idx_1;
  for (i20 = 0; i20 < i; i20++) {
    b_y->data[i20] = 0.0;
  }

  if (a->size[1] == 0) {
  } else {
    i = 7 * (a->size[1] - 1);
    for (cr = 0; cr <= i; cr += 7) {
      for (ic = cr + 1; ic <= cr + 7; ic++) {
        b_y->data[ic - 1] = 0.0;
      }
    }

    br = 0;
    for (cr = 0; cr <= i; cr += 7) {
      ar = 0;
      for (ib = br; ib + 1 <= br + 7; ib++) {
        if (a->data[ib] != 0.0) {
          ia = ar;
          for (ic = cr; ic + 1 <= cr + 7; ic++) {
            ia++;
            b_y->data[ic] += a->data[ib] * c[ia - 1];
          }
        }

        ar += 7;
      }

      br += 7;
    }
  }

  emxFree_real_T(&a);
  if ((b_y->size[1] == 1) || (b->size[0] == 1)) {
    for (i20 = 0; i20 < 7; i20++) {
      x[i20] = 0.0;
      i = b_y->size[1];
      for (cr = 0; cr < i; cr++) {
        b_x = x[i20] + b_y->data[i20 + b_y->size[0] * cr] * b->data[cr];
        x[i20] = b_x;
      }
    }
  } else {
    for (ic = 0; ic < 7; ic++) {
      x[ic] = 0.0;
    }

    ar = 0;
    for (ib = 0; ib + 1 <= b_y->size[1]; ib++) {
      if (b->data[ib] != 0.0) {
        ia = ar;
        for (ic = 0; ic < 7; ic++) {
          ia++;
          b_x = x[ic] + b->data[ib] * b_y->data[ia - 1];
          x[ic] = b_x;
        }
      }

      ar += 7;
    }
  }

  emxFree_real_T(&b_y);
  b_emxInit_real_T(&c_y, 1);

  /*  show the sum of the squares of the pixel error  */
  unnamed_idx_1 = (uint32_T)b->size[0];
  i20 = c_y->size[0];
  c_y->size[0] = (int32_T)unnamed_idx_1;
  emxEnsureCapacity((emxArray__common *)c_y, i20, (int32_T)sizeof(real_T));
  for (i = 0; i < (int32_T)unnamed_idx_1; i++) {
    c_y->data[i] = b->data[i] * b->data[i];
  }

  if (c_y->size[0] == 0) {
    *norm_err = 0.0;
  } else {
    *norm_err = c_y->data[0];
    for (i = 2; i <= c_y->size[0]; i++) {
      *norm_err += c_y->data[i - 1];
    }
  }

  emxFree_real_T(&c_y);
  for (i = 0; i < 7; i++) {
    delta_x_plus_x[i] = x[i] + x0[i];
  }
}

void solvePoseEst(const real_T x[7], const b_struct_T z[4], const real_T
                  object_points[9], real_T x_est[7], real_T *resnorm)
{
  real_T num_measurements;
  int32_T i;
  emxArray_real_T *A_lin;
  int32_T i14;
  emxArray_real_T *z_est;
  emxArray_real_T *z_meas;
  uint32_T k;
  real_T A[9];
  real_T Tcw[3];
  real_T b_object_points[3];
  real_T dv11[2];
  int32_T i15;
  real_T K[9];
  int32_T i16;
  real_T X;
  real_T Y;
  real_T Z;
  real_T B2;
  real_T B3;
  real_T b;
  real_T au;
  real_T dB1_x[7];
  real_T dB2_x[7];
  real_T dB3_x[7];
  real_T b_prime[7];
  emxArray_real_T *b_z_meas;
  num_measurements = 0.0;

  /*  only allow single blob detection for now */
  for (i = 0; i < 4; i++) {
    if (z[i].number_red == 1.0) {
      num_measurements += 2.0;
    }

    if (z[i].number_green == 1.0) {
      num_measurements += 2.0;
    }

    if (z[i].number_blue == 1.0) {
      num_measurements += 2.0;
    }
  }

  emxInit_real_T(&A_lin, 2);
  i14 = A_lin->size[0] * A_lin->size[1];
  A_lin->size[0] = (int32_T)num_measurements;
  A_lin->size[1] = 7;
  emxEnsureCapacity((emxArray__common *)A_lin, i14, (int32_T)sizeof(real_T));
  i = (int32_T)num_measurements * 7;
  for (i14 = 0; i14 < i; i14++) {
    A_lin->data[i14] = 0.0;
  }

  b_emxInit_real_T(&z_est, 1);
  i14 = z_est->size[0];
  z_est->size[0] = (int32_T)num_measurements;
  emxEnsureCapacity((emxArray__common *)z_est, i14, (int32_T)sizeof(real_T));
  i = (int32_T)num_measurements;
  for (i14 = 0; i14 < i; i14++) {
    z_est->data[i14] = 0.0;
  }

  b_emxInit_real_T(&z_meas, 1);
  i14 = z_meas->size[0];
  z_meas->size[0] = (int32_T)num_measurements;
  emxEnsureCapacity((emxArray__common *)z_meas, i14, (int32_T)sizeof(real_T));
  i = (int32_T)num_measurements;
  for (i14 = 0; i14 < i; i14++) {
    z_meas->data[i14] = 0.0;
  }

  k = 1U;
  for (i = 0; i < 4; i++) {
    for (i14 = 0; i14 < 9; i14++) {
      A[i14] = z[i].cam_param.intrinsic_mat[i14] * 1000.0;
    }

    /*  convert mapping from pixel/mm to pixel/m */
    for (i14 = 0; i14 < 3; i14++) {
      Tcw[i14] = z[i].cam_param.translation_w2c[i14] / 1000.0;
    }

    /*  convert mapping from pixel/mm to pixel/m */
    /*  red blobs */
    if (z[i].number_red == 1.0) {
      /*  do measurement estimation */
      for (i14 = 0; i14 < 3; i14++) {
        b_object_points[i14] = object_points[i14] / 1000.0;
      }

      b_project2_2D(A, z[i].cam_param.rotation_w2c, Tcw, *(real_T (*)[3])&x[0], *
                    (real_T (*)[4])&x[3], b_object_points, dv11);
      for (i14 = 0; i14 < 2; i14++) {
        z_est->data[(int32_T)((real_T)k + (real_T)i14) - 1] = dv11[i14];
      }

      /*  do jacobian calculation */
      for (i14 = 0; i14 < 3; i14++) {
        for (i15 = 0; i15 < 3; i15++) {
          K[i14 + 3 * i15] = 0.0;
          for (i16 = 0; i16 < 3; i16++) {
            K[i14 + 3 * i15] += A[i14 + 3 * i16] * z[i]
              .cam_param.rotation_w2c[i16 + 3 * i15];
          }
        }
      }

      X = object_points[0] / 1000.0;
      Y = object_points[1] / 1000.0;
      Z = object_points[2] / 1000.0;

      /*  calculate B's */
      num_measurements = ((((((x[3] * x[3] + x[4] * x[4]) - x[5] * x[5]) - x[6] *
        x[6]) * X + 2.0 * (x[4] * x[5] - x[3] * x[6]) * Y) + 2.0 * (x[4] * x[6]
        + x[3] * x[5]) * Z) + x[0]) - z[i].cam_param.translation_w2c[0] / 1000.0;

      /*  convert mapping from pixel/mm to pixel/m */
      B2 = (((2.0 * (x[4] * x[5] + x[3] * x[6]) * X + (((x[3] * x[3] - x[4] * x
                 [4]) + x[5] * x[5]) - x[6] * x[6]) * Y) + 2.0 * (x[5] * x[6] -
              x[3] * x[4]) * Z) + x[1]) - z[i].cam_param.translation_w2c[1] /
        1000.0;

      /*  convert mapping from pixel/mm to pixel/m */
      B3 = (((2.0 * (x[4] * x[6] - x[3] * x[5]) * X + 2.0 * (x[5] * x[6] + x[3] *
               x[4]) * Y) + (((x[3] * x[3] - x[4] * x[4]) - x[5] * x[5]) + x[6] *
              x[6]) * Z) + x[2]) - z[i].cam_param.translation_w2c[2] / 1000.0;

      /*  convert mapping from pixel/mm to pixel/m             */
      b = (K[2] * num_measurements + K[5] * B2) + K[8] * B3;
      au = (K[0] * num_measurements + K[3] * B2) + K[6] * B3;
      B2 = (K[1] * num_measurements + K[4] * B2) + K[7] * B3;
      dB1_x[0] = 1.0;
      dB1_x[1] = 0.0;
      dB1_x[2] = 0.0;
      dB1_x[3] = 2.0 * ((x[3] * X - x[6] * Y) + x[5] * Z);
      dB1_x[4] = 2.0 * ((x[4] * X + x[5] * Y) + x[6] * Z);
      dB1_x[5] = 2.0 * ((-x[5] * X + x[4] * Y) + x[3] * Z);
      dB1_x[6] = 2.0 * ((-x[6] * X - x[3] * Y) + x[4] * Z);
      dB2_x[0] = 0.0;
      dB2_x[1] = 1.0;
      dB2_x[2] = 0.0;
      dB2_x[3] = 2.0 * ((x[6] * X + x[3] * Y) - x[4] * Z);
      dB2_x[4] = 2.0 * ((x[5] * X - x[4] * Y) - x[3] * Z);
      dB2_x[5] = 2.0 * ((x[4] * X + x[5] * Y) + x[6] * Z);
      dB2_x[6] = 2.0 * ((x[3] * X - x[6] * Y) + x[5] * Z);
      dB3_x[0] = 0.0;
      dB3_x[1] = 0.0;
      dB3_x[2] = 1.0;
      dB3_x[3] = 2.0 * ((-x[5] * X + x[4] * Y) + x[3] * Z);
      dB3_x[4] = 2.0 * ((x[6] * X + x[3] * Y) - x[4] * Z);
      dB3_x[5] = 2.0 * ((-x[3] * X + x[6] * Y) - x[5] * Z);
      dB3_x[6] = 2.0 * ((x[4] * X + x[5] * Y) + x[6] * Z);
      for (i14 = 0; i14 < 7; i14++) {
        b_prime[i14] = (K[2] * dB1_x[i14] + K[5] * dB2_x[i14]) + K[8] *
          dB3_x[i14];
      }

      num_measurements = b * b;
      for (i14 = 0; i14 < 7; i14++) {
        A_lin->data[((int32_T)k + A_lin->size[0] * i14) - 1] = (((K[0] *
          dB1_x[i14] + K[3] * dB2_x[i14]) + K[6] * dB3_x[i14]) * b - au *
          b_prime[i14]) / num_measurements;
      }

      for (i14 = 0; i14 < 7; i14++) {
        A_lin->data[((int32_T)((real_T)k + 1.0) + A_lin->size[0] * i14) - 1] =
          (((K[1] * dB1_x[i14] + K[4] * dB2_x[i14]) + K[7] * dB3_x[i14]) * b -
           B2 * b_prime[i14]) / num_measurements;
      }

      z_meas->data[(int32_T)k - 1] = z[i].red_data[0];
      z_meas->data[(int32_T)k] = z[i].red_data[5];
      k += 2U;
    }

    /*  green blobs */
    if (z[i].number_green == 1.0) {
      /*  do measurement estimation */
      for (i14 = 0; i14 < 3; i14++) {
        b_object_points[i14] = object_points[3 + i14] / 1000.0;
      }

      b_project2_2D(A, z[i].cam_param.rotation_w2c, Tcw, *(real_T (*)[3])&x[0], *
                    (real_T (*)[4])&x[3], b_object_points, dv11);
      for (i14 = 0; i14 < 2; i14++) {
        z_est->data[(int32_T)((real_T)k + (real_T)i14) - 1] = dv11[i14];
      }

      /*  do jacobian calc */
      for (i14 = 0; i14 < 3; i14++) {
        for (i15 = 0; i15 < 3; i15++) {
          K[i14 + 3 * i15] = 0.0;
          for (i16 = 0; i16 < 3; i16++) {
            K[i14 + 3 * i15] += A[i14 + 3 * i16] * z[i]
              .cam_param.rotation_w2c[i16 + 3 * i15];
          }
        }
      }

      X = object_points[3] / 1000.0;

      /*  convert mapping from pixel/mm to pixel/m */
      Y = object_points[4] / 1000.0;

      /*  convert mapping from pixel/mm to pixel/m */
      Z = object_points[5] / 1000.0;

      /*  calculate B's */
      num_measurements = ((((((x[3] * x[3] + x[4] * x[4]) - x[5] * x[5]) - x[6] *
        x[6]) * X + 2.0 * (x[4] * x[5] - x[3] * x[6]) * Y) + 2.0 * (x[4] * x[6]
        + x[3] * x[5]) * Z) + x[0]) - z[i].cam_param.translation_w2c[0] / 1000.0;

      /*  convert mapping from pixel/mm to pixel/m */
      B2 = (((2.0 * (x[4] * x[5] + x[3] * x[6]) * X + (((x[3] * x[3] - x[4] * x
                 [4]) + x[5] * x[5]) - x[6] * x[6]) * Y) + 2.0 * (x[5] * x[6] -
              x[3] * x[4]) * Z) + x[1]) - z[i].cam_param.translation_w2c[1] /
        1000.0;

      /*  convert mapping from pixel/mm to pixel/m */
      B3 = (((2.0 * (x[4] * x[6] - x[3] * x[5]) * X + 2.0 * (x[5] * x[6] + x[3] *
               x[4]) * Y) + (((x[3] * x[3] - x[4] * x[4]) - x[5] * x[5]) + x[6] *
              x[6]) * Z) + x[2]) - z[i].cam_param.translation_w2c[2] / 1000.0;

      /*  convert mapping from pixel/mm to pixel/m             */
      b = (K[2] * num_measurements + K[5] * B2) + K[8] * B3;
      au = (K[0] * num_measurements + K[3] * B2) + K[6] * B3;
      B2 = (K[1] * num_measurements + K[4] * B2) + K[7] * B3;
      dB1_x[0] = 1.0;
      dB1_x[1] = 0.0;
      dB1_x[2] = 0.0;
      dB1_x[3] = 2.0 * ((x[3] * X - x[6] * Y) + x[5] * Z);
      dB1_x[4] = 2.0 * ((x[4] * X + x[5] * Y) + x[6] * Z);
      dB1_x[5] = 2.0 * ((-x[5] * X + x[4] * Y) + x[3] * Z);
      dB1_x[6] = 2.0 * ((-x[6] * X - x[3] * Y) + x[4] * Z);
      dB2_x[0] = 0.0;
      dB2_x[1] = 1.0;
      dB2_x[2] = 0.0;
      dB2_x[3] = 2.0 * ((x[6] * X + x[3] * Y) - x[4] * Z);
      dB2_x[4] = 2.0 * ((x[5] * X - x[4] * Y) - x[3] * Z);
      dB2_x[5] = 2.0 * ((x[4] * X + x[5] * Y) + x[6] * Z);
      dB2_x[6] = 2.0 * ((x[3] * X - x[6] * Y) + x[5] * Z);
      dB3_x[0] = 0.0;
      dB3_x[1] = 0.0;
      dB3_x[2] = 1.0;
      dB3_x[3] = 2.0 * ((-x[5] * X + x[4] * Y) + x[3] * Z);
      dB3_x[4] = 2.0 * ((x[6] * X + x[3] * Y) - x[4] * Z);
      dB3_x[5] = 2.0 * ((-x[3] * X + x[6] * Y) - x[5] * Z);
      dB3_x[6] = 2.0 * ((x[4] * X + x[5] * Y) + x[6] * Z);
      for (i14 = 0; i14 < 7; i14++) {
        b_prime[i14] = (K[2] * dB1_x[i14] + K[5] * dB2_x[i14]) + K[8] *
          dB3_x[i14];
      }

      num_measurements = b * b;
      for (i14 = 0; i14 < 7; i14++) {
        A_lin->data[((int32_T)k + A_lin->size[0] * i14) - 1] = (((K[0] *
          dB1_x[i14] + K[3] * dB2_x[i14]) + K[6] * dB3_x[i14]) * b - au *
          b_prime[i14]) / num_measurements;
      }

      for (i14 = 0; i14 < 7; i14++) {
        A_lin->data[((int32_T)((real_T)k + 1.0) + A_lin->size[0] * i14) - 1] =
          (((K[1] * dB1_x[i14] + K[4] * dB2_x[i14]) + K[7] * dB3_x[i14]) * b -
           B2 * b_prime[i14]) / num_measurements;
      }

      z_meas->data[(int32_T)k - 1] = z[i].green_data[0];
      z_meas->data[(int32_T)k] = z[i].green_data[5];
      k += 2U;
    }

    /*  blue blobs */
    if (z[i].number_blue == 1.0) {
      for (i14 = 0; i14 < 3; i14++) {
        b_object_points[i14] = object_points[6 + i14] / 1000.0;
      }

      b_project2_2D(A, z[i].cam_param.rotation_w2c, Tcw, *(real_T (*)[3])&x[0], *
                    (real_T (*)[4])&x[3], b_object_points, dv11);
      for (i14 = 0; i14 < 2; i14++) {
        z_est->data[(int32_T)((real_T)k + (real_T)i14) - 1] = dv11[i14];
      }

      for (i14 = 0; i14 < 3; i14++) {
        for (i15 = 0; i15 < 3; i15++) {
          K[i14 + 3 * i15] = 0.0;
          for (i16 = 0; i16 < 3; i16++) {
            K[i14 + 3 * i15] += A[i14 + 3 * i16] * z[i]
              .cam_param.rotation_w2c[i16 + 3 * i15];
          }
        }
      }

      X = object_points[6] / 1000.0;

      /*  convert mapping from pixel/mm to pixel/m */
      Y = object_points[7] / 1000.0;

      /*  convert mapping from pixel/mm to pixel/m */
      Z = object_points[8] / 1000.0;

      /*  calculate B's */
      num_measurements = ((((((x[3] * x[3] + x[4] * x[4]) - x[5] * x[5]) - x[6] *
        x[6]) * X + 2.0 * (x[4] * x[5] - x[3] * x[6]) * Y) + 2.0 * (x[4] * x[6]
        + x[3] * x[5]) * Z) + x[0]) - z[i].cam_param.translation_w2c[0] / 1000.0;

      /*  convert mapping from pixel/mm to pixel/m */
      B2 = (((2.0 * (x[4] * x[5] + x[3] * x[6]) * X + (((x[3] * x[3] - x[4] * x
                 [4]) + x[5] * x[5]) - x[6] * x[6]) * Y) + 2.0 * (x[5] * x[6] -
              x[3] * x[4]) * Z) + x[1]) - z[i].cam_param.translation_w2c[1] /
        1000.0;

      /*  convert mapping from pixel/mm to pixel/m */
      B3 = (((2.0 * (x[4] * x[6] - x[3] * x[5]) * X + 2.0 * (x[5] * x[6] + x[3] *
               x[4]) * Y) + (((x[3] * x[3] - x[4] * x[4]) - x[5] * x[5]) + x[6] *
              x[6]) * Z) + x[2]) - z[i].cam_param.translation_w2c[2] / 1000.0;

      /*  convert mapping from pixel/mm to pixel/m             */
      b = (K[2] * num_measurements + K[5] * B2) + K[8] * B3;
      au = (K[0] * num_measurements + K[3] * B2) + K[6] * B3;
      B2 = (K[1] * num_measurements + K[4] * B2) + K[7] * B3;
      dB1_x[0] = 1.0;
      dB1_x[1] = 0.0;
      dB1_x[2] = 0.0;
      dB1_x[3] = 2.0 * ((x[3] * X - x[6] * Y) + x[5] * Z);
      dB1_x[4] = 2.0 * ((x[4] * X + x[5] * Y) + x[6] * Z);
      dB1_x[5] = 2.0 * ((-x[5] * X + x[4] * Y) + x[3] * Z);
      dB1_x[6] = 2.0 * ((-x[6] * X - x[3] * Y) + x[4] * Z);
      dB2_x[0] = 0.0;
      dB2_x[1] = 1.0;
      dB2_x[2] = 0.0;
      dB2_x[3] = 2.0 * ((x[6] * X + x[3] * Y) - x[4] * Z);
      dB2_x[4] = 2.0 * ((x[5] * X - x[4] * Y) - x[3] * Z);
      dB2_x[5] = 2.0 * ((x[4] * X + x[5] * Y) + x[6] * Z);
      dB2_x[6] = 2.0 * ((x[3] * X - x[6] * Y) + x[5] * Z);
      dB3_x[0] = 0.0;
      dB3_x[1] = 0.0;
      dB3_x[2] = 1.0;
      dB3_x[3] = 2.0 * ((-x[5] * X + x[4] * Y) + x[3] * Z);
      dB3_x[4] = 2.0 * ((x[6] * X + x[3] * Y) - x[4] * Z);
      dB3_x[5] = 2.0 * ((-x[3] * X + x[6] * Y) - x[5] * Z);
      dB3_x[6] = 2.0 * ((x[4] * X + x[5] * Y) + x[6] * Z);
      for (i14 = 0; i14 < 7; i14++) {
        b_prime[i14] = (K[2] * dB1_x[i14] + K[5] * dB2_x[i14]) + K[8] *
          dB3_x[i14];
      }

      num_measurements = b * b;
      for (i14 = 0; i14 < 7; i14++) {
        A_lin->data[((int32_T)k + A_lin->size[0] * i14) - 1] = (((K[0] *
          dB1_x[i14] + K[3] * dB2_x[i14]) + K[6] * dB3_x[i14]) * b - au *
          b_prime[i14]) / num_measurements;
      }

      for (i14 = 0; i14 < 7; i14++) {
        A_lin->data[((int32_T)((real_T)k + 1.0) + A_lin->size[0] * i14) - 1] =
          (((K[1] * dB1_x[i14] + K[4] * dB2_x[i14]) + K[7] * dB3_x[i14]) * b -
           B2 * b_prime[i14]) / num_measurements;
      }

      z_meas->data[(int32_T)k - 1] = z[i].blue_data[0];
      z_meas->data[(int32_T)k] = z[i].blue_data[5];
      k += 2U;
    }
  }

  b_emxInit_real_T(&b_z_meas, 1);
  i14 = b_z_meas->size[0];
  b_z_meas->size[0] = z_meas->size[0];
  emxEnsureCapacity((emxArray__common *)b_z_meas, i14, (int32_T)sizeof(real_T));
  i = z_meas->size[0];
  for (i14 = 0; i14 < i; i14++) {
    b_z_meas->data[i14] = z_meas->data[i14] - z_est->data[i14];
  }

  emxFree_real_T(&z_meas);
  emxFree_real_T(&z_est);
  solveLSQ(A_lin, b_z_meas, x, dB1_x, resnorm, x_est);
  emxFree_real_T(&b_z_meas);
  emxFree_real_T(&A_lin);
}

/* End of code generation (solvePoseEst.cpp) */
