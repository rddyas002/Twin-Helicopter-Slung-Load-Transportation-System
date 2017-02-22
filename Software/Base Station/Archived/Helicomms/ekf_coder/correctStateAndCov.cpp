/*
 * correctStateAndCov.cpp
 *
 * Code generation for function 'correctStateAndCov'
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
#include "ekf_coder_emxutil.h"
#include "mpower.h"
#include "eye.h"
#include "ekf_coder_rtwutil.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
static void project2_2D(const real_T A[9], const real_T Rcw[9], const real_T
  Tcw[3], const real_T Twp[3], const real_T qwp[4], const real_T Po[3], real_T
  x[2]);

/* Function Definitions */
static void project2_2D(const real_T A[9], const real_T Rcw[9], const real_T
  Tcw[3], const real_T Twp[3], const real_T qwp[4], const real_T Po[3], real_T
  x[2])
{
  int32_T i;
  real_T dv1[9];
  real_T b_A[9];
  int32_T i1;
  int32_T i2;
  real_T dv2[3];
  real_T d0;
  real_T image_homog[3];
  for (i = 0; i < 2; i++) {
    x[i] = 0.0;
  }

  dv1[0] = ((rt_powd_snf(qwp[0], 2.0) + rt_powd_snf(qwp[1], 2.0)) - rt_powd_snf
            (qwp[2], 2.0)) - rt_powd_snf(qwp[3], 2.0);
  dv1[3] = 2.0 * (qwp[1] * qwp[2] - qwp[0] * qwp[3]);
  dv1[6] = 2.0 * (qwp[1] * qwp[3] + qwp[0] * qwp[2]);
  dv1[1] = 2.0 * (qwp[1] * qwp[2] + qwp[0] * qwp[3]);
  dv1[4] = ((rt_powd_snf(qwp[0], 2.0) - rt_powd_snf(qwp[1], 2.0)) + rt_powd_snf
            (qwp[2], 2.0)) - rt_powd_snf(qwp[3], 2.0);
  dv1[7] = 2.0 * (qwp[2] * qwp[3] - qwp[0] * qwp[1]);
  dv1[2] = 2.0 * (qwp[1] * qwp[3] - qwp[0] * qwp[2]);
  dv1[5] = 2.0 * (qwp[2] * qwp[3] + qwp[0] * qwp[1]);
  dv1[8] = ((rt_powd_snf(qwp[0], 2.0) - rt_powd_snf(qwp[1], 2.0)) - rt_powd_snf
            (qwp[2], 2.0)) + rt_powd_snf(qwp[3], 2.0);
  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < 3; i1++) {
      b_A[i + 3 * i1] = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        b_A[i + 3 * i1] += A[i + 3 * i2] * Rcw[i2 + 3 * i1];
      }
    }
  }

  for (i = 0; i < 3; i++) {
    d0 = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      d0 += dv1[i + 3 * i1] * Po[i1];
    }

    dv2[i] = (d0 + Twp[i]) - Tcw[i];
  }

  for (i = 0; i < 3; i++) {
    image_homog[i] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      image_homog[i] += b_A[i + 3 * i1] * dv2[i1];
    }
  }

  x[0] = image_homog[0] / image_homog[2];
  x[1] = image_homog[1] / image_homog[2];
}

void correctStateAndCov(const real_T x[16], const b_struct_T z[4], const real_T
  P[256], const real_T object_points[9], real_T Measurement_noise, real_T xCorr
  [16], real_T PCorr[256])
{
  real_T num_measurements;
  int32_T n;
  emxArray_real_T *H;
  int32_T i0;
  int32_T ar;
  emxArray_real_T *z_est;
  emxArray_real_T *z_meas;
  uint32_T k;
  real_T A[9];
  real_T b_z[3];
  real_T b_object_points[3];
  real_T dv0[2];
  int32_T ib;
  real_T K[9];
  int32_T c;
  real_T X;
  real_T Y;
  real_T B2;
  real_T B3;
  real_T b;
  real_T au;
  real_T dB1_x[16];
  real_T dB2_x[16];
  real_T dB3_x[16];
  real_T b_prime[16];
  emxArray_real_T *a;
  emxArray_real_T *y;
  int32_T m;
  int32_T ic;
  int32_T br;
  int32_T ia;
  emxArray_real_T *Kalman;
  emxArray_real_T *C;
  uint32_T unnamed_idx_1;
  emxArray_real_T *b_y;
  emxArray_real_T *b_C;
  int8_T I[256];
  real_T c_y[256];
  real_T b_I[256];

  /*  need to determine the number of rows of H */
  /*  2 eqs per blob per camera --> maximum of 24 equations */
  /*  z is a 1x4 (column per camera) struct with the following field */
  /*      f1 = 'number_red'; */
  /*      f2 = 'number_green'; */
  /*      f3 = 'number_blue';    */
  /*      f4 = 'red_data'; */
  /*      f5 = 'green_data'; */
  /*      f6 = 'blue_data';   */
  /*      f7 = 'time'; */
  num_measurements = 0.0;

  /*  only allow single blob detection for now */
  for (n = 0; n < 4; n++) {
    if (z[n].number_red == 1.0) {
      num_measurements += 2.0;
    }

    if (z[n].number_green == 1.0) {
      num_measurements += 2.0;
    }

    if (z[n].number_blue == 1.0) {
      num_measurements += 2.0;
    }
  }

  emxInit_real_T(&H, 2);
  i0 = H->size[0] * H->size[1];
  H->size[0] = (int32_T)num_measurements;
  H->size[1] = 16;
  emxEnsureCapacity((emxArray__common *)H, i0, (int32_T)sizeof(real_T));
  ar = ((int32_T)num_measurements << 4) - 1;
  for (i0 = 0; i0 <= ar; i0++) {
    H->data[i0] = 0.0;
  }

  b_emxInit_real_T(&z_est, 1);
  i0 = z_est->size[0];
  z_est->size[0] = (int32_T)num_measurements;
  emxEnsureCapacity((emxArray__common *)z_est, i0, (int32_T)sizeof(real_T));
  ar = (int32_T)num_measurements - 1;
  for (i0 = 0; i0 <= ar; i0++) {
    z_est->data[i0] = 0.0;
  }

  b_emxInit_real_T(&z_meas, 1);
  i0 = z_meas->size[0];
  z_meas->size[0] = (int32_T)num_measurements;
  emxEnsureCapacity((emxArray__common *)z_meas, i0, (int32_T)sizeof(real_T));
  ar = (int32_T)num_measurements - 1;
  for (i0 = 0; i0 <= ar; i0++) {
    z_meas->data[i0] = 0.0;
  }

  /*  calculate innovation */
  k = 1U;
  for (n = 0; n < 4; n++) {
    /*  red blobs */
    if (z[n].number_red == 1.0) {
      /*  do measurement estimation */
      for (i0 = 0; i0 < 9; i0++) {
        A[i0] = z[n].cam_param.intrinsic_mat[i0] * 1000.0;
      }

      /*  convert mapping from pixel/mm to pixel/m */
      /*  convert mapping from pixel/mm to pixel/m */
      for (i0 = 0; i0 < 3; i0++) {
        b_z[i0] = z[n].cam_param.translation_w2c[i0] / 1000.0;
      }

      for (i0 = 0; i0 < 3; i0++) {
        b_object_points[i0] = object_points[i0] / 1000.0;
      }

      project2_2D(A, z[n].cam_param.rotation_w2c, b_z, *(real_T (*)[3])&x[0],
                  *(real_T (*)[4])&x[6], b_object_points, dv0);
      for (i0 = 0; i0 < 2; i0++) {
        z_est->data[(int32_T)((real_T)k + (real_T)i0) - 1] = dv0[i0];
      }

      /*  do jacobian calculation */
      for (i0 = 0; i0 < 3; i0++) {
        for (ib = 0; ib < 3; ib++) {
          K[i0 + 3 * ib] = 0.0;
          for (c = 0; c < 3; c++) {
            K[i0 + 3 * ib] += A[i0 + 3 * c] * z[n].cam_param.rotation_w2c[c + 3 *
              ib];
          }
        }
      }

      X = object_points[0] / 1000.0;
      Y = object_points[1] / 1000.0;

      /*  calculate B's */
      num_measurements = (((((rt_powd_snf(x[6], 2.0) + rt_powd_snf(x[7], 2.0)) -
        rt_powd_snf(x[8], 2.0)) - rt_powd_snf(x[9], 2.0)) * X + 2.0 * (x[7] * x
        [8] - x[6] * x[9]) * Y) + x[0]) - z[n].cam_param.translation_w2c[0] /
        1000.0;

      /*  convert mapping from pixel/mm to pixel/m */
      B2 = ((2.0 * (x[7] * x[8] + x[6] * x[9]) * X + (((rt_powd_snf(x[6], 2.0) -
                rt_powd_snf(x[7], 2.0)) + rt_powd_snf(x[8], 2.0)) - rt_powd_snf
              (x[9], 2.0)) * Y) + x[1]) - z[n].cam_param.translation_w2c[1] /
        1000.0;

      /*  convert mapping from pixel/mm to pixel/m */
      B3 = ((2.0 * (x[7] * x[9] - x[6] * x[8]) * X + 2.0 * (x[8] * x[9] + x[6] *
              x[7]) * Y) + x[2]) - z[n].cam_param.translation_w2c[2] / 1000.0;

      /*  convert mapping from pixel/mm to pixel/m */
      b = (K[2] * num_measurements + K[5] * B2) + K[8] * B3;
      au = (K[0] * num_measurements + K[3] * B2) + K[6] * B3;
      B2 = (K[1] * num_measurements + K[4] * B2) + K[7] * B3;
      dB1_x[0] = 1.0;
      dB1_x[1] = 0.0;
      dB1_x[2] = 0.0;
      for (i0 = 0; i0 < 3; i0++) {
        dB1_x[i0 + 3] = 0.0;
      }

      dB1_x[6] = 2.0 * (x[6] * X - x[9] * Y);
      dB1_x[7] = 2.0 * (x[7] * X + x[8] * Y);
      dB1_x[8] = 2.0 * (-x[8] * X + x[7] * Y);
      dB1_x[9] = 2.0 * (-x[9] * X - x[6] * Y);
      for (i0 = 0; i0 < 3; i0++) {
        dB1_x[i0 + 10] = 0.0;
      }

      dB2_x[0] = 0.0;
      dB2_x[1] = 1.0;
      dB2_x[2] = 0.0;
      for (i0 = 0; i0 < 3; i0++) {
        dB1_x[i0 + 13] = 0.0;
        dB2_x[i0 + 3] = 0.0;
      }

      dB2_x[6] = 2.0 * (x[9] * X + x[6] * Y);
      dB2_x[7] = 2.0 * (x[8] * X - x[7] * Y);
      dB2_x[8] = 2.0 * (x[7] * X + x[8] * Y);
      dB2_x[9] = 2.0 * (x[6] * X - x[9] * Y);
      for (i0 = 0; i0 < 3; i0++) {
        dB2_x[i0 + 10] = 0.0;
      }

      dB3_x[0] = 0.0;
      dB3_x[1] = 0.0;
      dB3_x[2] = 1.0;
      for (i0 = 0; i0 < 3; i0++) {
        dB2_x[i0 + 13] = 0.0;
        dB3_x[i0 + 3] = 0.0;
      }

      dB3_x[6] = 2.0 * (-x[8] * X + x[7] * Y);
      dB3_x[7] = 2.0 * (x[9] * X + x[6] * Y);
      dB3_x[8] = 2.0 * (-x[6] * X + x[9] * Y);
      dB3_x[9] = 2.0 * (x[7] * X + x[8] * Y);
      for (i0 = 0; i0 < 3; i0++) {
        dB3_x[i0 + 10] = 0.0;
      }

      for (i0 = 0; i0 < 3; i0++) {
        dB3_x[i0 + 13] = 0.0;
      }

      for (i0 = 0; i0 < 16; i0++) {
        b_prime[i0] = (K[2] * dB1_x[i0] + K[5] * dB2_x[i0]) + K[8] * dB3_x[i0];
      }

      num_measurements = rt_powd_snf(b, 2.0);
      for (i0 = 0; i0 < 16; i0++) {
        H->data[((int32_T)k + H->size[0] * i0) - 1] = (((K[0] * dB1_x[i0] + K[3]
          * dB2_x[i0]) + K[6] * dB3_x[i0]) * b - au * b_prime[i0]) /
          num_measurements;
      }

      for (i0 = 0; i0 < 16; i0++) {
        H->data[((int32_T)((real_T)k + 1.0) + H->size[0] * i0) - 1] = (((K[1] *
          dB1_x[i0] + K[4] * dB2_x[i0]) + K[7] * dB3_x[i0]) * b - B2 *
          b_prime[i0]) / num_measurements;
      }

      z_meas->data[(int32_T)k - 1] = z[n].red_data[0];
      z_meas->data[(int32_T)k] = z[n].red_data[5];
      k += 2U;
    }

    /*  green blobs */
    if (z[n].number_green == 1.0) {
      /*  do measurement estimation */
      for (i0 = 0; i0 < 9; i0++) {
        A[i0] = z[n].cam_param.intrinsic_mat[i0] * 1000.0;
      }

      /*  convert mapping from pixel/mm to pixel/m */
      /*  convert mapping from pixel/mm to pixel/m */
      for (i0 = 0; i0 < 3; i0++) {
        b_z[i0] = z[n].cam_param.translation_w2c[i0] / 1000.0;
      }

      for (i0 = 0; i0 < 3; i0++) {
        b_object_points[i0] = object_points[3 + i0] / 1000.0;
      }

      project2_2D(A, z[n].cam_param.rotation_w2c, b_z, *(real_T (*)[3])&x[0],
                  *(real_T (*)[4])&x[6], b_object_points, dv0);
      for (i0 = 0; i0 < 2; i0++) {
        z_est->data[(int32_T)((real_T)k + (real_T)i0) - 1] = dv0[i0];
      }

      /*  do jacobian calc */
      for (i0 = 0; i0 < 3; i0++) {
        for (ib = 0; ib < 3; ib++) {
          K[i0 + 3 * ib] = 0.0;
          for (c = 0; c < 3; c++) {
            K[i0 + 3 * ib] += A[i0 + 3 * c] * z[n].cam_param.rotation_w2c[c + 3 *
              ib];
          }
        }
      }

      X = object_points[3] / 1000.0;

      /*  convert mapping from pixel/mm to pixel/m */
      Y = object_points[4] / 1000.0;

      /*  convert mapping from pixel/mm to pixel/m */
      /*  calculate B's */
      num_measurements = (((((rt_powd_snf(x[6], 2.0) + rt_powd_snf(x[7], 2.0)) -
        rt_powd_snf(x[8], 2.0)) - rt_powd_snf(x[9], 2.0)) * X + 2.0 * (x[7] * x
        [8] - x[6] * x[9]) * Y) + x[0]) - z[n].cam_param.translation_w2c[0] /
        1000.0;

      /*  convert mapping from pixel/mm to pixel/m */
      B2 = ((2.0 * (x[7] * x[8] + x[6] * x[9]) * X + (((rt_powd_snf(x[6], 2.0) -
                rt_powd_snf(x[7], 2.0)) + rt_powd_snf(x[8], 2.0)) - rt_powd_snf
              (x[9], 2.0)) * Y) + x[1]) - z[n].cam_param.translation_w2c[1] /
        1000.0;

      /*  convert mapping from pixel/mm to pixel/m */
      B3 = ((2.0 * (x[7] * x[9] - x[6] * x[8]) * X + 2.0 * (x[8] * x[9] + x[6] *
              x[7]) * Y) + x[2]) - z[n].cam_param.translation_w2c[2] / 1000.0;

      /*  convert mapping from pixel/mm to pixel/m */
      b = (K[2] * num_measurements + K[5] * B2) + K[8] * B3;
      au = (K[0] * num_measurements + K[3] * B2) + K[6] * B3;
      B2 = (K[1] * num_measurements + K[4] * B2) + K[7] * B3;
      dB1_x[0] = 1.0;
      dB1_x[1] = 0.0;
      dB1_x[2] = 0.0;
      for (i0 = 0; i0 < 3; i0++) {
        dB1_x[i0 + 3] = 0.0;
      }

      dB1_x[6] = 2.0 * (x[6] * X - x[9] * Y);
      dB1_x[7] = 2.0 * (x[7] * X + x[8] * Y);
      dB1_x[8] = 2.0 * (-x[8] * X + x[7] * Y);
      dB1_x[9] = 2.0 * (-x[9] * X - x[6] * Y);
      for (i0 = 0; i0 < 3; i0++) {
        dB1_x[i0 + 10] = 0.0;
      }

      dB2_x[0] = 0.0;
      dB2_x[1] = 1.0;
      dB2_x[2] = 0.0;
      for (i0 = 0; i0 < 3; i0++) {
        dB1_x[i0 + 13] = 0.0;
        dB2_x[i0 + 3] = 0.0;
      }

      dB2_x[6] = 2.0 * (x[9] * X + x[6] * Y);
      dB2_x[7] = 2.0 * (x[8] * X - x[7] * Y);
      dB2_x[8] = 2.0 * (x[7] * X + x[8] * Y);
      dB2_x[9] = 2.0 * (x[6] * X - x[9] * Y);
      for (i0 = 0; i0 < 3; i0++) {
        dB2_x[i0 + 10] = 0.0;
      }

      dB3_x[0] = 0.0;
      dB3_x[1] = 0.0;
      dB3_x[2] = 1.0;
      for (i0 = 0; i0 < 3; i0++) {
        dB2_x[i0 + 13] = 0.0;
        dB3_x[i0 + 3] = 0.0;
      }

      dB3_x[6] = 2.0 * (-x[8] * X + x[7] * Y);
      dB3_x[7] = 2.0 * (x[9] * X + x[6] * Y);
      dB3_x[8] = 2.0 * (-x[6] * X + x[9] * Y);
      dB3_x[9] = 2.0 * (x[7] * X + x[8] * Y);
      for (i0 = 0; i0 < 3; i0++) {
        dB3_x[i0 + 10] = 0.0;
      }

      for (i0 = 0; i0 < 3; i0++) {
        dB3_x[i0 + 13] = 0.0;
      }

      for (i0 = 0; i0 < 16; i0++) {
        b_prime[i0] = (K[2] * dB1_x[i0] + K[5] * dB2_x[i0]) + K[8] * dB3_x[i0];
      }

      num_measurements = rt_powd_snf(b, 2.0);
      for (i0 = 0; i0 < 16; i0++) {
        H->data[((int32_T)k + H->size[0] * i0) - 1] = (((K[0] * dB1_x[i0] + K[3]
          * dB2_x[i0]) + K[6] * dB3_x[i0]) * b - au * b_prime[i0]) /
          num_measurements;
      }

      for (i0 = 0; i0 < 16; i0++) {
        H->data[((int32_T)((real_T)k + 1.0) + H->size[0] * i0) - 1] = (((K[1] *
          dB1_x[i0] + K[4] * dB2_x[i0]) + K[7] * dB3_x[i0]) * b - B2 *
          b_prime[i0]) / num_measurements;
      }

      z_meas->data[(int32_T)k - 1] = z[n].green_data[0];
      z_meas->data[(int32_T)k] = z[n].green_data[5];
      k += 2U;
    }

    /*  blue blobs */
    if (z[n].number_blue == 1.0) {
      for (i0 = 0; i0 < 9; i0++) {
        A[i0] = z[n].cam_param.intrinsic_mat[i0] * 1000.0;
      }

      /*  convert mapping from pixel/mm to pixel/m */
      /*  convert mapping from pixel/mm to pixel/m */
      for (i0 = 0; i0 < 3; i0++) {
        b_z[i0] = z[n].cam_param.translation_w2c[i0] / 1000.0;
      }

      for (i0 = 0; i0 < 3; i0++) {
        b_object_points[i0] = object_points[6 + i0] / 1000.0;
      }

      project2_2D(A, z[n].cam_param.rotation_w2c, b_z, *(real_T (*)[3])&x[0],
                  *(real_T (*)[4])&x[6], b_object_points, dv0);
      for (i0 = 0; i0 < 2; i0++) {
        z_est->data[(int32_T)((real_T)k + (real_T)i0) - 1] = dv0[i0];
      }

      for (i0 = 0; i0 < 3; i0++) {
        for (ib = 0; ib < 3; ib++) {
          K[i0 + 3 * ib] = 0.0;
          for (c = 0; c < 3; c++) {
            K[i0 + 3 * ib] += A[i0 + 3 * c] * z[n].cam_param.rotation_w2c[c + 3 *
              ib];
          }
        }
      }

      X = object_points[6] / 1000.0;

      /*  convert mapping from pixel/mm to pixel/m */
      Y = object_points[7] / 1000.0;

      /*  convert mapping from pixel/mm to pixel/m */
      /*  calculate B's */
      num_measurements = (((((rt_powd_snf(x[6], 2.0) + rt_powd_snf(x[7], 2.0)) -
        rt_powd_snf(x[8], 2.0)) - rt_powd_snf(x[9], 2.0)) * X + 2.0 * (x[7] * x
        [8] - x[6] * x[9]) * Y) + x[0]) - z[n].cam_param.translation_w2c[0] /
        1000.0;

      /*  convert mapping from pixel/mm to pixel/m */
      B2 = ((2.0 * (x[7] * x[8] + x[6] * x[9]) * X + (((rt_powd_snf(x[6], 2.0) -
                rt_powd_snf(x[7], 2.0)) + rt_powd_snf(x[8], 2.0)) - rt_powd_snf
              (x[9], 2.0)) * Y) + x[1]) - z[n].cam_param.translation_w2c[1] /
        1000.0;

      /*  convert mapping from pixel/mm to pixel/m */
      B3 = ((2.0 * (x[7] * x[9] - x[6] * x[8]) * X + 2.0 * (x[8] * x[9] + x[6] *
              x[7]) * Y) + x[2]) - z[n].cam_param.translation_w2c[2] / 1000.0;

      /*  convert mapping from pixel/mm to pixel/m */
      b = (K[2] * num_measurements + K[5] * B2) + K[8] * B3;
      au = (K[0] * num_measurements + K[3] * B2) + K[6] * B3;
      B2 = (K[1] * num_measurements + K[4] * B2) + K[7] * B3;
      dB1_x[0] = 1.0;
      dB1_x[1] = 0.0;
      dB1_x[2] = 0.0;
      for (i0 = 0; i0 < 3; i0++) {
        dB1_x[i0 + 3] = 0.0;
      }

      dB1_x[6] = 2.0 * (x[6] * X - x[9] * Y);
      dB1_x[7] = 2.0 * (x[7] * X + x[8] * Y);
      dB1_x[8] = 2.0 * (-x[8] * X + x[7] * Y);
      dB1_x[9] = 2.0 * (-x[9] * X - x[6] * Y);
      for (i0 = 0; i0 < 3; i0++) {
        dB1_x[i0 + 10] = 0.0;
      }

      dB2_x[0] = 0.0;
      dB2_x[1] = 1.0;
      dB2_x[2] = 0.0;
      for (i0 = 0; i0 < 3; i0++) {
        dB1_x[i0 + 13] = 0.0;
        dB2_x[i0 + 3] = 0.0;
      }

      dB2_x[6] = 2.0 * (x[9] * X + x[6] * Y);
      dB2_x[7] = 2.0 * (x[8] * X - x[7] * Y);
      dB2_x[8] = 2.0 * (x[7] * X + x[8] * Y);
      dB2_x[9] = 2.0 * (x[6] * X - x[9] * Y);
      for (i0 = 0; i0 < 3; i0++) {
        dB2_x[i0 + 10] = 0.0;
      }

      dB3_x[0] = 0.0;
      dB3_x[1] = 0.0;
      dB3_x[2] = 1.0;
      for (i0 = 0; i0 < 3; i0++) {
        dB2_x[i0 + 13] = 0.0;
        dB3_x[i0 + 3] = 0.0;
      }

      dB3_x[6] = 2.0 * (-x[8] * X + x[7] * Y);
      dB3_x[7] = 2.0 * (x[9] * X + x[6] * Y);
      dB3_x[8] = 2.0 * (-x[6] * X + x[9] * Y);
      dB3_x[9] = 2.0 * (x[7] * X + x[8] * Y);
      for (i0 = 0; i0 < 3; i0++) {
        dB3_x[i0 + 10] = 0.0;
      }

      for (i0 = 0; i0 < 3; i0++) {
        dB3_x[i0 + 13] = 0.0;
      }

      for (i0 = 0; i0 < 16; i0++) {
        b_prime[i0] = (K[2] * dB1_x[i0] + K[5] * dB2_x[i0]) + K[8] * dB3_x[i0];
      }

      num_measurements = rt_powd_snf(b, 2.0);
      for (i0 = 0; i0 < 16; i0++) {
        H->data[((int32_T)k + H->size[0] * i0) - 1] = (((K[0] * dB1_x[i0] + K[3]
          * dB2_x[i0]) + K[6] * dB3_x[i0]) * b - au * b_prime[i0]) /
          num_measurements;
      }

      for (i0 = 0; i0 < 16; i0++) {
        H->data[((int32_T)((real_T)k + 1.0) + H->size[0] * i0) - 1] = (((K[1] *
          dB1_x[i0] + K[4] * dB2_x[i0]) + K[7] * dB3_x[i0]) * b - B2 *
          b_prime[i0]) / num_measurements;
      }

      z_meas->data[(int32_T)k - 1] = z[n].blue_data[0];
      z_meas->data[(int32_T)k] = z[n].blue_data[5];
      k += 2U;
    }
  }

  emxInit_real_T(&a, 2);
  emxInit_real_T(&y, 2);

  /*  pixel noise goes additively */
  eye((real_T)H->size[0], a);

  /*  Compute Kalman gain */
  k = (uint32_T)H->size[0];
  i0 = y->size[0] * y->size[1];
  y->size[0] = (int32_T)k;
  y->size[1] = 16;
  emxEnsureCapacity((emxArray__common *)y, i0, (int32_T)sizeof(real_T));
  m = H->size[0];
  i0 = y->size[0] * y->size[1];
  y->size[0] = y->size[0];
  y->size[1] = 16;
  emxEnsureCapacity((emxArray__common *)y, i0, (int32_T)sizeof(real_T));
  for (i0 = 0; i0 < 16; i0++) {
    ar = y->size[0] - 1;
    for (ib = 0; ib <= ar; ib++) {
      y->data[ib + y->size[0] * i0] = 0.0;
    }
  }

  if (m == 0) {
  } else {
    c = m * 15;
    for (n = 0; n <= c; n += m) {
      i0 = n + m;
      for (ic = n; ic + 1 <= i0; ic++) {
        y->data[ic] = 0.0;
      }
    }

    br = 0;
    for (n = 0; n <= c; n += m) {
      ar = 0;
      for (ib = br; ib + 1 <= br + 16; ib++) {
        if (P[ib] != 0.0) {
          ia = ar;
          i0 = n + m;
          for (ic = n; ic + 1 <= i0; ic++) {
            ia++;
            y->data[ic] += P[ib] * H->data[ia - 1];
          }
        }

        ar += m;
      }

      br += 16;
    }
  }

  emxInit_real_T(&Kalman, 2);
  i0 = Kalman->size[0] * Kalman->size[1];
  Kalman->size[0] = 16;
  Kalman->size[1] = H->size[0];
  emxEnsureCapacity((emxArray__common *)Kalman, i0, (int32_T)sizeof(real_T));
  ar = H->size[0] - 1;
  for (i0 = 0; i0 <= ar; i0++) {
    for (ib = 0; ib < 16; ib++) {
      Kalman->data[ib + Kalman->size[0] * i0] = H->data[i0 + H->size[0] * ib];
    }
  }

  emxInit_real_T(&C, 2);
  k = (uint32_T)y->size[0];
  unnamed_idx_1 = (uint32_T)Kalman->size[1];
  i0 = C->size[0] * C->size[1];
  C->size[0] = (int32_T)k;
  C->size[1] = (int32_T)unnamed_idx_1;
  emxEnsureCapacity((emxArray__common *)C, i0, (int32_T)sizeof(real_T));
  m = y->size[0];
  n = Kalman->size[1];
  i0 = C->size[0] * C->size[1];
  C->size[0] = C->size[0];
  C->size[1] = C->size[1];
  emxEnsureCapacity((emxArray__common *)C, i0, (int32_T)sizeof(real_T));
  ar = C->size[1] - 1;
  for (i0 = 0; i0 <= ar; i0++) {
    br = C->size[0] - 1;
    for (ib = 0; ib <= br; ib++) {
      C->data[ib + C->size[0] * i0] = 0.0;
    }
  }

  if ((m == 0) || (n == 0)) {
  } else {
    c = m * (n - 1);
    for (n = 0; n <= c; n += m) {
      i0 = n + m;
      for (ic = n; ic + 1 <= i0; ic++) {
        C->data[ic] = 0.0;
      }
    }

    br = 0;
    for (n = 0; n <= c; n += m) {
      ar = 0;
      for (ib = br; ib + 1 <= br + 16; ib++) {
        if (Kalman->data[ib] != 0.0) {
          ia = ar;
          i0 = n + m;
          for (ic = n; ic + 1 <= i0; ic++) {
            ia++;
            C->data[ic] += Kalman->data[ib] * y->data[ia - 1];
          }
        }

        ar += m;
      }

      br += 16;
    }
  }

  emxFree_real_T(&y);
  i0 = Kalman->size[0] * Kalman->size[1];
  Kalman->size[0] = 16;
  Kalman->size[1] = H->size[0];
  emxEnsureCapacity((emxArray__common *)Kalman, i0, (int32_T)sizeof(real_T));
  ar = H->size[0] - 1;
  for (i0 = 0; i0 <= ar; i0++) {
    for (ib = 0; ib < 16; ib++) {
      Kalman->data[ib + Kalman->size[0] * i0] = H->data[i0 + H->size[0] * ib];
    }
  }

  emxInit_real_T(&b_y, 2);
  unnamed_idx_1 = (uint32_T)Kalman->size[1];
  i0 = b_y->size[0] * b_y->size[1];
  b_y->size[0] = 16;
  b_y->size[1] = (int32_T)unnamed_idx_1;
  emxEnsureCapacity((emxArray__common *)b_y, i0, (int32_T)sizeof(real_T));
  n = Kalman->size[1];
  i0 = b_y->size[0] * b_y->size[1];
  b_y->size[0] = 16;
  b_y->size[1] = b_y->size[1];
  emxEnsureCapacity((emxArray__common *)b_y, i0, (int32_T)sizeof(real_T));
  ar = b_y->size[1] - 1;
  for (i0 = 0; i0 <= ar; i0++) {
    for (ib = 0; ib < 16; ib++) {
      b_y->data[ib + b_y->size[0] * i0] = 0.0;
    }
  }

  if (n == 0) {
  } else {
    c = (n - 1) << 4;
    for (n = 0; n <= c; n += 16) {
      for (ic = n; ic + 1 <= n + 16; ic++) {
        b_y->data[ic] = 0.0;
      }
    }

    br = 0;
    for (n = 0; n <= c; n += 16) {
      ar = 0;
      for (ib = br; ib + 1 <= br + 16; ib++) {
        if (Kalman->data[ib] != 0.0) {
          ia = ar;
          for (ic = n; ic + 1 <= n + 16; ic++) {
            ia++;
            b_y->data[ic] += Kalman->data[ib] * P[ia - 1];
          }
        }

        ar += 16;
      }

      br += 16;
    }
  }

  emxInit_real_T(&b_C, 2);
  i0 = b_C->size[0] * b_C->size[1];
  b_C->size[0] = C->size[0];
  b_C->size[1] = C->size[1];
  emxEnsureCapacity((emxArray__common *)b_C, i0, (int32_T)sizeof(real_T));
  ar = C->size[0] * C->size[1] - 1;
  for (i0 = 0; i0 <= ar; i0++) {
    b_C->data[i0] = C->data[i0] + a->data[i0] * Measurement_noise;
  }

  emxFree_real_T(&C);
  mpower(b_C, a);
  emxFree_real_T(&b_C);
  if ((b_y->size[1] == 1) || (a->size[0] == 1)) {
    i0 = Kalman->size[0] * Kalman->size[1];
    Kalman->size[0] = 16;
    Kalman->size[1] = a->size[1];
    emxEnsureCapacity((emxArray__common *)Kalman, i0, (int32_T)sizeof(real_T));
    for (i0 = 0; i0 < 16; i0++) {
      ar = a->size[1] - 1;
      for (ib = 0; ib <= ar; ib++) {
        Kalman->data[i0 + Kalman->size[0] * ib] = 0.0;
        br = b_y->size[1] - 1;
        for (c = 0; c <= br; c++) {
          Kalman->data[i0 + Kalman->size[0] * ib] += b_y->data[i0 + b_y->size[0]
            * c] * a->data[c + a->size[0] * ib];
        }
      }
    }
  } else {
    m = b_y->size[1];
    unnamed_idx_1 = (uint32_T)a->size[1];
    i0 = Kalman->size[0] * Kalman->size[1];
    Kalman->size[0] = 16;
    Kalman->size[1] = (int32_T)unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)Kalman, i0, (int32_T)sizeof(real_T));
    n = a->size[1];
    i0 = Kalman->size[0] * Kalman->size[1];
    Kalman->size[0] = 16;
    Kalman->size[1] = Kalman->size[1];
    emxEnsureCapacity((emxArray__common *)Kalman, i0, (int32_T)sizeof(real_T));
    ar = Kalman->size[1] - 1;
    for (i0 = 0; i0 <= ar; i0++) {
      for (ib = 0; ib < 16; ib++) {
        Kalman->data[ib + Kalman->size[0] * i0] = 0.0;
      }
    }

    if (n == 0) {
    } else {
      c = (n - 1) << 4;
      for (n = 0; n <= c; n += 16) {
        for (ic = n; ic + 1 <= n + 16; ic++) {
          Kalman->data[ic] = 0.0;
        }
      }

      br = 0;
      for (n = 0; n <= c; n += 16) {
        ar = 0;
        i0 = br + m;
        for (ib = br; ib + 1 <= i0; ib++) {
          if (a->data[ib] != 0.0) {
            ia = ar;
            for (ic = n; ic + 1 <= n + 16; ic++) {
              ia++;
              Kalman->data[ic] += a->data[ib] * b_y->data[ia - 1];
            }
          }

          ar += 16;
        }

        br += m;
      }
    }
  }

  emxFree_real_T(&b_y);
  emxFree_real_T(&a);
  i0 = z_meas->size[0];
  z_meas->size[0] = z_meas->size[0];
  emxEnsureCapacity((emxArray__common *)z_meas, i0, (int32_T)sizeof(real_T));
  ar = z_meas->size[0] - 1;
  for (i0 = 0; i0 <= ar; i0++) {
    z_meas->data[i0] -= z_est->data[i0];
  }

  emxFree_real_T(&z_est);
  if ((Kalman->size[1] == 1) || (z_meas->size[0] == 1)) {
    for (i0 = 0; i0 < 16; i0++) {
      xCorr[i0] = 0.0;
      ar = z_meas->size[0] - 1;
      for (ib = 0; ib <= ar; ib++) {
        num_measurements = xCorr[i0] + Kalman->data[i0 + Kalman->size[0] * ib] *
          z_meas->data[ib];
        xCorr[i0] = num_measurements;
      }
    }
  } else {
    m = Kalman->size[1];
    memset(&xCorr[0], 0, sizeof(real_T) << 4);
    ar = 0;
    for (ib = 0; ib + 1 <= m; ib++) {
      if (z_meas->data[ib] != 0.0) {
        ia = ar;
        for (ic = 0; ic < 16; ic++) {
          ia++;
          num_measurements = xCorr[ic] + z_meas->data[ib] * Kalman->data[ia - 1];
          xCorr[ic] = num_measurements;
        }
      }

      ar += 16;
    }
  }

  emxFree_real_T(&z_meas);
  for (i0 = 0; i0 < 16; i0++) {
    xCorr[i0] += x[i0];
  }

  memset(&I[0], 0, sizeof(int8_T) << 8);
  for (n = 0; n < 16; n++) {
    I[n + (n << 4)] = 1;
  }

  if ((Kalman->size[1] == 1) || (H->size[0] == 1)) {
    for (i0 = 0; i0 < 16; i0++) {
      for (ib = 0; ib < 16; ib++) {
        c_y[i0 + (ib << 4)] = 0.0;
        ar = Kalman->size[1] - 1;
        for (c = 0; c <= ar; c++) {
          c_y[i0 + (ib << 4)] += Kalman->data[i0 + Kalman->size[0] * c] *
            H->data[c + H->size[0] * ib];
        }
      }
    }
  } else {
    m = Kalman->size[1];
    memset(&c_y[0], 0, sizeof(real_T) << 8);
    for (n = 0; n < 242; n += 16) {
      for (ic = n; ic + 1 <= n + 16; ic++) {
        c_y[ic] = 0.0;
      }
    }

    br = 0;
    for (n = 0; n < 242; n += 16) {
      ar = 0;
      i0 = br + m;
      for (ib = br; ib + 1 <= i0; ib++) {
        if (H->data[ib] != 0.0) {
          ia = ar;
          for (ic = n; ic + 1 <= n + 16; ic++) {
            ia++;
            c_y[ic] += H->data[ib] * Kalman->data[ia - 1];
          }
        }

        ar += 16;
      }

      br += m;
    }
  }

  emxFree_real_T(&Kalman);
  emxFree_real_T(&H);
  for (i0 = 0; i0 < 16; i0++) {
    for (ib = 0; ib < 16; ib++) {
      b_I[ib + (i0 << 4)] = (real_T)I[ib + (i0 << 4)] - c_y[ib + (i0 << 4)];
    }
  }

  for (i0 = 0; i0 < 16; i0++) {
    for (ib = 0; ib < 16; ib++) {
      PCorr[i0 + (ib << 4)] = 0.0;
      for (c = 0; c < 16; c++) {
        PCorr[i0 + (ib << 4)] += b_I[i0 + (c << 4)] * P[c + (ib << 4)];
      }
    }
  }
}

/* End of code generation (correctStateAndCov.cpp) */
