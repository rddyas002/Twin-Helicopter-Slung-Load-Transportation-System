/*
 * projectStateAndCov.cpp
 *
 * Code generation for function 'projectStateAndCov'
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
#include "ekf_coder_rtwutil.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
static void projectState(const real_T x_prev[16], const real_T u[6], real_T dt,
  const real_T static_acceleration_earth[3], real_T x_next[16]);

/* Function Definitions */
static void projectState(const real_T x_prev[16], const real_T u[6], real_T dt,
  const real_T static_acceleration_earth[3], real_T x_next[16])
{
  real_T wx;
  real_T wy;
  real_T wz;
  int32_T i;
  real_T dv13[9];
  real_T b_u[3];
  real_T dv14[3];
  real_T t;
  int32_T i8;

  /*  implements discrete state differential step */
  memset(&x_next[0], 0, sizeof(real_T) << 4);

  /*  subtract gyro biases */
  wx = u[0] - x_prev[10];
  wy = u[1] - x_prev[11];
  wz = u[2] - x_prev[12];

  /*  subtract accelerometer biases */
  /*  project position */
  /*  p_{k+1} = p_{k} + dt*v_{k} */
  for (i = 0; i < 3; i++) {
    x_next[i] = x_prev[i] + dt * x_prev[i + 3];
  }

  /*  project velocity */
  /*  v_{k+1} = v_{k} + dt*a_{k} */
  /*  this function rotates the vector x from the body into the earth using the */
  /*  quaternion from the standard quaternion time derivative equation */
  dv13[0] = ((rt_powd_snf(x_prev[6], 2.0) + rt_powd_snf(x_prev[7], 2.0)) -
             rt_powd_snf(x_prev[8], 2.0)) - rt_powd_snf(x_prev[9], 2.0);
  dv13[3] = 2.0 * (x_prev[7] * x_prev[8] - x_prev[6] * x_prev[9]);
  dv13[6] = 2.0 * (x_prev[7] * x_prev[9] + x_prev[6] * x_prev[8]);
  dv13[1] = 2.0 * (x_prev[7] * x_prev[8] + x_prev[6] * x_prev[9]);
  dv13[4] = ((rt_powd_snf(x_prev[6], 2.0) - rt_powd_snf(x_prev[7], 2.0)) +
             rt_powd_snf(x_prev[8], 2.0)) - rt_powd_snf(x_prev[9], 2.0);
  dv13[7] = 2.0 * (x_prev[8] * x_prev[9] - x_prev[6] * x_prev[7]);
  dv13[2] = 2.0 * (x_prev[7] * x_prev[9] - x_prev[6] * x_prev[8]);
  dv13[5] = 2.0 * (x_prev[8] * x_prev[9] + x_prev[6] * x_prev[7]);
  dv13[8] = ((rt_powd_snf(x_prev[6], 2.0) - rt_powd_snf(x_prev[7], 2.0)) -
             rt_powd_snf(x_prev[8], 2.0)) + rt_powd_snf(x_prev[9], 2.0);
  b_u[0] = u[3] - x_prev[13];
  b_u[1] = u[4] - x_prev[14];
  b_u[2] = u[5] - x_prev[15];
  for (i = 0; i < 3; i++) {
    t = 0.0;
    for (i8 = 0; i8 < 3; i8++) {
      t += dv13[i + 3 * i8] * b_u[i8];
    }

    dv14[i] = t - static_acceleration_earth[i];
  }

  for (i = 0; i < 3; i++) {
    x_next[3 + i] = x_prev[i + 3] + dt * dv14[i];
  }

  /*  project quaternion */
  /*      lambda = 1-norm([q1,q2,q3,q4]); */
  /*      q_next = quaternionPropagate([q1;q2;q3;q4],[wx;wy;wz],dt,0.1,1e-3); */
  /*      x_next(7) = q_next(1); */
  /*      x_next(8) = q_next(2); */
  /*      x_next(9) = q_next(3); */
  /*      x_next(10) = q_next(4); */
  x_next[6] = x_prev[6] - dt * 0.5 * ((x_prev[7] * wx + x_prev[8] * wy) +
    x_prev[9] * wz);
  x_next[7] = x_prev[7] + dt * 0.5 * ((x_prev[6] * wx - x_prev[9] * wy) +
    x_prev[8] * wz);
  x_next[8] = x_prev[8] + dt * 0.5 * ((x_prev[9] * wx + x_prev[6] * wy) -
    x_prev[7] * wz);
  x_next[9] = x_prev[9] - dt * 0.5 * ((x_prev[8] * wx - x_prev[7] * wy) -
    x_prev[6] * wz);

  /*  normalize quaternion */
  wx = 0.0;
  wy = 2.2250738585072014E-308;
  for (i = 0; i < 4; i++) {
    wz = fabs(x_next[6 + i]);
    if (wz > wy) {
      t = wy / wz;
      wx = 1.0 + wx * t * t;
      wy = wz;
    } else {
      t = wz / wy;
      wx += t * t;
    }
  }

  wx = wy * sqrt(wx);
  for (i = 0; i < 4; i++) {
    x_next[6 + i] /= wx;
  }

  /*  project gyro and accel biases */
  x_next[10] = x_prev[10];
  x_next[11] = x_prev[11];
  x_next[12] = x_prev[12];
  x_next[13] = x_prev[13];
  x_next[14] = x_prev[14];
  x_next[15] = x_prev[15];
}

void projectStateAndCov(const real_T x[16], const real_T P[256], const real_T u
  [6], const real_T Q[324], real_T dt, const real_T static_acceleration_earth[3],
  real_T mean[16], real_T Cov[256])
{
  real_T Rq[9];
  real_T y;
  real_T dv3[9];
  real_T dv4[9];
  real_T dv5[9];
  real_T dv6[9];
  real_T b_x[9];
  real_T a_bar[3];
  real_T w_bar[3];
  real_T dv7[9];
  int32_T i;
  int32_T i6;
  real_T dv8[3];
  real_T dv9[9];
  real_T dv10[3];
  real_T dv11[3];
  real_T dv12[3];
  real_T c_x[12];
  real_T d_x[12];
  real_T A[256];
  static const int8_T a[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  real_T b_y;
  static const int8_T dfp_de[54] = { 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  real_T W[288];
  real_T b_A[256];
  real_T b_W[288];
  real_T c_A[256];
  real_T c_W[256];
  int32_T i7;

  /*  implements state prediction phase of KF */
  /*  state order */
  /*  x = [p v q w a]^T; */
  /*  p = x[1:3]: platform position w.r.t. room */
  /*  v = x[4:6]: platform velocity w.r.t. room */
  /*  q = x[7:10]: quaternion representing attitude between room and platform */
  /*  w = x[11:13]: gyro bias */
  /*  a = x[14:16]: accelerometer bais */
  /*  implement state-difference equation */
  projectState(x, u, dt, static_acceleration_earth, mean);

  /*      % calculate the second order term from papoulis */
  /*      accl = [u(4) - a_bias(1);u(5) - a_bias(2);u(6) - a_bias(3)]; */
  /*      df2 = zeros(16,16); */
  /*      dr2q1 = 2*[1 0 0;0 1 0;0 0 1]; */
  /*      dr2q2 = 2*[1 0 0;0 -1 0;0 0 -1]; */
  /*      dr2q3 = 2*[-1 0 0;0 1 0;0 0 -1]; */
  /*      dr2q4 = 2*[-1 0 0;0 -1 0;0 0 1]; */
  /*      df2(4:6,7:10) = dt*[dr2q1*accl,dr2q2*accl,dr2q3*accl,dr2q4*accl]; */
  /*      mean = mean + (1/2)*trace(df2*P); */
  /*  partial derivative of f w.r.t. the states */
  /*  bias removed */
  Rq[0] = ((rt_powd_snf(x[6], 2.0) + rt_powd_snf(x[7], 2.0)) - rt_powd_snf(x[8],
            2.0)) - rt_powd_snf(x[9], 2.0);
  Rq[3] = 2.0 * (x[7] * x[8] - x[6] * x[9]);
  Rq[6] = 2.0 * (x[7] * x[9] + x[6] * x[8]);
  Rq[1] = 2.0 * (x[7] * x[8] + x[6] * x[9]);
  Rq[4] = ((rt_powd_snf(x[6], 2.0) - rt_powd_snf(x[7], 2.0)) + rt_powd_snf(x[8],
            2.0)) - rt_powd_snf(x[9], 2.0);
  Rq[7] = 2.0 * (x[8] * x[9] - x[6] * x[7]);
  Rq[2] = 2.0 * (x[7] * x[9] - x[6] * x[8]);
  Rq[5] = 2.0 * (x[8] * x[9] + x[6] * x[7]);
  Rq[8] = ((rt_powd_snf(x[6], 2.0) - rt_powd_snf(x[7], 2.0)) - rt_powd_snf(x[8],
            2.0)) + rt_powd_snf(x[9], 2.0);

  /*  bias removed */
  y = dt / 2.0;
  b_eye(dv3);
  b_eye(dv4);
  b_eye(dv5);
  b_eye(dv6);
  b_x[0] = x[6];
  b_x[3] = -x[9];
  b_x[6] = x[8];
  b_x[1] = x[9];
  b_x[4] = x[6];
  b_x[7] = -x[7];
  b_x[2] = -x[8];
  b_x[5] = x[7];
  b_x[8] = x[6];
  for (i = 0; i < 3; i++) {
    a_bar[i] = u[i + 3] - x[i + 13];
    w_bar[i] = u[i] - x[i + 10];
    for (i6 = 0; i6 < 3; i6++) {
      dv7[i6 + 3 * i] = 2.0 * b_x[i6 + 3 * i];
    }
  }

  b_x[0] = x[7];
  b_x[3] = x[8];
  b_x[6] = x[9];
  b_x[1] = x[8];
  b_x[4] = -x[7];
  b_x[7] = -x[6];
  b_x[2] = x[9];
  b_x[5] = x[6];
  b_x[8] = -x[7];
  for (i6 = 0; i6 < 3; i6++) {
    dv8[i6] = 0.0;
    for (i = 0; i < 3; i++) {
      dv8[i6] += dv7[i6 + 3 * i] * a_bar[i];
    }

    for (i = 0; i < 3; i++) {
      dv9[i + 3 * i6] = 2.0 * b_x[i + 3 * i6];
    }
  }

  b_x[0] = -x[8];
  b_x[3] = x[7];
  b_x[6] = x[6];
  b_x[1] = x[7];
  b_x[4] = x[8];
  b_x[7] = x[9];
  b_x[2] = -x[6];
  b_x[5] = x[9];
  b_x[8] = -x[8];
  for (i6 = 0; i6 < 3; i6++) {
    dv10[i6] = 0.0;
    for (i = 0; i < 3; i++) {
      dv10[i6] += dv9[i6 + 3 * i] * a_bar[i];
    }

    for (i = 0; i < 3; i++) {
      dv7[i + 3 * i6] = 2.0 * b_x[i + 3 * i6];
    }
  }

  b_x[0] = -x[9];
  b_x[3] = -x[6];
  b_x[6] = x[7];
  b_x[1] = x[6];
  b_x[4] = -x[9];
  b_x[7] = x[8];
  b_x[2] = x[7];
  b_x[5] = x[8];
  b_x[8] = x[9];
  for (i6 = 0; i6 < 3; i6++) {
    dv11[i6] = 0.0;
    for (i = 0; i < 3; i++) {
      dv11[i6] += dv7[i6 + 3 * i] * a_bar[i];
    }

    for (i = 0; i < 3; i++) {
      dv9[i + 3 * i6] = 2.0 * b_x[i + 3 * i6];
    }
  }

  d_x[0] = x[7];
  d_x[4] = x[8];
  d_x[8] = x[9];
  d_x[1] = -x[6];
  d_x[5] = x[9];
  d_x[9] = -x[8];
  d_x[2] = -x[9];
  d_x[6] = -x[6];
  d_x[10] = x[7];
  d_x[3] = x[8];
  d_x[7] = -x[7];
  d_x[11] = -x[6];
  for (i6 = 0; i6 < 3; i6++) {
    dv12[i6] = 0.0;
    for (i = 0; i < 3; i++) {
      dv12[i6] += dv9[i6 + 3 * i] * a_bar[i];
    }

    c_x[i6] = dv8[i6];
    c_x[3 + i6] = dv10[i6];
    c_x[6 + i6] = dv11[i6];
    c_x[9 + i6] = dv12[i6];
    for (i = 0; i < 3; i++) {
      A[i + (i6 << 4)] = dv3[i + 3 * i6];
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      A[i + ((i6 + 3) << 4)] = (real_T)a[i + 3 * i6] * dt;
    }
  }

  for (i6 = 0; i6 < 4; i6++) {
    for (i = 0; i < 3; i++) {
      A[i + ((i6 + 6) << 4)] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      A[i + ((i6 + 10) << 4)] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      A[i + ((i6 + 13) << 4)] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      A[(i + (i6 << 4)) + 3] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      A[(i + ((i6 + 3) << 4)) + 3] = dv4[i + 3 * i6];
    }
  }

  for (i6 = 0; i6 < 4; i6++) {
    for (i = 0; i < 3; i++) {
      A[(i + ((i6 + 6) << 4)) + 3] = dt * c_x[i + 3 * i6];
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      A[(i + ((i6 + 10) << 4)) + 3] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      A[(i + ((i6 + 13) << 4)) + 3] = dt * -Rq[i + 3 * i6];
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 4; i++) {
      A[(i + (i6 << 4)) + 6] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 4; i++) {
      A[(i + ((i6 + 3) << 4)) + 6] = 0.0;
    }
  }

  A[102] = 1.0;
  A[118] = -w_bar[0] * dt / 2.0;
  A[134] = -w_bar[1] * dt / 2.0;
  A[150] = -w_bar[2] * dt / 2.0;
  A[103] = w_bar[0] * dt / 2.0;
  A[119] = 1.0;
  A[135] = w_bar[2] * dt / 2.0;
  A[151] = -w_bar[1] * dt / 2.0;
  A[104] = w_bar[1] * dt / 2.0;
  A[120] = -w_bar[2] * dt / 2.0;
  A[136] = 1.0;
  A[152] = w_bar[0] * dt / 2.0;
  A[105] = w_bar[2] * dt / 2.0;
  A[121] = w_bar[1] * dt / 2.0;
  A[137] = -w_bar[0] * dt / 2.0;
  A[153] = 1.0;
  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 4; i++) {
      A[(i + ((i6 + 10) << 4)) + 6] = y * d_x[i + (i6 << 2)];
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 4; i++) {
      A[(i + ((i6 + 13) << 4)) + 6] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      A[(i + (i6 << 4)) + 10] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      A[(i + ((i6 + 3) << 4)) + 10] = 0.0;
    }
  }

  for (i6 = 0; i6 < 4; i6++) {
    for (i = 0; i < 3; i++) {
      A[(i + ((i6 + 6) << 4)) + 10] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      A[(i + ((i6 + 10) << 4)) + 10] = dv5[i + 3 * i6];
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      A[(i + ((i6 + 13) << 4)) + 10] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      A[(i + (i6 << 4)) + 13] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      A[(i + ((i6 + 3) << 4)) + 13] = 0.0;
    }
  }

  for (i6 = 0; i6 < 4; i6++) {
    for (i = 0; i < 3; i++) {
      A[(i + ((i6 + 6) << 4)) + 13] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      A[(i + ((i6 + 10) << 4)) + 13] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      A[(i + ((i6 + 13) << 4)) + 13] = dv6[i + 3 * i6];
    }
  }

  y = 0.5 * dt;
  b_y = 0.5 * dt;
  c_eye(dv3);
  d_x[0] = -x[7];
  d_x[4] = -x[8];
  d_x[8] = -x[9];
  d_x[1] = x[6];
  d_x[5] = -x[9];
  d_x[9] = x[8];
  d_x[2] = x[9];
  d_x[6] = x[6];
  d_x[10] = -x[7];
  d_x[3] = -x[8];
  d_x[7] = x[7];
  d_x[11] = x[6];
  c_x[0] = x[7];
  c_x[4] = x[8];
  c_x[8] = x[9];
  c_x[1] = -x[6];
  c_x[5] = x[9];
  c_x[9] = -x[8];
  c_x[2] = -x[9];
  c_x[6] = -x[6];
  c_x[10] = x[7];
  c_x[3] = x[8];
  c_x[7] = -x[7];
  c_x[11] = -x[6];
  for (i6 = 0; i6 < 18; i6++) {
    for (i = 0; i < 3; i++) {
      W[i + (i6 << 4)] = (real_T)dfp_de[i + 3 * i6];
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      W[(i + (i6 << 4)) + 3] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      W[(i + ((i6 + 3) << 4)) + 3] = dv3[i + 3 * i6];
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      W[(i + ((i6 + 6) << 4)) + 3] = dt * Rq[i + 3 * i6];
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      W[(i + ((i6 + 9) << 4)) + 3] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      W[(i + ((i6 + 12) << 4)) + 3] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      W[(i + ((i6 + 15) << 4)) + 3] = -dt * Rq[i + 3 * i6];
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 4; i++) {
      W[(i + (i6 << 4)) + 6] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 4; i++) {
      W[(i + ((i6 + 3) << 4)) + 6] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 4; i++) {
      W[(i + ((i6 + 6) << 4)) + 6] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 4; i++) {
      W[(i + ((i6 + 9) << 4)) + 6] = y * d_x[i + (i6 << 2)];
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 4; i++) {
      W[(i + ((i6 + 12) << 4)) + 6] = b_y * c_x[i + (i6 << 2)];
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 4; i++) {
      W[(i + ((i6 + 15) << 4)) + 6] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      W[(i + (i6 << 4)) + 10] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      W[(i + ((i6 + 3) << 4)) + 10] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      W[(i + ((i6 + 6) << 4)) + 10] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      W[(i + ((i6 + 9) << 4)) + 10] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      W[(i + ((i6 + 12) << 4)) + 10] = dt * (real_T)a[i + 3 * i6];
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      W[(i + ((i6 + 15) << 4)) + 10] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      W[(i + (i6 << 4)) + 13] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      W[(i + ((i6 + 3) << 4)) + 13] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      W[(i + ((i6 + 6) << 4)) + 13] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      W[(i + ((i6 + 9) << 4)) + 13] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      W[(i + ((i6 + 12) << 4)) + 13] = 0.0;
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    for (i = 0; i < 3; i++) {
      W[(i + ((i6 + 15) << 4)) + 13] = dt * (real_T)a[i + 3 * i6];
    }
  }

  for (i6 = 0; i6 < 16; i6++) {
    for (i = 0; i < 16; i++) {
      b_A[i6 + (i << 4)] = 0.0;
      for (i7 = 0; i7 < 16; i7++) {
        b_A[i6 + (i << 4)] += A[i6 + (i7 << 4)] * P[i7 + (i << 4)];
      }
    }

    for (i = 0; i < 18; i++) {
      b_W[i6 + (i << 4)] = 0.0;
      for (i7 = 0; i7 < 18; i7++) {
        b_W[i6 + (i << 4)] += W[i6 + (i7 << 4)] * Q[i7 + 18 * i];
      }
    }

    for (i = 0; i < 16; i++) {
      c_A[i6 + (i << 4)] = 0.0;
      for (i7 = 0; i7 < 16; i7++) {
        c_A[i6 + (i << 4)] += b_A[i6 + (i7 << 4)] * A[i + (i7 << 4)];
      }

      c_W[i6 + (i << 4)] = 0.0;
      for (i7 = 0; i7 < 18; i7++) {
        c_W[i6 + (i << 4)] += b_W[i6 + (i7 << 4)] * W[i + (i7 << 4)];
      }
    }
  }

  for (i6 = 0; i6 < 16; i6++) {
    for (i = 0; i < 16; i++) {
      Cov[i + (i6 << 4)] = c_A[i + (i6 << 4)] + c_W[i + (i6 << 4)];
    }
  }

  /*      CovAug = (-1/4)*df2^2*P^2; */
  /*      Cov = Cov + CovAug; */
}

/* End of code generation (projectStateAndCov.cpp) */
