/*
 * correctStateAndCov.h
 *
 * Code generation for function 'correctStateAndCov'
 *
 * C source code generated on: Sun Aug 17 12:20:38 2014
 *
 */

#ifndef __CORRECTSTATEANDCOV_H__
#define __CORRECTSTATEANDCOV_H__
/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"

#include "rtwtypes.h"
#include "ekf_coder_types.h"

/* Function Declarations */
extern void correctStateAndCov(const real_T x[16], const b_struct_T z[4], const real_T P[256], const real_T object_points[9], real_T Measurement_noise, real_T xCorr[16], real_T PCorr[256]);
extern void project2_2D(const real_T A[9], const real_T Rcw[9], const real_T
  Tcw[3], const real_T Twp[3], const real_T qwp[4], const real_T Po[3], real_T
  x[2]);
#endif
/* End of code generation (correctStateAndCov.h) */
