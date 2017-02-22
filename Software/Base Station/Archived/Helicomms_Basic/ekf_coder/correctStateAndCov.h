/*
 * correctStateAndCov.h
 *
 * Code generation for function 'correctStateAndCov'
 *
 * C source code generated on: Mon Jun  9 07:29:50 2014
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
#endif
/* End of code generation (correctStateAndCov.h) */
