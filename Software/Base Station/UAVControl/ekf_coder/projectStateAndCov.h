/*
 * projectStateAndCov.h
 *
 * Code generation for function 'projectStateAndCov'
 *
 * C source code generated on: Sun Aug 17 12:20:38 2014
 *
 */

#ifndef __PROJECTSTATEANDCOV_H__
#define __PROJECTSTATEANDCOV_H__
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
extern void projectStateAndCov(const real_T x[16], const real_T P[256], const real_T u[6], const real_T Q[324], real_T dt, const real_T static_acceleration_earth[3], real_T mean[16], real_T Cov[256]);
#endif
/* End of code generation (projectStateAndCov.h) */
