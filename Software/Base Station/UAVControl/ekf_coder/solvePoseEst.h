/*
 * solvePoseEst.h
 *
 * Code generation for function 'solvePoseEst'
 *
 * C source code generated on: Sun Aug 17 12:20:38 2014
 *
 */

#ifndef __SOLVEPOSEEST_H__
#define __SOLVEPOSEEST_H__
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
extern void solvePoseEst(const real_T x[7], const b_struct_T z[4], const real_T object_points[9], real_T x_est[7], real_T *resnorm);
#endif
/* End of code generation (solvePoseEst.h) */
