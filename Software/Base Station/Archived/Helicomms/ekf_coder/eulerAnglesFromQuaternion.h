/*
 * eulerAnglesFromQuaternion.h
 *
 * Code generation for function 'eulerAnglesFromQuaternion'
 *
 * C source code generated on: Wed Jan 08 14:21:44 2014
 *
 */

#ifndef __EULERANGLESFROMQUATERNION_H__
#define __EULERANGLESFROMQUATERNION_H__
/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"

#include "rtwtypes.h"
#include "ekf_coder_types.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern void eulerAnglesFromQuaternion(const real_T x[4], real_T *roll, real_T *pitch, real_T *yaw);
#endif
/* End of code generation (eulerAnglesFromQuaternion.h) */
