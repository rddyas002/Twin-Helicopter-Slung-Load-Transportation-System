/*
 * inv.h
 *
 * Code generation for function 'inv'
 *
 * C source code generated on: Mon Jun  9 07:29:50 2014
 *
 */

#ifndef __INV_H__
#define __INV_H__
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
extern void inv(const real_T x[49], real_T y[49]);
extern void invNxN(const emxArray_real_T *x, emxArray_real_T *y);
#endif
/* End of code generation (inv.h) */
