/*
 * ekf_coder_emxutil.h
 *
 * Code generation for function 'ekf_coder_emxutil'
 *
 * C source code generated on: Mon Jun  9 07:29:51 2014
 *
 */

#ifndef __EKF_CODER_EMXUTIL_H__
#define __EKF_CODER_EMXUTIL_H__
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
extern void b_emxInit_real_T(emxArray_real_T **pEmxArray, int32_T numDimensions);
extern void emxEnsureCapacity(emxArray__common *emxArray, int32_T oldNumel, int32_T elementSize);
extern void emxFree_int32_T(emxArray_int32_T **pEmxArray);
extern void emxFree_real_T(emxArray_real_T **pEmxArray);
extern void emxInit_int32_T(emxArray_int32_T **pEmxArray, int32_T numDimensions);
extern void emxInit_real_T(emxArray_real_T **pEmxArray, int32_T numDimensions);
#endif
/* End of code generation (ekf_coder_emxutil.h) */
