/*
 * ekf_coder_initialize.cpp
 *
 * Code generation for function 'ekf_coder_initialize'
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
#include "ekf_coder_initialize.h"

/* Function Definitions */
void ekf_coder_initialize()
{
  rt_InitInfAndNaN(8U);
}

/* End of code generation (ekf_coder_initialize.cpp) */
