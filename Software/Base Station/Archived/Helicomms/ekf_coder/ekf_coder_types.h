/*
 * ekf_coder_types.h
 *
 * Code generation for function 'correctStateAndCov'
 *
 * C source code generated on: Wed Jan 08 14:21:44 2014
 *
 */

#ifndef __EKF_CODER_TYPES_H__
#define __EKF_CODER_TYPES_H__

/* Type Definitions */
typedef struct
{
    real_T intrinsic_mat[9];
    real_T rotation_w2c[9];
    real_T translation_w2c[3];
} struct_T;
typedef struct
{
    real_T number_red;
    real_T number_green;
    real_T number_blue;
    real_T red_data[10];
    real_T green_data[10];
    real_T blue_data[10];
    real_T time;
    struct_T cam_param;
} b_struct_T;
#ifndef struct_emxArray__common
#define struct_emxArray__common
typedef struct emxArray__common
{
    void *data;
    int32_T *size;
    int32_T allocatedSize;
    int32_T numDimensions;
    boolean_T canFreeData;
} emxArray__common;
#endif
#ifndef struct_emxArray_int32_T
#define struct_emxArray_int32_T
typedef struct emxArray_int32_T
{
    int32_T *data;
    int32_T *size;
    int32_T allocatedSize;
    int32_T numDimensions;
    boolean_T canFreeData;
} emxArray_int32_T;
#endif
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T
typedef struct emxArray_real_T
{
    real_T *data;
    int32_T *size;
    int32_T allocatedSize;
    int32_T numDimensions;
    boolean_T canFreeData;
} emxArray_real_T;
#endif

#endif
/* End of code generation (ekf_coder_types.h) */
