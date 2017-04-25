/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_helmholtzfun_api.h
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 23-Apr-2017 17:19:12
 */

#ifndef _CODER_HELMHOLTZFUN_API_H
#define _CODER_HELMHOLTZFUN_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_helmholtzfun_api.h"

/* Type Definitions */
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_real_T*/

#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T

typedef struct emxArray_real_T emxArray_real_T;

#endif                                 /*typedef_emxArray_real_T*/

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void helmholtzfun(real_T delta, emxArray_real_T *u, emxArray_real_T *SORu);
extern void helmholtzfun_api(const mxArray * const prhs[1], const mxArray *plhs
  [2]);
extern void helmholtzfun_atexit(void);
extern void helmholtzfun_initialize(void);
extern void helmholtzfun_terminate(void);
extern void helmholtzfun_xil_terminate(void);

#endif

/*
 * File trailer for _coder_helmholtzfun_api.h
 *
 * [EOF]
 */
