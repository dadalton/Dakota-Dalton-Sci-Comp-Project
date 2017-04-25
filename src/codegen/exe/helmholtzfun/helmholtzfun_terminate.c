/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: helmholtzfun_terminate.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 23-Apr-2017 17:19:12
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "helmholtzfun.h"
#include "helmholtzfun_terminate.h"
#include "helmholtzfun_data.h"

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void helmholtzfun_terminate(void)
{
  omp_destroy_nest_lock(&emlrtNestLockGlobal);
}

/*
 * File trailer for helmholtzfun_terminate.c
 *
 * [EOF]
 */
