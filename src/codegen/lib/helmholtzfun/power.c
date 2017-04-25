/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: power.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 23-Apr-2017 16:57:58
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "helmholtzfun.h"
#include "power.h"
#include "helmholtzfun_emxutil.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_real_T *a
 *                emxArray_real_T *y
 * Return Type  : void
 */
void power(const emxArray_real_T *a, emxArray_real_T *y)
{
  emxArray_real_T *x;
  int ub_loop;
  int loop_ub;
  int k;
  emxInit_real_T(&x, 2);
  ub_loop = x->size[0] * x->size[1];
  x->size[0] = 1;
  x->size[1] = a->size[1];
  emxEnsureCapacity((emxArray__common *)x, ub_loop, (int)sizeof(double));
  loop_ub = a->size[0] * a->size[1];
  for (ub_loop = 0; ub_loop < loop_ub; ub_loop++) {
    x->data[ub_loop] = a->data[ub_loop];
  }

  ub_loop = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = a->size[1];
  emxEnsureCapacity((emxArray__common *)y, ub_loop, (int)sizeof(double));
  ub_loop = a->size[1];

#pragma omp parallel for \
 num_threads(omp_get_max_threads())

  for (k = 1; k <= ub_loop; k++) {
    y->data[k - 1] = x->data[k - 1] * x->data[k - 1];
  }

  emxFree_real_T(&x);
}

/*
 * File trailer for power.c
 *
 * [EOF]
 */
