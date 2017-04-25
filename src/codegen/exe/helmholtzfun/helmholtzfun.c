/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: helmholtzfun.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 23-Apr-2017 17:19:12
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "helmholtzfun.h"
#include "helmholtzfun_emxutil.h"
#include "abs.h"
#include "rdivide.h"
#include "ifWhileCond.h"
#include "cos.h"
#include "sin.h"
#include "power.h"

/* Function Definitions */

/*
 * Dakota Dalton - 1366027
 * Scientific Computing - MECE 5397
 * Implementation of Helmholtz Equation in 2D - Semester Project
 * Project code AHc2-1
 * clearvars; %clc;
 *  Given values and Boundary Conditions
 * Arguments    : double delta
 *                emxArray_real_T *u
 *                emxArray_real_T *SORu
 * Return Type  : void
 */
void helmholtzfun(double delta, emxArray_real_T *u, emxArray_real_T *SORu)
{
  emxArray_real_T *x;
  int ixstop;
  double ndbl;
  double apnd;
  double cdiff;
  emxArray_real_T *y;
  int n;
  emxArray_real_T *r0;
  int k;
  int nm1d2;
  emxArray_real_T *gb;
  emxArray_real_T *maxval;
  emxArray_real_T *r1;
  emxArray_real_T *fb;
  emxArray_real_T *F;
  emxArray_int32_T *r2;
  emxArray_real_T *b_gb;
  emxArray_real_T *epsilon;
  emxArray_real_T *varargin_2;
  emxArray_boolean_T *b_epsilon;
  emxArray_real_T *b_u;
  int exitg4;
  int i;
  boolean_T exitg5;
  boolean_T exitg6;
  emxArray_real_T *c_gb;
  emxArray_boolean_T *c_epsilon;
  emxArray_real_T *b_SORu;
  int exitg1;
  boolean_T exitg2;
  boolean_T exitg3;

  /*  start = 0.15; */
  /*  step = -0.005; */
  /*  stop = 0.01; */
  /* for i = start:step:stop */
  /* given domain limits, these form a rectangle */
  /* given value for lambda */
  /* lambda = 0; */
  /* delta = 0.015; %step size, same for both x and y */
  emxInit_real_T(&x, 2);
  if (rtIsNaN(delta)) {
    ixstop = x->size[0] * x->size[1];
    x->size[0] = 1;
    x->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)x, ixstop, (int)sizeof(double));
    x->data[0] = rtNaN;
  } else if ((delta == 0.0) || (delta < 0.0)) {
    ixstop = x->size[0] * x->size[1];
    x->size[0] = 1;
    x->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)x, ixstop, (int)sizeof(double));
  } else if (rtIsInf(delta)) {
    ixstop = x->size[0] * x->size[1];
    x->size[0] = 1;
    x->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)x, ixstop, (int)sizeof(double));
    x->data[0] = -3.1415926535897931;
  } else {
    ndbl = floor(6.2831853071795862 / delta + 0.5);
    apnd = -3.1415926535897931 + ndbl * delta;
    if (delta > 0.0) {
      cdiff = apnd - 3.1415926535897931;
    } else {
      cdiff = 3.1415926535897931 - apnd;
    }

    if (fabs(cdiff) < 1.3951473992034527E-15) {
      ndbl++;
      apnd = 3.1415926535897931;
    } else if (cdiff > 0.0) {
      apnd = -3.1415926535897931 + (ndbl - 1.0) * delta;
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      n = (int)ndbl;
    } else {
      n = 0;
    }

    ixstop = x->size[0] * x->size[1];
    x->size[0] = 1;
    x->size[1] = n;
    emxEnsureCapacity((emxArray__common *)x, ixstop, (int)sizeof(double));
    if (n > 0) {
      x->data[0] = -3.1415926535897931;
      if (n > 1) {
        x->data[n - 1] = apnd;
        nm1d2 = (n - 1) / 2;
        for (k = 1; k < nm1d2; k++) {
          ndbl = (double)k * delta;
          x->data[k] = -3.1415926535897931 + ndbl;
          x->data[(n - k) - 1] = apnd - ndbl;
        }

        if (nm1d2 << 1 == n - 1) {
          x->data[nm1d2] = (-3.1415926535897931 + apnd) / 2.0;
        } else {
          ndbl = (double)nm1d2 * delta;
          x->data[nm1d2] = -3.1415926535897931 + ndbl;
          x->data[nm1d2 + 1] = apnd - ndbl;
        }
      }
    }
  }

  /* discretizing the domain */
  emxInit_real_T(&y, 2);
  if (rtIsNaN(delta)) {
    ixstop = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)y, ixstop, (int)sizeof(double));
    y->data[0] = rtNaN;
  } else if ((delta == 0.0) || (delta < 0.0)) {
    ixstop = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)y, ixstop, (int)sizeof(double));
  } else if (rtIsInf(delta)) {
    ixstop = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)y, ixstop, (int)sizeof(double));
    y->data[0] = -3.1415926535897931;
  } else {
    ndbl = floor(6.2831853071795862 / delta + 0.5);
    apnd = -3.1415926535897931 + ndbl * delta;
    if (delta > 0.0) {
      cdiff = apnd - 3.1415926535897931;
    } else {
      cdiff = 3.1415926535897931 - apnd;
    }

    if (fabs(cdiff) < 1.3951473992034527E-15) {
      ndbl++;
      apnd = 3.1415926535897931;
    } else if (cdiff > 0.0) {
      apnd = -3.1415926535897931 + (ndbl - 1.0) * delta;
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      n = (int)ndbl;
    } else {
      n = 0;
    }

    ixstop = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = n;
    emxEnsureCapacity((emxArray__common *)y, ixstop, (int)sizeof(double));
    if (n > 0) {
      y->data[0] = -3.1415926535897931;
      if (n > 1) {
        y->data[n - 1] = apnd;
        nm1d2 = (n - 1) / 2;
        for (k = 1; k < nm1d2; k++) {
          ndbl = (double)k * delta;
          y->data[k] = -3.1415926535897931 + ndbl;
          y->data[(n - k) - 1] = apnd - ndbl;
        }

        if (nm1d2 << 1 == n - 1) {
          y->data[nm1d2] = (-3.1415926535897931 + apnd) / 2.0;
        } else {
          ndbl = (double)nm1d2 * delta;
          y->data[nm1d2] = -3.1415926535897931 + ndbl;
          y->data[nm1d2 + 1] = apnd - ndbl;
        }
      }
    }
  }

  emxInit_real_T(&r0, 2);
  ixstop = r0->size[0] * r0->size[1];
  r0->size[0] = 1;
  r0->size[1] = x->size[1];
  emxEnsureCapacity((emxArray__common *)r0, ixstop, (int)sizeof(double));
  k = x->size[0] * x->size[1];
  for (ixstop = 0; ixstop < k; ixstop++) {
    r0->data[ixstop] = 3.1415926535897931 - x->data[ixstop];
  }

  emxInit_real_T(&gb, 2);
  emxInit_real_T(&maxval, 2);
  power(r0, gb);
  ixstop = maxval->size[0] * maxval->size[1];
  maxval->size[0] = 1;
  maxval->size[1] = x->size[1];
  emxEnsureCapacity((emxArray__common *)maxval, ixstop, (int)sizeof(double));
  k = x->size[0] * x->size[1];
  emxFree_real_T(&r0);
  for (ixstop = 0; ixstop < k; ixstop++) {
    maxval->data[ixstop] = 3.1415926535897931 * x->data[ixstop] /
      3.1415926535897931;
  }

  b_cos(maxval);
  ixstop = gb->size[0] * gb->size[1];
  gb->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)gb, ixstop, (int)sizeof(double));
  nm1d2 = gb->size[0];
  k = gb->size[1];
  k *= nm1d2;
  for (ixstop = 0; ixstop < k; ixstop++) {
    gb->data[ixstop] *= maxval->data[ixstop];
  }

  emxInit_real_T(&r1, 2);

  /* boundary conditions for y */
  ixstop = r1->size[0] * r1->size[1];
  r1->size[0] = 1;
  r1->size[1] = x->size[1];
  emxEnsureCapacity((emxArray__common *)r1, ixstop, (int)sizeof(double));
  k = x->size[0] * x->size[1];
  for (ixstop = 0; ixstop < k; ixstop++) {
    r1->data[ixstop] = 3.1415926535897931 - x->data[ixstop];
  }

  emxInit_real_T(&fb, 2);
  power(r1, fb);
  ixstop = fb->size[0] * fb->size[1];
  fb->size[0] = 1;
  fb->size[1] = x->size[1];
  emxEnsureCapacity((emxArray__common *)fb, ixstop, (int)sizeof(double));
  k = x->size[0] * x->size[1];
  emxFree_real_T(&r1);
  for (ixstop = 0; ixstop < k; ixstop++) {
    fb->data[ixstop] *= x->data[ixstop];
  }

  emxInit_real_T(&F, 2);

  /*  %applied force */
  ixstop = F->size[0] * F->size[1];
  F->size[0] = 1;
  F->size[1] = x->size[1];
  emxEnsureCapacity((emxArray__common *)F, ixstop, (int)sizeof(double));
  k = x->size[0] * x->size[1];
  for (ixstop = 0; ixstop < k; ixstop++) {
    F->data[ixstop] = 3.1415926535897931 * (x->data[ixstop] -
      -3.1415926535897931) / 6.2831853071795862;
  }

  b_sin(F);
  ixstop = maxval->size[0] * maxval->size[1];
  maxval->size[0] = 1;
  maxval->size[1] = y->size[1];
  emxEnsureCapacity((emxArray__common *)maxval, ixstop, (int)sizeof(double));
  k = y->size[0] * y->size[1];
  for (ixstop = 0; ixstop < k; ixstop++) {
    maxval->data[ixstop] = 1.5707963267948966 * (2.0 * (y->data[ixstop] -
      -3.1415926535897931) / 6.2831853071795862 + 1.0);
  }

  b_cos(maxval);
  ixstop = F->size[0] * F->size[1];
  F->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)F, ixstop, (int)sizeof(double));
  nm1d2 = F->size[0];
  k = F->size[1];
  k *= nm1d2;
  for (ixstop = 0; ixstop < k; ixstop++) {
    F->data[ixstop] *= maxval->data[ixstop];
  }

  /* F = 0; */
  /*  Gauss-Seidel/Liebmann method */
  ixstop = u->size[0] * u->size[1];
  u->size[0] = x->size[1];
  u->size[1] = x->size[1];
  emxEnsureCapacity((emxArray__common *)u, ixstop, (int)sizeof(double));
  k = x->size[1] * x->size[1];
  for (ixstop = 0; ixstop < k; ixstop++) {
    u->data[ixstop] = 0.0;
  }

  /* initial values of u to be iterated over */
  k = gb->size[1];
  for (ixstop = 0; ixstop < k; ixstop++) {
    u->data[u->size[0] * ixstop] = gb->data[gb->size[0] * ixstop];
  }

  /* boundary condition for y (bottom) */
  nm1d2 = u->size[0] - 1;
  k = fb->size[1];
  for (ixstop = 0; ixstop < k; ixstop++) {
    u->data[nm1d2 + u->size[0] * ixstop] = fb->data[fb->size[0] * ixstop];
  }

  emxInit_int32_T(&r2, 1);

  /* boundary condition for y (top) */
  k = u->size[0];
  ixstop = r2->size[0];
  r2->size[0] = k;
  emxEnsureCapacity((emxArray__common *)r2, ixstop, (int)sizeof(int));
  for (ixstop = 0; ixstop < k; ixstop++) {
    r2->data[ixstop] = ixstop;
  }

  emxInit_real_T(&b_gb, 2);
  ndbl = fb->data[0] - gb->data[0];
  cdiff = gb->data[0];
  ixstop = b_gb->size[0] * b_gb->size[1];
  b_gb->size[0] = 1;
  b_gb->size[1] = y->size[1];
  emxEnsureCapacity((emxArray__common *)b_gb, ixstop, (int)sizeof(double));
  k = y->size[1];
  for (ixstop = 0; ixstop < k; ixstop++) {
    b_gb->data[b_gb->size[0] * ixstop] = cdiff + (y->data[y->size[0] * ixstop] -
      -3.1415926535897931) / 6.2831853071795862 * ndbl;
  }

  nm1d2 = r2->size[0];
  for (ixstop = 0; ixstop < nm1d2; ixstop++) {
    u->data[r2->data[ixstop]] = b_gb->data[ixstop];
  }

  emxFree_real_T(&b_gb);
  emxInit_real_T(&epsilon, 2);

  /* bc for x (left) */
  /* used to count number of iterations */
  ixstop = epsilon->size[0] * epsilon->size[1];
  epsilon->size[0] = x->size[1];
  epsilon->size[1] = x->size[1];
  emxEnsureCapacity((emxArray__common *)epsilon, ixstop, (int)sizeof(double));
  k = x->size[1] * x->size[1];
  for (ixstop = 0; ixstop < k; ixstop++) {
    epsilon->data[ixstop] = 1.0;
  }

  /* calculating relative change per iteration */
  /*  ucon = u; */
  /*  loop = 1; */
  /*  dim = size(u); */
  /* while converge1 ~= converge2 */
  /* for i = start:step:stop         */
  /* converge2 = converge1; */
  /* delta = i */
  emxInit_real_T(&varargin_2, 2);
  emxInit_boolean_T(&b_epsilon, 2);
  emxInit_real_T(&b_u, 2);
  do {
    exitg4 = 0;
    ixstop = b_epsilon->size[0] * b_epsilon->size[1];
    b_epsilon->size[0] = epsilon->size[0];
    b_epsilon->size[1] = epsilon->size[1];
    emxEnsureCapacity((emxArray__common *)b_epsilon, ixstop, (int)sizeof
                      (boolean_T));
    k = epsilon->size[0] * epsilon->size[1];
    for (ixstop = 0; ixstop < k; ixstop++) {
      b_epsilon->data[ixstop] = (epsilon->data[ixstop] > 0.01);
    }

    if (ifWhileCond(b_epsilon)) {
      ixstop = epsilon->size[0] * epsilon->size[1];
      epsilon->size[0] = u->size[0];
      epsilon->size[1] = u->size[1];
      emxEnsureCapacity((emxArray__common *)epsilon, ixstop, (int)sizeof(double));
      k = u->size[0] * u->size[1];
      for (ixstop = 0; ixstop < k; ixstop++) {
        epsilon->data[ixstop] = u->data[ixstop];
      }

      /* iteration reference */
      /* sweeping through columns and rows, iterating values */
      for (i = 1; i - 1 <= y->size[1] - 3; i++) {
        for (nm1d2 = 1; nm1d2 - 1 <= x->size[1] - 3; nm1d2++) {
          u->data[nm1d2 + u->size[0] * i] = ((((u->data[(nm1d2 + u->size[0] * i)
            + 1] + u->data[(nm1d2 + u->size[0] * i) - 1]) + u->data[nm1d2 +
            u->size[0] * (1 + i)]) + u->data[nm1d2 + u->size[0] * (i - 1)]) -
            delta * delta * F->data[nm1d2]) * (1.0 / (4.0 - delta * delta));
        }

        /* right side x bc is a Neumann condition (insulated) */
        u->data[i + u->size[0] * (u->size[1] - 1)] = (((2.0 * u->data[i +
          u->size[0] * (u->size[1] - 2)] + u->data[(i + u->size[0] * (u->size[1]
          - 1)) + 1]) + u->data[(i + u->size[0] * (u->size[1] - 1)) - 1]) -
          delta * delta * F->data[i]) * (1.0 / (4.0 - delta * delta));
      }

      /* the largest value change in the matrix will determine the epsilon */
      ixstop = b_u->size[0] * b_u->size[1];
      b_u->size[0] = u->size[0];
      b_u->size[1] = u->size[1];
      emxEnsureCapacity((emxArray__common *)b_u, ixstop, (int)sizeof(double));
      k = u->size[0] * u->size[1];
      for (ixstop = 0; ixstop < k; ixstop++) {
        b_u->data[ixstop] = u->data[ixstop] - epsilon->data[ixstop];
      }

      rdivide(b_u, u, epsilon);
      b_abs(epsilon, varargin_2);
      ixstop = maxval->size[0] * maxval->size[1];
      maxval->size[0] = 1;
      maxval->size[1] = varargin_2->size[1];
      emxEnsureCapacity((emxArray__common *)maxval, ixstop, (int)sizeof(double));
      n = varargin_2->size[0];
      for (i = 0; i + 1 <= varargin_2->size[1]; i++) {
        k = i * n;
        nm1d2 = i * n + 1;
        ixstop = k + n;
        ndbl = varargin_2->data[k];
        if (n > 1) {
          if (rtIsNaN(varargin_2->data[k])) {
            k = nm1d2 + 1;
            exitg6 = false;
            while ((!exitg6) && (k <= ixstop)) {
              nm1d2 = k;
              if (!rtIsNaN(varargin_2->data[k - 1])) {
                ndbl = varargin_2->data[k - 1];
                exitg6 = true;
              } else {
                k++;
              }
            }
          }

          if (nm1d2 < ixstop) {
            while (nm1d2 + 1 <= ixstop) {
              if (varargin_2->data[nm1d2] > ndbl) {
                ndbl = varargin_2->data[nm1d2];
              }

              nm1d2++;
            }
          }
        }

        maxval->data[i] = ndbl;
      }

      nm1d2 = 1;
      n = maxval->size[1];
      ndbl = maxval->data[0];
      if (maxval->size[1] > 1) {
        if (rtIsNaN(maxval->data[0])) {
          k = 2;
          exitg5 = false;
          while ((!exitg5) && (k <= n)) {
            nm1d2 = k;
            if (!rtIsNaN(maxval->data[k - 1])) {
              ndbl = maxval->data[k - 1];
              exitg5 = true;
            } else {
              k++;
            }
          }
        }

        if (nm1d2 < maxval->size[1]) {
          while (nm1d2 + 1 <= n) {
            if (maxval->data[nm1d2] > ndbl) {
              ndbl = maxval->data[nm1d2];
            }

            nm1d2++;
          }
        }
      }

      ixstop = epsilon->size[0] * epsilon->size[1];
      epsilon->size[0] = 1;
      epsilon->size[1] = 1;
      emxEnsureCapacity((emxArray__common *)epsilon, ixstop, (int)sizeof(double));
      epsilon->data[0] = ndbl;

      /* counting the iterations */
      /*          if mod(iter,100) == 0 */
      /*              disp(iter) */
      /*              disp(epsilon) */
      /*              iterarray = [iterarray, iter]; */
      /*              epsarray = [epsarray, epsilon]; */
      /*          end */
    } else {
      exitg4 = 1;
    }
  } while (exitg4 == 0);

  emxFree_real_T(&b_u);
  emxFree_boolean_T(&b_epsilon);

  /* disp(min(min(u))) */
  /*      loop = loop + 1; */
  /*      ucon = [ucon, u]; */
  /*       */
  /*      rowsub  = 1:dim(1); */
  /*      colsub1 = (loop - 1) * dim(2) + 1 : loop * dim(2); */
  /*      colsub2 = (loop - 2) * dim(2) + 1 : (loop - 1) * dim(2);    */
  /*           */
  /*      if ucon(rowsub,colsub1) == ucon(rowsub, colsub2) ... */
  /*         & isfinite(ucon(rowsub, colsub1))  */
  /*          disp(start-loop*abs(step)) */
  /*          break */
  /*      end */
  /* end */
  /*  Gauss-Seidel with Successive Overrelaxtion (SOR) */
  /* coefficient to expedite convergence */
  ixstop = SORu->size[0] * SORu->size[1];
  SORu->size[0] = x->size[1];
  SORu->size[1] = x->size[1];
  emxEnsureCapacity((emxArray__common *)SORu, ixstop, (int)sizeof(double));
  k = x->size[1] * x->size[1];
  for (ixstop = 0; ixstop < k; ixstop++) {
    SORu->data[ixstop] = 0.0;
  }

  /* initial values of u to be iterated over */
  k = gb->size[1];
  for (ixstop = 0; ixstop < k; ixstop++) {
    SORu->data[SORu->size[0] * ixstop] = gb->data[gb->size[0] * ixstop];
  }

  /* boundary condition for y (bottom) */
  nm1d2 = SORu->size[0] - 1;
  k = fb->size[1];
  for (ixstop = 0; ixstop < k; ixstop++) {
    SORu->data[nm1d2 + SORu->size[0] * ixstop] = fb->data[fb->size[0] * ixstop];
  }

  /* boundary condition for y (top) */
  k = SORu->size[0];
  ixstop = r2->size[0];
  r2->size[0] = k;
  emxEnsureCapacity((emxArray__common *)r2, ixstop, (int)sizeof(int));
  for (ixstop = 0; ixstop < k; ixstop++) {
    r2->data[ixstop] = ixstop;
  }

  emxInit_real_T(&c_gb, 2);
  ndbl = fb->data[0] - gb->data[0];
  cdiff = gb->data[0];
  ixstop = c_gb->size[0] * c_gb->size[1];
  c_gb->size[0] = 1;
  c_gb->size[1] = y->size[1];
  emxEnsureCapacity((emxArray__common *)c_gb, ixstop, (int)sizeof(double));
  k = y->size[1];
  emxFree_real_T(&fb);
  emxFree_real_T(&gb);
  for (ixstop = 0; ixstop < k; ixstop++) {
    c_gb->data[c_gb->size[0] * ixstop] = cdiff + (y->data[y->size[0] * ixstop] -
      -3.1415926535897931) / 6.2831853071795862 * ndbl;
  }

  nm1d2 = r2->size[0];
  for (ixstop = 0; ixstop < nm1d2; ixstop++) {
    SORu->data[r2->data[ixstop]] = c_gb->data[ixstop];
  }

  emxFree_real_T(&c_gb);
  emxFree_int32_T(&r2);

  /* bc for x (left) */
  /* used to count number of iterations */
  ixstop = epsilon->size[0] * epsilon->size[1];
  epsilon->size[0] = x->size[1];
  epsilon->size[1] = x->size[1];
  emxEnsureCapacity((emxArray__common *)epsilon, ixstop, (int)sizeof(double));
  k = x->size[1] * x->size[1];
  for (ixstop = 0; ixstop < k; ixstop++) {
    epsilon->data[ixstop] = 1.0;
  }

  /* calculating relative change per iteration */
  emxInit_boolean_T(&c_epsilon, 2);
  emxInit_real_T(&b_SORu, 2);
  do {
    exitg1 = 0;
    ixstop = c_epsilon->size[0] * c_epsilon->size[1];
    c_epsilon->size[0] = epsilon->size[0];
    c_epsilon->size[1] = epsilon->size[1];
    emxEnsureCapacity((emxArray__common *)c_epsilon, ixstop, (int)sizeof
                      (boolean_T));
    k = epsilon->size[0] * epsilon->size[1];
    for (ixstop = 0; ixstop < k; ixstop++) {
      c_epsilon->data[ixstop] = (epsilon->data[ixstop] > 0.01);
    }

    if (ifWhileCond(c_epsilon)) {
      ixstop = epsilon->size[0] * epsilon->size[1];
      epsilon->size[0] = SORu->size[0];
      epsilon->size[1] = SORu->size[1];
      emxEnsureCapacity((emxArray__common *)epsilon, ixstop, (int)sizeof(double));
      k = SORu->size[0] * SORu->size[1];
      for (ixstop = 0; ixstop < k; ixstop++) {
        epsilon->data[ixstop] = SORu->data[ixstop];
      }

      /* iteration reference */
      /* sweeping through columns and rows, iterating values */
      for (i = 1; i - 1 <= y->size[1] - 3; i++) {
        for (nm1d2 = 1; nm1d2 - 1 <= x->size[1] - 3; nm1d2++) {
          SORu->data[nm1d2 + SORu->size[0] * i] = ((((SORu->data[(nm1d2 +
            SORu->size[0] * i) + 1] + SORu->data[(nm1d2 + SORu->size[0] * i) - 1])
            + SORu->data[nm1d2 + SORu->size[0] * (1 + i)]) + SORu->data[nm1d2 +
            SORu->size[0] * (i - 1)]) - delta * delta * F->data[nm1d2]) * (1.0 /
            (4.0 - delta * delta));
          SORu->data[nm1d2 + SORu->size[0] * i] = 1.2 * SORu->data[nm1d2 +
            SORu->size[0] * i] + -0.19999999999999996 * epsilon->data[nm1d2 +
            epsilon->size[0] * i];
        }

        /* right side x bc is a Neumann condition (insulated) */
        SORu->data[i + SORu->size[0] * (SORu->size[1] - 1)] = (((2.0 *
          SORu->data[i + SORu->size[0] * (SORu->size[1] - 2)] + SORu->data[(i +
          SORu->size[0] * (SORu->size[1] - 1)) + 1]) + SORu->data[(i +
          SORu->size[0] * (SORu->size[1] - 1)) - 1]) - delta * delta * F->data[i])
          * (1.0 / (4.0 - delta * delta));
        SORu->data[i + SORu->size[0] * (SORu->size[1] - 1)] = 1.2 * SORu->data[i
          + SORu->size[0] * (SORu->size[1] - 1)] + -0.19999999999999996 *
          epsilon->data[i + epsilon->size[0] * (epsilon->size[1] - 1)];
      }

      /* the largest value change in the matrix will determine the epsilon */
      ixstop = b_SORu->size[0] * b_SORu->size[1];
      b_SORu->size[0] = SORu->size[0];
      b_SORu->size[1] = SORu->size[1];
      emxEnsureCapacity((emxArray__common *)b_SORu, ixstop, (int)sizeof(double));
      k = SORu->size[0] * SORu->size[1];
      for (ixstop = 0; ixstop < k; ixstop++) {
        b_SORu->data[ixstop] = SORu->data[ixstop] - epsilon->data[ixstop];
      }

      rdivide(b_SORu, SORu, varargin_2);
      ixstop = maxval->size[0] * maxval->size[1];
      maxval->size[0] = 1;
      maxval->size[1] = varargin_2->size[1];
      emxEnsureCapacity((emxArray__common *)maxval, ixstop, (int)sizeof(double));
      n = varargin_2->size[0];
      for (i = 0; i + 1 <= varargin_2->size[1]; i++) {
        k = i * n;
        nm1d2 = i * n + 1;
        ixstop = k + n;
        ndbl = varargin_2->data[k];
        if (n > 1) {
          if (rtIsNaN(varargin_2->data[k])) {
            k = nm1d2 + 1;
            exitg3 = false;
            while ((!exitg3) && (k <= ixstop)) {
              nm1d2 = k;
              if (!rtIsNaN(varargin_2->data[k - 1])) {
                ndbl = varargin_2->data[k - 1];
                exitg3 = true;
              } else {
                k++;
              }
            }
          }

          if (nm1d2 < ixstop) {
            while (nm1d2 + 1 <= ixstop) {
              if (varargin_2->data[nm1d2] > ndbl) {
                ndbl = varargin_2->data[nm1d2];
              }

              nm1d2++;
            }
          }
        }

        maxval->data[i] = ndbl;
      }

      nm1d2 = 1;
      n = maxval->size[1];
      ndbl = maxval->data[0];
      if (maxval->size[1] > 1) {
        if (rtIsNaN(maxval->data[0])) {
          k = 2;
          exitg2 = false;
          while ((!exitg2) && (k <= n)) {
            nm1d2 = k;
            if (!rtIsNaN(maxval->data[k - 1])) {
              ndbl = maxval->data[k - 1];
              exitg2 = true;
            } else {
              k++;
            }
          }
        }

        if (nm1d2 < maxval->size[1]) {
          while (nm1d2 + 1 <= n) {
            if (maxval->data[nm1d2] > ndbl) {
              ndbl = maxval->data[nm1d2];
            }

            nm1d2++;
          }
        }
      }

      ixstop = epsilon->size[0] * epsilon->size[1];
      epsilon->size[0] = 1;
      epsilon->size[1] = 1;
      emxEnsureCapacity((emxArray__common *)epsilon, ixstop, (int)sizeof(double));
      epsilon->data[0] = fabs(ndbl);

      /* counting the iterations */
      /*      if mod(SORiter,100) == 0 */
      /*          disp(SORiter) */
      /*          disp(SORepsilon) */
      /*          SORiterarray = [SORiterarray, SORiter]; */
      /*          SORepsarray = [SORepsarray, SORepsilon]; */
      /*      end */
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  emxFree_real_T(&b_SORu);
  emxFree_boolean_T(&c_epsilon);
  emxFree_real_T(&varargin_2);
  emxFree_real_T(&maxval);
  emxFree_real_T(&epsilon);
  emxFree_real_T(&F);
  emxFree_real_T(&y);
  emxFree_real_T(&x);

  /*  Output & Visualization */
  /*  disp('Gauss-Seidel iterations:') */
  /*  disp(iter) */
  /* disp('Gauss-Seidel iterations with SOR (' + string(SORlambda) + '):') */
  /*  disp(SORiter) */
  /*  subplot(1,2,1) */
  /*  surface(x,y,u) */
  /*  subplot(1,2,2) */
  /*  surface(x,y,SORu) */
  /*  contour(x,y,u) */
  /*  contour3(x,y,SORu) */
  /*  mesh(x,y,SORu) */
}

/*
 * File trailer for helmholtzfun.c
 *
 * [EOF]
 */
