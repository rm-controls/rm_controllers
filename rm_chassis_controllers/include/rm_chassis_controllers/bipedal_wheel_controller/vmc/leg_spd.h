/*
 * File: leg_spd.h
 *
 * MATLAB Coder version            : 5.5
 * C/C++ source code generated on  : 10-Aug-2025 14:30:13
 */

#ifndef LEG_SPD_H
#define LEG_SPD_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void CalcJacobian(double phi1, double phi4, double J_[2][2]);
extern void leg_spd(double dphi1, double dphi4, double phi1, double phi4, double spd[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for leg_spd.h
 *
 * [EOF]
 */
