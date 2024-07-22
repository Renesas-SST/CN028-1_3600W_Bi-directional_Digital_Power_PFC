/*
 * digitalPower_lib.h
 *
 *  Created on: 2022年12月27日
 *      Author: a5073522
 */

#ifndef DIGITPOW_LIB_H_
#define DIGITPOW_LIB_H_

/* Filter data structure */
typedef struct
{
    float AlphaYn;
    float AlphaYn_1;
    float AlphaYn_2;
    float BetaYn;
    float BetaYn_1;
    float BetaYn_2;
    float Xn;
    float Xn_1;
    float Xn_2;

    /* denominator coefficient */
    float A0;
    float A1;
    float A2;
    /* numerator coefficient */
    float aB0;
    float aB1;
    float aB2;
    float bB0;
    float bB1;
    float bB2;
}t_SOGI;

typedef struct
{
    float Kp;
    float Kr;

    float OutMax;
    float OutMin;

    float Yn;
    float Yn_1;
    float Yn_2;
    float Xn;
    float Xn_1;
    float Xn_2;

    /* denominator coefficient */
    float A0;
    float A1;
    float A2;
    /* numerator coefficient */
    float B0;
    float B1;
    float B2;
}t_PR;

/* PID structure */
typedef struct
{
    float  Ref;            // reference value
    float  Fdb;            // feedback value
    float  Err;            // error
    float  SatErr;         // integration saturation error
    float  Kp;             // proportion gain
    float  Ki;             // integration gain
    float  Kd;             // differential gain
    float  P_Out;          // output value of proportion part
    float  I_Out;          // output value of integration part
    float  D_Out;          // output value of differential part
    float  OutMax;         // maximum threshold
    float  OutMin;         // minimum threshold
    float  OutPreSat;      // output value before threshold comparison
    float  Out;            // PID output value
    void  (*calc)();       // Point to PID calculation function
}t_AntiSatPID;

void AntiSatPID_calc(t_AntiSatPID *v);
void PR_para_init(float T, float Fac, t_PR * n);
void PR_calc(t_PR * n, float * out);
void duty_calculation(float * dutyH, float * dutyL, float Vout, float Vdc, float * vout_last);
void PLL(float T, t_SOGI * n, t_AntiSatPID * pi, float Fac_set, float * Theta, float * Vac_rms, float * Fac_get);

#endif /* DIGITPOW_LIB_H_ */
