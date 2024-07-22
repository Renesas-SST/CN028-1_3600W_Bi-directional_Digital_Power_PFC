/*
 * digitalPower.h
 *
 *  Created on: 2023年3月30日
 *      Author: a5073522
 */

#ifndef DIGITPOW_H_
#define DIGITPOW_H_

#include <digitPow_lib.h>

typedef struct para
{
    float Vdc_tgt;			/* The setting value of the voltage of DC */
    float Vac_rms_tgt;		/* The setting value of the voltage of AC */
    uint16_t Fac_tgt;		/* The setting value of the frequency of AC */

    float Theta;			/* The theta of AC obtained by PLL */
    float Ipeak;			/* The AC peak current from voltage loop */
    float Iref;				/* The reference current for current loop */
    float Vdc_flt;			/* The filter value of Vdc_adc */
    float Vac_ref;			/* The reference voltage for voltage loop */
    float Theta_ref;		/* The theta for AC output */
    float Vout;				/* The output from current loop */

    float monitor_Fac;		/* The frequency of AC obtained by PLL */
    float monitor_Vac_rms;	/* The RMS voltage of AC obtained by PLL */
    float monitor_Iac_rms;	/* The RMS current of AC */
    float monitor_Pac;		/* The AC power */

    float Vac_adc, Vdc_adc, Iac_adc, Temp_adc;          //measured by ADC
    float Vac_adc_old, Vdc_adc_old;
    uint32_t Vac_adc_raw, Vdc_adc_raw, Iac_adc_raw, Temp_adc_raw;
    float Vac_adc_vol, Vdc_adc_vol, Iac_adc_vol;

    uint16_t TH_Vac_rms_Min, TH_Vac_rms_Max;
}t_para;

typedef enum
{
    APP_PFC = 0,
    APP_GRID_CONNECTED_INVERTER = 1,
    APP_OFF_GRID_INVERTER = 2,
}app;

typedef enum
{
    STANDBY_STATE = 0,
    PLL_SYNC_STATE = 1,
    RUN_STATE = 2,
    FAULT_STATE = 3,
	WAIT_TO_RUN_STATE = 4,
}state;

typedef struct
{
    union
    {
        uint16_t byte;
        struct
        {
            uint8_t IAC_POE : 1;    //bit-0
            uint8_t IAC_OCP : 1;
            uint8_t VDC_POE : 1;
            uint8_t VDC_OVP : 1;
            uint8_t VDC_UVP : 1;
            uint8_t VAC_ERR : 1;
            uint8_t FAC_ERR : 1;
            uint8_t PLL_ERR : 1;
            uint8_t OTP     : 1;
            uint8_t OLP     : 1;
            uint8_t INNER_ERR:1;
            uint8_t         : 5;
        }bit;
    };
}t_err;

typedef struct sys_flag
{
    uint8_t state;
    uint8_t app;
    uint8_t f_pwm_start;
    uint8_t cross_zero;
    t_err err;
}t_sys_flag;

/* Filter data structure */
typedef struct
{
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
}t_iir_pfst;

typedef t_AntiSatPID *t_AntiSatPID_handle;

#define PLL_PID_DEFAULTS            \
{   0,                              \
    0,                              \
    0,                              \
    0,                              \
    92,                             \
    (float)(4232*Ts),               \
    0,                              \
    0,                              \
    0,                              \
    0,                              \
    (float)(60*2*M_PI),             \
    (float)(-60*2*M_PI),            \
	0,                              \
	0,                              \
    (void (*)())AntiSatPID_calc,    \
}

typedef struct
{
    uint8_t Enable;     //Enable bit
    float   Input;      //Input value
    float   Out;        //Output value
    float   OutLst;     //Previous output value
    float   MaxOut;     //Maximum threshold
    float   MinOut;     //Minimum threshold
    float   ACC_MaxStp; //Accelerate ramp step
    float   DEC_MaxStp; //Decelerate ramp step
    float   Init;       //Initial value
    void (*calc)();     //Point to ramp calculation function
} t_RAMP2;
typedef t_RAMP2 *t_RAMP2_handle;
#define RAMP2_DEFAULTS          \
{   0,                          \
    0,                          \
    0,                          \
    0,                          \
    0,                          \
    0,                          \
    0,                          \
    0,                          \
    0,                          \
    (void (*)(u32))RAMP2_calc   \
}

void dp_para_init(void);
void notch_flt_para_init(float Wc, float Wbw, t_iir_pfst * n);
void task_10kHz(void);
void task_66kHz(void);
void fast_protection(void);
void slow_protection(void);
void dp_routine(void);
float iir_filter(t_iir_pfst * n);

extern t_para g_para;
extern t_sys_flag g_sys;
extern t_iir_pfst notchflt_for_vdc;
extern t_SOGI g_sogi;
extern t_AntiSatPID g_pll_pi;
extern t_PR g_vloop_pr;
extern t_PR g_iloop_pr;

#endif /* DIGITPOW_H_ */
