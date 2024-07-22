/*
 * digitPow.c
 *
 *  Created on: 2022年8月22日
 *      Author: a5073522
 */

#include "hal_data.h"
#include "user_define.h"
#include "ra6t2.h"
#include "console.h"
#include "digitPow.h"

void dp_para_reset(void);
void dataflash_read(void);
void float2Byte(float * value, uint8_t * array);
void dataflash_wrt(void);
void RAMP2_calc(t_RAMP2 *v);
uint32_t narrow_pulse_elimination(uint32_t count, uint32_t period, uint32_t deadt);
void iloop_ctrl(void);
void vloop_ctrl(void);
void err_rst(void);
void dp_start(void);
void dp_stop(void);
float ac_ref_get(float vac_rms);
void PWM_output(float dutyH, float dutyL);
void PWM_modulation(void);

t_para g_para;
t_sys_flag g_sys;
t_AntiSatPID g_pll_pi = PLL_PID_DEFAULTS;
t_SOGI g_sogi;
t_AntiSatPID g_iloop_pi;
t_AntiSatPID g_vloop_pi;
t_PR g_iloop_pr;
t_PR g_vloop_pr;
t_RAMP2 ramp_for_vdc;
t_RAMP2 ramp_for_vac;
t_iir_pfst notchflt_for_vdc;

void dp_para_init(void)
{
	g_sys.app = APP_PFC;
    g_sys.state = STANDBY_STATE;
    g_sys.err.byte = 0;

	g_para.Fac_tgt = 50;
	g_para.Vdc_tgt = 400;
	g_para.Vac_rms_tgt = 220;

	g_para.Theta = 0;
	g_para.Ipeak = 0;
	g_para.Iref = 0;
	g_para.Vout = 0;
	g_para.Vdc_flt = 0;
	g_para.Theta_ref = 0;
	g_para.Vac_ref = 0;

	g_para.monitor_Fac = 0;
	g_para.monitor_Vac_rms = 0;
	g_para.monitor_Iac_rms = 0;
	g_para.monitor_Pac = 0;

    ramp_for_vdc.Enable = 1;
    ramp_for_vdc.ACC_MaxStp = 0.01;
    ramp_for_vdc.DEC_MaxStp = 0.01;
    ramp_for_vdc.MaxOut = 400;

    ramp_for_vac.Enable = 1;
    ramp_for_vac.ACC_MaxStp = 0.3;
    ramp_for_vac.DEC_MaxStp = 0.3;
    ramp_for_vac.MaxOut = 235 * 1.4142;
    ramp_for_vac.MinOut = 0;

    g_iloop_pi.Kp = 10;
    g_iloop_pi.Ki = 0.8;
    g_iloop_pi.OutMax = g_para.Vdc_adc;
    g_iloop_pi.OutMin = -g_para.Vdc_adc;

    g_vloop_pi.Kp = 1;
    g_vloop_pi.Ki = 0.003;
    g_vloop_pi.OutMax = 23;
    g_vloop_pi.OutMin = -23;

    g_iloop_pr.Kp = 10;
    g_iloop_pr.Kr = 52800;
    g_iloop_pr.OutMax = g_para.Vdc_adc;
    g_iloop_pr.OutMin = -g_para.Vdc_adc;

    g_vloop_pr.Kp = 0.01;
    g_vloop_pr.Kr = 20;
    g_vloop_pr.OutMax = 23;
    g_vloop_pr.OutMin = -23;

    notch_flt_para_init(2*M_PI*50*2, 2*M_PI*30, &notchflt_for_vdc);

    //dataflash_read();

    con.pi_iloop_kp = g_iloop_pi.Kp;
    con.pi_iloop_ki = g_iloop_pi.Ki;
    con.pi_vloop_kp = g_vloop_pi.Kp;
    con.pi_vloop_ki = g_vloop_pi.Ki;
    con.pr_iloop_kp = g_iloop_pr.Kp;
    con.pr_iloop_kr = g_iloop_pr.Kr;
    con.pr_vloop_kp = g_vloop_pr.Kp;
    con.pr_vloop_kr = g_vloop_pr.Kr;
    con.sys_app = g_sys.app;
}

void dp_para_reset(void)
{
	g_para.Iref = 0;

    ramp_for_vdc.OutLst = 0;
    ramp_for_vac.OutLst = 0;
    g_para.Theta_ref = 0;

    g_sys.app = con.sys_app;
    g_sys.f_pwm_start = 0;
    g_sys.cross_zero = 0;

#if DEBUG_PRO <= GOI
    g_sys.app = APP_OFF_GRID_INVERTER;
    g_para.Fac_tgt = 50;
    g_para.Vac_rms_tgt = 220;
#elif DEBUG_PRO <= PFC
    g_sys.app = APP_PFC;
    g_para.Vdc_tgt = 400;
#elif DEBUG_PRO <= GTI_CL
    g_sys.app = APP_GRID_CONNECTED_INVERTER;
#endif

    g_iloop_pi.I_Out = 0;
    g_iloop_pi.SatErr = 0;

    g_vloop_pi.I_Out = 0;
    g_vloop_pi.SatErr = 0;

    g_pll_pi.I_Out = 0;
    g_pll_pi.SatErr = 0;

    g_sogi.Xn = 0;
    g_sogi.Xn_1 = 0;
    g_sogi.Xn_2 = 0;
    g_sogi.AlphaYn = 0;
    g_sogi.AlphaYn_1 = 0;
    g_sogi.AlphaYn_2 = 0;
    g_sogi.BetaYn = 0;
    g_sogi.BetaYn_1 = 0;
    g_sogi.BetaYn_2 = 0;

    notchflt_for_vdc.Yn = 0;
    notchflt_for_vdc.Yn_1 = 0;
    notchflt_for_vdc.Yn_2 = 0;
    notchflt_for_vdc.Xn = 0;
    notchflt_for_vdc.Xn_1 = 0;
    notchflt_for_vdc.Xn_2 = 0;

    g_vloop_pr.Yn = 0;
    g_vloop_pr.Yn_1 = 0;
    g_vloop_pr.Yn_2 = 0;
    g_vloop_pr.Xn = 0;
    g_vloop_pr.Xn_1 = 0;
    g_vloop_pr.Xn_2 = 0;

    g_iloop_pr.Yn = 0;
    g_iloop_pr.Yn_1 = 0;
    g_iloop_pr.Yn_2 = 0;
    g_iloop_pr.Xn = 0;
    g_iloop_pr.Xn_1 = 0;
    g_iloop_pr.Xn_2 = 0;

    g_iloop_pi.Kp = con.pi_iloop_kp;
    g_iloop_pi.Ki = con.pi_iloop_ki;
    g_vloop_pi.Kp = con.pi_vloop_kp;
    g_vloop_pi.Ki = con.pi_vloop_ki;
    g_iloop_pr.Kp = con.pr_iloop_kp;
    g_iloop_pr.Kr = con.pr_iloop_kr;
    g_vloop_pr.Kp = con.pr_vloop_kp;
    g_vloop_pr.Kr = con.pr_vloop_kr;

    PR_para_init(Ts, g_para.Fac_tgt, &g_vloop_pr);
    PR_para_init(0.0000151515, g_para.Fac_tgt, &g_iloop_pr);
}

void dataflash_read(void)
{
    float * pf;
    uint32_t * p;
    pf = (float *)DATAFLASH_ADDR;
    p = (uint32_t *)DATAFLASH_ADDR;
    if (*p != 1)
    {
    	return;
    }
    pf++;
    g_iloop_pi.Kp = *pf;
    pf++;
    g_iloop_pi.Ki = *pf;
    pf++;
    g_vloop_pi.Kp = *pf;
    pf++;
    g_vloop_pi.Ki = *pf;
    pf++;
    g_vloop_pr.Kp = *pf;
    pf++;
    g_vloop_pr.Kr = *pf;
}

void float2Byte(float * value, uint8_t * array)
{
    uint8_t * p8;

    p8 = (uint8_t *)value;
    * array = *p8;
    p8++;
    array++;
    * array = *p8;
    p8++;
    array++;
    * array = *p8;
    p8++;
    array++;
    * array = *p8;
}

void dataflash_wrt(void)
{
    uint8_t dataflash_src[DATAFLASH_BYTE];
    dataflash_src[0] = 0x01;
    dataflash_src[1] = 0x00;
    dataflash_src[2] = 0x00;
    dataflash_src[3] = 0x00;
    float2Byte(&g_iloop_pi.Kp, &dataflash_src[4]);
    float2Byte(&g_iloop_pi.Ki, &dataflash_src[8]);
    float2Byte(&g_vloop_pi.Kp, &dataflash_src[12]);
    float2Byte(&g_vloop_pi.Ki, &dataflash_src[16]);
    float2Byte(&g_vloop_pr.Kp, &dataflash_src[20]);
    float2Byte(&g_vloop_pr.Kr, &dataflash_src[24]);

    g_flash0.p_api->open(g_flash0.p_ctrl, g_flash0.p_cfg);
    g_flash0.p_api->erase(g_flash0.p_ctrl, DATAFLASH_ADDR, DATAFLASH_BLOCK_NUM);
    R_BSP_SoftwareDelay(1U, BSP_DELAY_UNITS_SECONDS);
    g_flash0.p_api->write(g_flash0.p_ctrl, (uint32_t)dataflash_src, DATAFLASH_ADDR, DATAFLASH_BYTE);
    R_BSP_SoftwareDelay(1U, BSP_DELAY_UNITS_SECONDS);
    g_flash0.p_api->close(g_flash0.p_ctrl);
    __NOP();
}

void notch_flt_para_init(float Wc, float Wbw, t_iir_pfst * n)
{
    n->A0 = 4 + 4*Wbw*Ts + Wc*Ts*Wc*Ts;
    n->A1 = (2*Wc*Ts*Wc*Ts - 8) / n->A0;
    n->A2 = (4 + Wc*Ts*Wc*Ts - 4*Wbw*Ts) / n->A0;
    n->B0 = (4 + Wc*Ts*Wc*Ts) / n->A0;
    n->B1 = n->A1;
    n->B2 = n->B0;

    n->Yn = 0;
    n->Yn_1 = 0;
    n->Yn_2 = 0;
    n->Xn = 0;
    n->Xn_1 = 0;
    n->Xn_2 = 0;
}

float iir_filter(t_iir_pfst * n)
{
    n->Yn = n->Xn * n->B0 + n->Xn_1 * n->B1 + n->Xn_2 * n->B2 - n->Yn_1 * n->A1 - n->Yn_2 * n->A2;

    n->Yn_2 = n->Yn_1;
    n->Yn_1 = n->Yn;
    n->Xn_2 = n->Xn_1;
    n->Xn_1 = n->Xn;

    return n->Yn;
}

void RAMP2_calc(t_RAMP2 *v)
{
    static float Delta, MaxStp, OutputTemp;

    if(v->Enable)
    {
        Delta = v->Input - v->OutLst;
        if(((v->OutLst>=0)&&(Delta>0)) || ((v->OutLst<0)&&(Delta<0)))
        {
            MaxStp = v->ACC_MaxStp;
        }
        else if(((v->OutLst<0)&&(Delta>0)) || ((v->OutLst>=0)&&(Delta<0)))
        {
            MaxStp = v->DEC_MaxStp;
        }

        if(Delta > MaxStp)
            Delta = MaxStp;
        else if(Delta < -MaxStp)
            Delta = -MaxStp;

        OutputTemp = v->OutLst + Delta;
    }
    else
    {
        OutputTemp = v->Init;
    }

    if(OutputTemp > v->MaxOut)
        v->Out = v->MaxOut;
    else if(OutputTemp < v->MinOut)
        v->Out = v->MinOut;
    else
        v->Out = OutputTemp;
    v->OutLst = v->Out;
}

float ac_ref_get(float ac_rms)
{
	float ac_ref;
	g_para.Theta_ref = (float)(2 * M_PI * g_para.Fac_tgt * Ts + g_para.Theta_ref);
    if (g_para.Theta_ref > 2 * M_PI)
    {
    	g_para.Theta_ref -= (float)(2 * M_PI);
    }
    else if (g_para.Theta_ref < 0)
    {
    	g_para.Theta_ref += (float)(2 * M_PI);
    }
    ac_ref = (float)(ac_rms * 1.414 * R_TFU_sin(g_para.Theta_ref));

    return ac_ref;
}

void vloop_ctrl(void)
{
    float vdc_ref, vac_rms;

    if (g_sys.app != APP_OFF_GRID_INVERTER)
    {
        /* Vref */
        ramp_for_vdc.MinOut = g_para.monitor_Vac_rms * 1.5;
        ramp_for_vdc.Input = g_para.Vdc_tgt;
        RAMP2_calc(&ramp_for_vdc);
        vdc_ref = ramp_for_vdc.Out;
        /* Vfdb */
        notchflt_for_vdc.Xn = g_para.Vdc_adc;
        g_para.Vdc_flt = iir_filter(&notchflt_for_vdc);

        g_vloop_pi.Ref = vdc_ref;
        g_vloop_pi.Fdb = g_para.Vdc_flt;
        AntiSatPID_calc(&g_vloop_pi);
        g_para.Ipeak = g_vloop_pi.Out;
        g_para.Iref = -1 * g_para.Ipeak * R_TFU_cos(g_para.Theta);
    }
    else
    {
    	/* Vref */
        ramp_for_vac.Input = g_para.Vac_rms_tgt;
        RAMP2_calc(&ramp_for_vac);
        vac_rms = ramp_for_vac.Out;
        g_para.Vac_ref = ac_ref_get(vac_rms);

        g_vloop_pr.Xn = g_para.Vac_ref - g_para.Vac_adc;
        PR_calc(&g_vloop_pr, &g_para.Iref);
    }
}

void iloop_ctrl(void)
{
	if (g_sys.app != APP_OFF_GRID_INVERTER)
	{
		g_iloop_pi.Ref = g_para.Iref;
		g_iloop_pi.Fdb = g_para.Iac_adc;
	    g_iloop_pi.OutMax = g_para.Vdc_adc - g_para.Vac_adc;
	    g_iloop_pi.OutMin = -g_para.Vdc_adc - g_para.Vac_adc;
	    AntiSatPID_calc(&g_iloop_pi);
	    g_para.Vout = g_iloop_pi.Out + g_para.Vac_adc;
	}
	else
	{
	    g_iloop_pr.Xn = g_para.Iref - g_para.Iac_adc;
	    g_iloop_pr.OutMax = g_para.Vdc_adc;
	    g_iloop_pr.OutMin = -g_para.Vdc_adc;
	    PR_calc(&g_iloop_pr, &g_para.Vout);
	}
}

void task_10kHz(void)
{
    if (g_sys.app != APP_OFF_GRID_INVERTER)
    {
    	g_sogi.Xn = g_para.Vac_adc;
    	PLL(Ts, &g_sogi, &g_pll_pi, g_para.Fac_tgt, &g_para.Theta, &g_para.monitor_Vac_rms, &g_para.monitor_Fac);
    }
#if DEBUG_PRO == GOI_OLVO
    g_para.Vout = ac_ref_get(50);
#elif DEBUG_PRO == GOI_CL
    g_para.Iref = ac_ref_get(1);
#elif DEBUG_PRO == GTI_CL
    g_para.Ipeak = -3 * 1.414;
    g_para.Iref = -1 * g_para.Ipeak * R_TFU_cos(g_para.Theta);
#else
    vloop_ctrl();
#endif
}

void task_66kHz(void)
{
#if DEBUG_PRO != GOI_OLVO
    iloop_ctrl();
#endif
    PWM_modulation();
}

/* input:  g_para.Vout
 *         g_para.Vdc_adc
*/
void PWM_modulation(void)
{
	static float VoutLast = 0;
    float DutyH, DutyL;//the duty of the upper side

    /* duty calculation */
    duty_calculation(&DutyH, &DutyL, g_para.Vout, g_para.Vdc_adc, &VoutLast);

    /* PWM output */
    PWM_output(DutyH, DutyL);
}

void PWM_output(float dutyH, float dutyL)
{
    uint32_t CHOP_count, RECT_count;

    CHOP_count = (uint32_t)((float)g_CHOP_PWM_period * (1 - dutyH));
    CHOP_count = narrow_pulse_elimination(CHOP_count, g_CHOP_PWM_period, g_CHOP_deadt);

    R_GPT4->GTCCR[4] = CHOP_count;//GTCCRD	//只给GPT4赋值，波形无更新；同时给GPT6赋值，波形更新
    R_GPT5->GTCCR[4] = CHOP_count;//GTCCRD
    R_GPT6->GTCCR[4] = CHOP_count;//GTCCRD

    if (dutyL > 1)
    {
        RECT_count = 0;
    }
    else if (dutyL < 0)
    {
        RECT_count = g_RECT_PWM_period + g_RECT_deadt;
    }

    R_GPT7->GTCCR[4] = RECT_count;//GTCCRD
    R_GPT8->GTCCR[4] = RECT_count;//GTCCRD
    R_GPT9->GTCCR[4] = RECT_count;//GTCCRD

    if (g_sys.f_pwm_start == 0)
    {
    	DRV_SD_OUTPUT = 0;

        R_GPT0->GTSTP = 0x90;

        R_GPT4->GTCNT = 0;
        R_GPT7->GTCNT = 0;

        PWM_OUTPUT_ENABLE;
        R_GPT0->GTSTR = 0x90;

        g_sys.f_pwm_start = 1;
    }
}

uint32_t narrow_pulse_elimination(uint32_t count, uint32_t period, uint32_t deadt)
{
    if (count < (deadt / 2))
    {
        count = 0;
    }
    else if (count < deadt)
    {
        count = deadt;
    }
    else if (count < (period - deadt))
    {
        ;
    }
    else if (count < (period - deadt / 2))
    {
        count = period - deadt;
    }
    else
    {
        count = period + deadt;
    }
    return count;
}

void g_poeg0_callback(poeg_callback_args_t *p_args)
{
    if (PORT_INPUT_POEA == 0)
    {
        if (PORT_INPUT_IOCP1 == 0)
        {
        	g_sys.err.bit.IAC_POE = 1;
        }
        if (PORT_INPUT_VOVP1 == 0)
        {
        	g_sys.err.bit.VDC_POE = 1;
        }
        if (PORT_INPUT_IOCP1 == 1 && PORT_INPUT_VOVP1 == 1)
        {
        	g_sys.err.bit.IAC_POE = 1;
        	g_sys.err.bit.VDC_POE = 1;
        }
        NVIC->ICER[(((uint32_t) VECTOR_NUMBER_POEG0_EVENT) >> 5UL)] = (uint32_t) (1UL << (VECTOR_NUMBER_POEG0_EVENT & 0x1FUL));
    }
    //g_poeg0.p_api->reset(&g_poeg0_ctrl);
    UNUSED(p_args);
}

void fast_protection(void)
{
    /* over current */
    if (g_para.Iac_adc > ERR_IAC_MAX || g_para.Iac_adc < ERR_IAC_MIN)
    {
    	g_sys.err.bit.IAC_OCP = 1;
    }
    /* VDC over voltage */
    if (g_para.Vdc_adc > ERR_VDC_Tri_H)
    {
    	g_sys.err.bit.VDC_OVP = 1;
    }

    if (g_sys.err.byte != 0)
    {
        if (g_sys.state == RUN_STATE)
        {
            dp_stop();
        }
        g_sys.state = FAULT_STATE;
    }
}

void slow_protection(void)
{
    static uint16_t count = 0;

    /* VDC under voltage *//* FAC *//* Vac_rms */
    switch (g_sys.app)
    {
        case APP_PFC:
            if (g_sys.state == RUN_STATE)
            {
                if (g_para.Vdc_adc < g_para.monitor_Vac_rms * 1.3)
                {
                	g_sys.err.bit.VDC_UVP = 1;
                }
                if (g_para.monitor_Fac > ERR_FAC_MAX || g_para.monitor_Fac < ERR_FAC_MIN)
                {
                	g_sys.err.bit.FAC_ERR = 1;
                }
                if (g_para.monitor_Vac_rms > g_para.TH_Vac_rms_Max || g_para.monitor_Vac_rms < g_para.TH_Vac_rms_Min)
                {
                	g_sys.err.bit.VAC_ERR = 1;
                }
            }
            break;
        case APP_GRID_CONNECTED_INVERTER:
            if (g_sys.state == RUN_STATE)
            {
                if (g_para.Vdc_adc < g_para.monitor_Vac_rms * 1.3)
                {
                	g_sys.err.bit.VDC_UVP = 1;
                }
                if (g_para.monitor_Fac > ERR_FAC_MAX || g_para.monitor_Fac < ERR_FAC_MIN)
                {
                	g_sys.err.bit.FAC_ERR = 1;
                }
                if (g_para.monitor_Vac_rms > g_para.TH_Vac_rms_Max || g_para.monitor_Vac_rms < g_para.TH_Vac_rms_Min)
                {
                	g_sys.err.bit.VAC_ERR = 1;
                }
            }
            break;
        case APP_OFF_GRID_INVERTER:
        	if (g_sys.state == RUN_STATE)
        	{
                if (g_para.Vdc_adc < ERR_VDC_Tri_L)
                {
                	g_sys.err.bit.VDC_UVP = 1;
                }
        	}
            break;
        default:
            break;
    }
    /* over temperature */
    if (g_para.Temp_adc > ERR_TEMP_Tri_H)
    {
    	g_sys.err.bit.OTP = 1;
    }
    /* over load */
    if (g_para.monitor_Pac > ERR_PAC_MAX)
    {
        count++;
        if (count > ERR_PAC_CNT)
        {
        	g_sys.err.bit.OLP = 1;
        }
    }
    else
    {
        count = 0;
    }

    if (g_sys.err.byte != 0)
    {
        if (g_sys.state == RUN_STATE)
        {
        	dp_stop();
        }
        g_sys.state = FAULT_STATE;
    }
    else if (g_sys.state == FAULT_STATE)
    {
    	g_sys.state = STANDBY_STATE;
    }
}

void err_rst(void)
{
    if (R_GPT4->GTST_b.ODF == 1)
    {
        if (PORT_INPUT_POEA == 1)
        {
            R_GPT_POEG0->POEGG_b.PIDF = 0;
            NVIC->ISER[(((uint32_t) VECTOR_NUMBER_POEG0_EVENT) >> 5UL)] = (uint32_t) (1UL << (VECTOR_NUMBER_POEG0_EVENT & 0x1FUL));
        }
    }
    if (PORT_INPUT_IOCP1 == 1)
    {
    	g_sys.err.bit.IAC_POE = 0;
    }
    if (PORT_INPUT_VOVP1 == 1)
    {
    	g_sys.err.bit.VDC_POE = 0;
    }
    if (g_para.Iac_adc <= ERR_IAC_MAX && g_para.Iac_adc >= ERR_IAC_MIN)
    {
    	g_sys.err.bit.IAC_OCP = 0;
    }
    if (g_para.Vdc_adc < ERR_VDC_Rcy_H)
    {
    	g_sys.err.bit.VDC_OVP = 0;
    }
    g_sys.err.bit.VDC_UVP = 0;
    if (g_para.Temp_adc < ERR_TEMP_Rcy_H)
    {
    	g_sys.err.bit.OTP = 0;
    }
    g_sys.err.bit.OLP = 0;
    g_sys.err.bit.FAC_ERR = 0;
    g_sys.err.bit.VAC_ERR = 0;
    g_sys.err.bit.PLL_ERR = 0;
    g_sys.err.bit.INNER_ERR = 0;
}

void dp_start(void)
{
    dp_para_reset();
}

void dp_stop(void)
{
	R_GPT0->GTSTP = 0x90;

    /* PWM output disabled */
    R_GPT4->GTIOR_b.OAE = 0;
    R_GPT4->GTIOR_b.OBE = 0;
    R_GPT7->GTIOR_b.OAE = 0;
    R_GPT7->GTIOR_b.OBE = 0;

    DRV_SD_OUTPUT = 1;

    R_GPT4->GTIOR_b.GTIOA = 0x09;//Init:L,End:H,Match:L
    R_GPT5->GTIOR_b.GTIOA = 0x09;
    R_GPT6->GTIOR_b.GTIOA = 0x09;
    R_GPT4->GTIOR_b.GTIOB = 0x06;//Init:L,End:L,Match:H
    R_GPT5->GTIOR_b.GTIOB = 0x06;
    R_GPT6->GTIOR_b.GTIOB = 0x06;

    R_GPT7->GTIOR_b.GTIOA = 0x09;//Init:L,End:H,Match:L
    R_GPT8->GTIOR_b.GTIOA = 0x09;
    R_GPT9->GTIOR_b.GTIOA = 0x09;
    R_GPT7->GTIOR_b.GTIOB = 0x06;//Init:L,End:L,Match:H
    R_GPT8->GTIOR_b.GTIOB = 0x06;
    R_GPT9->GTIOR_b.GTIOB = 0x06;

    R_GPT0->GTSTR = 0x90;

    PORT_OUTPUT_RELAY = 0;
}

volatile uint32_t captured_kint_sig = 0;
bool gf_kint4_enable = true;
uint16_t kint4_time;
void dp_routine(void)
{
    switch (con.cmd)
    {
        case CMD_ON:
            if (g_sys.state == STANDBY_STATE)
            {
                dp_start();
                g_sys.state = PLL_SYNC_STATE;
            }
            break;
        case CMD_OFF:
            if (g_sys.state != FAULT_STATE)
            {
            	dp_stop();
                g_sys.state = STANDBY_STATE;
            }
            break;
        case CMD_ERR_RST:
            if (g_sys.state == FAULT_STATE)
            {
                err_rst();
                if (g_sys.err.byte == 0)
                {
                	g_sys.state = STANDBY_STATE;
                }
            }
            break;
        case CMD_PARA_WRT:
        	if (g_sys.state == STANDBY_STATE || g_sys.state == FAULT_STATE)
            {
                //dataflash_wrt();
            }
            break;
        default:
            break;
    }
    con.cmd = CMD_NONE;

	if (kint4_time >= 5000)
	{
		gf_kint4_enable = true;
		kint4_time = 0;
	}
    captured_kint_sig = g_kint_signal;
    g_kint_signal = 0;
    switch (captured_kint_sig)
    {
    	case CH_MASK_4:
    		if (gf_kint4_enable)
    		{
                if (g_sys.state == STANDBY_STATE)
                {
                    dp_start();
                    g_sys.state = PLL_SYNC_STATE;
                }
                else if (g_sys.state != FAULT_STATE)
                {
                	dp_stop();
                    g_sys.state = STANDBY_STATE;
                }
    			gf_kint4_enable = false;
    		}
    		break;
    	case CH_MASK_5:
            if (g_sys.state == FAULT_STATE)
            {
                err_rst();
                if (g_sys.err.byte == 0)
                {
                	g_sys.state = STANDBY_STATE;
                }
            }
    		break;
    	default:
    		break;
    }
    captured_kint_sig = 0;
}
