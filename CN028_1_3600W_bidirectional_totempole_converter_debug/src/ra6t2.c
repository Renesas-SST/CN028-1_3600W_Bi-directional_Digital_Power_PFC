/*
 * ra6t2.c
 *
 *  Created on: 2022年8月22日
 *      Author: a5073522
 */

#include "hal_data.h"
#include "ra6t2.h"
#include "console.h"
#include "can_fd.h"
#include "user_define.h"
#include "digitPow.h"

#define poeg_gtonccrA           (unsigned short *)0x4008A044
#define poeg_gtoncwpA           (unsigned short *)0x4008A040
#define tfu_trgsts              (unsigned char *)0x40021008

void R_Timer_10kHz_init(void);
void R_PWM_RECT_init(void);
void R_PWM_CHOP_init(void);
void R_POE_init(void);
void R_CMP_DA_init(void);
fsp_err_t R_ADC_B_Calibrate_user(void);
void R_ADC_init(void);
void R_TFU_init(void);
void R_ELC_init(void);
void R_Key_init(void);

fsp_err_t err = FSP_SUCCESS;
uint32_t g_RECT_PWM_period = 0x38D;		//66kHz@120MHz
uint32_t g_CHOP_PWM_period = 0x38D;
uint32_t g_RECT_deadt = 120; 			//1us@120MHz
uint32_t g_CHOP_deadt = 36; 			//300ns@120MHz

uint8_t gf_sci9_tx_complete = 1;
uint8_t gf_sci9_rx_complete = 0;

void relay_on(uint32_t delay, bsp_delay_units_t units)
{
    switch (units)
    {
        case BSP_DELAY_UNITS_SECONDS:
            R_BSP_SoftwareDelay(delay, BSP_DELAY_UNITS_SECONDS);
            break;
        case BSP_DELAY_UNITS_MILLISECONDS:
            R_BSP_SoftwareDelay(delay, BSP_DELAY_UNITS_MILLISECONDS);
            break;
        case BSP_DELAY_UNITS_MICROSECONDS:
            R_BSP_SoftwareDelay(delay, BSP_DELAY_UNITS_MICROSECONDS);
            break;
        default:
            break;
    }
    PORT_OUTPUT_RELAY = 1;
}

/* 10kHz timer init */
void R_Timer_10kHz_init(void)
{
    g_agt1_10kHz.p_api->open(&g_agt1_10kHz_ctrl, &g_agt1_10kHz_cfg);
}

/* Rectifying PWM init*/
void R_PWM_RECT_init(void)
{
    R_GPT_THREE_PHASE_Open(&g_three_phase1_ctrl, &g_three_phase1_cfg);

    R_GPT7->GTCR_b.MD = 13;//Complementary PWM mode2 (transfer at trough)
    R_GPT8->GTCR_b.MD = 13;
    R_GPT9->GTCR_b.MD = 13;

    R_GPT7->GTPR = g_RECT_PWM_period;
    R_GPT8->GTPR = g_RECT_PWM_period;
    R_GPT9->GTPR = g_RECT_PWM_period;

    R_GPT7->GTPBR = g_RECT_PWM_period;
    R_GPT8->GTPBR = g_RECT_PWM_period;
    R_GPT9->GTPBR = g_RECT_PWM_period;

    R_GPT7->GTPDBR = g_RECT_PWM_period;
    R_GPT8->GTPDBR = g_RECT_PWM_period;
    R_GPT9->GTPDBR = g_RECT_PWM_period;

    R_GPT7->GTIOR_b.GTIOA = 0x09;//Init:L,End:H,Match:L
    R_GPT8->GTIOR_b.GTIOA = 0x09;
    R_GPT9->GTIOR_b.GTIOA = 0x09;
    R_GPT7->GTIOR_b.GTIOB = 0x06;//Init:L,End:L,Match:H
    R_GPT8->GTIOR_b.GTIOB = 0x06;
    R_GPT9->GTIOR_b.GTIOB = 0x06;

    uint32_t half_period;
    half_period = (g_RECT_PWM_period + 1) / 2;
    R_GPT7->GTCCR[0] = half_period;//GTCCRA
    R_GPT8->GTCCR[0] = half_period;
    R_GPT9->GTCCR[0] = half_period;

    R_GPT7->GTCCR[2] = half_period;//GTCCRC
    R_GPT8->GTCCR[2] = half_period;
    R_GPT9->GTCCR[2] = half_period;

    R_GPT7->GTCCR[4] = half_period;//GTCCRD
    R_GPT8->GTCCR[4] = half_period;
    R_GPT9->GTCCR[4] = half_period;

    R_GPT7->GTDVU = g_RECT_deadt+1;
    R_GPT8->GTDVU = g_RECT_deadt+1;
    R_GPT9->GTDVU = g_RECT_deadt+1;

    R_GPT7->GTIOR_b.OAE = 0;
    R_GPT7->GTIOR_b.OBE = 0;
}

/* Chopping PWM  init & PDG */
void R_PWM_CHOP_init(void)
{
    R_GPT_THREE_PHASE_Open(&g_three_phase0_ctrl, &g_three_phase0_cfg);

    R_GPT4->GTCR_b.MD = 13;//Complementary PWM mode2 (transfer at trough)
    R_GPT5->GTCR_b.MD = 13;
    R_GPT6->GTCR_b.MD = 13;

    R_GPT4->GTPR = g_CHOP_PWM_period;
    R_GPT5->GTPR = g_CHOP_PWM_period;
    R_GPT6->GTPR = g_CHOP_PWM_period;

    R_GPT4->GTPBR = g_CHOP_PWM_period;
    R_GPT5->GTPBR = g_CHOP_PWM_period;
    R_GPT6->GTPBR = g_CHOP_PWM_period;

    R_GPT4->GTPDBR = g_CHOP_PWM_period;
    R_GPT5->GTPDBR = g_CHOP_PWM_period;
    R_GPT6->GTPDBR = g_CHOP_PWM_period;

    R_GPT4->GTIOR_b.GTIOA = 0x09;//Init:L,End:H,Match:L
    R_GPT5->GTIOR_b.GTIOA = 0x09;
    R_GPT6->GTIOR_b.GTIOA = 0x09;
    R_GPT4->GTIOR_b.GTIOB = 0x06;//Init:L,End:L,Match:H
    R_GPT5->GTIOR_b.GTIOB = 0x06;
    R_GPT6->GTIOR_b.GTIOB = 0x06;

    uint32_t half_period;
    half_period = (g_CHOP_PWM_period + 1) / 2;
    R_GPT4->GTCCR[0] = half_period;//GTCCRA
    R_GPT5->GTCCR[0] = half_period;
    R_GPT6->GTCCR[0] = half_period;

    R_GPT4->GTCCR[2] = half_period;//GTCCRC
    R_GPT5->GTCCR[2] = half_period;
    R_GPT6->GTCCR[2] = half_period;

    R_GPT4->GTCCR[4] = half_period;//GTCCRD
    R_GPT5->GTCCR[4] = half_period;
    R_GPT6->GTCCR[4] = half_period;

    R_GPT4->GTDVU = g_CHOP_deadt+1;
    R_GPT5->GTDVU = g_CHOP_deadt+1;
    R_GPT6->GTDVU = g_CHOP_deadt+1;

    R_GPT4->GTINTAD_b.ADTRAUEN = 1;
    R_GPT4->GTINTAD_b.ADTRADEN = 0;
    R_GPT4->GTINTAD_b.ADTRBUEN = 0;
    R_GPT4->GTINTAD_b.ADTRBDEN = 0;
    R_GPT4->GTADTRA = 0x1;

    R_GPT4->GTIOR_b.OAE = 0;
    R_GPT4->GTIOR_b.OBE = 0;
    R_GPT5->GTIOR_b.OAE = 0;
    R_GPT5->GTIOR_b.OBE = 0;
    R_GPT6->GTIOR_b.OAE = 0;
    R_GPT6->GTIOR_b.OBE = 0;
}

/* POE init */
void R_POE_init(void)
{
    g_poeg0.p_api->open(&g_poeg0_ctrl, &g_poeg0_cfg);
}

/* CMP and DA*/
void R_CMP_DA_init(void)
{
    //DA3 init
    g_dac3.p_api->open(&g_dac3_ctrl, &g_dac3_cfg);
    g_dac3.p_api->write(&g_dac3_ctrl, 2048);//Setting of DADR0 / 4096 * AVCC0 = 2048/4096*3.3=1.67V
    g_dac3.p_api->start(&g_dac3_ctrl);
    //CMP0 init
    g_comparator0.p_api->open(&g_comparator0_ctrl, &g_comparator0_cfg);
    g_comparator0.p_api->outputEnable(&g_comparator0_ctrl);

    g_dac0.p_api->open(&g_dac0_ctrl, &g_dac0_cfg);
    g_dac0.p_api->write(&g_dac0_ctrl, 0);
    g_dac0.p_api->start(&g_dac0_ctrl);
    g_dac1.p_api->open(&g_dac1_ctrl, &g_dac1_cfg);
    g_dac1.p_api->write(&g_dac1_ctrl, 0);
    g_dac1.p_api->start(&g_dac1_ctrl);
    g_dac2.p_api->open(&g_dac2_ctrl, &g_dac2_cfg);
    g_dac2.p_api->write(&g_dac2_ctrl, 0);
    g_dac2.p_api->start(&g_dac2_ctrl);
}

fsp_err_t R_ADC_B_Calibrate_user(void)
{
    /* Store and Clear ADC Start Trigger Enable */
    uint32_t adtrgenr = R_ADC_B->ADTRGENR;
    R_ADC_B->ADTRGENR = 0;

    /* Wait for converter to stop
     *  - See table 36.26 of the RA6T2 User Manual, R01UH0951EJ0100 */
    FSP_HARDWARE_REGISTER_WAIT(R_ADC_B->ADSR_b.ADACT0, 0)
    FSP_HARDWARE_REGISTER_WAIT(R_ADC_B->ADSR_b.ADACT1, 0)

    /* Clear the error status flags */
    R_ADC_B->ADERSCR     = R_ADC_B0_ADERSCR_ADERCLR0_Msk | R_ADC_B0_ADERSCR_ADERCLR1_Msk;
    R_ADC_B->ADOVFERSCR  = R_ADC_B0_ADOVFERSCR_ADOVFEC0_Msk | R_ADC_B0_ADOVFERSCR_ADOVFEC1_Msk;
    R_ADC_B->ADOVFCHSCR0 = R_ADC_B0_ADOVFCHSCR0_OVFCHCn_Msk;
    R_ADC_B->ADOVFEXSCR  = R_ADC_B0_ADOVFEXSCR_OVFEXC0_Msk | R_ADC_B0_ADOVFEXSCR_OVFEXC1_Msk |
                           R_ADC_B0_ADOVFEXSCR_OVFEXC2_Msk | R_ADC_B0_ADOVFEXSCR_OVFEXC5_Msk |
                           R_ADC_B0_ADOVFEXSCR_OVFEXC6_Msk | R_ADC_B0_ADOVFEXSCR_OVFEXC7_Msk |
                           R_ADC_B0_ADOVFEXSCR_OVFEXC8_Msk;

    R_ADC_B->ADCALSTR = 0x303;  //start the internal circuit calibration & start the gain and offset calibration

    /* Error Status Check */
    uint32_t read_err = R_ADC_B->ADERSCR ||
                        R_ADC_B->ADOVFERSR ||
                        R_ADC_B->ADOVFCHSR0 ||
                        R_ADC_B->ADOVFEXSR;

    /* Reset ADC Start Trigger Enable */
    R_ADC_B->ADTRGENR = adtrgenr;

    fsp_err_t fsp_err = (read_err ? FSP_ERR_INVALID_HW_CONDITION : FSP_SUCCESS);
    FSP_ERROR_LOG(fsp_err);

    return fsp_err;
}

/* A/D init */
void R_ADC_init(void)
{
    g_adc0.p_api->open(&g_adc0_ctrl, &g_adc0_cfg);
    g_adc0.p_api->scanCfg(&g_adc0_ctrl, &g_adc0_scan_cfg);
    err = R_ADC_B_Calibrate_user();
    if (FSP_SUCCESS != err)
    {
        APP_ERR_TRAP(err);
    }

    R_BSP_SoftwareDelay(1000, BSP_DELAY_UNITS_MICROSECONDS);

    R_ADC_B->ADGSPCR = 0x0007;
    R_ADC_B->ADTRGENR = 0x03;
}

void R_TFU_init(void)
{
    R_MSTP->MSTPCRC_b.MSTPC20 = 0;
}

void R_TFU_sincos(float theta, float * sin, float * cos)
{
    R_TFU->SCDT1 = theta;
    while ((* tfu_trgsts & 0x01) != 0)
    {
        __NOP();
    }
    * sin = R_TFU->SCDT1;
    * cos = R_TFU->SCDT0;
}

float R_TFU_sin(float theta)
{
    R_TFU->SCDT1 = theta;
    while ((* tfu_trgsts & 0x01) != 0)
    {
        __NOP();
    }
    return R_TFU->SCDT1;
}

float R_TFU_cos(float theta)
{
    R_TFU->SCDT1 = theta;
    while ((* tfu_trgsts & 0x01) != 0)
    {
        __NOP();
    }
    return R_TFU->SCDT0;
}

float R_TFU_hypot(float x, float y)
{
    float hypot_k, result;
    R_TFU->ATDT0 = x;
    R_TFU->ATDT1 = y;
    hypot_k = R_TFU->ATDT0;
    if ((* tfu_trgsts & 0x01) != 0)
    {
        __NOP();
    }
    result = hypot_k * (float)0.6072529350088812561694;
    return result;
}

void R_ELC_init(void)
{
    R_ELC_Open(&g_elc_ctrl, &g_elc_cfg);
    R_ELC_LinkSet (&g_elc_ctrl, ELC_PERIPHERAL_ADC0, ELC_EVENT_AGT1_INT);
    R_ELC_Enable (&g_elc_ctrl);
}

void R_UART_init(void)
{
    R_SCI_B_UART_Open(&g_uart9_ctrl, &g_uart9_cfg);
    g_dtc0_sci9_txi_cfg.p_info->transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE;
#if 1	/* 115200bps */
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.abcse = 0;
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.abcs = 0;
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.bgdm = 1;
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.cks = 0;
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.brr = 64;
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.mddr = (uint8_t) 256;
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.brme = false;
#endif
#if 0	 /* 921600bps */
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.abcse = 0;
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.abcs = 0;
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.bgdm = 1;
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.cks = 0;
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.brr = 6;
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.mddr = (uint8_t) 220;
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.brme = true;
#endif
    R_SCI_B_UART_BaudSet (&g_uart9_ctrl, g_uart9_cfg_extend.p_baud_setting);
    R_SCI_B_UART_Read(&g_uart9_ctrl, g_sci9_rx_buf, HEADER_SIZE);
}

void g_uart9_callback(uart_callback_args_t *p_args)
{
    if(NULL != p_args)
    {
        if (UART_EVENT_TX_COMPLETE  == p_args->event)
        {
            gf_sci9_tx_complete = 1;
        }
        if (UART_EVENT_RX_COMPLETE  == p_args->event)
        {
            gf_sci9_rx_complete = 1;
        }
    }
}

void R_PWM_FAN_init(void);
void R_PWM_FAN_init(void)
{
    g_agt0.p_api->open(&g_agt0_ctrl, &g_agt0_cfg);
    g_agt0.p_api->start(&g_agt0_ctrl);
}

void R_Key_init(void)
{
    err = R_KINT_Open(&g_kint0_ctrl, &g_kint0_cfg);
    if (FSP_SUCCESS != err)
    {
        APP_ERR_TRAP(err);
    }
    err = R_KINT_Enable(&g_kint0_ctrl);
    if (FSP_SUCCESS != err)
    {
    	R_KINT_Close(&g_kint0_ctrl);
        APP_ERR_TRAP(err);
    }
}

volatile uint32_t g_kint_signal = 0;
void kint_callback(keymatrix_callback_args_t *p_args)
{
	if (NULL != p_args)
	{
		/* capture KINT event for a particular channel mask */
		g_kint_signal = p_args->channel_mask;
	}
}

void hardware_init(void)
{
	g_RECT_PWM_period = PWM_CLK * 1000 / (RECT_PWM_FRE * 2);
	g_CHOP_PWM_period = PWM_CLK * 1000 / (CHOP_PWM_FRE * 2);
	g_RECT_deadt = RECT_DEAD_T * PWM_CLK / 1000;
	g_CHOP_deadt = CHOP_DEAD_T * PWM_CLK / 1000;

    R_PWM_RECT_init();
    R_PWM_CHOP_init();
	R_GPT4->GTIOR_b.PSYE = 1;
	R_GPT7->GTIOR_b.PSYE = 1;

    R_Timer_10kHz_init();
    R_POE_init();
    R_CMP_DA_init();
    R_ADC_init();
    R_TFU_init();
    R_ELC_init();
    //R_Key_init();
    R_UART_init();
    R_CAN_init();
}

void startup(void)
{
    g_agt1_10kHz.p_api->start(&g_agt1_10kHz_ctrl);
    R_GPT0->GTSTR = 0x90;
}
