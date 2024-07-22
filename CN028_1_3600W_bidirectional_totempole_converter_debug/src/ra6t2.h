/*
 * ra6t2.h
 *
 *  Created on: 2022年8月22日
 *      Author: a5073522
 */

#ifndef RA6T2_H_
#define RA6T2_H_

#define APP_ERR_TRAP(err)        if(err) {__asm("BKPT #0\n");} /* trap upon the error  */
#define UNUSED(x)               (void)(x)

#define PWM_CLK					120		//MHz
#define RECT_PWM_FRE			66		//kHz
#define CHOP_PWM_FRE			66		//kHz
#define RECT_DEAD_T				1000	//ns
#define CHOP_DEAD_T				200		//ns

#define PORT_INPUT_POEA         R_PORT12->PIDR_b.PIDR7		//PC07
#define PORT_OUTPUT_RELAY       R_PORT14->P0DR_b.PODR9		//PE09
#define PORT_INPUT_IOCP1        R_PORT12->PIDR_b.PIDR5		//PC05
#define PORT_INPUT_VOVP1        R_PORT11->PIDR_b.PIDR1		//PB01
#define DRV_SD_OUTPUT			R_PORT14->P0DR_b.PODR12		//PE12
#define LED_HEARTBEAT			R_PORT14->P0DR_b.PODR13		//PE13
#define PORT_OUTPUT_FAN			R_PORT12->P0DR_b.PODR8		//PC08
#define KEY_ENB					R_PORT13->PIDR_b.PIDR12		//PD12
#define KEY_RST					R_PORT13->PIDR_b.PIDR13		//PD13

#define PWM_OUTPUT_ENABLE      {R_GPT4->GTIOR_b.OAE = 1;\
                                R_GPT4->GTIOR_b.OBE = 1;\
                                R_GPT7->GTIOR_b.OAE = 1;\
                                R_GPT7->GTIOR_b.OBE = 1;}

/* channel masks */
typedef enum
{
    CH_MASK_0 = 0x01,
    CH_MASK_1 = 0x02,
    CH_MASK_2 = 0x04,
    CH_MASK_3 = 0x08,
    CH_MASK_4 = 0x10,
    CH_MASK_5 = 0x20,
    CH_MASK_6 = 0x40,
    CH_MASK_7 = 0x80
}channel_mask_t;

void R_UART_init(void);
float R_TFU_sin(float theta);
float R_TFU_cos(float theta);
void R_TFU_sincos(float theta, float * sin, float * cos);
float R_TFU_hypot(float x, float y);

void relay_on(uint32_t delay, bsp_delay_units_t units);
void hardware_init(void);
void startup(void);

extern fsp_err_t err;
extern uint32_t g_RECT_PWM_period;
extern uint32_t g_CHOP_PWM_period;
extern uint32_t g_RECT_deadt;
extern uint32_t g_CHOP_deadt;
extern uint8_t gf_sci9_tx_complete;
extern uint8_t gf_sci9_rx_complete;
extern volatile uint32_t g_kint_signal;

#endif /* RA6T2_H_ */
