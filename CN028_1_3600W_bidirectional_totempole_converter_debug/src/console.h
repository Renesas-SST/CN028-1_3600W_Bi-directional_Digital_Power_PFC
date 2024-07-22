/*
 * console.h
 *
 *  Created on: 2022年8月24日
 *      Author: a5073522
 */

#ifndef CONSOLE_H_
#define CONSOLE_H_

typedef struct con_flag
{
    bool uart_connect;
    uint8_t uart_rx_err;
    uint16_t uart_rx_timeout;
    bool can_connect;
    uint16_t can_tx_time;
    bool can_tx_en;
    uint8_t can_tx_num;
}t_con_flag;

typedef struct
{
    float pi_iloop_kp;
    float pi_iloop_ki;
    float pi_vloop_kp;
    float pi_vloop_ki;
    float pr_vloop_kp;
    float pr_vloop_kr;
    float pr_iloop_kp;
    float pr_iloop_kr;
    uint8_t sys_app;

    uint8_t cmd;
    t_con_flag f;
}t_console;

typedef enum
{
    CMD_NONE,
    CMD_ON,
    CMD_OFF,
    CMD_ERR_RST,
    CMD_PARA_WRT,
}t_cmd;

/* Buffers size definitions */
#define TX_BUF_SIZE     1024
#define RX_BUF_SIZE     21//512

typedef enum e_protocol
{
    HEAD = 0,
    CMD,
    LEN0,
    LEN1,
    SUM,
    HEADER_SIZE,
    READ1_LEN = 7,
    READ2_LEN = 16,
    MONITOR_LEN = 15,
    READ1_NUM = HEADER_SIZE + READ1_LEN,
    READ2_NUM = HEADER_SIZE + READ2_LEN,
    MONITOR_NUM = HEADER_SIZE + MONITOR_LEN,
}t_protocol;

typedef enum e_serial_cmd
{
    CONNECT = 0x01,
    DISCONNECT = 0x02,

    RUN = 0x03,
    STOP = 0x04,
    MONITOR = 0x05,
    ERR_RST = 0x06,
    READ1 = 0x07,
    READ2 = 0X08,
    WRITEA = 0x09,
    WRITEB = 0x0A,
    WRITE1 = 0x0B,
    WRITE2 = 0x0C,

    RET_OK = 0xF0,
    RET_ERR = 0xF1,
}t_serial_cmd;

void serial_para_init(void);
void uart_operation(void);
void FloatTo2Byte(uint8_t * dest, float src, uint16_t offset, float resolution);
float TwoByte2Float(uint8_t * src, uint16_t offset, float resolution);

extern t_console con;
extern uint8_t g_sci9_rx_buf[RX_BUF_SIZE];

#endif /* CONSOLE_H_ */
