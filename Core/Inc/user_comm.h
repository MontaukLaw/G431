#ifndef _USER_COMM_H_
#define _USER_COMM_H_

// #include <string.h>

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "switch_hal.h"
#include "stm32g4xx_hal.h"
#include "stm32g431xx.h"
#include "comm.h"
#include "app.h"
#include <stdint.h>

#include <math.h>
#include <string.h>
// #include "myiic.h"
#include "icm42688.h"
#include "icm42688_hard_i2c.h"
// #include "stdbool.h"
#include "bat_val.h"
#include "mems.h"
#include "led.h"
#include "bl.h"
#include "g.sensor.h"
#include "key.h"
#include "charge_state.h"

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} GPIO_Channel;

#define RESET_GQ_KEY_SHAKE_DELAY 5
// 6秒关灯
#define PWOER_DOWN_COUNTER 100 // 600

#define ICM42688DelayMs(_nms) HAL_Delay(_nms)

#define CH_DEF(n) {CH##n##_GPIO_Port, CH##n##_Pin}

#define INPUT_CH_NUMBER 16

#define ADC_CHANNEL_NUMBER 16

#define ADC_BUFFER_SIZE 10 // 80 // 10 // Define the size of the ADC buffer

#define TOTAL_POINTS (ADC_CHANNEL_NUMBER * INPUT_CH_NUMBER)

// 点数量加上帧尾, 加左右两边帧头
// 16个字节的MEMS数据
#define OLD_FRAME_LEN (TOTAL_POINTS + 8 + 4 + 16)

#define FRAME_LEN (TOTAL_POINTS + 4)
#define FRAME_HEAD_LEN 4

// 帧头4B,报文1B,类型1B,CMD1B,结果1B,校验和1B
#define TX_BUF_BYTES (TOTAL_POINTS + FRAME_HEAD_LEN + 5)

#define TX_BUF_LEN 512

#define STANDARD_PROTOCAL_LEN 64

#define UART_RX_BUF_LEN STANDARD_PROTOCAL_LEN

// 采集数据指令
#define CMD_GET_DATA 0x01

#define CMD_TYPE_REQUEST 0x00
#define CMD_TYPE_RESPONSE 0x01
#define CMD_TYPE_NOTIFICATION 0x02

#define CMD_TOTAL_LEN 0x08

#define CMD_RESULT_SUCCESS 0x00
#define CMD_RESULT_FAIL 0x01

#define ADC2_DMA_BUF_LEN 100

#endif
