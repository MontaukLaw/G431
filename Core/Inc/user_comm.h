#ifndef _USER_COMM_H_
#define _USER_COMM_H_

#include "app.h"
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "switch_hal.h"
#include "stm32g4xx_hal.h"
#include "stm32g431xx.h"

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} GPIO_Channel;

#define CH_DEF(n) {CH##n##_GPIO_Port, CH##n##_Pin}

#define INPUT_CH_NUMBER 16

#define ADC_CHANNEL_NUMBER 16

#define ADC_BUFFER_SIZE 10 // Define the size of the ADC buffer

#define TOTAL_POINTS (ADC_CHANNEL_NUMBER * INPUT_CH_NUMBER) 

#define FRAME_LEN (TOTAL_POINTS + 4)

#endif
