#include "user_comm.h"

const GPIO_Channel channels[64] = {CH_DEF(0), CH_DEF(1), CH_DEF(2), CH_DEF(3), CH_DEF(4), CH_DEF(5), CH_DEF(6), CH_DEF(7),
                                   CH_DEF(8), CH_DEF(9), CH_DEF(10), CH_DEF(11), CH_DEF(12), CH_DEF(13), CH_DEF(14), CH_DEF(15)};

void set_channel_pin(uint8_t ch, GPIO_PinState pin_status)
{
    if (ch < sizeof(channels) / sizeof(channels[0]))
    {
        HAL_GPIO_WritePin(channels[ch].port, channels[ch].pin, pin_status);
    }
}

void turn_on_input_ch(uint8_t input_ch)
{

    static uint16_t last_ch = 0;

    // 关闭上一个通道
    set_channel_pin(last_ch, GPIO_PIN_RESET);

    // 打开下一个通道
    set_channel_pin(input_ch, GPIO_PIN_SET);

    last_ch = input_ch;
}

static void close_all_adc_ch(void)
{
    HAL_GPIO_WritePin(HC4067_EN_GPIO_Port, HC4067_EN_Pin, GPIO_PIN_SET);
}

void set_adc_ch(uint8_t adc_ch)
{

    if (adc_ch < 16)
    {

        HAL_GPIO_WritePin(HC4067_S0_GPIO_Port, HC4067_S0_Pin, (adc_ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(HC4067_S1_GPIO_Port, HC4067_S1_Pin, (adc_ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(HC4067_S2_GPIO_Port, HC4067_S2_Pin, (adc_ch & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(HC4067_S3_GPIO_Port, HC4067_S3_Pin, (adc_ch & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        HAL_GPIO_WritePin(HC4067_EN_GPIO_Port, HC4067_EN_Pin, GPIO_PIN_RESET);
    }
}
