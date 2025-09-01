#include "user_comm.h"

uint16_t bat_val_dma_buf[ADC2_DMA_BUF_LEN] = {0};
uint8_t bat_percentage;
uint8_t bat_smooth_percentage = 100;

// 针对4.2伏特锂电池的点亮百分比算法.
// 3720-3645 = 75 75/26 = 2.88
// 4200- 4100 = 100 100/6 = 16.66
// 100 - 16.66 = 83.33
uint8_t bat_mini_volt_to_percentage(uint16_t mvolts)
{
    uint8_t battery_level;

    if (mvolts >= 4200)
    {
        battery_level = 100;
    }
    else if (mvolts > 3720)
    {
        battery_level = 100 - (4200 - mvolts) / 6;
    }
    else if (mvolts > 3200)
    {
        battery_level = 20 - (3720 - mvolts) / 26;
    }
    else
    {
        battery_level = 0;
    }

    return battery_level;
}

static uint8_t count_bat_volt(uint16_t adc_val)
{
    // 计算电压值
    // Vbat = (adc_val / 4095) * Vref * (R1 + R2) / R2
    // Vref = 3.3V, R1 = 150k, R2 = 150k
    // Vbat = (adc_val / 4095) * 3.3 * (150 + 150) / 150 = (adc_val / 4095) * 6.6
    float vbat = ((float)adc_val / 4095.0f) * 6.6f;
    uint16_t vbat_mvolts = (uint16_t)(vbat * 1000); // 转换为毫伏

    bat_percentage = bat_mini_volt_to_percentage(vbat_mvolts);

    // 平滑处理
    bat_smooth_percentage = ema_u8(bat_percentage, bat_smooth_percentage, 5, 10);
}

void bat_task(void)
{

    if (bat_adc_done == 0)
        return;

    // static uint32_t last_sys_tick = 0;
    // uint32_t now = HAL_GetTick();
    // if (now - last_sys_tick < 1000)
    // {
    //     return;
    // }

    // last_sys_tick = now;
    uint32_t val_total = 0;
    for (uint8_t i = 0; i < ADC2_DMA_BUF_LEN; i++)
    {
        val_total += bat_val_dma_buf[i];
    }

    count_bat_volt((uint16_t)(val_total / ADC2_DMA_BUF_LEN));

    static uint32_t print_counter = 0;
    print_counter++;
    if (print_counter > 1000)
    {
        uint8_t tx_buf[50];
        sprintf((char *)tx_buf, "bat: %d%% %d%% \r\n", bat_smooth_percentage, bat_percentage);
        HAL_UART_Transmit_DMA(&huart1, tx_buf, strlen((char *)tx_buf));
        print_counter = 0;
    }

    bat_adc_done = 0;
}
