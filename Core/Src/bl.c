#include "user_comm.h"

uint8_t bl_tx_buf[OLD_FRAME_LEN] = {0};

// 40ms传输一帧数据
void bl_task(void)
{
    static uint32_t last_send_tick = 0;

    // 调节帧率
    if (HAL_GetTick() - last_send_tick < 40)
        return;

    // HAL_UART_Transmit_DMA(&huart2, (uint8_t *)bl_tx_buf, OLD_FRAME_LEN);
    HAL_UART_Transmit(&huart2, (uint8_t *)bl_tx_buf, OLD_FRAME_LEN, 0xffff);

    last_send_tick = HAL_GetTick();
}