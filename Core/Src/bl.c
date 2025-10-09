#include "user_comm.h"

uint8_t bl_tx_buf[OLD_FRAME_LEN] = {0};
uint8_t AT_FB[] = {'+', 'R', 'E', 'A', 'D', 'Y', '\r', '\n'};
uint8_t CONNECTED_AT_FB[] = {'+', 'C', 'O', 'N', 'N', 'E',};
uint8_t DIS_CONN_AT_FB[] = {'+', 'D', 'I', 'S', 'C', 'O', 'N'};
uint8_t bl_conn_status = 0;

// 40ms传输一帧数据
void bl_task(void)
{
    static uint32_t last_send_tick = 0;

    // 调节帧率
    if (HAL_GetTick() - last_send_tick < 40)
        return;

    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)bl_tx_buf, OLD_FRAME_LEN);
    // HAL_UART_Transmit(&huart2, (uint8_t *)bl_tx_buf, OLD_FRAME_LEN, 0xffff);

    last_send_tick = HAL_GetTick();
}

void recv_handler_u2(uint8_t len)
{
    if (memcmp(uart2_rx_buf, AT_FB, sizeof(AT_FB)) == 0)
    {
        // bl_conn_status = 1;
    }
    else if (memcmp(uart2_rx_buf, CONNECTED_AT_FB, sizeof(CONNECTED_AT_FB)) == 0)
    {
        bl_conn_status = 1;
        all_led_off();
    }
    else if (memcmp(uart2_rx_buf, DIS_CONN_AT_FB, sizeof(DIS_CONN_AT_FB)) == 0)
    {
        bl_conn_status = 0;
        all_led_off();
    }
}

void u2_task(void)
{

    if (got_rx_u2)
    {
        recv_handler_u2(got_rx_u2);
        got_rx_u2 = 0;
    }
}