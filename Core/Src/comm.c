#include "user_comm.h"

uint8_t uart1_rx_buf[STANDARD_PROTOCAL_LEN];
uint8_t uart2_rx_buf[STANDARD_PROTOCAL_LEN];

static uint8_t got_rx = 0;
uint8_t got_rx_u2 = 0;


// protocal
// 帧头4字节 0xaa 0x55 0x03 0x99
// 报文序号 1字节
// 报文类型 1字节
// 指令    1字节
// 校验和 1字节
void start_uart_rx(void)
{
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);

    HAL_UART_Receive_DMA(&huart1, uart1_rx_buf, UART_RX_BUF_LEN);

    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);

    HAL_UART_Receive_DMA(&huart2, uart2_rx_buf, UART_RX_BUF_LEN);
}

// 0xaa 0x55 0x03 0x99
void recv_handler(uint16_t len)
{
    // step 1, check frame head
    if (uart1_rx_buf[0] != 0xaa || uart1_rx_buf[1] != 0x55 || uart1_rx_buf[2] != 0x03 || uart1_rx_buf[3] != 0x99)
    {
        // 不是帧头, 清除数据
        memset(uart1_rx_buf, 0, UART_RX_BUF_LEN);
        return;
    }

    // step 2, check frame total sum
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len - 1; i++)
    {
        sum += uart1_rx_buf[i];
    }

    if (sum != uart1_rx_buf[len - 1])
    {
        // 校验和不对, 清除数据
        memset(uart1_rx_buf, 0, UART_RX_BUF_LEN);
        return;
    }

    uint8_t frame_id = uart1_rx_buf[4];   // 帧序号
    uint8_t frame_type = uart1_rx_buf[5]; // 帧类型
    uint8_t cmd = uart1_rx_buf[6];        // 指令

    // step 3, check command
    if (cmd == CMD_GET_DATA && frame_type == CMD_TYPE_REQUEST)
    {
        send_data(frame_id);
    }
}

void comm_handler(void)
{
    if (got_rx == CMD_TOTAL_LEN)
    {
        recv_handler(got_rx);
        got_rx = 0;
    }
}

void uart1_it_handler(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
    {
        // 清除 IDLE 标志
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);

        // 此时说明可能一帧数据接收完了
        // 获取当前 DMA 计数器剩余量, 计算本次接收字节数
        uint16_t remain = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
        if (remain)
        {
            uint16_t data_len = UART_RX_BUF_LEN - remain;
            got_rx = data_len;
        }

        // 处理完毕后，可以选择清零或记录这段数据
        // 然后继续保持 DMA 接收，让它随时准备接收新的数据
        HAL_UART_AbortReceive(&huart1);
        // HAL_UART_DMAStop(&huart1);
        // 重新开启接收
        HAL_UART_Receive_DMA(&huart1, uart1_rx_buf, UART_RX_BUF_LEN);
    }
}

void uart2_it_handler(void)
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) != RESET)
    {
        // 清除 IDLE 标志
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);

        // 此时说明可能一帧数据接收完了
        // 获取当前 DMA 计数器剩余量, 计算本次接收字节数
        uint16_t remain = __HAL_DMA_GET_COUNTER(huart2.hdmarx);
        if (remain)
        {
            uint16_t data_len = UART_RX_BUF_LEN - remain;
            got_rx_u2 = data_len;
        }

        // 处理完毕后，可以选择清零或记录这段数据
        // 然后继续保持 DMA 接收，让它随时准备接收新的数据
        HAL_UART_AbortReceive(&huart2);
        // HAL_UART_DMAStop(&huart1);
        // 重新开启接收
        HAL_UART_Receive_DMA(&huart2, uart2_rx_buf, UART_RX_BUF_LEN);
    }
}

void com_task(void)
{
    comm_handler();
}
