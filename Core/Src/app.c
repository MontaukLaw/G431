#include "user_comm.h"

uint8_t adc_dma_buffer[ADC_BUFFER_SIZE] = {0};
volatile uint8_t adc_busy = 0;
volatile uint8_t uart_busy = 0; // UART是否忙碌
volatile uint16_t input_ch = 0;
volatile uint16_t point_idx = 0;
volatile uint8_t points_data[FRAME_LEN] = {0};

volatile uint8_t tx_data[OLD_FRAME_LEN] = {0};
volatile uint8_t imu_rest_tx_data[5] = {0x03, 0xAA, 0x55, 0x03, 0x99};
volatile uint8_t time_to_change_adc_ch = 0;
volatile uint16_t adc_ch = 0;

volatile uint8_t check_reset_imu = 0;
volatile uint8_t bl_uart_tx_done = 0;

__IO uint8_t bat_adc_done = 0;
__IO static uint32_t fac_us = 0;

__IO uint32_t tim7_counter = 0;

volatile uint8_t tx_buf[TX_BUF_LEN] = {0}; // 发送缓冲区

uint8_t mems_data[16] = {0xF1, 0xFF, 0x7F, 0x3F, 0xE0, 0xB6, 0x2F, 0x3A,
                         0xF0, 0xF0, 0x08, 0xBA, 0x88, 0x7C, 0x84, 0xBA};
// F1 FF 7F 3F E0 B6 2F 3A F0 F0 08 BA 88 7C 84 BA
void fill_tx_data(void)
{
    // 01 06
    uint16_t first_part_len = TOTAL_POINTS / 2;
    tx_data[0] = 0x01;
    tx_data[1] = 0x06;
    //  AA 55 03 99
    tx_data[first_part_len + 2] = 0xaa;
    tx_data[first_part_len + 3] = 0x55;
    tx_data[first_part_len + 4] = 0x03;
    tx_data[first_part_len + 5] = 0x99;

    tx_data[first_part_len + 6] = 0x02;
    tx_data[first_part_len + 7] = 0x06;

    memcpy(&tx_data[TOTAL_POINTS + 8], mems_data, 16);

    // memcpy((const void *)(tx_data + TOTAL_POINTS + 8), (const void *)points_data, 16);

    tx_data[TOTAL_POINTS + 16 + 8] = 0xaa;
    tx_data[TOTAL_POINTS + 16 + 9] = 0x55;
    tx_data[TOTAL_POINTS + 16 + 10] = 0x03;
    tx_data[TOTAL_POINTS + 16 + 11] = 0x99;
}

uint8_t adc_cn_arr[16] = {0, 1, 2, 3,
                          8, 9, 10, 11,
                          4, 5, 6, 7,
                          12, 13, 14, 15};

// 陀螺仪4元数
extern float q_out[];
extern uint8_t bl_tx_buf[];

/*{15, 1, 2, 3,
  4, 9, 10, 11,
  12, 5, 6, 7,
  8, 13, 14, 0};
*/
void main_task_adc(void)
{
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, ADC_BUFFER_SIZE) != HAL_OK)
    {
        // 启动DMA失败
        Error_Handler();
    }

    adc_busy = 1;
    while (adc_busy)
        ;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        // HAL_GPIO_TogglePin(FOR_TEST1_GPIO_Port, FOR_TEST1_Pin);
        adc_busy = 0;
    }
    else if (hadc->Instance == ADC2)
    {
        bat_adc_done = 1;
    }
}

void adc_data_handler_with_idx(uint8_t point_nmb)
{
    // 简单计算平均值
    uint32_t adc_sum = 0;
    uint32_t i = 0;

    for (i = 4; i < 7; i++)
    {
        adc_sum += adc_dma_buffer[i];
    }
    // tx_buf[0] = adc_sum / ADC_BUFFER_SIZE; // 计算平均值
    float result = adc_sum / 4;      // / (ADC_BUFFER_SIZE - 0);
    points_data[point_nmb] = result; // 将结果存储到points_data中
}

void adc_data_handler(void)
{
    // 简单计算平均值
    uint32_t adc_sum = 0;
    uint32_t i = 0;

    for (i = 0; i < ADC_BUFFER_SIZE; i++)
    {
        adc_sum += adc_dma_buffer[i];
    }
    // tx_buf[0] = adc_sum / ADC_BUFFER_SIZE; // 计算平均值
    float result = adc_sum / (ADC_BUFFER_SIZE - 0);
    points_data[point_idx] = result; // 将结果存储到points_data中

    // if(result <= ZERO_VAL){
    // 	  points_data[point_idx] = 0;
    // }else{
    // 		points_data[point_idx] = result - ZERO_VAL;
    // }
    // points_data[point_idx] = adc_sum / (ADC_BUFFER_SIZE - 5) -  ;
}

// 切换输入通道
// 因为输入切换比较容易, 改gpio就行.
static void change_input_ch(void)
{
    input_ch++;
    if (input_ch >= INPUT_CH_NUMBER)
    {
        input_ch = 0;
        time_to_change_adc_ch = 1;
    }
}

// 切换adc通道
static void change_adc_ch(void)
{
    if (time_to_change_adc_ch == 0)
    {
        return;
    }

    adc_ch++;
    // 切换adc通道
    // set_adc_ch(adc_cn_arr[adc_ch]);

    set_adc_ch(adc_ch);

    // if (adc_ch > ADC_CHANNEL_NUMBER)
    // {
    //     adc_ch = 0;
    // }

    time_to_change_adc_ch = 0;
}

void uart_send(void)
{
    // static uint8_t counter = 0;
    // counter++;
    // points_data[0] = counter; // 更新帧头
    // HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart1, (const uint8_t *)points_data, FRAME_LEN);
    // memset(tx_buf, 0, OLD_FRAME_LEN); // 清空发送缓冲区

    // fill_tx_data();
    /// tx_data[2] = counter;
    memcpy(&tx_data[2], (const void *)points_data, TOTAL_POINTS / 2);
    memcpy(&tx_data[TOTAL_POINTS / 2 + 8], (const void *)&points_data[TOTAL_POINTS / 2], TOTAL_POINTS / 2);

    // 4元数放在最后16个字节
    memcpy(&tx_data[TOTAL_POINTS + 8], (const void *)q_out, 16);

    // 将数据复制到bl的传输数组中
    memcpy(bl_tx_buf, (const void *)tx_data, OLD_FRAME_LEN);

    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart1, (uint8_t *)tx_data, OLD_FRAME_LEN);

    uart_busy = 1;
}

// 切换数据索引
static void change_point_idx(void)
{
    point_idx++;

    if (point_idx >= TOTAL_POINTS)
    {

        uart_send();
        adc_ch = 0;
        set_adc_ch(adc_ch);
        // set_adc_ch(adc_cn_arr[adc_ch]); // 重置ADC通道
        HAL_Delay(100); // 等待UART发送完成

        // points_data[0] = frame_id;
        // // HAL_UART_Transmit_DMA(&huart1, points_data, FRAME_LEN); // 发送点数据
        // HAL_UART_Transmit_DMA(&huart4, points_data, FRAME_LEN); // 发送点数据
        // uart_busy = 1;
        // HAL_GPIO_WritePin(FOR_TEST1_GPIO_Port, FOR_TEST1_Pin, GPIO_PIN_SET);
        // delay_ms(20);
        point_idx = 0;
    }
}

void main_task(void)
{
    if (uart_busy)
    {
        return;
    }

    // 打开GPIO输出
    // 切换输入gpio, 这样速度最快
    set_channel_pin(input_ch, GPIO_PIN_SET);

    // 开启ADC
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, ADC_BUFFER_SIZE); // != HAL_OK;

    adc_busy = 1;
    while (adc_busy)
        ;

    // 关闭GPIO输出
    set_channel_pin(input_ch, GPIO_PIN_RESET);

    adc_data_handler();

    change_input_ch();

    // 切换逻辑开关
    change_adc_ch();

    // 检查发送数据
    change_point_idx();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uart_busy = 0; // UART发送完成

        check_reset_imu = 1;
    }
    else if (huart->Instance == USART2)
    {
        // 蓝牙发送完成
        bl_uart_tx_done = 1;
    }
}

void init_frame_tail(void)
{

    // 初始化帧尾
    points_data[TOTAL_POINTS] = 0xAA;     // 帧尾第一个字节
    points_data[TOTAL_POINTS + 1] = 0x55; // 帧尾第二个字节
    points_data[TOTAL_POINTS + 2] = 0x03; // 帧尾第三个字节
    points_data[TOTAL_POINTS + 3] = 0x99; // 帧尾第四个字节
}

// 帧头0xaa 0x55 0x03 0x99
// 序号
// 报文类型
// 指令
// 执行结果
void send_data(uint8_t frame_id)
{
    // 帧头
    tx_buf[0] = 0xAA;
    tx_buf[1] = 0x55;
    tx_buf[2] = 0x03;
    tx_buf[3] = 0x99;

    tx_buf[4] = frame_id;           // 帧序号
    tx_buf[5] = CMD_TYPE_RESPONSE;  // 帧类型为响应
    tx_buf[6] = CMD_GET_DATA;       // 指令为获取数据
    tx_buf[7] = CMD_RESULT_SUCCESS; // 执行结果为成功

    memcpy(&tx_buf[8], (const void *)points_data, TOTAL_POINTS); // 拷贝数据到发送缓冲区

    // 计算校验和
    uint8_t sum = 0;
    for (uint16_t i = 0; i < TX_BUF_BYTES - 1; i++)
    {
        sum += tx_buf[i];
    }

    tx_buf[TX_BUF_BYTES - 1] = sum; // 设置校验和

    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart1, (const uint8_t *)tx_buf, TX_BUF_BYTES);
    uart_busy = 1;
}

// const uint8_t adc_idx_v2[] = {
//     7, 6, 5, 4,
//     3, 2, 1, 0,
//     8, 9, 10, 11,
//     12, 13, 14, 15};

const uint8_t adc_idx_v2[] = {
    11, 10, 9, 8,
    3, 2, 1, 0,
    4, 5, 6, 7,
    12, 13, 14, 15};

const uint8_t input_idx_v2[] = {
    7, 6, 5, 4,
    3, 2, 1, 0,
    15, 14, 13, 12,
    11, 10, 9, 8};

void shutdown_led(void)
{
    static uint32_t counter = 0;
    counter++;
    // 一个周期2ms, 20s是
    if (counter == 10000)
    {
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET); // Turn on blue LED
    }
}

void start_adc_collecting(void)
{

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, ADC_BUFFER_SIZE); // != HAL_OK;
}

void imu_rest_cmd_task(void)
{
    static uint8_t imu_reseted_sent = 0;
    if (check_reset_imu)
    {
        if (imu_reseted && imu_reseted_sent == 0)
        {

            HAL_UART_Transmit_DMA(&huart1, (uint8_t *)imu_rest_tx_data, sizeof(imu_rest_tx_data));
            imu_reseted_sent = 1;
        }

        check_reset_imu = 0;
    }

    if (bl_uart_tx_done)
    {
        if (imu_reseted && imu_reseted_sent)
        {

            HAL_UART_Transmit_DMA(&huart2, (uint8_t *)imu_rest_tx_data, sizeof(imu_rest_tx_data));

            imu_reseted_sent = 0;
            imu_reseted = 0;
        }

        bl_uart_tx_done = 0;
    }
}

void main_task_adc_first(void)
{
    // if (uart_busy)
    // {
    //     return;
    // }

    // 诺亦腾的需求, 20秒后关灯.
    // shutdown_led();

    uint16_t input_idx = 0;
    uint16_t adc_idx = 0;
    uint16_t point_nmb = 0;

    for (input_idx = 0; input_idx < INPUT_CH_NUMBER; input_idx++)
    {

        // 先打开GPIO输出
        set_channel_pin(input_idx_v2[input_idx], GPIO_PIN_SET);
        // set_channel_pin(input_idx, GPIO_PIN_SET);

        for (adc_idx = 0; adc_idx < ADC_CHANNEL_NUMBER; adc_idx++)
        {

            set_adc_ch(adc_idx_v2[adc_idx]);

            // delay_us(10);

            // 开启ADC
            // HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, ADC_BUFFER_SIZE); // != HAL_OK;
            // HAL_Delay(1);
            delay_us(3);
            // adc_busy = 1;
            // while (adc_busy)
            //      ;

            // HAL_GPIO_WritePin(HC4067_EN_GPIO_Port, HC4067_EN_Pin, GPIO_PIN_SET);

            point_nmb = input_idx * ADC_CHANNEL_NUMBER + adc_idx;
            adc_data_handler_with_idx(point_nmb);
        }

        // 关闭GPIO输出
        set_channel_pin(input_idx_v2[input_idx], GPIO_PIN_RESET);
        // set_channel_pin(input_idx, GPIO_PIN_RESET);
    }

    // 问答模式使用comm_handler
    uart_send();
}

void delay_init(void)
{
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK); // SysTick频率为HCLK
    fac_us = 170;                                        // 不论是否使用OS,fac_us都需要使用
}

void delay_us(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD; // LOAD的值
    ticks = nus * fac_us;            // 需要的节拍数
    told = SysTick->VAL;             // 刚进入时的计数器值
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow; // 这里注意一下SYSTICK是一个递减的计数器就可以了.
            else
                tcnt += reload - tnow + told;
            told = tnow;
            if (tcnt >= ticks)
                break; // 时间超过/等于要延迟的时间,则退出.
        }
    };
}

// tim7 callback
// 1ms中断一次
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        tim7_counter++;
    }
}

// tools
// uint8_t ema_u8(uint8_t new_data, uint8_t last_data, float alpha)
// {
//     return (uint8_t)(alpha * new_data + (1 - alpha) * last_data);
// }

uint8_t ema_u8(uint8_t new_data, uint8_t last_data, uint8_t a_num, uint8_t a_den)
{
    return (uint8_t)((a_num * new_data + (a_den - a_num) * last_data) / a_den);
}

uint16_t ema_u16(uint16_t new_data, uint16_t last_data, uint16_t a_num, uint16_t a_den)
{
    return (uint16_t)((a_num * new_data + (a_den - a_num) * last_data) / a_den);
}