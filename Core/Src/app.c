#include "user_comm.h"

uint8_t adc_dma_buffer[ADC_BUFFER_SIZE] = {0};
volatile uint8_t adc_busy = 0;
volatile uint8_t uart_busy = 0; // UART是否忙碌
volatile uint16_t input_ch = 0;
volatile uint16_t point_idx = 0;
volatile uint8_t points_data[FRAME_LEN] = {0};
volatile uint8_t time_to_change_adc_ch = 0;
volatile uint16_t adc_ch = 0;

volatile uint8_t tx_buf[TX_BUF_LEN] = {0}; // 发送缓冲区

uint8_t adc_cn_arr[16] = {0, 1, 2, 3,
                          8, 9, 10, 11,
                          4, 5, 6, 7,
                          12, 13, 14, 15};
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
    set_adc_ch(adc_cn_arr[adc_ch]);

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
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart1, (const uint8_t *)points_data, FRAME_LEN);
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

        set_adc_ch(adc_cn_arr[adc_ch]); // 重置ADC通道
        // HAL_Delay(10);                  // 等待UART发送完成

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

    tx_buf[4] = frame_id; // 帧序号

    tx_buf[5] = CMD_TYPE_RESPONSE; // 帧类型为响应
    tx_buf[6] = CMD_GET_DATA;      // 指令为获取数据

    tx_buf[7] = CMD_RESULT_SUCCESS; // 执行结果为成功

    memcpy(&tx_buf[8], points_data, TOTAL_POINTS); // 拷贝数据到发送缓冲区

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
