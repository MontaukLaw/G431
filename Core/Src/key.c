#include "user_comm.h"

// 10ms执行一次
void key_task(void)
{
    static uint32_t last_run_tck = 0;
    uint32_t now = HAL_GetTick();
    static uint8_t key_down_counter = 0;

    // 调节帧率
    if (now - last_run_tck < 10)
        return;

    if (HAL_GPIO_ReadPin(POWER_KEY_GPIO_Port, POWER_KEY_Pin) == GPIO_PIN_RESET)
    {
        // KEY1按下
        key_down_counter++;
        // 6秒关机
        if (key_down_counter >= PWOER_DOWN_COUNTER)
        {
            // 拉高PowerCtrl
            HAL_GPIO_WritePin(POWER_CTRL_GPIO_Port, POWER_CTRL_Pin, GPIO_PIN_RESET);
            while (1)
            {
                // 等待关机
            }
        }
        // 大于50ms
        else if (key_down_counter >= RESET_GQ_KEY_SHAKE_DELAY) // 1s
        {
            key_down_counter = 0;

            // 陀螺仪复位
            icm42688_pipeline_reset();
        }
    }
    else
    {
        key_down_counter = 0;
    }

    last_run_tck = now;
}