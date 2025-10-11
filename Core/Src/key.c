#include "user_comm.h"

uint8_t imu_reseted = 0;

void key_task_test(void)
{

    if (HAL_GPIO_ReadPin(POWER_KEY_GPIO_Port, POWER_KEY_Pin) == GPIO_PIN_RESET)
    {
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
    }
}

static uint32_t key_down_counter = 0;
// 10ms执行一次
void key_task(void)
{
    static uint32_t last_run_tck = 0;
    uint32_t now = HAL_GetTick();


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

            // 关灯
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET); // Turn off blue LED
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
            while (1)
            {
                // 拉高PowerCtrl
                HAL_GPIO_WritePin(POWER_CTRL_GPIO_Port, POWER_CTRL_Pin, GPIO_PIN_RESET);
                // 等待关机`
            }
        }
    }
    else
    {
        if (key_down_counter >= RESET_GQ_KEY_SHAKE_DELAY) // 1s
        {
            // key_down_counter = 0;
            imu_reseted = 1;
            // 陀螺仪复位
            icm42688_pipeline_reset();
        }
        key_down_counter = 0;
    }

    last_run_tck = now;
}