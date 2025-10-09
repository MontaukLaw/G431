#include "user_comm.h"

uint8_t charge_state = CHARGING_NO_POWER;
// 状态	CHARGE	STANDBY	说明
// 充电中	0	1	电池正在充电
// 充满	1	0	电池已充满
// 未插 USB / 无供电	1	1	芯片未工作或无输入
void charge_state_task(void)
{

    static uint32_t last_send_tick = 0;

    // 调节帧率
    if (HAL_GetTick() - last_send_tick < 2000)
        return;

    last_send_tick = HAL_GetTick();

    // 读取充电状态引脚
    GPIO_PinState charge_pin_state = HAL_GPIO_ReadPin(CHRG_GPIO_Port, CHRG_Pin);

    GPIO_PinState standby_pin_state = HAL_GPIO_ReadPin(STDBY_GPIO_Port, STDBY_Pin);

    if (charge_pin_state == GPIO_PIN_SET && standby_pin_state == GPIO_PIN_RESET)
    {
        // 引脚状态为 (1, 0)，表示充电已完成
        // 在此处添加处理充电完成状态的代码
        charge_state = CHARGING_FINISHED;
    }
    else if (charge_pin_state == GPIO_PIN_RESET && standby_pin_state == GPIO_PIN_SET)
    {
        // 引脚状态为 (0, 1)，表示正在充电
        // 在此处添加处理正在充电状态的代码
        charge_state = CHARGING_ONGOING;
    }
    else if (charge_pin_state == GPIO_PIN_SET && standby_pin_state == GPIO_PIN_SET)
    {
        // 引脚状态为 (1, 1)，表示未插 USB 或无供电
        // 在此处添加处理未插 USB 或无供电状态的代码
        charge_state = CHARGING_NO_POWER;
    }
}