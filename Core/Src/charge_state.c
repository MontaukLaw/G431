#include "user_comm.h"

void get_charge_state(void)
{
    // 读取充电状态引脚
    GPIO_PinState charge_pin_state = HAL_GPIO_ReadPin(CHRG_GPIO_Port, CHRG_Pin);

    GPIO_PinState standby_pin_state = HAL_GPIO_ReadPin(STDBY_GPIO_Port, STDBY_Pin);

    if (charge_pin_state == GPIO_PIN_SET)
    {
        // 引脚为高电平，表示未充电
        // 在此处添加处理未充电状态的代码
    }
    else
    {
        // 引脚为低电平，表示正在充电
        // 在此处添加处理充电状态的代码
    }
}