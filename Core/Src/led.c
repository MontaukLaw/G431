#include "user_comm.h"

extern uint8_t bat_smooth_percentage;

void red_blink(void)
{
    static uint32_t last_blink_tick = 0;
    static uint8_t led_state = 0;

    if (led_state)
    {
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
        led_state = 0;
    }
    else
    {
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
        led_state = 1;
    }
}

// 中间间隔避免频繁切换
//
void power_led_control(void)
{
    static uint32_t last_change_tick = 0;

    uint32_t now = HAL_GetTick();

    // 1s切换一次
    if (now - last_change_tick < 1000)
        return;

    last_change_tick = now;

    // if (bat_smooth_percentage > 80)
    // {
    //
    //     HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
    // }
    // else if (bat_smooth_percentage > 20 && bat_smooth_percentage < 78)
    // {
    // }
    // else if (bat_smooth_percentage < 18)
    // {
    //     // 低电量, 红灯闪烁
    //     red_blink();
    // }
    if (bat_smooth_percentage > 20)
    {
        // 绿灯常亮
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
    }
    else if (bat_smooth_percentage < 19)
    {
        // 低电量, 红灯闪烁
        red_blink();
    }
}

void led_task(void)
{

    power_led_control();
}