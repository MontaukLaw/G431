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

void all_led_off(void)
{
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
}

void power_led_control(void)
{
    static uint32_t last_change_tick = 0;

    uint32_t now = HAL_GetTick();

    // 2s切换一次
    if (now - last_change_tick < 2000)
        return;

    last_change_tick = now;

    // if (bat_smooth_mvolts > 3750)
    // {
    //     // 绿灯常亮
    //     all_led_off();
    //     HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
    // }
    // else if (bat_smooth_mvolts < 3700 && bat_smooth_mvolts > 3500)
    // {
    //     /// 关绿灯
    //     all_led_off();
    //     // 红灯常亮
    //     HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
    // }
    // else if (bat_smooth_mvolts <= 3500)
    // {

    //     all_led_off();
    //     // 低电量, 红灯闪烁
    //     red_blink();
    // }

    if (bat_smooth_mvolts > 3300)
    {
        // 蓝色灯常亮
        all_led_off();
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
    }
    else if (bat_smooth_mvolts < 3250 && bat_smooth_mvolts > 3100)
    {
        all_led_off();
        // 绿灯常亮
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
    }
    else if (bat_smooth_mvolts < 3050 && bat_smooth_mvolts > 3000)
    {
        /// 关绿灯
        all_led_off();
        // 红灯常亮
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
    }
    else if (bat_smooth_mvolts <= 3000)
    {

        all_led_off();
        // 低电量, 红灯闪烁
        red_blink();
    }
}

// 中间间隔避免频繁切换
//
void power_led_control__(void)
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
    else if (bat_smooth_percentage < 19 && bat_smooth_percentage > 10)
    {
        /// 关绿灯
        all_led_off();
        // HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
        // 红灯常亮
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
    }
    else if (bat_smooth_percentage < 9)
    {

        all_led_off();

        /// 关绿灯
        // HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);

        // 低电量, 红灯闪烁
        red_blink();
    }
}

void blink_blue(void)
{
    static uint32_t last_change_tick = 0;

    uint32_t now = HAL_GetTick();

    // 1s切换一次
    if (now - last_change_tick < 1000)
        return;

    last_change_tick = now;

    static uint8_t led_state = 0;

    if (led_state)
    {
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
        led_state = 0;
    }
    else
    {
        all_led_off();
        led_state = 1;
    }
}

void white_led(void)
{
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
}

void led_ctl(void)
{
    // 最高优先级
    if (charge_state == CHARGING_ONGOING)
    {
        white_led();
    }
    else if (bl_conn_status == 0)
    {
        blink_blue();
    }
    else
    {
        power_led_control();
    }
}

void led_task(void)
{

    if (HAL_GPIO_ReadPin(POWER_KEY_GPIO_Port, POWER_KEY_Pin) == GPIO_PIN_RESET)
    {
        // // 全关
        // HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET); // Turn off blue LED
        // HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);

        all_led_off();
        // 仅仅亮红灯
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);

        return;
    }

    led_ctl();
    // power_led_control();
}