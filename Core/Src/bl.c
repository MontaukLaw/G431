#include "user_comm.h"

void bl_task(void)
{
    static uint32_t last_send_tick = 0;
    
    // 调节帧率
    if (HAL_GetTick() - last_send_tick < 100)
        return;
        
    last_send_tick = HAL_GetTick();


}