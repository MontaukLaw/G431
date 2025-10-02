#include "user_comm.h"

icm42688RawData_t acc, gyro;
icm42688Float3_t acc_g, gyro_dps;
icm42688RawData_t acc_mapped, gyro_mapped;

float q_out[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // 初始四元数

// 10ms执行一次.
void gsensor_task(void)
{

    static uint32_t last_run_tck = 0;
    uint32_t now = HAL_GetTick();

    // 调节帧率
    if (now - last_run_tck < 10)
        return;

    icm42688_read_accel(&acc);
    icm42688_read_gyro(&gyro);

    acc_mapped = map_vec3_by_orientation(acc);
    gyro_mapped = map_vec3_by_orientation(gyro);

    float dt = (now - last_run_tck) * 0.001f; // s

    // icm42688_pipeline_update(&acc, &gyro, dt, q_out);
    icm42688_pipeline_update(&acc_mapped, &gyro_mapped, dt, q_out);

    // char tx_buf[96];
    // int n = snprintf(tx_buf, sizeof(tx_buf),
    //                  "%.4f, %.4f, %.4f, %.4f\r\n",
    //                  g_q[0], g_q[1], g_q[2], g_q[3]);
    // if (n > 0)
    // {
    //     HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, (uint16_t)n, 50);
    // }

    last_run_tck = now;
}