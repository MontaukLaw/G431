#include "user_comm.h"

/*================= 量程与灵敏度（按你的寄存器配置改） =================*/
// 加速度计灵敏度（LSB/g）: ±2g=16384, ±4g=8192, ±8g=4096, ±16g=2048
#define ACCEL_SENS_LSB_PER_G 16384.0f // 示例：±2g

// 陀螺仪灵敏度（LSB/(°/s)）: ±250=131.072, ±500=65.536, ±1000=32.768, ±2000=16.384
#define GYRO_SENS_LSB_PER_DPS 32.768f // 示例：±1000 dps

/*================= Madgwick 参数 =================*/
#define MADGWICK_BETA 0.05f // 收敛/噪声权衡；0.02~0.1 之间调

/* ========= 低通滤波参数 ========= */
// 一阶 IIR: y = y + alpha*(x - y) ；alpha 越小越“稳”
#ifndef LPF_ALPHA_ACC
#define LPF_ALPHA_ACC 0.2f
#endif
#ifndef LPF_ALPHA_GYRO
#define LPF_ALPHA_GYRO 0.2f
#endif

/* ========= 判定阈值 ========= */
#define A_MIN_G 0.6f         // 允许的 |a| 下限（g）
#define A_MAX_G 1.6f         // 允许的 |a| 上限（g）
#define GYRO_SKIP_DPS 200.0f // 陀螺过大→跳过本帧加计校正
#define CLIP_RAW_NEAR 32000  // 接近满量程视为剪裁
#define EPSF 1e-9f

/* ========= 类型 ========= */
typedef struct
{
    float x, y, z;
} vec3f_t;

typedef struct
{
    float q0, q1, q2, q3;
} quat_t;

/* 低通滤波状态 */
static vec3f_t s_acc_lpf = {0}, s_gyro_lpf = {0};
static bool s_lpf_inited = false;

static quat_t g_q = {1.0f, 0.0f, 0.0f, 0.0f}; // 初始朝向单位四元数

/* ========= 工具函数 ========= */
static inline float inv_sqrtf(float x) { return 1.0f / sqrtf(x); }

static inline void lpf_init(vec3f_t *st, float x, float y, float z)
{
    st->x = x;
    st->y = y;
    st->z = z;
}

static inline void lpf_update(vec3f_t *st, float alpha, float x, float y, float z)
{
    st->x += alpha * (x - st->x);
    st->y += alpha * (y - st->y);
    st->z += alpha * (z - st->z);
}

/* ========= 步骤1：计数 -> 物理单位 ========= */
static void convert_raw_to_units(const icm42688RawData_t *rawA,
                                 const icm42688RawData_t *rawG,
                                 vec3f_t *acc_g, vec3f_t *gyro_dps)
{
    acc_g->x = (float)rawA->x / ACCEL_SENS_LSB_PER_G;
    acc_g->y = (float)rawA->y / ACCEL_SENS_LSB_PER_G;
    acc_g->z = (float)rawA->z / ACCEL_SENS_LSB_PER_G;

    gyro_dps->x = (float)rawG->x / GYRO_SENS_LSB_PER_DPS;
    gyro_dps->y = (float)rawG->y / GYRO_SENS_LSB_PER_DPS;
    gyro_dps->z = (float)rawG->z / GYRO_SENS_LSB_PER_DPS;
}

/* ========= 步骤2：IIR 低通 ========= */
static void lowpass_acc_gyro(vec3f_t *acc_g, vec3f_t *gyro_dps)
{
    if (!s_lpf_inited)
    {
        lpf_init(&s_acc_lpf, acc_g->x, acc_g->y, acc_g->z);
        lpf_init(&s_gyro_lpf, gyro_dps->x, gyro_dps->y, gyro_dps->z);
        s_lpf_inited = true;
    }
    else
    {
        lpf_update(&s_acc_lpf, LPF_ALPHA_ACC, acc_g->x, acc_g->y, acc_g->z);
        lpf_update(&s_gyro_lpf, LPF_ALPHA_GYRO, gyro_dps->x, gyro_dps->y, gyro_dps->z);
    }
    *acc_g = s_acc_lpf;
    *gyro_dps = s_gyro_lpf;
}

/* ========= 步骤3：加速度有效性检查 + 归一化 =========
   通过返回 true 表示“本帧可用加计做校正”，并输出单位向量 axn,ayn,azn */
static bool accel_check_and_normalize(const vec3f_t *acc_g,
                                      const vec3f_t *gyro_dps,
                                      float *axn, float *ayn, float *azn)
{
    // 剪裁/饱和
    // （放在原始层做也可，这里略）

    // 数值合法性
    if (!isfinite(acc_g->x) || !isfinite(acc_g->y) || !isfinite(acc_g->z))
        return false;

    // 幅值窗口（|a| 应接近 1g）
    float an2 = acc_g->x * acc_g->x + acc_g->y * acc_g->y + acc_g->z * acc_g->z;
    if (!isfinite(an2) || an2 < EPSF)
        return false;
    float a_abs = sqrtf(an2);
    if (a_abs < A_MIN_G || a_abs > A_MAX_G)
        return false;

    // 剧烈运动时跳过（陀螺过大）
    float gmax = fmaxf(fabsf(gyro_dps->x), fmaxf(fabsf(gyro_dps->y), fabsf(gyro_dps->z)));
    if (gmax > GYRO_SKIP_DPS)
        return false;

    // 归一化
    float inva = 1.0f / a_abs;
    *axn = acc_g->x * inva;
    *ayn = acc_g->y * inva;
    *azn = acc_g->z * inva;
    return true;
}

/* ========= 步骤4：Madgwick IMU-only 更新 =========
   输入：gyro = dps，内部转为 rad/s；acc = 单位向量（若跳过加计则传 0） */
static void madgwick_update_imu(float gx_dps, float gy_dps, float gz_dps,
                                float ax_n, float ay_n, float az_n,
                                bool use_acc, float dt)
{
    float q0 = g_q.q0, q1 = g_q.q1, q2 = g_q.q2, q3 = g_q.q3;

    // dps -> rad/s
    const float DEG2RAD = 0.017453292519943295f;
    float gx = gx_dps * DEG2RAD;
    float gy = gy_dps * DEG2RAD;
    float gz = gz_dps * DEG2RAD;	

    // 陀螺项
    float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // 加计校正
    if (use_acc)
    {
        float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
        float _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;

        float f1 = _2q1 * q3 - _2q0 * q2 - ax_n;
        float f2 = _2q0 * q1 + _2q2 * q3 - ay_n;
        float f3 = 1.0f - _2q1 * q1 - _2q2 * q2 - az_n;

        float s0 = -_2q2 * f1 + _2q1 * f2;
        float s1 = _2q3 * f1 + _2q0 * f2 - _4q1 * f3;
        float s2 = -_2q0 * f1 + _2q3 * f2 - _4q2 * f3;
        float s3 = _2q1 * f1 + _2q2 * f2;

        float sn = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
        if (sn > EPSF)
        {
            float invsn = inv_sqrtf(sn);
            s0 *= invsn;
            s1 *= invsn;
            s2 *= invsn;
            s3 *= invsn;

            qDot0 -= MADGWICK_BETA * s0;
            qDot1 -= MADGWICK_BETA * s1;
            qDot2 -= MADGWICK_BETA * s2;
            qDot3 -= MADGWICK_BETA * s3;
        }
    }

    // 积分
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    // 归一化 + NaN保护
    float n2 = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
    if (!(n2 > EPSF) || !isfinite(n2))
    {
        q0 = 1.0f;
        q1 = q2 = q3 = 0.0f;
    }
    else
    {
        float invn = inv_sqrtf(n2);
        q0 *= invn;
        q1 *= invn;
        q2 *= invn;
        q3 *= invn;
    }
    g_q.q0 = q0;
    g_q.q1 = q1;
    g_q.q2 = q2;
    g_q.q3 = q3;
}

/* ========= 步骤5：对外总入口（你只需要调用这个） =========
   输入原始计数 rawA/rawG、dt(s)，输出四元数 q_out[4] */
void icm42688_pipeline_update(const icm42688RawData_t *rawA,
                              const icm42688RawData_t *rawG,
                              float dt, float q_out[4])
{
    // 1) 计数->物理
    vec3f_t acc_g, gyro_dps;
    convert_raw_to_units(rawA, rawG, &acc_g, &gyro_dps);

    // 2) 低通
    lowpass_acc_gyro(&acc_g, &gyro_dps);

    // 3) 加计检查+归一化
    float axn = 0, ayn = 0, azn = 0;
    bool use_acc = accel_check_and_normalize(&acc_g, &gyro_dps, &axn, &ayn, &azn);

    // 4) 融合更新
    if (!(dt > 0.0f && dt < 0.5f))
        dt = 0.01f; // 容错
    madgwick_update_imu(gyro_dps.x, gyro_dps.y, gyro_dps.z,
                        axn, ayn, azn, use_acc, dt);

    // 5) 输出四元数
    q_out[0] = g_q.q0; // w
    q_out[1] = g_q.q1; // x
    q_out[2] = g_q.q2; // y
    q_out[3] = g_q.q3; // z
}

/* 可选：复位姿态与滤波器 */
void icm42688_pipeline_reset(void)
{
    g_q.q0 = 1.0f; g_q.q1 = g_q.q2 = g_q.q3 = 0.0f;
    s_lpf_inited = false;  // 下次会用首帧初始化 LPF
}

// static inline void normalize3(float *x, float *y, float *z)
// {
//     float n = (*x) * (*x) + (*y) * (*y) + (*z) * (*z);
//     if (n > 0.0f)
//     {
//         float inv = inv_sqrtf(n);
//         *x *= inv;
//         *y *= inv;
//         *z *= inv;
//     }
// }


// /*================= Madgwick IMU-only 更新 =================*/
// static void madgwick_update_imu(float gx, float gy, float gz,
//                                 float ax, float ay, float az,
//                                 float dt)
// {
//     float q0 = g_q.q0, q1 = g_q.q1, q2 = g_q.q2, q3 = g_q.q3;

//     // 加速度归一化，如果幅值异常（震动/饱和），跳过加计校正
//     float anorm = ax * ax + ay * ay + az * az;
//     bool accel_ok = (anorm > 0.5f && anorm < 2.0f * 2.0f); // 粗略范围：~1g ± 容差
//     if (accel_ok)
//     {
//         float inv = inv_sqrtf(anorm);
//         ax *= inv;
//         ay *= inv;
//         az *= inv;
//     }

//     // 角速度：°/s -> rad/s
//     const float DEG2RAD = 0.017453292519943295f;
//     gx *= DEG2RAD;
//     gy *= DEG2RAD;
//     gz *= DEG2RAD;

//     // ===== 计算 q 的导数（由陀螺项给出）=====
//     float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
//     float qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
//     float qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
//     float qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

//     // ===== 使用加速度（重力）做梯度下降修正 =====
//     if (accel_ok)
//     {
//         // 参考 Madgwick 论文中 IMU-only 的梯度
//         float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
//         float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;
//         float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
//         float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

//         // f(g) 误差：把重力方向约束到 z 轴（在机体系中）
//         float f1 = _2q1 * q3 - _2q0 * q2 - ax;
//         float f2 = _2q0 * q1 + _2q2 * q3 - ay;
//         float f3 = 1.0f - _2q1 * q1 - _2q2 * q2 - az;

//         // 梯度（雅可比转置 * f）
//         float s0 = -_2q2 * f1 + _2q1 * f2;
//         float s1 = _2q3 * f1 + _2q0 * f2 - _4q1 * f3 + _8q1 * (q1q1 + q2q2) * 0.0f; // 简化常见实现
//         float s2 = -_2q0 * f1 + _2q3 * f2 - _4q2 * f3 + _8q2 * (q1q1 + q2q2) * 0.0f;
//         float s3 = _2q1 * f1 + _2q2 * f2;

//         // 归一化梯度
//         float sn = inv_sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3 + 1e-20f);
//         s0 *= sn;
//         s1 *= sn;
//         s2 *= sn;
//         s3 *= sn;

//         // 加计校正项（beta 为步长）
//         qDot0 -= MADGWICK_BETA * s0;
//         qDot1 -= MADGWICK_BETA * s1;
//         qDot2 -= MADGWICK_BETA * s2;
//         qDot3 -= MADGWICK_BETA * s3;
//     }

//     // ===== 积分更新 =====
//     q0 += qDot0 * dt;
//     q1 += qDot1 * dt;
//     q2 += qDot2 * dt;
//     q3 += qDot3 * dt;

//     // 归一化
//     float invn = inv_sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3 + 1e-20f);
//     g_q.q0 = q0 * invn;
//     g_q.q1 = q1 * invn;
//     g_q.q2 = q2 * invn;
//     g_q.q3 = q3 * invn;
// }

// /*================= 对外接口 =================*/
// /**
//  * @brief  从 ICM42688 原始 int16 数据更新四元数
//  * @param  raw_gx,raw_gy,raw_gz  陀螺原始计数（LSB）
//  * @param  raw_ax,raw_ay,raw_az  加速度原始计数（LSB）
//  * @param  dt    采样周期（秒）
//  * @param  q_out 输出四元数 float[4] = {q0,q1,q2,q3}
//  */
// // void icm42688_update_quat(int16_t raw_gx, int16_t raw_gy, int16_t raw_gz,
// //                           int16_t raw_ax, int16_t raw_ay, int16_t raw_az,
// //                           float dt, float q_out[4])
// void icm42688_update_quat(icm42688RawData_t raw_g, icm42688RawData_t raw_a, float dt, float *q_out)
// {
//     // 转换到物理单位
//     float gx_dps = ((float)raw_g.x) / GYRO_SENS_LSB_PER_DPS;
//     float gy_dps = ((float)raw_g.y) / GYRO_SENS_LSB_PER_DPS;
//     float gz_dps = ((float)raw_g.z) / GYRO_SENS_LSB_PER_DPS;

//     float ax_g = ((float)raw_a.x) / ACCEL_SENS_LSB_PER_G;
//     float ay_g = ((float)raw_a.y) / ACCEL_SENS_LSB_PER_G;
//     float az_g = ((float)raw_a.z) / ACCEL_SENS_LSB_PER_G;

//     // 0.5g–2g
//     if (abs(ax_g) > 2.0f || abs(ax_g) < 0.5 || abs(ay_g) > 2.0f || abs(ay_g) < 0.5 || abs(az_g) > 2.0f || abs(az_g) < 0.5)
//     {
//         return;
//     }

//     // 更新融合
//     madgwick_update_imu(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g, dt);

//     // 返回当前四元数
//     q_out[0] = g_q.q0; // w
//     q_out[1] = g_q.q1; // x
//     q_out[2] = g_q.q2; // y
//     q_out[3] = g_q.q3; // z
// }

// /*================= 可选：复位姿态 =================*/
// void icm42688_reset_quat_identity(void)
// {
//     g_q.q0 = 1.0f;
//     g_q.q1 = 0.0f;
//     g_q.q2 = 0.0f;
//     g_q.q3 = 0.0f;
// }
