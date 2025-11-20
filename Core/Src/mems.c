#include "user_comm.h"
/*================= 量程与灵敏度（按你的寄存器配置改） =================*/
// 加速度计灵敏度（LSB/g）: ±2g=16384, ±4g=8192, ±8g=4096, ±16g=2048
#if MEMS_USING == ICM42688
#define ACCEL_SENS_LSB_PER_G 16384.0f // 示例：±2g
#elif MEMS_USING == QMI8658
#define ACCEL_SENS_LSB_PER_G 2048.0f // 示例：±16g
#endif

#if MEMS_USING == ICM42688
// 陀螺仪灵敏度（LSB/(°/s)）: ±250=131.072, ±500=65.536, ±1000=32.768, ±2000=16.384
#define GYRO_SENS_LSB_PER_DPS 32.768f // 示例：±1000 dps
#elif MEMS_USING == QMI8658
// 陀螺仪灵敏度（LSB/(°/s)）: ±16=2048, ±32=1024, ±64=512, ±128=256
#define GYRO_SENS_LSB_PER_DPS 256.0f // 示例：±128 dps
#endif

// ------- 可调参数 -------
static const float G_NORM = 1.0f;              // 你的 convert_raw_to_units 已输出为 g
static const float ACC_DEV_THR = 0.05f;        // 静止判定：|norm(acc)-1g| < 0.05g
static const float GYR_NORM_THR = 2.0f;        // 静止判定：|gyro| < 2 dps
static const float BIAS_LEARN_RATE = 0.2f;     // 静止时 z 轴零偏学习强度 [1/s]
static const float BIAS_LEARN_RATE_XY = 0.05f; // 静止时 x/y 也可缓慢学习

/*================= Madgwick 参数 =================*/
// #define MADGWICK_BETA 0.05f // 收敛/噪声权衡；0.02~0.1 之间调
#define MADGWICK_BETA 0.2f

/* ========= 低通滤波参数 ========= */
// 一阶 IIR: y = y + alpha*(x - y) ；alpha 越小越“稳”
#ifndef LPF_ALPHA_ACC
#define LPF_ALPHA_ACC 0.18f
#endif
#ifndef LPF_ALPHA_GYRO
#define LPF_ALPHA_GYRO 0.25f
#endif

/* ========= 判定阈值 ========= */
#define A_MIN_G 0.6f         // 允许的 |a| 下限（g）
#define A_MAX_G 1.6f         // 允许的 |a| 上限（g）
#define GYRO_SKIP_DPS 200.0f // 陀螺过大→跳过本帧加计校正
#define CLIP_RAW_NEAR 32000  // 接近满量程视为剪裁
#define EPSF 1e-9f

/* 低通滤波状态 */
static vec3f_t s_acc_lpf = {0}, s_gyro_lpf = {0};
static bool s_lpf_inited = false;

quat_t g_q = {1.0f, 0.0f, 0.0f, 0.0f}; // 初始朝向单位四元数

/* ========= 工具函数 ========= */
static inline float inv_sqrtf(float x) { return 1.0f / sqrtf(x); }

static inline void lpf_init(vec3f_t *st, float x, float y, float z)
{
    st->x = x;
    st->y = y;
    st->z = z;
}

static inline void lpf_update_handle(vec3f_t *st,
                                     float alpha, // 建议 0.20 ~ 0.30
                                     float x, float y, float z)
{
    // 一阶低通
    st->x += alpha * (x - st->x);
    st->y += alpha * (y - st->y);
    st->z += alpha * (z - st->z);
}

static inline void lpf_update_fast(vec3f_t *st,
                                   float base_alpha,
                                   float x, float y, float z,
                                   float gyro_norm)
{
    // 1) 动态放宽带宽
    float alpha = base_alpha;

    float k = gyro_norm * 0.12f; // 根据运动强度提升带宽
    if (k > 1.0f)
        k = 1.0f;
    alpha = base_alpha + (0.25f * k); // 最大 0.25

    // 2) 低频 (LPF)
    float lp_x = st->x + alpha * (x - st->x);
    float lp_y = st->y + alpha * (y - st->y);
    float lp_z = st->z + alpha * (z - st->z);

    // 3) 高频增强 (HPF 融合)
    float hp_x = x - lp_x;
    float hp_y = y - lp_y;
    float hp_z = z - lp_z;

    // 高频保留 50%
    st->x = lp_x + 0.5f * hp_x;
    st->y = lp_y + 0.5f * hp_y;
    st->z = lp_z + 0.5f * hp_z;
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

void lowpass_acc_gyro(vec3f_t *acc_g, vec3f_t *gyro_dps)
{
    if (!s_lpf_inited)
    {
        lpf_init(&s_acc_lpf, acc_g->x, acc_g->y, acc_g->z);
        lpf_init(&s_gyro_lpf, gyro_dps->x, gyro_dps->y, gyro_dps->z);
        s_lpf_inited = true;
    }
    else
    {
        // 手柄版参数
        lpf_update_handle(&s_acc_lpf, 0.6f, acc_g->x, acc_g->y, acc_g->z);
        lpf_update_handle(&s_gyro_lpf, 0.8f, gyro_dps->x, gyro_dps->y, gyro_dps->z);

        // lpf_update_handle(&s_acc_lpf, 0.18f, acc_g->x, acc_g->y, acc_g->z);
        // lpf_update_handle(&s_gyro_lpf, 0.5f, gyro_dps->x, gyro_dps->y, gyro_dps->z);
    }

    *acc_g = s_acc_lpf;
    *gyro_dps = s_gyro_lpf;
}

bool accel_check_and_normalize(vec3f_t *a, vec3f_t *g, float *ax, float *ay, float *az)
{
    float an = sqrt(a->x * a->x + a->y * a->y + a->z * a->z);
    float gn = sqrt(g->x * g->x + g->y * g->y + g->z * g->z);

    // 加速度必须接近 1g （±0.1g）
    if (an < 0.9f || an > 1.1f)
        return false;

    // 陀螺角速度 < 40°/s 才认为是"慢动作"
    if (gn > (40.0f * DEG2RAD))
        return false;

    float inv = 1.0f / an;
    *ax = a->x * inv;
    *ay = a->y * inv;
    *az = a->z * inv;

    return true;
}

/* ========= 步骤4：Madgwick IMU-only 更新 =========
   输入：gyro = dps，内部转为 rad/s；acc = 单位向量（若跳过加计则传 0） */
void madgwick_update_imu(float gx_dps, float gy_dps, float gz_dps, float ax_n, float ay_n, float az_n, bool use_acc, float dt)
{
    float q0 = g_q.q0, q1 = g_q.q1, q2 = g_q.q2, q3 = g_q.q3;

    // dps -> rad/s

#if MEMS_USING == ICM42688
    float gx = gx_dps * DEG2RAD;
    float gy = gy_dps * DEG2RAD;
    float gz = gz_dps * DEG2RAD;
#elif MEMS_USING == QMI8658
    float gx = gx_dps;
    float gy = gy_dps;
    float gz = gz_dps;
#endif

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

// ------- 运行时状态 -------
vec3f_t g_gyro_bias_dps = {0}; // 陀螺零偏（dps）
float g_run_time = 0.0f;
int g_startup_cal_done = 0;

// （可选）静止期偏航保持
static int g_yaw_hold_enabled = 1;
static float g_hold_yaw_rad = 0.0f;  // 记录一个“可信 yaw”
static float g_yaw_hold_gain = 0.5f; // 静止时把当前 yaw 以 0.5 [1/s] 拉回

static inline float vec_norm(vec3f_t v) { return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z); }

int is_stationary(const vec3f_t *acc_g, const vec3f_t *gyr_dps)
{
    float acc_dev = fabsf(vec_norm(*acc_g) - G_NORM);
    float gyr_mag = vec_norm(*gyr_dps);
    return (acc_dev < ACC_DEV_THR) && (gyr_mag < GYR_NORM_THR);
}

// 欧拉提取/替换（Z-Y-X，适合只替换 yaw）
static void quat_to_euler_zyx(float q[4], float *roll, float *pitch, float *yaw)
{
    // q = [w,x,y,z]
    float w = q[0], x = q[1], y = q[2], z = q[3];
    // roll (x)
    float sinr_cosp = 2.f * (w * x + y * z);
    float cosr_cosp = 1.f - 2.f * (x * x + y * y);
    *roll = atan2f(sinr_cosp, cosr_cosp);
    // pitch (y)
    float sinp = 2.f * (w * y - z * x);
    *pitch = (fabsf(sinp) >= 1.f) ? copysignf(M_PI / 2.f, sinp) : asinf(sinp);
    // yaw (z)
    float siny_cosp = 2.f * (w * z + x * y);
    float cosy_cosp = 1.f - 2.f * (y * y + z * z);
    *yaw = atan2f(siny_cosp, cosy_cosp);
}

static void euler_zyx_to_quat(float roll, float pitch, float yaw, float q[4])
{
    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);
    q[0] = cy * cp * cr + sy * sp * sr; // w
    q[1] = cy * cp * sr - sy * sp * cr; // x
    q[2] = cy * sp * cr + sy * cp * sr; // y
    q[3] = sy * cp * cr - cy * sp * sr; // z
}

void startup_bias_calib(const vec3f_t *gyro_dps, float dt)
{
    static vec3f_t sum = {0};
    static float t = 0.f;

    if (g_startup_cal_done)
        return;

    sum.x += gyro_dps->x * dt;
    sum.y += gyro_dps->y * dt;
    sum.z += gyro_dps->z * dt;
    t += dt;

    if (t >= STARTUP_CAL_TIME)
    {
        g_gyro_bias_dps.x = sum.x / t;
        g_gyro_bias_dps.y = sum.y / t;
        g_gyro_bias_dps.z = sum.z / t;
        g_startup_cal_done = 1;
    }
}

void online_bias_learn(const vec3f_t *gyro_dps, int stationary, float dt)
{
    if (!stationary)
        return;

    // 目标：静止时角速度应为 0 → 把 bias 往当前读数“回拉”
    g_gyro_bias_dps.z += BIAS_LEARN_RATE * (gyro_dps->z - g_gyro_bias_dps.z) * dt;

    // 也可以给 x/y 极低速学习，抑制温飘/累积误差
    g_gyro_bias_dps.x += BIAS_LEARN_RATE_XY * (gyro_dps->x - g_gyro_bias_dps.x) * dt;
    g_gyro_bias_dps.y += BIAS_LEARN_RATE_XY * (gyro_dps->y - g_gyro_bias_dps.y) * dt;
}

void yaw_hold_when_stationary(float q[4], int stationary, float dt)
{

    // if (!g_yaw_hold_enabled)
    //     return;

    float r, p, y;
    quat_to_euler_zyx(q, &r, &p, &y);

    if (stationary)
    {
        // 缓慢把当前 yaw 拉向 g_hold_yaw_rad
        float dy = g_hold_yaw_rad - y;

        // 把误差 wrap 到 (-pi, pi)
        while (dy > M_PI)
            dy -= 2.f * M_PI;
        while (dy < -M_PI)
            dy += 2.f * M_PI;

        float y_corr = y + g_yaw_hold_gain * dy * dt;
        euler_zyx_to_quat(r, p, y_corr, q);
    }
    else
    {
        // 运动时更新“可信 yaw”为当前 yaw（或你也可低通）
        g_hold_yaw_rad = y;
    }
}

/* ========= 步骤5：对外总入口（你只需要调用这个） =========
   输入原始计数 rawA/rawG、dt(s)，输出四元数 q_out[4] */
void icm42688_pipeline_update(const icm42688RawData_t *rawA,
                              const icm42688RawData_t *rawG,
                              float dt)
{
    // 1) 计数->物理
    vec3f_t acc_g, gyro_dps;
    convert_raw_to_units(rawA, rawG, &acc_g, &gyro_dps);

    // NEW: 启动静置标定（前3秒）
    // if (dt <= 0.f || dt > 0.5f)
    //     dt = 0.01f;

    g_run_time += dt;
    if (!g_startup_cal_done && g_run_time <= STARTUP_CAL_TIME + 0.5f)
    {
        startup_bias_calib(&gyro_dps, dt);
    }

    // 2) 低通
    lowpass_acc_gyro(&acc_g, &gyro_dps);

    // NEW: 在线零偏学习（静止时）
    int stationary = is_stationary(&acc_g, &gyro_dps);
    online_bias_learn(&gyro_dps, stationary, dt);

    // NEW: 应用零偏（一定要在送入滤波/融合前减去）
    gyro_dps.x -= g_gyro_bias_dps.x;
    gyro_dps.y -= g_gyro_bias_dps.y;
    gyro_dps.z -= g_gyro_bias_dps.z;

    // 3) 加计检查+归一化
    float axn = 0, ayn = 0, azn = 0;
    bool use_acc = accel_check_and_normalize(&acc_g, &gyro_dps, &axn, &ayn, &azn);

    // 4) 融合更新
    // if (!(dt > 0.0f && dt < 0.5f))
    //     dt = 0.01f; // 容错

    madgwick_update_imu(gyro_dps.x, gyro_dps.y, gyro_dps.z, axn, ayn, azn, use_acc, dt);

    // 5) 输出四元数
    q_out[0] = g_q.q0; // w
    q_out[1] = g_q.q1; // x
    q_out[2] = g_q.q2; // y
    q_out[3] = g_q.q3; // z

    // NEW: （可选）静止时偏航保持，抑制残余慢漂
    yaw_hold_when_stationary(q_out, stationary, dt);
}

/* 可选：复位姿态与滤波器 */
void icm42688_pipeline_reset(void)
{
    g_q.q0 = 1.0f;
    g_q.q1 = g_q.q2 = g_q.q3 = 0.0f;
    s_lpf_inited = false; // 下次会用首帧初始化 LPF
}
