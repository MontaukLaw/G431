#ifndef _MEMS_H_
#define _MEMS_H_

#define STARTUP_CAL_TIME 3.0f // 上电静置标定窗口（秒）

/* ========= 类型 ========= */
typedef struct
{
    float x, y, z;
} vec3f_t;

typedef struct
{
    float q0, q1, q2, q3;
} quat_t;

void icm42688_reset_quat_identity(void);

void icm42688_update_quat(icm42688RawData_t raw_g, icm42688RawData_t raw_a, float dt, float q_out[4]);

void icm42688_pipeline_update(const icm42688RawData_t *rawA, const icm42688RawData_t *rawG, float dt);

void icm42688_pipeline_reset(void);

void startup_bias_calib(const vec3f_t *gyro_dps, float dt);

void lowpass_acc_gyro(vec3f_t *acc_g, vec3f_t *gyro_dps);

int is_stationary(const vec3f_t *acc_g, const vec3f_t *gyr_dps);

void online_bias_learn(const vec3f_t *gyro_dps, int stationary, float dt);

// bool accel_check_and_normalize(const vec3f_t *acc_g, const vec3f_t *gyro_dps, float *axn, float *ayn, float *azn);

void madgwick_update_imu(float gx_dps, float gy_dps, float gz_dps, float ax_n, float ay_n, float az_n, bool use_acc, float dt);

void yaw_hold_when_stationary(float q[4], int stationary, float dt);

bool accel_check_and_normalize(vec3f_t *a, vec3f_t *g, float *ax, float *ay, float *az);

extern float g_run_time;
extern int g_startup_cal_done;

extern vec3f_t g_gyro_bias_dps;
extern quat_t g_q;

// 陀螺仪4元数
extern float q_out[];

#endif
