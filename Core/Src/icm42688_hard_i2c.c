#include "user_comm.h"

static uint8_t icm42688_iic_read_reg(uint8_t reg)
{
    uint8_t regval = 0;
    /* 读 1 字节 */
    HAL_I2C_Master_Transmit(&hi2c3, ICM42688_ADDRESS << 1, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c3, ICM42688_ADDRESS << 1, &regval, 1, HAL_MAX_DELAY);
    return regval;
}

static void icm42688_iic_write_reg(uint8_t reg, uint8_t value)
{

    // HAL_I2C_Master_Transmit(&hi2c3, ICM42688_ADDRESS << 1, &reg, 1, HAL_MAX_DELAY);
    // HAL_I2C_Master_Transmit(&hi2c3, ICM42688_ADDRESS << 1, &value, 1, HAL_MAX_DELAY);

    uint8_t data[2];
    data[0] = reg;
    data[1] = value;
    HAL_I2C_Master_Transmit(&hi2c3, ICM42688_ADDRESS << 1, data, 2, HAL_MAX_DELAY);
}

static void icm42688_iic_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_I2C_Master_Transmit(&hi2c3, ICM42688_ADDRESS << 1, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c3, ICM42688_ADDRESS << 1, buf, len, HAL_MAX_DELAY);
}

void init_42688(void)
{
    /* 1) 软复位（Bank0: DEVICE_CONFIG=0x11, 写 0x01） */
    icm42688_iic_write_reg(ICM42688_DEVICE_CONFIG, 0x01);
    HAL_Delay(30); // 软复位完成

    /* 2) WHO_AM_I 校验 */
    uint8_t who = icm42688_iic_read_reg(ICM42688_WHO_AM_I);
    if (who != ICM42688_ID)
    {
        return; // 芯片不在或地址不对
    }

    /* 3) 选 Bank0 */
    icm42688_iic_write_reg(ICM42688_REG_BANK_SEL, 0x00);

    /* 4) 先开温度 + GYRO/ACCEL 低噪声(LN) */
    uint8_t pwr = icm42688_iic_read_reg(ICM42688_PWR_MGMT0);
    pwr &= ~(1u << 5);   // TEMP_DIS=0 → 使能温度
    pwr &= ~(0x3u << 2); // 清 GYRO_MODE
    pwr |= (0x3u << 2);  // 11 = Gyro LN
    pwr &= ~(0x3u << 0); // 清 ACCEL_MODE
    pwr |= (0x3u << 0);  // 11 = Accel LN
    icm42688_iic_write_reg(ICM42688_PWR_MGMT0, pwr);

    /* 模式切换后给一点时间（≥200µs，取 1ms 更稳） */
    HAL_Delay(30);

    /* 5) 配 ACCEL: ±2g, 100Hz（先清字段再写） */
    bsp_Icm42688GetAres(AFS_2G); // 设置与寄存器一致的灵敏度刻度
    uint8_t v = icm42688_iic_read_reg(ICM42688_ACCEL_CONFIG0);
    v &= ~((uint8_t)(0x7u << 5)); // 清 FS [7:5]
    v &= ~((uint8_t)0x0Fu);       // 清 ODR[3:0]
    v |= (uint8_t)((AFS_2G & 0x7u) << 5);
    v |= (uint8_t)(AODR_100Hz & 0x0Fu); // 100Hz
    icm42688_iic_write_reg(ICM42688_ACCEL_CONFIG0, v);

    /* 6) 配 GYRO: ±1000 dps, 100Hz（先清再写） */
    bsp_Icm42688GetGres(GFS_1000DPS);
    uint8_t g = icm42688_iic_read_reg(ICM42688_GYRO_CONFIG0);
    g &= ~((uint8_t)(0x7u << 5));              // 清 FS [7:5]
    g &= ~((uint8_t)0x0Fu);                    // 清 ODR[3:0]
    g |= (uint8_t)((GFS_1000DPS & 0x7u) << 5); // 001 = ±1000 dps
    g |= (uint8_t)(GODR_100Hz & 0x0Fu);        // 100Hz
    icm42688_iic_write_reg(ICM42688_GYRO_CONFIG0, g);

    /* 7) 稳定一下再开始读数据 */
    HAL_Delay(30);
}

void init_42688_(void)
{

    icm42688_iic_write_reg(ICM42688_DEVICE_CONFIG, 0x01);
    HAL_Delay(2);

    uint8_t reg_val = 0;
    /* 读取 who am i 寄存器 */
    reg_val = icm42688_iic_read_reg(ICM42688_WHO_AM_I);

    if (reg_val != ICM42688_ID)
    {
        return;
    }

    // while (1)
    // {
    //     icm42688_iic_read_reg(ICM42688_WHO_AM_I);
    //     HAL_Delay(100);
    // }

    // return;

    HAL_Delay(1);
    bsp_Icm42688GetAres(AFS_2G);
    icm42688_iic_write_reg(ICM42688_REG_BANK_SEL, 0x00); // Bank 0

    uint8_t v = icm42688_iic_read_reg(ICM42688_ACCEL_CONFIG0);

    /* 清空 FS 和 ODR 两个字段（保留 bit4=0） */
    v &= ~((uint8_t)(0x7u << 5)); // 清 [7:5] ACCEL_FS_SEL
    v &= ~((uint8_t)0x0Fu);       // 清 [3:0] ACCEL_ODR

    /* 写入目标配置（确保枚举值与手册一致：AFS_2G=0b011, AODR_100Hz=0b1000） */
    v |= (uint8_t)((AFS_2G & 0x7u) << 5);
    v |= (uint8_t)(AODR_100Hz & 0x0Fu);
    icm42688_iic_write_reg(ICM42688_ACCEL_CONFIG0, v);

    bsp_Icm42688GetGres(GFS_1000DPS);
    // 选 Bank0
    icm42688_iic_write_reg(ICM42688_REG_BANK_SEL, 0x00);

    // 2) 配置 GYRO_CONFIG0：±1000 dps，100 Hz
    uint8_t g = icm42688_iic_read_reg(ICM42688_GYRO_CONFIG0);
    g &= ~((uint8_t)(0x7u << 5)); // 清 GYRO_FS_SEL[7:5]
    g &= ~((uint8_t)0x0Fu);       // 清 GYRO_ODR[3:0]

    g |= (uint8_t)((GFS_1000DPS & 0x7u) << 5); // 001 = ±1000 dps
    g |= (uint8_t)(GODR_100Hz & 0x0Fu);        // 1000 = 100 Hz
    icm42688_iic_write_reg(ICM42688_GYRO_CONFIG0, g);

    // 选 Bank0
    icm42688_iic_write_reg(ICM42688_REG_BANK_SEL, 0x00);

    // 配置 PWR_MGMT0: 使能温度, Gyro/Accel 低噪声(LN)
    v = icm42688_iic_read_reg(ICM42688_PWR_MGMT0);

    // 清相关位: TEMP_DIS(5), GYRO_MODE[3:2], ACCEL_MODE[1:0]
    v &= ~(1u << 5);   // TEMP_DIS=0 → 使能温度
    v &= ~(0x3u << 2); // 清 GYRO_MODE
    v &= ~(0x3u << 0); // 清 ACCEL_MODE

    // 置目标模式: GYRO_MODE=11 (LN), ACCEL_MODE=11 (LN)
    v |= (0x3u << 2);
    v |= (0x3u << 0);

    icm42688_iic_write_reg(ICM42688_PWR_MGMT0, v);

    // 从 OFF → 其它模式后 200µs 内不要再写寄存器；取 1ms 缓冲更稳妥
    ICM42688DelayMs(1);
}

// 可选：在驱动里维护当前刻度（用于工程量换算）
static float g_accSens_mg_per_lsb = 2000.0f / 32768.0f;   // AFS_2G
static float g_gyroSens_dps_per_lsb = 1000.0f / 32768.0f; // GFS_1000DPS

static void icm42688_set_acc_sens(uint8_t Ascale)
{
    // 2/4/8/16 g → mg/LSB
    switch (Ascale)
    {
    case AFS_2G:
        g_accSens_mg_per_lsb = 2000.0f / 32768.0f;
        break;
    case AFS_4G:
        g_accSens_mg_per_lsb = 4000.0f / 32768.0f;
        break;
    case AFS_8G:
        g_accSens_mg_per_lsb = 8000.0f / 32768.0f;
        break;
    case AFS_16G:
        g_accSens_mg_per_lsb = 16000.0f / 32768.0f;
        break;
    }
}
static void icm42688_set_gyro_sens(uint8_t Gscale)
{
    // 15.125/31.25/.../2000 dps → dps/LSB
    switch (Gscale)
    {
    case GFS_1000DPS:
        g_gyroSens_dps_per_lsb = 1000.0f / 32768.0f;
        break;
        // 其他挡位按需补齐
    }
}

#define MASK_TEMP_DIS (1u << 5)     // 0 = 使能温度
#define MASK_GYRO_MODE (0x3u << 2)  // 11 = Gyro LN
#define MASK_ACCEL_MODE (0x3u << 0) // 11 = Accel LN

bool icm42688_set_power_ln_and_verify(void)
{
    /* 确保在 Bank0 */
    icm42688_iic_write_reg(ICM42688_REG_BANK_SEL, 0x00);

    /* 目标：温度开、Gyro LN、Accel LN → 0b0000_1111 = 0x0F */
    uint8_t target = 0x0F;

    icm42688_iic_write_reg(ICM42688_PWR_MGMT0, target);

    /* 关键：等待 ≥200us（建议 1ms）再访问任何寄存器 */
    HAL_Delay(1);

    /* 读回校验（保守起见屏蔽保留位，仅校验相关位） */
    __IO uint8_t v = icm42688_iic_read_reg(ICM42688_PWR_MGMT0);

    bool ok_temp = ((v & MASK_TEMP_DIS) == 0);              // TEMP_DIS=0
    bool ok_gyro = ((v & MASK_GYRO_MODE) == (0x3u << 2));   // GYRO_MODE=11
    bool ok_accel = ((v & MASK_ACCEL_MODE) == (0x3u << 0)); // ACCEL_MODE=11

    return ok_temp && ok_gyro && ok_accel;
}

bool icm42688_init(void)
{
    // 1) 软复位（Bank0: DEVICE_CONFIG=0x11，写 0x01）
    icm42688_iic_write_reg(ICM42688_DEVICE_CONFIG, 0x01);
    HAL_Delay(2); // 软复位完成

    // 2) WHO_AM_I 校验（应=0x47）
    uint8_t who = icm42688_iic_read_reg(ICM42688_WHO_AM_I);
    if (who != ICM42688_ID)
    {
        return false; // 地址/连线/接口模式异常
    }

    // 设置并核验 PWR_MGMT0
    if (!icm42688_set_power_ln_and_verify())
    {
        // 失败：可重试一次或执行软复位再来
        // 也可以把 v 打印出来定位是哪几位没生效
        return false;
    }

    // // 3) 选 Bank0（稳妥起见）
    // icm42688_iic_write_reg(ICM42688_REG_BANK_SEL, 0x00);

    // // 4) 先开电源路径：温度开、Gyro/Accel = LN 模式
    // uint8_t pwr = icm42688_iic_read_reg(ICM42688_PWR_MGMT0);
    // pwr &= ~(1u << 5);                        // TEMP_DIS=0（使能温度）
    // pwr = (pwr & ~(0x3u << 2)) | (0x3u << 2); // GYRO_MODE=11 (LN)
    // pwr = (pwr & ~(0x3u << 0)) | (0x3u << 0); // ACCEL_MODE=11 (LN)
    // icm42688_iic_write_reg(ICM42688_PWR_MGMT0, pwr);
    HAL_Delay(1); // OFF→其它模式后 ≥200µs

    // 5) 配 Accel：±2g，100Hz（先清字段再写）
    uint8_t v = icm42688_iic_read_reg(ICM42688_ACCEL_CONFIG0);
    v &= ~((uint8_t)(0x7u << 5)); // 清 FS [7:5]
    v &= ~((uint8_t)0x0Fu);       // 清 ODR[3:0]
    v |= (uint8_t)((AFS_2G & 0x7u) << 5);
    v |= (uint8_t)(AODR_100Hz & 0x0Fu);
    icm42688_iic_write_reg(ICM42688_ACCEL_CONFIG0, v);
    icm42688_set_acc_sens(AFS_2G);

    // 6) 配 Gyro：±1000 dps，100Hz（先清再写）
    uint8_t g = icm42688_iic_read_reg(ICM42688_GYRO_CONFIG0);
    g &= ~((uint8_t)(0x7u << 5)); // 清 FS [7:5]
    g &= ~((uint8_t)0x0Fu);       // 清 ODR[3:0]
    g |= (uint8_t)((GFS_1000DPS & 0x7u) << 5);
    g |= (uint8_t)(GODR_100Hz & 0x0Fu);
    icm42688_iic_write_reg(ICM42688_GYRO_CONFIG0, g);
    icm42688_set_gyro_sens(GFS_1000DPS);

    // （可选）设置 LPF/带宽：GYRO_ACCEL_CONFIG0 按需配置

    // 7) 等待 ≥1 个 ODR 周期，确保首帧已产生（100Hz→≥10ms）
    HAL_Delay(10);

    return true;
}

void iic_get_data(icm42688RawData_t *accData, icm42688RawData_t *gyroData)
{

    // uint8_t buffer[12] = {0};
    uint8_t buf[12];

    icm42688_iic_read_regs(ICM42688_TEMP_DATA1, buf, 2);
    int16_t temp = (int16_t)((buf[0] << 8) | buf[1]);
    // icm42688_iic_read_regs(ICM42688_TEMP_DATA0, buf, 2);

    // 温度数据转换为实际值（单位: °C）

    // 加速度 6 字节一次读
    icm42688_iic_read_regs(ICM42688_ACCEL_DATA_X1, buf, 6);
    int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4] << 8) | buf[5]);

    // 陀螺 6 字节一次读
    icm42688_iic_read_regs(ICM42688_GYRO_DATA_X1, buf, 6);
    int16_t gx = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t gy = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t gz = (int16_t)((buf[4] << 8) | buf[5]);

    // icm42688_iic_read_regs(ICM42688_ACCEL_DATA_X1, buffer, 2);
    // icm42688_iic_read_regs(ICM42688_ACCEL_DATA_X0, buffer, 2);

    // icm42688_iic_read_regs(ICM42688_ACCEL_DATA_Y1, buffer, 2);
    // icm42688_iic_read_regs(ICM42688_ACCEL_DATA_Y0, buffer, 2);

    // icm42688_iic_read_regs(ICM42688_ACCEL_DATA_Z1, buffer, 2);
    // icm42688_iic_read_regs(ICM42688_ACCEL_DATA_Z0, buffer, 2);

    // icm42688_iic_read_regs(ICM42688_GYRO_DATA_X1, buffer, 2);
    // icm42688_iic_read_regs(ICM42688_GYRO_DATA_X0, buffer, 2);

    // icm42688_iic_read_regs(ICM42688_GYRO_DATA_Y1, buffer, 2);
    // icm42688_iic_read_regs(ICM42688_GYRO_DATA_Y0, buffer, 2);

    // icm42688_iic_read_regs(ICM42688_GYRO_DATA_Z1, buffer, 2);
    // icm42688_iic_read_regs(ICM42688_GYRO_DATA_Z0, buffer, 2);

    // icm42688_iic_read_regs(ICM42688_ACCEL_DATA_X1, buffer, 12);
    //     uint8_t buffer[12] = {0};
    //     icm42688_read_regs(ICM42688_TEMP_DATA1, buffer, 2);
    //     icm42688_read_regs(ICM42688_TEMP_DATA0, buffer, 2);

    // accData->x = ((uint16_t)buffer[0] << 8) | buffer[1];
    // accData->y = ((uint16_t)buffer[2] << 8) | buffer[3];
    // accData->z = ((uint16_t)buffer[4] << 8) | buffer[5];
    // gyroData->x = ((uint16_t)buffer[6] << 8) | buffer[7];
    // gyroData->y = ((uint16_t)buffer[8] << 8) | buffer[9];
    // gyroData->z = ((uint16_t)buffer[10] << 8) | buffer[11];

    // accData->x = (int16_t)(accData->x * accSensitivity);
    // accData->y = (int16_t)(accData->y * accSensitivity);
    // accData->z = (int16_t)(accData->z * accSensitivity);

    // gyroData->x = (int16_t)(gyroData->x * gyroSensitivity);
    // gyroData->y = (int16_t)(gyroData->y * gyroSensitivity);
    // gyroData->z = (int16_t)(gyroData->z * gyroSensitivity);
}

// v_old.x = v_new.y;
// v_old.y = -v_new.x;
// v_old.z = v_new.z;
bool icm42688_read_accel(icm42688RawData_t *acc)
{
    uint8_t b[6];
    if (!acc)
        return false;
    icm42688_iic_read_regs(ICM42688_ACCEL_DATA_X1, b, 6);
    acc->x = (int16_t)((b[0] << 8) | b[1]);
    acc->y = (int16_t)((b[2] << 8) | b[3]);
    acc->z = (int16_t)((b[4] << 8) | b[5]);
    return true;
}

bool icm42688_read_gyro(icm42688RawData_t *gyro)
{
    uint8_t b[6];
    if (!gyro)
        return false;
    icm42688_iic_read_regs(ICM42688_GYRO_DATA_X1, b, 6);
    gyro->x = (int16_t)((b[0] << 8) | b[1]);
    gyro->y = (int16_t)((b[2] << 8) | b[3]);
    gyro->z = (int16_t)((b[4] << 8) | b[5]);
    return true;
}

icm42688RawData_t map_vec3_by_orientation(icm42688RawData_t v_new)
{
    icm42688RawData_t v_old = {0};

    // v_old.x = v_new.y;
    // v_old.y = -v_new.x;
    // v_old.z = v_new.z;

    v_old.x = -v_new.y;
    v_old.y = v_new.x;
    v_old.z = v_new.z;

    return v_old;
}

void icm42688_raw_to_units(const icm42688RawData_t *rawA,
                           const icm42688RawData_t *rawG,
                           icm42688Float3_t *acc_g,    // 单位: g
                           icm42688Float3_t *gyro_dps) // 单位: dps
{
    if (acc_g && rawA)
    {
        acc_g->x = (rawA->x * g_accSens_mg_per_lsb) / 1000.0f;
        acc_g->y = (rawA->y * g_accSens_mg_per_lsb) / 1000.0f;
        acc_g->z = (rawA->z * g_accSens_mg_per_lsb) / 1000.0f;
    }
    if (gyro_dps && rawG)
    {
        gyro_dps->x = rawG->x * g_gyroSens_dps_per_lsb;
        gyro_dps->y = rawG->y * g_gyroSens_dps_per_lsb;
        gyro_dps->z = rawG->z * g_gyroSens_dps_per_lsb;
    }
}
