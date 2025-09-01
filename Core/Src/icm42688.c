#include "user_comm.h"

float accSensitivity = 0.244f; // 加速度的最小分辨率 mg/LSB
float gyroSensitivity = 32.8f; // 陀螺仪的最小分辨率

// /*ICM42688使用的ms级延时函数，须由用户提供。*/
// // #define ICM42688DelayMs(_nms) delay_ms(_nms)

// /* I²C: 读单寄存器 */
// static uint8_t icm42688_read_reg(uint8_t reg)
// {
//     uint8_t regval = 0;
//     /* 读 1 字节 */
//     (void)IICreadBytes(ICM42688_ADDRESS, reg, 1, &regval);
//     return regval;
// }

// /* I²C: 连续读多个寄存器 */
// static void icm42688_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
// {
//     if (len == 0)
//         return;
//     (void)IICreadBytes(ICM42688_ADDRESS, reg, (uint8_t)len, buf);
// }

// /* I²C: 写单寄存器 */
// static uint8_t icm42688_write_reg(uint8_t reg, uint8_t value)
// {
//     return IICwriteBytes(ICM42688_ADDRESS, reg, 1, &value);
// }

float bsp_Icm42688GetAres(uint8_t Ascale)
{
    switch (Ascale)
    {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (11), 4 Gs (10), 8 Gs (01), and 16 Gs  (00).
    case AFS_2G:
        accSensitivity = 2000 / 32768.0f;
        break;
    case AFS_4G:
        accSensitivity = 4000 / 32768.0f;
        break;
    case AFS_8G:
        accSensitivity = 8000 / 32768.0f;
        break;
    case AFS_16G:
        accSensitivity = 16000 / 32768.0f;
        break;
    }

    return accSensitivity;
}

float bsp_Icm42688GetGres(uint8_t Gscale)
{
    switch (Gscale)
    {
    case GFS_15_125DPS:
        gyroSensitivity = 15.125f / 32768.0f;
        break;
    case GFS_31_25DPS:
        gyroSensitivity = 31.25f / 32768.0f;
        break;
    case GFS_62_5DPS:
        gyroSensitivity = 62.5f / 32768.0f;
        break;
    case GFS_125DPS:
        gyroSensitivity = 125.0f / 32768.0f;
        break;
    case GFS_250DPS:
        gyroSensitivity = 250.0f / 32768.0f;
        break;
    case GFS_500DPS:
        gyroSensitivity = 500.0f / 32768.0f;
        break;
    case GFS_1000DPS:
        gyroSensitivity = 1000.0f / 32768.0f;
        break;
    case GFS_2000DPS:
        gyroSensitivity = 2000.0f / 32768.0f;
        break;
    }
    return gyroSensitivity;
}

// /*******************************************************************************
//  * 名    称： bsp_Icm42688RegCfg
//  * 功    能： Icm42688 寄存器配置
//  * 入口参数： 无
//  * 出口参数： 无
//  * 作　　者： Baxiange
//  * 创建日期： 2024-07-25
//  * 修    改：
//  * 修改日期：
//  * 备    注：
//  *******************************************************************************/
// int8_t bsp_Icm42688RegCfg(void)
// {
//     uint8_t reg_val = 0;
//     /* 读取 who am i 寄存器 */
//     reg_val = icm42688_read_reg(ICM42688_WHO_AM_I);
//     // printf("reg_val:%d\n", reg_val);
//     icm42688_write_reg(ICM42688_REG_BANK_SEL, 0);    // 设置bank 0区域寄存器
//     icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x01); // 软复位传感器
//     ICM42688DelayMs(100);

//     if (reg_val == ICM42688_ID)
//     {
//         //        icm42688_write_reg(ICM42688_REG_BANK_SEL, 1); //设置bank 1区域寄存器
//         //        icm42688_write_reg(ICM42688_INTF_CONFIG4, 0x02); //设置为4线SPI通信

//         //        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0); //设置bank 0区域寄存器
//         //        icm42688_write_reg(ICM42688_FIFO_CONFIG, 0x00); //Stream-to-FIFO Mode(page63)

//         //        reg_val = icm42688_read_reg(ICM42688_INT_SOURCE0);
//         //        icm42688_write_reg(ICM42688_INT_SOURCE0, 0x00);
//         //        icm42688_write_reg(ICM42688_FIFO_CONFIG2, 0x00); // watermark
//         //        icm42688_write_reg(ICM42688_FIFO_CONFIG3, 0x02); // watermark
//         ////        icm42688_write_reg(ICM42688_INT_SOURCE0, reg_val);
//         ////        icm42688_write_reg(ICM42688_FIFO_CONFIG1, 0x63); // Enable the accel and gyro to the FIFO

//         //        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
//         //        icm42688_write_reg(ICM42688_INT_CONFIG, 0x36);

//         //        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
//         //        reg_val = icm42688_read_reg(ICM42688_INT_SOURCE0);
//         //        reg_val |= (1 << 2); //FIFO_THS_INT1_ENABLE
//         //        icm42688_write_reg(ICM42688_INT_SOURCE0, reg_val);

//         bsp_Icm42688GetAres(AFS_8G);
//         icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
//         reg_val = icm42688_read_reg(ICM42688_ACCEL_CONFIG0); // page74
//         reg_val |= (AFS_2G << 5);                            // 量程 ±2g
//         reg_val |= (AODR_100Hz);                             // 输出速率 100HZ
//         icm42688_write_reg(ICM42688_ACCEL_CONFIG0, reg_val);

//         bsp_Icm42688GetGres(GFS_1000DPS);
//         icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
//         reg_val = icm42688_read_reg(ICM42688_GYRO_CONFIG0); // page73
//         reg_val |= (GFS_1000DPS << 5);                      // 量程 ±1000dps
//         reg_val |= (GODR_100Hz);                            // 输出速率 100HZ
//         icm42688_write_reg(ICM42688_GYRO_CONFIG0, reg_val);

//         icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
//         reg_val = icm42688_read_reg(ICM42688_PWR_MGMT0); // 读取PWR—MGMT0当前寄存器的值(page72)
//         reg_val &= ~(1 << 5);                            // 使能温度测量
//         reg_val |= ((3) << 2);                           // 设置GYRO_MODE  0:关闭 1:待机 2:预留 3:低噪声
//         reg_val |= (3);                                  // 设置ACCEL_MODE 0:关闭 1:关闭 2:低功耗 3:低噪声
//         icm42688_write_reg(ICM42688_PWR_MGMT0, reg_val);
//         ICM42688DelayMs(1); // 操作完PWR—MGMT0寄存器后 200us内不能有任何读写寄存器的操作

//         return 0;
//     }
//     return -1;
// }
// /*******************************************************************************
//  * 名    称： bsp_Icm42688Init
//  * 功    能： Icm42688 传感器初始化
//  * 入口参数： 无
//  * 出口参数： 0: 初始化成功  其他值: 初始化失败
//  * 作　　者： Baxiange
//  * 创建日期： 2024-07-25
//  * 修    改：
//  * 修改日期：
//  * 备    注：
//  *******************************************************************************/
// int8_t bsp_Icm42688Init(void)
// {
//     return (bsp_Icm42688RegCfg());
// }

// /*******************************************************************************
//  * 名    称： bsp_IcmGetTemperature
//  * 功    能： 读取Icm42688 内部传感器温度
//  * 入口参数： 无
//  * 出口参数： 无
//  * 作　　者： Baxiange
//  * 创建日期： 2024-07-25
//  * 修    改：
//  * 修改日期：
//  * 备    注： datasheet page62
//  *******************************************************************************/
// int8_t bsp_IcmGetTemperature(int16_t *pTemp)
// {
//     uint8_t buffer[2] = {0};

//     icm42688_read_regs(ICM42688_TEMP_DATA1, buffer, 2);

//     *pTemp = (int16_t)(((int16_t)((buffer[0] << 8) | buffer[1])) / 132.48 + 25);
//     return 0;
// }

// /*******************************************************************************
//  * 名    称： bsp_IcmGetAccelerometer
//  * 功    能： 读取Icm42688 加速度的值
//  * 入口参数： 三轴加速度的值
//  * 出口参数： 无
//  * 作　　者： Baxiange
//  * 创建日期： 2024-07-25
//  * 修    改：
//  * 修改日期：
//  * 备    注： datasheet page62
//  *******************************************************************************/
// int8_t bsp_IcmGetAccelerometer(icm42688RawData_t *accData)
// {
//     uint8_t buffer[6] = {0};

//     icm42688_read_regs(ICM42688_ACCEL_DATA_X1, buffer, 6);

//     accData->x = ((uint16_t)buffer[0] << 8) | buffer[1];
//     accData->y = ((uint16_t)buffer[2] << 8) | buffer[3];
//     accData->z = ((uint16_t)buffer[4] << 8) | buffer[5];

//     accData->x = (int16_t)(accData->x * accSensitivity);
//     accData->y = (int16_t)(accData->y * accSensitivity);
//     accData->z = (int16_t)(accData->z * accSensitivity);

//     return 0;
// }

// /*******************************************************************************
//  * 名    称： bsp_IcmGetGyroscope
//  * 功    能： 读取Icm42688 陀螺仪的值
//  * 入口参数： 三轴陀螺仪的值
//  * 出口参数： 无
//  * 作　　者： Baxiange
//  * 创建日期： 2024-07-25
//  * 修    改：
//  * 修改日期：
//  * 备    注： datasheet page63
//  *******************************************************************************/
// int8_t bsp_IcmGetGyroscope(icm42688RawData_t *GyroData)
// {
//     uint8_t buffer[6] = {0};

//     icm42688_read_regs(ICM42688_GYRO_DATA_X1, buffer, 6);

//     GyroData->x = ((uint16_t)buffer[0] << 8) | buffer[1];
//     GyroData->y = ((uint16_t)buffer[2] << 8) | buffer[3];
//     GyroData->z = ((uint16_t)buffer[4] << 8) | buffer[5];

//     GyroData->x = (int16_t)(GyroData->x * gyroSensitivity);
//     GyroData->y = (int16_t)(GyroData->y * gyroSensitivity);
//     GyroData->z = (int16_t)(GyroData->z * gyroSensitivity);
//     return 0;
// }

// /*******************************************************************************
//  * 名    称： bsp_IcmGetRawData
//  * 功    能： 读取Icm42688加速度陀螺仪数据
//  * 入口参数： 六轴
//  * 出口参数： 无
//  * 作　　者： Baxiange
//  * 创建日期： 2024-07-25
//  * 修    改：
//  * 修改日期：
//  * 备    注： datasheet page62,63
//  *******************************************************************************/
// int8_t bsp_IcmGetRawData(icm42688RawData_t *accData, icm42688RawData_t *GyroData)
// {
//     uint8_t buffer[12] = {0};

//     icm42688_read_regs(ICM42688_ACCEL_DATA_X1, buffer, 12);

//     accData->x = ((uint16_t)buffer[0] << 8) | buffer[1];
//     accData->y = ((uint16_t)buffer[2] << 8) | buffer[3];
//     accData->z = ((uint16_t)buffer[4] << 8) | buffer[5];
//     GyroData->x = ((uint16_t)buffer[6] << 8) | buffer[7];
//     GyroData->y = ((uint16_t)buffer[8] << 8) | buffer[9];
//     GyroData->z = ((uint16_t)buffer[10] << 8) | buffer[11];

//     accData->x = (int16_t)(accData->x * accSensitivity);
//     accData->y = (int16_t)(accData->y * accSensitivity);
//     accData->z = (int16_t)(accData->z * accSensitivity);

//     GyroData->x = (int16_t)(GyroData->x * gyroSensitivity);
//     GyroData->y = (int16_t)(GyroData->y * gyroSensitivity);
//     GyroData->z = (int16_t)(GyroData->z * gyroSensitivity);

//     return 0;
// }

// int8_t get_raw_data(icm42688RawData_t *accData, icm42688RawData_t *GyroData)
// {
//     uint8_t buffer[12] = {0};
//     icm42688_read_regs(ICM42688_TEMP_DATA1, buffer, 2);

//     icm42688_read_regs(ICM42688_TEMP_DATA0, buffer, 2);

//     icm42688_read_regs(ICM42688_ACCEL_DATA_X1, buffer, 2);
//     icm42688_read_regs(ICM42688_ACCEL_DATA_X0, buffer, 2);

//     icm42688_read_regs(ICM42688_ACCEL_DATA_Y1, buffer, 2);
//     icm42688_read_regs(ICM42688_ACCEL_DATA_Y0, buffer, 2);

//     icm42688_read_regs(ICM42688_ACCEL_DATA_Z1, buffer, 2);
//     icm42688_read_regs(ICM42688_ACCEL_DATA_Z0, buffer, 2);

//     icm42688_read_regs(ICM42688_GYRO_DATA_X1, buffer, 2);
//     icm42688_read_regs(ICM42688_GYRO_DATA_X0, buffer, 2);

//     icm42688_read_regs(ICM42688_GYRO_DATA_Y1, buffer, 2);
//     icm42688_read_regs(ICM42688_GYRO_DATA_Y0, buffer, 2);

//     icm42688_read_regs(ICM42688_GYRO_DATA_Z1, buffer, 2);
//     icm42688_read_regs(ICM42688_GYRO_DATA_Z0, buffer, 2);
// }
