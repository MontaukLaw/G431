#include "user_comm.h"

/* 全局变量缓存区 */
uint8_t g_imu_init = 0;
qmi8658_state g_imu;

static float gyro[3];
static float accel[3];
static float euler_angle[3] = {0, 0, 0};
/**
 * @brief   写一个字节到QMI8568的寄存器
 * @param   reg: 寄存器地址
 * @param   data: 寄存器数据
 * @retval  写入结果
 * @arg     0: 成功
 * @arg     1: 失败
 */
uint8_t atk_qmi8658_write_byte(uint8_t reg, uint8_t data)
{
    uint8_t tx[2];
    tx[0] = reg;
    tx[1] = data;
    return HAL_I2C_Master_Transmit(&hi2c3, QMI8658_ADDR << 1, tx, 2, HAL_MAX_DELAY);
}

/**
 * @brief   在QMI8658指定寄存器地址读出一个数据
 * @param   reg:        寄存器地址
 * @retval  读到的数据
 */
uint8_t atk_qmi8658_read_byte(uint8_t reg)
{
    uint8_t data = 0;
    HAL_I2C_Master_Transmit(&hi2c3, QMI8658_ADDR << 1, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c3, QMI8658_ADDR << 1, &data, 1, HAL_MAX_DELAY);
    return data;
}

/**
 * @brief   从QMI8568读取N字节数据
 * @param   reg:        寄存器地址
 * @param   date:       数据存储buf
 * @param   len:        数据长度
 * @retval  读出结果
 * @retval  0, 操作成功
 *          其他, 操作失败
 */
int atk_qmi8568_read_nbytes(uint8_t reg, uint8_t *date, uint8_t len)
{
    HAL_I2C_Master_Transmit(&hi2c3, QMI8658_ADDR << 1, &reg, 1, HAL_MAX_DELAY);
    return HAL_I2C_Master_Receive(&hi2c3, QMI8658_ADDR << 1, date, len, HAL_MAX_DELAY);
}
void usart1_send_char(uint8_t ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
}
/**
 * @brief   检查QMI8658A的ID
 * @param   无
 * @retval  检查结果
 * @arg     0: 正确
 * @arg     1: 错误
 */
uint8_t atk_qmi8658_check_whoami(void)
{
    uint8_t qmi8658_chip_id = 0;
    uint8_t qmi8658_revision_id = 0;
    uint8_t i = 0;

    while ((qmi8658_chip_id != 0x05) && (i < 5)) /* 多次尝试检查设备标识符 */
    {
        qmi8658_chip_id = atk_qmi8658_read_byte(Register_WhoAmI);
        if (qmi8658_chip_id == 0x05)
        {
            qmi8658_revision_id = atk_qmi8658_read_byte(Register_Revision); /* 读取设备ID */
            break;
        }
        i++;
    }
    if ((qmi8658_chip_id == 0x05) && (qmi8658_revision_id == 0x7c)) /* 读取到准确的标识符和设备ID */
    {
        printf("qmi8658 chip id: %#x, device id: %#x\r\n", qmi8658_chip_id, qmi8658_revision_id);
        return 0;
    }
    else
    {
        printf("qmi8658 chip id error: %#x, device id: %#x\r\n", qmi8658_chip_id, qmi8658_revision_id);
        return 1;
    }
}

/**
 * @brief   传感器软件复位
 * @param   无
 * @retval  无
 */
void atk_qmi8658_reset(void)
{
    atk_qmi8658_write_byte(Register_Reset, 0xB0); /* 复位QMI8658 */
    delay_ms(150);
}

/**
 * @brief   使能陀螺仪、加速度计
 * @param   enableFlags ：
 *          QMI8658_DISABLE_ALL  : 都不使能
 *          QMI8658_ACC_ENABLE   : 使能加速度计
 *          QMI8658_GYR_ENABLE   : 使能陀螺仪
 *          QMI8658_ACCGYR_ENABLE: 使能陀螺仪、加速度计
 * @retval  无
 */
void atk_qmi8658_enablesensors(unsigned char enableFlags)
{
#if defined(QMI8658_SYNC_SAMPLE_MODE)
    g_imu.cfg.syncsample = 1;
    atk_qmi8658_enable_ahb_clock(0);
    atk_qmi8658_write_byte(Register_Ctrl7, enableFlags | 0x80);
#else
    atk_qmi8658_write_byte(Register_Ctrl7, enableFlags);
#endif
    g_imu.cfg.ensensors = enableFlags & 0x03;
    delay_ms(2);
}

/*****************************************************************************************************************/
/**
 * @brief   配置加速度计参数
 * @param   range       ：量程
 * @param   odr         ：odr输出速率
 * @param   lpfEnable   ：低通滤波器 ：Qmi8658Lpf_Enable 打开，Qmi8658Lpf_Disable 关闭
 * @param   stEnable    ：陀螺仪自检 ：Qmi8658St_Enable 自检，Qmi8658St_Disable 不自检
 * @retval  无
 */
void atk_qmi8658_config_acc(enum qmi8658_accrange range, enum qmi8658_accodr odr, enum qmi8658_LpfConfig lpfEnable, enum qmi8658_StConfig stEnable)
{
    unsigned char ctl_dada;

    switch (range)
    {
    case Qmi8658accrange_2g:
        g_imu.ssvt_a = (1 << 14);
        break;
    case Qmi8658accrange_4g:
        g_imu.ssvt_a = (1 << 13);
        break;
    case Qmi8658accrange_8g:
        g_imu.ssvt_a = (1 << 12);
        break;
    case Qmi8658accrange_16g:
        g_imu.ssvt_a = (1 << 11);
        break;
    default:
        range = Qmi8658accrange_8g;
        g_imu.ssvt_a = (1 << 12);
        break;
    }
    if (stEnable == Qmi8658St_Enable)
    {
        ctl_dada = (unsigned char)range | (unsigned char)odr | 0x80;
    }
    else
    {
        ctl_dada = (unsigned char)range | (unsigned char)odr;
    }
    atk_qmi8658_write_byte(Register_Ctrl2, ctl_dada);
    /* set LPF & HPF */
    atk_qmi8568_read_nbytes(Register_Ctrl5, &ctl_dada, 1);
    ctl_dada &= 0xf0;
    if (lpfEnable == Qmi8658Lpf_Enable)
    {
        ctl_dada |= A_LSP_MODE_3;
        ctl_dada |= 0x01;
    }
    else
    {
        ctl_dada &= ~0x01;
    }
    atk_qmi8658_write_byte(Register_Ctrl5, ctl_dada);
}

/**
 * @brief   读取补偿后QMI8658陀螺仪和加速度的数据（无判断）
 * @param   acc  ：存储加速度计xyz轴数据
 * @param   gyro ：存储陀螺仪xyz轴数据
 * @retval  无
 */
void atk_qmi8658_read_sensor_data(float *acc, float *gyro)
{
    unsigned char buf_reg[12];
    short raw_acc_xyz[3];
    short raw_gyro_xyz[3];
    unsigned char axis = 0;
    static int cali_count = 0;
    static float offset_acc[3] = {0.0, 0.0, 0.0};
    static float offset_gyro[3] = {0.0, 0.0, 0.0};
    static float accel_calibration_sum[3] = {0.0f, 0.0f, 0.0f};
    static float gyro_calibration_sum[3] = {0.0f, 0.0f, 0.0f};

    float acc_raw[3];
    float gyro_raw[3];

    /* 读取加速度计和陀螺仪数据 */
    atk_qmi8568_read_nbytes(Register_Ax_L, buf_reg, 12);

    raw_acc_xyz[0] = (short)((unsigned short)(buf_reg[1] << 8) | (buf_reg[0]));
    raw_acc_xyz[1] = (short)((unsigned short)(buf_reg[3] << 8) | (buf_reg[2]));
    raw_acc_xyz[2] = (short)((unsigned short)(buf_reg[5] << 8) | (buf_reg[4]));

    raw_gyro_xyz[0] = (short)((unsigned short)(buf_reg[7] << 8) | (buf_reg[6]));
    raw_gyro_xyz[1] = (short)((unsigned short)(buf_reg[9] << 8) | (buf_reg[8]));
    raw_gyro_xyz[2] = (short)((unsigned short)(buf_reg[11] << 8) | (buf_reg[10]));

    /* 加速度单位：m/s2 */
    acc_raw[0] = (float)(raw_acc_xyz[0] * ONE_G) / g_imu.ssvt_a;
    acc_raw[1] = (float)(raw_acc_xyz[1] * ONE_G) / g_imu.ssvt_a;
    acc_raw[2] = (float)(raw_acc_xyz[2] * ONE_G) / g_imu.ssvt_a;

    /* 陀螺仪单位：rad/s */
    gyro_raw[0] = (float)(raw_gyro_xyz[0] * M_PI) / (g_imu.ssvt_g * 180); /* *pi/180 */
    gyro_raw[1] = (float)(raw_gyro_xyz[1] * M_PI) / (g_imu.ssvt_g * 180);
    gyro_raw[2] = (float)(raw_gyro_xyz[2] * M_PI) / (g_imu.ssvt_g * 180);

    if (g_imu_init != 1) /* flash中已经存储校准标记？ */
    {
        if (cali_count == 0) /* 没有存储，重新计算 */
        {
            memset((void *)accel_calibration_sum, 0, sizeof(accel_calibration_sum));
            memset((void *)gyro_calibration_sum, 0, sizeof(gyro_calibration_sum));
            cali_count++;
        }
        else if (cali_count < MAX_CALI_COUNT)
        {
            for (axis = 0; axis < 3; axis++)
            {
                if (axis == 2)
                {
                    accel_calibration_sum[axis] += (acc_raw[axis] - ONE_G);
                }
                else
                {
                    accel_calibration_sum[axis] += acc_raw[axis];
                }
                gyro_calibration_sum[axis] += gyro_raw[axis];
            }
            cali_count++;
        }
        else if (cali_count == MAX_CALI_COUNT)
        {
            for (axis = 0; axis < 3; axis++)
            {
                offset_gyro[axis] = (0.0f - (gyro_calibration_sum[axis] / (MAX_CALI_COUNT - 1)));
                offset_acc[axis] = (0.0f - (accel_calibration_sum[axis] / (MAX_CALI_COUNT - 1)));
            }
            g_imu_init = 1;

            cali_count++;
        }
    }
    else /* 有存储，直接计算 */
    {
        for (axis = 0; axis < 3; axis++)
        {
            acc[axis] = acc_raw[axis] + offset_acc[axis];
            gyro[axis] = gyro_raw[axis] + offset_gyro[axis];
        }
    }
}

/**
 * @brief   配置陀螺仪参数
 * @param   range       ：量程
 * @param   odr         ：odr输出速率
 * @param   lpfEnable   ：低通滤波器 ：Qmi8658Lpf_Enable 打开，Qmi8658Lpf_Disable 关闭
 * @param   stEnable    ：陀螺仪自检 ：Qmi8658St_Enable 自检，Qmi8658St_Disable 不自检
 * @retval  无
 */
void atk_qmi8658_config_gyro(enum qmi8658_gyrrange range, enum qmi8658_gyrodr odr, enum qmi8658_LpfConfig lpfEnable, enum qmi8658_StConfig stEnable)
{
    /* Set the CTRL3 register to configure dynamic range and ODR */
    unsigned char ctl_dada;

    /* Store the scale factor for use when processing raw data */
    switch (range)
    {
    case Qmi8658gyrrange_16dps:
        g_imu.ssvt_g = 2048;
        break;
    case Qmi8658gyrrange_32dps:
        g_imu.ssvt_g = 1024;
        break;
    case Qmi8658gyrrange_64dps:
        g_imu.ssvt_g = 512;
        break;
    case Qmi8658gyrrange_128dps:
        g_imu.ssvt_g = 256;
        break;
    case Qmi8658gyrrange_256dps:
        g_imu.ssvt_g = 128;
        break;
    case Qmi8658gyrrange_512dps:
        g_imu.ssvt_g = 64;
        break;
    case Qmi8658gyrrange_1024dps:
        g_imu.ssvt_g = 32;
        break;
    case Qmi8658gyrrange_2048dps:
        g_imu.ssvt_g = 16;
        break;
    default:
        range = Qmi8658gyrrange_512dps;
        g_imu.ssvt_g = 64;
        break;
    }

    if (stEnable == Qmi8658St_Enable)
    {
        ctl_dada = (unsigned char)range | (unsigned char)odr | 0x80;
    }
    else
    {
        ctl_dada = (unsigned char)range | (unsigned char)odr;
    }
    atk_qmi8658_write_byte(Register_Ctrl3, ctl_dada);

    /* Conversion from degrees/s to rad/s if necessary */
    /* set LPF & HPF */
    atk_qmi8568_read_nbytes(Register_Ctrl5, &ctl_dada, 1);
    ctl_dada &= 0x0f;
    if (lpfEnable == Qmi8658Lpf_Enable)
    {
        ctl_dada |= G_LSP_MODE_3;
        ctl_dada |= 0x10;
    }
    else
    {
        ctl_dada &= ~0x10;
    }
    atk_qmi8658_write_byte(Register_Ctrl5, ctl_dada);
}

/**
 * @brief   配置QMI8658陀螺仪和加速度计的量程、输出频率参数等
 * @param   low_power ： 0： 正常模式 1：低功耗模式
 * @retval  无
 */
void atk_qmi8658_config_reg(unsigned char low_power)
{
    atk_qmi8658_enablesensors(QMI8658_DISABLE_ALL);
    if (low_power)
    {
        g_imu.cfg.ensensors = QMI8658_ACC_ENABLE;
        g_imu.cfg.accrange = Qmi8658accrange_8g;
        g_imu.cfg.accodr = Qmi8658accodr_LowPower_21Hz;
        g_imu.cfg.gyrrange = Qmi8658gyrrange_1024dps;
        g_imu.cfg.gyrodr = Qmi8658gyrodr_250Hz;
    }
    else
    {
        g_imu.cfg.ensensors = QMI8658_ACCGYR_ENABLE;  /* 使能陀螺仪、加速度计 */
        g_imu.cfg.accrange = Qmi8658accrange_16g;     /* ±16g */
        g_imu.cfg.accodr = Qmi8658accodr_500Hz;       /* 500Hz采样 */
        g_imu.cfg.gyrrange = Qmi8658gyrrange_2048dps; // Qmi8658gyrrange_128dps; /* ±128dps */
        g_imu.cfg.gyrodr = Qmi8658gyrodr_500Hz;       /* 500Hz采样 */
    }

    if (g_imu.cfg.ensensors & QMI8658_ACC_ENABLE)
    {
        atk_qmi8658_config_acc(g_imu.cfg.accrange, g_imu.cfg.accodr, Qmi8658Lpf_Enable, Qmi8658St_Enable); /* 设置参数并开启加速度计自检和低通滤波器 */
    }
    if (g_imu.cfg.ensensors & QMI8658_GYR_ENABLE)
    {
        atk_qmi8658_config_gyro(g_imu.cfg.gyrrange, g_imu.cfg.gyrodr, Qmi8658Lpf_Enable, Qmi8658St_Enable); /* 设置参数并开启陀螺仪自检和低通滤波器 */
    }
}

/**
 * @brief   陀螺仪校准
 * @param   无
 * @retval  检查结果
 * @arg     0: 校准成功
 * @arg     1: 校准失败
 */
uint8_t atk_qmi8658_calibration(void)
{
    uint8_t sta = 0;
    atk_qmi8658_write_byte(Register_Ctrl7, 0x00); /* 关闭陀螺仪、加速度计 */
    atk_qmi8658_write_byte(Register_Ctrl9, 0xA2);
    delay_ms(2000);
    sta = atk_qmi8658_read_byte(Register_COD_Status);
    if (sta == 0x00)
    {
        return 0;
    }
    else
        return 1;
}

/**
 * @brief       发送加速度传感器数据和陀螺仪数据
 * @param       aacx,aacy,aacz    : x,y,z三个方向上面的加速度值
 * @param       gyrox,gyroy,gyroz : x,y,z三个方向上面的陀螺仪值
 * @retval      无
 */
void qmi8658_send_data(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz)
{
    uint8_t tbuf[18];
    tbuf[0] = (aacx >> 8) & 0XFF;
    tbuf[1] = aacx & 0XFF;
    tbuf[2] = (aacy >> 8) & 0XFF;
    tbuf[3] = aacy & 0XFF;
    tbuf[4] = (aacz >> 8) & 0XFF;
    tbuf[5] = aacz & 0XFF;
    tbuf[6] = (gyrox >> 8) & 0XFF;
    tbuf[7] = gyrox & 0XFF;
    tbuf[8] = (gyroy >> 8) & 0XFF;
    tbuf[9] = gyroy & 0XFF;
    tbuf[10] = (gyroz >> 8) & 0XFF;
    tbuf[11] = gyroz & 0XFF;
    /* 无磁力计数据,所以这里直接屏蔽掉.用0替代. */
    tbuf[12] = 0;
    tbuf[13] = 0;
    tbuf[14] = 0;
    tbuf[15] = 0;
    tbuf[16] = 0;
    tbuf[17] = 0;
    usart1_niming_report(0X02, tbuf, 18); /* 自定义帧,0X02 */
}

/**
 * @brief   判断数据更新后，在读取补偿后QMI8658陀螺仪和加速度的数据(推荐使用)
 * @param   acc  : 加速度计 X,Y,Z缓存区;
 * @param   gyro : 陀螺仪 X,Y,Z缓存区;
 * @retval  无
 */
void atk_qmi8658_read_xyz(float *acc, float *gyro)
{
    unsigned char status = 0;
    unsigned char data_ready = 0;
    int retry = 0;

    while (retry++ < 3)
    {
#if defined(QMI8658_SYNC_SAMPLE_MODE)
        atk_qmi8568_read_nbytes(Register_StatusInt, &status, 1);

        if (status & 0x01)
        {
            delay_us(12); /* delay 12us <=500Hz， 12us 1000Hz, 4us 2000Hz 2us > 2000Hz */
        }
        if ((status & 0x01) || (status & 0x03))
        {
            data_ready = 1;
            break;
        }
#else
        /* 检查加速度计和陀螺仪数据是否可用 */
        atk_qmi8568_read_nbytes(Register_Status0, &status, 1);
        if (status & 0x03)
        {
            data_ready = 1;
            break;
        }
#endif
    }
    if (data_ready)
    {
        atk_qmi8658_read_sensor_data(acc, gyro);

        g_imu.imu[0] = acc[0];
        g_imu.imu[1] = acc[1];
        g_imu.imu[2] = acc[2];
        g_imu.imu[3] = gyro[0];
        g_imu.imu[4] = gyro[1];
        g_imu.imu[5] = gyro[2];
    }
    else
    {
        acc[0] = g_imu.imu[0];
        acc[1] = g_imu.imu[1];
        acc[2] = g_imu.imu[2];
        gyro[0] = g_imu.imu[3];
        gyro[1] = g_imu.imu[4];
        gyro[2] = g_imu.imu[5];
        //  printf("数据还未准备好\r\n");
        /* 调试使用 */
    }
}

/**
 * @brief       传送数据给 ANO_TC匿名科创地面站v4.exe
 * @param       fun  : 功能字. 0XA0~0XAF
 * @param       data : 数据缓存区,最多28字节!!
 * @param       len  : data区有效数据个数
 * @retval      无
 */
void usart1_niming_report(uint8_t fun, uint8_t *data, uint8_t len)
{
    uint8_t send_buf[32];
    uint8_t i;

    if (len > 28)
    {
        return; /* 最多28字节数据 */
    }

    send_buf[len + 4] = 0; /* 校验数置零 */
    send_buf[0] = 0XAA;    /* 帧头 */
    send_buf[1] = 0XAA;    /* 帧头 */
    send_buf[2] = fun;     /* 功能字 */
    send_buf[3] = len;     /* 数据长度 */

    for (i = 0; i < len; i++)
    {
        send_buf[4 + i] = data[i]; /* 复制数据 */
    }

    for (i = 0; i < len + 4; i++)
    {
        send_buf[len + 4] += send_buf[i]; /* 计算校验和 */
    }

    for (i = 0; i < len + 5; i++)
    {
        usart1_send_char(send_buf[i]); /* 发送数据到串口1 */
    }
}

/**
 * @brief   初始化QMI8658
 * @param   无
 * @retval  初始化结果
 * @arg     0: 成功
 * @arg     1: 失败
 */
uint8_t atk_qmi8658_init(void)
{
    HAL_Delay(2000); 
    atk_qmi8658_reset(); /* 复位传感器 */

    if (atk_qmi8658_check_whoami()) /* 检查设备ID是否正确 */
    {
        printf("qmi8658 whoami error\r\n");
        return 1;
    }
    if (atk_qmi8658_calibration())
    {
        printf("qmi8658 calibration error\r\n");
        return 1;
    }
    printf("calibration done\r\n");

    atk_qmi8658_write_byte(Register_Ctrl1, 0x60); /* I2C驱动 */
    atk_qmi8658_write_byte(Register_Ctrl7, 0x00); /* 关闭陀螺仪、加速度计 */

    atk_qmi8658_config_reg(0);                      /* 配置陀螺仪和加速度计的量程和数据输出速率等参数 */
    atk_qmi8658_enablesensors(g_imu.cfg.ensensors); /* 使能陀螺仪、加速度计 */

    printf("QMI8658A Ready!\r\n");

    return 0;
}

/**
 * @brief   获取传感器温度
 * @param   无
 * @retval  温度值,单位为℃
 */
float atk_qmi8658_get_temperature(void)
{
    short temp = 0;
    float temp_f = 0;
    uint8_t buf[2];

    atk_qmi8568_read_nbytes(Register_Tempearture_L, buf, 2); /* 读取温度数据 */
    temp = ((short)buf[1] << 8) | buf[0];
    temp_f = (float)temp / 256.0f;
    return temp_f;
}

/**
 * @brief       通过串口1上报结算后的姿态数据给电脑
 * @param       roll     : 横滚角.单位0.1度。 -9000 -> 9000 对应 -90.00  ->  90.00度
 * @param       pitch    : 俯仰角.单位 0.1度。-18000 -> 18000 对应 -180.00 -> 180.00 度
 * @param       yaw      : 航向角.单位为0.1度 -18000 -> 18000  对应 -180.00 -> 180.00 度
 * @param       prs      : 气压计高度,单位:cm
 * @param       fly_mode : 飞行模式
 * @param       armed    : 锁定状态
 * @retval      无
 */
void usart1_report_imu(short roll, short pitch, short yaw, int prs, uint8_t fly_mode, uint8_t armed)
{
    uint8_t tbuf[12];

    tbuf[0] = (roll >> 8) & 0XFF;
    tbuf[1] = roll & 0XFF;
    tbuf[2] = (pitch >> 8) & 0XFF;
    tbuf[3] = pitch & 0XFF;
    tbuf[4] = (yaw >> 8) & 0XFF;
    tbuf[5] = yaw & 0XFF;
    tbuf[6] = (prs >> 24) & 0XFF;
    tbuf[7] = (prs >> 16) & 0XFF;
    tbuf[8] = (prs >> 8) & 0XFF;
    tbuf[9] = prs & 0XFF;
    tbuf[10] = fly_mode;
    tbuf[11] = armed;
    usart1_niming_report(0X01, tbuf, 12); /* 状态帧,0X01 */
}

void qmi8658_pipeline_update(vec3f_t acc_g, vec3f_t gyro_dps, float dt)
{

    static float g_run_time = 0.0f;

    // // NEW: 启动静置标定（前3秒）
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

    // printf("gyro: %.4f %.4f %.4f\n", gyro_dps.x, gyro_dps.y, gyro_dps.z);
    // printf("use_acc = %d\n", use_acc);

    // 4) 融合更新
    // if (!(dt > 0.0f && dt < 0.5f))
    //     dt = 0.01f; // 容错
    madgwick_update_imu(gyro_dps.x, gyro_dps.y, gyro_dps.z,
                        axn, ayn, azn, use_acc, dt);

    // 5) 输出四元数
    q_out[0] = g_q.q0; // w
    q_out[1] = g_q.q1; // x
    q_out[2] = g_q.q2; // y
    q_out[3] = g_q.q3; // z

    // NEW: （可选）静止时偏航保持，抑制残余慢漂
    yaw_hold_when_stationary(q_out, stationary, dt);

    // printf("qout: %.4f, %.4f, %.4f, %.4f\r\n", q_out[0], q_out[1], q_out[2], q_out[3]);
}

void euler_to_quaternion(float roll, float pitch, float yaw,
                         float *q0, float *q1, float *q2, float *q3)
{
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    *q0 = cr * cp * cy + sr * sp * sy; // w
    *q1 = sr * cp * cy - cr * sp * sy; // x
    *q2 = cr * sp * cy + sr * cp * sy; // y
    *q3 = cr * cp * sy - sr * sp * cy; // z
}

void qmi8658_task_5ms(void)
{
    static uint32_t last = 0;
    uint32_t now = HAL_GetTick();

    // 改成 5ms（200Hz）
    if (now - last < 5)
        return;

    float dt = (now - last) * 0.001f;
    last = now;

    // 0.005
    // printf("dt: %.4f\r\n", dt);

    // 保护 dt
    if (dt <= 0 || dt > 0.008f)
    {
        // printf("qmi8658_task: dt error %.4f\r\n", dt);
        return;
    }

    atk_qmi8658_read_xyz(accel, gyro);

    // 坐标旋转（你的原逻辑）
    // float ax = accel[0], ay = accel[1];
    // accel[0] = -ay;
    // accel[1] = -ax;

    // float gx = gyro[0], gy = gyro[1];
    // gyro[0] = -gy;
    // gyro[1] = -gx;

    /* 获取并显示欧拉角 */
    imu_get_eulerian_angles(accel, gyro, euler_angle, dt); // IMU_DELTA_T);

    float roll = euler_angle[1] * DEG2RAD;  // X
    float pitch = euler_angle[0] * DEG2RAD; // Y
    float yaw = euler_angle[2] * DEG2RAD;   // Z

    // printf("Pitch %.2f  Roll %.2f  Yaw %.2f\r\n", pitch, roll, yaw);

    euler_to_quaternion(roll, pitch, yaw, &q_out[0], &q_out[1], &q_out[2], &q_out[3]);

    /* 上报匿名状态帧 */
    // qmi8658_send_data(accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);                                        /* 发送加速度+陀螺仪原始数据 */
    // usart1_report_imu((int)(euler_angle[1] * 100), (int)(euler_angle[0] * 100), (int)(euler_angle[2] * 100), 0, 0, 0); /* Pitch和Roll角位置调换 */

    // euler_to_quaternion(euler_angle[0] * DEG2RAD, euler_angle[1] * DEG2RAD, euler_angle[2] * DEG2RAD,
    //                     &q_out[0], &q_out[1], &q_out[2], &q_out[3]);
}

void qmi8658_task(void)
{
    static uint32_t last = 0;
    uint32_t now = HAL_GetTick();

    // 改成 5ms（200Hz）
    if (now - last < 5)
        return;

    float dt = (now - last) * 0.001f;
    last = now;

    // 0.005
    // printf("dt: %.4f\r\n", dt);

    // 保护 dt
    if (dt <= 0 || dt > 0.008f)
    {
        // printf("qmi8658_task: dt error %.4f\r\n", dt);
        return;
    }

    atk_qmi8658_read_xyz(accel, gyro);

    // 坐标旋转（你的原逻辑）
    float ax = accel[0], ay = accel[1];
    accel[0] = -ay;
    accel[1] = -ax;

    float gx = gyro[0], gy = gyro[1];
    gyro[0] = -gy;
    gyro[1] = -gx;

    if (!g_imu_init)
        return;

    vec3f_t acc_g = {accel[0] / ONE_G, accel[1] / ONE_G, accel[2] / ONE_G};
    vec3f_t gyro_rs = {gyro[0], gyro[1], gyro[2]}; // 已经是 rad/s

    qmi8658_pipeline_update(acc_g, gyro_rs, dt);
}
