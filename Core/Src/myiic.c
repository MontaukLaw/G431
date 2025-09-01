#include "user_comm.h"

/* ------------------- 可调参数 ------------------- */
#ifndef IIC_STRETCH_TIMEOUT
#define IIC_STRETCH_TIMEOUT (2000) /* SCL 拉伸等待超时循环计数 */
#endif

#ifndef IIC_DELAY_CYCLES
#define IIC_DELAY_CYCLES (100) /* 半个 SCL 周期的空转延时，按你目标速率调 */
#endif

/* ------------------- 小延时（空转） ------------------- */
static inline void iic_delay(void)
{
    for (volatile int i = 0; i < IIC_DELAY_CYCLES; ++i)
    {
        __NOP();
    }
}

/* 等待 SCL 真正到高电平（支持从机 clock stretching） */
static inline int wait_scl_high(void)
{
    uint32_t cnt = IIC_STRETCH_TIMEOUT;
    SCL_H; /* 释放 SCL */
    while (!SCL_read)
    { /* 从机可能拉低 SCL 进行时钟拉伸 */
        if (cnt-- == 0)
            return -1;
    }
    return 0;
}

/* ------------------- I2C 基本时序 ------------------- */
void IIC_Start(void)
{
    /* 总线空闲：SDA=1, SCL=1 */
    SDA_H;
    SCL_H;
    iic_delay();

    /* 起始：SDA 由高->低，SCL 维持高 */
    SDA_L;
    iic_delay();

    /* 准备传输：SCL 拉低 */
    SCL_L;
    iic_delay();
}

void IIC_Stop(void)
{
    /* 停止：先拉低 SDA，确保 SCL 可到高，然后 SDA 由低->高 */
    SDA_L;
    iic_delay();

    if (wait_scl_high() != 0)
    {
        /* 如果 SCL 被从机一直拉低，这里超时返回；可根据需要加入恢复策略 */
    }
    iic_delay();

    SDA_H; /* SDA 在 SCL 高电平期间从低到高 -> STOP */
    iic_delay();
}

/* 发送 1 字节（MSB first），不包含 ACK 位 */
void IIC_Send_Byte(u8 txd)
{
    for (int i = 7; i >= 0; --i)
    {
        /* 准备下一个位：在 SCL 低时改变 SDA */
        SCL_L;
        iic_delay();

        if (txd & (1u << i))
            SDA_H;
        else
            SDA_L;
        iic_delay();

        /* 产生时钟高沿，让从机采样 */
        if (wait_scl_high() != 0)
        {
            /* SCL 拉伸超时处理（可选） */
        }
        iic_delay();

        /* 时钟回到低电平，完成该位传输 */
        SCL_L;
        iic_delay();
    }
}

/* 等待从机 ACK：SDA=0 表示 ACK，SDA=1 表示 NACK
 * 返回 0=ACK，1=NACK
 */
u8 IIC_Wait_Ack(void)
{
    /* 释放 SDA（高电平=输入），从机在 ACK 时会拉低 */
    SDA_H;
    iic_delay();

    if (wait_scl_high() != 0)
    {
        /* SCL 拉伸超时，按 NACK 处理 */
        SCL_L;
        return 1u;
    }
    iic_delay();

    /* 采样 SDA */
    u8 ack = (SDA_read ? 1u : 0u);

    /* 结束 ACK 时钟 */
    SCL_L;
    iic_delay();

    return ack; /* 0 = ACK, 1 = NACK */
}

/* 主机发 ACK（SDA=0） */
void IIC_Ack(void)
{
    /* 在 SCL 低电平期间拉低 SDA，随后打一拍时钟 */
    SCL_L;
    SDA_L;
    iic_delay();

    if (wait_scl_high() != 0)
    { /* ignore */
    }
    iic_delay();

    SCL_L;
    SDA_H; /* 释放 SDA，避免影响下一位 */
    iic_delay();
}

/* 主机发 NACK（SDA=1） */
void IIC_NAck(void)
{
    SCL_L;
    SDA_H; /* 释放 SDA 为高，即 NACK */
    iic_delay();

    if (wait_scl_high() != 0)
    { /* ignore */
    }
    iic_delay();

    SCL_L;
    iic_delay();
}

/* 读 1 字节；ack=1 发送 ACK，ack=0 发送 NACK */
u8 IIC_Read_Byte(unsigned char ack)
{
    u8 receive = 0;

    /* 读数据期间保持 SDA 释放（输入） */
    SDA_H;
    for (int i = 7; i >= 0; --i)
    {
        SCL_L;
        iic_delay();

        if (wait_scl_high() != 0)
        {
            /* SCL 拉伸超时，可在此决定返回值或置错误标志 */
        }
        iic_delay();

        /* 采样 SDA */
        if (SDA_read)
            receive |= (1u << i);

        SCL_L;
        iic_delay();
    }

    if (ack)
        IIC_Ack();
    else
        IIC_NAck();
    return receive;
}

/* ------------------- 常用寄存器级便捷封装 ------------------- */
void IIC_Write_One_Byte(u8 daddr, u8 reg, u8 data)
{
    IIC_Start();
    IIC_Send_Byte((daddr << 1) | 0x00); /* 写方向 */
    if (IIC_Wait_Ack())
        goto _stop;
    IIC_Send_Byte(reg);
    if (IIC_Wait_Ack())
        goto _stop;
    IIC_Send_Byte(data);
    (void)IIC_Wait_Ack();
_stop:
    IIC_Stop();
}

u8 IIC_Read_One_Byte(u8 daddr, u8 reg)
{
    u8 val = 0xFF;

    IIC_Start();
    IIC_Send_Byte((daddr << 1) | 0x00); /* 先写寄存器地址 */
    if (IIC_Wait_Ack())
        goto _stop;
    IIC_Send_Byte(reg);
    if (IIC_Wait_Ack())
        goto _stop;

    /* 重启读 */
    IIC_Start();
    IIC_Send_Byte((daddr << 1) | 0x01);
    if (IIC_Wait_Ack())
        goto _stop;

    val = IIC_Read_Byte(0); /* 读 1 字节并 NACK 结束 */
_stop:
    IIC_Stop();
    return val;
}

/* 多字节写 */
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8 *data)
{
    IIC_Start();
    IIC_Send_Byte((dev << 1) | 0x00);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 2;
    }

    for (u8 i = 0; i < length; i++)
    {
        IIC_Send_Byte(data[i]);
        if (IIC_Wait_Ack())
        {
            IIC_Stop();
            return 3;
        }
    }
    IIC_Stop();
    return 0;
}

/* 多字节读 */
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
{
    IIC_Start();
    IIC_Send_Byte((dev << 1) | 0x00);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 2;
    }

    IIC_Start();
    IIC_Send_Byte((dev << 1) | 0x01);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 3;
    }

    for (u8 i = 0; i < length; i++)
    {
        data[i] = IIC_Read_Byte(i < (length - 1) ? 1 : 0); /* 最后一个发 NACK */
    }
    IIC_Stop();
    return 0;
}

/* 单比特/多比特写（常用于配置位域） */
u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
{
    u8 b;
    if (IICreadBytes(dev, reg, 1, &b))
        return 1;
    b = (data ? (b | (1u << bitNum)) : (b & ~(1u << bitNum)));
    return IICwriteBytes(dev, reg, 1, &b);
}

u8 IICwriteBits(u8 dev, u8 reg, u8 bitStart, u8 length, u8 data)
{
    u8 b, mask;
    if (IICreadBytes(dev, reg, 1, &b))
        return 1;
    mask = ((1u << length) - 1u) << (bitStart - length + 1u);
    data = (data << (bitStart - length + 1u)) & mask;
    b = (b & ~mask) | data;
    return IICwriteBytes(dev, reg, 1, &b);
}

u8 IICreadByte(u8 dev, u8 reg, u8 *data)
{
    return IICreadBytes(dev, reg, 1, data);
}

unsigned char I2C_ReadOneByte(unsigned char dev, unsigned char reg)
{
    return IIC_Read_One_Byte(dev, reg);
}

unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
{
    IIC_Write_One_Byte(dev, reg, data);
    return 0;
}
