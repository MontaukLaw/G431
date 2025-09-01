#ifndef __IOI2C_G4_H
#define __IOI2C_G4_H
#include "stm32g4xx.h"

/* ---------------- 端口/时钟：SCL=PA8, SDA=PC11 ---------------- */
#define IIC_SCL_PORT GPIOA
#define IIC_SDA_PORT GPIOC
#define IIC_SCL_PIN 8u  /* PA8 */
#define IIC_SDA_PIN 11u /* PC11 */
#define TRUE 1
#define FALSE 0
#define true TRUE
#define false FALSE

#define I2C_Direction_Trans 0
#define I2C_Direction_Rec 1

#define IIC_GPIO_EN()                        \
    do                                       \
    {                                        \
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; \
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN; \
    } while (0)

/* ---------------- 通用 GPIO 宏（BSRR/IDR） ---------------- */
#define GPIO_SET(port, pin) ((port)->BSRR = (1u << (pin)))         /* 输出高（开漏=释放） */
#define GPIO_CLR(port, pin) ((port)->BSRR = (1u << ((pin) + 16u))) /* 输出低（拉低总线） */
#define GPIO_READ(port, pin) (((port)->IDR >> (pin)) & 1u)

/* ---------------- 推荐：始终开漏输出；写1=释放，写0=拉低 ---------------- */
#define SCL_RELEASE() GPIO_SET(IIC_SCL_PORT, IIC_SCL_PIN)
#define SCL_DRIVE_LOW() GPIO_CLR(IIC_SCL_PORT, IIC_SCL_PIN)
#define SDA_RELEASE() GPIO_SET(IIC_SDA_PORT, IIC_SDA_PIN)
#define SDA_DRIVE_LOW() GPIO_CLR(IIC_SDA_PORT, IIC_SDA_PIN)
#define SCL_READ() GPIO_READ(IIC_SCL_PORT, IIC_SCL_PIN)
#define SDA_READ() GPIO_READ(IIC_SDA_PORT, IIC_SDA_PIN)

/* 兼容你旧宏名（尽量少动业务代码） */
#define SCL_H SCL_RELEASE()
#define SCL_L SCL_DRIVE_LOW()
#define SDA_H SDA_RELEASE()
#define SDA_L SDA_DRIVE_LOW()
#define SCL_read SCL_READ()
#define SDA_read SDA_READ()

/* 如果老代码需要“切输入/输出”的语义，也给出可用版本 */
#define SDA_TO_INPUT()                                                 \
    do                                                                 \
    {                                                                  \
        IIC_SDA_PORT->MODER &= ~(3u << (IIC_SDA_PIN * 2u)); /* 输入 */ \
        IIC_SDA_PORT->PUPDR &= ~(3u << (IIC_SDA_PIN * 2u));            \
        IIC_SDA_PORT->PUPDR |= (1u << (IIC_SDA_PIN * 2u)); /* 上拉 */  \
    } while (0)

#define SDA_TO_OUTPUT_OD()                                                        \
    do                                                                            \
    {                                                                             \
        IIC_SDA_PORT->MODER &= ~(3u << (IIC_SDA_PIN * 2u));                       \
        IIC_SDA_PORT->MODER |= (1u << (IIC_SDA_PIN * 2u));   /* 输出 */           \
        IIC_SDA_PORT->OTYPER |= (1u << IIC_SDA_PIN);         /* 开漏 */           \
        IIC_SDA_PORT->OSPEEDR |= (3u << (IIC_SDA_PIN * 2u)); /* Very High */      \
        IIC_SDA_PORT->PUPDR &= ~(3u << (IIC_SDA_PIN * 2u));                       \
        IIC_SDA_PORT->PUPDR |= (1u << (IIC_SDA_PIN * 2u)); /* 上拉（若无外拉） */ \
    } while (0)

#define SDA_IN() SDA_TO_INPUT()
#define SDA_OUT() SDA_TO_OUTPUT_OD()

/* ---------------- 引脚初始化 ---------------- */
static inline void IIC_PinInit(void)
{
    IIC_GPIO_EN();

    /* SCL: PA8 → 输出、开漏、Very High、上拉 */
    IIC_SCL_PORT->MODER &= ~(3u << (IIC_SCL_PIN * 2u));
    IIC_SCL_PORT->MODER |= (1u << (IIC_SCL_PIN * 2u));   // 输出
    IIC_SCL_PORT->OTYPER |= (1u << IIC_SCL_PIN);         // 开漏
    IIC_SCL_PORT->OSPEEDR |= (3u << (IIC_SCL_PIN * 2u)); // Very High
    IIC_SCL_PORT->PUPDR &= ~(3u << (IIC_SCL_PIN * 2u));
    // IIC_SCL_PORT->PUPDR |= (1u << (IIC_SCL_PIN * 2u)); // 上拉（若有外拉可不使能）

    /* SDA: PC11 → 输出、开漏、Very High、上拉 */
    IIC_SDA_PORT->MODER &= ~(3u << (IIC_SDA_PIN * 2u));
    IIC_SDA_PORT->MODER |= (1u << (IIC_SDA_PIN * 2u));
    IIC_SDA_PORT->OTYPER |= (1u << IIC_SDA_PIN);
    IIC_SDA_PORT->OSPEEDR |= (3u << (IIC_SDA_PIN * 2u));
    IIC_SDA_PORT->PUPDR &= ~(3u << (IIC_SDA_PIN * 2u));
    // IIC_SDA_PORT->PUPDR |= (1u << (IIC_SDA_PIN * 2u));

    /* 默认释放两线为高 */
    SCL_RELEASE();
    SDA_RELEASE();
}

/* 兼容旧名 */
#define IIC_Init() IIC_PinInit()

/* 关/开中断（替代 CLI/SEI） */
#define CLI() __disable_irq()
#define SEI() __enable_irq()

#ifndef u8
#define u8 uint8_t
#endif

/* 其余你的函数原型保持不变 */
void IIC_Start(void);
void IIC_Stop(void);
void IIC_Send_Byte(u8 txd);
u8 IIC_Read_Byte(unsigned char ack);
u8 IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);

void IIC_Write_One_Byte(u8 daddr, u8 addr, u8 data);
u8 IIC_Read_One_Byte(u8 daddr, u8 addr);
unsigned char I2C_Readkey(unsigned char I2C_Addr);

unsigned char I2C_ReadOneByte(unsigned char I2C_Addr, unsigned char addr);
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8 *data);
u8 IICwriteBits(u8 dev, u8 reg, u8 bitStart, u8 length, u8 data);
u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data);
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data);
u8 IICreadByte(u8 dev, u8 reg, u8 *data);

#endif /* __IOI2C_G4_H */
