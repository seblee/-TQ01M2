/******************************************************************************
* @ File name --> iic.h
* @ Brief     --> MCU模拟IIC通讯函数
* @           --> 要改变传输频率，请修改延时函数中的数值即可
******************************************************************************/
#ifndef __IIC_DEV_H_
#define __IIC_DEV_H_

/******************************************************************************
                                 外部函数头文件                        
******************************************************************************/
#include <rtthread.h>
#include "board.h"
#include "sys_conf.h"
/******************************************************************************
                                 外部引脚修改区                        
******************************************************************************/
/*  IIC_SCL时钟端口、引脚定义 */
#define IIC_SCL_PORT GPIOB
#define IIC_SCL_PIN (GPIO_Pin_8)
#define IIC_SCL_PORT_RCC RCC_AHB1Periph_GPIOB

/*  IIC_SDA时钟端口、引脚定义 */
#define IIC_SDA_PORT GPIOB
#define IIC_SDA_PIN (GPIO_Pin_9)
#define IIC_SDA_PORT_RCC RCC_AHB1Periph_GPIOB

/******************************************************************************
                             对于低速晶振的支持
                     是否使用延时函数进行调整通讯频率
******************************************************************************/

#define _USER_DELAY_CLK 1 //定义了则使用延时调整通讯频率
//0：不使用延时函数调整通讯频率，对于低速MCU时候用
//1：使用延时函数调整通讯频率，对于高速MCU时候用

/******************************************************************************
                                位带操作
******************************************************************************/
//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).M4同M3类似,只是寄存器地址变了.
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
#define MEM_ADDR(addr) *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))
//IO口地址映射
#define GPIOA_ODR_Addr (GPIOA_BASE + 20) //0x40020014
#define GPIOB_ODR_Addr (GPIOB_BASE + 20) //0x40020414
#define GPIOC_ODR_Addr (GPIOC_BASE + 20) //0x40020814
#define GPIOD_ODR_Addr (GPIOD_BASE + 20) //0x40020C14
#define GPIOE_ODR_Addr (GPIOE_BASE + 20) //0x40021014
#define GPIOF_ODR_Addr (GPIOF_BASE + 20) //0x40021414
#define GPIOG_ODR_Addr (GPIOG_BASE + 20) //0x40021814
#define GPIOH_ODR_Addr (GPIOH_BASE + 20) //0x40021C14
#define GPIOI_ODR_Addr (GPIOI_BASE + 20) //0x40022014

#define GPIOA_IDR_Addr (GPIOA_BASE + 16) //0x40020010
#define GPIOB_IDR_Addr (GPIOB_BASE + 16) //0x40020410
#define GPIOC_IDR_Addr (GPIOC_BASE + 16) //0x40020810
#define GPIOD_IDR_Addr (GPIOD_BASE + 16) //0x40020C10
#define GPIOE_IDR_Addr (GPIOE_BASE + 16) //0x40021010
#define GPIOF_IDR_Addr (GPIOF_BASE + 16) //0x40021410
#define GPIOG_IDR_Addr (GPIOG_BASE + 16) //0x40021810
#define GPIOH_IDR_Addr (GPIOH_BASE + 16) //0x40021C10
#define GPIOI_IDR_Addr (GPIOI_BASE + 16) //0x40022010

//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n) BIT_ADDR(GPIOA_ODR_Addr, n) //输出
#define PAin(n) BIT_ADDR(GPIOA_IDR_Addr, n)  //输入

#define PBout(n) BIT_ADDR(GPIOB_ODR_Addr, n) //输出
#define PBin(n) BIT_ADDR(GPIOB_IDR_Addr, n)  //输入

#define PCout(n) BIT_ADDR(GPIOC_ODR_Addr, n) //输出
#define PCin(n) BIT_ADDR(GPIOC_IDR_Addr, n)  //输入

#define PDout(n) BIT_ADDR(GPIOD_ODR_Addr, n) //输出
#define PDin(n) BIT_ADDR(GPIOD_IDR_Addr, n)  //输入

#define PEout(n) BIT_ADDR(GPIOE_ODR_Addr, n) //输出
#define PEin(n) BIT_ADDR(GPIOE_IDR_Addr, n)  //输入

#define PFout(n) BIT_ADDR(GPIOF_ODR_Addr, n) //输出
#define PFin(n) BIT_ADDR(GPIOF_IDR_Addr, n)  //输入

#define PGout(n) BIT_ADDR(GPIOG_ODR_Addr, n) //输出
#define PGin(n) BIT_ADDR(GPIOG_IDR_Addr, n)  //输入

#define PHout(n) BIT_ADDR(GPIOH_ODR_Addr, n) //输出
#define PHin(n) BIT_ADDR(GPIOH_IDR_Addr, n)  //输入

#define PIout(n) BIT_ADDR(GPIOI_ODR_Addr, n) //输出
#define PIin(n) BIT_ADDR(GPIOI_IDR_Addr, n)  //输入

#define IIC_SCL PBout(8)
#define IIC_SDA PBout(9) //IIC发送数据用
#define IN_SDA PBin(9)   //IIC读取数据用

/******************************************************************************
                               通讯频率延时函数
                    需要调整通讯频率的请修改此函数值即可
******************************************************************************/

#if _USER_DELAY_CLK == 1 //定义了则使用
#include "Delay.h"
#define IIC_Delay() Delay_us(2) //要改变请修改delay_us()中的数值即可

#endif

/******************************************************************************
                                 外部功能函数
******************************************************************************/

void IIC_GPIO_Init(void); //GPIO初始化

void IIC_Start(void); //IIC启动

void IIC_Stop(void); //IIC停止

void IIC_Ack(unsigned char a); //主机向从机发送应答信号

unsigned char IIC_Write_Byte(unsigned char dat); //向IIC总线发送一个字节数据

unsigned char IIC_Read_Byte(void); //从IIC总线上读取一个字节数据

#endif /* IIC_DEV*/
