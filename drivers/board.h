/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-09-22     Bernard      add board.h to this bsp
 */

// <<< Use Configuration Wizard in Context Menu >>>
#ifndef __BOARD_H__
#define __BOARD_H__

#include <stm32f4xx.h>


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
#define PAout(n)    BIT_ADDR(GPIOA_ODR_Addr, n) //输出
#define PAin(n)     BIT_ADDR(GPIOA_IDR_Addr, n)  //输入

#define PBout(n)    BIT_ADDR(GPIOB_ODR_Addr, n) //输出
#define PBin(n)     BIT_ADDR(GPIOB_IDR_Addr, n)  //输入

#define PCout(n)    BIT_ADDR(GPIOC_ODR_Addr, n) //输出
#define PCin(n)     BIT_ADDR(GPIOC_IDR_Addr, n)  //输入

#define PDout(n)    BIT_ADDR(GPIOD_ODR_Addr, n) //输出
#define PDin(n)     BIT_ADDR(GPIOD_IDR_Addr, n)  //输入

#define PEout(n)    BIT_ADDR(GPIOE_ODR_Addr, n) //输出
#define PEin(n)     BIT_ADDR(GPIOE_IDR_Addr, n)  //输入

#define PFout(n)    BIT_ADDR(GPIOF_ODR_Addr, n) //输出
#define PFin(n)     BIT_ADDR(GPIOF_IDR_Addr, n)  //输入

#define PGout(n)    BIT_ADDR(GPIOG_ODR_Addr, n) //输出
#define PGin(n)     BIT_ADDR(GPIOG_IDR_Addr, n)  //输入

#define PHout(n)    BIT_ADDR(GPIOH_ODR_Addr, n) //输出
#define PHin(n)     BIT_ADDR(GPIOH_IDR_Addr, n)  //输入

#define PIout(n)    BIT_ADDR(GPIOI_ODR_Addr, n) //输出
#define PIin(n)     BIT_ADDR(GPIOI_IDR_Addr, n)  //输入

/* board configuration */
// <o> SDCard Driver <1=>SDIO sdcard <0=>SPI MMC card
// 	<i>Default: 1
#define STM32_USE_SDIO			0

/* whether use board external SRAM memory */
// <e>Use external SRAM memory on the board
// 	<i>Enable External SRAM memory
#define STM32_EXT_SRAM          0
//	<o>Begin Address of External SRAM
//		<i>Default: 0x68000000
#define STM32_EXT_SRAM_BEGIN    0x68000000 /* the begining address of external SRAM */
//	<o>End Address of External SRAM
//		<i>Default: 0x68080000
#define STM32_EXT_SRAM_END      0x68080000 /* the end address of external SRAM */
// </e>

// <o> Internal SRAM memory size[Kbytes] <8-64>
//	<i>Default: 64
#ifdef __ICCARM__
// Use *.icf ram symbal, to avoid hardcode.
extern char __ICFEDIT_region_RAM_end__;
#define STM32_SRAM_END          &__ICFEDIT_region_RAM_end__
#else
#define STM32_SRAM_SIZE         128
#define STM32_SRAM_END          (0x20000000 + STM32_SRAM_SIZE * 1024)
#endif

#if defined(__CC_ARM) || defined(__CLANG_ARM)
extern int Image$$RW_IRAM1$$ZI$$Limit;
#define STM32_SRAM_BEGIN    (&Image$$RW_IRAM1$$ZI$$Limit)
#elif __ICCARM__
#pragma section="HEAP"
#define STM32_SRAM_BEGIN    (__segment_end("HEAP"))
#else
extern int __bss_end;
#define STM32_SRAM_BEGIN    (&__bss_end)
#endif

void rt_hw_board_init(void);

#endif

// <<< Use Configuration Wizard in Context Menu >>>
