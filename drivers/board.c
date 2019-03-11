/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      first implementation
 */

#include <rthw.h>
#include <rtthread.h>

#include "stm32f4xx.h"
#include "board.h"
#include "usart.h"
#include "adc_bsp.h"
#include "i2c_bsp.h"
#include "gpio.h"
#include "dio_bsp.h"
#include "pwm_bsp.h"
#include "rtc_bsp.h"
#include "TH_SENSOR_BSP.h"

/**
 * @addtogroup STM32
 */

/*@{*/

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
#ifdef VECT_TAB_RAM
    /* Set the Vector Table base location at 0x20000000 */
    NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else /* VECT_TAB_FLASH  */
    /* Set the Vector Table base location at 0x08004000 */
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);
#endif

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}

/*******************************************************************************
 * Function Name  : SysTick_Configuration
 * Description    : Configures the SysTick for OS tick.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void SysTick_Configuration(void)
{
    RCC_ClocksTypeDef rcc_clocks;
    rt_uint32_t cnts;

    RCC_GetClocksFreq(&rcc_clocks);

    cnts = (rt_uint32_t)rcc_clocks.HCLK_Frequency / RT_TICK_PER_SECOND;
    cnts = cnts / 8;

    SysTick_Config(cnts);
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
}

/**
 * This is the timer interrupt service routine.
 *
 */
void SysTick_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}

/**
 * This function will initial STM32 board.
 */
void rt_hw_board_init()
{
    /* NVIC Configuration */
    NVIC_Configuration();

    /* Configure the SysTick */
    SysTick_Configuration();

#ifdef RT_USING_HEAP
    rt_system_heap_init((void *)STM32_SRAM_BEGIN, (void *)STM32_SRAM_END);
#endif

    rt_components_board_init();

#ifdef RT_USING_CONSOLE
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif
}

int hw_drivers_init(void)
{

    drv_adc_dma_init(); //模拟输入初始化

    drv_dio_init(); //数字输入输出初始化

    drv_pwm_init(); //模拟输出初始化

    drv_i2c_init(); //IIC初始化

    AM_Init(); //AM Sensor init

    drv_rtc_init(); //RTC初始化

    Drv_CNT_Pluse_Init(); //脉冲计数

    //	xPort_Usart_Init(UART_HEAT);//加热器
    //	drv_led_init();
    return 0;
}
INIT_DEVICE_EXPORT(hw_drivers_init);
/*@}*/
