/*
 * File      : rtc.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version.
 * 2011-11-26     aozima       implementation time.
 */

#include <rtthread.h>
#include <stm32f4xx.h>
#include "rtc_bsp.h"
#include "pcf8563.h"

static struct rt_device rtc;
static rt_err_t rt_rtc_open(rt_device_t dev, rt_uint16_t oflag)
{
    if (dev->rx_indicate != RT_NULL)
    {
        /* Open Interrupt */
    }
 
    return RT_EOK;
}

static rt_size_t rt_rtc_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    return 0;
}

rt_err_t rt_rtc_control(rt_device_t dev, int cmd, void *args) //modified by GP
{
    rt_time_t *time;
    RT_ASSERT(dev != RT_NULL);

    switch (cmd)
    {
    case RT_DEVICE_CTRL_RTC_GET_TIME:
        time = (rt_time_t *)args;
        /* read device */
        *time = pcf8563_GetCounter();
        break;

    case RT_DEVICE_CTRL_RTC_SET_TIME:
    {
        time = (rt_time_t *)args;

        /* Change the current time */
        pcf8563_SetCounter(*time);
    }
    break;
    }

    return RT_EOK;
}

/*******************************************************************************
* Function Name  : RTC_Configuration
* Description    : Configures the RTC.
* Input          : None
* Output         : None
* Return         : 0 reday,-1 error.
*******************************************************************************/
int RTC_Configuration(void)
{
//    u32 count = 0x200000;

//    /* Enable PWR and BKP clocks */
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

//    /* Allow access to BKP Domain */
//    PWR_BackupAccessCmd(ENABLE);

//    /* Reset Backup Domain */
//    BKP_DeInit();

//    /* Enable LSE */
//    RCC_LSEConfig(RCC_LSE_ON);
//    /* Wait till LSE is ready */
//    while ((RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) && (--count))
//        ;
//    if (count == 0)
//    {
//        return -1;
//    }

//    /* Select LSE as RTC Clock Source */
//    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

//    /* Enable RTC Clock */
//    RCC_RTCCLKCmd(ENABLE);

//    /* Wait for RTC registers synchronization */
//    RTC_WaitForSynchro();

//    /* Wait until last write operation on RTC registers has finished */
//    RTC_WaitForLastTask();

//    /* Set RTC prescaler: set RTC period to 1sec */
//    RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */

//    /* Wait until last write operation on RTC registers has finished */
//    RTC_WaitForLastTask();

    return 0;
}
#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops rtc_ops = 
{
    RT_NULL, 
    rt_rtc_open,
    RT_NULL,
    rt_rtc_read,
    RT_NULL,
    rt_rtc_control
};
#endif

void drv_rtc_init(void)
{
    rtc.type = RT_Device_Class_RTC;
    if (PCF8563_Config() == RT_EOK)
    {
        rt_kprintf("rtc OK  \r\n");
    }
    else
    {
        rt_kprintf("rtc configure fail...\r\n");
        return;
    }
    
#ifdef RT_USING_DEVICE_OPS
    rtc.ops          = &rtc_ops;
#else
    /* register rtc device */
    rtc.init = RT_NULL;
    rtc.open = rt_rtc_open;
    rtc.close = RT_NULL;
    rtc.read = rt_rtc_read;
    rtc.write = RT_NULL;
    rtc.control = rt_rtc_control;
#endif
    
  
   

    /* no private */
    rtc.user_data = RT_NULL;

    rt_device_register(&rtc, "rtc", RT_DEVICE_FLAG_RDWR);

    return;
}

#include <time.h>

void get_local_time(time_t *t)
{
    time(t);
}
 

void Rtc_sts_update(sys_reg_st *gds_sys_ptr)
{
    time_t now;
    struct tm *ti;

    time(&now);
    now += 28800;
    ti = localtime(&now);

    gds_sys_ptr->status.ComSta.Sys_Time.Year = ti->tm_year + 1900;
    gds_sys_ptr->status.ComSta.Sys_Time.Mon = ti->tm_mon + 1;
    gds_sys_ptr->status.ComSta.Sys_Time.Day = ti->tm_mday;
    gds_sys_ptr->status.ComSta.Sys_Time.Hour = ti->tm_hour;
    gds_sys_ptr->status.ComSta.Sys_Time.Min = ti->tm_min;
    gds_sys_ptr->status.ComSta.Sys_Time.Sec = ti->tm_sec;
    gds_sys_ptr->status.ComSta.Sys_Time.Weekday = ti->tm_wday;

    gds_sys_ptr->status.ComSta.Sys_Time.u32Systime = now;
    // rt_kprintf("%04d-%02d-%02d %02d:%02d:%02d %d now= %d\n", ti->tm_year + 1900, ti->tm_mon + 1, ti->tm_mday, ti->tm_hour, ti->tm_min, ti->tm_sec, ti->tm_wday, now);

    return;
}
