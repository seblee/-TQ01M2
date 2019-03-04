/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2006-08-31     Bernard      first implementation
 * 2011-06-05     Bernard      modify for STM32F107 version
 */

#include <rthw.h>
#include <rtthread.h>
#include <board.h>

#include "thread_entries.h"
#include "global_var.h"
#include "event_record.h"
/**
 * @addtogroup STM32
 */

enum
{
    INIT_THREAD_THREAD_PRIO = 7,
    MODBUS_SLAVE_THREAD_PRIO,
    MONITOR_SLAVE_THREAD_PRIO,
    MODBUS_MASTER_THREAD_PRIO,
    NET_THREAD_PRIO,
    MODULE_CTR_THREAD_PRIO,
    // TCOM_THREAD_PRIO,
    // TEAM_THREAD_PRIO,
    MBM_FSM_THREAD_PRIO,
    DI_THREAD_PRIO,
    DAQ_THREAD_PRIO,
    CORE_THREAD_PRIO,
    SURV_THREAD_PRIO,
    CPAD_THREAD_PRIO,
    BKG_THREAD_PRIO,
    TESTCASE_THREAD_PRIO,
    USR_MAX_PRIO
};

void rt_init_thread_entry(void *parameter);

/*@{*/

int main(void)
{
    /* user app entry */
    /**
     * init thread 
     */
    rt_thread_t init_thread;
    init_thread = rt_thread_create("init", rt_init_thread_entry, RT_NULL,
                                   2560, INIT_THREAD_THREAD_PRIO, 20);
    RT_ASSERT(init_thread != RT_NULL);
    if (init_thread != RT_NULL)
        rt_thread_startup(init_thread);

    rt_thread_t daq_thread;
    daq_thread = rt_thread_create("daq", daq_thread_entry, RT_NULL,
                                  512, DAQ_THREAD_PRIO, 5);
    RT_ASSERT(daq_thread != RT_NULL);
    if (daq_thread != RT_NULL)
        rt_thread_startup(daq_thread);

    // rt_thread_t di_thread;
    // di_thread = rt_thread_create("di", di_thread_entry, RT_NULL,
    //                              256, DI_THREAD_PRIO, 5);
    // RT_ASSERT(di_thread != RT_NULL);
    // if (di_thread != RT_NULL)
    //     rt_thread_startup(di_thread);

    return 0;
}

/*@}*/

void rt_init_thread_entry(void *parameter)
{

    /* Filesystem Initialization */
#if defined(RT_USING_DFS) && defined(RT_USING_DFS_ELMFAT)
    /* mount sd card fat partition 1 as root directory */
//    if (dfs_mount("sd0", "/", "elm", 0, 0) == 0)
//    {
//        rt_kprintf("File System initialized!\n");
//    }
//    else
//        rt_kprintf("File System initialzation failed!\n");
#endif /* RT_USING_DFS */

    sys_global_var_init();
    sys_local_var_init();
    // drv_can_init();
    // init_work_mode();
    init_evnet_log();
    init_alarm_log();

    //    if ((*(__IO uint32_t *)FLASH_APP_FLAG_ADDR) != FLASH_APP_FLAG_WORD)
    //    {
    //        set_boot_flag();
    //    }
}
