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

    rt_thread_t modbus_master_thread;
    modbus_master_thread = rt_thread_create("mb_master", modbus_master_thread_entry, RT_NULL,
                                            512, MODBUS_MASTER_THREAD_PRIO, 5);
    RT_ASSERT(modbus_master_thread != RT_NULL);
    if (modbus_master_thread != RT_NULL)
        rt_thread_startup(modbus_master_thread);

    rt_thread_t modbus_slave_thread;
    modbus_slave_thread = rt_thread_create("mb_slave", modbus_slave_thread_entry, RT_NULL,
                                           512, MODBUS_SLAVE_THREAD_PRIO, 20);
    RT_ASSERT(modbus_slave_thread != RT_NULL);
    if (modbus_slave_thread != RT_NULL)
        rt_thread_startup(modbus_slave_thread);

    rt_thread_t CPAD_slave_thread;
    CPAD_slave_thread = rt_thread_create("CPAD_slave", cpad_modbus_slave_thread_entry, RT_NULL,
                                         2048, CPAD_THREAD_PRIO, 5);
    RT_ASSERT(CPAD_slave_thread != RT_NULL);
    if (CPAD_slave_thread != RT_NULL)
        rt_thread_startup(CPAD_slave_thread);

    rt_thread_t mbm_fsm_thread;
    mbm_fsm_thread = rt_thread_create("mbm_fsm", mbm_fsm_thread_entry, RT_NULL,
                                      512, MBM_FSM_THREAD_PRIO, 5);
    RT_ASSERT(mbm_fsm_thread != RT_NULL);
    if (mbm_fsm_thread != RT_NULL)
        rt_thread_startup(mbm_fsm_thread);

    rt_thread_t di_thread;
    di_thread = rt_thread_create("di", di_thread_entry, RT_NULL,
                                 256, DI_THREAD_PRIO, 5);
    RT_ASSERT(di_thread != RT_NULL);
    if (di_thread != RT_NULL)
        rt_thread_startup(di_thread);

    rt_thread_t daq_thread;
    daq_thread = rt_thread_create("daq", daq_thread_entry, RT_NULL,
                                  512, DAQ_THREAD_PRIO, 5);
    RT_ASSERT(daq_thread != RT_NULL);
    if (daq_thread != RT_NULL)
        rt_thread_startup(daq_thread);

    rt_thread_t core_thread;
    core_thread = rt_thread_create("core", core_thread_entry, RT_NULL,
                                   512, CORE_THREAD_PRIO, 5);
    RT_ASSERT(core_thread != RT_NULL);
    if (core_thread != RT_NULL)
        rt_thread_startup(core_thread);

    rt_thread_t cpad_thread;
    cpad_thread = rt_thread_create("cpad", cpad_thread_entry, RT_NULL,
                                   512, CPAD_THREAD_PRIO, 5);
    RT_ASSERT(cpad_thread != RT_NULL);
    if (cpad_thread != RT_NULL)
        rt_thread_startup(cpad_thread);

    rt_thread_t bkg_thread;
    bkg_thread = rt_thread_create("background", bkg_thread_entry, RT_NULL,
                                  512, BKG_THREAD_PRIO, 5);
    RT_ASSERT(bkg_thread != RT_NULL);
    if (bkg_thread != RT_NULL)
        rt_thread_startup(bkg_thread);

    rt_thread_t testcase_thread;
    testcase_thread = rt_thread_create("testcase",
                                       testcase_thread_entry, RT_NULL,
                                       512, TESTCASE_THREAD_PRIO, 5); // 初始化进程
    RT_ASSERT(testcase_thread != RT_NULL);
    if (testcase_thread != RT_NULL)
        rt_thread_startup(testcase_thread);

    rt_thread_t net_thead;
    net_thead = rt_thread_create("network",
                                 net_thread_entry, RT_NULL,
                                 3072, NET_THREAD_PRIO, 20); // 初始化进程
    RT_ASSERT(net_thead != RT_NULL);
    // if (net_thead != RT_NULL)
    //     rt_thread_startup(net_thead);

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

void sys_reboot(void)
{
    NVIC_SystemReset();
}

#ifdef RT_USING_FINSH
#include <finsh.h>
FINSH_FUNCTION_EXPORT(sys_reboot, software reset.);
#ifdef FINSH_USING_MSH
MSH_CMD_EXPORT(sys_reboot, software reset.);
#endif
#endif
