#define _WATCHDOG_H

#include "stm32f4xx_iwdg.h"
#include "watchdog_bsp.h"

// mode  0--运行状态下的时间   1--sleep 下的时间
void watchdog_init(void)
{
    //RCC_LSICmd(ENABLE);//打开LSI
    //while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY)==RESET);//等待直到LSI稳定
    
    /* Enable write access to IWDG_PR and IWDG_RLR registers */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    /* IWDG counter clock: 32KHz(LSI)    */   
    IWDG_SetPrescaler(IWDG_Prescaler_128);
    /* Set counter reload value to 300  1s */
    IWDG_SetReload(500); //Tout=(128*500)/32 (ms).
    /* Reload IWDG counter */
    IWDG_ReloadCounter();
    /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
    IWDG_Enable();
}

void dog(void)
{
    IWDG_ReloadCounter();
}
