#include "drv_swj.h"
#include <rtthread.h>
int stm32_SWJ_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource13, GPIO_AF_SWJ);
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource13, GPIO_AF_SWJ);
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    return 0; 
}
INIT_DEVICE_EXPORT(stm32_SWJ_init);
