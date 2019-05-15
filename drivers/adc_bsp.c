/**
  ******************************************************************************
  * @file    bsp_xxx.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   adc1 应用bsp / DMA 模式
  ******************************************************************************/
/* Private variables ---------------------------------------------------------*/
#include "adc_bsp.h"
#include <rtthread.h>

#define PER AI_MAX_CNT //外设数量
#define NUM 10         //采集次数

volatile uint16_t ADC1ConvertedValue[AI_MAX_CNT];
volatile uint16_t fir[NUM][PER];
void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx, uint32_t chx, uint32_t par, uint32_t mar, uint16_t ndtr);
/**
  * @brief  initialize ADC1 clock , gpio and configurate adc dma setting
  * @param  none
  * @retval none
  */
//模拟输入初始化

uint16_t drv_adc_dma_init(void)
{
    MYDMA_Config(DMA2_Stream0, DMA_Channel_0, (uint32_t)&ADC1->DR, (uint32_t)fir, NUM * PER);
    ADC_SoftwareStartConv(ADC1); //开启DMA
    return 0;
}
void DMA_GPIO_config()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //使能GPIOA时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //使能GPIOC时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //不带上拉下拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 |
                                  GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;     //模拟输入
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //不带上拉下拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure); //初始化
}
void DMA_ADC_config()
{
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能ADC1时钟

    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE);  //ADC1复位
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, DISABLE); //复位结束

    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                      //独立模式
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_10Cycles; //两个采样阶段之间的延迟x个时钟
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;              //DMA使能（DMA传输下要设置使能）
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;                   //预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz
    ADC_CommonInit(&ADC_CommonInitStructure);                                     //初始化

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;                      //12位模式
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;                                //扫描（开启DMA传输要设置扫描）
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;                          //开启连续转换（开启DMA传输要设置连续转换）
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; //禁止触发检测，使用软件触发
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                      //右对齐
    ADC_InitStructure.ADC_NbrOfConversion = PER;                                //有几个通道传输就写几 （DMA传输下要设置为通道数）
    ADC_Init(ADC1, &ADC_InitStructure);                                         //ADC初始化

    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_144Cycles); //ADC1ConvertedValue[0]
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_144Cycles); //ADC1ConvertedValue[1]
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_144Cycles); //ADC1ConvertedValue[2]
    ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 4, ADC_SampleTime_144Cycles); //ADC1ConvertedValue[3]
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 5, ADC_SampleTime_144Cycles);  //ADC1ConvertedValue[4]
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 6, ADC_SampleTime_144Cycles);  //ADC1ConvertedValue[5]
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 7, ADC_SampleTime_144Cycles);  //ADC1ConvertedValue[6]

    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE); //开启AD转换器
}

//DMAx的各通道配置
//这里的传输形式是固定的,这点要根据不同的情况来修改
//从存储器->外设模式/8位数据宽度/存储器增量模式
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
//chx:DMA通道选择,@ref DMA_channel DMA_Channel_0~DMA_Channel_7
//par:外设地址
//mar:存储器地址
//ndtr:数据传输量
void DMA_config(DMA_Stream_TypeDef *DMA_Streamx, uint32_t chx, uint32_t par, uint32_t mar, uint16_t ndtr)
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    if ((uint32_t)DMA_Streamx > (uint32_t)DMA2) //得到当前stream是属于DMA2还是DMA1
    {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); //DMA2时钟使能
    }
    else
    {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE); //DMA1时钟使能
    }
    DMA_DeInit(DMA_Streamx);

    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE)
    {
    } //等待DMA可配置

    /* 配置 DMA Stream */
    DMA_InitStructure.DMA_Channel = chx;                                        //通道选择
    DMA_InitStructure.DMA_PeripheralBaseAddr = par;                             //DMA外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr = mar;                                //DMA 存储器0地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                     //存储器到外设模式
    DMA_InitStructure.DMA_BufferSize = ndtr;                                    //数据传输量
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            //外设非增量模式
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     //存储器增量模式
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //外设数据长度:8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;         //存储器数据长度:8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                             //使用普通模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                       //中等优先级
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;         //存储器突发单次传输
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; //外设突发单次传输
    DMA_Init(DMA_Streamx, &DMA_InitStructure);                          //初始化DMA Stream
    DMA_ClearFlag(DMA2_Stream0, DMA_IT_TC);
    DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01; //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;        //响应优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    while (DMA_GetCmdStatus(DMA2_Stream0) != DISABLE)
    {
    }
    DMA_Cmd(DMA2_Stream0, ENABLE);
}
void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx, uint32_t chx, uint32_t par, uint32_t mar, uint16_t ndtr)
{
    DMA_GPIO_config();
    DMA_ADC_config();
    DMA_config(DMA_Streamx, chx, par, mar, ndtr);
}

void filter(void)
{
    register uint16_t sum = 0;
    u8 count = 0, i = 0, j = 0;
    for (; i < PER; i++)
    {
        while (j < NUM)
        {
            if ((int)fir[j][i] < 0)
            {
            }
            else
            {
                sum += fir[j][i];
                count++;
            }
            j++;
        }
        ADC1ConvertedValue[i] = sum / count;
        sum = 0;
        count = 0;
        j = 0;
    }
}

//开启一次DMA传输
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
//ndtr:数据传输量
void DMA2_Stream0_IRQHandler(void)
{
    if (DMA_GetFlagStatus(DMA2_Stream0, DMA_IT_TCIF0) == SET)
    {
        filter();
        DMA_ClearFlag(DMA2_Stream0, DMA_IT_TCIF0);
    }
}
