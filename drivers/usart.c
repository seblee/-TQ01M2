/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2010-03-29     Bernard      remove interrupt Tx and DMA Rx mode
 * 2012-02-08     aozima       update for F4.
 * 2012-07-28     aozima       update for ART board.
 * 2016-05-28     armink       add DMA Rx mode
 */

#include "stm32f4xx.h"
#include "usart.h"
#include "board.h"
#include "local_status.h"
#include "port.h"

#include <rtdevice.h>

/* UART GPIO define. */
#define UART1_GPIO_TX       GPIO_Pin_9
#define UART1_TX_PIN_SOURCE GPIO_PinSource9
#define UART1_GPIO_RX       GPIO_Pin_10
#define UART1_RX_PIN_SOURCE GPIO_PinSource10
#define UART1_GPIO          GPIOA
#define UART1_GPIO_RCC      RCC_AHB1Periph_GPIOA
#define RCC_APBPeriph_UART1 RCC_APB2Periph_USART1

#define UART2_GPIO_TX       GPIO_Pin_2
#define UART2_TX_PIN_SOURCE GPIO_PinSource2
#define UART2_GPIO_RX       GPIO_Pin_3
#define UART2_RX_PIN_SOURCE GPIO_PinSource3
#define UART2_GPIO          GPIOA
#define UART2_GPIO_RCC      RCC_AHB1Periph_GPIOA
#define RCC_APBPeriph_UART2 RCC_APB1Periph_USART2

#define UART3_GPIO_TX       GPIO_Pin_8
#define UART3_TX_PIN_SOURCE GPIO_PinSource8
#define UART3_GPIO_RX       GPIO_Pin_9
#define UART3_RX_PIN_SOURCE GPIO_PinSource9
#define UART3_GPIO          GPIOD
#define UART3_GPIO_RCC      RCC_AHB1Periph_GPIOD
#define RCC_APBPeriph_UART3 RCC_APB1Periph_USART3

#define UART4_GPIO_TX       GPIO_Pin_10
#define UART4_TX_PIN_SOURCE GPIO_PinSource10
#define UART4_GPIO_RX       GPIO_Pin_11
#define UART4_RX_PIN_SOURCE GPIO_PinSource11
#define UART4_GPIO          GPIOC
#define UART4_GPIO_RCC      RCC_AHB1Periph_GPIOC
#define RCC_APBPeriph_UART4 RCC_APB1Periph_UART4

#define UART5_GPIO_TX       GPIO_Pin_12
#define UART5_TX_PIN_SOURCE GPIO_PinSource12
#define UART5_GPIO_RX       GPIO_Pin_2
#define UART5_RX_PIN_SOURCE GPIO_PinSource2
#define UART5_TX            GPIOC
#define UART5_RX            GPIOD
#define UART5_GPIO_RCC_TX   RCC_AHB1Periph_GPIOC
#define UART5_GPIO_RCC_RX   RCC_AHB1Periph_GPIOD
#define RCC_APBPeriph_UART5 RCC_APB1Periph_UART5

static uint16_t u16FrameCheckSum;
static uint8_t u8FrameDataLength;
uint8_t g_ComBuff[UART_NUM][PROTOCOL_FRAME_MaxLen];
PROTOCOL_STATUS g_ComStat[UART_NUM]; //通讯通道工作状态
uint8_t g_ComGap[UART_NUM];          //通讯帧接收间隔定时器
ProtocolLayer Protocol[UART_NUM];

/* STM32 uart driver */
struct stm32_uart
{
    USART_TypeDef *uart_device;
    IRQn_Type irq;
#ifdef RT_SERIAL_USING_DMA
    struct stm32_uart_dma
    {
        /* dma stream */
        DMA_Stream_TypeDef *rx_stream;
        /* dma channel */
        uint32_t rx_ch;
        /* dma flag */
        uint32_t rx_flag;
        /* dma irq channel */
        uint8_t rx_irq_ch;
        /* setting receive len */
        rt_size_t setting_recv_len;
        /* last receive index */
        rt_size_t last_recv_index;
    } dma;
#endif /* RT_SERIAL_USING_DMA */ 
}; 
 
#ifdef RT_SERIAL_USING_DMA 
static void DMA_Configuration(struct rt_serial_device *serial);
#endif /* RT_SERIAL_USING_DMA */ 

static rt_err_t stm32_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct stm32_uart* uart;
    USART_InitTypeDef USART_InitStructure;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    uart = (struct stm32_uart *)serial->parent.user_data;

    USART_InitStructure.USART_BaudRate = cfg->baud_rate;

    if (cfg->data_bits == DATA_BITS_8){
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    } else if (cfg->data_bits == DATA_BITS_9) {
        USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    }

    if (cfg->stop_bits == STOP_BITS_1){
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
    } else if (cfg->stop_bits == STOP_BITS_2){
        USART_InitStructure.USART_StopBits = USART_StopBits_2;
    }

    if (cfg->parity == PARITY_NONE){
        USART_InitStructure.USART_Parity = USART_Parity_No;
    } else if (cfg->parity == PARITY_ODD) {
        USART_InitStructure.USART_Parity = USART_Parity_Odd;
    } else if (cfg->parity == PARITY_EVEN) {
        USART_InitStructure.USART_Parity = USART_Parity_Even;
    }

    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(uart->uart_device, &USART_InitStructure);

    /* Enable USART */
    USART_Cmd(uart->uart_device, ENABLE);

    return RT_EOK;
}

static rt_err_t stm32_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct stm32_uart* uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        UART_DISABLE_IRQ(uart->irq);
        /* disable interrupt */
        USART_ITConfig(uart->uart_device, USART_IT_RXNE, DISABLE);
        break;
    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        UART_ENABLE_IRQ(uart->irq);
        /* enable interrupt */
        USART_ITConfig(uart->uart_device, USART_IT_RXNE, ENABLE);
        break;
#ifdef RT_SERIAL_USING_DMA
        /* USART config */
    case RT_DEVICE_CTRL_CONFIG :
        if ((rt_uint32_t)(arg) == RT_DEVICE_FLAG_DMA_RX) {
            DMA_Configuration(serial);
        }
#endif /* RT_SERIAL_USING_DMA */      
    }

    return RT_EOK;
}

static int stm32_putc(struct rt_serial_device *serial, char c)
{
    struct stm32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;

    USART_ClearFlag(uart->uart_device,USART_FLAG_TC);
    uart->uart_device->DR = c;
    while (!(uart->uart_device->SR & USART_FLAG_TC));

    return 1;
}

static int stm32_getc(struct rt_serial_device *serial)
{
    int ch;
    struct stm32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;

    ch = -1;
    if (uart->uart_device->SR & USART_FLAG_RXNE)
    {
        ch = uart->uart_device->DR & 0xff;
    }

    return ch;
}

#ifdef RT_SERIAL_USING_DMA
/**
 * DMA initialize by DMA_InitStruct structure
 *
 * @param serial serial device
 * @param setting_recv_len setting receive length
 * @param mem_base_addr memory 0 base address for DMA stream
 */
static void dma_uart_config(struct rt_serial_device *serial, uint32_t setting_recv_len,
        void *mem_base_addr)
{
    struct stm32_uart *uart = (struct stm32_uart *) serial->parent.user_data;
    DMA_InitTypeDef DMA_InitStructure;

    /* rx dma config */
    uart->dma.setting_recv_len = setting_recv_len;
    DMA_DeInit(uart->dma.rx_stream);
    while (DMA_GetCmdStatus(uart->dma.rx_stream) != DISABLE);
    DMA_InitStructure.DMA_Channel = uart->dma.rx_ch;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(uart->uart_device->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)mem_base_addr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = uart->dma.setting_recv_len;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(uart->dma.rx_stream, &DMA_InitStructure);
}

/**
 * Serial port receive idle process. This need add to uart idle ISR.
 *
 * @param serial serial device
 */
static void dma_uart_rx_idle_isr(struct rt_serial_device *serial) {
    struct stm32_uart *uart = (struct stm32_uart *) serial->parent.user_data;
    rt_size_t recv_total_index, recv_len;
    rt_base_t level;

    /* disable interrupt */
    level = rt_hw_interrupt_disable();

    recv_total_index = uart->dma.setting_recv_len - DMA_GetCurrDataCounter(uart->dma.rx_stream);
    recv_len = recv_total_index - uart->dma.last_recv_index;
    uart->dma.last_recv_index = recv_total_index;
    /* enable interrupt */
    rt_hw_interrupt_enable(level);

    if (recv_len) rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_DMADONE | (recv_len << 8));

    /* read a data for clear receive idle interrupt flag */
    USART_ReceiveData(uart->uart_device);
}

/**
 * DMA receive done process. This need add to DMA receive done ISR.
 *
 * @param serial serial device
 */
static void dma_rx_done_isr(struct rt_serial_device *serial)
{
    struct stm32_uart *uart = (struct stm32_uart *) serial->parent.user_data;
    rt_size_t recv_len;
    rt_base_t level;

    if (DMA_GetFlagStatus(uart->dma.rx_stream, uart->dma.rx_flag) != RESET)
    {
        /* disable interrupt */
        level = rt_hw_interrupt_disable();

        recv_len = uart->dma.setting_recv_len - uart->dma.last_recv_index;
        /* reset last recv index */
        uart->dma.last_recv_index = 0;
        /* enable interrupt */
        rt_hw_interrupt_enable(level);

        if (recv_len) rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_DMADONE | (recv_len << 8));

        /* start receive data */
        DMA_ClearFlag(uart->dma.rx_stream, uart->dma.rx_flag);
    }
}
#endif /* RT_SERIAL_USING_DMA */ 

/**
 * Uart common interrupt process. This need add to uart ISR.
 *
 * @param serial serial device
 */
static void uart_isr(struct rt_serial_device *serial)
{
    struct stm32_uart *uart = (struct stm32_uart *) serial->parent.user_data;

    RT_ASSERT(uart != RT_NULL);

    if(USART_GetITStatus(uart->uart_device, USART_IT_RXNE) != RESET)
    {
        rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND);
        /* clear interrupt */
        USART_ClearITPendingBit(uart->uart_device, USART_IT_RXNE);
    }
#ifdef RT_SERIAL_USING_DMA       
    if(USART_GetITStatus(uart->uart_device, USART_IT_IDLE) != RESET)
    {
        dma_uart_rx_idle_isr(serial);
    }
#endif /* RT_SERIAL_USING_DMA */       
    if (USART_GetITStatus(uart->uart_device, USART_IT_TC) != RESET)
    {
        /* clear interrupt */
        USART_ClearITPendingBit(uart->uart_device, USART_IT_TC);
    }
    if (USART_GetFlagStatus(uart->uart_device, USART_FLAG_ORE) == SET)
    {
        USART_ReceiveData(uart->uart_device);
    }
}


static const struct rt_uart_ops stm32_uart_ops =
{
    stm32_configure,
    stm32_control,
    stm32_putc,
    stm32_getc,
};

#if defined(RT_USING_UART1)
/* UART1 device driver structure */
struct stm32_uart uart1 =
{
    USART1,     
    USART1_IRQn,
#ifdef RT_SERIAL_USING_DMA  
    {
        DMA2_Stream5,
        DMA_Channel_4,
        DMA_FLAG_TCIF5,
        DMA2_Stream5_IRQn,
        0,
    },
#endif /* RT_SERIAL_USING_DMA */    
};
struct rt_serial_device serial1;

void USART1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&serial1);

    /* leave interrupt */
    rt_interrupt_leave();
}

#ifdef RT_SERIAL_USING_DMA   
void DMA2_Stream5_IRQHandler(void) {
    /* enter interrupt */
    rt_interrupt_enter();

    dma_rx_done_isr(&serial1);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_SERIAL_USING_DMA */

#endif /* RT_USING_UART1 */

#if defined(RT_USING_UART2)
/* UART2 device driver structure */
struct stm32_uart uart2 =
{
    USART2,
    USART2_IRQn,
#ifdef RT_SERIAL_USING_DMA
    {
        DMA1_Stream5,
        DMA_Channel_4,
        DMA_FLAG_TCIF5,
        DMA1_Stream5_IRQn,
        0,
        0,
    },
#endif /* RT_SERIAL_USING_DMA */    
};
struct rt_serial_device serial2;

void USART2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&serial2);

    /* leave interrupt */
    rt_interrupt_leave();
}

#ifdef RT_SERIAL_USING_DMA
void DMA1_Stream5_IRQHandler(void) {
    /* enter interrupt */
    rt_interrupt_enter();

    dma_rx_done_isr(&serial2);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_SERIAL_USING_DMA */

#endif /* RT_USING_UART2 */

#if defined(RT_USING_UART3)
/* UART3 device driver structure */
struct stm32_uart uart3 =
{
    USART3,  
    USART3_IRQn,
#ifdef RT_SERIAL_USING_DMA  
    {
        DMA1_Stream1,
        DMA_Channel_4,
        DMA_FLAG_TCIF1,
        DMA1_Stream1_IRQn,
        0,
        0,
    },
#endif /* RT_SERIAL_USING_DMA */    
};
struct rt_serial_device serial3;

void USART3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&serial3);

    /* leave interrupt */
    rt_interrupt_leave();
}

#ifdef RT_SERIAL_USING_DMA
void DMA1_Stream1_IRQHandler(void) {
    /* enter interrupt */
    rt_interrupt_enter();

    dma_rx_done_isr(&serial3);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_SERIAL_USING_DMA */

#endif /* RT_USING_UART3 */

#if defined(RT_USING_UART4)
/* UART4 device driver structure */
struct stm32_uart uart4 =
{
    UART4,
    UART4_IRQn,
#ifdef RT_SERIAL_USING_DMA
    {
        DMA1_Stream2,
        DMA_Channel_4,
        DMA_FLAG_TCIF2,
        DMA1_Stream2_IRQn,
        0,
        0,
    },
#endif /* RT_SERIAL_USING_DMA */
};
struct rt_serial_device serial4;

void UART4_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&serial4);

    /* leave interrupt */
    rt_interrupt_leave();
}

#ifdef RT_SERIAL_USING_DMA
void DMA1_Stream2_IRQHandler(void) {
    /* enter interrupt */
    rt_interrupt_enter();

    dma_rx_done_isr(&serial4);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_SERIAL_USING_DMA */

#endif /* RT_USING_UART4 */

#if defined(RT_USING_UART5)
/* UART5 device driver structure */
struct stm32_uart uart5 =
{
    UART5,  
    UART5_IRQn,
#ifdef RT_SERIAL_USING_DMA  
    {
        DMA1_Stream0,
        DMA_Channel_4,
        DMA_FLAG_TCIF0,
        DMA1_Stream0_IRQn,
        0,
        0,
    },
#endif /* RT_SERIAL_USING_DMA */    
};
struct rt_serial_device serial5;

void UART5_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&serial5);

    /* leave interrupt */
    rt_interrupt_leave();
}

#ifdef RT_SERIAL_USING_DMA
void DMA1_Stream0_IRQHandler(void) {
    /* enter interrupt */
    rt_interrupt_enter();

    dma_rx_done_isr(&serial5);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_SERIAL_USING_DMA */

#endif /* RT_USING_UART5 */

static void RCC_Configuration(void)
{
#ifdef RT_USING_UART1
    /* Enable UART1 GPIO clocks */
    RCC_AHB1PeriphClockCmd(UART1_GPIO_RCC, ENABLE);
    /* Enable UART1 clock */
    RCC_APB2PeriphClockCmd(RCC_APBPeriph_UART1, ENABLE);
#endif /* RT_USING_UART1 */

#ifdef RT_USING_UART2
    /* Enable UART2 GPIO clocks */
    RCC_AHB1PeriphClockCmd(UART2_GPIO_RCC, ENABLE);
    /* Enable UART2 clock */
    RCC_APB1PeriphClockCmd(RCC_APBPeriph_UART2, ENABLE);
#endif /* RT_USING_UART1 */

#ifdef RT_USING_UART3
    /* Enable UART3 GPIO clocks */
    RCC_AHB1PeriphClockCmd(UART3_GPIO_RCC, ENABLE);
    /* Enable UART3 clock */
    RCC_APB1PeriphClockCmd(RCC_APBPeriph_UART3, ENABLE);
#endif /* RT_USING_UART3 */

#ifdef RT_USING_UART4
    /* Enable UART4 GPIO clocks */
    RCC_AHB1PeriphClockCmd(UART4_GPIO_RCC, ENABLE);
    /* Enable UART4 clock */
    RCC_APB1PeriphClockCmd(RCC_APBPeriph_UART4, ENABLE);
#endif /* RT_USING_UART4 */

#ifdef RT_USING_UART5
    /* Enable UART5 GPIO clocks */
    RCC_AHB1PeriphClockCmd(UART5_GPIO_RCC_TX | UART5_GPIO_RCC_RX, ENABLE);
    /* Enable UART5 clock */
    RCC_APB1PeriphClockCmd(RCC_APBPeriph_UART5, ENABLE);
#endif /* RT_USING_UART5 */
}

static void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

#ifdef RT_USING_UART1
    /* Configure USART1 Rx/tx PIN */
    GPIO_InitStructure.GPIO_Pin = UART1_GPIO_RX | UART1_GPIO_TX;
     /* Connect alternate function */
    GPIO_PinAFConfig(UART1_GPIO, UART1_TX_PIN_SOURCE, GPIO_AF_USART1);
    GPIO_PinAFConfig(UART1_GPIO, UART1_RX_PIN_SOURCE, GPIO_AF_USART1);    
    
    GPIO_Init(UART1_GPIO, &GPIO_InitStructure);
#endif /* RT_USING_UART1 */

#ifdef RT_USING_UART2
    /* Configure USART2 Rx/tx PIN */
    GPIO_InitStructure.GPIO_Pin = UART2_GPIO_RX | UART2_GPIO_TX;
    /* Connect alternate function */
    GPIO_PinAFConfig(UART2_GPIO, UART2_TX_PIN_SOURCE, GPIO_AF_USART2);
    GPIO_PinAFConfig(UART2_GPIO, UART2_RX_PIN_SOURCE, GPIO_AF_USART2);
    
    GPIO_Init(UART2_GPIO, &GPIO_InitStructure);
#endif /* RT_USING_UART2 */

#ifdef RT_USING_UART3
    /* Configure USART3 Rx/tx PIN */
    GPIO_InitStructure.GPIO_Pin = UART3_GPIO_TX | UART3_GPIO_RX;
    /* Connect alternate function */
    GPIO_PinAFConfig(UART3_GPIO, UART3_TX_PIN_SOURCE, GPIO_AF_USART3);
    GPIO_PinAFConfig(UART3_GPIO, UART3_RX_PIN_SOURCE, GPIO_AF_USART3);
    
    GPIO_Init(UART3_GPIO, &GPIO_InitStructure);
#endif /* RT_USING_UART3 */

#ifdef RT_USING_UART4
    /* Configure USART4 Rx/tx PIN */
    GPIO_InitStructure.GPIO_Pin = UART4_GPIO_TX | UART4_GPIO_RX;
    /* Connect alternate function */
    GPIO_PinAFConfig(UART4_GPIO, UART4_TX_PIN_SOURCE, GPIO_AF_UART4);
    GPIO_PinAFConfig(UART4_GPIO, UART4_RX_PIN_SOURCE, GPIO_AF_UART4);
    
    GPIO_Init(UART4_GPIO, &GPIO_InitStructure);
#endif /* RT_USING_UART4 */

#ifdef RT_USING_UART5
    /* Configure USART5 Rx/tx PIN */
    GPIO_InitStructure.GPIO_Pin = UART5_GPIO_TX;
 /* Connect alternate function */
    GPIO_PinAFConfig(UART5_TX, UART5_TX_PIN_SOURCE, GPIO_AF_UART5); 
    GPIO_Init(UART5_TX, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = UART5_GPIO_RX;
    GPIO_PinAFConfig(UART5_RX, UART5_RX_PIN_SOURCE, GPIO_AF_UART5);
    GPIO_Init(UART5_RX, &GPIO_InitStructure);
#endif /* RT_USING_UART5 */
}

static void NVIC_Configuration(struct stm32_uart *uart)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = uart->irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

#ifdef RT_SERIAL_USING_DMA
static void DMA_Configuration(struct rt_serial_device *serial) {
    struct stm32_uart *uart = (struct stm32_uart *) serial->parent.user_data;
    struct rt_serial_rx_fifo *rx_fifo = (struct rt_serial_rx_fifo *)serial->serial_rx;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* enable transmit idle interrupt */
    USART_ITConfig(uart->uart_device, USART_IT_IDLE , ENABLE);

    /* DMA clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    /* rx dma config */
    dma_uart_config(serial, serial->config.bufsz, rx_fifo->buffer);

    DMA_ClearFlag(uart->dma.rx_stream, uart->dma.rx_flag);
    DMA_ITConfig(uart->dma.rx_stream, DMA_IT_TC, ENABLE);
    USART_DMACmd(uart->uart_device, USART_DMAReq_Rx, ENABLE);
    DMA_Cmd(uart->dma.rx_stream, ENABLE);

    /* rx dma interrupt config */
    NVIC_InitStructure.NVIC_IRQChannel = uart->dma.rx_irq_ch;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
#endif /* RT_SERIAL_USING_DMA */

int stm32_hw_usart_init(void)
{
    struct stm32_uart *uart;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    RCC_Configuration();
    GPIO_Configuration();

#ifdef RT_USING_UART1
    uart = &uart1;

    serial1.ops    = &stm32_uart_ops;
    serial1.config = config;

    NVIC_Configuration(&uart1);

    /* register UART1 device */
    rt_hw_serial_register(&serial1,
                          "uart1",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_RX,
                          uart);
#endif /* RT_USING_UART1 */

#ifdef RT_USING_UART2
    uart = &uart2;

    serial2.ops    = &stm32_uart_ops;
    serial2.config = config;

    NVIC_Configuration(&uart2);

    /* register UART1 device */
    rt_hw_serial_register(&serial2,
                          "uart2",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_RX,
                          uart);
#endif /* RT_USING_UART2 */

#ifdef RT_USING_UART3
    uart = &uart3;

    serial3.ops    = &stm32_uart_ops;
    serial3.config = config;

    NVIC_Configuration(&uart3);

    /* register UART3 device */
    rt_hw_serial_register(&serial3,
                          "uart3",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_RX,
                          uart);
#endif /* RT_USING_UART3 */

#ifdef RT_USING_UART4
    uart = &uart4;

    serial4.ops    = &stm32_uart_ops;
    serial4.config = config;

    NVIC_Configuration(&uart4);

    /* register UART4 device */
    rt_hw_serial_register(&serial4,
                          "uart4",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_RX,
                          uart);
#endif /* RT_USING_UART4 */

#ifdef RT_USING_UART5
    uart = &uart5;

    serial5.ops    = &stm32_uart_ops;
    serial5.config = config;

    NVIC_Configuration(&uart5);

    /* register UART5 device */
    rt_hw_serial_register(&serial5,
                          "uart5",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_RX,
                          uart);
#endif /* RT_USING_UART5 */
    return 0;
}
INIT_BOARD_EXPORT(stm32_hw_usart_init); 
/**********************************佛山吉宝加热器***********************************************/
#ifndef  RT_USING_UART1 
void Usart1_Rcc_Conf(void)
{
    /* Enable UART1 GPIO clocks */
    RCC_AHB1PeriphClockCmd(UART1_GPIO_RCC, ENABLE);
    /* Enable UART1 clock */
    RCC_APB2PeriphClockCmd(RCC_APBPeriph_UART1, ENABLE);
    return;
}

void Usart1_Gpio_Conf(void)
{   
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;


    /* Configure USART1 Rx/tx PIN */
    GPIO_InitStructure.GPIO_Pin = UART1_GPIO_RX | UART1_GPIO_TX;
     /* Connect alternate function */
    GPIO_PinAFConfig(UART1_GPIO, UART1_TX_PIN_SOURCE, GPIO_AF_USART1);
    GPIO_PinAFConfig(UART1_GPIO, UART1_RX_PIN_SOURCE, GPIO_AF_USART1);    
    
    GPIO_Init(UART1_GPIO, &GPIO_InitStructure);
 
    return;
}
void Usart1_Init_Conf(void)
{
    //USART1配置
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    ENTER_CRITICAL_SECTION(); //关全局中断

    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);

    //=====================中断初始化======================================
    //设置NVIC优先级分组为Group2：0-3抢占式优先级，0-3的响应式优先级
    //	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    EXIT_CRITICAL_SECTION(); //开全局中断
    return;
}

void Usart1_Var_Init(uint8_t port)
{
    g_ComStat[port] = INIT_Apply;
    g_ComGap[port] = 0;
    Protocol[port].StatckStatus = PROTOCOL_STACK_IDLE;
    rt_memset(&g_ComBuff[port][0], 0x00, PROTOCOL_FRAME_MaxLen);
    return;
}
void Heat_Usart_Init(uint8_t port)
{
    Usart1_Var_Init(port);
    Usart1_Rcc_Conf();
    Usart1_Gpio_Conf();
    Usart1_Init_Conf();
    return;
}
//通信串口初始化
void xPort_Usart_Init(uint8_t ucMB_Number)
{
    switch (ucMB_Number)
    {
    case UART_HEAT:
        Heat_Usart_Init(ucMB_Number);
        break;
    case UART_PM25:

        break;
    default:
        break;
    }
    return;
}

void Heat_Usart_Close(void)
{
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    USART_Cmd(USART1, DISABLE);
    return;
}
//关闭串口
void xPort_Usart_Close(uint8_t ucMB_Number)
{
    switch (ucMB_Number)
    {
    case UART_HEAT:
        Heat_Usart_Close();
        break;
    case UART_PM25:

        break;
    default:
        break;
    }
    return;
}
#endif
//串口发送数据
uint8_t UART_Send(uint8_t *u8Buf, uint8_t u8Len)
{
    uint8_t u8Se;
    uint8_t i;
    if (u8Len == 0)
    {
        return u8Se;
    }
    for (i = 0; i < u8Len; i++)
    {
        USART_SendData(USART1, u8Buf[i]); //发送数据
        while (USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET)
            ; //等待发送结束
        u8Se = 1;
    }
    return u8Se;
}

//串口发送数据
uint8_t Heat_Send(uint8_t u8Type, uint8_t u8OC, uint8_t u8Temp, uint16_t u16WL)
{
    uint8_t u8SendBuf[HEATWRITE_NUM + 2] = {0};
    uint8_t u8Head = 0xAA;
    uint8_t u8Addr = 0x01;
    uint16_t u16Sum = 0x00;
    uint8_t i;

    if (u8Type == HEAT_READPARA)
    {
        u8SendBuf[0] = 0xA5;
        return UART_Send(&u8SendBuf[0], 1);
    }
    else
    {
        u8SendBuf[0] = u8Head;
        u8SendBuf[1] = u8Addr;
        u8SendBuf[2] = u8OC;
        u8SendBuf[3] = u8Temp;
        u8SendBuf[4] = u16WL >> 8;
        u8SendBuf[5] = (uint8_t)u16WL;
        for (i = 1; i < HEATWRITE_NUM - 1; i++)
        {
            u16Sum += u8SendBuf[i];
        }
        u8SendBuf[HEATWRITE_NUM - 1] = u16Sum >> 8;
        u8SendBuf[HEATWRITE_NUM] = (uint8_t)u16Sum;

        return UART_Send(&u8SendBuf[0], HEATWRITE_NUM + 1);
    }
}

uint8_t xPortSerialGetByte(char *pucByte, uint8_t ucMB_Number)
{
    switch (ucMB_Number)
    {
    case UART_HEAT:
        *pucByte = USART_ReceiveData(USART1);
        break;
    case UART_PM25:
        *pucByte = USART_ReceiveData(UART5);
        break;
    default:
        break;
    }
    return 1;
}

uint8_t AnalyseProtocol(uint8_t *u8Buff)
{
    extern local_reg_st l_sys;

    //出水状态
    l_sys.HeatWater_st = u8Buff[1];
    //出水流量
    l_sys.HeatWater_Flow = u8Buff[3];
    l_sys.HeatWater_Flow = (l_sys.HeatWater_Flow << 8) | u8Buff[4];
    rt_kprintf("u8Buff[1]=%d,3=%d,4=%d,l_sys.HeatWater_Flow=%d\n", u8Buff[1], u8Buff[3], u8Buff[4], l_sys.HeatWater_Flow);

    return 1;
}
/*************************************************
Function:void Comm_Service(void)// 函数名称
Description:// 函数功能、性能等的描述
			执行通讯任务
			状态机转换（正常情况）: RECV_Wait->RECV_Going->RECV_Over->
                                    SEND_Wait->SEND_Going->SEND_Over->RECV_Wait
Author:xdp（作者）
Calls:// 被本函数调用的函数清单
Called By:// 调用本函数的函数清单
			main();
Input:// 输入参数说明，包括每个参数的作
    // 用、取值说明及参数间关系。
Output:// 对输出参数的说明。
Return:// 函数返回值的说明
Others:// 其它说明
*************************************************/
void Comm_Service(void)
{
    extern sys_reg_st g_sys;
    uint8_t port; //通讯通道

    //    for (port=0; port<UART_NUM; port++)
    {
        port = UART_HEAT;
        g_ComGap[port]++;

        //		rt_kprintf("g_ComStat=%d,TEST=%d\n",g_ComStat[port],g_sys.status.ComSta.TEST);
        switch (g_ComStat[port])
        {
        case RECV_Wait: //接收等待（检查接收端是否打开）-------------------------

            break;
        case RECV_Going:                                 //接收进行（由g_ComGap检查）----------------------------
            if (g_ComGap[port] > PROTOCOL_FRAME_ByteGap) //接收超时？
            {
                g_ComStat[port] = INIT_Apply; //置"初始化"状态
            } 
            break;
        case RECV_Over:                                //接收完成（转换状态明确）-------------------------------
                                                       //	            xPort_Usart_Close(port);                           //关闭通讯通道
            if (!AnalyseProtocol(&g_ComBuff[port][0])) //
            {
            }
            rt_memset(&g_ComBuff[port][0], 0x00, PROTOCOL_FRAME_MaxLen);
            g_ComStat[port] = SEND_Wait; //置"发送等待"状态
            break;
        case SEND_Wait: //发送等待（由g_ComGap检查）----------------------------
                        //	            if (g_ComGap[port]> PROTOCOL_FRAME_SendGap)               //发送延时到？
        {
            g_ComGap[port] = 0;           //通讯字节间隔定时开始
            g_ComStat[port] = SEND_Going; //置"正在发送"状态
        }
        break;
        case SEND_Going: //正在发送（检查发送端是否打开）------------------------
            if (g_ComGap[port] < 100)
            {
                break;
            }
        case SEND_Over:
        case INIT_Apply:                 //接收错误----------------------------------------------
        default:                         //其他状态------------------------------------------------------
                                         //	          xPort_Usart_Init(port);          //通道初始化（复位）
            g_ComStat[port] = RECV_Wait; //置"接收等待"状态
                                         //							memset(&g_ComBuff[port][0],0x00,PROTOCOL_FRAME_MaxLen);
            break;
        }
    }
    return;
}
#ifndef  RT_USING_UART1 
//加热器串口接收数据
uint8_t HeatPortSerialReceiveFSM(uint8_t port)
{
    extern sys_reg_st g_sys;
    uint8_t RXDByte;
    uint16_t u16CheckSum;

    g_sys.status.ComSta.TEST |= 0x01;
    /* Always read the character. */
    (void)xPortSerialGetByte((char *)&RXDByte, port);
    g_ComGap[port] = 0;
    switch (Protocol[port].StatckStatus)
    {
    case PROTOCOL_STACK_IDLE:
        // 空闲状态,等待接收帧首字符(01)
        //		    if ((g_ComStat[port]==RECV_Wait)                        //当前正"接收等待"
        //		    &&(0x01== RXDByte))
        if (0x01 == RXDByte)
        {
            g_ComStat[port] = RECV_Going; //置"接收进行"状态
            g_ComBuff[port][0] = RXDByte;
            u16FrameCheckSum = RXDByte;
            Protocol[port].DataCount = 1;
            Protocol[port].StatckStatus++;
            g_sys.status.ComSta.TEST |= 0x02;
            g_sys.status.ComSta.TEST4 = u16FrameCheckSum;
        }
        break;
    case PROTOCOL_STACK_CK:
        if ((HEAT_OPEN == RXDByte) //当前正"接收等待"
            || (HEAT_CLOSE == RXDByte))
        {
            // 累加校验和
            u16FrameCheckSum += RXDByte;
            g_ComBuff[port][(Protocol[port].DataCount)++] = RXDByte;
            u8FrameDataLength = 14;
            Protocol[port].StatckStatus++;
            g_sys.status.ComSta.TEST |= 0x04;
        }
        else
        {
            // 帧错误,协议解析复位
            Protocol[port].StatckStatus = PROTOCOL_STACK_IDLE;
            g_ComStat[port] = INIT_Apply; //置"发送等待"状态
        }
        g_sys.status.ComSta.TEST5 = u16FrameCheckSum;
        break;
    case PROTOCOL_STACK_RCV:
        // 累加校验和
        u16FrameCheckSum += RXDByte;
        g_ComBuff[port][(Protocol[port].DataCount)++] = RXDByte;
        u8FrameDataLength--;
        // 接收数据域的数据
        if (!u8FrameDataLength)
        {
            // 接收校验和
            g_sys.status.ComSta.TEST6 = u16FrameCheckSum;
            u8FrameDataLength = 2;
            Protocol[port].StatckStatus++;
        }
        g_sys.status.ComSta.TEST |= 0x08;
        break;
    case PROTOCOL_STACK_CS:
        g_ComBuff[port][(Protocol[port].DataCount)++] = RXDByte;
        u8FrameDataLength--;
        // 接收数据域的数据
        if (!u8FrameDataLength)
        {
            g_sys.status.ComSta.TEST |= 0x10;
            // 接收校验和
            u16CheckSum = g_ComBuff[port][16];
            u16CheckSum = (u16CheckSum << 8) | g_ComBuff[port][17];
            g_sys.status.ComSta.TEST2 = u16FrameCheckSum;
            g_sys.status.ComSta.TEST3 = u16CheckSum;
            if (u16CheckSum == u16FrameCheckSum)
            {
                g_sys.status.ComSta.TEST |= 0x20;
                Protocol[port].StatckStatus = PROTOCOL_STACK_IDLE;
                g_ComStat[port] = RECV_Over; //置"接收完成"状态
            }
            else
            {
                // 校验和不正确,协议解析复位
                g_sys.status.ComSta.TEST |= 0x40;
                Protocol[port].StatckStatus = PROTOCOL_STACK_IDLE;
                g_ComStat[port] = INIT_Apply; //置"初始化"状态
            }
        }
        break;
    case PROTOCOL_STACK_END:

        break;
    default:
        Protocol[port].StatckStatus = PROTOCOL_STACK_IDLE;
        g_ComStat[port] = INIT_Apply; //置"初始化"状态
        break;
    }
    return 1;
}

void USART1_IRQHandler(void)
{

    rt_interrupt_enter();
    //溢出错误
    if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) == SET)
    {
        ;
    }
    //接收中断
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        HeatPortSerialReceiveFSM(UART_HEAT);
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
    //发送中断
    if (USART_GetITStatus(USART1, USART_IT_TXE) == SET)
    {
        ;
    }
    rt_interrupt_leave();
}
#endif

