/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-31     CGY       the first version
 */
#include "rc_ppm_input.h"
#include "loco_config.h"


#if(FBW_BORAD_TYPES == 0 ) //locolion_P1飞控， 采用STM32H750芯片
/************************************************************************************
 *    遥控器ppm信号输入捕获gpio初始化
***************************************************************************************/
/* 消息队列控制块 */
static struct rt_messagequeue ppm_Queue;
static rt_uint16_t  msg_pool3[1024];
TIM_HandleTypeDef TIM5_Handler;         //定时器5句柄
int PPM_IN_Init(void)
{
    TIM_IC_InitTypeDef TIM5_CH1Config;

    TIM5_Handler.Instance=TIM5;                          //通用定时器5
    TIM5_Handler.Init.Prescaler=200-1;                   //分频      TIM5的时钟频率为200M
    TIM5_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    TIM5_Handler.Init.Period=0XFFFFFFFF;                 //自动装载值
    TIM5_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_IC_Init(&TIM5_Handler);

    TIM5_CH1Config.ICPolarity=TIM_ICPOLARITY_RISING;     //上升沿捕获
    TIM5_CH1Config.ICSelection=TIM_ICSELECTION_DIRECTTI; //映射到TI1上
    TIM5_CH1Config.ICPrescaler=TIM_ICPSC_DIV1;           //配置输入分频,不分频
    TIM5_CH1Config.ICFilter=0;                           //配置输入滤波器,不滤波
    HAL_TIM_IC_ConfigChannel(&TIM5_Handler,&TIM5_CH1Config,TIM_CHANNEL_1); //配置TIM5通道1
    HAL_TIM_IC_Start_IT(&TIM5_Handler,TIM_CHANNEL_1);    //开始捕获TIM5的通道1
    __HAL_TIM_ENABLE_IT(&TIM5_Handler,TIM_IT_UPDATE);    //使能更新中断

    /* 初始化消息队列 */
    rt_mq_init(&ppm_Queue, "ppm_Queue",
               &msg_pool3[0],        /*内存池指向msg_pool */
               2,                    /*每个消息的大小是2字节 16位*/
               sizeof(msg_pool3),    /*内存池的大小是msg_pool的大小*/
               RT_IPC_FLAG_FIFO);    /*如果有多个线程等待,按照FIFO的方法分配消息 */

    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(PPM_IN_Init);
//定时器5底层驱动，时钟使能，引脚配置
//此函数会被HAL_TIM_IC_Init()调用
//htim:定时器5句柄
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_TIM5_CLK_ENABLE();            //使能TIM5时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();           //开启GPIAB时钟

    GPIO_Initure.Pin=GPIO_PIN_0;            //PA0
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用推挽输出
    GPIO_Initure.Pull=GPIO_PULLDOWN;        //下拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;     //高速
    GPIO_Initure.Alternate=GPIO_AF2_TIM5;   //PA0复用为TIM5通道1
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);

    HAL_NVIC_SetPriority(TIM5_IRQn,2,0);    //设置中断优先级，抢占优先级5，子优先级0
    HAL_NVIC_EnableIRQ(TIM5_IRQn);          //开启ITM5中断
}

//定时器5中断服务函数
void TIM5_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_TIM_IRQHandler(&TIM5_Handler); //定时器共用处理函数

    /* leave interrupt */
    rt_interrupt_leave();
}

//定时器输入捕获中断处理回调函数,该函数在HAL_TIM_IRQHandler中会被调用
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) //捕获中断发生时执行
{
    static uint16_t temp_cnt1,temp_cnt2;
    if ( GPIOA->IDR & GPIO_PIN_0)
    {
        temp_cnt1 = HAL_TIM_ReadCapturedValue(&TIM5_Handler,TIM_CHANNEL_1);   //获取当前的捕获值.

        TIM_RESET_CAPTUREPOLARITY(&TIM5_Handler,TIM_CHANNEL_1);                      //一定要先清除原来的设置！！
        TIM_SET_CAPTUREPOLARITY(&TIM5_Handler,TIM_CHANNEL_1,TIM_ICPOLARITY_FALLING); //配置TIM5通道1下升沿捕获
    }
    else
    {
        temp_cnt2 = HAL_TIM_ReadCapturedValue(&TIM5_Handler,TIM_CHANNEL_1);     //获取当前的捕获值.
        uint16_t _tmp;
        if ( temp_cnt2 >= temp_cnt1 )
        _tmp = temp_cnt2 - temp_cnt1;
        else
        _tmp = 0xffff - temp_cnt1 + temp_cnt2 + 1;
        rt_mq_send(&ppm_Queue,&_tmp, sizeof(rt_uint16_t));

        TIM_RESET_CAPTUREPOLARITY(&TIM5_Handler,TIM_CHANNEL_1);   //一定要先清除原来的设置！！
        TIM_SET_CAPTUREPOLARITY(&TIM5_Handler,TIM_CHANNEL_1,TIM_ICPOLARITY_RISING); //定时器5通道1设置为上降沿捕获
    }
}

int ppm_get_data(uint16_t *times)
{
    /* 从消息队列中接收消息 */
    return (rt_mq_recv(&ppm_Queue, times, sizeof(uint16_t), 0)== RT_EOK);
}

int get_ppm_Queue_num()
{
    /* 队列中已有的消息数 */
    return ppm_Queue.entry;
}

#endif




#if(FBW_BORAD_TYPES == 1 ) //locolion_A1飞控， 采用STM32F407芯片
/************************************************************************************
 *    遥控器ppm信号输入捕获gpio初始化
***************************************************************************************/
/* 消息队列控制块 */
static struct rt_messagequeue ppm_Queue;
static char msg_pool3[1024];
TIM_HandleTypeDef TIM3_Handler;         //定时器5句柄
int PPM_IN_Init(void)
{
    TIM_IC_InitTypeDef TIM3_CH1Config;

    TIM3_Handler.Instance=TIM3;                          //通用定时器5
    TIM3_Handler.Init.Prescaler=84-1;                   //分频      TIM5的时钟频率为200M
    TIM3_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    TIM3_Handler.Init.Period=0XFFFFFFFF;                 //自动装载值
    TIM3_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_IC_Init(&TIM3_Handler);

    TIM3_CH1Config.ICPolarity=TIM_ICPOLARITY_RISING;     //上升沿捕获
    TIM3_CH1Config.ICSelection=TIM_ICSELECTION_DIRECTTI; //映射到TI1上
    TIM3_CH1Config.ICPrescaler=TIM_ICPSC_DIV1;           //配置输入分频,不分频
    TIM3_CH1Config.ICFilter=0;                           //配置输入滤波器,不滤波
    HAL_TIM_IC_ConfigChannel(&TIM3_Handler,&TIM3_CH1Config,TIM_CHANNEL_1); //配置TIM5通道1
    HAL_TIM_IC_Start_IT(&TIM3_Handler,TIM_CHANNEL_1);    //开始捕获TIM5的通道1
    __HAL_TIM_ENABLE_IT(&TIM3_Handler,TIM_IT_UPDATE);    //使能更新中断

    /* 初始化消息队列 */
    rt_mq_init(&ppm_Queue, "ppm_Queue",
               &msg_pool3[0],        /*内存池指向msg_pool */
               2,                    /*每个消息的大小是 128 - void*/
               sizeof(msg_pool3),    /*内存池的大小是msg_pool的大小*/
               RT_IPC_FLAG_FIFO);    /*如果有多个线程等待,按照FIFO的方法分配消息 */

    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(PPM_IN_Init);
//定时器5底层驱动，时钟使能，引脚配置
//此函数会被HAL_TIM_IC_Init()调用
//htim:定时器5句柄
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_TIM3_CLK_ENABLE();            //使能TIM5时钟
    __HAL_RCC_GPIOC_CLK_ENABLE();           //开启GPIAB时钟

    GPIO_Initure.Pin=GPIO_PIN_6;            //PA0
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用推挽输出
    GPIO_Initure.Pull=GPIO_PULLDOWN;        //下拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;     //高速
    GPIO_Initure.Alternate=GPIO_AF2_TIM3;   //PA0复用为TIM5通道1
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);

    HAL_NVIC_SetPriority(TIM3_IRQn,2,0);    //设置中断优先级，抢占优先级5，子优先级0
    HAL_NVIC_EnableIRQ(TIM3_IRQn);          //开启ITM5中断
}

//定时器5中断服务函数
void TIM3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_TIM_IRQHandler(&TIM3_Handler); //定时器共用处理函数

    /* leave interrupt */
    rt_interrupt_leave();
}

//定时器输入捕获中断处理回调函数,该函数在HAL_TIM_IRQHandler中会被调用
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) //捕获中断发生时执行
{
    static uint16_t temp_cnt1,temp_cnt2;
    if ( GPIOC->IDR & GPIO_PIN_6)
    {
        temp_cnt1 = HAL_TIM_ReadCapturedValue(&TIM3_Handler,TIM_CHANNEL_1);   //获取当前的捕获值.

        TIM_RESET_CAPTUREPOLARITY(&TIM3_Handler,TIM_CHANNEL_1);                      //一定要先清除原来的设置！！
        TIM_SET_CAPTUREPOLARITY(&TIM3_Handler,TIM_CHANNEL_1,TIM_ICPOLARITY_FALLING); //配置TIM5通道1下升沿捕获
    }
    else
    {
        temp_cnt2 = HAL_TIM_ReadCapturedValue(&TIM3_Handler,TIM_CHANNEL_1);     //获取当前的捕获值.
        uint16_t _tmp;
        if ( temp_cnt2 >= temp_cnt1 )
        _tmp = temp_cnt2 - temp_cnt1;
        else
        _tmp = 0xffff - temp_cnt1 + temp_cnt2 + 1;
        rt_mq_send(&ppm_Queue,&_tmp, sizeof(rt_uint16_t));

        TIM_RESET_CAPTUREPOLARITY(&TIM3_Handler,TIM_CHANNEL_1);   //一定要先清除原来的设置！！
        TIM_SET_CAPTUREPOLARITY(&TIM3_Handler,TIM_CHANNEL_1,TIM_ICPOLARITY_RISING); //定时器5通道1设置为上降沿捕获
    }
}

int ppm_get_data(uint16_t *times)
{
    /* 从消息队列中接收消息 */
    return (rt_mq_recv(&ppm_Queue, times, sizeof(uint16_t), 0)== RT_EOK);
}

int get_ppm_Queue_num()
{
    /* 队列中已有的消息数 */
    return ppm_Queue.entry;
}

#endif


#if(FBW_BORAD_TYPES == 2 ) //智茂飞控， 采用STM32F405芯片
/************************************************************************************
 *    遥控器ppm信号输入捕获gpio初始化
***************************************************************************************/
/* 消息队列控制块 */
static struct rt_messagequeue ppm_Queue;
static rt_uint16_t  msg_pool3[1024];
TIM_HandleTypeDef TIM1_Handler;         //定时器5句柄
int PPM_IN_Init(void)
{
    TIM_IC_InitTypeDef TIM1_CH4Config;

    TIM1_Handler.Instance=TIM1;                          //通用定时器1
    TIM1_Handler.Init.Prescaler=168-1;                   //分频      TIM1的时钟频率为168M
    TIM1_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    TIM1_Handler.Init.Period=0XFFFFFFFF;                 //自动装载值
    TIM1_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_IC_Init(&TIM1_Handler);

    TIM1_CH4Config.ICPolarity=TIM_ICPOLARITY_RISING;     //上升沿捕获
    TIM1_CH4Config.ICSelection=TIM_ICSELECTION_DIRECTTI; //映射到TI1上
    TIM1_CH4Config.ICPrescaler=TIM_ICPSC_DIV1;           //配置输入分频,不分频
    TIM1_CH4Config.ICFilter=0;                           //配置输入滤波器,不滤波
    HAL_TIM_IC_ConfigChannel(&TIM1_Handler,&TIM1_CH4Config,TIM_CHANNEL_4); //配置TIM1通道4
    HAL_TIM_IC_Start_IT(&TIM1_Handler,TIM_CHANNEL_4);    //开始捕获TIM1的通道4
    __HAL_TIM_ENABLE_IT(&TIM1_Handler,TIM_IT_UPDATE);    //使能更新中断
    __HAL_TIM_ENABLE_IT(&TIM1_Handler,TIM_IT_CC4);    //使能更新中断

    /* 初始化消息队列 */
    rt_mq_init(&ppm_Queue, "ppm_Queue",
               &msg_pool3[0],        /*内存池指向msg_pool */
               2,                    /*每个消息的大小是2字节 16位*/
               sizeof(msg_pool3),    /*内存池的大小是msg_pool的大小*/
               RT_IPC_FLAG_FIFO);    /*如果有多个线程等待,按照FIFO的方法分配消息 */

    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(PPM_IN_Init);
//定时器5底层驱动，时钟使能，引脚配置
//此函数会被HAL_TIM_IC_Init()调用
//htim:定时器5句柄
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_TIM1_CLK_ENABLE();            //使能TIM1时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();           //开启GPIOA时钟

    GPIO_Initure.Pin=GPIO_PIN_11;            //PA0
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用
    GPIO_Initure.Pull=GPIO_PULLDOWN;        //下拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;     //高速
    GPIO_Initure.Alternate=GPIO_AF1_TIM1;   //PA0复用为TIM5通道1
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);

    HAL_NVIC_SetPriority(TIM1_CC_IRQn,2,0);    //设置中断优先级，抢占优先级5，子优先级0
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);          //开启ITM1中断
}

//定时器5中断服务函数
void TIM1_CC_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_TIM_IRQHandler(&TIM1_Handler); //定时器共用处理函数

    /* leave interrupt */
    rt_interrupt_leave();
}

//定时器输入捕获中断处理回调函数,该函数在HAL_TIM_IRQHandler中会被调用
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) //捕获中断发生时执行
{
    static uint16_t temp_cnt1,temp_cnt2;
    if ( GPIOA->IDR & GPIO_PIN_11)
    {
        temp_cnt1 = HAL_TIM_ReadCapturedValue(&TIM1_Handler,TIM_CHANNEL_4);   //获取当前的捕获值.

        TIM_RESET_CAPTUREPOLARITY(&TIM1_Handler,TIM_CHANNEL_4);                      //一定要先清除原来的设置！！
        TIM_SET_CAPTUREPOLARITY(&TIM1_Handler,TIM_CHANNEL_4,TIM_ICPOLARITY_FALLING); //配置TIM5通道1下升沿捕获
    }
    else
    {
        temp_cnt2 = HAL_TIM_ReadCapturedValue(&TIM1_Handler,TIM_CHANNEL_4);     //获取当前的捕获值.
        uint16_t _tmp;
        if ( temp_cnt2 >= temp_cnt1 )
        _tmp = temp_cnt2 - temp_cnt1;
        else
        _tmp = 0xffff - temp_cnt1 + temp_cnt2 + 1;
        rt_mq_send(&ppm_Queue,&_tmp, sizeof(rt_uint16_t));

        TIM_RESET_CAPTUREPOLARITY(&TIM1_Handler,TIM_CHANNEL_4);   //一定要先清除原来的设置！！
        TIM_SET_CAPTUREPOLARITY(&TIM1_Handler,TIM_CHANNEL_4,TIM_ICPOLARITY_RISING); //定时器5通道1设置为上降沿捕获
    }
}

int ppm_get_data(uint16_t *times)
{
    /* 从消息队列中接收消息 */
    return (rt_mq_recv(&ppm_Queue, times, sizeof(uint16_t), 0)== RT_EOK);
}

int get_ppm_Queue_num()
{
    /* 队列中已有的消息数 */
    return ppm_Queue.entry;
}

#endif
