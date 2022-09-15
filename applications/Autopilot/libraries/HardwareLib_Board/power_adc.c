/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-12-25     CGY       the first version
 */
#include "loco_config.h"
#include "stm32f4xx.h"

#define REFER_VOLTAGE       330         /* 参考电压 3.3V,数据精度乘以100保留2位小数*/
#define CONVERT_BITS        (1 << 12)   /* 转换位数为12位 */

ADC_HandleTypeDef ADC1_Handler;//ADC句柄

//初始化ADC
//ch: ADC_channels
//通道值 0~16取值范围为：ADC_CHANNEL_0~ADC_CHANNEL_16
int MY_ADC_Init(void)
{
    ADC1_Handler.Instance=ADC1;
    ADC1_Handler.Init.ClockPrescaler=ADC_CLOCK_SYNC_PCLK_DIV4;   //4分频，ADCCLK=PCLK2/4=90/4=22.5MHZ
    ADC1_Handler.Init.Resolution=ADC_RESOLUTION_12B;             //12位模式
    ADC1_Handler.Init.DataAlign=ADC_DATAALIGN_RIGHT;             //右对齐
    ADC1_Handler.Init.ScanConvMode=DISABLE;                      //非扫描模式
    ADC1_Handler.Init.EOCSelection=DISABLE;                      //关闭EOC中断
    ADC1_Handler.Init.ContinuousConvMode=DISABLE;                //关闭连续转换
    ADC1_Handler.Init.NbrOfConversion=1;                         //1个转换在规则序列中 也就是只转换规则序列1
    ADC1_Handler.Init.DiscontinuousConvMode=DISABLE;             //禁止不连续采样模式
    ADC1_Handler.Init.NbrOfDiscConversion=0;                     //不连续采样通道数为0
    ADC1_Handler.Init.ExternalTrigConv=ADC_SOFTWARE_START;       //软件触发
    ADC1_Handler.Init.ExternalTrigConvEdge=ADC_EXTERNALTRIGCONVEDGE_NONE;//使用软件触发
    ADC1_Handler.Init.DMAContinuousRequests=DISABLE;             //关闭DMA请求
    HAL_ADC_Init(&ADC1_Handler);                                 //初始化
    return RT_EOK;
}

/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(MY_ADC_Init);

//获得ADC值
//ch: 通道值 0~16，取值范围为：ADC_CHANNEL_0~ADC_CHANNEL_16
//返回值:转换结果
uint16_t Get_Adc(uint32_t ch)
{
      ADC_ChannelConfTypeDef ADC1_ChanConf;

      ADC1_ChanConf.Channel=ch;                                   //通道
      ADC1_ChanConf.Rank=1;                                       //第1个序列，序列1
      ADC1_ChanConf.SamplingTime=ADC_SAMPLETIME_480CYCLES;        //采样时间
      ADC1_ChanConf.Offset=0;
      HAL_ADC_ConfigChannel(&ADC1_Handler,&ADC1_ChanConf);        //通道配置

      HAL_ADC_Start(&ADC1_Handler);                               //开启ADC

      HAL_ADC_PollForConversion(&ADC1_Handler,10);                //轮询转换

      return (uint16_t)HAL_ADC_GetValue(&ADC1_Handler);            //返回最近一次ADC1规则组的转换结果
}

//返回值:通道ch的转换结果
float get_adc_average(uint32_t ch)
{
    static float temp_val=0;
    static float temp_val_lpf=0;
    temp_val = (Get_Adc(ch) * REFER_VOLTAGE / CONVERT_BITS)/100.0f;

    temp_val_lpf += 0.5 *(temp_val*4.00f - temp_val_lpf);

    return temp_val_lpf;
}

