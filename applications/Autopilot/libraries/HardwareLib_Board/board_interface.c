/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-21     CGY       the first version
 */
#include "loco_config.h"
#include "board_interface.h"
#include <rtthread.h>
#include <board.h>
#include <drv_common.h>
#include "drv_spi.h"

int get_loop_run_interval_ms(void)
{
    static int last_time= 0;
    int now_time = rt_tick_get();
    uint8_t dt_ms = (now_time-last_time);
    last_time = now_time;

    return dt_ms;
}

/*------------------------------------------ FBW_BORAD_TYPES CONFIG  ---------------------------------------------------*/
#if(FBW_BORAD_TYPES == 0 ) //locolion_P1飞控， 采用STM32H750芯片
#include "stm32h7xx_hal.h"
void HAL_PCD_MspInit(PCD_HandleTypeDef * hpcd)
{
    //rtt的usb虚拟串口默认启用了RTS和DTR（终端准备好接收）：串口助手需要勾选RTS和 DTR，才能接收发送数据
    GPIO_InitTypeDef GPIO_InitStruct;
    if(hpcd->Instance == USB2_OTG_FS)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();       //使能GPIOA时钟
        //配置PA11和PA12
        GPIO_InitStruct.Pin=GPIO_PIN_11|GPIO_PIN_12;
        GPIO_InitStruct.Mode=GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull=GPIO_NOPULL;
        GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        HAL_PWREx_EnableUSBVoltageDetector();   //必须使能
        __HAL_RCC_USB2_OTG_FS_CLK_ENABLE();     //使能OTG FS时钟
        //设置中断优先级在drv_usbd.c文件中
        //HAL_NVIC_SetPriority(OTG_FS_IRQn,7,0);  //设置中断优先级
        //HAL_NVIC_EnableIRQ(OTG_FS_IRQn);        //使能OTG FS中断
    }
    else if (hpcd->Instance == USB1_OTG_HS)
    {
        //USB1 OTG本例程没用到,故不做处理
    }
}

//ADC底层驱动，引脚配置，时钟使能
//此函数会被HAL_ADC_Init()调用
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_ADC3_CLK_ENABLE();                 //使能ADC3时钟
    __HAL_RCC_GPIOC_CLK_ENABLE();                //开启GPIOC时钟
    __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_CLKP); //ADC外设时钟选择

    GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_3;      //PC2 PC3
    GPIO_Initure.Mode=GPIO_MODE_ANALOG;          //模拟
    GPIO_Initure.Pull=GPIO_NOPULL;               //不带上下拉
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);
}

/*SPI引脚初始化*/
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(hspi->Instance == SPI1)     //与板载flash、SD卡、气压计、显示屏，通信。
    {
        /* Peripheral clock enable */
        __HAL_RCC_SPI1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /*SPI1 GPIO Configuration
        PA5     ------> SPI1_SCK
        PA6     ------> SPI1_MISO
        PA7     ------> SPI1_MOSI
        */
        GPIO_InitStruct.Pin = GPIO_PIN_5| GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    if(hspi->Instance == SPI2)//与imu模块通信
    {
        /* Peripheral clock enable */
        __HAL_RCC_SPI2_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /*SPI2 GPIO Configuration
        PB13     ------> SPI2_SCK
        PB14     ------> SPI2_MISO
        PB15     ------> SPI2_MOSI
        */
        GPIO_InitStruct.Pin = GPIO_PIN_13| GPIO_PIN_14 | GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

int spi_device_attach(void)
{
    /*******传感器片选引脚*********/
    rt_pin_mode(GET_PIN(A,2), PIN_MODE_OUTPUT); //PA2,显示屏的片选
    rt_pin_mode(GET_PIN(A,3), PIN_MODE_OUTPUT); //PA3,W25QXX的片选
    rt_pin_mode(GET_PIN(A,4), PIN_MODE_OUTPUT); //PA4,SD卡的片选
    rt_pin_mode(GET_PIN(C,4), PIN_MODE_OUTPUT); //PC4,BMP388的片选

    rt_pin_write(GET_PIN(A,2), PIN_HIGH);
    rt_pin_write(GET_PIN(A,3), PIN_HIGH);
    rt_pin_write(GET_PIN(A,4), PIN_HIGH);
    rt_pin_write(GET_PIN(C,4), PIN_HIGH);

    rt_pin_mode(GET_PIN(C,0), PIN_MODE_OUTPUT); //PC0,屏幕D/C
    rt_pin_mode(GET_PIN(C,1), PIN_MODE_OUTPUT); //PC1,屏幕背光
    rt_pin_write(GET_PIN(C,0), PIN_HIGH);
    rt_pin_write(GET_PIN(C,1), PIN_HIGH);


    /*******传感器片选引脚(IMU)*********/
    rt_pin_mode(GET_PIN(B,12), PIN_MODE_OUTPUT);   //PB12，ICM20602的片选
    rt_pin_mode(GET_PIN(D,8),  PIN_MODE_OUTPUT);   //PD8,ICM42605的片选
    rt_pin_mode(GET_PIN(D,9),  PIN_MODE_OUTPUT);   //PD9,SPL06的片选
    rt_pin_mode(GET_PIN(D,10), PIN_MODE_OUTPUT);   //PD10,BMM150的片选

    rt_pin_write(GET_PIN(B,12), PIN_HIGH);
    rt_pin_write(GET_PIN(D,8),  PIN_HIGH);
    rt_pin_write(GET_PIN(D,9),  PIN_HIGH);
    rt_pin_write(GET_PIN(D,10), PIN_HIGH);

    rt_hw_spi_device_attach("spi1",W25QXX_SPI_DEVICE_NAME, GPIOA,GPIO_PIN_3);  //片选引脚PA3
    rt_hw_spi_device_attach("spi1",SD_CARD_SPI_DEVICE_NAME, GPIOA,GPIO_PIN_4); //片选引脚PA4
    rt_hw_spi_device_attach("spi1",DISPLAY_SPI_DEVICE_NAME, GPIOA,GPIO_PIN_2); //片选引脚PA2

    //rt_hw_spi_device_attach("spi2",ICM20602_SPI_DEVICE_NAME, GPIOB,GPIO_PIN_12); //片选引脚PB12
    rt_hw_spi_device_attach("spi2",ICM42605_SPI_DEVICE_NAME, GPIOD,GPIO_PIN_8);  //片选引脚PD8
    rt_hw_spi_device_attach("spi2",BMM150_SPI_DEVICE_NAME, GPIOD,GPIO_PIN_10);   //片选引脚PD10
    rt_hw_spi_device_attach("spi2",SPL06_SPI_DEVICE_NAME, GPIOD,GPIO_PIN_9);     //片选引脚PD9
    rt_hw_spi_device_attach("spi1",BMP388_SPI_DEVICE_NAME, GPIOC,GPIO_PIN_4);    //片选引脚PC4

    return RT_EOK;
}
INIT_DEVICE_EXPORT(spi_device_attach);

/* 使用 STM32CubeMx 生成的*/

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{
  if(htim_pwm->Instance==TIM1)
  {
    __HAL_RCC_TIM1_CLK_ENABLE();
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM1)
  {
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  }
}

int mpu_init(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct;

    /* Disable the MPU */
    HAL_MPU_Disable();

    /* Configure the MPU attributes as WT for AXI SRAM */
    MPU_InitStruct.Enable            = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress       = 0x20000000;
    MPU_InitStruct.Size              = MPU_REGION_SIZE_512KB;
    MPU_InitStruct.AccessPermission  = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable      = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable       = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsShareable       = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number            = MPU_REGION_NUMBER0;
    MPU_InitStruct.TypeExtField      = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable  = 0X00;
    MPU_InitStruct.DisableExec       = MPU_INSTRUCTION_ACCESS_ENABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

#ifdef BSP_USING_SDRAM
    /* Configure the MPU attributes as WT for SDRAM */
    MPU_InitStruct.Enable            = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress       = 0xC0000000;
    MPU_InitStruct.Size              = MPU_REGION_SIZE_32MB;
    MPU_InitStruct.AccessPermission  = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable      = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable       = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsShareable       = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number            = MPU_REGION_NUMBER1;
    MPU_InitStruct.TypeExtField      = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable  = 0x00;
    MPU_InitStruct.DisableExec       = MPU_INSTRUCTION_ACCESS_ENABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
#endif

#ifdef BSP_USING_ETH
    /* Configure the MPU attributes as Device not cacheable
       for ETH DMA descriptors */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = 0x30040000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER2;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /* Configure the MPU attributes as Cacheable write through
       for LwIP RAM heap which contains the Tx buffers */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = 0x30044000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER3;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
#endif

    /* Configure the MPU attributes as WT for QSPI */
    MPU_InitStruct.Enable            = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress       = 0x90000000;
    MPU_InitStruct.Size              = MPU_REGION_SIZE_8MB;
    MPU_InitStruct.AccessPermission  = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable      = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable       = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsShareable       = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number            = MPU_REGION_NUMBER4;
    MPU_InitStruct.TypeExtField      = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable  = 0X00;
    MPU_InitStruct.DisableExec       = MPU_INSTRUCTION_ACCESS_ENABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /* Enable the MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

    /* Enable CACHE */
    SCB_EnableICache();
    SCB_EnableDCache();
    SCB->CACR|=1<<2;   //强制D-Cache透写,如不开启,实际使用中可能遇到各种问题
    return RT_EOK;

}
INIT_BOARD_EXPORT(mpu_init);
#endif



/*------------------------------------------ FBW_BORAD_TYPES CONFIG  ---------------------------------------------------*/
#if(FBW_BORAD_TYPES == 1 ) //locolion_A1飞控， 采用STM32F407芯片
#include <stm32f4xx.h>
void HAL_PCD_MspInit(PCD_HandleTypeDef* hpcd)//USB硬件引脚、时钟初始化
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(hpcd->Instance==USB_OTG_FS)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**USB_OTG_FS GPIO Configuration
        PA11     ------> USB_OTG_FS_DM
        PA12     ------> USB_OTG_FS_DP
         */
        GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        /* Peripheral clock enable */
        __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
        HAL_NVIC_SetPriority(OTG_FS_IRQn, 7, 0);
        HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
        /* USER CODE BEGIN USB_OTG_FS_MspInit 1 */
    }
}
//ADC底层驱动，引脚配置，时钟使能
//此函数会被HAL_ADC_Init()调用
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_ADC3_CLK_ENABLE();                 //使能ADC3时钟
    __HAL_RCC_GPIOC_CLK_ENABLE();                //开启GPIOC时钟
    __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_CLKP); //ADC外设时钟选择

    GPIO_Initure.Pin=GPIO_PIN_4|GPIO_PIN_5;      //PC2 PC3
    GPIO_Initure.Mode=GPIO_MODE_ANALOG;          //模拟
    GPIO_Initure.Pull=GPIO_NOPULL;               //不带上下拉
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{
  if(htim_pwm->Instance==TIM1)
  {
    __HAL_RCC_TIM1_CLK_ENABLE();
  }
  if(htim_pwm->Instance==TIM4)
  {
    __HAL_RCC_TIM4_CLK_ENABLE();
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim) //pwm输出引脚初始化
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM1)
  {
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  }
  if(htim->Instance==TIM4)
  {
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  }


}


/*SPI引脚初始化*/
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(hspi->Instance == SPI1)
    {
        /* Peripheral clock enable */
        __HAL_RCC_SPI1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**SPI1 GPIO Configuration
        PA5     ------> SPI3_MISO
        PA6     ------> SPI3_SCK
        PA7     ------> SPI3_MOSI
        */
        GPIO_InitStruct.Pin = GPIO_PIN_5| GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    if(hspi->Instance == SPI2)
    {
        /* Peripheral clock enable */
        __HAL_RCC_SPI2_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**SPI2 GPIO Configuration
        PB13     ------> SPI3_MISO
        PB14     ------> SPI3_SCK
        PB15     ------> SPI3_MOSI
        */
        GPIO_InitStruct.Pin = GPIO_PIN_13| GPIO_PIN_14 | GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

int spi_device_attach(void)
{
    /*******传感器片选引脚*********/
    rt_pin_mode(GET_PIN(D,8), PIN_MODE_OUTPUT); //PD8
    rt_pin_mode(GET_PIN(D,9), PIN_MODE_OUTPUT); //PD9
    rt_pin_mode(58, PIN_MODE_OUTPUT); //PD10
    rt_pin_mode(33, PIN_MODE_OUTPUT); //PC1

    rt_pin_write(56, PIN_HIGH);
    rt_pin_write(57, PIN_HIGH);
    rt_pin_write(58, PIN_HIGH);
    rt_pin_write(33, PIN_HIGH);

    rt_hw_spi_device_attach("spi2",ICM20602_SPI_DEVICE_NAME, GPIOD, GPIO_PIN_8);       //片选引脚PD8
//    rt_hw_spi_device_attach("spi2",SPL06_SPI_DEVICE_NAME, GPIOD,GPIO_PIN_9);         //片选引脚PD10
//    rt_hw_spi_device_attach("spi2","spi22", GPIOD,GPIO_PIN_9);//片选引脚PD9
//    rt_hw_spi_device_attach("spi2","spi23", GPIOC,GPIO_PIN_1);//片选引脚PC1
    return RT_EOK;
}
INIT_DEVICE_EXPORT(spi_device_attach);
#endif
/*------------------------------------------ FBW_BORAD_TYPES CONFIG  ---------------------------------------------------*/
#if(FBW_BORAD_TYPES == 2 ) //智茂飞控， 采用STM32F405芯片
#include <stm32f4xx.h>
//void HAL_PCD_MspInit(PCD_HandleTypeDef* hpcd)//USB硬件引脚、时钟初始化
//{
//    GPIO_InitTypeDef GPIO_InitStruct = {0};
//    if(hpcd->Instance==USB_OTG_FS)
//    {
//        __HAL_RCC_GPIOA_CLK_ENABLE();
//        /**USB_OTG_FS GPIO Configuration
//        PA11     ------> USB_OTG_FS_DM
//        PA12     ------> USB_OTG_FS_DP
//         */
//        GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
//        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//        GPIO_InitStruct.Pull = GPIO_NOPULL;
//        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//        GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
//        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//        /* Peripheral clock enable */
//        __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
//        HAL_NVIC_SetPriority(OTG_FS_IRQn, 7, 0);
//        HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
//        /* USER CODE BEGIN USB_OTG_FS_MspInit 1 */
//    }
//}

//ADC底层驱动，引脚配置，时钟使能
//此函数会被HAL_ADC_Init()调用
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_ADC1_CLK_ENABLE();            //使能ADC3时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();           //开启GPIOA时钟

    GPIO_Initure.Pin=GPIO_PIN_0;                 //PB0
    GPIO_Initure.Mode=GPIO_MODE_ANALOG;          //模拟
    GPIO_Initure.Pull=GPIO_NOPULL;               //不带上下拉
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{
  if(htim_pwm->Instance==TIM2)
  {
    __HAL_RCC_TIM2_CLK_ENABLE();
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim) //pwm输出引脚初始化
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM2)
  {
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

/*SPI引脚初始化*/
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(hspi->Instance == SPI3)
    {
        /* Peripheral clock enable */
        __HAL_RCC_SPI3_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_3| GPIO_PIN_4 | GPIO_PIN_5;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

int spi_device_attach(void)
{
    /*******传感器片选引脚*********/
    rt_pin_mode(GET_PIN(C,8), PIN_MODE_OUTPUT); //PC8

    rt_pin_write(GET_PIN(C,8), PIN_HIGH);

    rt_hw_spi_device_attach("spi3",ICM20689_SPI_DEVICE_NAME, GPIOC, GPIO_PIN_8);       //片选引脚PC8

    return RT_EOK;
}
INIT_DEVICE_EXPORT(spi_device_attach);

#endif

