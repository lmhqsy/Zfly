/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-21     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_HARDWARELIB_BOARD_BOARD_INTERFACE_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_HARDWARELIB_BOARD_BOARD_INTERFACE_H_
#include "loco_config.h"

int get_loop_run_interval_ms(void);

//传感器的SPI驱动名称 一次从01至20     即能容纳20个spi通信的传感器
#define ICM20602_SPI_DEVICE_NAME        "spi01" //ICM20602
#define ICM42605_SPI_DEVICE_NAME        "spi02" //ICM42605
#define BMM150_SPI_DEVICE_NAME          "spi03" //BMM150
#define SPL06_SPI_DEVICE_NAME           "spi04" //SPL06
#define BMP388_SPI_DEVICE_NAME          "spi05" //BMP388
#define ICM20689_SPI_DEVICE_NAME        "spi06" //ICM20689

//flash、屏的SPI驱动名称 一次从21至30   即能容纳10个spi通信的芯片
#define W25QXX_SPI_DEVICE_NAME          "spi21" //W25QXX
#define SD_CARD_SPI_DEVICE_NAME         "spi22" //SD卡
#define DISPLAY_SPI_DEVICE_NAME         "spi23" //显示屏



#define UART1_NAME       "uart1"    /* 串口设备名称 */
#define UART2_NAME       "uart2"    /* 串口设备名称 */
#define UART3_NAME       "uart3"    /* 串口设备名称 */
#define UART4_NAME       "uart4"    /* 串口设备名称 */
#define UART5_NAME       "uart5"    /* 串口设备名称 */
#define UART6_NAME       "uart6"    /* 串口设备名称 */
#define UART7_NAME       "uart7"    /* 串口设备名称 */
#define UART8_NAME       "uart8"    /* 串口设备名称 */


/*------------------------------------------ FBW_BORAD_TYPES CONFIG  ---------------------------------------------------*/
#if(FBW_BORAD_TYPES == 0 ) //locolion_P1飞控， 采用STM32H750芯片
/*-------------------------- PWM CONFIG  --------------------------*/
#define BSP_USING_PWM1
#define BSP_USING_PWM1_CH1
#define BSP_USING_PWM1_CH2
#define BSP_USING_PWM1_CH3
#define BSP_USING_PWM1_CH4
#define PWM_DEV_NAME           "pwm1"  /* PWM设备名称 */
#define MOTOR1_CH    1
#define MOTOR2_CH    2
#define MOTOR3_CH    3
#define MOTOR4_CH    4
/*-------------------------- ADC CONFIG  --------------------------*/
#define POWER_ADC_V     ADC_CHANNEL_0
#define POWER_ADC_I     ADC_CHANNEL_1


/*-------------------------- SPI CONFIG  --------------------------*/
#define BSP_USING_SPI1
#define BSP_USING_SPI2

/*-------------------------- USB CONFIG  --------------------------*/
#define BSP_USING_USBDEVICE

/*-------------------------- IIC CONFIG  --------------------------*/
#define BSP_USING_I2C1
#ifdef  BSP_USING_I2C1
#define BSP_I2C1_SCL_PIN    GET_PIN(B, 6)
#define BSP_I2C1_SDA_PIN    GET_PIN(B, 7)
#endif

#define BSP_USING_I2C4
#ifdef  BSP_USING_I2C4
#define BSP_I2C4_SCL_PIN    GET_PIN(B, 8)
#define BSP_I2C4_SDA_PIN    GET_PIN(B, 9)
#endif

/*-------------------------- UART CONFIG  --------------------------*/
// assume max 8 ports
#define SERIALMANAGER_NUM_PORTS 8   //串口的个数
#define BSP_USING_UART1
#define BSP_UART1_TX_PIN       "PA9"
#define BSP_UART1_RX_PIN       "PA10"

#define BSP_USING_UART2
#define BSP_UART2_TX_PIN       "PD5"
#define BSP_UART2_RX_PIN       "PD6"

#define BSP_USING_UART3
#define BSP_UART3_TX_PIN       "PC10"
#define BSP_UART3_RX_PIN       "PC11"

#define BSP_USING_UART4
#define BSP_UART4_TX_PIN       "PD1"
#define BSP_UART4_RX_PIN       "PD0"

#define BSP_USING_UART5        //PC12 PD2 引脚的发送接收弄反了 ，需要在drv_usart.c文件里修改
#define BSP_UART5_TX_PIN       "PC12"
#define BSP_UART5_RX_PIN       "PD2"



#define BSP_USING_UART6
#define BSP_UART6_TX_PIN       "PC6"
#define BSP_UART6_RX_PIN       "PC7"

#define BSP_USING_UART7
#define BSP_UART7_TX_PIN       "PB4"
#define BSP_UART7_RX_PIN       "PB3"

#define BSP_USING_UART8
#define BSP_UART8_TX_PIN       "PE1"
#define BSP_UART8_RX_PIN       "PE0"

/*-------------------------- RGB CONFIG  --------------------------*/
#define LED1_PIN      GET_PIN(E, 6)   //R
#define LED2_PIN      GET_PIN(E, 5)   //G
#define LED3_PIN      GET_PIN(E, 4)   //B
#define LED_ON                 0x00     //低电平点亮
#define LED_OFF                0x01
#endif

/*------------------------------------------ FBW_BORAD_TYPES CONFIG  ---------------------------------------------------*/
#if(FBW_BORAD_TYPES == 1 )//locolion_P1飞控， 采用STM32F407芯片
#include <stm32f4xx.h>
/*-------------------------- SPI CONFIG  --------------------------*/
#define BSP_USING_PWM1
#define BSP_USING_PWM1_CH1
#define BSP_USING_PWM1_CH2
#define BSP_USING_PWM1_CH3
#define BSP_USING_PWM1_CH4
#define PWM_DEV_NAME           "pwm1"  /* PWM设备名称 */

#define MOTOR1_CH    1
#define MOTOR2_CH    2
#define MOTOR3_CH    3
#define MOTOR4_CH    4

/*-------------------------- ADC CONFIG  --------------------------*/
#define POWER_ADC_V     ADC_CHANNEL_14
#define POWER_ADC_I     ADC_CHANNEL_15

#define BSP_USING_SPI1
#define BSP_USING_SPI2

/*-------------------------- USB CONFIG  --------------------------*/
#define BSP_USING_USBDEVICE

/*-------------------------- IIC CONFIG  --------------------------*/
#define BSP_USING_I2C1
#ifdef BSP_USING_I2C1
#define BSP_I2C1_SCL_PIN    GET_PIN(B, 6)
#define BSP_I2C1_SDA_PIN    GET_PIN(B, 7)
#endif

/*-------------------------- UART CONFIG  --------------------------*/
// assume max 5 ports
#define SERIALMANAGER_NUM_PORTS 5   //串口的个数
#define BSP_USING_UART1
#define BSP_UART1_TX_PIN       "PA9"
#define BSP_UART1_RX_PIN       "PA10"

#define BSP_USING_UART2
#define BSP_UART2_TX_PIN       "PD5"
#define BSP_UART2_RX_PIN       "PD6"

#define BSP_USING_UART3
#define BSP_UART3_TX_PIN       "PB10"
#define BSP_UART3_RX_PIN       "PB11"

#define BSP_USING_UART4
#define BSP_UART4_TX_PIN       "PA0"
#define BSP_UART4_RX_PIN       "PA1"

#define BSP_USING_UART5
#define BSP_UART5_TX_PIN       "PC12"
#define BSP_UART5_RX_PIN       "PD2"

/*-------------------------- RGB CONFIG  --------------------------*/
#define LED1_PIN      GET_PIN(E, 1)
#define LED2_PIN      GET_PIN(E, 2)
#define LED3_PIN      GET_PIN(E, 3)
#define LED_ON                 0x00     //低电平点亮
#define LED_OFF                0x01
#endif



/*------------------------------------------ FBW_BORAD_TYPES CONFIG  ---------------------------------------------------*/
#if(FBW_BORAD_TYPES == 2 ) //智茂飞控， 采用STM32F405芯片

#define BORAD_zhi_mao_B1     1

#define BSP_USING_PWM2
#define BSP_USING_PWM2_CH1
#define BSP_USING_PWM2_CH2
#define BSP_USING_PWM2_CH3
#define BSP_USING_PWM2_CH4
#define PWM_DEV_NAME           "pwm2"  /* PWM设备名称 */

//a版
//#define MOTOR1_CH    3
//#define MOTOR2_CH    1
//#define MOTOR3_CH    2
//#define MOTOR4_CH    4

//b版
#define MOTOR1_CH    1
#define MOTOR2_CH    3
#define MOTOR3_CH    4
#define MOTOR4_CH    2

/*-------------------------- ADC CONFIG  --------------------------*/
#define POWER_ADC_V     ADC_CHANNEL_8
#define POWER_ADC_I     ADC_CHANNEL_1

/*-------------------------- SPI CONFIG  --------------------------*/
#define BSP_USING_SPI3

/*-------------------------- USB CONFIG  --------------------------*/
//#define BSP_USING_USBDEVICE

/*-------------------------- IIC CONFIG  --------------------------*/
#define BSP_USING_I2C1
#ifdef  BSP_USING_I2C1
#define BSP_I2C1_SCL_PIN    GET_PIN(B, 6)
#define BSP_I2C1_SDA_PIN    GET_PIN(B, 7)
#endif

/*-------------------------- UART CONFIG  --------------------------*/
// assume max 5 ports
#define SERIALMANAGER_NUM_PORTS 6   //串口的个数
#define BSP_USING_UART1
#define BSP_UART1_TX_PIN       "PA9"
#define BSP_UART1_RX_PIN       "PA10"

#define BSP_USING_UART2
#define BSP_UART2_TX_PIN       "PD5"
#define BSP_UART2_RX_PIN       "PD6"

#define BSP_USING_UART3
#define BSP_UART3_TX_PIN       "PB10"
#define BSP_UART3_RX_PIN       "PB11"

#define BSP_USING_UART4
#define BSP_UART4_TX_PIN       "PC10"
#define BSP_UART4_RX_PIN       "PC11"

#define BSP_USING_UART5
#define BSP_UART5_TX_PIN       "PC12"
#define BSP_UART5_RX_PIN       "PD2"


#define BSP_USING_UART6
#define BSP_UART6_TX_PIN       "PC6"
#define BSP_UART6_RX_PIN       "PC7"
#define BSP_UART6_RX_USING_DMA

/*-------------------------- RGB CONFIG  --------------------------*/
#define LED1_PIN       GET_PIN(A, 15)
#define LED2_PIN       GET_PIN(D, 9)
#define LED3_PIN       GET_PIN(C, 2)
#define LED_ON                 0x01     //高电平点亮
#define LED_OFF                0x00

#endif


#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_HARDWARELIB_BOARD_BOARD_INTERFACE_H_ */
