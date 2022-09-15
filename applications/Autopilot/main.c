/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-01-14     RT-Thread    first version
 */

#include <rtthread.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include <board.h>
#include <rtdevice.h>
#include "loco_config.h"
#include <drv_common.h>
#include "CL_AHRS.h"
#include "CL_RC_Channel.h"
#include "CL_NavigationKF.h"
#include "DL_T265.h"
#include "CL_Notify.h"

/*
 *   Note：本文件中的线程专门用于传感器原始数据读取、处理、姿态解算、EKF位置估计等相关算法的实现，为核心线程。
 *   Note：目的是为AutoCopter.c中的无人机控制任务提供姿态、位置、速度、遥控器等处理过的实时数据。
 *   Note：是无人机或无人车或无人船或固定翼等相关控制的实时数处理，为控制提供可靠的反馈数据。
 */

static rt_thread_t core_thread0 = RT_NULL;      /* 线程句柄 */
static rt_thread_t core_thread1 = RT_NULL;      /* 线程句柄 */
static rt_thread_t core_thread2 = RT_NULL;      /* 线程句柄 */
static rt_thread_t core_thread3 = RT_NULL;      /* 线程句柄 */
static rt_thread_t core_thread4 = RT_NULL;      /* 线程句柄 */
static rt_thread_t core_thread5 = RT_NULL;      /* 线程句柄 */
static rt_thread_t core_thread6 = RT_NULL;      /* 线程句柄 */
static rt_thread_t core_thread7 = RT_NULL;      /* 线程句柄 */
static rt_thread_t core_thread8 = RT_NULL;      /* 线程句柄 */
static rt_thread_t core_thread254 = RT_NULL;    /* 线程句柄 */




void imu_sensors_task(void* parameter);//线程
void radio_data_process_task(void* parameter);//线程
void usb_dataTx_Task(void* parameter);//线程
void tof_uart_dataRx_Task(void* parameter);//线程
void opt_uart_dataRx_Task(void* parameter);//线程
void opt2_uart_dataRx_Task(void* parameter);//线程
void t265_uart_dataRx_Task(void* parameter);//线程
void ks103_uart_dataRx_Task(void* parameter);//线程
void ROS_uart_dataRx_Task(void* parameter);//线程
void rgb_state_task(void* parameter);//线程

int main(void)
{
    //imu传感器数据处理任务以及姿态结算（十轴）任务
    core_thread0 = rt_thread_create("imu_sensors_task",imu_sensors_task, RT_NULL, 1024,2,20);//优先级2
    if(core_thread0 != RT_NULL)
        rt_thread_startup(core_thread0);
    //遥控器原始数据读取及处理任务
    core_thread3 = rt_thread_create("radio_data_process_task",radio_data_process_task, RT_NULL, 1024,6,20);//优先级15
    if(core_thread3 != RT_NULL)
        rt_thread_startup(core_thread3);
    //usb发送传感器、参数等数据的任务
    core_thread2 = rt_thread_create("usb_dataTx_Task",usb_dataTx_Task, RT_NULL, 1024,100,20);//优先级100
    if(core_thread2 != RT_NULL)
        rt_thread_startup(core_thread2);
    //激光类传感器原始读取及处理任务
    core_thread1 = rt_thread_create("tof_uart_dataRx_Task",tof_uart_dataRx_Task, RT_NULL, 1024,7,20);//优先级30
    if(core_thread1 != RT_NULL)
        rt_thread_startup(core_thread1);
    //光流类传感器原始读取及处理任务
    core_thread4 = rt_thread_create("opt_uart_dataRx_Task",opt_uart_dataRx_Task, RT_NULL, 1024,5,20);//优先级5
    if(core_thread4 != RT_NULL)
        rt_thread_startup(core_thread4);
    //光流类传感器原始读取及处理任务
    core_thread5 = rt_thread_create("opt2_uart_dataRx_Task",opt2_uart_dataRx_Task, RT_NULL, 1024,9,20);//优先级5
    if(core_thread5 != RT_NULL)
        rt_thread_startup(core_thread5);

    core_thread6 = rt_thread_create("t265_uart_dataRx_Task",t265_uart_dataRx_Task, RT_NULL, 1024,17,20);//优先级30
    if(core_thread6 != RT_NULL)
        rt_thread_startup(core_thread6);

    core_thread7 = rt_thread_create("ks103_uart_dataRx_Task",ks103_uart_dataRx_Task, RT_NULL, 1024,18,20);//优先级30
    if(core_thread7 != RT_NULL)
        rt_thread_startup(core_thread7);

    core_thread8 = rt_thread_create("ROS_uart_dataRx_Task",ROS_uart_dataRx_Task, RT_NULL, 1024,19,20);//优先级30
    if(core_thread8 != RT_NULL)
        rt_thread_startup(core_thread8);


    //初始化无人机控制相关线程
    AutoCopter_thread_task_init();
    //rgb状态指示任务
    core_thread254 = rt_thread_create("rgb_state_task",rgb_state_task, RT_NULL, 1024,100,20);//优先级100
    if(core_thread254 != RT_NULL)
        rt_thread_startup(core_thread254);

    while(1)
    {
        rt_thread_mdelay(1000);
        uint8_t major,minor;

        cpu_usage_get(&major,&minor);


        rt_kprintf("CPU USAGE %u.%u %%\n",major,minor);
    }

    return RT_EOK;
}

/***********************************************************
  *@brief imu传感器数据处理任务
*************************************************************/
void imu_sensors_task(void* parameter)  //
{
    state_t state;
    baro_t  baro;
    Vector3f gyro,acce;
    sensorData_t sensorData ;
    static uint32_t tick = 0;
    while(1)
    {
        rt_thread_mdelay(1);

        if(rate_do_excute(500,tick))  //500Hz
        {
            for (uint8_t i = 0; i < (imu.instance); ++i) {
                gyro_data_update(&gyro,i); //角速度数据更新
                acce_data_update(&acce,i); //加速度数据更新
            }
            sensorData.gyro = gyro;
            sensorData.acce = acce;
            Vector3f magn;
            magn.x=(float)raw_magn.x/1090.0f;
            magn.y=(float)raw_magn.y/1090.0f;
            magn.z=(float)raw_magn.z/1090.0f;

            sensorData.magn = magn;
            ahrs_update_attitude(&sensorData,&state,0.002f); //姿态解算
            navigation_accel_update(acce);  //导航加速度更新
        }

        if(rate_do_excute(50,tick))  //50Hz
        {
            Baro_Data_Update(&baro);        //气压计数据更新
            //hmc5883l_get_magn_raw(&raw_magn);
        }

        tick++;
    }
}

/***********************************************************
  *@brief 处理遥控器的原始数据，采用ppm、sbus等信号
*************************************************************/
void radio_data_process_task(void* parameter)  //遥控器数据处理线程
{
    static uint32_t tick = 0;
    while(1)
    {
        rt_thread_mdelay(1);                        //调度
        raw_ppm_data_update();                      //获取原始的遥控器通道值
        if(rate_do_excute(200,tick))                //200Hz
        {
            main_channel_value_update();            //通道值更新
            position_estimation_update(0.005f);     //位置估计更新
        }
        tick++;
    }
}

/***********************************************************
  *@brief  用于和上位机通信，将相关数据通过usb发送到上位机
*************************************************************/
void usb_dataTx_Task(void* parameter)//线程
{
    static uint32_t tick = 0;
    while(1)
    {
        rt_thread_mdelay(1);
        ANO_TC_data_exchange(); //将数据发送至匿名上位机,用于飞控调试

        if(rate_do_excute(15,tick))                //15Hz
        {
            ultra_start_work();
        }
        tick++;
    }
}

/**************************************串口*********************************************/
static struct rt_semaphore tofuart_rx_sem;    /* 用于接收消息的信号量 */
/* 接收数据回调函数 */
static rt_err_t tofuart_input(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断,调用此回调函数,然后发送接收信号量 */
    rt_sem_release(&tofuart_rx_sem);
    return RT_EOK;
}
/***********************************************************
  *@brief  激光数据接收
*************************************************************/
void tof_uart_dataRx_Task(void* parameter)//线程2
{
    char com_data;
    rt_device_t dev = serial_device_find(SerialProtocol_Tof);
    if (dev) {
        /* 初始化信号量 */
        rt_sem_init(&tofuart_rx_sem, "tofuart_rx_sem", 1, RT_IPC_FLAG_FIFO);
        /* 设置接收回调函数 */
        rt_device_set_rx_indicate(dev, tofuart_input);
    }
    while (1)
    {
        if(dev)
        {
            /* 从串口读取一个字节的数据,没有读取到则等待接收信号量 */
            while (rt_device_read(dev,-1, &com_data, 1) != 1)
            {
                /* 阻塞等待接收信号量,等到信号量后再次读取数据 */
                rt_sem_take(&tofuart_rx_sem, RT_WAITING_FOREVER);
            }
            TFmini_Statemachine(com_data); //读取数据及解析数据
        } else {
            rt_thread_mdelay(10);
        }
    }
}
/**************************************串口*********************************************/
static struct rt_semaphore opt_uart_rx_sem;    /* 用于接收消息的信号量 */
/* 接收数据回调函数 */
static rt_err_t opt_uart_input(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断,调用此回调函数,然后发送接收信号量 */
    rt_sem_release(&opt_uart_rx_sem);
    return RT_EOK;
}
/***********************************************************
  *@brief  光流数据接收
*************************************************************/
void opt_uart_dataRx_Task(void* parameter)//线程2
{
    char com_data;
    rt_device_t dev = serial_device_find(SerialProtocol_Opt);
    if (dev) {
        /* 初始化信号量 */
        rt_sem_init(&opt_uart_rx_sem, "opt_uart_rx_sem", 1, RT_IPC_FLAG_FIFO);
        /* 设置接收回调函数 */
        rt_device_set_rx_indicate(dev, opt_uart_input);
        set_serial_device_baud_rate(dev,19200);
        rt_device_open(dev, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    }
    while (1)
    {
        if(dev)
        {
            /* 从串口读取一个字节的数据,没有读取到则等待接收信号量 */
            while (rt_device_read(dev,-1, &com_data, 1) != 1)
            {
                /* 阻塞等待接收信号量,等到信号量后再次读取数据 */
                rt_sem_take(&opt_uart_rx_sem, RT_WAITING_FOREVER);
            }
            LC302_Flow_Receive(com_data); //读取数据及解析数据
        } else {
            rt_thread_mdelay(10);
        }
    }
}

/**************************************串口*********************************************/
static struct rt_semaphore opt2_uart_rx_sem;    /* 用于接收消息的信号量 */
/* 接收数据回调函数 */
static rt_err_t opt2_uart_input(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断,调用此回调函数,然后发送接收信号量 */
    rt_sem_release(&opt2_uart_rx_sem);
    return RT_EOK;
}
/***********************************************************
  *@brief  光流数据接收
*************************************************************/
void opt2_uart_dataRx_Task(void* parameter)//线程2
{
    char com_data;
    rt_device_t dev = serial_device_find(SerialProtocol_Opt2);
    if (dev) {
        /* 初始化信号量 */
        rt_sem_init(&opt2_uart_rx_sem, "opt2_uart_rx_sem", 1, RT_IPC_FLAG_FIFO);
        /* 设置接收回调函数 */
        rt_device_set_rx_indicate(dev, opt2_uart_input);
        set_serial_device_baud_rate(dev,19200);
        rt_device_open(dev, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
        lc306_config_init();
    }
    while (1)
    {
        if(dev)
        {
            /* 从串口读取一个字节的数据,没有读取到则等待接收信号量 */
            while (rt_device_read(dev,-1, &com_data, 1) != 1)
            {
                /* 阻塞等待接收信号量,等到信号量后再次读取数据 */
                rt_sem_take(&opt2_uart_rx_sem, RT_WAITING_FOREVER);
            }
            lc306_flow_receive(com_data);  //摄像头位置朝向机头
        } else {
            rt_thread_mdelay(10);
        }
    }
}

/**************************************串口*********************************************/
//static struct rt_semaphore t265uart_rx_sem;    /* 用于接收消息的信号量 */
///* 接收数据回调函数 */
//static rt_err_t t265uart_input(rt_device_t dev, rt_size_t size)
//{
//    /* 串口接收到数据后产生中断,调用此回调函数,然后发送接收信号量 */
//    rt_sem_release(&t265uart_rx_sem);
//    return RT_EOK;
//}
/***********************************************************
  *@brief  激光数据接收
*************************************************************/
void t265_uart_dataRx_Task(void* parameter)//线程2
{
    static _Vio_Rx T265_data;
    rt_device_t dev = serial_device_find(SerialProtocol_Vision);
    if(dev) set_serial_device_baud_rate(dev,230400);
    while (1)
    {
        if(dev)
        {
            rt_size_t len;
            if (rt_mb_recv(uart6_mb,&len,RT_WAITING_FOREVER) != RT_EOK)
            {
                return;
            }
            T265_data.len = rt_device_read(dev,0,T265_data.buf,len);
            T265_data_deal(T265_data);
        }
        else
        {
            rt_thread_mdelay(10);
        }
    }
}

/**************************************串口*********************************************/
static struct rt_semaphore ks103uart_rx_sem;    /* 用于接收消息的信号量 */
/* 接收数据回调函数 */
static rt_err_t ks103uart_input(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断,调用此回调函数,然后发送接收信号量 */
    rt_sem_release(&ks103uart_rx_sem);
    return RT_EOK;
}
/***********************************************************
  *@brief  激光数据接收
*************************************************************/
void ks103_uart_dataRx_Task(void* parameter)//线程2
{
    char com_data;
    rt_device_t dev = serial_device_find(SerialProtocol_KS103);
    if (dev) {
        set_serial_device_baud_rate(dev,9600);
        /* 初始化信号量 */
        rt_sem_init(&ks103uart_rx_sem, "ks103uart_rx_sem", 1, RT_IPC_FLAG_FIFO);
        /* 设置接收回调函数 */
        rt_device_set_rx_indicate(dev, ks103uart_input);
    }

    ultr.cmd[0]=0xe8;
    ultr.cmd[1]=0x02;
    ultr.cmd[2]=0x73;  //电源降噪
    if(dev) rt_device_write(dev,0,ultr.cmd,3);
    rt_thread_mdelay(100);
    ultr.name = 0x01;

    while (1)
    {
        if(dev)
        {
            /* 从串口读取一个字节的数据,没有读取到则等待接收信号量 */
            while (rt_device_read(dev,-1, &com_data, 1) != 1)
            {
                /* 阻塞等待接收信号量,等到信号量后再次读取数据 */
                rt_sem_take(&ks103uart_rx_sem, RT_WAITING_FOREVER);
            }
            ultrasonic_receive(com_data); //读取数据及解析数据
        } else {
            rt_thread_mdelay(10);
        }
    }
}


/********************************************************************************串口*****************************************************************************************/
static struct rt_semaphore ros_uart_rx_sem;    /* 用于接收消息的信号量 */
/* 接收数据回调函数 */
static rt_err_t ros_uart_input(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断,调用此回调函数,然后发送接收信号量 */
    rt_sem_release(&ros_uart_rx_sem);
    return RT_EOK;
}
#include "mavlink.h"
#include "CL_ROS_Nav.h"
/***********************************************************
  *@brief  光流数据接收
*************************************************************/
void ROS_uart_dataRx_Task(void* parameter)//线程2
{
    char com_data;
    rt_device_t dev = serial_device_find(SerialProtocol_ROS);
    if (dev) {
        /* 初始化信号量 */
        rt_sem_init(&ros_uart_rx_sem, "ros_uart_rx_sem", 1, RT_IPC_FLAG_FIFO);
        /* 设置接收回调函数 */
        rt_device_set_rx_indicate(dev,ros_uart_input);
        set_serial_device_baud_rate(dev,115200);
        rt_device_open(dev, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    }

    uint8_t ret;
    static mavlink_message_t msg;
    static mavlink_status_t  status;
    while (1)
    {
        if(dev)
        {
            /* 从串口读取一个字节的数据,没有读取到则等待接收信号量 */
            while (rt_device_read(dev,-1, &com_data, 1) != 1)
            {
                /* 阻塞等待接收信号量,等到信号量后再次读取数据 */
                rt_sem_take(&ros_uart_rx_sem, RT_WAITING_FOREVER);
            }
            /* 读取到的数据通过串口错位输出 */
            ret = mavlink_parse_char(0, com_data, &msg, &status);

            if(MAVLINK_FRAMING_OK == ret)
            {

                MAVLinkRcv_Handler(msg);                 //接收完一帧数据并处理
            }

//            LC302_Flow_Receive(com_data); //读取数据及解析数据
        } else {
            rt_thread_mdelay(10);
        }
    }
}
/*******************************************************************************************************************************************************************/

/***********************************************************
  *@brief  RGB状态指示任务。
*************************************************************/
void rgb_state_task(void* parameter)  //线程
{
    static uint32_t tick = 0;
    while(1)
    {
        rt_thread_mdelay(1);
        LED_1ms_DRV(); //0~20

        if(rate_do_excute(100,tick))  //50Hz
        {
            led_state_update(10);
        }

        tick++;
    }
}

#if(FBW_BORAD_TYPES == 0 ) //locolion_P1飞控， 采用STM32H750芯片
//从片外flash中启动程序。
static int vtor_config(void)
{
    /* Vector Table Relocation in Internal QSPI_FLASH */
    SCB->VTOR = QSPI_BASE;
    return 0;
}
INIT_BOARD_EXPORT(vtor_config);
#endif


