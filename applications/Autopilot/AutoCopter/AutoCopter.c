/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-27     CGY       the first version
 */
#include "AutoCopter.h"
#include "loco_config.h"
#include "mode_all.h"
#include "rc_copter.h"
/*
 *   Note：本文件中不涉及传感器原始的读取、处理等，主要为获取处理好的传感器数据进行无人机的相关控制。
 */

static rt_thread_t copter_thread1 = RT_NULL;      /* 线程句柄 */
static rt_thread_t copter_thread2 = RT_NULL;      /* 线程句柄 */
static rt_thread_t copter_thread3 = RT_NULL;      /* 线程句柄 */
static rt_thread_t copter_thread4 = RT_NULL;      /* 线程句柄 */
static rt_thread_t copter_thread5 = RT_NULL;      /* 线程句柄 */


static rt_thread_t copter_thread25 = RT_NULL;      /* 线程句柄 */

/***********************************************************
  *@brief  初始化无人机控制相关线程任务。
*************************************************************/
void AutoCopter_thread_task_init(void) //无人机相关任务初始化
{
    //创建姿态角速度控制任务,以无人机的角速度控制为核心。
    copter_thread1 = rt_thread_create("attitude_rate_control_task",attitude_rate_control_task, RT_NULL, 1024,11,20);//优先级11
    if(copter_thread1 != RT_NULL)
        rt_thread_startup(copter_thread1);

    //创建飞行模式控制任务,以无人机的角度\位置\高度等控制为核心。
    copter_thread2 = rt_thread_create("flight_mode_task",flight_mode_task, RT_NULL, 1024,12,20);//优先级12
    if(copter_thread2 != RT_NULL)
        rt_thread_startup(copter_thread2);

    //创建电池和罗盘任务,以电池容量测量及罗盘数据补偿为核心。
    copter_thread3 = rt_thread_create("update_batt_compass_task",update_batt_compass_task, RT_NULL, 1024,15,20);//优先级15
    if(copter_thread3 != RT_NULL)
        rt_thread_startup(copter_thread3);

    //创建飞行数据记录任务,以记录无人机飞行模式\姿态\位置等数据为核心。
    copter_thread4 = rt_thread_create("logging_task",logging_task, RT_NULL, 1024,16,20);//优先级16
    if(copter_thread4 != RT_NULL)
        rt_thread_startup(copter_thread4);

    //创建参数储存任务,以储存pid、陀螺仪和磁力计校准等参数为核心。
    copter_thread5 = rt_thread_create("parameter_storage_task",parameter_storage_task, RT_NULL, 1024,50,20);//优先级50
    if(copter_thread5 != RT_NULL)
        rt_thread_startup(copter_thread5);

    //创建用户任务,以用户需求为核心。
    copter_thread25 = rt_thread_create("userhook_task1",userhook_task1, RT_NULL, 1024,60,20);//优先级60
    if(copter_thread25 != RT_NULL)
        rt_thread_startup(copter_thread25);
}

/***********************************************************
  *@brief  姿态角速度控制任务，以无人机的角速度控制为核心。
*************************************************************/
void attitude_rate_control_task(void* parameter)  //控制姿态
{
    stabilize_mode_pid_par_init();
    while(1)
    {
        rt_thread_mdelay(2); //500Hz
        attitude_rate_pid_controller_run();           //无人机角速度内环pid控制。

        int d = get_loop_run_interval_ms();
        if (d!=2) rt_kprintf("a:%d",(rt_uint16_t)d);
        //rt_kprintf("dt: %d\r\n",(rt_uint16_t)get_loop_run_interval_ms());
    }
}

/***********************************************************
  *@brief  飞行模式控制任务，以无人机的角度、位置、高度等控制为核心。
*************************************************************/
void flight_mode_task(void* parameter)//线程
{
    altitude_hold_mode_pid_par_init();  //高度控制的pid参数初始化
    position_hold_mode_pid_par_init();  //定点位置控制的pid参数初始化
    flow_hold_mode_pid_par_init();
    while(1)
    {
        rt_thread_mdelay(5); //200Hz

        read_mode_switch();

        update_flight_mode(get_new_mode());
    }
}

/***********************************************************
  *@brief 电池和罗盘任务，以电池容量测量及罗盘数据补偿为核心。
*************************************************************/
void update_batt_compass_task(void* parameter)//线程2
{

    while(1)
    {
        rt_thread_mdelay(1);
    }
}

/***********************************************************
  *@brief  飞行数据记录任务，以记录无人机飞行模式、姿态、位置等数据为核心。
*************************************************************/
void logging_task(void* parameter)//线程2
{
    while(1)
    {
        rt_thread_mdelay(10);
    }
}

/***********************************************************
  *@brief  参数储存任务，以储存pid、陀螺仪和磁力计校准等参数为核心。
*************************************************************/
void parameter_storage_task(void* parameter)//线程
{
    while(1)
    {
        rt_thread_mdelay(10);
    }
}

/***********************************************************
  *@brief 创建用户任务，以用户需求为核心。
*************************************************************/
void userhook_task1(void* parameter)//线程2
{
    while(1)
    {
        rt_thread_mdelay(10);
    }
}

