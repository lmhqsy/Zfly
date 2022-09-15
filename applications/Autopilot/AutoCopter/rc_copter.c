/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-05     CGY       the first version
 */
//遥控器数据分析
#include "usb_transfer.h"
#include <rtthread.h>
#include "loco_config.h"
#include "CL_AHRS.h"
#include "mode_all.h"
#include "CL_RC_Channel.h"
#include "rc_copter.h"
//enum FlightMode {
//                ACRO =          0,  // manual body-frame angular rate with manual throttle
//                STABILIZE =     1,  // manual airframe angle with manual throttle
//                ALT_HOLD =      2,  // manual airframe angle with automatic throttle
//                POS_HOLD =      3,  //
//                FLOW_HOLD =     4,  // fully automatic waypoint control using mission commands
//                GUIDED =        5,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
//                LOITER =        6,  // automatic horizontal acceleration with automatic throttle
//                RTL =           7,  // automatic return to launching point
//                LAND =          8,  // automatic return to launching point
//                FlightMode_NumMode
//    };
//飞行模式编号如上。使用方法为：例如位置pos0所对应的是飞行模式为1， 三段开关时：低位时的位置为pos0、中位为pos3、高位为pos5
//摇杆位置编号：                                                        pos0  pos1  pos2  pos3  pos4  pos5
uint8_t FlightModeProtocol[6]={1,    2,    3,    2,    5,    4};   //默认的飞行模式

uint16_t get_flight_modes(uint8_t position)//获取摇杆位置的期望模式
{
    static uint16_t mode_name = 0;

    switch(position) {
        case 0:
            mode_name = FlightModeProtocol[0];
        break;
        case 1:
            mode_name = FlightModeProtocol[1];
        break;
        case 2:
            mode_name = FlightModeProtocol[2];
        break;
        case 3:
            mode_name = FlightModeProtocol[3];
        break;
        case 4:
            mode_name = FlightModeProtocol[4];
        break;
        case 5:
            mode_name = FlightModeProtocol[5];
        break;
        default:
        break;
    }
    return mode_name;
}


void mode_switch_changed(int8_t new_pos)   //通过遥控器的拨码开关设定飞行模式
{
    set_flight_mode(get_flight_modes(new_pos),RC_COMMAND); //设定飞行模式
}


void read_mode_switch() //读取遥控器开关位置并设置飞行模式
{
    int8_t position=0;
    if (read_6pos_switch(&position)) {
        //set flight mode and simple mode setting
        mode_switch_changed(position);
    }
}




