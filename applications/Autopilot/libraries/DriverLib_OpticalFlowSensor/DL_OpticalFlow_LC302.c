/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-29     CGY       the first version
 */
#include "DL_OpticalFlow_LC302.h"
#include "CL_DigitalFilter.h"
int32_t last_lc302_update_ms;

bool get_lc302_health()
{
    if(rt_tick_get() - last_lc302_update_ms > 100){
        return  false;
    }
    else  return  true;
}

/****************************************************************************************
*@brief
*@param[in]
*****************************************************************************************/
flow_data  Flow;
void LC302_Flow_Receive(uint8_t data)
{
    static uint8_t data_len = 0;
    static uint8_t state = 0;

    if(state==0&&data==0xFE)    //
    {
        state=1;
    }
    else if(state==1&&data==0x0A)   //
    {
        state=2;
        data_len = 12;
        Flow.dataLen = 0;
    }
    else if(state==2&&data_len>0)
    {
        data_len--;
        Flow.data[Flow.dataLen++]=data;
        if(data_len==0)
        state = 3;
    }
    else if(state==3 && Flow.data[11]==0x55)
    {
        state=0;
        Flow.sonar_timestamp = Flow.data[5]<<8|Flow.data[4];    //  20800us
        Flow.qual = Flow.data[8];
        if (Flow.qual==0xF5) {

                //BORAD_zhi_mao_B1
                Flow.raw_x =  ((int16_t)(Flow.data[1]<<8)|Flow.data[0]);   //
                Flow.raw_y = -((int16_t)(Flow.data[3]<<8)|Flow.data[2]);

//                Flow.raw_x = -((int16_t)(Flow.data[1]<<8)|Flow.data[0]);   //
//                Flow.raw_y =  ((int16_t)(Flow.data[3]<<8)|Flow.data[2]);


            last_lc302_update_ms = rt_tick_get();
        } else {
            Flow.raw_x = 0;   //
            Flow.raw_y = 0;
        }
    }
    else
    {
        state=0;
    }
}







