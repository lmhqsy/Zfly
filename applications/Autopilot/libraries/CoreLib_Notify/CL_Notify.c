/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-12     CGY       the first version
 */
#include "CL_Notify.h"
#include <rtdevice.h>
#include "board_interface.h"
#include "loco_config.h"
#include "mode_all.h"
#include "mavlink.h"


int led_init(void)
{
    /* 设定 LED 引脚为推挽输出模式 */
    rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED2_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED3_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LED1_PIN, LED_OFF);
    rt_pin_write(LED2_PIN, LED_OFF);
    rt_pin_write(LED3_PIN, LED_OFF);

    rt_pin_mode(GET_PIN(A,4), PIN_MODE_OUTPUT);
    rt_pin_write(GET_PIN(A,4), 0);
    return  0;
}
/* 导出到自动初始化 */
INIT_DEVICE_EXPORT(led_init);

//#define LED1_ON       rt_pin_write(LED1_PIN, PIN_HIGH);
//#define LED1_OFF      rt_pin_write(LED1_PIN, PIN_LOW);
//#define LED2_ON       rt_pin_write(LED2_PIN, PIN_HIGH);
//#define LED2_OFF      rt_pin_write(LED2_PIN, PIN_LOW);
//#define LED3_ON       rt_pin_write(LED3_PIN, PIN_HIGH);
//#define LED3_OFF      rt_pin_write(LED3_PIN, PIN_LOW);



uint16_t led_accuracy =20;//该时间相当于控制led的周期  ms
float LED_Brightness[3] = {0,20,0}; //TO 20 //RGB  刚启动，B色灯常亮
void LED_1ms_DRV() //0~20
{
    static uint16_t led_cnt[3];
    for(uint8_t i=0;i<3;i++)
    {
        if(led_cnt[i] < LED_Brightness[i])
        {
            switch(i)
            {
                case 0:
                    LED1_ON;
                break;
                case 1:
                    LED2_ON;
                break;
                case 2:
                    LED3_ON;
                break;
            }
        }
        else
        {
            switch(i)
            {
                case 0:
                    LED1_OFF;
                break;
                case 1:
                    LED2_OFF;
                break;
                case 2:
                    LED3_OFF;
                break;
            }
        }

        if(++led_cnt[i]>=led_accuracy)
        {
            led_cnt[i] = 0;
        }
    }
}
#define safe_div(numerator,denominator,safe_value) ( (denominator == 0)? (safe_value) : ((numerator)/(denominator)) )
//                    调用周期   LED  暗到亮的时间间隔
void ledBreath(uint8_t dT_ms,uint8_t led,uint16_t T)   //LED呼吸函数
{
    static uint8_t dir[3];
    uint8_t i;
    for(i=0; i<3; i++)
    {
        if(led & (1<<i))
        {
            switch(dir[i])
            {
                case 0:
                    LED_Brightness[i] += safe_div(led_accuracy,((float)T/(dT_ms)),0);
                    if(LED_Brightness[i]>led_accuracy)
                    {
                        dir[i] = 1;
                    }

                break;
                case 1:
                    LED_Brightness[i] -= safe_div(led_accuracy,((float)T/(dT_ms)),0);
                    if(LED_Brightness[i]<0)
                    {
                        dir[i] = 0;
                    }

                break;

                default:
                    dir[i] = 0;
                break;
            }
        }
        else
            LED_Brightness[i] = 0;
    }
}

void ledOnOff(uint8_t led) //
{
    uint8_t i;
    for(i=0; i<3; i++)
    {
        if(led & (1<<i))
            LED_Brightness[i] = 20;
        else
            LED_Brightness[i] = 0;
    }
}
                 //  调用周期   LED     亮时间    灭时间
void ledFlash(uint8_t dT_ms,uint8_t led, uint16_t on_ms,uint16_t off_ms)//LED闪烁函数
{
    static uint16_t tim_tmp;
    if(tim_tmp < on_ms)
        ledOnOff(led);
    else
        ledOnOff(0);
    tim_tmp += dT_ms;
    if(tim_tmp >= (on_ms + off_ms))
        tim_tmp = 0;
}
#define BIT_RLED 0x01       //红色
#define BIT_GLED 0x02       //绿色
#define BIT_BLED 0x04       //蓝色


#define BIT_WLED 0x07       //白色
#define BIT_PLED 0x05       //紫色
#define BIT_YLED 0x03       //黄色
#define BIT_CLED 0x06       //青色

void led_state_update(uint8_t dT_ms)
{
    //陀螺仪未校准成功时  白色灯快闪烁
    if(!gyroIsCalibrationComplete(get_gyro_instance())||!acceIsCalibrationComplete(get_gyro_instance()))
    {
      ledFlash(dT_ms,BIT_WLED,40,40);
    }
    //失去遥控器信号时  第一个led呼吸
    else if(!get_rc_health())
    {
      ledBreath(dT_ms,BIT_RLED,300);
    }
    //正在校准电调，三个灯全常亮
    else if(get_esc_calibrating_state())
    {
        ledFlash(dT_ms,BIT_WLED,40,0);
    }
//       //电压低于限定电压时  红色灯快闪烁
//       else if(LED_STA.noRc)
//       {
//          ledFlash(10,BIT_RLED,60,60);
//       }
    else if(!get_t265_health())
    {
        ledBreath(dT_ms,BIT_GLED,300);
    }
    else//无其他提示，正常显示模式档位及外置光流、Gps等状态
    {
        int8_t position=0;
        read_6pos_switch(&position);
        {
            if (position == 0) {
                ledFlash(dT_ms,BIT_RLED,400,400);
            }
            if (position == 3) {
                ledFlash(dT_ms,BIT_GLED,400,400);
            }
            if (position == 5) {
                ledFlash(dT_ms,BIT_BLED,400,400);
            }
        }
    }


    static uint8_t counter = 0;
    static mavlink_message_t mav_msg;
    static uint8_t msg_buf[150];
    static uint8_t msg_size;
    counter++;
    if (counter > 1000) {
        counter = 0;
        if (!get_tof_health()) {
            const char b[]={"tof_no_health"};
            mavlink_msg_statustext_pack_chan(0,1,0,&mav_msg,0,b,1,1);
            msg_size = mavlink_msg_to_send_buffer(msg_buf,&mav_msg);
            send_data(msg_buf,msg_size);
        }
        if (!get_t265_health()) {
            const char b[]={"t265_no_health"};
            mavlink_msg_statustext_pack_chan(0,1,0,&mav_msg,0,b,1,1);
            msg_size = mavlink_msg_to_send_buffer(msg_buf,&mav_msg);
            send_data(msg_buf,msg_size);
        }
        if (!get_lc302_health()) {
            const char b[]={"opt_lc302_no_health"};
            mavlink_msg_statustext_pack_chan(0,1,0,&mav_msg,0,b,1,1);
            msg_size = mavlink_msg_to_send_buffer(msg_buf,&mav_msg);
            send_data(msg_buf,msg_size);
        }
        if (!get_rc_health()) {
            const char b[]={"rc_no_health"};
            mavlink_msg_statustext_pack_chan(0,1,0,&mav_msg,0,b,1,1);
            msg_size = mavlink_msg_to_send_buffer(msg_buf,&mav_msg);
            send_data(msg_buf,msg_size);
        }

    }


    if (get_adc_average(POWER_ADC_V) < 3.7f*3.0f &&get_adc_average(POWER_ADC_V) > 2.0f*3.0f ) {
         rt_pin_write(GET_PIN(A,4), 1);
    }
    else {
         rt_pin_write(GET_PIN(A,4), 0);
    }

    if (get_adc_average(POWER_ADC_V) < 3.4f*3.0f &&get_adc_average(POWER_ADC_V) > 2.0f*3.0f ) {
        set_flight_mode(LAND_Mode,UNKNOWN);  //低于10.2V自动降落
    }

}


/*******************************************************CPU占用率计算***************************************************************/

#include <rtthread.h>
#include <rthw.h>


#define CPU_USAGE_CALC_TICK    1000
#define CPU_USAGE_LOOP        100


static rt_uint8_t  cpu_usage_major = 0, cpu_usage_minor= 0;
static rt_uint32_t total_count = 0;


static void cpu_usage_idle_hook(void)
{
    rt_tick_t tick;
    rt_uint32_t count;
    volatile rt_uint32_t loop;


    if (total_count == 0)
    {
        /* get total count */
        rt_enter_critical();
        tick = rt_tick_get();
        while(rt_tick_get() - tick < CPU_USAGE_CALC_TICK)
        {
            total_count ++;
            loop = 0;


            while (loop < CPU_USAGE_LOOP) loop ++;
        }
        rt_exit_critical();
    }


    count = 0;
    /* get CPU usage */
    tick = rt_tick_get();
    while (rt_tick_get() - tick < CPU_USAGE_CALC_TICK)
    {
        count ++;
        loop  = 0;
        while (loop < CPU_USAGE_LOOP) loop ++;
    }


    /* calculate major and minor */
    if (count < total_count)
    {
        count = total_count - count;
        cpu_usage_major = (count * 100) / total_count;
        cpu_usage_minor = ((count * 100) % total_count) * 100 / total_count;
    }
    else
    {
        total_count = count;


        /* no CPU usage */
        cpu_usage_major = 0;
        cpu_usage_minor = 0;
    }
}


void cpu_usage_get(rt_uint8_t *major, rt_uint8_t *minor)
{
    RT_ASSERT(major != RT_NULL);
    RT_ASSERT(minor != RT_NULL);


    *major = cpu_usage_major;
    *minor = cpu_usage_minor;
}


int cpu_usage_init(void)
{
    /* set idle thread hook */
    rt_thread_idle_sethook(cpu_usage_idle_hook);
    return RT_EOK;
}

/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(cpu_usage_init);

