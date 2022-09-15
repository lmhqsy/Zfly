/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-01     CGY       the first version
 */

#include "mode_all.h"
#include "loco_config.h"
#include "mode_all.h"
#include "stdbool.h"
#include "math.h"


void position_hold_mode_pid_par_init()
{
    pidInit(&pid[x_velocity],2.3f,0.8f,0.0f,100,100,400,0.005f,true,20.0f);   //
    pidInit(&pid[y_velocity],2.3f,0.8f,0.0f,100,100,400,0.005f,true,20.0f);   //p 4.0     d 0.1
    pidInit(&pid[x_position],0.4f,0.0f,0.0f,50,50,100,0.005f,true,20.0f);     //
    pidInit(&pid[y_position],0.4f,0.0f,0.0f,50,50,100,0.005f,true,20.0f);     //p 4.0     d 0.1

    biquadFilterInitLPF(&pilot_acce_LPF[0],200,50);
    biquadFilterInitLPF(&pilot_acce_LPF[1],200,50);
}


void mode_position_hold_initialization() //初始化飞行参数
{
    pid_reset(&pid[x_velFlow]);
    pid_reset(&pid[y_velFlow]);

    if (!is_active_z_ctrl()) {
        init_posittion_z_controller();
    }
}

void mode_position_hold_run() //定点模式  (飞行员控制)
{
    Vector2f bf_angles;
    //获取飞行员的摇杆期望,单位为角度.
    get_pilot_desired_lean_angles(&bf_angles.x,&bf_angles.y, 0);
    //返回飞行员的偏航摇杆的期望,单位度每秒。
    float target_yaw_rate =  get_pilot_desired_yaw_rate(get_channel_yaw_control_in());



    float roll_in = get_channel_roll_control_in();  //用于判断遥控器摇杆位置
    float pitc_in = get_channel_pitch_control_in(); //用于判断遥控器摇杆位置

    Vector2f flow_angles; //光流定点控制的输出量
    flowhold_flow_to_angle(&flow_angles,(roll_in != 0) || (pitc_in != 0));

    //摇杆量和光流定点控制量结合
    bf_angles.x += flow_angles.x;
    bf_angles.y += flow_angles.y;
    //外环角度控制器,输入飞行员的摇杆期望(单位:角度,偏航为角速度)
    input_euler_angle_roll_pitch_euler_rate_yaw(bf_angles.x,bf_angles.y,target_yaw_rate);

   //高度控制
    //get pilot desired climb rate 得到飞行员爬升的速度期望。
    float target_climb_rate = get_pilot_desired_climb_rate(get_channel_thr_control_in());
    set_pos_target_z_from_climb_rate_cm(target_climb_rate); //发布升降期望速度
    posittion_update_z_controller(); //高度控制器
}





