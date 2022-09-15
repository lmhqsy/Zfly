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

Vector3f path[500];
uint8_t path_num=0;

uint8_t set_num=0;
#define  USE_External_Route     1     //为1时：使用树莓派发送过来的航点期望， 为0时：使用飞控内部设置的航点期望

void mode_guided_initialization() //初始化飞行参数
{
    pid_reset(&pid[x_position]);
    pid_reset(&pid[x_velocity]);
    pid_reset(&pid[y_position]);
    pid_reset(&pid[y_velocity]);
    if (!is_active_z_ctrl()) {
        init_posittion_z_controller();
    }
    path[0] = get_key_fly_position_cm();  //获取一键起飞时记录的位置

    if(get_pos_vel_type()==2) //飞控内部设定航线
    {
       //机头方向是y轴正方向，右手边是x轴正方向
        path[1].x = path[0].x+0;       //x轴 不动
        path[1].y = path[0].y+150;     //y轴 向前150cm

        path[2].x = path[0].x+150;
        path[2].y = path[0].y+150;

        path[3].x = path[0].x+150;
        path[3].y = path[0].y;
        //回到原点
        path[4].x = path[0].x;
        path[4].y = path[0].y;

        //这里以走以一个边长为150cm的正方形为例，走完航点自动降落。
        set_num = 4;  //设定的航点总数
    }
    else
        send_take_off_flag_to_cv();           //给树莓派发送启动航线任务命令


    path_num=0;                               //航点计数复位
    set_waypoint_destination(&path[0],false); //设置航点目的地
}


Vector3f t265_path[10];
void mode_guided_run()//指导飞行模式  (飞行员控制)
{
    if(get_pos_vel_type()==2) //飞控内部设定航线
    {
        if (path_num < set_num) {  //航线尚未走完
            if (is_reached_waypoint_destination()) {     //到达某个端点
                path_num++;
                set_waypoint_destination(&path[path_num],false);   //将下一个航点位置设置为导航控制模块的目标位置
            }
        } else if ((path_num == set_num) && is_reached_waypoint_destination()) {  //航线运行完成,自动进入降落模式
            set_flight_mode(LAND_Mode,UNKNOWN);
        }
    }
    else //树莓派发送过来的航点
    {
        t265_path[0].x = path[0].x+t265_target.pos_x;
        t265_path[0].y = path[0].y+t265_target.pos_y;

        if (!is_t265_landFlag()) {                              //航线尚未走完
            if (is_reached_waypoint_destination()) {            //到达某个端点
                set_waypoint_destination(&t265_path[0],false);  //将下一个航点位置设置为导航控制模块的目标位置
            }
        } else if (is_t265_landFlag()&& is_reached_waypoint_destination()) {
            set_flight_mode(LAND_Mode,UNKNOWN);
        }

        if (!get_t265_health()) {
            set_flight_mode(LAND_Mode,UNKNOWN);
        }
    }

    float target_roll, target_pitch;
    float wp_roll, wp_pitch;
    float roll_in = get_channel_roll_control_in();  //用于判断遥控器摇杆位置
    float pitc_in = get_channel_pitch_control_in(); //用于判断遥控器摇杆位置
    //获取飞行员的摇杆期望,单位为角度.
    get_pilot_desired_lean_angles(&target_roll,&target_pitch, 0);
    //返回飞行员的偏航摇杆的期望,单位度每秒.
    float target_yaw_rate =  get_pilot_desired_yaw_rate(get_channel_yaw_control_in());

    //得到飞行员爬升的速度期望。 get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(get_channel_thr_control_in());
    set_pos_target_z_from_climb_rate_cm(target_climb_rate);
    //run waypoint controller
    update_waypoint_navigation();
    if ((roll_in != 0) || (pitc_in != 0)) {
        wp_roll  = 0;
        wp_pitch = 0;
    }
    else {
        wp_roll  = get_wp_nav_roll();
        wp_pitch = get_wp_nav_pitch();
    }

    //摇杆量和导航控制量结合
    target_roll  += wp_roll;
    target_pitch += wp_pitch;

    //外环角度控制器,输入飞行员的摇杆期望(单位:角度,偏航为角速度)
    input_euler_angle_roll_pitch_euler_rate_yaw(target_roll,target_pitch,target_yaw_rate);
    posittion_update_z_controller(); //高度控制器
}








