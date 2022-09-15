/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-13     CGY       the first version
 */
#include "mode_all.h"
#include "loco_config.h"
#include "stdbool.h"
#include "math.h"

void flow_hold_mode_pid_par_init()
{
    pidInit(&pid[x_velFlow],2.2f,0.8f,0.0f,100,100,400,0.005f,true,20.0f);   //
    pidInit(&pid[y_velFlow],2.2f,0.8f,0.0f,100,100,400,0.005f,true,20.0f);   //p 4.0     d 0.1

    pidInit(&pid[x_posFlow],0.2f,0.0f,0.0f,50,50,100,0.005f,true,20.0f);     //
    pidInit(&pid[y_posFlow],0.2f,0.0f,0.0f,50,50,100,0.005f,true,20.0f);     //p 4.0     d 0.1

    biquadFilterInitLPF(&pilot_acce_LPF[0],200,50);
    biquadFilterInitLPF(&pilot_acce_LPF[1],200,50);
}


//光流位置与速度信息用于位置控制
void flowhold_flow_to_angle(Vector2f *bf_angles, bool stick_input)
{
    static bool braking = false;
    static uint32_t last_stick_input_ms;
    static Vector3f posFlow_target;
    uint32_t now_time = rt_tick_get();

    Vector3f body_acce = get_navigation_body_frame_acce();
    Vector3f current_pos = get_position_neu_cm();
    Vector3f current_vel = get_velocity_neu_cms();
    pid_rate_updata(&pid[x_velFlow],pid[x_posFlow].output, (current_vel.x + 0.00f*body_acce.x));
    pid_rate_updata(&pid[y_velFlow],pid[y_posFlow].output, (current_vel.y + 0.00f*body_acce.y));

    //电机不活跃或遥控器有输入,则清除光流控制量
    if (motor.state != motor_active || current_pos.z<10 || stick_input == true) {
          pid_reset(&pid[x_velFlow]);
          pid_reset(&pid[y_velFlow]);
          pid_reset(&pid[x_posFlow]);
          pid_reset(&pid[y_posFlow]);
          bf_angles->x = 0;
          bf_angles->y = 0;
          posFlow_target.x = current_pos.x;
          posFlow_target.y = current_pos.y;

          braking = true;
          last_stick_input_ms = now_time;
    }
    if (!stick_input && braking) {
        // stop braking if either 3s has passed, or we have slowed below 0.3m/s
        if (now_time - last_stick_input_ms > 3000 ||pythagorous2(current_vel.x,current_vel.y)<=30) {
            braking = false;
            posFlow_target.x = current_pos.x;
            posFlow_target.y = current_pos.y;
        }
    }
    if (!stick_input && !braking) {

        pid_rate_updata(&pid[x_posFlow],posFlow_target.x,current_pos.x);
        pid_rate_updata(&pid[y_posFlow],posFlow_target.y,current_pos.y);

        bf_angles->x = biquadFilterApply(&pilot_acce_LPF[0],limit(-pid[x_velFlow].output * 0.1f,-20,20));
        bf_angles->y = biquadFilterApply(&pilot_acce_LPF[1],limit( pid[y_velFlow].output * 0.1f,-20,20));
    }

    if (!stick_input && braking) {
        // calculate brake angle for each axis separately
        pid_reset_integral(&pid[x_velFlow]);
        pid_reset_integral(&pid[y_velFlow]);
        pid[x_posFlow].output=0;
        pid[y_posFlow].output=0;

        bf_angles->x = biquadFilterApply(&pilot_acce_LPF[0],limit(-pid[x_velFlow].output * 0.1f*1.5f ,-20,20));
        bf_angles->y = biquadFilterApply(&pilot_acce_LPF[1],limit( pid[y_velFlow].output * 0.1f*1.5f ,-20,20));
    }

//    bf_angles->x = limit(atan2_approx(-pid[x_velFlow].output,980)*DEG_PER_RAD,-20,20);
//    bf_angles->y = limit(atan2_approx( pid[y_velFlow].output,980)*DEG_PER_RAD,-20,20);
}


void mode_flow_hold_initialization() //初始化飞行参数
{

    if (get_pos_vel_type()==1) {
        pidInit(&pid[x_velFlow],1.2f,0.8f,0.0f,100,100,400,0.005f,true,20.0f);   //
        pidInit(&pid[y_velFlow],1.2f,0.8f,0.0f,100,100,400,0.005f,true,20.0f);   //p 4.0     d 0.1

        pidInit(&pid[x_posFlow],0.2f,0.0f,0.0f,50,50,100,0.005f,true,20.0f);     //
        pidInit(&pid[y_posFlow],0.2f,0.0f,0.0f,50,50,100,0.005f,true,20.0f);     //p 4.0     d 0.1
    }
    if (get_pos_vel_type()==2) {
        pidInit(&pid[x_velFlow],2.5f,0.8f,0.0f,100,100,400,0.005f,true,20.0f);   //
        pidInit(&pid[y_velFlow],2.5f,0.8f,0.0f,100,100,400,0.005f,true,20.0f);   //p 4.0     d 0.1

        pidInit(&pid[x_posFlow],0.2f,0.0f,0.0f,50,50,100,0.005f,true,20.0f);     //
        pidInit(&pid[y_posFlow],0.2f,0.0f,0.0f,50,50,100,0.005f,true,20.0f);     //p 4.0     d 0.1
    }


    pid_reset(&pid[x_velFlow]);
    pid_reset(&pid[y_velFlow]);

    if (!is_active_z_ctrl()) {
        init_posittion_z_controller();
    }
//    send_cmd_restart_t265();
}

Vector3f _key_fly_pos_cm;
Vector3f get_key_fly_position_cm(void) //返回记录一键起飞的位置
{
    return _key_fly_pos_cm;
}


void mode_flow_hold_run() //定点模式  (飞行员控制)
{
    Vector2f bf_angles;
    //获取飞行员的摇杆期望,单位为角度.
    get_pilot_desired_lean_angles(&bf_angles.x,&bf_angles.y, 0);
    //返回飞行员的偏航摇杆的期望,单位度每秒。
    float target_yaw_rate =  get_pilot_desired_yaw_rate(get_channel_yaw_control_in());

    //get pilot desired climb rate 得到飞行员爬升的速度期望。
    float target_climb_rate = get_pilot_desired_climb_rate(get_channel_thr_control_in());


   static uint16_t one_key_take_off=0,speed=0,init=0;
    if (is_one_button_takeoff()) {
        if (one_key_take_off++>300) {
          if(one_key_take_off==302){
              motor.state = motor_active;
              _key_fly_pos_cm = get_position_neu_cm(); //获取一键起飞时的当前位置
          }
        }
        else motor.state = motor_idle;

        if (absolute(INS.Position.z-100)<10||init==1) {
            speed = 0;
            init = 1;
            //set_flight_mode(GUIDED_Mode,UNKNOWN);
            set_flight_mode(ROS_Nav_Mode,UNKNOWN);
        }
        else  speed = 20;
    }
    else{
        one_key_take_off=0,speed=0,init=0;
        //set_flight_mode(FLOW_HOLD,UNKNOWN);
    }
    target_climb_rate += speed;


    set_pos_target_z_from_climb_rate_cm(target_climb_rate); //发布升降期望速度

    float roll_in = get_channel_roll_control_in();  //用于判断遥控器摇杆位置
    float pitc_in = get_channel_pitch_control_in(); //用于判断遥控器摇杆位置

    Vector2f flow_angles; //光流定点控制的输出量
    flowhold_flow_to_angle(&flow_angles,(roll_in != 0) || (pitc_in != 0));

    //摇杆量和光流定点控制量结合
    bf_angles.x += flow_angles.x;
    bf_angles.y += flow_angles.y;

    //外环角度控制器,输入飞行员的摇杆期望(单位:角度,偏航为角速度)
    input_euler_angle_roll_pitch_euler_rate_yaw(bf_angles.x,bf_angles.y,target_yaw_rate);
    posittion_update_z_controller(); //高度控制器
}

