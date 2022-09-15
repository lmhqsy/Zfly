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
#include "DL_OpticalFlow_LC306.h"
//Vector2f opt_filter;
//Vector2f gyr_filter;
//Vector2f optflow_gyro_filter;
//static uint8_t isOptFilterInit=0;
//Vector2f opticalflow_rotate_complementary_filter(float dt) //光流角速度与陀螺仪角速度融合
//{
//    //static float opt_alpha=0;
//    static float gyr_alpha=0;
//    if(isOptFilterInit == 0) {
//        isOptFilterInit = 1;
//        //opt_alpha = calc_lowpass_alpha(dt,100);
//        gyr_alpha = calc_lowpass_alpha(dt,3);
//        biquadFilterInitLPF(&flow_LPF[0],1/dt,20);
//        biquadFilterInitLPF(&flow_LPF[1],1/dt,20);
//    }
//
//    Vector3f _gyro_deg = get_imu_sensorsGyro();
//
////    opt_filter.x += opt_alpha*((float)Flow.raw_x - opt_filter.x);
////    opt_filter.y += opt_alpha*((float)Flow.raw_y - opt_filter.y);
//    opt_filter.x = biquadFilterApply(&flow_LPF[1],Flow306.raw_x); //光流角速度rad/s
//    opt_filter.y = biquadFilterApply(&flow_LPF[0],Flow306.raw_y); //光流角速度rad/s
//
//    gyr_filter.x += gyr_alpha*(_gyro_deg.x - gyr_filter.x);
//    gyr_filter.y += gyr_alpha*(_gyro_deg.y - gyr_filter.y);
//
//    optflow_gyro_filter.x = opt_filter.x/200.0f + limit( gyr_filter.y/57.3f,-3,3); //用陀螺仪数据抵消光流在原地晃动的数据
//    optflow_gyro_filter.y = opt_filter.y/200.0f + limit(-gyr_filter.x/57.3f,-3,3); //用陀螺仪数据抵消光流在原地晃动的数据
//
//    return optflow_gyro_filter;  //补偿后的光流数据 rad/s
//}

Vector2f opt_pos;
Vector2f opt_filter;
Vector2f gyr_filter;
Vector2f optflow_gyro_filter;
static uint8_t isOptFilterInit=0;
Vector2f opticalflow_rotate_complementary_filter(float dt) //光流角速度与陀螺仪角速度融合
{
    //static float opt_alpha=0;
    //static float gyr_alpha=0;
    if(isOptFilterInit == 0) {
        isOptFilterInit = 1;
        //opt_alpha = calc_lowpass_alpha(dt,100);
        //gyr_alpha = calc_lowpass_alpha(dt,3);
        biquadFilterInitLPF(&flow_LPF[0],1/dt,20);
        biquadFilterInitLPF(&flow_LPF[1],1/dt,20);
        biquadFilterInitLPF(&gyro_LPF[0],1/dt,3.7);
        biquadFilterInitLPF(&gyro_LPF[1],1/dt,3.7);
    }
    float RotateScale = 260.0f;
    Vector3f _gyro_deg = get_imu_sensorsGyro();

//    opt_filter.x += opt_alpha*((float)Flow.raw_x - opt_filter.x);
//    opt_filter.y += opt_alpha*((float)Flow.raw_y - opt_filter.y);
    opt_filter.x = biquadFilterApply(&flow_LPF[0],Flow.raw_x); //光流角速度rad/s
    opt_filter.y = biquadFilterApply(&flow_LPF[1],Flow.raw_y); //光流角速度rad/s

//    gyr_filter.x += gyr_alpha*(_gyro_deg.x - gyr_filter.x);
//    gyr_filter.y += gyr_alpha*(_gyro_deg.y - gyr_filter.y);
    gyr_filter.x = biquadFilterApply(&gyro_LPF[0],_gyro_deg.x); //角速度rad/s
    gyr_filter.y = biquadFilterApply(&gyro_LPF[1],_gyro_deg.y); //角速度rad/s


    optflow_gyro_filter.x = opt_filter.x/RotateScale + limit( gyr_filter.y/57.3f,-3,3); //用陀螺仪数据抵消光流在原地晃动的数据
    optflow_gyro_filter.y = opt_filter.y/RotateScale + limit(-gyr_filter.x/57.3f,-3,3); //用陀螺仪数据抵消光流在原地晃动的数据


    opt_pos.x += optflow_gyro_filter.x*limit(INS.Position.z, 10, 250)*dt;
    opt_pos.y += optflow_gyro_filter.y*limit(INS.Position.z, 10, 250)*dt;

    return opt_pos;  //补偿后的光流数据 rad/s
}


#include "CL_ROS_Nav.h"
Vector2f t265_filter;
Vector2f gyro_filter;
Vector2f t265_gyro_filter;
static uint8_t isT265FilterInit=0;
Vector2f t265_rotate_complementary_filter(float dt) //双目速度与陀螺仪角速度融合
{
    if(isT265FilterInit == 0) {
        isT265FilterInit = 1;
        biquadFilterInitLPF(&t265_LPF[0],1/dt,25);
        biquadFilterInitLPF(&t265_LPF[1],1/dt,25);
        biquadFilterInitLPF(&t265_gyro_LPF[0],1/dt,5);
        biquadFilterInitLPF(&t265_gyro_LPF[1],1/dt,5);
    }
    float RotateScale = 6.5f;
    Vector3f _gyro_deg = get_imu_sensorsGyro();

//    t265_filter.x = biquadFilterApply(&t265_LPF[0],t265_estimate.vel_x); //
//    t265_filter.y = biquadFilterApply(&t265_LPF[1],t265_estimate.vel_y); //

    t265_filter.x = biquadFilterApply(&t265_LPF[0],-t265_speed_estimate.x*100); //
    t265_filter.y = biquadFilterApply(&t265_LPF[1],t265_speed_estimate.y*100); //

    gyro_filter.x = biquadFilterApply(&t265_gyro_LPF[0],_gyro_deg.x); //角速度rad/s
    gyro_filter.y = biquadFilterApply(&t265_gyro_LPF[1],_gyro_deg.y); //角速度rad/s


    t265_gyro_filter.x = t265_filter.x + limit( gyro_filter.y/57.3f,-3,3)*RotateScale; //用陀螺仪数据抵消光流在原地晃动的数据
    t265_gyro_filter.y = t265_filter.y + limit(-gyro_filter.x/57.3f,-3,3)*RotateScale; //用陀螺仪数据抵消光流在原地晃动的数据

    return t265_gyro_filter;  //补偿后的光流数据 rad/s
}




