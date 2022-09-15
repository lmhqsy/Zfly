/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-12-11     CGY       the first version
 */
#include "float.h"
#include "usb_transfer.h"
#include <rtthread.h>
#include "loco_config.h"
#include "CL_AHRS.h"
#include "mode_all.h"
#include "CL_RC_Channel.h"
#include "rc_copter.h"
#include "DL_T265.h"
#include "DL_Compass_bmm150.h"
#define MP_ENABLE              1
#include "mavlink.h"

//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)      ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

#define MYHWADDR    0x05
#define SWJADDR     0xAF
#define PARNUM      100
int debug_flag[5];
float debug_fg[5];
rt_uint16_t paraToSend;
rt_uint32_t Par_List[100];      //参数列表
rt_uint8_t data_to_send[100];   //发送数据缓存
static rt_uint8_t RxBuffer[256],data_cnt=0,data_Receive_ok;

rt_device_t dev1,dev,dev2;
int Console_init (void)
{
    state.arm_power = ArmPowerOFF;
    dev1 = serial_device_find(SerialProtocol_Console);
    if(dev1){
        uint8_t set_baud[]={"AT+BAUD8"};
        set_serial_device_baud_rate(dev1,9600);
        rt_device_open(dev1, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
        send_data(set_baud,sizeof(set_baud));
        set_serial_device_baud_rate(dev1,115200);
        rt_device_open(dev1, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    }

    dev = rt_device_find("vcom");
    if(dev) rt_device_open(dev, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);

    dev2 = serial_device_find(SerialProtocol_ROS);
    if(dev2) rt_device_open(dev2, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    return  0;
}
/* 导出到自动初始化 */
INIT_APP_EXPORT(Console_init);

void send_data(rt_uint8_t *dataToSend,rt_uint8_t length)
{
   if(dev ) rt_device_write(dev , 0, dataToSend,length);
   if(dev1) rt_device_write(dev1, 0, dataToSend,length);
}

void send_data_ros(rt_uint8_t *dataToSend,rt_uint8_t length)
{
   if(dev2) rt_device_write(dev2, 0, dataToSend,length);
}


Vector3i16 raw_magn;

#include "CL_ROS_Nav.h"


void ANO_TC_data_exchange()
{
    static rt_uint32_t count_ms = 1;
#if(MP_ENABLE)

    static mavlink_message_t mav_msg;
    static uint8_t msg_buf[150];
    static uint8_t msg_size;
    if(!(count_ms % 1000))  //1Hz
    {
        mavlink_msg_heartbeat_pack_chan(0,1,0,&mav_msg,MAV_TYPE_QUADROTOR,MAV_AUTOPILOT_ARDUPILOTMEGA,STABILIZE,STABILIZE,1);
        msg_size = mavlink_msg_to_send_buffer(msg_buf,&mav_msg);
        send_data(msg_buf,msg_size);
    }

    if(!(count_ms % 100))
    {
        attitude_t attitude = get_ahrs_eulerAngles();//为四元数姿态解算得到的欧拉角  单位：度
        mavlink_msg_attitude_pack_chan(0,1,0,&mav_msg,0,attitude.roll/57.3,attitude.pitch/57.3,attitude.yaw/57.3,0,0,0);
        msg_size = mavlink_msg_to_send_buffer(msg_buf,&mav_msg);
        send_data(msg_buf,msg_size);

        Vector3f gyro_deg = get_imu_sensorsGyro();   //为原始数据经第一次低通滤波后的角速度数据 deg/s
        Vector3f acce_g =   get_imu_sensorsAcce();   //为原始数据经第一次低通滤波后的加速度数据 cm/ss
        mavlink_msg_scaled_imu_pack(0,1,&mav_msg,0,acce_g.x,acce_g.y,acce_g.z,gyro_deg.x,gyro_deg.y,gyro_deg.z,1,1,1,120);
        msg_size = mavlink_msg_to_send_buffer(msg_buf,&mav_msg);
        send_data(msg_buf,msg_size);
    }

    if(!(count_ms % 50))  //20Hz
    {
        Vector3f current_pos = get_position_neu_cm();
        Vector3f current_vel = get_velocity_neu_cms();
        mavlink_msg_global_position_int_pack(0,1,&mav_msg,0,1000,2000,0,current_pos.z/100*1000,current_vel.x*100,current_vel.y*100,current_vel.z*100,0);
        msg_size = mavlink_msg_to_send_buffer(msg_buf,&mav_msg);
        send_data(msg_buf,msg_size);
    }

    if(!(count_ms % 200))
    {
        mavlink_msg_rc_channels_raw_pack_chan(0,1,0,&mav_msg,0,1,rc_ppm_in[0],rc_ppm_in[1],rc_ppm_in[2],rc_ppm_in[3],rc_ppm_in[4],rc_ppm_in[5],rc_ppm_in[6],rc_ppm_in[7],0);
        msg_size = mavlink_msg_to_send_buffer(msg_buf,&mav_msg);
        send_data(msg_buf,msg_size);
    }





#else
//    if(paraToSend < 0xffff)
//    {
//
//    }
//    /*-------------------------------------------------------------------*
//     *o 对应上位机“飞控状态栏->第三行数据”
//     *-------------------------------------------------------------------*/
    if(!(count_ms % 100))
    {
        attitude_t attitude = get_ahrs_eulerAngles();//为四元数姿态解算得到的欧拉角  单位：度
                       /*i ROL                 PIT         YAW             高度*100              飞行模式i       */
        send_status(attitude.roll,attitude.pitch,imuAttitudeYaw,get_position_z_up_cm()*100,get_new_mode(),40);
    }
//    /*-------------------------------------------------------------------*
//     *o 对应上位机“传感器数据栏->左侧竖列”从上到下依次对应
//     *o 对应数据波形窗口1~9 从最上到下依次对应
//     *-------------------------------------------------------------------*/
//    static Vector3i16 Raw_mag;
//    if(!(count_ms % PERIOD_POWER))
//    {
//        bmm150_get_magn_raw(&Raw_mag);
//    }
//    if(!(count_ms % PERIOD_SENSOR))
//    {
//        Vector3f gyro_deg = get_imu_sensorsGyro();   //为原始数据经第一次低通滤波后的角速度数据 deg/s
//        Vector3f acce_g =   get_imu_sensorsAcce();   //为原始数据经第一次低通滤波后的加速度数据 cm/ss
//
//        Vector3f current_pos = get_position_neu_cm();
//                    /*i x,y,z 轴加速度栏 i*/
//        send_senser(current_pos.x,current_pos.y,acce_g.z,
//                    /*i x,y,z 轴角速度栏 i*/
//                    gyro_deg.x,gyro_deg.y,gyro_deg.z,
//                    /*i x,y,z 轴磁力计栏 i*/
//                    pid[x_position].target,pid[y_position].target,t265_estimate.pos_x);
//    }
//    /*-------------------------------------------------------------------*
//     *o 对应上位机“传感器数据栏->右侧竖列”
//     *-------------------------------------------------------------------*/
    if(!(count_ms % PERIOD_SENSOR2))
    {
       /*i           气压高度                                    测距高度                             温度*10（上位机会除以10） i*/
//       send_senser2(ultr.distance*100,tfdata.distance*100,get_adc_average(POWER_ADC_V)*10);
//        send_senser2(debug_fg[1]*100,pid[z_position].target*100,pid[z_velocity].target*10);
//       Vector3f current_pos = get_position_neu_cm();
//       Vector3f current_vel = get_velocity_neu_cms();
//
//       send_senser2(current_pos.x*100,current_vel.x*100,msg_id*10);
       // send_senser2(t265_target.pos_x*100,t265_target.pos_y*100,t265_estimate.pos_y*10);

       //send_senser2(ros_command.target_system*100,ros_command.command*100,msg_id*10);
       // float target_yaw_rate =  get_pilot_desired_yaw_rate(get_channel_yaw_control_in());
     //send_senser2(t265_position_estimate.x*10000,t265_position_estimate.y*100*100,ros_nav_target.yaw_rate*10);


       //send_senser2(t265_speed_estimate.x*10000,t265_speed_estimate.y*100*100,msg_id*10);
       send_senser2(ros_nav_target.vx*10000,ros_nav_target.vy*100*100,ros_nav_target.yaw_rate*180/3.14*10);

    }
//
//
//    if(!(count_ms % PERIOD_RCDATA))
//    {
//        send_rc_data(rc_ppm_in[2],rc_ppm_in[3],rc_ppm_in[0],rc_ppm_in[1],rc_ppm_in[4],rc_ppm_in[5],rc_ppm_in[6],rc_ppm_in[7],get_lc302_health(),_throttle_out); //未处理过的遥控器数据
//    }

#endif

    if(!(count_ms % 500))  //2Hz
    {
//        static mavlink_message_t mav_msg;
//        static uint8_t msg_buf[150];
//        mavlink_msg_heartbeat_pack_chan(0,1,0,&mav_msg,MAV_TYPE_QUADROTOR,MAV_AUTOPILOT_ARDUPILOTMEGA,STABILIZE,STABILIZE,1);
//        uint8_t size_msg = mavlink_msg_to_send_buffer(msg_buf,&mav_msg);
//        send_data(msg_buf,size_msg);
    }
    if (msg_id == MAVLINK_MSG_ID_COMMAND_LONG) {  //命令消息应答
        static mavlink_message_t mav_msg;
        static uint8_t msg_buf[150];
         mavlink_msg_command_ack_pack_chan(0,0,0,&mav_msg,ros_command.command,0,0,0,ros_command.target_system,ros_command.target_component);
         uint8_t size_msg = mavlink_msg_to_send_buffer(msg_buf,&mav_msg);

         send_data_ros(msg_buf,size_msg);
         //ros_command.command ==400 解锁
         //ros_command.command ==22 起飞
         //ros_command.command ==21 降落
    }






    if(!(count_ms % 100))  //树莓派上电一定时间后,发送启动T265指令
    {
        static uint32_t cv_cpuTick=0;
        if(state.arm_power == ArmPowerOFF && cv_cpuTick>=10)//树莓派上电一定时间后，发送启动T265指令
        {
            send_cmd_start_t265();
            cv_cpuTick=0;
        }
        //树莓派上电后延时一段时间，计时
        else if(state.arm_power == ArmPowerOFF && cv_cpuTick<25)
        {
            cv_cpuTick++;
        }
    }




    count_ms++;
}


void data_receive_prepare(uint8_t data)
{
    static rt_uint8_t _data_len = 0;
    static rt_uint8_t state = 0;
    if(state==0&&data==0xAA)    //帧头0xAA
    {
        state=1;
        RxBuffer[0]=data;
    }
    else if(state==1&&data==0xAF)   //数据源，0xAF表示数据来自上位机
    {
        state=2;
        RxBuffer[1]=data;
    }
    else if(state==2)       //数据目的地
    {
        state=3;
        RxBuffer[2]=data;
    }
    else if(state==3)       //功能字
    {
        state=4;
        RxBuffer[3]=data;
    }
    else if(state==4)       //数据长度
    {
        state = 5;
        RxBuffer[4]=data;
        _data_len = data;
        data_cnt = 0;
    }
    else if(state==5&&_data_len>0)
    {
        _data_len--;
        RxBuffer[5+data_cnt++]=data;
        if(_data_len==0)
                state=6;
    }
    else if(state==6)
    {
        state = 0;
        RxBuffer[5+data_cnt]=data;
        data_Receive_ok = 1;
    }
    else
        state = 0;
}


void send_status(float angle_rol, float angle_pit, float angle_yaw,
                 rt_uint32_t alt, rt_uint8_t fly_model, rt_uint8_t armed)
{
    rt_uint16_t _temp;
    rt_uint32_t _temp2 = alt;
    rt_uint8_t _cnt=0, i=0,sum = 0;

    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=MYHWADDR;
    data_to_send[_cnt++]=SWJADDR;
    data_to_send[_cnt++]=0x01;
    data_to_send[_cnt++]=0;

    _temp = (int)(angle_rol*100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int)(angle_pit*100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int)(angle_yaw*100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    data_to_send[_cnt++]=BYTE3(_temp2);
    data_to_send[_cnt++]=BYTE2(_temp2);
    data_to_send[_cnt++]=BYTE1(_temp2);
    data_to_send[_cnt++]=BYTE0(_temp2);

    data_to_send[_cnt++] = fly_model;

    data_to_send[_cnt++] = armed;

    data_to_send[4] = _cnt-5;

    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];
    data_to_send[_cnt++]=sum;

    send_data(data_to_send,_cnt);
}

void send_senser(rt_int16_t a_x,rt_int16_t a_y,rt_int16_t a_z,
                 rt_int16_t g_x,rt_int16_t g_y,rt_int16_t g_z,
                 rt_int16_t m_x,rt_int16_t m_y,rt_int16_t m_z)
{
    rt_int16_t _temp;
    rt_int8_t _cnt=0, i=0,sum = 0;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=MYHWADDR;
    data_to_send[_cnt++]=SWJADDR;
    data_to_send[_cnt++]=0x02;
    data_to_send[_cnt++]=0;

    _temp = (int)a_x;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int)a_y;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int)a_z;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    _temp = (int)g_x;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int)g_y;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int)g_z;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    _temp = (int)m_x;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int)m_y;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int)m_z;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    data_to_send[4] = _cnt-5;

    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;

    send_data(data_to_send, _cnt);
}

void send_senser2(rt_int32_t bar_alt,rt_int32_t csb_alt, rt_int16_t sensertmp)
{
    rt_uint8_t _cnt=0, i=0, sum=0;

    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=MYHWADDR;
    data_to_send[_cnt++]=SWJADDR;
    data_to_send[_cnt++]=0x07;
    data_to_send[_cnt++]=0;

    data_to_send[_cnt++]=BYTE3(bar_alt);
    data_to_send[_cnt++]=BYTE2(bar_alt);
    data_to_send[_cnt++]=BYTE1(bar_alt);
    data_to_send[_cnt++]=BYTE0(bar_alt);

    data_to_send[_cnt++]=BYTE3(csb_alt);
    data_to_send[_cnt++]=BYTE2(csb_alt);
    data_to_send[_cnt++]=BYTE1(csb_alt);
    data_to_send[_cnt++]=BYTE0(csb_alt);

    data_to_send[_cnt++]=BYTE1(sensertmp);
    data_to_send[_cnt++]=BYTE0(sensertmp);

    data_to_send[4] = _cnt-5;

    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;

    send_data(data_to_send, _cnt);
}

void send_rc_data(rt_uint16_t thr, rt_uint16_t yaw, rt_uint16_t rol,rt_uint16_t pit,
                  rt_uint16_t aux1,rt_uint16_t aux2,rt_uint16_t aux3,
                  rt_uint16_t aux4,rt_uint16_t aux5,rt_uint16_t aux6)
{
    rt_uint8_t _cnt=0, i=0,sum = 0;

    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=MYHWADDR;
    data_to_send[_cnt++]=SWJADDR;
    data_to_send[_cnt++]=0x03;
    data_to_send[_cnt++]=0;
    data_to_send[_cnt++]=BYTE1(thr);
    data_to_send[_cnt++]=BYTE0(thr);
    data_to_send[_cnt++]=BYTE1(yaw);
    data_to_send[_cnt++]=BYTE0(yaw);
    data_to_send[_cnt++]=BYTE1(rol);
    data_to_send[_cnt++]=BYTE0(rol);
    data_to_send[_cnt++]=BYTE1(pit);
    data_to_send[_cnt++]=BYTE0(pit);
    data_to_send[_cnt++]=BYTE1(aux1);
    data_to_send[_cnt++]=BYTE0(aux1);
    data_to_send[_cnt++]=BYTE1(aux2);
    data_to_send[_cnt++]=BYTE0(aux2);
    data_to_send[_cnt++]=BYTE1(aux3);
    data_to_send[_cnt++]=BYTE0(aux3);
    data_to_send[_cnt++]=BYTE1(aux4);
    data_to_send[_cnt++]=BYTE0(aux4);
    data_to_send[_cnt++]=BYTE1(aux5);
    data_to_send[_cnt++]=BYTE0(aux5);
    data_to_send[_cnt++]=BYTE1(aux6);
    data_to_send[_cnt++]=BYTE0(aux6);

    data_to_send[4] = _cnt-5;

    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];
    data_to_send[_cnt++]=sum;

    send_data(data_to_send,_cnt);
}

void send_power(rt_uint16_t votage, rt_uint16_t current)
{
    rt_uint16_t temp;
    rt_uint8_t _cnt=0, i=0,sum = 0;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=MYHWADDR;
    data_to_send[_cnt++]=SWJADDR;
    data_to_send[_cnt++]=0x05;
    data_to_send[_cnt++]=0;

    temp = votage;
    data_to_send[_cnt++]=BYTE1(temp);
    data_to_send[_cnt++]=BYTE0(temp);
    temp = current;
    data_to_send[_cnt++]=BYTE1(temp);
    data_to_send[_cnt++]=BYTE0(temp);

    data_to_send[4] = _cnt-5;

    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];

    data_to_send[_cnt++]=sum;
    send_data(data_to_send, _cnt);
}


/*********************************************************************
*@brief  向上位机发送GPS数据
*@param[in]
**********************************************************************/
void send_location(rt_uint8_t state,rt_uint8_t sat_num,rt_int32_t lon,rt_int32_t lat,float back_home_angle)
{
    rt_uint8_t _cnt=0;
    rt_uint16_t _temp;
    rt_uint32_t _temp2;

    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=MYHWADDR;
    data_to_send[_cnt++]=SWJADDR;
    data_to_send[_cnt++]=0x04;
    data_to_send[_cnt++]=0;

    data_to_send[_cnt++]=state;
    data_to_send[_cnt++]=sat_num;

    _temp2 = lon;//经度
    data_to_send[_cnt++]=BYTE3(_temp2);
    data_to_send[_cnt++]=BYTE2(_temp2);
    data_to_send[_cnt++]=BYTE1(_temp2);
    data_to_send[_cnt++]=BYTE0(_temp2);

    _temp2 = lat;//纬度
    data_to_send[_cnt++]=BYTE3(_temp2);
    data_to_send[_cnt++]=BYTE2(_temp2);
    data_to_send[_cnt++]=BYTE1(_temp2);
    data_to_send[_cnt++]=BYTE0(_temp2);


    _temp = (rt_int16_t)(100 *back_home_angle);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);


    data_to_send[4] = _cnt-5;

    rt_uint8_t sum = 0;
    for(rt_uint8_t i=0;i<_cnt;i++)
        sum += data_to_send[i];
    data_to_send[_cnt++]=sum;

    send_data(data_to_send, _cnt);
}
/*********************************************************************
*@brief   向上位机发送机体速度
*@param[in]
**********************************************************************/
void send_speed(float x_s,float y_s,float z_s)
{
    rt_uint8_t _cnt=0;
    rt_int16_t _temp;

    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=MYHWADDR;
    data_to_send[_cnt++]=SWJADDR;
    data_to_send[_cnt++]=0x0B;
    data_to_send[_cnt++]=0;

    _temp = (int)(0.1f *x_s);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int)(0.1f *y_s);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int)(0.1f *z_s);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);


    data_to_send[4] = _cnt-5;

    rt_uint8_t sum = 0;
    for(rt_uint8_t i=0;i<_cnt;i++)
        sum += data_to_send[i];
    data_to_send[_cnt++]=sum;

    send_data(data_to_send, _cnt);
}

void send_moto_pwm(rt_uint16_t m_1,rt_uint16_t m_2,rt_uint16_t m_3,rt_uint16_t m_4,
                   rt_uint16_t m_5,rt_uint16_t m_6,rt_uint16_t m_7,rt_uint16_t m_8)
{
    rt_uint8_t _cnt=0, i=0,sum = 0;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=MYHWADDR;
    data_to_send[_cnt++]=SWJADDR;
    data_to_send[_cnt++]=0x06;
    data_to_send[_cnt++]=0;

    data_to_send[_cnt++]=BYTE1(m_1);
    data_to_send[_cnt++]=BYTE0(m_1);
    data_to_send[_cnt++]=BYTE1(m_2);
    data_to_send[_cnt++]=BYTE0(m_2);
    data_to_send[_cnt++]=BYTE1(m_3);
    data_to_send[_cnt++]=BYTE0(m_3);
    data_to_send[_cnt++]=BYTE1(m_4);
    data_to_send[_cnt++]=BYTE0(m_4);
    data_to_send[_cnt++]=BYTE1(m_5);
    data_to_send[_cnt++]=BYTE0(m_5);
    data_to_send[_cnt++]=BYTE1(m_6);
    data_to_send[_cnt++]=BYTE0(m_6);
    data_to_send[_cnt++]=BYTE1(m_7);
    data_to_send[_cnt++]=BYTE0(m_7);
    data_to_send[_cnt++]=BYTE1(m_8);
    data_to_send[_cnt++]=BYTE0(m_8);

    data_to_send[4] = _cnt-5;

    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];

    data_to_send[_cnt++]=sum;
    send_data(data_to_send, _cnt);
}

void send_string(const char *str)
{
    rt_uint8_t _cnt=0, i=0,sum = 0;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=MYHWADDR;
    data_to_send[_cnt++]=SWJADDR;
    data_to_send[_cnt++]=0xA0;
    data_to_send[_cnt++]=0;
    while(*(str+i) != '\0')
    {
        data_to_send[_cnt++] = *(str+i++);
        if(_cnt > 50)
            break;
    }

    data_to_send[4] = _cnt-5;
    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];
    data_to_send[_cnt++]=sum;

    send_data(data_to_send, _cnt);
}


void send_parame(rt_uint16_t num)
{
    rt_uint8_t _cnt=0, i=0,sum = 0;
    int32_t data;
    if(num > PARNUM)
        return;
    ParUsedToParList();
    data = Par_List[num];
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=MYHWADDR;
    data_to_send[_cnt++]=SWJADDR;
    data_to_send[_cnt++]=0xE1;
    data_to_send[_cnt++]=0;
    data_to_send[_cnt++]=BYTE1(num);
    data_to_send[_cnt++]=BYTE0(num);
    data_to_send[_cnt++]=BYTE3(data);
    data_to_send[_cnt++]=BYTE2(data);
    data_to_send[_cnt++]=BYTE1(data);
    data_to_send[_cnt++]=BYTE0(data);

    data_to_send[4] = _cnt-5;

    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];

    data_to_send[_cnt++]=sum;

    send_data(data_to_send, _cnt);

}

