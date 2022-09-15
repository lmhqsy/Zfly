/*；
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-01     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_AUTOCOPTER_MODE_ALL_H_
#define APPLICATIONS_AUTOPILOT_AUTOCOPTER_MODE_ALL_H_
#include "stdbool.h"
#include "loco_config.h"

enum FlightMode {
                ACRO_Mode =          0,  // manual body-frame angular rate with manual throttle
                STABILIZE_Mode =     1,  // manual airframe angle with manual throttle
                ALT_HOLD_Mode =      2,  // manual airframe angle with automatic throttle
                POS_HOLD_Mode =      3,  //
                FLOW_HOLD_Mode =     4,  // fully automatic waypoint control using mission commands
                GUIDED_Mode =        5,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
                LOITER_Mode =        6,  // automatic horizontal acceleration with automatic throttle
                RTL_Mode =           7,  // automatic return to launching point
                LAND_Mode =          8,
                CIRCLE_Mode =        9,
                ROS_Nav_Mode =       10,
                LINE_FIND_Mode =     15,

                FlightMode_NumMode
    };


enum ModeReason {
                UNKNOWN = 0,
                RC_COMMAND = 1,
                GCS_COMMAND = 2,
                RADIO_FAILSAFE = 3,
                BATTERY_FAILSAFE = 4,
                GCS_FAILSAFE = 5,
                EKF_FAILSAFE = 6,
                GPS_GLITCH = 7,
                ModeReason_NumReason
    };

uint16_t set_mode_cnt;

uint16_t get_mode_cut(void);
bool set_flight_mode(enum FlightMode mode, enum ModeReason reason);
// called at 100hz or more
void update_flight_mode(int16_t new_mode);
uint16_t get_new_mode(void);

//获取飞行员的摇杆期望,单位为角度,并且做幅值限制
void get_pilot_desired_lean_angles(float *roll_out, float *pitch_out, float angle_max);
float get_pilot_desired_yaw_rate(float yaw_in);

//得到处理后的油门期望,范围0 to 1
float get_pilot_desired_throttle(void);
float get_pilot_desired_climb_rate(float throttle_control);




/**********************************************************************************/
/***定高模式***/
void altitude_hold_mode_pid_par_init(void);
void mode_altitude_hold_initialization(void);
void mode_altitude_hold_run(void);          //定高模式  （飞行员控制飞行器角度、油门中位时高度保持）
/***定点模式***/
void position_hold_mode_pid_par_init(void);
void mode_position_hold_initialization(void);
void mode_position_hold_run(void);           //定点模式  （飞行员控制）
/***姿态模式***/
void stabilize_mode_pid_par_init(void);
void mode_stabilize_initialization(void);
void mode_stabilize_run(void);             //自稳模式  (飞行员控制飞行器角度,手动控制油门)
/***指导模式***/
void mode_guided_initialization(void); //指导飞行模式  (飞行员控制)
void mode_guided_run(void); //指导飞行模式  (飞行员控制)
/***降落模式***/
void mode_land_initialization(void);      //初始化飞行参数
void mode_land_run(void);                 //定高模式  （飞行员控制飞行器角度、油门中位时高度保持）//200hz
/***光流定点模式***/
void flow_hold_mode_pid_par_init(void);
void mode_flow_hold_initialization(void);
void mode_flow_hold_run(void);                                      //定点模式  （飞行员控制）
void flowhold_flow_to_angle(Vector2f *bf_angles, bool stick_input); //光流位置与速度信息用于位置控制
Vector3f get_key_fly_position_cm(void); //返回记录一键起飞的位置
/***环形飞行模式***/
void circle_mode_pid_par_init(void);
void mode_circle_initialization(void);
void mode_circle_run(void);

/***ros导航飞行模式***/
void ros_nav_mode_pid_par_init(void);
void mode_ros_nav_initialization(void);
void mode_ros_nav_run(void);

/***寻线飞行模式***/
void line_find_mode_pid_par_init(void);
void mode_line_find_initialization(void);
void mode_line_find_run(void);


#endif /* APPLICATIONS_AUTOPILOT_AUTOCOPTER_MODE_ALL_H_ */
