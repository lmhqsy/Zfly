/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-20     CGY       the first version
 */
#ifndef APPLICATIONS_LOCO_CONFIG_H_
#define APPLICATIONS_LOCO_CONFIG_H_

#include <rtdbg.h>
#include <board.h>
#include <rtdevice.h>
#include "board_interface.h"
#include "rc_ppm_input.h"
#include "power_adc.h"
#include "AutoCopter.h"

#include <DL_SerialPort.h>
#include "DL_InertialSensor.h"
#include "DL_InertialSensor_icm20602.h"
#include "DL_InertialSensor_icm20689.h"
#include "DL_InertialSensor_icm42605.h"
#include "DL_Compass_hmc5883l.h"
#include "DL_BaroSensor.h"
#include "DL_Baro_spl06.h"
#include "DL_Baro_bmp388.h"
#include "DL_GPS_Sensor.h"
#include "DL_T265.h"

#include "DL_OpticalFlow.h"
#include "DL_OpticalFlow_LC302.h"
#include "DL_OpticalFlow_LC306.h"
#include "DL_RangeSensor.h"
#include "DL_TFmini.h"
#include "DL_ultrasonic.h"

#include "stdint.h"
#include "stdbool.h"
#include "usb_transfer.h"

#include "CL_WayNavigation.h"
#include "CL_PositionCtrl.h"
#include "CL_AttitudeCtrl.h"
#include "CL_DigitalFilter.h"
#include "CL_MotorsCopter.h"
#include "CL_NavigationKF.h"
#include "CL_RC_Channel.h"
#include "CL_Vector.h"
#include "CL_Math.h"
#include "CL_Matrix.h"
#include "CL_AHRS.h"
#include "CL_ADRC.h"
#include "CL_PID.h"
#include "CL_Notify.h"
#include "CL_MathCtrl.h"


enum Autopilot_Controller_BORAD_TYPES {
        BORAD_locolion_P1 = 0,
        BORAD_locolion_A1 = 1,
        BORAD_zhi_mao_A1 = 2,
        FBW_BORAD_TYPES_Num
    };

#define  FBW_BORAD_TYPES     2    //飞控板载硬件类型       0：locolion_P1(H750)    1:locolion_A1 (f407)  2：智茂飞控  .....  10:自制飞控


#define  IMU_TYPES     0          //飞控IMU硬件类型       0：locolion_P1(H750)    1:locolion_A1 (f407)  .....  10:自制飞控





#endif /* APPLICATIONS_LOCO_CONFIG_H_ */
