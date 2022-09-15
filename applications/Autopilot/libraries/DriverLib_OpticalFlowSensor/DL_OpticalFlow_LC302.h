/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-29     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_OPTICALFLOWSENSOR_DL_OPTICALFLOW_LC302_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_OPTICALFLOWSENSOR_DL_OPTICALFLOW_LC302_H_

#include "loco_config.h"

typedef struct {
    uint8_t  data[30];
    int raw_x;
    int raw_y;
    int qual;
    int dataLen;
    int sonar_timestamp;
    Vector2f pos;
}flow_data;

extern flow_data  Flow;
extern flow_data  Flow306;

void Mini_Flow_Receive(uint8_t data);
void LC302_Flow_Receive(uint8_t data);
Vector2f opticalflow_rotate_complementary_filter(float dt); //光流角速度与陀螺仪角速度融合
bool get_lc302_health(void);
#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_OPTICALFLOWSENSOR_DL_OPTICALFLOW_LC302_H_ */
