/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-29     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_OPTICALFLOWSENSOR_DL_OPTICALFLOW_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_OPTICALFLOWSENSOR_DL_OPTICALFLOW_H_

#include "loco_config.h"


Vector2f opticalflow_rotate_complementary_filter(float dt); //光流角速度与陀螺仪角速度融合
Vector2f t265_rotate_complementary_filter(float dt); //双目速度与陀螺仪角速度融合
#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_OPTICALFLOWSENSOR_DL_OPTICALFLOW_H_ */
