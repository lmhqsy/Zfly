/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-29     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_BAROSENSOR_DL_BAROSENSOR_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_BAROSENSOR_DL_BAROSENSOR_H_

#include "loco_config.h"
#include "stdio.h"
#include "stdbool.h"

void Baro_Data_Update(baro_t *baro);

bool baroIsCalibrationComplete(void);
float pressureToAltitude(const float pressure);
void performBaroCalibrationCycle(float baroPressureSamp);

/*限幅平均滤波法*/
float pressureFilter(float in);
int32_t applyBarometerMedianFilter(int32_t newPressureReading);

float get_baro_sensorsAltitude(void);  //海拔高度开机时复位为0  单位cm


#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_BAROSENSOR_DL_BAROSENSOR_H_ */
