/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-31     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_HARDWARELIB_BOARD_RC_PPM_INPUT_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_HARDWARELIB_BOARD_RC_PPM_INPUT_H_
#include "loco_config.h"

int PPM_IN_Init(void);
int ppm_get_data(uint16_t *times);
int get_ppm_Queue_num(void);

#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_HARDWARELIB_BOARD_RC_PPM_INPUT_H_ */
