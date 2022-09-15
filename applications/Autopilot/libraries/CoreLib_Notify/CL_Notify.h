/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-12     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_NOTIFY_CL_NOTIFY_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_NOTIFY_CL_NOTIFY_H_


#include <board.h>
#include "CL_Vector.h"
#include <rtdbg.h>
int led_init(void);
void LED_1ms_DRV(void); //0~20
void led_state_update(uint8_t dT_ms);

#define LED1_ON       rt_pin_write(LED1_PIN, LED_ON);
#define LED1_OFF      rt_pin_write(LED1_PIN, LED_OFF);
#define LED2_ON       rt_pin_write(LED2_PIN, LED_ON);
#define LED2_OFF      rt_pin_write(LED2_PIN, LED_OFF);
#define LED3_ON       rt_pin_write(LED3_PIN, LED_ON);
#define LED3_OFF      rt_pin_write(LED3_PIN, LED_OFF);


void cpu_usage_get(rt_uint8_t *major, rt_uint8_t *minor);
int cpu_usage_init(void);

#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_NOTIFY_CL_NOTIFY_H_ */
