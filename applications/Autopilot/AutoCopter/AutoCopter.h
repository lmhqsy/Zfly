/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-27     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_AUTOCOPTER_AUTOCOPTER_H_
#define APPLICATIONS_AUTOPILOT_AUTOCOPTER_AUTOCOPTER_H_



void AutoCopter_thread_task_init(void);


void attitude_rate_control_task(void* parameter);//线程
void flight_mode_task(void* parameter);//线程
void update_batt_compass_task(void* parameter);//线程
void logging_task(void* parameter);//线程
void userhook_task1(void* parameter);//线程
void parameter_storage_task(void* parameter);//线程

#endif /* APPLICATIONS_AUTOPILOT_AUTOCOPTER_AUTOCOPTER_H_ */
