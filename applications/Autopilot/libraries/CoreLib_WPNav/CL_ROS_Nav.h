/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-06-27     Administrator       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_WPNAV_CL_ROS_NAV_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_WPNAV_CL_ROS_NAV_H_

#include "mavlink.h"
#include "mavlink.h"

void MAVLinkRcv_Handler(mavlink_message_t MAVLinkMsg);


extern mavlink_vision_position_estimate_t t265_position_estimate;
extern mavlink_vision_position_estimate_t t265_pos;

extern mavlink_vision_speed_estimate_t    t265_speed_estimate;
extern mavlink_vision_speed_estimate_t    t265_speed;

extern mavlink_set_position_target_local_ned_t  ros_nav_target;

extern mavlink_command_long_t  ros_command;
extern mavlink_command_ack_t   ros_command_ack;

extern mavlink_heartbeat_t  hear_t;
extern uint16_t msg_id;
#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_WPNAV_CL_ROS_NAV_H_ */
