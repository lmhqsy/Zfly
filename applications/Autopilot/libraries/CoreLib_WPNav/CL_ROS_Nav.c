/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-06-27     Administrator       the first version
 */
#include "CL_ROS_Nav.h"
#include <board.h>
#include "mavlink.h"
#include "DL_T265.h"
 mavlink_vision_position_estimate_t t265_position_estimate;
 mavlink_vision_position_estimate_t t265_pos;

 mavlink_vision_speed_estimate_t    t265_speed_estimate;
 mavlink_vision_speed_estimate_t    t265_speed;

 mavlink_set_position_target_local_ned_t  ros_nav_target;

 mavlink_command_long_t  ros_command;
 mavlink_command_ack_t   ros_command_ack;
 mavlink_heartbeat_t  hear_t;

 uint16_t msg_id;
void MAVLinkRcv_Handler(mavlink_message_t MAVLinkMsg)
{
//    hear_t.mavlink_version = MAVLinkMsg.msgid;
//    send_senser2(hear_t.type*100,hear_t.mavlink_version*100,hear_t.system_status*10);

   msg_id= MAVLinkMsg.msgid;
  /* MSG_ID: 系统信息*/
  if(MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE == MAVLinkMsg.msgid)   //双目估计的位置
  {

      //得到视觉位置和角度信息   x左右移动，向右为正方向；前后移动，向后为正方向。
      mavlink_msg_vision_position_estimate_decode(&MAVLinkMsg,&t265_position_estimate);
  }

  if(MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE == MAVLinkMsg.msgid)  //双目估计的速度
  {
      //得到视觉位置和角度信息   x左右移动，向右为正方向；前后移动，向后为正方向。
      mavlink_msg_vision_speed_estimate_decode(&MAVLinkMsg,&t265_speed_estimate);
  }


  if(MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED == MAVLinkMsg.msgid)   //发布的位置速度期望
  {

      mavlink_msg_set_position_target_local_ned_decode(&MAVLinkMsg,&ros_nav_target);
  }

  if(MAVLINK_MSG_ID_COMMAND_LONG == MAVLinkMsg.msgid)   //发布的命令（起飞 降落）
  {
      mavlink_msg_command_long_decode(&MAVLinkMsg,&ros_command);
  }


  if(MAVLINK_MSG_ID_HEARTBEAT == MAVLinkMsg.msgid)   //心跳包
  {

      mavlink_msg_heartbeat_decode(&MAVLinkMsg,&hear_t);

  }

  //rt_pin_write(GET_PIN(A,4), 1);

  last_t265_update_ms = rt_tick_get();

}











