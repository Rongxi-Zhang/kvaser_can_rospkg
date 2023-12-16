/*==================================================================
  Software License Agreement (GNU General Public License v3.0)
  http://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2023, Rongxi Zhang
  All rights reserved.

  E-mail: rongxizhangcar@gmail.com
 ===================================================================*/

#ifndef KVASERCAN_BRIDGE_HPP
#define KVASERCAN_BRIDGE_HPP

/* C++ Standard */
#include <deque>
/* ROS standard */
#include <can_msgs/Frame.h>
#include <ros/ros.h>

/* Kvaser interface */
#include "kvaser_can_rospkg/FramePlus.h"
#include "kvaser_can_rospkg/WriteService.h"
#include "kvaser_can_rospkg/kvasercan_interface.hpp"

/* define */

/* Declare */
bool serviceCallback(kvaser_can_rospkg::WriteService::Request &,
                     kvaser_can_rospkg::WriteService::Response &);

/**
 * @brief
 *
 * @details
 *
 */
class kvasercan_bridge {
 public:
  kvasercan_bridge(ros::NodeHandle &, const int32_t[]);
  ~kvasercan_bridge();

 public:
  void notifyCallback();
  void start();
  void stop();
  bool serviceCallback(kvaser_can_rospkg::WriteService::Request &,
                       kvaser_can_rospkg::WriteService::Response &);

 private:
  uint32_t hardware_id;
  uint32_t circuit_id;
  int32_t bitrate[2];
  uint32_t filter_code;
  uint32_t filter_mask;
  bool is_canfd, is_exclusive, is_virtual;

 private:
  // CAN object
  Kvaser::CAN kvaser_can;
  // node handle
  ros::NodeHandle node_handle;
  // Async Spinner, 1 thread
  ros::AsyncSpinner async_spinner;
  // can msgs publisher
  ros::Publisher can_msgs_publisher;
  // can write service
  ros::ServiceServer can_write_server;
  // kvaser can read buffer (deque)
  int32_t window_size;
  std::deque<Kvaser::CANFrame> frame_sliderwindow;

}; /* class kvasercan_bridge */

#endif /* KVASERCAN_BRIDGE_HPP */