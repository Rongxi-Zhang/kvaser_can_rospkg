/*==================================================================
  Software License Agreement (GNU General Public License v3.0)
  http://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2023, Rongxi Zhang
  All rights reserved.

  E-mail: rongxizhangcar@gmail.com
 ===================================================================*/

/* Kvaser bridge */
#include "kvaser_can_rospkg/kvasercan_bridge.hpp"

// global variables
int32_t params[10];

int main(int argc, char* argv[]) {
  // ROS initial
  ros::init(argc, argv, "kvasercan_bridge");

  // private handle
  ros::NodeHandle nh("~");

  // get params 
  nh.getParam("can_hardware_id", params[0]);
  nh.getParam("can_circuit_id", params[1]);
  nh.getParam("can_bitrate_0", params[2]);
  nh.getParam("can_bitrate_1", params[3]);
  nh.getParam("can_filter_code", params[4]);
  nh.getParam("can_filter_mask", params[5]);
  nh.getParam("can_window_size", params[6]);
  nh.getParam("can_is_canfd", params[7]);
  nh.getParam("can_is_exclusive", params[8]);
  nh.getParam("can_is_virtual", params[9]);

  // kvasercan_bridge
  kvasercan_bridge bridge(nh, params);

  ros::shutdown();
  return 0;
}