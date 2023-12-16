/*==================================================================
  Software License Agreement (GNU General Public License v3.0)
  http://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2023, Rongxi Zhang
  All rights reserved.

  E-mail: rongxizhangcar@gmail.com
 ===================================================================*/

/* ROS Message */
#include <can_msgs/Frame.h>
#include <kvaser_can_rospkg/FramePlus.h>
#include <ros/ros.h>

#include "kvaser_can_rospkg/kvasercan_interface.hpp"

/* Declare */
using namespace Kvaser;

void ReadCallback() { std::cout << "callback" << std::endl; }

int main(int argc, char* argv[]) {
  try {
    CAN can(1, 0);
    can.SetOpenMethod(true, false, true);
    can.OpenChannel();
    // kvBusParamsTq paramsArb = {80, 16, 16, 16, 47, 2};
    // kvBusParamsTq paramsDat = {40, 31, 8, 8, 0, 2};
    // can.SetBitrate(paramsArb, paramsDat);
    can.SetBitrate(500, 2000);
    can.SetDriverModes(canDRIVER_NORMAL);
    can.BusOn();
    can.SetAcceptanceFilters(0, 0, false);
    // CANFramePlus* frame_ptr = new CANFramePlus(true);
    // can.ReadWait(frame_ptr, false);
    // can.ReadSyncSpecific(333, 2000);
    can.RegisterCallBack(ReadCallback, canNOTIFY_RX);
    do {
      CANFramePlus* frame_ptr = new CANFramePlus(true);
      can.ReadWait(frame_ptr, false);
      delete frame_ptr;
    } while (1);
  } catch (std::runtime_error& e) {
    std::cout << "Please Check your codes." << std::endl;
  }

  return 0;
}
