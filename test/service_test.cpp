#include <ros/ros.h>

#include "kvaser_can_rospkg/WriteService.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "service_client_node");
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<kvaser_can_rospkg::WriteService>(
      "/kvasercan_bridge/can0_write");

  kvaser_can_rospkg::WriteService srv;

  srv.request.id = 1572;
  srv.request.dlc = 8;
  srv.request.is_canfd = false;
  srv.request.data = {241, 0, 0, 0, 1, 0, 0, 0};

  if (client.call(srv)) {
    std::string bool_str = srv.response.result ? "true" : "false";
    ROS_INFO("CAN Write Result: %s", bool_str.c_str());
  } else {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return 0;
}
