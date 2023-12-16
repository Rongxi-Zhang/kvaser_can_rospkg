/*==================================================================
  Software License Agreement (GNU General Public License v3.0)
  http://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2023, Rongxi Zhang
  All rights reserved.

  E-mail: rongxizhangcar@gmail.com
 ===================================================================*/

/* Kvaser bridge */
#include "kvaser_can_rospkg/kvasercan_bridge.hpp"

/*------------- Member Functions for class kvasercan_bridge -------------*/

/**
 * kvasercan_bridge
 *
 * @param  {ros::NodeHandle} nh_ :
 * @param  {int32_t []} params_  :
 */
kvasercan_bridge::kvasercan_bridge(ros::NodeHandle& nh_,
                                   const int32_t params_[])
    : kvaser_can(params_[0], params_[1]), node_handle(nh_), async_spinner(1) {
  // set can params
  hardware_id = params_[0];
  circuit_id = params_[1];
  bitrate[0] = params_[2];
  bitrate[1] = params_[3];
  filter_code = params_[4];
  filter_mask = params_[5];
  window_size = params_[6];
  is_canfd = params_[7];
  is_exclusive = params_[8];
  is_virtual = params_[9];

  // set kvasercan
  kvaser_can.SetOpenMethod(is_canfd, is_exclusive, is_virtual);
  kvaser_can.OpenChannel();
  if (!kvaser_can.is_canfd)
    kvaser_can.SetBitrate(bitrate[0]);
  else
    kvaser_can.SetBitrate(bitrate[0], bitrate[1]);
  kvaser_can.SetDriverModes(canDRIVER_NORMAL);
  kvaser_can.BusOn();
  kvaser_can.SetAcceptanceFilters(filter_code, filter_mask, false);
  kvaser_can.RegisterCallBack([this]() { this->notifyCallback(); },
                              canNOTIFY_RX);

  // set publisher
  std::string topic_name =
      "/kvasercan_bridge/can" + std::to_string(kvaser_can.channel_index);
  ROS_INFO("topic name: %s", topic_name.c_str());

  can_msgs_publisher =
      kvaser_can.is_canfd ? node_handle.advertise<kvaser_can_rospkg::FramePlus>(
                                topic_name, window_size * 10)
                          : node_handle.advertise<can_msgs::Frame>(
                                topic_name, window_size * 10);

  // set service
  std::string service_name = topic_name + "_write";
  can_write_server = node_handle.advertiseService(
      service_name, &kvasercan_bridge::serviceCallback, this);
  ROS_INFO("sevice name: %s", service_name.c_str());

  // wait for valid
  ros::Time::waitForValid();

  // start spinner
  start();
}

/**
 * kvasercan_bridge::~kvasercan_bridge
 *
 */
kvasercan_bridge::~kvasercan_bridge() {
  stop();
  ros::waitForShutdown();
}

/**
 * kvasercan_bridge::notifyCallback
 *
 */
void kvasercan_bridge::notifyCallback() {
  // ROS_INFO("CALL BACK.");
}

/**
 * kvasercan_bridge::start
 *
 */
void kvasercan_bridge::start() {
  // spinner start
  if (async_spinner.canStart())
    async_spinner.start();
  else
    return;

  // can read start
  try {
    do {
      Kvaser::CANFrame* curr_frame_ptr =
          new Kvaser::CANFrame(kvaser_can.is_canfd);
      kvaser_can.ReadWait(curr_frame_ptr, false);
      ROS_INFO("0x%0.2x", curr_frame_ptr->id);

      if (frame_sliderwindow.size() <= window_size) {
        frame_sliderwindow.push_front(*curr_frame_ptr);
      } else {
        Kvaser::CANFrame back_frame = frame_sliderwindow.back();
        frame_sliderwindow.pop_back();
        if (!is_canfd) {
          can_msgs::Frame receive_frame;
          receive_frame.header.frame_id =
              std::to_string(kvaser_can.channel_index);
          receive_frame.header.stamp = ros::Time::now();
          receive_frame.id = back_frame.id;
          receive_frame.is_error = back_frame.is_error;
          receive_frame.is_extended = back_frame.is_extended;
          receive_frame.is_rtr = back_frame.is_rtr;
          receive_frame.dlc = back_frame.dlc;
          std::copy(back_frame.data.begin(), back_frame.data.end(),
                    receive_frame.data.begin());
          can_msgs_publisher.publish(receive_frame);
          // ROS_INFO("publsih1.");

        } else {
          kvaser_can_rospkg::FramePlus receive_frame;
          receive_frame.header.frame_id =
              std::to_string(kvaser_can.channel_index);
          receive_frame.header.stamp = ros::Time::now();
          receive_frame.id = back_frame.id;
          receive_frame.is_error = back_frame.is_error;
          receive_frame.is_extended = back_frame.is_extended;
          receive_frame.is_rtr = back_frame.is_rtr;
          receive_frame.dlc = back_frame.dlc;
          receive_frame.data.resize(CANFD_BUFFER_SIZE);
          std::copy(back_frame.data.begin(), back_frame.data.end(),
                    receive_frame.data.begin());
          can_msgs_publisher.publish(receive_frame);
          // ROS_INFO("publsih2.");
        }
      }

    } while (true);
  } catch (std::runtime_error& e) {
    std::cout << "Please Check CAN Status." << std::endl;

    kvaser_can.BusOff();
    kvaser_can.CloseChannel();
    kvaser_can.UnloadLibrary();

    std::cout << "rosrun kvaser_can_rospkg vcanfd_monitior_test ChannelIdx."
              << std::endl;
  }
}

/**
 * kvasercan_bridge::stop
 *
 */
void kvasercan_bridge::stop() { async_spinner.stop(); }

/**
 * kvasercan_bridge
 *
 * @param  {kvaser_can_rospkg::WriteService::Request} req  :
 * @param  {kvaser_can_rospkg::WriteService::Response} res :
 * @return {bool}                                          :
 */
bool kvasercan_bridge::serviceCallback(
    kvaser_can_rospkg::WriteService::Request& req,
    kvaser_can_rospkg::WriteService::Response& res) {
  // create frame
  Kvaser::CANFrame write_frame(req.is_canfd);
  write_frame.setID(req.id);
  if (!kvaser_can.is_canfd)
    write_frame.setCANFrameType(canMSG_STD);
  else
    write_frame.setCANFrameType(canFDMSG_FDF | canFDMSG_BRS);
  write_frame.dlc = req.dlc;
  std::copy(req.data.begin(), req.data.end(),
            std::back_inserter(write_frame.data));
  // write
  if (kvaser_can.Write(&write_frame))
    res.result = true;
  else
    res.result = false;

  return res.result;
}
