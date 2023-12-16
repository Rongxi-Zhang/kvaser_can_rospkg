/*==================================================================
  Software License Agreement (GNU General Public License v3.0)
  http://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2023, Rongxi Zhang
  All rights reserved.

  E-mail: rongxizhangcar@gmail.com
 ===================================================================*/

#ifndef KVASERCAN_INTERFACE_HPP
#define KVASERCAN_INTERFACE_HPP

/* C++ Standard */
#include <algorithm>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

/* C Standard */
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

/* Kvaser Canlib */
extern "C" {
#include <canlib.h>
}

/* Define */
#define CAN_BUFFER_SIZE 8
#define CANFD_BUFFER_SIZE 64
#define READ_WAIT_INFINITE (unsigned long)(-1)

/* Declare */
using namespace std;

namespace Kvaser {

/**
 * @brief   This is a C++ abstract class type
 *          for a Kvaser CAN card device.
 * @details This card class is used as the
 *          base class to derive the channel class.
 */
class CANCard {
 public:
  // Card Name
  string card_name;
  // Card type
  uint32_t card_type;
  // Card UPC Number
  string card_upc_no;
  // Firmware Version
  uint16_t firmware_ver[4];
  // Card Serial Number
  uint64_t card_serial_no;
  // Driver Name
  string driver_name;
  // Driver Version
  uint16_t driver_ver[4];
  // CANFd support or not
  uint32_t cap;
  // Setting bus parameters
  uint64_t cap_ex[2];
}; /* class CANCard */

/**
 * @brief   This is a C++ abstract class type
 *          for a Kvaser CAN Channel
 * @details This channel class is derived from
 *          the base card class.
 */
class CANChannel : public CANCard {
 public:
  static void getNumberOfChannels();
  static void getAllChannelInfomation(std::vector<std::shared_ptr<CANChannel>>&,
                                      const int&);
  static void getAllCardInformation(std::vector<std::shared_ptr<CANCard>>&,
                                    const int&);

 public:
  // Channel index
  uint32_t channel_idx{0};
  // Channel Max Bitrate
  uint32_t max_bitrate{0};
  // Channel Number
  uint32_t channel_no_on_card{0};

 public:
  // Available channel
  static int channel_count;
  // friend class
  friend class CAN;

}; /* class CANChannel */

/**
 * @brief   This is a C++ abstract basic class type
 *          for a Kvaser CAN/CANFD frame
 * @details The initialization object that needs
 *          to be explicited.
 */
class CANFrame {
 public:
  explicit CANFrame(bool is_fd_) : is_fd(is_fd_){};
  void setCANFrameType(const uint32_t&);
  void setID(const long&);
  void getFlags(uint32_t&);
  void clear();

 public:
  long id;
  bool is_rtr;
  bool is_extended;
  bool is_error;
  uint32_t dlc;
  std::vector<uint8_t> data;
  ulong timestamp;

 protected:
  bool is_fd;
  uint32_t type;
}; /* class CANFrame */

/**
 * @brief   This is a C++ derived class type
 *          for a Kvaser CAN/CANFD frame
 * @details This class is derived from the CANFrame.
 */
class CANFramePlus : public CANFrame {
 public:
  explicit CANFramePlus(bool is_fd_) : CANFrame(is_fd_) {}
  void setCANFrameType(const uint32_t&);

 private:
  // Data Frames / Remote Requests / Wakeup Frames
  bool wakeup_mode{false};
  bool nerr_active{false};
  bool tx_ack{false};
  bool tx_rq{false};
  bool msg_delayed{false};
  bool single_shot{false};
  bool tx_nack{false};
  bool arb_lost{false};

 private:
  // FD Frame
  bool fd_msg{false};
  bool fd_bitrate_switch{false};
  bool fd_sndr_err_pass_md{false};

 private:
  // Error Frames
  bool has_err{false};
  bool hw_overrun_err{false};
  bool sw_overrun_err{false};
  bool stuff_err{false};
  bool form_err{false};
  bool crc_err{false};
  bool bit0_err{false};
  bool bit1_err{false};
  bool any_overrun_err{false};
  bool any_bit_err{false};
  bool any_rx_err{false};

}; /* class CANFramePlus */

/**
 * @brief   This C++ abstract class is used to
 *          allow users to use a CAN channel.
 * @details
 *
 */
class CAN {
 public:
  CAN(const uint32_t&, const uint32_t&);
  ~CAN();
  CAN(CAN&& p) = delete;
  CAN& operator=(CAN&& p) = delete;
  CAN(const CAN&) = delete;
  CAN& operator=(const CAN&) = delete;

 public:
  void SetOpenMethod(bool, bool, bool);
  void OpenChannel();
  void SetBitrate(const uint32_t&);
  void SetBitrate(const uint32_t&, const uint32_t&);
  void SetBitrate(const kvBusParamsTq&);
  void SetBitrate(const kvBusParamsTq&, const kvBusParamsTq&);
  void SetDriverModes(const uint32_t&);
  void BusOn();
  void SetAcceptanceFilters(const uint32_t&, const uint32_t&, bool);
  void ReadWait(CANFrame*, bool);
  void ReadSyncSpecific(long, ulong);
  bool Write(CANFrame*);
  void RegisterCallBack(std::function<void(void)>, const uint32_t&);
  static void CallBack(canNotifyData* data);
  void BusOff();
  void CloseChannel();
  void UnloadLibrary();

 public:
  // Channel index
  uint32_t channel_index;
  // CAN/CANFD Flag
  bool is_canfd{true};

 private:
  // Hardware ID (s/n)
  uint32_t hardware_id;
  // Channel on card
  uint32_t circuit_id;
  // Channel Bitrate
  int32_t bitrate[2];
  // Driver Modes
  uint32_t driver_mode;

  // Open Flag
  bool is_open{false};
  // On Bus
  bool is_onbus{false};
  // Vitural Flag
  bool is_vitural{true};
  // Exclusive Flag
  bool is_exclusive{false};

  // Define callback function
  std::function<void(void)> CallbackFunc;

  // CAN Handle
  std::shared_ptr<CanHandle> handle;

 private:
  static std::vector<std::shared_ptr<CANChannel>> channel_ptrs;

}; /* class CAN */

/* namespce Functions */
void checkStatus(string name, canStatus stat);

} /* namespace Kvaser */

#endif /* KVASERCAN_INTERFACE_HPP */
