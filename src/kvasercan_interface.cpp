/*==================================================================
  Software License Agreement (GNU General Public License v3.0)
  http://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2023, Rongxi Zhang
  All rights reserved.

  E-mail: rongxizhangcar@gmail.com
 ===================================================================*/

#include "kvaser_can_rospkg/kvasercan_interface.hpp"

namespace Kvaser {

/*----------------Member Functions for class CANChannel-------------*/

/**
 * CANChannel::getNumberOfChannels
 *
 * @brief This funcs is used to get the
 *        number of all kvaser can channel.
 */
void CANChannel::getNumberOfChannels() {
  if (channel_count == 0) {
    canStatus stat = canGetNumberOfChannels(&channel_count);
    if (stat != canOK) {
      string name = "canGetNumberOfChannels";
      checkStatus(name, stat);
    } else
      std::cout << "Found Kvaser CAN Channels: " << channel_count << " channels"
                << std::endl;
  }
}

/**
 * CANChannel::getAllChannelInfomation
 *
 * @param  {vector<shared_ptr<CANChannel>>} channel_vector :
 *         Store all channel information
 * @param  {int} max_num                                   :
 *         max channel number
 *         allowed on one kvaser can card
 */
void CANChannel::getAllChannelInfomation(
    vector<shared_ptr<CANChannel>>& channel_vector, const int& max_num) {
  // if not init
  if (channel_count == 0) {
    getNumberOfChannels();
  }

  // check channel number
  if (channel_count < 0 || channel_count > max_num) {
    std::string error_msg = "The number of channels is abnormal!";
    throw std::runtime_error(error_msg);
  }

  // record can status
  canStatus stat;

  // clear vector
  channel_vector.clear();

  for (int i = 0; i < channel_count; ++i) {
    // curr channel
    CANChannel curr_channel;

    // get Card Information
    {
      // get card name
      char name_[256];
      stat = canGetChannelData(i, canCHANNELDATA_DEVDESCR_ASCII, &name_,
                               sizeof(name_));
      if (stat != canOK) {
        string name = "canGetChannelData-devName";
        checkStatus(name, stat);
      } else
        curr_channel.card_name = std::string(name_);

      // get card type
      stat = canGetChannelData(i, canCHANNELDATA_CARD_TYPE,
                               &curr_channel.card_type,
                               sizeof(curr_channel.card_type));

      // get card upc no
      uint32_t upc_no[2];
      stat = canGetChannelData(i, canCHANNELDATA_CARD_UPC_NO, &upc_no,
                               sizeof(upc_no));
      if (stat != canOK) {
        string name = "canGetChannelData-upcNo";
        checkStatus(name, stat);
      } else {
        std::ostringstream oss;
        oss << std::hex << (upc_no[1] >> 12) << "-";
        oss << (((upc_no[1] & 0xfff) << 8) | ((upc_no[0] >> 24) & 0xff)) << "-";
        oss << std::setfill('0') << std::setw(5) << ((upc_no[0] >> 4) & 0xfffff)
            << "-";
        oss << (upc_no[0] & 0x0f);
        curr_channel.card_upc_no = oss.str();
      }

      // get card file version
      stat = canGetChannelData(i, canCHANNELDATA_CARD_FIRMWARE_REV,
                               &curr_channel.firmware_ver,
                               sizeof(curr_channel.firmware_ver));
      if (stat != canOK) {
        string name = "canGetChannelData-fileVersion";
        checkStatus(name, stat);
      }

      // get card serial no
      stat = canGetChannelData(i, canCHANNELDATA_CARD_SERIAL_NO,
                               &curr_channel.card_serial_no,
                               sizeof(curr_channel.card_serial_no));
      if (stat != canOK) {
        string name = "canGetChannelData-serialNo";
        checkStatus(name, stat);
      }

      // get card driver name
      char driverName[256];
      stat = canGetChannelData(i, canCHANNELDATA_DRIVER_NAME, &driverName,
                               sizeof(driverName));
      if (stat != canOK) {
        string name = "canGetChannelData-driverName";
        checkStatus(name, stat);
      } else
        curr_channel.driver_name = std::string(driverName);

      // get card driver version
      stat = canGetChannelData(i, canCHANNELDATA_DRIVER_FILE_VERSION,
                               &curr_channel.driver_ver,
                               sizeof(curr_channel.driver_ver));
      if (stat != canOK) {
        string name = "canGetChannelData-driverVersion";
        checkStatus(name, stat);
      }

      // get card capabillties
      stat = canGetChannelData(i, canCHANNELDATA_CHANNEL_CAP, &curr_channel.cap,
                               sizeof(curr_channel.cap));
      if (stat != canOK) {
        string name = "canGetChannelData-capabillties";
        checkStatus(name, stat);
      }

      // get card extend capabillties
      stat =
          canGetChannelData(i, canCHANNELDATA_CHANNEL_CAP_EX,
                            &curr_channel.cap_ex, sizeof(curr_channel.cap_ex));
      if (stat != canOK) {
        string name = "canGetChannelData-extend-capabillties";
        checkStatus(name, stat);
      }
    }

    // get Channel Information
    {
      curr_channel.channel_idx = i;

      // get channel no on card
      stat = canGetChannelData(i, canCHANNELDATA_CHAN_NO_ON_CARD,
                               &curr_channel.channel_no_on_card,
                               sizeof(curr_channel.channel_no_on_card));
      if (stat != canOK) {
        string name = "canGetChannelData-channelNoOnCard";
        checkStatus(name, stat);
      }

      // get channel max bitrate
      stat = canGetChannelData(i, canCHANNELDATA_MAX_BITRATE,
                               &curr_channel.max_bitrate,
                               sizeof(curr_channel.max_bitrate));
      if (stat != canOK) {
        string name = "canGetChannelData-channelMaxBitrate";
        checkStatus(name, stat);
      }
    }

    // push back
    channel_vector.push_back(
        std::make_shared<CANChannel>(std::move(curr_channel)));
  }
}

/**
 * CANChannel::getAllCardInformation
 *
 * @param  {std::vector<std::shared_ptr<CANCard>>} card_vector :
 *         Store all card information
 * @param  {int} max_num
 *         max channel nunmber                                     :
 */
void CANChannel::getAllCardInformation(
    std::vector<std::shared_ptr<CANCard>>& card_vector, const int& max_num) {
  std::vector<std::shared_ptr<CANChannel>> channels;
  getAllChannelInfomation(channels, max_num);

  for (const auto& channel : channels) {
    bool found = false;

    for (const auto& card : card_vector) {
      if (card->card_serial_no == channel->card_serial_no) found = true;
    }

    if (!found) {
      card_vector.emplace_back(
          std::dynamic_pointer_cast<CANCard>(std::move(channel)));
    }
  }
}

/*----------------Member Functions of class CANFrame----------------*/

/**
 * CANFrame::setCANFrameType
 *
 * @param  {uint32_t} flags : 0xFFFFFFFF TRUE,
 *                            0x00000000 FALSE
 */
void CANFrame::setCANFrameType(const uint32_t& flags) {
  // Basic type
  is_rtr = ((flags & canMSG_RTR) > 0);
  is_extended = ((flags & canMSG_EXT) > 0);
  is_error = ((flags & canMSG_ERROR_FRAME) > 0);
  // For CANFd
  if (is_fd) {
    if (!(flags & canFDMSG_FDF) > 0) {
      throw std::runtime_error(
          "Error parameter: canFDMSG_FDF is set to false!\n");
    }

    if ((flags & canMSG_RTR) > 0) {
      throw std::runtime_error(
          "Note that canMSG_RTR cannot be set for CAN FD messages.\n");
    }
  }
  // Record detail types
  type = flags;
}

/**
 * CANFrame::setID
 *
 */
void CANFrame::setID(const long& id_) { id = id_; }

/**
 * CANFrame::getFlags
 *
 * @param  {uint32_t} flags :
 */
void CANFrame::getFlags(uint32_t& flags) { flags = type; }

/**
 * CANFrame::clear
 *
 */
void CANFrame::clear() {
  long id = 0;
  bool is_rtr = false;
  bool is_extended = false;
  bool is_error = false;
  uint32_t dlc = 0;
  data.clear();
  timestamp = 0;
  type = 0;
}

/*--------------Member Functions of class CANFramePlus-------------*/

/**
 * CANFramePlus::setCANFrameType
 *
 * @param  {uint32_t} flags : 0xFFFFFFFF TRUE,
 *                            0x00000000 FALSE
 */
void CANFramePlus::setCANFrameType(const uint32_t& flags) {
  // Basic Funcs
  CANFrame::setCANFrameType(flags);

  // Data Frames / Remote Requests / Wakeup Frames
  wakeup_mode = ((flags & canMSG_WAKEUP) > 0);
  nerr_active = ((flags & canMSG_NERR) > 0);
  tx_ack = ((flags & canMSG_TXACK) > 0);
  tx_rq = ((flags & canMSG_TXRQ) > 0);
  msg_delayed = ((flags & canMSG_DELAY_MSG) > 0);
  single_shot = ((flags & canMSG_SINGLE_SHOT) > 0);
  tx_nack = ((flags & canMSG_TXNACK) > 0);
  arb_lost = ((flags & canMSG_ABL) > 0);

  // FD Frames
  fd_msg = ((flags & canFDMSG_FDF) > 0);
  fd_bitrate_switch = ((flags & canFDMSG_BRS) > 0);
  fd_sndr_err_pass_md = ((flags & canFDMSG_ESI) > 0);

  // Error Frames
  has_err = ((flags & canMSGERR_MASK) > 0);
  hw_overrun_err = ((flags & canMSGERR_HW_OVERRUN) > 0);
  sw_overrun_err = ((flags & canMSGERR_SW_OVERRUN) > 0);
  stuff_err = ((flags & canMSGERR_STUFF) > 0);
  form_err = ((flags & canMSGERR_FORM) > 0);
  crc_err = ((flags & canMSGERR_CRC) > 0);
  bit0_err = ((flags & canMSGERR_BIT0) > 0);
  bit1_err = ((flags & canMSGERR_BIT1) > 0);
  any_overrun_err = ((flags & canMSGERR_OVERRUN) > 0);
  any_bit_err = ((flags & canMSGERR_BIT) > 0);
  any_rx_err = ((flags & canMSGERR_BUSERR) > 0);
}

/*-------------------Member Functions of class CAN------------------*/
/**
 * CAN construct func
 *
 * @param  {uint32_t} hw_id : card serial number
 * @param  {uint32_t} ch_id : channel id on the card
 */
CAN::CAN(const uint32_t& hw_id, const uint32_t& ch_id)
    : hardware_id(hw_id), circuit_id(ch_id), driver_mode(canDRIVER_NORMAL) {
  // Canlib Initialization
  // On Linux, no re-enumeration is needed since enumeration
  // takes place when a device is plugged in or unplugged.
  canInitializeLibrary();
  handle = std::make_shared<CanHandle>();
  CANChannel::getAllChannelInfomation(channel_ptrs, 10);
}

/**
 * CAN::~CAN
 *
 */
CAN::~CAN() {
  // if (is_onbus) {
  //   BusOff();
  //   is_onbus = false;
  // }
  // if (is_open) {
  //   CloseChannel();
  //   is_open = false;
  // }
  // if (!(is_onbus || is_open)) UnloadLibrary();

  channel_ptrs.clear();
}

/**
 * CAN::SetOpenMethod
 *
 * @param  {bool} is_canfd_     : default true
 * @param  {bool} is_exculsive_ : default false
 * @param  {bool} is_virtual_   : default true
 */
void CAN::SetOpenMethod(bool is_canfd_, bool is_exculsive_, bool is_virtual_) {
  is_canfd = is_canfd_;
  is_exclusive = is_exculsive_;
  is_vitural = is_virtual_;
}

/**
 * CAN::OpenChannel
 *
 * @details Called after call SetOpenMethod
 *
 * @param {} Empty
 */
void CAN::OpenChannel() {
  channel_index = 0;
  bool channel_found = false;

  for (const auto& channel_ptr : channel_ptrs) {
    if (hardware_id == channel_ptr->card_serial_no &&
        circuit_id == channel_ptr->channel_no_on_card) {
      channel_index = channel_ptr->channel_idx;
      channel_found = true;
      break;
    }
  }

  if (channel_found) {
    if (is_canfd) {
      if (!(channel_ptrs[channel_index]->cap & canCHANNEL_CAP_CAN_FD))
        throw std::runtime_error("Not Support CAN FD!\n");
    }

    if (is_vitural) {
      if (!(channel_ptrs[channel_index]->cap & canCHANNEL_CAP_CAN_FD))
        throw std::runtime_error("Not Support Virtual CAN!\n");
    }

    uint32_t flags = (is_canfd ? canOPEN_CAN_FD : 0);
    flags |= (is_exclusive ? canOPEN_EXCLUSIVE : 0);
    flags |= (is_vitural ? canOPEN_ACCEPT_VIRTUAL : 0);

    *handle = canOpenChannel(channel_index, flags);
    if (*handle < 0)
      throw std::runtime_error("Channel Open Failed!\n");
    else
      is_open = true;

  } else {
    throw std::runtime_error("Channel Not Found!\n");
  }
}

/**
 * CAN::SetBitrate (Classic CAN)
 *
 * @details Using Predefined Bitrate Constants
 *
 * @param  {uint32_t} bitrate_ : 1000 = 1Mbps, > 125
 *
 */
void CAN::SetBitrate(const uint32_t& bitrate_) {
  if (!is_canfd) {
    switch (bitrate_) {
      case 1000:
        bitrate[0] = canBITRATE_1M;
        break;
      case 500:
        bitrate[0] = canBITRATE_500K;
        break;
      case 250:
        bitrate[0] = canBITRATE_250K;
        break;
      case 125:
        bitrate[0] = canBITRATE_125K;
        break;
      default:
        throw std::runtime_error(
            "For Classic CAN, use the predefined bitrate parameters\n");
        break;
    }
    canStatus stat = canSetBusParams(*handle, bitrate[0], 0, 0, 0, 0, 0);
    if (stat != canOK) {
      string name = "canSetBusParams";
      checkStatus(name, stat);
    }
  } else {
    throw std::runtime_error("Classic CAN, Not CAN FD!\n");
  }
}

/**
 * CAN::SetBitrate (CAN FD)
 *
 * @details Using Predefined Bitrate Constants
 *
 * @param  {uint32_t} bitrate_fda/fdd : 1000 = 1Mbps
 */
void CAN::SetBitrate(const uint32_t& bitrate_fda, const uint32_t& bitrate_fdd) {
  if (is_canfd) {
    switch (bitrate_fda) {
      case 1000:
        bitrate[0] = canFD_BITRATE_1M_80P;
        break;
      case 500:
        bitrate[0] = canFD_BITRATE_500K_80P;
        break;
      default:
        throw std::runtime_error(
            "For CAN FD, set the arbitration phase bitrate to 500 kbit/s, with "
            "sampling point at 80%\n");
    }

    switch (bitrate_fdd) {
      case 8000:
        bitrate[1] = canFD_BITRATE_8M_60P;
        break;
      case 4000:
        bitrate[1] = canFD_BITRATE_4M_80P;
        break;
      case 2000:
        bitrate[1] = canFD_BITRATE_2M_80P;
        break;
      case 1000:
        bitrate[1] = canFD_BITRATE_1M_80P;
        break;
      default:
        throw std::runtime_error(
            "For CAN FD, set the data phase bitrate to 500 kbit/s, with "
            "sampling point at 80%\n");
    }

    canStatus stat_a, stat_d;
    stat_a = canSetBusParams(*handle, bitrate[0], 0, 0, 0, 0, 0);
    stat_d = canSetBusParamsFd(*handle, bitrate[1], 0, 0, 0);

    if (stat_a != canOK || stat_d != canOK) {
      string name = "canSetBusParams / canSetBusParamsFd";
      checkStatus(name, stat_a);
      checkStatus(name, stat_d);
    }
  } else {
    throw std::runtime_error("CAN FD, Not Classic CAN!\n");
  }
}

/**
 * CAN::SetBitrate (CAN Tq)
 *
 * @param  {kvBusParamsTq} params :
 */
void CAN::SetBitrate(const kvBusParamsTq& params) {
  if (!is_canfd) {
    if (!(channel_ptrs[channel_index]->cap_ex[0] &
          canCHANNEL_CAP_EX_BUSPARAMS_TQ))
      throw std::runtime_error("Not Support canSetBusParamsTq");
    canStatus stat = canSetBusParamsTq(*handle, params);
    if (stat != canOK) {
      string name = "canSetBusParamsTq";
      checkStatus(name, stat);
    }
  } else {
    throw std::runtime_error("Classic CAN, Not CAN FD!\n");
  }
}

/**
 * CAN::SetBitrate (CAN FD Tq)
 *
 * @param  {kvBusParamsTq} params_fda :
 * @param  {kvBusParamsTq} fdd        :
 */
void CAN::SetBitrate(const kvBusParamsTq& params_fda,
                     const kvBusParamsTq& params_fdd) {
  if (is_canfd) {
    if (!(channel_ptrs[channel_index]->cap_ex[0] &
          canCHANNEL_CAP_EX_BUSPARAMS_TQ))
      throw std::runtime_error("Not Support canSetBusParamsFdTq");

    canStatus stat = canSetBusParamsFdTq(*handle, params_fda, params_fdd);
    if (stat != canOK) {
      string name = "canSetBusParamsTq / canSetBusParamsFdTq";
      checkStatus(name, stat);
    }
  } else {
    throw std::runtime_error("CAN FD, Not Classic CAN!\n");
  }
}

/**
 * CAN::SetDriverModes
 *
 * @param  {uint32_t} drivertype :
 *         -canDRIVER_NORMAL           4
 *         -canDRIVER_SILENT           1
 *         -canDRIVER_SELFRECEPTION    8
 *         -canDRIVER_OFF              0
 */
void CAN::SetDriverModes(const uint32_t& drivertype) {
  driver_mode = drivertype;
  if (driver_mode == canDRIVER_SILENT) {
    if (!(channel_ptrs[channel_index]->cap & canCHANNEL_CAP_SILENT_MODE))
      throw std::runtime_error("Not Support Slient Mode\n");
  }
  canStatus stat = canSetBusOutputControl(*handle, driver_mode);
  if (stat != canOK) {
    string name = "canSetBusOutputControl";
    checkStatus(name, stat);
  }
}

/**
 * CAN::BusOn
 *
 */
void CAN::BusOn() {
  canStatus stat = canBusOn(*handle);
  if (stat != canOK) {
    string name = "canBusOn";
    checkStatus(name, stat);
  } else
    is_onbus = true;
}

/**
 * CAN::SetAcceptanceFilters
 *
 * @details The message is accepted
 *          if ((code XOR id) AND mask) == 0.
 *
 * @param  {uint32_t} code : A relevant binary 1 in a code means "the
 *                           corresponding bit in the identifier must be 1".
 * @param  {uint32_t} mask : A binary 1 in a mask means "the corresponding bit
 *                           in the code is relevant".
 */
void CAN::SetAcceptanceFilters(const uint32_t& code, const uint32_t& mask,
                               bool is_ext) {
  canStatus stat_code, stat_mask;

  if (!is_ext) {
    stat_code = canAccept(*handle, code, canFILTER_SET_CODE_STD);
    stat_mask = canAccept(*handle, mask, canFILTER_SET_MASK_STD);
  } else {
    stat_code = canAccept(*handle, code, canFILTER_SET_CODE_EXT);
    stat_mask = canAccept(*handle, mask, canFILTER_SET_MASK_EXT);
  }

  if (stat_code != canOK || stat_mask != canOK) {
    string name = "SetAcceptanceFilter";
    checkStatus(name, stat_code);
    checkStatus(name, stat_mask);
  }
}

/**
 * CAN::ReadWait (canReadWait)
 *
 * @details If you want to wait until a message arrives
 *          (or a timeout occurs) and then read it, call it.
 * @param  {CANFrame*} frame_ptr : basic class ptr
 * @param  {bool} is_plus        : is pointed to a CANFramePlus object?
 *
 */
void CAN::ReadWait(CANFrame* frame_ptr, bool is_plus) {
  if (is_plus) {
    CANFramePlus* frame_ptr = static_cast<CANFramePlus*>(frame_ptr);
  }

  const int size = is_canfd ? CANFD_BUFFER_SIZE : CAN_BUFFER_SIZE;
  uint8_t msg[size];
  uint32_t msg_flag;

  frame_ptr->clear();
  canStatus stat =
      canReadWait(*handle, &(frame_ptr->id), &msg, &(frame_ptr->dlc), &msg_flag,
                  &(frame_ptr->timestamp), READ_WAIT_INFINITE);
  if (stat != canOK) {
    string name = "canReadWait";
    checkStatus(name, stat);
  }

  frame_ptr->setCANFrameType(msg_flag);

  for (int i = 0; i < size; ++i) {
    frame_ptr->data.emplace_back(std::move(msg[i]));
  }
}

/**
 * CAN::ReadSyncSpecific
 *
 * @param  {CANFrame*} frame_ptr : basic class ptr
 * @param  {long} id_            : desired id
 * @param  {ulong} timestamp_    : The timeout in milliseconds.
 *                                 0xFFFFFFFF gives an infinite timeout.
 */
void CAN::ReadSyncSpecific(long id, ulong timeout) {
  canStatus stat = canReadSyncSpecific(*handle, id, timeout);
  if (stat != canOK) {
    string name = "canReadSyncSpecific";
    checkStatus(name, stat);
  }
}

/**
 * CAN::Write
 *
 * @param  {long} id_            : The identifier of the CAN message to send.
 * @param  {CANFrame*} frame_ptr : A pointer to the message data, or nullptr.
 */
bool CAN::Write(CANFrame* frame_ptr) {
  const int size = is_canfd ? CANFD_BUFFER_SIZE : CAN_BUFFER_SIZE;

  uint8_t msg[size];
  memset(msg, 0, sizeof(msg));

  size_t dataSize = frame_ptr->dlc;
  size_t copySize = std::min(dataSize, sizeof(msg));
  memcpy(msg, frame_ptr->data.data(), copySize);

  uint32_t flags = 0;
  frame_ptr->getFlags(flags);

  canStatus stat = canWrite(*handle, frame_ptr->id, msg, frame_ptr->dlc, flags);
  if (stat != canOK) {
    string name = "canWrite";
    checkStatus(name, stat);
    return false;
  } else
    return true;
}

/**
 * CAN::RegisterCallBack
 *
 * @param  {std::function<void(void)>} callable : your callback func name
 */
void CAN::RegisterCallBack(std::function<void(void)> callable,
                           const uint32_t& notifyFlags) {
  CallbackFunc = callable;

  canStatus stat = canSetNotify(*handle, &(CAN::CallBack), notifyFlags,
                                static_cast<void*>(this));
  if (stat != canOK) {
    string name = "canSetNotify";
    checkStatus(name, stat);
  }
}

/**
 * CAN::CallBack
 *
 * @details This function associates a callback function with the CAN circuit.
 *
 * @param  {canNotifyData*} data :
 */
void CAN::CallBack(canNotifyData* data) {
  switch (data->eventType) {
    case canEVENT_STATUS:
      // std::cout << "CAN Status Event" << std::endl;
      break;
    case canEVENT_ERROR:
      // std::cout << "CAN Error Event" << std::endl;
      break;
    case canEVENT_TX:
      // std::cout << "CAN Tx Event" << std::endl;
      break;
    case canEVENT_RX:
      // std::cout << "CAN Rx Event" << std::endl;
      break;
  }

  static_cast<CAN*>(data->tag)->CallbackFunc();
}

void CAN::BusOff() {
  if (is_onbus) {
    canStatus stat = canBusOff(*handle);
    if (stat != canOK) {
      string name = "canBusOff";
      checkStatus(name, stat);
    } else
      is_onbus = false;
  }
}

void CAN::CloseChannel() {
  if (!is_onbus && is_open) {
    canStatus stat = canClose(*handle);
    if (stat != canOK) {
      string name = "canBusOff";
      checkStatus(name, stat);
    } else
      is_open = false;
  }
}

void CAN::UnloadLibrary() {
  if (!(is_onbus || is_open)) {
    canStatus stat = canUnloadLibrary();
    if (stat != canOK) {
      string name = "canUnloadLibrary";
      checkStatus(name, stat);
    }
  }
}

/*------------------------namespce Functions------------------------*/
/**
 * checkStatus
 *
 * @param  {string} name    : canlib function name
 * @param  {canStatus} stat : return status of this function
 *
 */
void checkStatus(string name, canStatus stat) {
  if (stat != canOK) {
    char buf[50];
    buf[0] = '\0';
    canGetErrorText(stat, buf, sizeof(buf));
    std::string error_msg = name + std::string(" failed, stat=") +
                            std::to_string((int)stat) + " (" + buf + ")\n";
    throw std::runtime_error(error_msg);
  }
}

/*----------------------Static Variable Init----------------------*/
int CANChannel::channel_count = 0;
std::vector<std::shared_ptr<CANChannel>> CAN::channel_ptrs;

} /* namespace Kvaser */
