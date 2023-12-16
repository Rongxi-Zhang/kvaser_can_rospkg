/*==================================================================
  Software License Agreement (GNU General Public License v3.0)
  http://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2023, Rongxi Zhang
  All rights reserved.

  E-mail: rongxizhangcar@gmail.com
 ===================================================================*/

#include "kvaser_can_rospkg/kvasercan_interface.hpp"

/* Declare */
using namespace Kvaser;

int main(int argc, char* argv[]) {
  std::cout << std::endl;

  // get all card information
  std::vector<std::shared_ptr<CANCard>> cards;
  CANChannel::getAllCardInformation(cards, 10);

  if (cards.size() > 0) {
    for (const auto& card : cards) {
      std::cout << "\033[1;33mCard \033[0m" << (&card - &cards[0]) << ":"
                << std::endl;
      // Basic Information
      std::cout << "  S/N: " << card->card_serial_no << std::endl;
      std::cout << "  UPC: " << card->card_upc_no << std::endl;
      std::cout << "  Name: " << card->card_name << std::endl;
      std::cout << "  Firmware Version: v";
      std::cout << card->firmware_ver[3] << ".";
      std::cout << card->firmware_ver[2] << ".";
      std::cout << card->firmware_ver[1] << std::endl;
      std::cout << "  Driver: " << card->driver_name << " v";
      std::cout << card->driver_ver[3] << ".";
      std::cout << card->driver_ver[2] << ".";
      std::cout << card->driver_ver[1] << std::endl;

      // Channel Capabilities
      std::cout << "  Capabilities: ";
      if (card->cap & canCHANNEL_CAP_EXTENDED_CAN) std::cout << "Ext, ";
      if (card->cap & canCHANNEL_CAP_BUS_STATISTICS) std::cout << "Stat, ";
      if (card->cap & canCHANNEL_CAP_ERROR_COUNTERS) std::cout << "ErrCnt, ";
      if (card->cap & canCHANNEL_CAP_CAN_DIAGNOSTICS) std::cout << "Diag, ";
      if (card->cap & canCHANNEL_CAP_GENERATE_ERROR) std::cout << "ErrGen, ";
      if (card->cap & canCHANNEL_CAP_GENERATE_OVERLOAD) std::cout << "OvlGen, ";
      if (card->cap & canCHANNEL_CAP_TXREQUEST) std::cout << "TxRq, ";
      if (card->cap & canCHANNEL_CAP_TXACKNOWLEDGE) std::cout << "TxAck, ";
      if (card->cap & canCHANNEL_CAP_VIRTUAL) std::cout << "Virtual, ";
      if (card->cap & canCHANNEL_CAP_SIMULATED) std::cout << "Simulated, ";
      if (card->cap & canCHANNEL_CAP_REMOTE) std::cout << "Remote, ";
      if (card->cap & canCHANNEL_CAP_CAN_FD) std::cout << "FD, ";
      if (card->cap & canCHANNEL_CAP_CAN_FD_NONISO) std::cout << "Non-ISO, ";
      if (card->cap & canCHANNEL_CAP_SILENT_MODE) std::cout << "Silent, ";
      if (card->cap & canCHANNEL_CAP_SINGLE_SHOT) std::cout << "Single-shot, ";
      if (card->cap & canCHANNEL_CAP_CANTEGRITY) std::cout << "CANtegrity, ";
      std::cout << std::endl;

      // Channel Extend Capabilities
      std::cout << "  Extend Capabilities: "
                << ((card->cap_ex[0] & canCHANNEL_CAP_EX_BUSPARAMS_TQ) ? "Yes"
                                                                       : "No")
                << std::endl;

      // get all channel information on the card
      std::vector<std::shared_ptr<CANChannel>> channels;
      CANChannel::getAllChannelInfomation(channels, 10);

      for (const auto& channel : channels) {
        if (channel->card_serial_no == card->card_serial_no) {
          std::cout << "\033[1;34m  Channel \033[0m"
                    << channel->channel_no_on_card << ":" << std::endl;
          std::cout << "    Index: " << channel->channel_idx << std::endl;
          std::cout << "    Max Bitrate: " << channel->max_bitrate << std::endl;
        }
      }

      std::cout << std::endl;
    }
  }

  return 0;
}