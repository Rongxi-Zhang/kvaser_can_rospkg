#include <canlib.h>
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "kvaser_can_rospkg/FramePlus.h"

#define PEAK_CANID_START 0x101
#define PEAK_CANID_END 0x13B

#define GTRACK_ANG_TO_RAD 0.017453292519943
#define GTRACK_RAD_TO_ANG 57.2957795131

#define ALARM_INTERVAL_IN_S (1)
#define WRITE_WAIT_INFINITE (unsigned long)(-1)

typedef struct {
  /** @brief Range, m */
  float range;
  /** @brief Azimuth, rad */
  float azimuth;
  /** @brief Elevation, rad */
  float elev;
  /** @brief Radial velocity, m/s */
  float doppler;
  /**  @brief   Range detection SNR, linear */
  float snr;
  /**  @brief  angle, radian */
  float angle;
} GTRACK_measurement_vector;

static unsigned int msgCounter = 0;
static int willExit = 0;
static uint32_t tx_id = PEAK_CANID_START;

static void check(char *id, canStatus stat) {
  if (stat != canOK) {
    char buf[50];
    buf[0] = '\0';
    canGetErrorText(stat, buf, sizeof(buf));
    printf("%s: failed, stat=%d (%s)\n", id, (int)stat, buf);
  }
}

static void sighand(int sig, siginfo_t *info, void *ucontext) {
  static unsigned int last;
  (void)info;
  (void)ucontext;

  switch (sig) {
    case SIGINT:
      willExit = 1;
      break;
    case SIGALRM:
      if (msgCounter - last) {
        printf("msg/s = %d, total=%u\n",
               (msgCounter - last) / ALARM_INTERVAL_IN_S, msgCounter);
      }
      last = msgCounter;
      alarm(ALARM_INTERVAL_IN_S);
      break;
  }
}

static void printUsageAndExit(char *prgName) {
  printf("Usage: '%s <channel>'\n", prgName);
  exit(1);
}

static void Out_Put_Peak_List(GTRACK_measurement_vector *point,
                              uint32_t peak_num, const canHandle &hnd,
                              canStatus &stat) {
  uint8_t frame = ceil(peak_num / 5.0);
  // uint32_t tx_id = PEAK_CANID_START;

  for (uint16_t i = 0; i < frame; i++) {
    uint8_t msg[64];
    memset(msg, 0, 64);

    msg[12] = (uint8_t)((peak_num >> 8) & 0xFF);
    msg[25] = (uint8_t)(peak_num & 0xFF);

    for (uint16_t j = 0; j < 6; j++) {
      if ((i * 6 + j) < peak_num) {
        GTRACK_measurement_vector current_measurement = point[i * 6 + j];

        uint32_t tmp_data;  // float32

        tmp_data = (uint32_t)(current_measurement.range * 100);
        msg[0 + 13 * j] = (uint8_t)((tmp_data >> 8) & 0xFF);
        msg[1 + 13 * j] = (uint8_t)(tmp_data & 0xFF);

        tmp_data = (uint32_t)((current_measurement.azimuth + 100) * 100);
        msg[2 + 13 * j] = (uint8_t)((tmp_data >> 8) & 0xFF);
        msg[3 + 13 * j] = (uint8_t)(tmp_data & 0xFF);

        tmp_data = (uint32_t)((current_measurement.elev + 100) * 100);
        msg[4 + 13 * j] = (uint8_t)((tmp_data >> 8) & 0xFF);
        msg[5 + 13 * j] = (uint8_t)(tmp_data & 0xFF);

        tmp_data = (uint32_t)((current_measurement.doppler + 100) * 100);
        msg[6 + 13 * j] = (uint8_t)((tmp_data >> 8) & 0xFF);
        msg[7 + 13 * j] = (uint8_t)(tmp_data & 0xFF);

        tmp_data = (uint32_t)(current_measurement.snr);
        msg[8 + 13 * j] = (uint8_t)((tmp_data >> 8) & 0xFF);
        msg[9 + 13 * j] = (uint8_t)(tmp_data & 0xFF);

        tmp_data = (uint32_t)((current_measurement.angle + 100) * 100);
        msg[10 + 13 * j] = (uint8_t)((tmp_data >> 8) & 0xFF);
        msg[11 + 13 * j] = (uint8_t)(tmp_data & 0xFF);
      }
    }

    if ((stat == canOK) && !willExit) {
      std::cout << "ID 0x" << std::hex << tx_id << std::endl;
      stat = canWrite(hnd, tx_id, msg, 64,
                      canFDMSG_FDF | canFDMSG_BRS | canMSG_EXT);

      if (tx_id == PEAK_CANID_END) tx_id = PEAK_CANID_START;

      if (errno == 0) {
        check("\ncanWrite", stat);
      } else {
        perror("\ncanWrite Error");
      }
    }
    if (stat == canOK) {
      // sleep(1);
      ++tx_id;
    }

    memset(msg, 0, 64);
  }
}

int main(int argc, char *argv[]) {
  // 打开CSV文件
  std::ifstream file(
      "/home/rongxi/MATLAB_Workspace/FrontRADAR/dataset/匀速40kph/"
      "Peak_16_44_14_954.csv");
  if (!file.is_open()) {
    std::cerr << "Error opening CSV file." << std::endl;
    return 1;
  }

  std::string line;
  std::getline(file, line);

  std::vector<GTRACK_measurement_vector> Peak_Mes;
  GTRACK_measurement_vector current_measurement;

  int peak_count = 0;
  int current_frame = -1;

  while (std::getline(file, line)) {
    std::istringstream iss(line);
    std::string token;

    int column = 0;
    while (std::getline(iss, token, ',')) {
      switch (column) {
        case 0:  // no
          break;
        case 1:  // peak_count
        {
          int frame = std::stoi(token);
          if (frame != current_frame) {
            current_frame = frame;
            peak_count++;
          }
        } break;
        case 3:  // range
          current_measurement.range = std::stof(token);
          break;
        case 5:  // azimuth
          current_measurement.azimuth = std::stof(token);
          break;
        case 6:  // elev
          current_measurement.elev = std::stof(token);
          break;
        case 4:  // doppler
          current_measurement.doppler = std::stof(token);
          break;
        case 7:  // snr
          current_measurement.snr = std::stof(token);
          break;
        case 8:  // angle
          current_measurement.angle = std::stof(token);
          break;
      }
      column++;
    }

    Peak_Mes.push_back(current_measurement);
  }

  std::cout << "peak_count: " << peak_count << std::endl;
  // for (const auto& measurement : Peak_Mes)
  // {
  //     std::cout << "Range: " << measurement.range
  //               << " Azimuth: " << measurement.azimuth
  //               << " Elevation: " << measurement.elev
  //               << " Doppler: " << measurement.doppler
  //               << " SNR: " << measurement.snr
  //               << " Angle: " << measurement.angle << std::endl;
  // }

  file.close();

  /*******************************************************************/

  canHandle hnd;
  canStatus stat;
  char msg_[64] = "kvaser";

  int channel;
  struct sigaction sigact;

  if (argc != 2) {
    printUsageAndExit(argv[0]);
  }

  {
    char *endPtr = NULL;
    errno = 0;
    channel = strtol(argv[1], &endPtr, 10);
    if ((errno != 0) || ((channel == 0) && (endPtr == argv[1]))) {
      printUsageAndExit(argv[0]);
    }
  }

  /* Use sighand as our signal handler for SIGALRM */
  sigact.sa_flags = SA_SIGINFO | SA_RESTART;
  sigemptyset(&sigact.sa_mask);
  sigact.sa_sigaction = sighand;
  if (sigaction(SIGALRM, &sigact, NULL) != 0) {
    perror("sigaction SIGALRM failed");
    return -1;
  }
  /* Use sighand and allow SIGINT to interrupt syscalls */
  sigact.sa_flags = SA_SIGINFO;
  if (sigaction(SIGINT, &sigact, NULL) != 0) {
    perror("sigaction SIGINT failed");
    return -1;
  }

  printf("Writing CAN FD messages on channel %d\n", channel);

  canInitializeLibrary();

  /* Open channel, set parameters and go on bus */
  hnd = canOpenChannel(channel, canOPEN_CAN_FD | canOPEN_REQUIRE_EXTENDED |
                                    canOPEN_ACCEPT_VIRTUAL);
  if (hnd < 0) {
    printf("canOpenChannel %d", channel);
    check("", (canStatus)hnd);
    return -1;
  }
  //   stat = canSetBusParams(hnd, canFD_BITRATE_1M_80P, 0, 0, 0, 0, 0);
  //   check("canSetBusParams", stat);
  //   if (stat != canOK) {
  //     goto ErrorExit;
  //   }
  stat = canSetBusParamsFd(hnd, canFD_BITRATE_2M_80P, 0, 0, 0);
  check("canSetBusParamsFd", stat);
  if (stat != canOK) {
    goto ErrorExit;
  }
  stat = canBusOn(hnd);
  check("canBusOn", stat);
  if (stat != canOK) {
    goto ErrorExit;
  }

  alarm(ALARM_INTERVAL_IN_S);

  // for (auto elem : Peak_Mes) {
  //   // test
  //   Out_Put_Peak_List(&elem, 300, hnd, stat);
  //   std::cout << std::endl;
  // }
  // For vcan, max 300*64Bytes
  // for (int i = 0; i < 2; ++i) {
  //   Out_Put_Peak_List(&Peak_Mes[i], 300, hnd, stat);
  //   std::cout << "Peak Count " << i << std::endl;
  // }

  // Out_Put_Peak_List(&Peak_Mes[0], 300, hnd, stat);

  while ((stat == canOK) && !willExit) {
    long int id = 100000;
    stat =
        canWrite(hnd, id, msg_, 6, canFDMSG_FDF | canFDMSG_BRS );

    if (errno == 0) {
      check("\ncanWrite", stat);
    } else {
      perror("\ncanWrite Error");
    }

    if (stat == canOK) {
      msgCounter++;
      sleep(1);
    }
  }

  sighand(SIGALRM, NULL, NULL);

ErrorExit:

  stat = canBusOff(hnd);
  check("canBusOff", stat);
  stat = canClose(hnd);
  check("canClose", stat);
  stat = canUnloadLibrary();
  check("canUnloadLibrary", stat);

  return 0;
}
