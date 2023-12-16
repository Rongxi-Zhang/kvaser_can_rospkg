#include <canlib.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>

#define ALARM_INTERVAL_IN_S (1)
#define READ_WAIT_INFINITE  (unsigned long)(-1)

static unsigned int msgCounter = 0;

static void check(char *id, canStatus stat)
{
    if (stat != canOK) {
        char buf[50];
        buf[0] = '\0';
        canGetErrorText(stat, buf, sizeof(buf));
        printf("%s: failed, stat=%d (%s)\n", id, (int)stat, buf);
    }
}

static void printUsageAndExit(char *prgName)
{
    printf("Usage: '%s <channel>'\n", prgName);
    exit(1);
}

static void sighand(int sig, siginfo_t *info, void *ucontext)
{
    static unsigned int last;
    (void)info;
    (void)ucontext;

    switch (sig) {
    case SIGINT:
        break;
    case SIGALRM:
        if (msgCounter != last) {
            printf("rx : %u total: %u\n", msgCounter - last, msgCounter);
        }
        last = msgCounter;
        alarm(ALARM_INTERVAL_IN_S);
        break;
    }
}

int main(int argc, char *argv[])
{
    canHandle hnd;
    canStatus stat;
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

    printf("Reading CAN FD messages on channel %d\n", channel);

    canInitializeLibrary();

    /* Open channel, set parameters and go on bus */
    hnd = canOpenChannel(channel, canOPEN_CAN_FD | canOPEN_ACCEPT_VIRTUAL);
    if (hnd < 0) {
        printf("canOpenChannel %d", channel);
        check("", (canStatus)hnd);
        return -1;
    }
    // stat = canSetBusParams(hnd, canFD_BITRATE_1M_80P, 0, 0, 0, 0, 0);
    // check("canSetBusParams", stat);
    // if (stat != canOK) {
    //     goto ErrorExit;
    // }
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

    do {
        long id;
        unsigned char msg[64];
        unsigned int dlc;
        unsigned int flag;
        unsigned long time;

        stat = canReadWait(hnd, &id, &msg, &dlc, &flag, &time, READ_WAIT_INFINITE);

        if (stat == canOK) {
            char *can_std;
            unsigned int i;

            msgCounter++;
            if (flag & canMSG_ERROR_FRAME) {
                printf("(%u) ERROR FRAME flags:0x%x time:%lu\n", msgCounter, flag, time);
                continue;
            }

            if (flag & canFDMSG_FDF) {
                if (flag & canFDMSG_BRS) {
                    can_std = "FD+";
                } else {
                    can_std = "FD ";
                }
            } else {
                can_std = "STD";
            }

            printf("CH:%2d %s:%s:%2u:%08lx", channel, can_std, (flag & canMSG_EXT) ? "X" : " ", dlc,
                   id);

            if (flag & canFDMSG_ESI) {
                printf("ESI ");
            }

            printf(" flags:0x%x time:%lu", flag, time);

            for (i = 0; i < dlc; i++) {
                unsigned char byte = msg[i];

                if ((i % 16) == 0) {
                    printf("\n    ");
                }
                printf(" %02x ", byte);
            }
            printf("\n");
        } else {
            if (errno == 0) {
                check("\ncanReadWait", stat);
            } else {
                perror("\ncanReadWait error");
            }
        }

    } while (stat == canOK);

    sighand(SIGALRM, NULL, NULL);

ErrorExit:

    alarm(0);
    stat = canBusOff(hnd);
    check("canBusOff", stat);
    stat = canClose(hnd);
    check("canClose", stat);
    stat = canUnloadLibrary();
    check("canUnloadLibrary", stat);

    return 0;
}
