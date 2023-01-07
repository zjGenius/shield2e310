#ifndef WHITE_MODULE_H
#define WHITE_MODULE_H

#include <iostream>
#include <string>
#include <mutex>
#include <thread>
#include <chrono>
#include <vector>
#include "unistd.h"
#include "paramRead.h"
#include <sys/times.h>  // times
#include <sys/types.h>  // pid_t
#include <termios.h>    //termios, tcgetattr(), tcsetattr()
#include <sys/ioctl.h>  // ioctl
#include <stdio.h>      // printf
#include <fcntl.h>      // open
#include <string.h>     // bzero
#include <stdlib.h>     // exit
//串口结构
typedef struct
{
    char      prompt;    // prompt after reciving data
    int       baudrate;  // baudrate
    char      databit;   // data bits, 5, 6, 7, 8
    char      debug;     // debug mode, 0: none, 1: debug
    char      echo;      // echo mode, 0: none, 1: echo
    char      fctl;      // flow control, 0: none, 1: hardware, 2: software
    char      tty;       // tty: 0, 1, 2, 3, 4, 5, 6, 7
    char      parity;    // parity 0: none, 1: odd, 2: even
    char      stopbit;   // stop bits, 1, 2
    const int reserved;  // reserved, must be zero
} portinfo_t;
typedef portinfo_t *pportinfo_t;

class UartControl
{
private:
    int mFdcom = -1;
    char mUartName[64];
    int mBaudrate;
public:
    explicit UartControl(char *uartName,int baudrate);
    int convbaud(unsigned long int baudrate);
    int PortOpen(pportinfo_t pportinfo, char *ptty);
    int PortSet(const pportinfo_t pportinfo);
    void PortClose();
    int PortSend(char *data, int datalen);
    int PortRecv(char *data, int datalen);

    bool fdValid() const {
        mFdcom == -1?false:true;
    }
};

#endif