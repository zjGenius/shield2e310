#ifndef WHITE_COMMON_H
#define WHITE_COMMON_H

#include <stdio.h>      // printf
#include <fcntl.h>      // open
#include <string.h>     // bzero
#include <stdlib.h>     // exit
#include <sys/times.h>  // times
#include <sys/types.h>  // pid_t
#include <termios.h>    //termios, tcgetattr(), tcsetattr()
#include <unistd.h>
#include <sys/ioctl.h>  // ioctl
#define TTY_DEV "/dev/ttyUSB1"  //端口路径
#define _TRAP_INI_FILE "trap.ini"

#define TIMEOUT_SEC(buflen, baud) (buflen * 20 / baud + 2)  //接收超时
#define TIMEOUT_USEC 0
#define BUF_SIZE 4097

#pragma pack(1)
typedef struct
{
    unsigned int head;                  //包头 0xEEEEEEEE
    unsigned short version;             //版本 0x10
    unsigned int len;                   //数据长度
    unsigned short year;              
    unsigned char mon;
    unsigned char day;
    unsigned char hour;
    unsigned char min;
    unsigned char sec;
    unsigned short msec;
    unsigned int packetNum;             //包编号
    unsigned char whiteFlag;            //白名单标志
    unsigned int resv1;                 //预留
    unsigned int resv2;
    unsigned char checkSum;             //校验和
    unsigned int tail;                  //包尾 0xAAAAAAAA
} WhiteListData;

typedef struct WhiteStateToServer
{
    int id;
    char state;
    char resv[10];
}WhiteStateToServer;
#pragma pack()

#endif