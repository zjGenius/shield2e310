#ifndef TRAP_COMMON_H
#define TRAP_COMMON_H

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <netinet/tcp.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/types.h>
#include <asm/byteorder.h>
#include "paramRead.h"
using namespace std;

#define TRAP_MAX_COUNT 10
#define _PACK_INI_FILE "package.ini"
#define _TRAP_INI_FILE "trap.ini"

#define Jammer_Max_Times 10
#define Heart_Max_Times 5
#define Time_Max_Diff 10
#define JAMMER_CONTROL_FLAG 0x44332211

//和诱骗设备交互的数据结构
#pragma pack(1)
typedef struct 
{
    uint32_t head;
    uint8_t orderNumber;
    uint8_t orderType;
    uint16_t len;
}PackHead;

typedef struct 
{
    PackHead packHead;
    uint8_t sn[16];
    uint32_t curDate;
    uint32_t curTime;
    double curLon;
    double curLat;
    double curHeight;
    uint8_t GPSFreqState;
    uint8_t GLOFreqState;
    uint8_t BDFreqState;
    uint8_t GpsEPHState;
    uint8_t GloEPHState;
    uint8_t BdEPHState;
    uint8_t SimState;
    uint8_t bak[16];
}TrapStatusInfo;

//开关控制
typedef struct 
{
    PackHead packHead;
    uint8_t type;                   ///< 频点类型，0-全部，1-GPS,2-GLO,3-BDS
    bool enable;                    ///< true开,false关
}SwitchControlOrder;
//功率控制
typedef struct
{
    PackHead packHead;
    uint8_t value;
}PowerControlOrder;
#pragma pack()

//和界面交互的数据结构
#pragma pack(1)
typedef struct TrapControlInfo
{
    int8_t cmd_id;                  ///< 命令id
    int8_t switchState;             ///< 开关状态
    uint32_t ipInteger;             ///< 整型IP
    char resr[4];                   ///< 预留位
}TrapControlInfo;

typedef struct WhiteStateInfo
{
    int id;
    char state;
    char resv[10];
}WhiteStateInfo;

typedef struct TrapStatusToServer
{
    uint32_t id;
    float lon;
    float lat;
    float height;
    char gpsSwitch;
    char gloSwitch;
    char bdSwitch;
    char gpsStarState;
    char gloStarState;
    char bdStarState;
    char state;
    char errState;
    char resv[9];
}TrapStatusToServer;

typedef struct TrapPowerInfo
{
    int8_t cmd_id;                  ///< 命令id
    int8_t value;                   ///< 开关状态
    uint32_t ipInteger;             ///< 整型IP
    char resr[4];                   ///< 预留位
}TrapPowerInfo;

#pragma pack()

enum JammerStatus
{
    js_disconnect,
    js_connect,
    js_waitconnect,
};

enum JammerFlag
{
    jf_white,
    jf_manual,
};


#endif