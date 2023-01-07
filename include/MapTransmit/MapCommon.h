#ifndef MAP_COMMON_H
#define MAP_COMMON_H

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <netinet/tcp.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <asm/unistd.h>
#include <math.h>
#include "paramRead.h"

#define SEND_PORT_NUM 4001  ///<上位机端口
#define RECV_PORT_NUM 4444  ///<接收端口
#define _TIME_INI_FILE "package.ini"
#define TRANSMIT_FILE   "relay.ini"
#define URD360_FILE "~/URD360.sh"
#define FREQLIST_FILE "tfreq.ini"
#define DATA_TYPE_POS 19
#define RMTP_DATA_POS (19 + 2 + 8)
#define Max 20
#define SendAllTimes 500        //ms
#define MaxTimeout   30         //s
#define PI                      3.141592654
#define EARTH_RADIUS            6378.137        //地球近似半径
// Rmtp帧头标志
#define RMTP_MESSAGE_START_FLAG 0xEEEEEEEE
// Rmtp帧尾标志
#define RMTP_MESSAGE_END_FLAG 0xAAAAAAAA

#pragma pack(1)

/*!
 * @brief Rmtp协议报文类型
 */
enum DataType
{
    rdt_invalid = -1,                    ///< 无效
    rdt_order = 0,                       ///< 指令
    rdt_deviceInfo = 1,                  ///< 监测设备信息列表数据
    rdt_freqList = 2,                    ///< 监测设备的频段频谱数据
    rdt_droneList = 3,                   ///< “节点”识别到的无人机列表数据
    rdt_spectrum = 4,                    ///< 无人机的典型特征频谱图数据
    rdt_interact = 5,                    ///< 站点相互控制数据
    rdt_reply = 6,                       ///< 包响应(执行结果)
    rdt_dronePosition = 7,               ///< 站点交叉定位的无人机列表数据
    rdt_file = 8,                        ///< 文件
    rdt_gps = 9,                         ///< GPS信道环境监测数据
    rdt_serialPort = 10,                 ///< 串口数据转发
    rdt_externalDeviceSend = 13,         ///< 通用驱动发送
    rdt_externalDeviceRecv = 13,         ///< 通用驱动接收
    rdt_timestamp = 16,                  ///< 对时报文
    rdt_trapcontrol = 20,                ///< 诱骗控制
    rdt_trappowercontrol = 23,           ///< 诱骗功率控制
};

/** @brief 检测设备信息定义*/
typedef struct
{
    int   eqt_id;        ///<1监测设备ID
    char  eqt_name[20];  ///<2设备名称
    float longitude;     ///<3设备所在东经
    float latitude;      ///<4设备所在北纬
    int   height;        ///<5设备所在海拔（米）
    short workstat;      ///<6设备工作状态（0：空闲 1：工作）
    int   sysangle;      ///<7系统方位角
    char  eqt_type[4];   ///<8设备类型
    char  runstat[4];    ///<9设备运行状态
    int   range;         ///<10天线覆盖范围（度）
    int   rcvid;         ///<11接收设备ID
    float   para1;         ///<12暂无
    float   para2;         ///<13暂无
    int   para3;         ///<14暂无
    int   pos_num;       ///<15站点编号
}DevData;

/** @brief 单站识别的无人机数据部分定义*/
typedef struct 
{
    int32_t mId;                            ///< 目标ID
    char mUniqueId[32];                  ///< 唯一Id       char[32] fixed bytes in tcp datagram
    int mInfoLen;
    char mInfo[64];                      ///< 目标信息部分长度,目标信息(长度包括在string的size里面)
    int32_t mStationId;                     ///< 监测站ID
    int32_t mAzimuth;                       ///< 监测站识别的目标方向角
    int32_t mRange;                         ///< 目标距离
    float mLongitude;                       ///< 经度 (东经)
    float mLatitude;                        ///< 纬度 (北纬)
    int32_t mAltitude;                      ///< 高度 (海拔)
    double mFrequency;                      ///< 目标使用频率
    double mBandwidth;                      ///< 目标带宽
    double mSignalStrength;                 ///< 目标信号强度
    int8_t mDuration[18];                   ///< 目标信号持续时间
    int8_t mConfidence;                     ///< 置信度
    int8_t mReserve;                        ///< 保留字节
    int8_t mHour;                           ///< 发现时间: 时
    int8_t mMinute;                         ///< 发现时间: 分
    int8_t mSecond;                         ///< 发现时间: 秒
    int16_t mMillisecond;                   ///< 发现时间: 毫秒
    int8_t mType;                           ///< 数据类型
}PlaneData;

//trasmit 协议头
typedef struct PackHead_t
{
    int32_t frameHead;//帧头 0xEEEEEEEE
    int16_t version;//协议版本 16
    int32_t frameLen;//帧长度 1038+34
    int16_t year;
    int8_t mon;
    int8_t day;
    int8_t hour;
    int8_t minute;
    int8_t sec;
    int16_t msec;
    int16_t dataType;//数据类型
    int64_t frameCount;//包编号
}PackHead_t;

// trasmit 协议尾
typedef struct PackTail_t
{
    int8_t checkSum;//校验和
    int32_t frameTail;//帧尾
}PackTail_t;

typedef struct SockInfo
{
    int8_t cmd_id;                  ///< 命令id
    uint32_t ipaddr;                 ///< ip地址
    uint8_t mac[6];                 ///< 物理地址
}SockInfo;

// typedef struct TrapControlInfo
// {
//     int8_t cmd_id;                  ///< 命令id
//     int8_t switchState;             ///< 开关状态
//     uint32_t ip_integer;            ///< 指定ip控制
//     char resr[4];                   ///< 预留位
// }TrapControlInfo;

// typedef struct TrapPowerInfo
// {
//     int8_t cmd_id;                  ///< 命令id
//     int8_t switchState;             ///< 开关状态
//     uint32_t ip_integer;            ///< 指定ip控制
//     char resr[4];                   ///< 预留位
// }TrapPowerInfo;

#pragma pack()

#endif