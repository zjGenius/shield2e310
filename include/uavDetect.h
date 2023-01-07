#ifndef _UAVDETECT_H
#define _UAVDETECT_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "readDetectParam.h"
#include "sharedFuncs.h"
#include "main.h"
#include <pthread.h>
// 配置来源选择
#define INI_PARAM // ini param or code

// 机型库来源选择
// #define Code_LIB // uav code lib
#define INI_LIB // local ini sql
// #define SQL_LIB   //database
#ifndef SQL_LIB
#define FreeSpace 0.1 // free space left, G
#endif

#ifdef SQL_LIB
#define FreeSpace 10
#endif

// #pragma pack(1)

// 定义行方向无人机图传检测结果的结构体
struct uavRow
{
    int id;                          // 无人机脉冲ID，不同天线同一无人机ID号相同
    int uavIndex;                    // 无人机在特征库内的编号
    int antIndex;                    // 天线编号
    int onAnt[NCh];                  // 是否在相应天线上检出
    int colIndex;                    // 单列脉冲所在列索引或最大幅度脉冲所在列索引
    int allColIndex[NFFT / NSumCol]; // 记录所有同种脉冲列索引
    int nCol;                        // 同种脉冲列的个数
    int q1[MaxPulseInGroup];         // 脉冲起点索引
    int w[MaxPulseInGroup];          // 脉冲宽度索引
    float meanColAmp;                // 单列脉宽内的幅度平均值或最大单列幅度平均值
    float snr;
    int nw; // 检测到的脉宽个数
};

// 遥控脉冲相位参数结构体
struct pulseParam
{
    int antIndex;       // 天线编号
    float freq;         // 脉冲频点,MHz
    float pulseTime;    // 脉冲时间,ms
    float pulseW;       // 脉宽,ms
    float pulseBW;      // 带宽,MHz
    float meanAmp[NCh]; // 脉冲平均幅度,dB
    float maxAmp[NCh];
};

// 定义无人机遥控脉冲结构体
struct ctrPulse
{
    int antIndex;       // 天线编号
    float freq;         // 脉冲频点,MHz
    float pulseTime;    // 脉冲时间,ms
    float pulseW;       // 脉宽,ms
    float pulseBW;      // 带宽,MHz
    float meanAmp[NCh]; // 脉冲平均幅度,dB
};

// 定义无人机遥控脉冲串结构体
struct pulseGroup
{
    int id;                                                     // 无人机脉冲id
    int uavIndex;                                               // 无人机在特征库内的编号
    float azimuth;                                              // 无人机遥控方位角
    float range;                                                // 无人机遥控距离
    int onAnt[NCh];                                             // 是否在相应天线上检出
    int antFace[MaxPulseInGroup];                               // 面对的天线，测角选择的天线
    float freq[MaxPulseInGroup];                                // 跳频点脉冲频率,MHz
    float pulseBW;                                              // 带宽,MHz
    float pulseTime[MaxPulseInGroup];                           // 脉冲时间,ms
    float pulseW[MaxPulseInGroup];                              // 脉宽,ms
    float meanAmp[NCh][MaxPulseInGroup];                        // 所有天线上的所有脉冲平均幅度,dB
    float meanAmpComplex[NCh][MaxPulseInGroup];                 // 所有天线上的所有脉冲复数平均幅度,dB
    float meanPhase[NCh][MaxPulseInGroup];                      // 所有天线上的所有脉冲的平均相位，rad
    float phaseVar[NCh][MaxPulseInGroup];                       // 所有天线上所有脉冲的相位方差
    float phaseHist[NCh][MaxPulseInGroup][PhaseBinNum];         // 相位直方图
    float weightedPhaseHist[NCh][MaxPulseInGroup][PhaseBinNum]; // 加权相位直方图
    float angle[MaxPulseInGroup];                               // 脉冲的方位角,度
    float distance[MaxPulseInGroup];                            // 脉冲计算的距离,m
    char possibility;                                           // 目标的置信度，%
    int nPulse;                                                 // 脉冲个数
};

// 日期结构体
struct dateTime
{
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int seconds;
    int nanoSecond;
};

// 跟踪记录用的目标信息
struct objInfo
{
    int objID;                                       // 目标ID号
    int uavIndex;                                    // 目标在特征库内的编号
    unsigned int frameCount;                         // 上次相关数据的帧计数
    float countTime;                                 // 上次相关数据的帧计数代表的时间，ms
    int sourceIndex;                                 // 目标信息来自图传脉冲串2,来自遥控脉冲串1
    int pulseIndex;                                  // 目标信息来自脉冲串的索引
    struct dateTime firstDetectedTime;               // 首次检测到的时间
    struct dateTime detectedTime[MaxHistoryNum];     // 每次检测结果的时间
    float freq[MaxHistoryNum];                       // 中心频率,MHz
    float azimuth[MaxHistoryNum];                    // 方位角,度
    float range[MaxHistoryNum];                      // 无人机的距离,m
    float pitchAngle[MaxHistoryNum];                 // 无人机的俯仰角，度
    float pulseBW[MaxHistoryNum];                    // 带宽
    float pulseAmp[MaxHistoryNum];                   // 脉冲信号强度
    float pulseTime[MaxHistoryNum][MaxPulseTimeNum]; // 脉冲时间，ms
    float pulseW[MaxHistoryNum][MaxPulseTimeNum];    // 脉宽
    unsigned char possibility[MaxHistoryNum];
    int num;                             // 当前存储的历史记录条数
    int count;                           // 自首次出现以来出现的总次数
    int isUpdated;                       // 本次是否更新
    float compCorrAmp[MaxCompassAngNum]; // 盾天线幅度
    int validAmpN;                       // 有效幅度个数
    int angleConfi;
};

/*
//单个目标信息用于显示
struct OneObjInfo
{
    int objID;//目标ID号
    char name[100];//无人机目标名称
    float azimuth;//无人机方位角,-1为无效
    float pitchAngle;//无人机的俯仰角，度
    float range;//无人机距离
    float longitude;//无人机所在的经度，度
    float latitude;//无人机所在的纬度，度
    int height;//无人机的海拔高度，m
    float freq;//无人机中心频率，MHz
    float pulseBW;//带宽，MHz
    float pulseW;//目标的脉宽,ms
    float pulseAmp;//目标信号强度，dB
    float pulseTime[MaxVidPulse];//脉冲时间
    float pulseWs[MaxVidPulse];//脉宽，ms
    float freqPoints[MaxVidPulse];//频点,MHz
    int nPulse;//脉冲个数
    unsigned char possibility;//目标的置信度，0-100
    int objDuration;//无人机存在时间，秒
    unsigned char wHour;//时
    unsigned char wMinute;//分
    unsigned char wSecond;//秒
    unsigned char typ;//类型，wifi3，图传2,遥控1,其他0
};

//发送到qt的目标信息
struct ObjForUI
{
    unsigned int frameCount;//相关数据的第一个帧计数
    int amount;
    struct OneObjInfo obj[MAX_PLANE_AMOUNT];
};

*/
// 设备参数
struct deviceParam
{
    unsigned int frameCount;          // 第一行数据的帧计数
    float cenFreqOnAllCh[NCorr];      // 通道中心频率
    float gainOnAllCh[NCorr];         // 通道增益
    int childBoardTemperature[NCorr]; // 子板温度
    int A_BoardTemperature;           // A板温度
    int B_BoardTemperature;           // B板温度
    struct dateTime gpsTime;          // GPS时间
    int lngLatValid;                  // 经纬度是否有效，-1:无效，0：从罗盘获取，1：手动设置
    float longitude;                  // 经度
    float latitude;                   // 纬度
    int angleValid;                   // 罗盘角度状态，-1:无效，0：从罗盘获取，1：手动设置
    float compassAngle;               // 罗盘角度
    char FPGA_BoardA_Version[12];     // FPGA的A板程序版本
    char FPGA_BoardB_Version[12];     // FPGA的B板程序版本
    char decim;                       // 433和900M频段信号采样抽取，带宽和采样率降低
    unsigned int timeCount;           // 时间计数
    char timeCountVersion;            // 1表示帧计数在sumCorr第一个int32位，0.5秒间隔9375。2表示抽取后的版本，帧计数0.5秒间隔5e4
    int turntableAngle;               // 转台角
};

// 单个目标信息用于显示
struct OneObjInfo
{
    int objID;                     // 目标ID号
    char name[100];                // 无人机目标名称
    float azimuth;                 // 无人机方位角,-1为无效
    float pitchAngle;              // 无人机的俯仰角，度
    float range;                   // 无人机距离
    float longitude;               // 无人机所在的经度，度
    float latitude;                // 无人机所在的纬度，度
    int height;                    // 无人机的海拔高度，m
    float freq;                    // 无人机中心频率，MHz
    float pulseBW;                 // 带宽，MHz
    float pulseW;                  // 目标的脉宽,ms
    float pulseAmp;                // 目标信号强度，dB
    float pulseTime[MaxVidPulse];  // 脉冲时间
    float pulseWs[MaxVidPulse];    // 脉宽，ms
    float freqPoints[MaxVidPulse]; // 频点,MHz
    int nPulse;                    // 脉冲个数
    unsigned char possibility;     // 目标的置信度，0-100
    int objDuration;               // 无人机存在时间，秒
    unsigned char wHour;           // 时
    unsigned char wMinute;         // 分
    unsigned char wSecond;         // 秒
    unsigned char typ;             // 类型，wifi3，图传2,遥控1,其他0
};

// 发送到qt的目标信息
struct ObjForUI
{
    unsigned int frameCount; // 相关数据的第一个帧计数
    int amount;
    struct OneObjInfo obj[MAX_PLANE_AMOUNT];
};

typedef struct siteInfo
{
    float longitude;    ///< 监测站所在的经度，度
    float latitude;     ///< 监测站所在的纬度，度
    int height;         ///< 监测站的海拔高度，m
    short status;       ///< 设备工作状态，0:空闲，1:工作
    float angle;        ///< 系统方位角，度
    char compassStatus; ///< 罗盘工作状态，-1异常，0原始正常（未修正磁偏角），1原始正常（修正磁偏角）2修改正常（未修正磁偏角），3修改正常（修正磁偏角）
    char gpsStatus;     ///< GPS工作状态,-1异常，0原始正常，1修改正常
} siteInfo_t;

typedef struct singleSiteObj
{
    int objID;          ///< 目标ID号
    char SerialNum[64]; /// 序列号
    char name[64];      ///< 无人机目标名称
    unsigned short pulesTime[16];
    unsigned int frameCount;
    float azimuth;                    ///< 无人机方位角,-1为无效
    float range;                      ///< 无人机距离
    float longitude;                  ///< 无人机所在的经度，度
    float latitude;                   ///< 无人机所在的纬度，度
    float height;                     ///< 无人机的海拔高度，m
    float freq;                       ///< 无人机中心频率，kHz
    float pulseBW;                    ///< 目标带宽，kHz
    float pulseW;                     ///< 目标脉宽，ms
    float pulseAmp;                   ///< 目标信号强度，dB
    unsigned short pulseTimeCount[9]; ///< 脉冲时间微秒，第一个是起点时间毫秒数（16:30:04:100则是4*1000+100=4100毫秒）。后面8个是相对时间计数，5微秒*计数
    unsigned char possibility;        ///< 目标的置信度，0-100
    double objDuration;               ///< 无人机信号持续时间，微秒
    unsigned char wHour;              ///< 时
    unsigned char wMinute;            ///< 分
    unsigned char wSecond;            ///< 秒
    unsigned short wMilliseconds;     ///< 毫秒
    unsigned char typ;                ///< 类型，图传2,遥控1,其他0
    unsigned char mode;               ///< 调制识别信息
} singleSiteObj_t;

typedef struct message
{
    struct siteInfo stationInfo;
    int amount;
    struct singleSiteObj obj[MAX_PLANE_AMOUNT];
} message_t;

typedef struct udp_messagePtr
{
    int flag;
    struct message messagePtr;
} udp_obj;
// 滤波函数参数结构体
struct filterParam
{
    float **matIn;   // 输入矩阵
    float **matIn2;  // 输入矩阵
    int m;           // 矩阵行数
    int n;           // 矩阵列数
    int n1;          // 滤波左边界
    int n2;          // 右边界
    float **matOut;  // 输出矩阵
    float **matOut2; // 输出矩阵
    float *mask;     // 滤波模板
    int h;           // 模板高度
    int w;           // 模板宽度
    int edgeFlag;    // 边界是否处理，0则不处理，>0则处理
};
// #pragma pack()

// void extractBox(struct boxData *, float ***, float ***, float *, float, float);
float findThresh(float *, int, float, int, float *);

// void updateBoxdata(struct boxData &, int &, int);

char evalPr(struct objInfo);
int uavDetect(char *ip, int *sumCorr, float freqcenter, int *udpDatah, struct UAVLib *UAVtypes, struct detectParams detectedParam, int nUAV, int workmode);
void vidDetect(struct pulseGroup *, int *, float *, float (*sumCorrAmp)[128], float (*sumCorrPhase)[128], int, int, float *, float *, float, float, struct UAVLib *, int, struct detectParams);
void ctrDetect(struct pulseGroup *, int *, float *, float (*sumCorrAmp)[128], float (*sumCorrPhase)[128], int, int, float *, float *, float, float, struct UAVLib *, int, struct detectParams);
int mark2Pulse(struct ctrPulse *, int **, float ***, int, int, int, int, float, float, int, float *, struct detectParams);
// void setDateTime(struct tm *, struct dateTime *);
// int getDiffTime(struct tm *, struct dateTime);
int getDiffDateTime(struct dateTime, struct dateTime);
int check_free_space2(const int free_space, char *filename);
// void robust_stat(float *Real, float *Imag, int vec_len, float *mean_Real, float *mean_Imag);

#endif
