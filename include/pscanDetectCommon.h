#ifndef _PSCANDETECTCOMMON_H
#define _PSCANDETECTCOMMON_H

#include <stdio.h>
#include <math.h>
#include <vector>
#include <sys/time.h>
#include <ctime>
#include <sys/stat.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/core.hpp"
#include <opencv2/imgproc.hpp>
#include "main.h"
using namespace cv;
using namespace std;

#pragma pack(1)
struct detect_pulse
{
    float time;
    float fre;
    float bw;
    float pw;
    float start_time;
    float end_time;
    float start_fre;
    float end_fre;
    float SNR;
    float Prob;
    float meanAmp;
    float meanPhase;
    float PWM;
};

struct SHORT_DATA_HEAD
{

    unsigned int frameCount;
    unsigned long long freq;
    int gain;
    int total_packets;
    unsigned int resolution;
    int rows;
    int cols;
    double timeRresol;
};

typedef struct
{
    unsigned int frameCount;
    float cenFreqOnAllCh[5];          // 通道中心频率MHz
    float gainOnAllCh[5];             // 通道增益
    char childBoardTemperature[5];    // 子板温度
    char A_BoardTemperature;          // A板温度
    char B_BoardTemperature;          // B板温度
    int lngLatValid;                  // GPS原始有效（包括经纬度以及时戳）：0; GPS修改有效（包括经纬度以及时戳）：1；-1无效
    unsigned long long int timeStamp; // GPS时间
    float longitude;                  // 经度
    float latitude;                   // 纬度
    float longitudeOriginal;          // 经度    //自动加库该字段为行数
    float latitudeOriginal;           // 纬度    //自动加库该字段为列数
    int angleValid;                   // 航向角原始有效（不加磁偏角）：0;航向角原始有效（加磁偏角）：1;
                                      // 航向角修改有效（不加磁偏角）：2;航向角修改有效（加磁偏角）：3
                                      // 差分gps原始有效 : 4; 722gps原始有效 ：5;
                                      // 转台角度有效 : 6; 无效 : -1;
    float compassAngle;               // 罗盘角度（已/100）
    float compassAngleOriginal;       // 罗盘角（已/100）
    char FPGA_BoardA_Version[12];     // FPGA的A板程序版本
    char FPGA_BoardB_Version[12];     // FPGA的B板程序版本
                                      // float				   box_t;  //新加温度
                                      // float				   box_h;  //新加湿度
    char deviceType;
    unsigned char answerYear;
    unsigned char answerMon;
    unsigned char answerDay;
    unsigned char answerVersionId;
    unsigned char resv;
    short resv1;
} devParam_t;

struct shortCorrData
{
    devParam_t deviceParam;
    unsigned int frameCount;
    unsigned long long freq; // 频点
    int gain;
    int total_packets;
    unsigned int resolution;
    int rows;
    int cols;
    double timeResol;
    float scorr_buff[4016 * 2 * 300 * 4 * 2 * sizeof(float)];
};
#pragma pack()

bool comp(const detect_pulse &a, const detect_pulse &b);

#endif