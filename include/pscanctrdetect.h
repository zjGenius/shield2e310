#ifndef _PSCANCTRDETECT_H
#define _PSCANCTRDETECT_H

#include "pscanDetectCommon.h"

using namespace cv;
using namespace std;

#pragma pack(1)
struct ctrPulses
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
    float meanAmp[NCh];
    float meanPhase[NCh];
    float PWM;
};

struct index
{
    vector<int> x;
    vector<int> y;
    int num = 0;
};

struct ctr_end_Pulses
{
    float start_freq_err;
    float end_freq_err;
    float bw_err;
    int CH;
};

#pragma pack()

void timeBegin();
void timeEnd(std::string s);
int rectwave_filter_Opencv(Mat &, float *, int, int, int, int);
int narrowband_pulse_detect_v2(float *, vector<ctrPulses> &, vector<ctrPulses> &, int, int, int, double, double, double);
int ctr_detect(float *, struct SHORT_DATA_HEAD, vector<detect_pulse> &, vector<detect_pulse> &s);
#endif
