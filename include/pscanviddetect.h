#ifndef _PSCANVIDDETECT_H
#define _PSCANVIDDETECT_H

#include "pscanDetectCommon.h"

#pragma pack(1)
struct TEMPLATE
{
    float LeftFreq;
    float RightFreq;    
    int   Left_point;
    int   Right_point;
    float Snr;
    int   CH;
    float percentage;

    vector<int> Timeind;
    Mat   signal;
};
struct vid_template
{
    int Left_point;
    int Right_point;
    int Upper_point;
    int Down_point;
    float freq;
    float Lf;
    float Rf;
    float time;
    float Ut;
    float Dt;
    float bw;
    float snr;
    Mat snr_template;
    vector<int> Timeind;
    float percentage = 0;
};


#pragma pack()
int Template_detect(vector<TEMPLATE> &Template, Mat Amp,int nRows,int nCols,int antennaNum, float cfreq, float freqResolution, int gain);
int PWM_detect(Mat Amp, vector<TEMPLATE> &Template, int nRows, float cfreq, float freqResolution);
int TargetAndPulse(vector<struct TEMPLATE> &Template, vector<struct detect_pulse> &outvidtarget,
                                   vector<struct detect_pulse> &outvidpulse,Mat Amp, float cfreq, float freqResolution , float timeResolution);
int vid_detect(vector<struct TEMPLATE> &, float * ,struct SHORT_DATA_HEAD, vector<struct detect_pulse> &, vector<struct detect_pulse> &);

#endif
