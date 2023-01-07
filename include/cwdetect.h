#ifndef CWDETECT_H
#define CWDETECT_H
#include "vector"
#include "pscanctrdetect.h"
#include "pscanviddetect.h"
#include "pscanDetectCommon.h"

using namespace std;

int cwDetect(int *sumCorr, float freqcenter, int *udpData, vector<struct detect_pulse> outcwpulse, int workmode);
int shortsumcorr_cpu(unsigned int *data, float ***Amp, int nrows, int ncols, float *dataAmp);
void freeGrid(float ***p, int m, int n, int k);
float ***createGrid(int m, int n, int k);
#endif // CWDETECT_H
