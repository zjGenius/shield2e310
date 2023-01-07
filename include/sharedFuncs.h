#ifndef _SHAREDFUNCS_H
#define _SHAREDFUNCS_H
#include "readDetectParam.h"

// float amp2dist(float *, int *, float, float *);
// float angleSelMean(float *, int);
// float phaseSelAngle(int *, float *, float *, int *);
// float angleSelAngle(int *, float *, int *);
// float ampSelAngle(int *, float *, float *, int *, int);
// void  phase2Angle(float *, float, float, float *, int);
// void  amp2Angle(float *, float *, float *, int *);
// float selAntParam(float *, float, struct detectParams);
// float selAntParamNew(float *, float, struct detectParams);
// void selAntParamPro(float *, float , struct detectParams , float *);
// void selAntOffAmp(float *, float , struct detectParams);
// float calcAngle(float *, float *, float , float , int *, float *, int &, struct detectParams);
// float calcAnglePro(float *, float *, float , float* , int *, float *, int &, struct detectParams );
// float offampSelAngle(int *, float *, float *, int *, int , float *);

// float ***createGrid(int, int, int);
// void freeGrid(float ***p, int, int, int);
float **createMatrix(int m, int n);
void freeMatrix(float **p, int m);
void filter(float (*matIn)[128], int, int, float (*matout)[128], float *, int, int, int *, int);
// void *filter_thread(void *);
// void *integral_thread(void *);
// void *integral_thread_cv(void *);
// void *integral_thread_cv2(void *);
// void *ampAndIntegral_thread(void *);
float amp2dist(float *, int *, float, float *);
float angleSelMean(float *, int);
// float phaseSelAngle(int *, float *, float *, int *);
// float angleSelAngle(int *, float *, int *);
// float ampSelAngle(int *, float *, float *, int *, int);
// float offampSelAngle(int *, float *, float *, int *, int, float *);
// void  phase2Angle(float *, float, float, float *, int);
// void  amp2Angle(float *, float *, float *, int *);

// float selAntParam(float *, float, struct detectParams &);
// float selAntParamNew(float *, float, struct detectParams &);
void selAntParamPro(float *, float, struct detectParams, float *);
// void selAntOffAmp(float *, float , struct detectParams&);
// void selAntOffAmpPro(float *, float , struct detectParams&);
// void selAntOffAziPro(float *, float , struct detectParams&);
// void selAntOffAmpLOrR(float *, float , struct detectParams&);
// float calcAngle(float *, float *, float , float , int *, float *, int &, struct detectParams &);
// float calcAnglePro(float *, float *, float, float *, int *, float *, int &, struct detectParams);
#endif
