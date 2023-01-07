#include "uavDetect.h"
#include "paramRead.h"
// #include <opencv2/highgui.hpp>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/highgui/highgui_c.h>
// #include <vector>
#include "sharedFuncs.h"

// /*
//    动态申请一个大的三维数组
//  */
// float ***createGrid(int m, int n, int k)
// {
//     int      i, j;
//     float ***p = NULL;
//     p = (float ***)malloc(m * sizeof(float **));
//     for (i = 0; i < m; i++)
//     {
//         p[i] = (float **)malloc(n * sizeof(float *));
//         for (j = 0; j < n; j++)
//         {
//             p[i][j] = (float *)malloc(k * sizeof(float));
//         }
//     }
//     return p;
// }

// /*
//    释放大的三维数组的内存
//  */
// void freeGrid(float ***p, int m, int n, int k)
// {
//     int i, j;
//     if (p != NULL)
//     {
//         for (i = 0; i < m; i++)
//         {
//             for (j = 0; j < n; j++)
//             {
//                 free(p[i][j]);
//             }
//             free(p[i]);
//         }
//         free(p);
//     }
// }

// /*

//  */
/***
 * @description:动态申请一个大的二维数组
 * @param[in] m 第一维大小
 * @param[in] n 第二维大小
 * @return 二维指针
 */
 float **createMatrix(int m, int n)
 {
     int i, j;
     float **p = NULL;
     p = (float **)malloc(m * sizeof(float *));
     for (i = 0; i < m; i++)
     {
         p[i] = (float *)malloc(n * sizeof(float *));
     }
     return p;
 }

 /***
  * @description:释放二维矩阵
  * @param[in] p 需要释放的二维指针
  * @param[in] m 第一维大小
  * @return {*}
  */
 void freeMatrix(float **p, int m)
 {
     int i, j;
     if (p != NULL)
     {
         for (i = 0; i < m; i++)
         {
             free(p[i]);
         }
         free(p);
     }
 }

//滤波
// edgeFlag=0，表示边界不处理，edge=1表示处理边界
void filter(float (*matIn)[NFFT / NSumCol], int m, int n, float (*matOut)[NFFT / NSumCol], float *mask, int h, int w, int *nSpan, int edgeFlag)
{
    int i, j, k;

    if (edgeFlag) //处理边界
    {
        for (i = 0; i < m; i++)
        {
            for (j = nSpan[0]; j < nSpan[1]; j++)
            {
                *(*(matOut + i) + j) = 0.0;
                for (k = 0; k < h * w; k++)
                {
                    if (i + k / w - (h - 1) / 2 >= 0 && i + k / w - (h - 1) / 2 < m && j + k % w - (w - 1) / 2 >= 0 && j + k % w - (w - 1) / 2 < n)
                        *(*(matOut + i) + j) += *(*(matIn + (i + k / w - (h - 1) / 2)) + (j + (k % w - (w - 1) / 2))) * mask[k];
                }
            }
        }
    }
    else //不处理边界
    {
        for (i = 0; i < m; i++)
        {
            for (j = nSpan[0]; j < nSpan[1]; j++)
            {
                *(*(matOut + i) + j) = 0.0;
                for (k = 0; k < h * w; k++)
                {
                    if (i >= (h - 1) / 2 && i < m - (h - 1) / 2 && j >= (w - 1) / 2 && j < n - (w - 1) / 2)
                        *(*(matOut + i) + j) += *(*(matIn + (i + k / w - (h - 1) / 2)) + (j + (k % w - (w - 1) / 2))) * mask[k];
                }
            }
        }
    }
}

// //多线程滤波
// // edgeFlag=0，表示边界不处理，edge=1表示处理边界
// void *filter_thread(void *arg)
// {
//     struct filterParam *filterPtr = (struct filterParam *)arg;
//     float **            matIn = filterPtr->matIn;
//     int                 m = filterPtr->m;
//     int                 n = filterPtr->n;
//     int                 n1 = filterPtr->n1;
//     int                 n2 = filterPtr->n2;
//     float **            matOut = filterPtr->matOut;
//     float *             mask = filterPtr->mask;
//     int                 h = filterPtr->h;
//     int                 w = filterPtr->w;
//     int                edgeFlag = filterPtr->edgeFlag;

//     int i, j, k;

//     if (edgeFlag)  //处理边界
//     {
//         for (i = 0; i < m; i++)
//         {
//             for (j = n1; j < n2; j++)
//             {
//                 *(*(matOut + i) + j) = 0.0;
//                 for (k = 0; k < h * w; k++)
//                 {
//                     if (i + k / w - (h - 1) / 2 >= 0 && i + k / w - (h - 1) / 2 < m && j + k % w - (w - 1) / 2 >= 0 && j + k % w - (w - 1) / 2 < n)
//                         *(*(matOut + i) + j) += *(*(matIn + (i + k / w - (h - 1) / 2)) + (j + (k % w - (w - 1) / 2))) * mask[k];
//                 }
//             }
//         }
//     }
//     else  //不处理边界
//     {
//         for (i = 0; i < m; i++)
//         {
//             for (j = n1; j < n2; j++)
//             {
//                 *(*(matOut + i) + j) = 0.0;
//                 for (k = 0; k < h * w; k++)
//                 {
//                     if (i >= (h - 1) / 2 && i < m - (h - 1) / 2 && j >= (w - 1) / 2 && j < n - (w - 1) / 2)
//                         *(*(matOut + i) + j) += *(*(matIn + (i + k / w - (h - 1) / 2)) + (j + (k % w - (w - 1) / 2))) * mask[k];
//                 }
//             }
//         }
//     }
//     return NULL;
// }

// //多线程积分图像法
// //处理边界
// void *integral_thread(void *arg)
// {
//     struct filterParam *filterPtr = (struct filterParam *)arg;
//     float **            matIn = filterPtr->matIn;
//     int                 nRows = filterPtr->m;
//     int                 nCols = filterPtr->n;
//     int                 n1 = filterPtr->n1;
//     int                 n2 = filterPtr->n2;
//     float **            matOut = filterPtr->matOut;
//     float *             mask = filterPtr->mask;
//     int                 h = filterPtr->h;
//     int                 w = filterPtr->w;
//     int                edgeFlag = filterPtr->edgeFlag;

//     float* rowIntegral = (float*)malloc(sizeof(float) * nCols);
//     n1 = n1 >= w ? n1 : w;

//     int i, j, k, x1, x2, y1, y2;
//     float tl,tr,bl,br,sumValue;

//     for(j = 0; j < nRows; j++)
//     {
// 	for(i = 0; i < nCols; i++)
// 	{
// 	    rowIntegral[i] = 0;
// 	    matOut[j][i] = 0;
// 	}
// 	for(i = n1 - w; i < n2; i++)
// 	{
// 	    if (i <= n1)
// 	    {
// 		//rowIntegral[n1] = matIn[j][n1];
// 		rowIntegral[n1] = 0;
// 	    }
// 	    else
// 	    {
// 		rowIntegral[i] = matIn[j][i] + rowIntegral[i-1];
// 	    }

// //  	    printf("%0.2f, ", rowIntegral[i]);

// 	    if(j == 0)	matOut[j][i] = rowIntegral[i];
// 	    else
// 	    {
// 		matOut[j][i] = matOut[j-1][i] + rowIntegral[i];
// 	    }
// 	}
// //  	printf("\n");
//     }
//     for (i = nRows - 1; i >= 0; i--)
//     {
//         for (j = n2; j >= n1; j--)
//         {
// 	    x1 = j - w;
//             x2 = j;
//             y1 = i - h;
//             y2 = i;
//             if(x1 < 0)  x1 = 0;
//             if(y1 < 0)   y1 = 0;
//             tl = matOut[y1][x1];
//             tr = matOut[y2][x1];
//             bl = matOut[y1][x2];
//             br = matOut[y2][x2];
//             matOut[i][j] = (br - bl - tr +tl)/(h * w);
//         }
//     }

//     //for(i = floor((h+1)/2); i < nRows; i++)
// //     for(i = 0; i < nRows-h; i++)
// //     {
// // 	*(matOut + i + h) = *(matOut + i);
// // // 	for(j = n1; i < n2; i++)
// // // 	{
// // // 	    //matOut[i - (int)((h+1)/2)][j - (int)((w+1)/2)] = matOut[i][j];
// // // 	    *(*(matOut + i + h*2) + j) = matOut[i][j];
// // // 	    printf("%f\n",matOut[i][j]);
// // // 	}
// //     }
//     free(rowIntegral);
// }

/*
 * 幅度估计距离
 * meanAmp:	脉冲对应的一圈相关的幅度
 * onAnt:	在哪些天上检出目标
 * miu:		距离系数
 * gain:	接收机增益,dB
 */
float amp2dist(float *meanAmp, int *onAnt, float miu, float *gain)
{
    int i;
    float maxAmp = 0;
    int maxIndex;
    for (i = 0; i < 4; i++)
    {
        if (onAnt[i] && meanAmp[i] - (gain[i] - 195) * 0.5 > maxAmp)
        {
            maxAmp = meanAmp[i] - (gain[i] - 195) * 0.5;
            maxIndex = i;
        }
    }
    return pow(10.0, miu - maxAmp / 20.0);
}

// /*
//  * 螺旋天线求俯仰角
//  */
// void pitchAzi(float *amps, float *phis, float *ds, float lambda, float *attena_dirs, int n, float theta0, float &angleResult, struct detectParams param)
// {
//     int max_iters = 5,indW,indMaxAmp;
//     float cosTheta = cos(theta0),sigma_amp = 3, minW = 1000, maxAmp = 0, sumWeight = 0,tmpYs,sumX = 0,sumY = 0, sumNewWeight = 0;
//     float sigmaAmp = 3,aziNew;
//     float weights[3],azis[3],diffAzi[3],newWeights[3],ys[3];
//     for(int i = 0; i < n; i++)
//     {
//         if(maxAmp < amps[i])
//         {
//             maxAmp = amps[i];
//             indMaxAmp = i;
//         }
//         attena_dirs[i] = attena_dirs[i] / 180 * PI;
//         ys[i] = lambda * phis[i] / (2 * PI * ds[i]);
//     }

//     for(int i = 0; i < n; i++)
//     {
//         weights[i] = exp((amps[i] - maxAmp)/sigmaAmp);
//         if (weights[i] < minW)
//         {
//             minW = weights[i];
//             indW = i;
//         }
//         sumWeight += weights[i];
//     }

//     sumWeight = sumWeight - minW;
//     weights[indW] = 0;
//     sumWeight = sumWeight==0 ? 0.1:sumWeight;

//     for(int i = 0; i < n; i++)
//     {
// 	    weights[i] = weights[i] / sumWeight;
//     }

//     for(int iter = 0; iter < max_iters; iter++)
//     {
//         for(int j = 0; j < n; j++)
//         {
//             tmpYs = ys[j]/cosTheta;
//             tmpYs = tmpYs > 1 ? 1 - 1e-3:tmpYs;
//             tmpYs = tmpYs < -1 ? -1 + 1e-3:tmpYs;
//             azis[j] = asin(tmpYs) + attena_dirs[j];
//             sumX += cos(azis[j]) * weights[j];
//             sumY += sin(azis[j]) * weights[j];
//         }
//         aziNew = atan2(sumY,sumX);

//         for(int j = 0; j < n; j++)
//         {
//             diffAzi[j] = fmod(aziNew - attena_dirs[j] + 3 * PI, 2 * PI) - PI;
//             newWeights[j] = weights[j] * exp(-pow(diffAzi[j]/(param.beam_width/2),2)/2) / (exp(-pow(diffAzi[j]/(param.beam_width/2),2)/2) + exp(-2));
//             sumNewWeight += newWeights[j];
//         }

//         for(int j = 0; j < n; j++)
//         {
//             newWeights[j] = newWeights[j] / sumNewWeight;
//             tmpYs = ys[j]/(sin(aziNew - attena_dirs[j])+0.001) * newWeights[j];
//             tmpYs = tmpYs > 1 ? 1 - 1e-3:tmpYs;
//             tmpYs = tmpYs < -1 ? -1 + 1e-3:tmpYs;
//             cosTheta += tmpYs;
//         }

//         cosTheta = cosTheta > 1 ? 1:cosTheta;
//         cosTheta = cosTheta < 0.5 ? 0.5:cosTheta;
//     }

//     aziNew = fmod(aziNew * 180 / PI + 360, 360);

//     if(fmod(angleResult + 360,360 / param.antennaNum) <= 33 && fmod(angleResult + 360,360 / param.antennaNum) >= 12)
// 	    angleResult = aziNew;
// }

// /*
//  * 多项式拟合方法
//  */
// float polyval(float *ampPoly,float diffAmp, int pNum)
// {
//     float val = 0.0;
//     for(int i = 0; i < pNum; i++)
//     {
//         val += ampPoly[i] * pow(diffAmp, pNum - i - 1);
//     }
//     return val;
// }

// /*
//  * 如果两个结果较接近且概率也接近一致时，取两者加权平均，采用meanshift算法
//  */
// // float meanshiftFusion(vector<float>candidateAzis, vector<float>probs, float candSelAzi, float std_azi)
// // {
// //     vector<float> weights(probs.size(),0);
// //     vector<float> centeredCandAzis(probs.size(),0);
// //     float centeredBestAzi = 0.0;
// //     float sumWeight = 0.0;

// //     for(int i = 0; i < candidateAzis.size(); i++)
// //     {
// // 	centeredCandAzis.at(i) = fmod(candidateAzis.at(i) - candSelAzi + 900, 360) - 180;
// //     }

// //     for(int i = 0; i < 5 ;i++)
// //     {
// // 	for(int j = 0; j < probs.size(); j++)
// // 	{
// // 	    weights.at(j) = exp(-pow(centeredCandAzis.at(j),2) / 2 / pow(std_azi,2));
// // 	    weights.at(j) = weights.at(j) * probs.at(j);
// // 	}
// // 	sumWeight = 0.0;
// // 	centeredBestAzi = 0.0;
// // 	for(int j = 0; j < probs.size(); j++)
// // 	{
// // 	    centeredBestAzi += weights.at(j)*centeredCandAzis.at(j);
// // 	    sumWeight += weights.at(j);
// // 	}
// // 	centeredBestAzi = centeredBestAzi / sumWeight;
// // 	candSelAzi = candSelAzi + centeredBestAzi;
// // 	for(int j = 0; j < candidateAzis.size(); j++)
// // 	{
// // 	    centeredCandAzis.at(j) = fmod(candidateAzis.at(j) - candSelAzi + 900, 360) - 180;
// // 	}
// //     }

// //     return candSelAzi;
// // }

// /*
//  * 螺旋天线解模糊,2.0版本
//  */
// // float azimuth_deblur_2(float *d, float lambda, float *meanAmp,float *meanPhase, vector<float>candidateAzis, struct detectParams param, int calibLibInd, vector<float> priorLogProbs, vector<int> candidateAziCh)
// // {
// //     int antennaNum = param.antennaNum, rChInd,minLogProbsInd = 0,maxProbsInd = 0;
// //     float amp_obs_std = param.amp_obs_std; //单位dB
// //     float phi_obs_std = param.phi_obs_std / 180.0 * PI; //param.phi_obs_std单位°
// //     float thisAmp = 0.0, diffAmp = 0.0,estimatedAzi = 0.0,sigma_dir = 0.0, base_prob = 0.0, difFromAttenaDir = 0.0, tmpDifFromAttenaDir = 0.0, difAzi = 0.0, log_probs = 0.0;
// //     float predict_phase = 0.0, slope = 0.0,sigma_phase = 0.0,dif_phase = 0.0,minLogProbs = 1000,sumProbs = 0.0,maxProbs = 0, predict_azi_accuracy = 0.0;
// //     float thisChAmpSpan[2] = {0.0},thisChPhSpan[2] = {0.0};
// //     int valid_channel_no[candidateAzis.size()][2];
// // //     vector<int>aziSamples(360,0);
// //     //iota(aziSamples.begin(),aziSamples.end(),0);
// // //     vector<float>log_probs_all_samples(aziSamples.size(),0);
// //     vector<float>log_probs_by_amps(candidateAzis.size(),1);
// //     vector<float>log_probs_by_phis(candidateAzis.size(),1);
// //     vector<float>unnormalized_log_probs(candidateAzis.size(),1);
// //     vector<float>probs(candidateAzis.size(),1);
// //     vector<int>valid_num_amp_channel(candidateAzis.size(),0);

// //     struct calibrateParamPro calibParam;
// //     calibParam = param.calibParamNew[calibLibInd];
// //     int deblurChN = 0;
// //     if(lambda / (2 * d[0]) >= 1)
// // 	deblurChN = 1;
// //     else
// // 	deblurChN = asin(lambda / (2 * d[0])) / PI *180 > (360 / antennaNum * 1.5) ? 1 : 2;
// // //     printf("%f, %f\n",asin(lambda / (2 * d[0])) / PI *180,  (360 / antennaNum * 1.33));

// //     if(candidateAzis.size() == 0)
// // 	    return -1;
// //     for(int i = 0; i < candidateAzis.size(); i++)
// //     {
// // 	valid_channel_no[i][0] = fmod((candidateAziCh.at(i) - 1) + antennaNum, antennaNum);
// // 	valid_channel_no[i][1] = candidateAziCh.at(i);
// //     }
// //     for(int i = 0; i < antennaNum; i++)
// //     {
// //         rChInd = fmod(i + 1 + antennaNum, antennaNum);
// //         thisAmp = meanAmp[i] - calibParam.maxAmpPattern[i];
// //         diffAmp = meanAmp[rChInd] - calibParam.maxAmpPattern[rChInd] - thisAmp;
// //         for(int j = 0; j < 2; j++)
// //         {
// //             thisChAmpSpan[j] = fmod(calibParam.ampLowUpLimit[i][j] - calibParam.phRefAzi[i] + 900, 360) - 180;
// //         }
// //         estimatedAzi = polyval(calibParam.ampPoly[i],diffAmp,5);//5为多项式次数
// //         sigma_dir = fabs(amp_obs_std * calibParam.ampPoly[i][3]);
// //         sigma_dir = sqrt(pow(sigma_dir,2) + pow(calibParam.patternFitAccuracy[i],2));

// //         sigma_dir = sigma_dir > (lambda / d[i] * 180 / PI / 3)?(lambda / d[i] * 180 / PI / 3):sigma_dir;
// // // 	printf("i = %d,estimatedAzi = %f,thisChAmpSpan[0] = %f,thisChAmpSpan[1] = %f\n",i,estimatedAzi,thisChAmpSpan[0],thisChAmpSpan[1]);

// // 	estimatedAzi = estimatedAzi > thisChAmpSpan[0]?estimatedAzi : thisChAmpSpan[0];//约束计算的比幅角度在可用范围内
// // 	estimatedAzi = estimatedAzi < thisChAmpSpan[1]?estimatedAzi : thisChAmpSpan[1];
// // // 	printf("i = %d,estimatedAzi = %f,thisChAmpSpan[0] = %f,thisChAmpSpan[1] = %f\n",i,estimatedAzi,thisChAmpSpan[0],thisChAmpSpan[1]);

// //         estimatedAzi = fmod(estimatedAzi + calibParam.phRefAzi[i] + 720,360);

// //         base_prob = 1.0 / (fabs(360.0 - 1.0*fabs(thisChAmpSpan[1] - thisChAmpSpan[0]))+ 1e-3);//fabs(thisChAmpSpan[1] - thisChAmpSpan[0]);//1.0 / (360.0 - fabs(1.0));//thisChAmpSpan[1] - thisChAmpSpan[0]

// // //         for(int aziSInd = 0; aziSInd < aziSamples.size(); aziSInd++)
// // //         {
// // //             difAzi = fabs(fmod(aziSamples.at(aziSInd) - estimatedAzi + 900, 360) - 180);
// // //             difFromAttenaDir = fmod(aziSamples.at(aziSInd) - calibParam.phRefAzi[i] + 900, 360) - 180;
// // //             if (difFromAttenaDir >= thisChAmpSpan[0] && difFromAttenaDir <= thisChAmpSpan[1])
// // //             log_probs = -pow(difAzi,2)/pow(sigma_dir,2)/2 - log(2*PI)/2 - log(sigma_dir);
// // //             else
// // //             log_probs = log(base_prob);
// // //             log_probs_all_samples.at(aziSInd) += log_probs;
// // //         }

// //         for(int k = 0; k < candidateAzis.size(); k++)
// //         {
// //             difFromAttenaDir = fmod(candidateAzis.at(k) - calibParam.phRefAzi[i] + 900, 360) - 180;
// // //             if (difFromAttenaDir >= thisChAmpSpan[0] && difFromAttenaDir <= thisChAmpSpan[1])//在可用范围内
// // // 	    if(valid_channel_no[k][0] == i || valid_channel_no[k][1] == i)
// // 	    if((candidateAziCh.at(k) == i && deblurChN == 1) || (valid_channel_no[k][0] == i || valid_channel_no[k][1] == i && deblurChN == 2))
// //             {
// //                 difAzi = fabs(fmod(candidateAzis.at(k) - estimatedAzi + 900, 360) - 180);
// //                 log_probs_by_amps.at(k) = log_probs_by_amps.at(k) - pow(difAzi,2)/pow(sigma_dir,2)/2 - log(2*PI)/2 -log(sigma_dir);
// //                 valid_num_amp_channel.at(k)++;
// //             }
// //             // else
// //             // {
// //             //     //log_probs_by_amps.at(k) = log_probs_by_amps.at(k) + log(base_prob);
// //             // }
// //         }
// //     }

// // //     printf("**********************************\n");
// // //     printf("log_probs_by_amps = ");
// // //     for(int i = 0; i < log_probs_by_amps.size(); i++)
// // //     {
// // //         printf("%f, ",log_probs_by_amps.at(i) / valid_num_amp_channel.at(i));
// // //     }
// // //     printf("\n");
// //     //参数log_probs_all_samples未用到
// // //     for(int aziSInd = 0; aziSInd < aziSamples.size(); aziSInd++)
// // // 	log_probs_all_samples.at(aziSInd) /= antennaNum;
// // //     log_probs_all_samples.resize(aziSamples.size(),0);

// //     float dif_from_attena_dirs,tmp_dif_from_attena_dirs_F,tmp_dif_from_attena_dirs_S;
// //     for(int i = 0; i < candidateAzis.size(); i++)
// //     {
// //         tmp_dif_from_attena_dirs_F = 360;
// //         tmp_dif_from_attena_dirs_S = 360;
// //         valid_channel_no[i][0] = 0;
// //         valid_channel_no[i][1] = 0;

// //         for(int j = 0; j < antennaNum; j++)
// //         {
// //             dif_from_attena_dirs = fabs(fmod(candidateAzis.at(i) - calibParam.phRefAzi[j] + 900, 360) - 180);
// //             if(dif_from_attena_dirs < tmp_dif_from_attena_dirs_F)
// //             {
// //                 tmp_dif_from_attena_dirs_S = tmp_dif_from_attena_dirs_F;
// //                 valid_channel_no[i][1] = valid_channel_no[i][0];
// //                 tmp_dif_from_attena_dirs_F = dif_from_attena_dirs;
// //                 valid_channel_no[i][0] = j;
// //                 continue;
// //             }
// //             if(dif_from_attena_dirs < tmp_dif_from_attena_dirs_S)
// //             {
// //                 tmp_dif_from_attena_dirs_S = dif_from_attena_dirs;
// //                 valid_channel_no[i][1] = j;
// //             }
// //         }
// //     }

// //     for(int i = 0; i < antennaNum; i++)
// //     {
// //         for(int j = 0; j < 2; j++)
// //         {
// //             thisChPhSpan[j] = fmod(calibParam.phLowUpLimit[i][j] - calibParam.phRefAzi[i] + 900, 360) - 180;
// //         }
// //         base_prob = 1/(360.0 - abs(thisChPhSpan[1] - thisChPhSpan[0]));

// // //         for(int aziSInd = 0; aziSInd < aziSamples.size(); aziSInd++)
// // //         {
// // //             difAzi = fabs(fmod(aziSamples.at(aziSInd) - estimatedAzi + 900, 360) - 180);
// // //             difFromAttenaDir = fmod(aziSamples.at(aziSInd) - calibParam.phRefAzi[i] + 900, 360) - 180;
// // //             if(difFromAttenaDir >= thisChPhSpan[0] && difFromAttenaDir <= thisChPhSpan[1])
// // //             {
// // //                 predict_phase = 2 * PI * d[i] * sin((aziSamples.at(aziSInd) - calibParam.phRefAzi[i])/180*PI)/lambda;
// // //                 slope = 2 * PI * d[i] * cos((aziSamples.at(aziSInd) - calibParam.phRefAzi[i])/180*PI)/lambda*PI/180;
// // //                 sigma_phase = sqrt(pow(aziSamples.at(aziSInd) * slope,2) + pow(phi_obs_std, 2));
// // //                 dif_phase = abs(fmod(meanPhase[i] - predict_phase + 5*PI, 2*PI) - PI);
// // //                 log_probs =  - pow(dif_phase,2)/pow(sigma_phase,2)/2 - log(2*PI)/2 - log(sigma_phase);
// // //             }
// // //             else
// // //                 log_probs = log(base_prob);
// // //                 log_probs_all_samples.at(aziSInd) += log_probs;
// // //         }

// //         for(int k = 0; k < candidateAzis.size(); k++)
// //         {
// //             difFromAttenaDir = fmod(candidateAzis.at(k) - calibParam.phRefAzi[i] + 900, 360) - 180;
// //             //if(difFromAttenaDir >= thisChPhSpan[0] && difFromAttenaDir <= thisChPhSpan[1])
// // //             if(valid_channel_no[k][0] == i || valid_channel_no[k][1] == i)
// // // 	    if(candidateAziCh.at(k) == i)
// // 	    if((candidateAziCh.at(k) == i && deblurChN == 1) || (valid_channel_no[k][0] == i || valid_channel_no[k][1] == i && deblurChN == 2))
// //             {
// //                 tmpDifFromAttenaDir = abs(difFromAttenaDir) - 30 > 0 ? abs(difFromAttenaDir) - 30:0;
// //                 predict_azi_accuracy = calibParam.phFitAccuracy[i] * (tmpDifFromAttenaDir / 15 + 1);
// //                 predict_phase = 2 * PI * d[i] * sin((candidateAzis.at(k) - calibParam.phRefAzi[i])/180*PI)/lambda;
// //                 slope = 2 * PI * d[i] * cos((candidateAzis.at(k) - calibParam.phRefAzi[i])/180*PI)/lambda*PI/180;
// //                 sigma_phase = sqrt(pow(predict_azi_accuracy * slope,2) + pow(phi_obs_std, 2));
// //                 dif_phase = abs(fmod(meanPhase[i] - predict_phase + 5*PI, 2*PI) - PI);
// //                 log_probs_by_phis.at(k) = log_probs_by_phis.at(k) - pow(dif_phase,2)/pow(sigma_phase,2)/2 - log(2*PI)/2 - log(sigma_phase);
// //             }
// //             // else
// //             //     log_probs_by_phis.at(k) = log_probs_by_phis.at(k) + log(base_prob);
// //         }
// //     }

// // //     printf("**********************************\n");
// // //     printf("log_probs_by_phis = ");
// // //     for(int i = 0; i < log_probs_by_phis.size(); i++)
// // //     {
// // //         printf("%f, ",log_probs_by_phis.at(i)/deblurChN);
// // //     }
// // //     printf("\n");
// // //
// // //     printf("**********************************\n");
// // //     printf("priorLogProbs = ");
// // //     for(int i = 0; i < priorLogProbs.size(); i++)
// // //     {
// // //         printf("%f, ",priorLogProbs.at(i));
// // //     }
// // //     printf("\n");

// //     for(int i = 0; i < candidateAzis.size(); i++)
// //     {
// //         unnormalized_log_probs.at(i) = log_probs_by_amps.at(i)/valid_num_amp_channel.at(i) + log_probs_by_phis.at(i)/deblurChN;
// //         //unnormalized_log_probs.at(i) = unnormalized_log_probs.at(i)/antennaNum;
// // 	    unnormalized_log_probs.at(i) += priorLogProbs.at(i);
// //         if(minLogProbs > unnormalized_log_probs.at(i))
// //         {
// //             minLogProbs = unnormalized_log_probs.at(i);
// //             minLogProbsInd = i;
// //         }
// //     }
// //     sumProbs = 0.0;
// //     for(int i = 0; i < candidateAzis.size(); i++)
// //     {
// //         unnormalized_log_probs.at(i) = unnormalized_log_probs.at(i) - minLogProbs;
// //         probs.at(i) = exp(unnormalized_log_probs.at(i));
// //         sumProbs = sumProbs + probs.at(i);
// //     }

// //     maxProbs = 0.0;
// //     maxProbsInd = 0;
// //     for(int i = 0; i < candidateAzis.size(); i++)
// //     {
// //         probs.at(i) = probs.at(i) / sumProbs;
// //         if(probs.at(i) > maxProbs)
// //         {
// //             maxProbsInd = i;
// //             maxProbs = probs.at(i);
// //         }
// //     }
// // //     printf("**********************************\n");
// // //     printf("probs = ");
// // //     for(int i = 0; i < probs.size(); i++)
// // //     {
// // //         printf("%f, ",probs.at(i));
// // //     }
// // //     printf("\n");

// //     float std_azi = 0.0;
// //     for(int i = 0; i < antennaNum; i++)
// // 	std_azi += calibParam.phFitAccuracy[i];
// //     std_azi = std_azi / antennaNum;
// //     std_azi = std_azi > 1 ? std_azi : 1;

// //     float candSelAzi = 0.0;
// //     candSelAzi = candidateAzis.at(maxProbsInd);
// //     if(candidateAzis.size() > 1)
// //     {
// // 	candSelAzi =  meanshiftFusion(candidateAzis, probs, candSelAzi, std_azi);
// //     }
// //     return candSelAzi;
// // }

// /*
//  * 螺旋天线解模糊
//  */
// float azimuth_deblur(float *ds, float lambda, float *amps, float *phis, float *attena_dirs, int n, float *candidate_azis, int m,float *offAzis, struct detectParams param)
// {
//     float max_amp = 0,this_amp,this_attena_dir,attena_dir_max_amp,delta_amp,slope,b,mean_dir,sigma_dir;
//     int channel_ind_max_amp,best_prob_ind = 0;
//     float amp_obs_std = param.amp_obs_std; //单位dB
//     float phi_obs_std = param.phi_obs_std / 180.0 * PI; //param.phi_obs_std单位°
//     float beam_width = param.beam_width;
//     float angleSpan = param.angleSpan;
//     float probs_by_amps[3];
//     float probs_by_phis[3];
//     float dif_azis[3];
//     float prob_tmp[3];
//     float offsetAzi = 0.0;
//     double sum_probs_by_amps = 0.0,sum_probs_by_phis = 0.0;
//     float sigma2_theta = -pow(beam_width/2,2)/(-0.6931);//log(1/2) = -0.6931;//天线方向图曲线为10*log10(exp(-(theta-theta0).^2/sigma2_theta));

//     for (int i = 0; i < n; i++)
//     {
//         if (amps[i] > max_amp)
//         {
//             max_amp = amps[i];
//             channel_ind_max_amp = i;
//         }
//     }
//     attena_dir_max_amp = attena_dirs[channel_ind_max_amp];

//     for (int j = 0; j < m; j++)
//     {
//         probs_by_amps[j] = 1.0;
//         probs_by_phis[j] = 1.0;
//     }

//     FILE *fp = fopen("./log/deblurLog.txt","a+");

//     for (int i = 0; i < n; i++)
//     {
//         if (i == channel_ind_max_amp)
//             continue;
//         offsetAzi = 0.0;
//         if(param.isUseAziLib == 1)
//             offsetAzi = offAzis[i];
//         this_amp = amps[i];
//         this_attena_dir = attena_dirs[i];
//         //将this_attena_dir转换到attena_dir_max_amp正负180°附近
//         this_attena_dir = fmod(this_attena_dir-attena_dir_max_amp + offsetAzi +540,360)-180 +attena_dir_max_amp;
//         delta_amp = max_amp-this_amp;
//         //比幅法估计方位均值和方差
//         slope = sigma2_theta / 2 / (attena_dir_max_amp-this_attena_dir)/(10*log10(exp(1)));
//         b = (attena_dir_max_amp+this_attena_dir)/2;
//         mean_dir = slope*delta_amp+b;
//         sigma_dir = fabs(amp_obs_std*slope);

//         for (int j = 0; j < m; j++)
//         {
//             dif_azis[j] = fabs(fmod(candidate_azis[j]-mean_dir+180,360)-180);
// 	        if(param.writeDeblurLog >= 1) fprintf(fp,"dif_azis[%d] = %f ", j, dif_azis[j]);
//             prob_tmp[j] = exp(-pow(dif_azis[j],2)/pow(sigma_dir,2))/sigma_dir;
//             probs_by_amps[j] = probs_by_amps[j]*prob_tmp[j];
//         }
//         if(param.writeDeblurLog >= 1) fprintf(fp,"\n");
//     }

//     for (int i = 0; i < m; i++)
// 	 sum_probs_by_amps += probs_by_amps[i];

//     float this_candidate_azi,mean_dif_phi = 0;
//     int num_dif_phis = 0;
//     float predict_phis[3];
//     float dif_phis[3];

//     for (int i = 0; i < m; i++)
//     {
// 	num_dif_phis = 0;
// 	mean_dif_phi = 0;
//         this_candidate_azi = candidate_azis[i];
//         for (int j = 0; j < n; j++)
//         {
//             predict_phis[j] = sin((this_candidate_azi-attena_dirs[j])/180*PI)*2*PI*ds[j]/lambda;
//             //%查看是否在天线正对正负45°(angleSpan)范围内
//             dif_phis[j] = fabs(fmod(predict_phis[j]-phis[j]+7*PI,2*PI)-PI);
// 	    if (fabs(predict_phis[j]) < sin(angleSpan/180.0*PI)*2*PI*ds[j]/lambda)
//             {
//                 mean_dif_phi += dif_phis[j];
//                 num_dif_phis++;
//             }
//         }
//         mean_dif_phi = mean_dif_phi / num_dif_phis;
//         probs_by_phis[i] = exp(- pow(mean_dif_phi,2) / pow(phi_obs_std,2));
//         sum_probs_by_phis += probs_by_phis[i];
//     }

//     float best_prob = 0;
//     float probs[3];

//     for (int j = 0; j < m; j++)
//     {
//         probs_by_amps[j] = probs_by_amps[j]/sum_probs_by_amps;
//         probs_by_phis[j] = probs_by_phis[j]/sum_probs_by_phis;
//         probs[j] = probs_by_amps[j] * probs_by_phis[j];
//         //probs[j] = probs_by_amps[j];
//     }
//     for (int j = 0; j < m; j++)
//     {
//         if(probs[j] > best_prob)
//         {
//             best_prob = probs[j];
//             best_prob_ind = j;
//         }
//     }
//     if(param.writeDeblurLog >= 1) fprintf(fp, "resultAngle = %f \n", candidate_azis[best_prob_ind]);
//     fclose(fp);
//     return  candidate_azis[best_prob_ind];
// }

// //解决阿基米德螺旋天线相位翻转问题(解模糊)
// void checkArchiPh(float *meanPhase, float *meanAmp, int antennaNum, int archimedeanSNR)
// {
//     int lInd,rInd;
//     for(int i = 0; i < antennaNum; i++)
//     {
//         lInd = fmod((i - 1) + antennaNum, antennaNum);
//         rInd = fmod(i + 1 + antennaNum, antennaNum);
// 	//printf("ch = %d, meanPhase = %f,meanAmp[lInd] = %f, meanAmp[rInd] = %f\n", i, meanPhase[i],meanAmp[lInd], meanAmp[rInd]);
//         if((meanAmp[lInd] - meanAmp[rInd] > archimedeanSNR) && meanPhase[i] > 0)
//             meanPhase[i] = meanPhase[i] - 2*PI;
//         if((meanAmp[rInd] - meanAmp[lInd] > archimedeanSNR) && meanPhase[i] < 0)
//             meanPhase[i] = meanPhase[i] + 2*PI;
// 	//printf("ch = %d, meanPhase = %f\n", i, meanPhase[i]);
//     }

// }

// int checkPitch(float &angleResult, int theta2, int antFace, int theta2Ind, float freq, struct detectParams detectedParam)
// {
//     int antennaNum = detectedParam.antennaNum;
//     int indL,indR;
//     float thetaL,thetaR;
//     if (antFace < theta2Ind)
//     {
//         indL = antFace;
//         indR = theta2Ind;
//         thetaL = angleResult;
//         thetaR = theta2;
//     }
//     else
//     {
//         indR = antFace;
//         indL = theta2Ind;
//         thetaR = angleResult;
//         thetaL = theta2;
//     }

//     if ( fabs(antFace - theta2Ind) == antennaNum - 1)
//     {
//         indL = antFace == 0 ? theta2Ind : antFace;
//         indR = antFace == 0 ? antFace : theta2Ind;
//         thetaL = antFace == 0 ? theta2 : angleResult;
//         thetaR = antFace == 0 ? angleResult : theta2;
//     }
//     float pitchCoef;
//     if (freq > 5000)
//         pitchCoef= detectedParam.fitCoef5800[indL];
//     else if (freq > 2400 && freq < 5000)
//         pitchCoef= detectedParam.fitCoef2450[indL];
//     else
// 	    return -1;

//     thetaL = fmod(thetaL - indL * 360 / antennaNum + 360 + 360 / antennaNum * 2, 360/(antennaNum / 4)) - 360 / antennaNum * 2;
//     thetaR = fmod(thetaR - indL * 360 / antennaNum + 360 + 360 / antennaNum * 2, 360/(antennaNum / 4)) - 360 / antennaNum * 2 - 360 / antennaNum;

//     angleResult = (thetaL - thetaR) * (fabs(thetaL) - fabs(thetaR)) / (fabs(thetaL) + fabs(thetaR)) * pitchCoef;
//     angleResult = angleResult + 360 / antennaNum * (0.5 + indL);
//     return 0;
// }

// //比相测角,固定d值
// void phase2Angle(float *meanPhase, float d, float freq, float *angle, int antennaNum)
// {
//     int    i;
//     float  diffPhase;
//     double maxPhase = 0.0;  //理论最大相位差

//     maxPhase = 2 * PI * freq * 1e6 * d / LightSpeed;  //理论最大相位差
//     for (i = 0; i < antennaNum; i++)
//     {
//         // diffPhase=meanPhase[i]-initPhase[i];
//         diffPhase = meanPhase[i];
//         // diffPhase=fmod(diffPhase+PI+10*PI,2*PI)-PI;
//         // diffPhase=-diffPhase;
//         if (diffPhase > maxPhase)  //若相位差超过理论值，则置为理论值
//         {
//             diffPhase = maxPhase - 1e-3;
//         }
//         else if (diffPhase < -maxPhase)
//         {
//             diffPhase = -maxPhase + 1e-3;
//         }
//         angle[i] = asin(diffPhase / maxPhase) / PI * 180;
//         angle[i] = fmod(angle[i] + 360.0/antennaNum * i + 720, 360);  //将角度范围限定在[0,360]
//     }
// }

// //单个通道相位，比相测角(精细化标校)
// void phase2AngleSingleChElabCalib(float *meanPhase, int n, float d, float freq, float *angle, int antennaNum, int antInd, struct detectParams detectedParam, int calibLibInd)
// {
//     int    i;
//     float  diffPhase;
//     double maxPhase = 0.0;  //理论最大相位差

//     for (i = 0; i < n; i++)
//     {
//         // diffPhase=meanPhase[i]-initPhase[i];
//         maxPhase = 2 * PI * freq * 1e6 * d / LightSpeed;  //理论最大相位差
//         diffPhase = meanPhase[i];
//         // diffPhase=fmod(diffPhase+PI+10*PI,2*PI)-PI;
//         // diffPhase=-diffPhase;
//         if (diffPhase > maxPhase)  //若相位差超过理论值，则置为理论值
//         {
//             diffPhase = maxPhase - 1e-3;
//         }
//         else if (diffPhase < -maxPhase)
//         {
//             diffPhase = -maxPhase + 1e-3;
//         }
//         angle[i] = asin(diffPhase / maxPhase) / PI * 180;
// 	if(detectedParam.calibType == 0)
// 	    angle[i] = fmod(angle[i] + 360.0/antennaNum * antInd + 720, 360);  //将角度范围限定在[0,360]
// 	else if(detectedParam.calibType == 1)
// 	    angle[i] = fmod(angle[i] + detectedParam.calibParamNew[calibLibInd].phRefAzi[antInd] + 720, 360);  //将角度范围限定在[0,360]

//     }
// }

// //单个通道相位，比相测角
// void phase2AngleSingleCh(float *meanPhase, int n, float d, float freq, float *angle, int antennaNum, int antInd)
// {
//     int    i;
//     float  diffPhase;
//     double maxPhase = 0.0;  //理论最大相位差

//     for (i = 0; i < n; i++)
//     {
//         // diffPhase=meanPhase[i]-initPhase[i];
//         maxPhase = 2 * PI * freq * 1e6 * d / LightSpeed;  //理论最大相位差
//         diffPhase = meanPhase[i];
//         // diffPhase=fmod(diffPhase+PI+10*PI,2*PI)-PI;
//         // diffPhase=-diffPhase;
//         if (diffPhase > maxPhase)  //若相位差超过理论值，则置为理论值
//         {
//             diffPhase = maxPhase - 1e-3;
//         }
//         else if (diffPhase < -maxPhase)
//         {
//             diffPhase = -maxPhase + 1e-3;
//         }
//         angle[i] = asin(diffPhase / maxPhase) / PI * 180;
//         angle[i] = fmod(angle[i] + 360.0/antennaNum * antInd + 720, 360);  //将角度范围限定在[0,360]
//     }
// }
// //比相测角,将角度范围限定在[-45,45]
// void phase2Angle11(float *meanPhase, float *d, float freq, float *angle, int antennaNum)
// {
//     int    i;
//     float  diffPhase;
//     double maxPhase = 0.0;  //理论最大相位差

//     for (i = 0; i < antennaNum; i++)
//     {
//         // diffPhase=meanPhase[i]-initPhase[i];
//         maxPhase = 2 * PI * freq * 1e6 * d[i] / LightSpeed;  //理论最大相位差
//         diffPhase = meanPhase[i];
//         // diffPhase=fmod(diffPhase+PI+10*PI,2*PI)-PI;
//         // diffPhase=-diffPhase;
//         if (diffPhase > maxPhase)  //若相位差超过理论值，则置为理论值
//         {
//             diffPhase = maxPhase - 1e-3;
//         }
//         else if (diffPhase < -maxPhase)
//         {
//             diffPhase = -maxPhase + 1e-3;
//         }
//         angle[i] = asin(diffPhase / maxPhase) / PI * 180;
//     }
// }

// //比相测角(单个通道单个d值)
// void phase2AnglePro(float *meanPhase, float *d, float freq, float *angle, int antennaNum)
// {
//     int    i;
//     float  diffPhase;
//     double maxPhase = 0.0;  //理论最大相位差

//     for (i = 0; i < antennaNum; i++)
//     {
//         // diffPhase=meanPhase[i]-initPhase[i];
//         maxPhase = 2 * PI * freq * 1e6 * d[i] / LightSpeed;  //理论最大相位差
//         diffPhase = meanPhase[i];
//         // diffPhase=fmod(diffPhase+PI+10*PI,2*PI)-PI;
//         // diffPhase=-diffPhase;
//         if (diffPhase > maxPhase)  //若相位差超过理论值，则置为理论值
//         {
//             diffPhase = maxPhase - 1e-3;
//         }
//         else if (diffPhase < -maxPhase)
//         {
//             diffPhase = -maxPhase + 1e-3;
//         }
//         angle[i] = asin(diffPhase / maxPhase) / PI * 180;
// 	angle[i] = fmod(angle[i] + 360.0/antennaNum * i + 720, 360);  //将角度范围限定在[0,360]
//     }
// }

// //比相测角(精细化标校)
// void phase2AngleElabCalib(float *meanPhase, float *d, float freq, float *angle, int antennaNum, struct detectParams &detectedParam, int calibLibInd)
// {
//     int    i;
//     float  diffPhase, theta;
//     double maxPhase = 0.0;  //理论最大相位差

//     for (i = 0; i < antennaNum; i++)
//     {
//         // diffPhase=meanPhase[i]-initPhase[i];
//         maxPhase = 2 * PI * freq * 1e6 * d[i] / LightSpeed;  //理论最大相位差
//         diffPhase = meanPhase[i];
//         // diffPhase=fmod(diffPhase+PI+10*PI,2*PI)-PI;
//         // diffPhase=-diffPhase;
//         if (diffPhase > maxPhase)  //若相位差超过理论值，则置为理论值
//         {
//             diffPhase = maxPhase - 1e-3;
//         }
//         else if (diffPhase < -maxPhase)
//         {
//             diffPhase = -maxPhase + 1e-3;
//         }
//         if(detectedParam.fitPh == 1 && (freq >= detectedParam.fitFreqSpan[0] && freq <= detectedParam.fitFreqSpan[1]))
// 	{
// 	    theta = polyval(detectedParam.calibParamNew[calibLibInd].phPoly[i],meanPhase[i],5);
// 	    theta = theta > 1 ? 1:theta;
// 	    theta = theta < -1 ? -1:theta;
//             angle[i] = asin(theta) / PI * 180;
// 	}
//         else
//             angle[i] = asin(diffPhase / maxPhase) / PI * 180;
//         if(detectedParam.calibType == 0)
//             angle[i] = fmod(angle[i] + 360.0/antennaNum * i + 720, 360);  //将角度范围限定在[0,360]
//         else if(detectedParam.calibType == 1)
//             angle[i] = fmod(angle[i] + detectedParam.calibParamNew[calibLibInd].phRefAzi[i] + 720, 360);  //将角度范围限定在[0,360]
//     }
// }

// float asind2fff(float x)
// {
// 	double y=x;
// 	if(x>=1.0)
// 		y=1.0;
// 	else if(x<(float)-1.0)
// 		y=-1.0;

// 	return asin(y);
// }

// //800M夹角四天线比相拟合测角（唐宇提供算法）
// void phase2AngleFit(float *meanPhase, float d, float freq, float *angle, int antennaNum, float *fitCoef)
// {
// 	float l,b1,c1;
// 	int j=0;
// 	for(int i=0;i<antennaNum;i++)
// 	{
// 		if(meanPhase[i]>0)
// 		{
// 			j=2*i;
// 		}
// 		else
// 		{
// 			j=(2*i-1+2*antennaNum)%(2*antennaNum);
// 		}

// 		l=fitCoef[3*j];
// 		b1=fitCoef[3*j+1];
// 		c1=fitCoef[3*j+2];

// 		angle[i]=asind2fff(meanPhase[i]/l)+b1*sin(c1*meanPhase[i]);
// 	}
// }

// void calcMould2Var(float *tempPh, float *tempPhMould, int antennaNum, float* minVar, int flag)
// {
//       float phVar = 0;
//       if (flag!=-1 )
//       {
// 	  if (tempPh[flag] != 0)
// 	      tempPh[flag] = tempPh[flag] - tempPh[flag] / abs(tempPh[flag]) * 2 * PI;
//       }
//       for (int i = 0; i < antennaNum; i++)
//       {
// 	  phVar = (tempPh[i] - tempPhMould[i]) * (tempPh[i] - tempPhMould[i]) + phVar;
//       }
//       if (phVar < *minVar)
// 	  *minVar = phVar;
// }

// float calcMin2Var(float *meanPhase, float *tempPhMould, int antennaNum)
// {
//       float tempPh_1[4],tempPh_2[4],tempPh_3[4],tempPh_4[4];
//       float minVar = 1000;
//       calcMould2Var(meanPhase, tempPhMould, antennaNum, &minVar, -1);
//       for (int i = 0; i <antennaNum; i++)
//       {
// 	  memcpy(tempPh_1,meanPhase,sizeof(float)*antennaNum);
// 	  calcMould2Var(tempPh_1, tempPhMould, antennaNum, &minVar, i);
// 	  for (int j = i + 1; j<antennaNum; j++)
// 	  {
// 	      memcpy(tempPh_2,tempPh_1,sizeof(float)*antennaNum);
// 	      calcMould2Var(tempPh_2, tempPhMould, antennaNum, &minVar, j);
// 	      for (int k = j + 1; k<antennaNum; k++)
// 	      {
// 		  memcpy(tempPh_3,tempPh_2,sizeof(float)*antennaNum);
// 		  calcMould2Var(tempPh_3, tempPhMould, antennaNum, &minVar, k);
// 		  for (int m = k + 1; m<antennaNum; m++)
// 		  {
// 		      memcpy(tempPh_4,tempPh_3,sizeof(tempPh_3));
// 		      calcMould2Var(tempPh_4, tempPhMould, antennaNum, &minVar, m);
// 		  }
// 	      }
// 	  }
//       }
//       return minVar;
// }

// void phMould2Angle(float *meanPhase, float *phMould, float *angleResult, int antennaNum)
// {
//       float phVar = 10000;
//       float tempPhVar = phVar;
//       float tempPh[4];
//       for (int i = 0; i < 360; i++)
//       {
// 	  for (int j = 0; j <antennaNum; j++)
// 	  {
// 	      tempPh[j] = phMould[i*4+j];
// 	  }
// 	  phVar = calcMin2Var(meanPhase, tempPh, antennaNum);
// 	  //printf("meanPhase = %f,%f,%f,%f,phVar = %f\n",meanPhase[0],meanPhase[1],meanPhase[2],meanPhase[3],phVar);
// 	  if (tempPhVar > phVar)
// 	  {
// 	      tempPhVar = phVar;
// 	      *angleResult = i;
// 	  }
//       }
//       //printf("*angleResult = %f\n",*angleResult);
// }
// /*
// //相位方差选天线
// float phaseSelAngle(int *onAnt, float *phaseVar, float *angle, int *antIndexPtr)
// {
//     int i;
//     int isInSpan;
//     const float span=50;//天线测角的左右范围
//     *antIndexPtr=-1;//选中的天线编号
//     float minVar=1000.0;

//     for(i=0;i<NCorr;i++)
//     {
//         isInSpan=false;
//         if(i==0)
//         {
//             if(angle[i]<span || angle[i]>360-span)
//                 isInSpan=true;
//         }
//         else
//         {
//             if(angle[i]<72*i+span && angle[i]>72*i-span)
//                 isInSpan=true;
//         }
//         if(isInSpan)
//         {
//             if(phaseVar[i]<phaseVar[(i-2+5)%5] && phaseVar[i]<phaseVar[(i+2+5)%5])
//             {
//                 *antIndexPtr=i;
//                 break;
//             }
//         }
//     }

//     if(*antIndexPtr<0)
//     {
//         for(i=0;i<NCorr;i++)
//         {
//             if(onAnt[i] && phaseVar[i]<minVar)
//             {
//                 minVar=phaseVar[i];
//                 *antIndexPtr=i;
//             }
//         }
//     }
//     return angle[*antIndexPtr];
// }
// */

// //相位方差选天线
// float phaseSelAngle(int *onAnt, float *phaseVar, float *angle, int *antIndexPtr)
// {
//     int         i;
//     int        isInSpan;
//     const float span = 50;  //天线测角的左右范围
//     *antIndexPtr = -1;      //选中的天线编号
//     float minVar = 1000.0;

//     int isMeet[NCorr];

//     for (i = 0; i < NCorr; i++)
//     {
//         isMeet[i] = false;
//         isInSpan = false;
//         if (i == 0)
//         {
//             if (angle[i] < span || angle[i] > 360 - span)
//                 isInSpan = true;
//         }
//         else
//         {
//             if (angle[i] < 72 * i + span && angle[i] > 72 * i - span)
//                 isInSpan = true;
//         }
//         if (isInSpan)
//         {
//             if (phaseVar[i] < phaseVar[(i - 2 + 5) % 5] && phaseVar[i] < phaseVar[(i + 2 + 5) % 5])
//             {
//                 isMeet[i] = true;
//             }
//         }
//     }

//     for (i = 0; i < NCorr; i++)
//     {
//         if (onAnt[i] && isMeet[i])
//         {
//             if (phaseVar[i] < minVar)
//             {
//                 minVar = phaseVar[i];
//                 *antIndexPtr = i;
//             }
//         }
//     }

//     if (*antIndexPtr < 0)
//     {
//         for (i = 0; i < NCorr; i++)
//         {
//             if (onAnt[i] && phaseVar[i] < minVar)
//             {
//                 minVar = phaseVar[i];
//                 *antIndexPtr = i;
//             }
//         }
//     }

//     return angle[*antIndexPtr];
// }

// //角度合理性选天线，针对900M频段
// float angleSelAngle(int *onAnt, float *angle, int *antIndexPtr)
// {
//     int         i, j;
//     const float span = 50;  //天线测角的左右范围
//     *antIndexPtr = -1;      //选中的天线编号

//     int  isMeet[NCorr];
//     float diffAngle[NCorr];  //角度差
//     float x, x1, x2, xmax, minDiffAngle;
//     int   index = -1;

//     // printf("angle=%.1f,%.1f,%.1f,%.1f,%.1f\n", angle[0], angle[1], angle[2], angle[3], angle[4]);

//     for (i = 0; i < NCorr; i++)
//     {
//         //判断是否在合理范围内
//         isMeet[i] = false;
//         if (i == 0)
//         {
//             if (angle[i] < span || angle[i] > 360 - span)
//                 isMeet[i] = true;
//         }
//         else
//         {
//             if (angle[i] < 72 * i + span && angle[i] > 72 * i - span)
//                 isMeet[i] = true;
//         }

//         xmax = -1000;
//         //求相邻3组天线测角的最大角度差
//         for (j = 0; j < 3; j++)
//         {
//             if (j < 2)
//                 x1 = abs(angle[(i + j) % NCorr] - angle[(i + j + 1) % NCorr]);
//             else
//                 x1 = abs(angle[(i + j) % NCorr] - angle[i]);

//             x2 = 360 - x1;
//             if (x1 < x2)
//                 x = x1;
//             else
//                 x = x2;
//             if (x > xmax)
//                 xmax = x;
//         }
//         diffAngle[i] = xmax;
//     }

//     //挑选合适的组
//     minDiffAngle = 1000;
//     for (i = 0; i < NCorr; i++)
//     {
//         if (onAnt[(i + 1) % NCorr] && isMeet[(i + 1) % NCorr] && diffAngle[i] < minDiffAngle)
//         {
//             minDiffAngle = diffAngle[i];
//             index = i;
//         }
//     }

//     //若没有符合条件的，则直接找最小
//     if (index < 0)
//     {
//         minDiffAngle = 1000;
//         for (i = 0; i < NCorr; i++)
//         {
//             if (diffAngle[i] < minDiffAngle)
//             {
//                 minDiffAngle = diffAngle[i];
//                 index = i;
//             }
//         }
//     }

//     x1 = abs(angle[index] - angle[(index + 1) % NCorr]);
//     if (x1 > 360 - x1)
//         x1 = 360 - x1;
//     x2 = abs(angle[(index + 1) % NCorr] - angle[(index + 2) % NCorr]);
//     if (x2 > 360 - x2)
//         x2 = 360 - x2;

//     if (x2 < x1)
//         index = (index + 1) % NCorr;

//     *antIndexPtr = index;

//     if (abs(angle[index] - angle[(index + 1) % NCorr]) < 180)
//         return (angle[index] + angle[(index + 1) % NCorr]) / 2.0;
//     else
//         return (angle[index] + angle[(index + 1) % NCorr] + 360) / 2.0;
// }

// /*
// //幅度选天线
// float ampSelAngle(int *onAnt, float *meanAmp, float *angle, int *antIndexPtr)
// {
//     int         i;
//     const float span = 50;  //天线测角的左右范围
//     *antIndexPtr = -1;      //选中的天线编号
//     float maxAmp = 0.0;

//     int isMeet[NCorr];
//     int isInSpan[NCorr];
//     int isOk;

//     for (i = 0; i < NCorr; i++)
//     {
//         isMeet[i] = false;
//         isInSpan[i] = false;
//         if (i == 0)
//         {
//             if (angle[i] < span || angle[i] > 360 - span)
//                 isInSpan[i] = true;
//         }
//         else
//         {
//             if (angle[i] < 72 * i + span && angle[i] > 72 * i - span)
//                 isInSpan[i] = true;
//         }
//         if (isInSpan[i])
//         {
//             if (meanAmp[i] > meanAmp[(i - 2 + 5) % 5] && meanAmp[i] > meanAmp[(i + 2 + 5) % 5])
//             {
//                 isMeet[i] = true;
//             }
//         }
//     }

//     for (i = 0; i < NCorr; i++)
//     {
//         if (onAnt[i] && isMeet[i])
//         {
//             if (meanAmp[i] > maxAmp)
//             {
//                 maxAmp = meanAmp[i];
//                 *antIndexPtr = i;
//             }
//         }
//     }

//     if (*antIndexPtr < 0)
//     {
//         isOk=false;
//         for(i=0;i<NCorr;i++)
//         {
//             if(isInSpan[i] && onAnt[i])
//             {
//                 isOk=true;
//                 break;
//             }
//         }

//         if(isOk)
//         {
//             for (i = 0; i < NCorr; i++)
//             {
//                 if (onAnt[i] && meanAmp[i] > maxAmp)
//                 {
//                     maxAmp = meanAmp[i];
//                     *antIndexPtr = i;
//                 }
//             }
//         }
//         else
//         {
//             for (i = 0; i < NCorr; i++)
//             {
//                 if (onAnt[i] && meanAmp[i] > maxAmp)
//                 {
//                     maxAmp = meanAmp[i];
//                     *antIndexPtr = i;
//                 }
//             }
//         }
//     }

//     return angle[*antIndexPtr];
// }
// */

// //幅度选天线
// float ampSelAngle800(int *onAnt, float *meanAmp, float *angle, int *antIndexPtr)
// {
//     int         i;
//     const float span = 55;  //天线测角的左右范围
//     *antIndexPtr = -1;      //选中的天线编号
//     float maxAmp = 0.0;

//     int isMeet[4];
//     int isInSpan[4];
//     int isOk;

//     for (i = 0; i < 4; i++)
//     {
//         isMeet[i] = false;
//         isInSpan[i] = false;
//         if (i == 0)
//         {
//             if (angle[i] < span || angle[i] > 360 - span)
//                 isInSpan[i] = true;
//         }
//         else
//         {
//             if (angle[i] < 90 * i + span && angle[i] > 90 * i - span)
//                 isInSpan[i] = true;
//         }
//         if (isInSpan[i])
//         {
//             if (meanAmp[i] > meanAmp[(i + 2) % 4])
//             {
//                 isMeet[i] = true;
//             }
//         }
//     }

//     for (i = 0; i < 4; i++)
//     {
//         // if (onAnt[i] && isMeet[i])
//         if ((onAnt[(i + 1) % 4] || onAnt[(i + 4 - 1) % 4] || onAnt[i]) && isMeet[i])
//         // if (isMeet[i])
//         {
//             if (meanAmp[i] > maxAmp)
//             {
//                 maxAmp = meanAmp[i];
//                 *antIndexPtr = i;
//             }
//         }
//     }

//     if (*antIndexPtr < 0)
//     {
//         isOk = false;
//         for (i = 0; i < 4; i++)
//         {
//             isMeet[i] = false;
//             if (isInSpan[i] && onAnt[i])
//             {
//                 isMeet[i] = true;
//                 isOk = true;
//             }
//         }

//         if (!isOk)
//         {
//             for (i = 0; i < 4; i++)
//             {
//                 isMeet[i] = false;
//                 if (isInSpan[i] && (onAnt[(i + 1) % 4] || onAnt[(i + 4 - 1) % 4]))
//                 {
//                     isMeet[i] = true;
//                     isOk = true;
//                 }
//             }
//         }

//         if (isOk)
//         {
//             for (i = 0; i < 4; i++)
//             {
//                 if (isMeet[i] && meanAmp[i] > maxAmp)
//                 {
//                     maxAmp = meanAmp[i];
//                     *antIndexPtr = i;
//                 }
//             }
//         }
//         else
//         {
//             for (i = 0; i < 4; i++)
//             {
//                 if (onAnt[i] && meanAmp[i] > maxAmp)
//                 {
//                     maxAmp = meanAmp[i];
//                     *antIndexPtr = i;
//                 }
//             }
//         }
//     }

//     return angle[*antIndexPtr];
// }

// //螺旋天线高频幅度选天线
// float offampSelAngle(int *onAnt, float *meanAmp, float *angle, int *antIndexPtr, int antennaNum, float *offAmpLOrR)
// {
//     int         i;
//     const float span = 180*1.33/antennaNum;//天线测角的左右范围
//     *antIndexPtr = -1;//选中的天线编号
//     float maxAmp = 0.0;

//     char angleParam[] = "ANGLE";
//     char filePackageIni[] = "./package.ini";
//     char key[] = "ampSelAntType";
//     int ampSelAntType = GetIniKeyInt(angleParam, key, filePackageIni);

//     int isMeet[NCh];
//     int isInSpan[NCh];
//     int isOk;

//     for (i = 0; i < antennaNum; i++)
//     {
//         isMeet[i] = false;
//         isInSpan[i] = false;
//         if (i == 0)
//         {
//             if (angle[i] < span || angle[i] > 360 - span)
//                 isInSpan[i] = true;
//         }
//         else
//         {
//             if (angle[i] < 360*i/antennaNum + span && angle[i] > 360*i/antennaNum - span)
//                 isInSpan[i] = true;
//         }
//         if (isInSpan[i])
//         {
//             if (meanAmp[i] > meanAmp[(i - (int)(antennaNum /2)+antennaNum) % antennaNum] && meanAmp[i] > meanAmp[(i + (int)(antennaNum/2)) % antennaNum])
//             //if (meanAmp[i] > meanAmp[(i - 2 + antennaNum) % antennaNum] && meanAmp[i] > meanAmp[(i + 2) % antennaNum])
// 	    {
//                 isMeet[i] = true;
//             }
//         }
//     }

//     for (i = 0; i < antennaNum; i++)
//     {
//         if((ampSelAntType == 1 && onAnt[i] && isMeet[i])||(ampSelAntType == 0 && ((onAnt[(i + 1) % antennaNum] || onAnt[(i + antennaNum - 1) % antennaNum] || onAnt[i]) && isMeet[i])))
// 	// if (onAnt[i] && isMeet[i])
//         // if ((onAnt[(i + 1) % antennaNum] || onAnt[(i + antennaNum - 1) % antennaNum] || onAnt[i]) && isMeet[i])
//         // if (isMeet[i])
//         {
//             if (meanAmp[i] > maxAmp)
//             {
//                 maxAmp = meanAmp[i];
//                 *antIndexPtr = i;
//             }
//         }
//     }

//     int lInd,rInd;
//     if (*antIndexPtr >= 0)
//     {
// 	lInd = fmod((*antIndexPtr - 1) + antennaNum, antennaNum);
//         rInd = fmod(*antIndexPtr + 1 + antennaNum, antennaNum);
// 	if(isMeet[lInd] && meanAmp[lInd] + offAmpLOrR[*antIndexPtr * 2] > meanAmp[*antIndexPtr])
// 	    *antIndexPtr = lInd;
// 	if(isMeet[rInd] && meanAmp[rInd] + offAmpLOrR[*antIndexPtr * 2 + 1] > meanAmp[*antIndexPtr])
// 	    *antIndexPtr = rInd;
//     }

//     if (*antIndexPtr < 0)
//     {
//         isOk = false;
//         for (i = 0; i < antennaNum; i++)
//         {
//             isMeet[i] = false;
//             if (isInSpan[i] && onAnt[i])
//             {
//                 isMeet[i] = true;
//                 isOk = true;
//             }
//         }

//         if (!isOk)
//         {
//             for (i = 0; i < antennaNum; i++)
//             {
//                 isMeet[i] = false;
//                 if (isInSpan[i] && (onAnt[(i + 1) % antennaNum] || onAnt[(i + antennaNum - 1) % antennaNum]))
//                 {
//                     isMeet[i] = true;
//                     isOk = true;
//                 }
//             }
//         }

//         if (isOk)
//         {
//             for (i = 0; i < antennaNum; i++)
//             {
//                 if (isMeet[i] && meanAmp[i] > maxAmp)
//                 {
//                     maxAmp = meanAmp[i];
//                     *antIndexPtr = i;
//                 }
//             }
//         }
//         else
//         {
//             for (i = 0; i < antennaNum; i++)
//             {
//                 if (onAnt[i] && meanAmp[i] > maxAmp)
//                 {
//                     maxAmp = meanAmp[i];
//                     *antIndexPtr = i;
//                 }
//             }
//         }
//     }

//     return angle[*antIndexPtr];
// }

// //幅度选天线
// float ampSelAngle(int *onAnt, float *meanAmp, float *angle, int *antIndexPtr, int antennaNum)
// {
//     int         i;
//     const float span = 180*1.33/antennaNum;//天线测角的左右范围
//     *antIndexPtr = -1;//选中的天线编号
//     float maxAmp = 0.0;

//     char angleParam[] = "ANGLE";
//     char filePackageIni[] = "./package.ini";
//     char key[] = "ampSelAntType";
//     int ampSelAntType = GetIniKeyInt(angleParam, key, filePackageIni);
//     ampSelAntType = ampSelAntType == 1 ? 1: 0;

//     int isMeet[NCh];
//     int isInSpan[NCh];
//     int isOk;

//     for (i = 0; i < antennaNum; i++)
//     {
//         isMeet[i] = false;
//         isInSpan[i] = false;
//         if (i == 0)
//         {
//             if (angle[i] < span || angle[i] > 360 - span)
//                 isInSpan[i] = true;
//         }
//         else
//         {
//             if (angle[i] < 360*i/antennaNum + span && angle[i] > 360*i/antennaNum - span)
//                 isInSpan[i] = true;
//         }
//         if (isInSpan[i])
//         {
//             if (meanAmp[i] > meanAmp[(i - (int)(antennaNum /2)+antennaNum) % antennaNum] && meanAmp[i] > meanAmp[(i + (int)(antennaNum/2)) % antennaNum])
//             //if (meanAmp[i] > meanAmp[(i - 2 + antennaNum) % antennaNum] && meanAmp[i] > meanAmp[(i + 2) % antennaNum])
// 	    {
//                 isMeet[i] = true;
//             }
//         }
//     }

//     for (i = 0; i < antennaNum; i++)
//     {
//         if((ampSelAntType == 1 && onAnt[i] && isMeet[i])||(ampSelAntType == 0 && ((onAnt[(i + 1) % antennaNum] || onAnt[(i + antennaNum - 1) % antennaNum] || onAnt[i]) && isMeet[i])))
// 	// if (onAnt[i] && isMeet[i])
//         // if ((onAnt[(i + 1) % antennaNum] || onAnt[(i + antennaNum - 1) % antennaNum] || onAnt[i]) && isMeet[i])
//         // if (isMeet[i])
//         {
//             if (meanAmp[i] > maxAmp)
//             {
//                 maxAmp = meanAmp[i];
//                 *antIndexPtr = i;
//             }
//         }
//     }

//     if (*antIndexPtr < 0)
//     {
//         isOk = false;
//         for (i = 0; i < antennaNum; i++)
//         {
//             isMeet[i] = false;
//             if (isInSpan[i] && onAnt[i])
//             {
//                 isMeet[i] = true;
//                 isOk = true;
//             }
//         }

//         if (!isOk)
//         {
//             for (i = 0; i < antennaNum; i++)
//             {
//                 isMeet[i] = false;
//                 if (isInSpan[i] && (onAnt[(i + 1) % antennaNum] || onAnt[(i + antennaNum - 1) % antennaNum]))
//                 {
//                     isMeet[i] = true;
//                     isOk = true;
//                 }
//             }
//         }

//         if (isOk)
//         {
//             for (i = 0; i < antennaNum; i++)
//             {
//                 if (isMeet[i] && meanAmp[i] > maxAmp)
//                 {
//                     maxAmp = meanAmp[i];
//                     *antIndexPtr = i;
//                 }
//             }
//         }
//         else
//         {
//             for (i = 0; i < antennaNum; i++)
//             {
//                 if (onAnt[i] && meanAmp[i] > maxAmp)
//                 {
//                     maxAmp = meanAmp[i];
//                     *antIndexPtr = i;
//                 }
//             }
//         }
//     }

//     return angle[*antIndexPtr];
// }

//多个脉冲的角度，挑选出聚集的，再取平均
float angleSelMean(float *angle, int n)
{
    // int i, j;
    // float tmp;
    // float bin = 36.0; //区间大小
    // int *binCount = (int *)malloc(n * sizeof(int));
    // float meanAngle = 0.0;
    // int maxBinCount, index;

    // //排序，从小到大
    // for (i = 0; i < n; i++)
    // {
    //     for (j = i + 1; j < n; j++)
    //     {
    //         if (angle[j] < angle[i]) //若小的在后，则交换
    //         {
    //             tmp = angle[j];
    //             angle[j] = angle[i];
    //             angle[i] = tmp;
    //         }
    //     }
    // }

    // //统计最聚集的区间
    // for (i = 0; i < n; i++)
    // {
    //     binCount[i] = 1;
    //     for (j = i + 1; j < n; j++)
    //     {
    //         if (angle[j] < angle[i] + bin)
    //         {
    //             binCount[i]++;
    //         }
    //     }
    //     if (angle[i] + bin > 360)
    //     {
    //         for (j = 0; j < i; j++)
    //         {
    //             if (angle[j] < fmod(angle[i] + bin, 360))
    //             {
    //                 binCount[i]++;
    //             }
    //         }
    //     }
    // }
    // //挑选出计数最大的区间
    // maxBinCount = binCount[0];
    // index = 0;
    // for (i = 1; i < n; i++)
    // {
    //     if (binCount[i] > maxBinCount)
    //     {
    //         maxBinCount = binCount[i];
    //         index = i;
    //     }
    // }

    // //聚集区间取平均
    // meanAngle = 0.0;
    // if (angle[index] + bin < 360)
    // {
    //     for (i = index; i < n; i++)
    //     {
    //         if (angle[i] < angle[index] + bin)
    //         {
    //             meanAngle += angle[i];
    //         }
    //     }
    // }
    // else
    // {
    //     for (i = 0; i < n; i++)
    //     {
    //         if (angle[i] >= angle[index] && angle[i] < angle[index] + bin)
    //         {
    //             meanAngle += angle[i];
    //         }
    //         else if (angle[i] < fmod(angle[index] + bin, 360))
    //         {
    //             meanAngle += angle[i] + 360;
    //         }
    //     }
    // }
    // meanAngle /= binCount[index];
    // meanAngle = fmod(meanAngle, 360);

    // free(binCount);
    // binCount = NULL;
    // return meanAngle;
    return 0;
}

// /*
//  * 比幅测角
//  */
// void amp2Angle(float *meanAmp, float *coef, float *angle, int *antIndexPtr)
// {
//     int   i;
//     float diffAmp;
//     float sumPair[NCorr] = { 0.0 };
//     float maxSumPair = 0.0;
//     int   maxIndex = 0;

//     for (i = 0; i < NCorr; i++)
//     {
//         sumPair[i] = meanAmp[i] + meanAmp[(i + 1) % NCorr];
//         if (sumPair[i] > maxSumPair)
//         {
//             maxSumPair = sumPair[i];
//             maxIndex = i;
//         }
//     }
//     *antIndexPtr = maxIndex;

//     diffAmp = meanAmp[(maxIndex + 1) % NCorr] - meanAmp[maxIndex];
//     *angle = coef[0 + maxIndex * 3] * diffAmp * diffAmp + coef[1 + maxIndex * 3] * diffAmp + coef[2 + maxIndex * 3];
//     //*angle=fmod(*angle+72*maxIndex+720,360);//将角度范围限定在[0,360]
//     *angle = fmod(*angle + 72 * maxIndex + 36 + 720, 360);  //将角度范围限定在[0,360]
// }

// /*
//  * 根据频率选择天线间距和初始相位,(全频段标校)
//  */
// float selAntParamNew(float *initPhase, float freq, struct detectParams &param)
// {
//     int antennaNum=param.antennaNum;
//     int tmp = 10000,selInd = -1;
//     for(int k = 0; k < NCh; k++)
// 	    initPhase[k] = 0.0;

//     for (int i = 0; i < MaxCalibFreqNum; i++)
//     {
//         if(param.calibParam[i].calibFreq <= 0)
//             break;
//         if(fabs(param.calibParam[i].calibFreq - freq) < tmp)
//         {
//             tmp = fabs(param.calibParam[i].calibFreq - freq);
//             selInd = i;
//         }
//     }

//     for (int j = 0; j < antennaNum; j++)
//     {
//     	initPhase[j] = param.calibParam[selInd].calibInitPh[j];
//     }

//     float d;
//     d = (selInd == -1) ? 0 : param.calibParam[selInd].calibD;
//     return d;
// }

/*
 * 根据频率选择天线间距和初始相位,(全频段标校,多个d)
 */
void selAntParamPro(float *initPhase, float freq, struct detectParams param, float *d)
{
    //     int validSwNum = 0,calibNum = 0, selInd = -1, tmp = 10000, validSwInd = 0;
    //     float centFreq = param.centFreq;
    // 	for(int i = 0; i < MaxCalibFreqNum; i++)
    // 	{
    // 		if(param.calibParamNew[i].calibFreq <= 0)
    // 			break;
    //         if(param.calibParamNew[i].slopeFreq[0] <= param.antSwNum && param.calibParamNew[i].slopeFreq[0] > 0)
    //             validSwNum++;
    // 		calibNum++;
    // 	}

    //     for(int i = 0; i < param.antSwNum; i++)
    //     {
    //         if(centFreq <= param.antSwitchFreq[i] && centFreq > 0)
    //         {
    //             validSwInd = i + 1;
    //             break;
    //         }
    //     }
    //     int antennaNum=param.antennaNum;
    //     for(int k = 0; k < NCorr; k++)
    //     {
    //         initPhase[k] = 0.0;
    //         d[k] = 0.0;
    //     }
    // 	if(centFreq <= 0 || validSwNum < calibNum || param.antSwOnOff == 0) //centFreq未赋值,或者分层标校数少于总标校
    // 	{
    // 		for (int i = 0; i < MaxCalibFreqNum; i++)
    // 		{
    // 			if(param.calibParamNew[i].calibFreq <= 0)
    // 				break;
    // 			if(fabs(param.calibParamNew[i].calibFreq - freq) < tmp)
    // 			{
    // 				tmp = fabs(param.calibParamNew[i].calibFreq - freq);
    // 				selInd = i;
    // 			}
    // 		}
    // 	}
    // 	else
    // 	{
    // 		for (int i = 0; i < MaxCalibFreqNum; i++)
    // 		{
    // 			if(param.calibParamNew[i].calibFreq <= 0)
    // 				break;
    // 			if(fabs(param.calibParamNew[i].calibFreq - freq) < tmp && fabs(validSwInd - param.calibParamNew[i].slopeFreq[0])<0.01)
    // 			{
    // 				tmp = fabs(param.calibParamNew[i].calibFreq - freq);
    // 				selInd = i;
    // 			}
    // 		}
    // 	}

    // 	if(selInd < 0)//分层未选到标校情况
    // 	{
    // 		tmp = 10000;
    // 		for (int i = 0; i < MaxCalibFreqNum; i++)
    // 		{
    // 			if(param.calibParamNew[i].calibFreq <= 0)
    // 				break;
    // 			if(fabs(param.calibParamNew[i].calibFreq - freq) < tmp)
    // 			{
    // 				tmp = fabs(param.calibParamNew[i].calibFreq - freq);
    // 				selInd = i;
    // 			}
    // 		}
    // 	}

    //     for (int j = 0; j < antennaNum; j++)
    //     {
    //         initPhase[j] = param.calibParamNew[selInd].calibInitPh[j];
    //         d[j] = param.calibParamNew[selInd].calibD[j];
    //     }
    //     param.calibLibInd = selInd;
    //     param.validSwInd = validSwInd;
}

// /*
//  * 根据频率选择天线间距和初始相位
//  */
// float selAntParam(float *initPhase, float freq, struct detectParams &param)
// {
// 	int antennaNum=param.antennaNum;

// 	for(int i=0;i<NCorr;i++)
// 		initPhase[i]=0.0;

// 	float freqList[5]={433,825,1800,2450,5800};
// 	float dList[5]={param.d433,param.d800,param.d1800,param.d2450,param.d5800};
// 	float phaseList[5*5]={0,};
// 	for(int i=0;i<param.antennaNum;i++)
// 	{
// 		phaseList[i]=param.initPhase433[i];
// 		phaseList[i+5]=param.initPhase800[i];
// 		phaseList[i+10]=param.initPhase1800[i];
// 		phaseList[i+15]=param.initPhase2450[i];
// 		phaseList[i+20]=param.initPhase5800[i];
// 	}

// 	float fdiff=fabs(freqList[0]-freq);
// 	int index=0;
// 	float f=0.0;
// 	for(int i=1;i<5;i++)
// 	{
// 		f=fabs(freqList[i]-freq);
// 		if(f<fdiff)
// 		{
// 			fdiff=f;
// 			index=i;
// 		}
// 	}
// 	float d;
// 	if (freq>910 && freq<=945)
// 		d=param.d920coef[0]*powf(freq-910,2)+param.d920coef[1]*(freq-910)+param.d920coef[2];
// 	else
// 		d=dList[index];
// 	for(int i=0;i<param.antennaNum;i++)
// 	{
// 		initPhase[i]=phaseList[index*5+i];
// 	}

//     return d;
// }

// /*
//  * 根据频率选择幅度补偿系数
//  */
// void selAntOffAmpPro(float *offAmp, float freq, struct detectParams &param)
// {
//     int antennaNum=param.antennaNum;
//     int tmp = 10000,selInd = -1;
//     int maxAmp = 0;
//     for(int k = 0; k < NCh; k++)
//     {
// 	offAmp[k] = 0.0;
//     }
//     for (int i = 0; i < MaxCalibFreqNum; i++)
//     {
// 	if(param.calibType == 0)
// 	{
// 	    if(param.offsetAmp[i].calibFreq <= 0)
// 		break;
// 	    if(fabs(param.offsetAmp[i].calibFreq - freq) < tmp)
// 	    {
// 		tmp = fabs(param.offsetAmp[i].calibFreq - freq);
// 		selInd = i;
// 	    }
// 	}
// 	else if(param.calibType == 1)
// 	{
// 	    if(param.calibParamNew[i].calibFreq <= 0)
// 		break;
// 	    if(fabs(param.calibParamNew[i].calibFreq - freq) < tmp)
// 	    {
// 		tmp = fabs(param.calibParamNew[i].calibFreq - freq);
// 		selInd = i;
// 	    }
// 	}
//     }

//     if(param.calibType == 1)
//     {
// 	for (int j = 0; j < antennaNum; j++)
// 	{
// 	    if(param.calibParamNew[selInd].maxAmpPattern[j] > maxAmp)
// 		maxAmp = param.calibParamNew[selInd].maxAmpPattern[j];
// 	}
//     }

//     for (int j = 0; j < antennaNum; j++)
//     {
// 	if(param.calibType == 0)
// 	{
// 	    offAmp[j] = param.offsetAmp[selInd].calibAmp[j];
// 	}else if(param.calibType == 1)
// 	{
// 	    offAmp[j] = maxAmp - param.calibParamNew[selInd].maxAmpPattern[j];
// 	}
//     }
// }

// /*
//  * 根据频率选择波速中心补偿
//  */
// void selAntOffAziPro(float *offAzi, float freq, struct detectParams &param)
// {
//     int antennaNum=param.antennaNum;
//     int tmp = 10000,selInd = -1;
//     for(int k = 0; k < NCh; k++)
//     {
// 	offAzi[k] = 0.0;
//     }
//     for (int i = 0; i < MaxCalibFreqNum; i++)
//     {
// 	if(param.calibType == 0)
// 	{
// 	    if(param.offsetAzi[i].calibFreq <= 0)
// 		break;
// 	    if(fabs(param.offsetAzi[i].calibFreq - freq) < tmp)
// 	    {
// 		tmp = fabs(param.offsetAzi[i].calibFreq - freq);
// 		selInd = i;
// 	    }
// 	}
// 	else if(param.calibType == 1)
// 	{
// 	    if(param.calibParamNew[i].calibFreq <= 0)
// 		break;
// 	    if(fabs(param.calibParamNew[i].calibFreq - freq) < tmp)
// 	    {
// 		tmp = fabs(param.calibParamNew[i].calibFreq - freq);
// 		selInd = i;
// 	    }
// 	}
//     }

//     for (int j = 0; j < antennaNum; j++)
//     {
// 	if(param.calibType == 0)
// 	{
// 	    offAzi[j] = param.offsetAzi[selInd].calibAzi[j];
// 	}else if(param.calibType == 1)
// 	{
// 	    offAzi[j] = fmod(param.calibParamNew[selInd].ampBeamCent[j] - 360.0/antennaNum * j + 540, 360) -180;
// 	}
//     }
// }

// /*
//  * 根据频率选择幅度补偿系数
//  */
// void selAntOffAmp(float *offAmp, float freq, struct detectParams &param)
// {
// 	int antennaNum=param.antennaNum;
// 	for(int i=0;i<antennaNum;i++)
// 		offAmp[i]=0.0;

//     float d = 0.0;
//     if (abs(freq - 2450) < 75)
//     {
//         for (int i = 0; i < antennaNum; i++)
//             offAmp[i] = param.offsetAmp2450[i];
//     }
//     else if (abs(freq - 5800) < 75)
//     {
//         for (int i = 0; i < antennaNum; i++)
//             offAmp[i] = param.offsetAmp5800[i];
//     }
//     else if (abs(freq - 1800) < 75)
//     {
//         for (int i = 0; i < antennaNum; i++)
//             offAmp[i] = param.offsetAmp1800[i];
//     }
//     else if (abs(freq - 825) < 75)
//     {
//         for (int i = 0; i < antennaNum; i++)
//             offAmp[i] = param.offsetAmp800[i];
//     }
//     else if (abs(freq - 433) < 75)
//     {
//         for (int i = 0; i < antennaNum; i++)
//             offAmp[i] = param.offsetAmp433[i];
//     }
// }

// // void selActiveCh(float* meanAmp, float *maxAmpPattern, vector<int>&activeCh, int antennaNum, int* onAnt)
// // {
// //     vector<float> activeProbs;
// //     float tmpProbs = 0.0, maxProbs = 0, secondProbs = 0;
// //     int maxProbsInd = 0, secondProbsInd = 0;

// //     for(int i = 0; i < antennaNum; i++)
// //     {
// //         tmpProbs = exp((meanAmp[i] - maxAmpPattern[i]) / 3);
// // 		//printf("meanAmp[%d] = %f, maxAmpPattern = %f, tmpProbs = %f, onAnt = %d\n",i,meanAmp[i],maxAmpPattern[i],tmpProbs, onAnt[i]);
// //         activeProbs.push_back(tmpProbs);
// //         if(maxProbs < tmpProbs && onAnt[i])
// //         {
// //             maxProbs = tmpProbs;
// //             maxProbsInd = i;
// //         }
// //     }

// //     for(int i = 0; i < activeProbs.size(); i++)
// //     {
// //         activeProbs[i] = activeProbs[i] / maxProbs;
// //         if(activeProbs[i] > 0.5 && onAnt[i])
// //             activeCh.push_back(i);
// // 	tmpProbs = activeProbs[i];
// // 	if(secondProbs < tmpProbs && i != maxProbsInd && onAnt[i])
// //         {
// //             secondProbs = tmpProbs;
// //             secondProbsInd = i;
// //         }
// //     }
// //     if(activeCh.size() == 0)
// //     {
// // 	if(onAnt[maxProbsInd]) activeCh.push_back(maxProbsInd);
// // 	if(onAnt[secondProbsInd]) activeCh.push_back(secondProbsInd);
// //     }
// //     else if(activeCh.size() == 1 && onAnt[secondProbsInd])
// //     {
// // 	if(fabs(maxProbsInd - secondProbsInd) == 1 || fabs(maxProbsInd - secondProbsInd) == 7)
// // 	    activeCh.push_back(secondProbsInd);
// // 	else
// // 	{
// // 	    int lInd = fmod((maxProbsInd - 1) + antennaNum, antennaNum);
// // 	    int rInd = fmod(maxProbsInd + 1 + antennaNum, antennaNum);
// // 	    tmpProbs = 0;
// // 	    if(onAnt[lInd])
// // 	    {
// // 		tmpProbs = activeProbs[lInd];
// // 		secondProbsInd = lInd;
// // 	    }
// // 	    if(activeProbs[rInd] > tmpProbs && onAnt[rInd])
// // 	    {
// // 		secondProbsInd = rInd;
// // 	    }
// // 	    activeCh.push_back(secondProbsInd);
// // 	}
// //     }
// //     // printf("%d\n",activeCh.size());
// //     if(activeCh.empty())
// // 	activeCh.push_back(maxProbsInd);
// // }
// /*
//  * 根据频率选择幅度补偿系数
//  */
// void selAntOffAmpLOrR(float *offAmp, float freq, struct detectParams &param)
// {
// 	int antennaNum=param.antennaNum;

// 	for(int i=0;i<antennaNum * 2;i++)
// 		offAmp[i]=0.0;

//     float d = 0.0;
//     if (abs(freq - 2450) < 75)
//     {
//         for (int i = 0; i < antennaNum; i++)
//             offAmp[i] = param.offsetAmp2450LOrR[i];
//     }
//     else if (abs(freq - 5800) < 75)
//     {
//         for (int i = 0; i < antennaNum*2; i++)
//             offAmp[i] = param.offsetAmp5800LOrR[i];
//     }
//     else if (abs(freq - 1800) < 75)
//     {
//         for (int i = 0; i < antennaNum; i++)
//             offAmp[i] = param.offsetAmp1800LOrR[i];
//     }
//     else if (abs(freq - 845) < 55)
//     {
//         for (int i = 0; i < antennaNum; i++)
//             offAmp[i] = param.offsetAmp800LOrR[i];
//     }
//     else if (abs(freq - 915) <= 15)
//     {
//         for (int i = 0; i < antennaNum; i++)
//             offAmp[i] = param.offsetAmp915LOrR[i];
//     }
//     else if (abs(freq - 433) < 75)
//     {
//         for (int i = 0; i < antennaNum; i++)
//             offAmp[i] = param.offsetAmp433LOrR[i];
//     }
// }

// /*
//  * vector 去重复
//  */
// // void RemoveRepeat(vector<int> &activeCh)
// // {
// //     // sort(activeCh.begin(), activeCh.end());
// //     // auto it = unique(activeCh.begin(), activeCh.end());
// //     // activeCh.erase(it, activeCh.end());
// // }

// /*
//  * 相邻天线过渡带内，角度取两者平均
//  */
// float angleOverlap(float angleResult, float *angle, int antennaNum, float overlapAngle)
// {
// 	overlapAngle = overlapAngle / (antennaNum / 4);
// 	float theta=360.0/(float)antennaNum;
// 	float span = 180*1.33/antennaNum;//天线测角的左右范围
// 	int antIndex=angleResult/theta;
// 	float cenAngle=(antIndex+0.5)*theta;
// 	float result,meanCoef = 0.0;
// 	//printf("------angle:%f,%f\n",angle[antIndex],angle[(antIndex+1)%antennaNum]);
// 	if(fabs(angleResult-cenAngle)<overlapAngle/2.0)
// 	{
// 		//printf("angleResult=%.1f, antIndex=%d, %.1f, %.1f,",angleResult,antIndex,angle[antIndex],angle[(antIndex+1) % antennaNum]);
// 		if((antIndex * theta + span) > angle[antIndex] && ((antIndex + 1) * theta - span) < angle[(antIndex+1) % antennaNum])
// 		{
// 			meanCoef = (2 - 1 / (1 + 0.5*exp(abs(angleResult - cenAngle) - overlapAngle / 4)))*0.5;
// 			meanCoef = (angle[antIndex] == angleResult ? meanCoef:(1-meanCoef));
// 			result = meanCoef * angle[antIndex]+(1 - meanCoef) * angle[(antIndex+1) % antennaNum];
// 			//printf("meanCoef = %f,%f\n",meanCoef,1-meanCoef);
// 		}
// 		else
// 			result=angleResult;
// 		//printf("result=%.1f\n",result);
// 	}
// 	else
// 		result=angleResult;

// 	return result;
// }

// float calcAnglePro(float *meanPhase, float *meanAmp, float freq, float *d, int *onAnt, float *phaseVar, int &antFace, struct detectParams detectedParam)
// {
//     return 0;
// }
