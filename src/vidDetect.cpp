#include "uavDetect.h"
// #include <unistd.h>
// #include <math.h>

//迭代求取最佳的阈值
float findThresh(float *s, int n, float dT, int MaxIterations, float *maxt)
{
    int i = 0, k = 0;
    int n1 = 0;
    float t, tnew = 0.0;
    float t1 = 0.0, t2 = 0.0;
    float maxt1 = *maxt;
    for (i = 0; i < n; i++)
    {
        tnew += s[i] / n;
    }
    t = tnew + 2 * dT;
    while (fabs(tnew - t) > dT && k < MaxIterations)
    {
        t = tnew;
        for (i = 0; i < n; i++)
        {
            if (s[i] > t)
            {
                t1 += s[i];
                n1++;
            }
            else
                t2 += s[i];
        }
        if (n > n1)
            tnew = (t1 / n1 + t2 / (n - n1)) / 2.0;
        else
            tnew = t;
        maxt1 = t1 / n1;
        t1 = 0;
        t2 = 0;
        n1 = 0;
        k++;
        *maxt = maxt1;
    }
    return tnew;
}

/*
//根据阈值分割序列，求取凸起的起点和宽度索引
int threshSegm(float *s, int n, float thresh, int delt, int minw, int *q1, int *w, float *snr)
{
    int i,j;

    int flag = true;
    int nFall = 0;//记录碰到的小沟壑的点数
    int nw = 0;
    float signalAmp=0.0,noiseAmp=0.0;
    int nSigPoint=0, nNoisePoint=0;

    q1[0] = -1;
    for (i = 0; i < n; i++)
    {
        if (s[i]>thresh)
        {
            if (!flag)
            {
                q1[nw] = i;
                flag = true;
            }
            nFall = 0;
        }
        else
        {
            if (flag)
            {
                nFall++;
                if (nFall >= delt)
                {
                    if (q1[0] > 0)
                    {
                        w[nw] = i - delt + 1 - q1[nw];
                        if (w[nw] > minw)
                        {
                            nw++;
                        }
                    }
                    flag = false;
                }
            }
        }
    }

    //估计信噪比
    *snr=0;
    if(nw>0)
    {
        for(i=0;i<nw;i++)
        {
            for(j=q1[i];j<=q1[i]+w[i];j++)
            {
                signalAmp+=s[j];
                nSigPoint++;
            }
            if(i==0)
            {
                for(j=0;j<q1[i];j++)
                {
                    noiseAmp+=s[j];
                    nNoisePoint++;
                }
            }
            else
            {
                for(j=q1[i-1]+w[i-1]+1;j<q1[i];j++)
                {
                    noiseAmp+=s[j];
                    nNoisePoint++;
                }
            }
        }
        signalAmp/=nSigPoint;
        noiseAmp/=nNoisePoint;
        *snr=signalAmp-noiseAmp;
    }

    return nw;
}
*/

//根据阈值分割序列，求取凸起的起点和宽度索引
int threshSegm(float *s, int n, float thresh, int delt, int minw, int *q1, int *w, float *colSnr)
{
    int i, j;

    int flag = true;
    int nFall = 0; //记录碰到的小沟壑的点数
    int nw = 0;
    float signalAmp = 0.0, noiseAmp = 0.0;

    q1[0] = -1;
    for (i = 0; i < n; i++)
    {
        if (s[i] > thresh)
        {
            if (!flag)
            {
                q1[nw] = i;
                flag = true;
            }
            nFall = 0;
        }
        else
        {
            if (flag)
            {
                nFall++;
                if (nFall >= delt)
                {
                    if (q1[0] > 0)
                    {
                        w[nw] = i - delt + 1 - q1[nw];
                        if (w[nw] > minw)
                        {
                            nw++;
                        }
                    }
                    flag = false;
                }
            }
        }
    }

    //估计信噪比
    if (nw > 0)
    {
        for (i = 0; i < nw; i++)
        {
            signalAmp = 0.0;
            noiseAmp = 0.0;
            for (j = q1[i]; j <= q1[i] + w[i]; j++)
            {
                signalAmp += s[j];
            }
            signalAmp /= w[i] + 1;
            if (i == 0)
            {
                for (j = q1[i] - 3; j < q1[i]; j++)
                {
                    if (j < 0)
                    {
                        noiseAmp += s[0];
                        break;
                    }
                    noiseAmp += s[j];
                }
                noiseAmp /= 3.0;
            }
            else
            {
                for (j = q1[i - 1] + w[i - 1] + 1; j < q1[i]; j++)
                {
                    noiseAmp += s[j];
                }
                noiseAmp /= q1[i] - q1[i - 1] - w[i - 1] - 1;
            }
            colSnr[i] = signalAmp - noiseAmp;
        }
    }

    return nw;
}

/*
 * 判断脉冲时间是否满足周期性
 * t2:脉冲边缘时间，ms
 * index:脉冲索引
 * n:脉冲索引个数
 * pulseT:周期
 * nPulseT:周期个数
 */
int matchPulseT(int (*match)[MaxNumVidT], float *t2, int *index, int n, float *pulseT, int nPulseT, float pulseTErr)
{
    float diffTime, resTime, TErr;
    float sumPulseT = 0.0;
    int nt = 0, nT = 0, nBigT = 0;
    int count = 0, maxCount = 0, maxIndex = 0;

    for (int i = 0; i < nPulseT; i++)
        sumPulseT += pulseT[i];

    int oneMatch[5][MaxNumVidT];          //记录倍数周期上的脉冲编号
    float oneMatchTimeErr[5][MaxNumVidT]; //记录倍数周期的时间偏差

    for (int i = 0; i < n; i++)
    {
        if (i > MaxNumVidT - 1)
            break;
        for (int k = 0; k < nPulseT; k++)
        {
            oneMatch[k][0] = index[i];
            for (int m = 1; m < MaxNumVidT; m++)
                oneMatch[k][m] = -1;

            for (int j = 0; j < n; j++)
            {
                if (j > MaxNumVidT - 1)
                    break;
                if (i == j)
                    continue;

                diffTime = t2[index[j]] - t2[index[i]];
                if (diffTime < 0)
                    continue;

                nBigT = floor(diffTime / sumPulseT);
                resTime = diffTime - nBigT * sumPulseT;

                nt = 0;
                while (1)
                {
                    TErr = abs(resTime);
                    if (TErr < pulseTErr)
                    {
                        nT = nPulseT * nBigT + nt;
                        if (nT > 0 && nT < MaxNumVidT)
                        {
                            if (oneMatch[k][nT] < 0)
                            {
                                oneMatch[k][nT] = index[j];
                                oneMatchTimeErr[k][nT] = TErr;
                            }
                            else if (TErr < oneMatchTimeErr[k][nT])
                            {
                                oneMatch[k][nT] = index[j];
                                oneMatchTimeErr[k][nT] = TErr;
                            }
                        }
                        break;
                    }
                    if (resTime < 0)
                        break;
                    resTime -= pulseT[(k + nt) % nPulseT];
                    nt++;
                }
            }
        }
        //统计每种起点的周期匹配次数，选择匹配次数最大的
        maxCount = 0;
        maxIndex = 0;
        for (int k = 0; k < nPulseT; k++)
        {
            count = 0;
            for (int p = 0; p < MaxNumVidT; p++)
            {
                if (oneMatch[k][p] > 0)
                {
                    count++;
                }
            }
            if (count > maxCount)
            {
                maxCount = count;
                maxIndex = k;
            }
        }

        for (int k = 0; k < MaxNumVidT; k++)
            match[i][k] = oneMatch[maxIndex][k];
    }
}

//求中位值
int getMid(int *arr, int size)
{
    int i, j, t;
    int mid;
    for (i = 0; i < size - 1; i++) // i为排序的趟数
    {
        for (j = 0; j < size - i - 1; j++) // j为第i趟需比较的次数
        {
            if (arr[j] > arr[j + 1])
            {
                t = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = t;
            }
        }
    }
    if (size % 2 == 0) //判断元素个数是否为偶数
    {
        mid = (arr[size / 2] + arr[size / 2 - 1]) / 2;
    }
    else
        mid = arr[(size - 1) / 2];
    return mid;
}
// //平滑，处理单频信号
// void smoothSingleSig(int n, float *diffAmpCorr)
// {
//     for(int k = 0 ; k < n-1; k++)
//     {
// 	if(diffAmpCorr[k] > 0 && diffAmpCorr[k]/2 + diffAmpCorr[k+1]/2 < diffAmpCorr[k]/4)
// 	{
// 	    diffAmpCorr[k] = (diffAmpCorr[k]+diffAmpCorr[k+1])/2;
// 	    diffAmpCorr[k+1] = diffAmpCorr[k];
// 	}
//     }
// }
//平滑，处理单频信号
void smoothSingleSig(int n, float *ampCorr)
{
    float diffAmpCorr[n];
    for (int k = 0; k < n - 1; k++)
    {
        diffAmpCorr[k] = ampCorr[k + 1] - ampCorr[k];
    }
    for (int k = 0; k < n - 2; k++)
    {
        if (abs(diffAmpCorr[k] / 2 + diffAmpCorr[k + 1] / 2) < abs(diffAmpCorr[k] / 5) || abs(diffAmpCorr[k] / 2 + diffAmpCorr[k + 1] / 2) < abs(diffAmpCorr[k + 1] / 5))
        {
            ampCorr[k + 1] = ampCorr[k] / 2 + ampCorr[k + 2] / 2;
        }
    }
}

void getShieldFreqInd(int *shieldFreqInd, float cenFreqOnAllCh, struct detectParams detectedParam, int nCols, float dF)
{

    float tmpFreq = 0.0;
    for (int i = 0; i < nCols; i++)
    {
        shieldFreqInd[i] = 1;
    }

    for (int i = 0; i < detectedParam.shieldPhN; i++)
    {
        if (detectedParam.shieldPhFreqSpan[2 * i] > cenFreqOnAllCh + 0.5 * nCols * dF || detectedParam.shieldPhFreqSpan[2 * i + 1] < cenFreqOnAllCh - 0.5 * nCols * dF)
            continue;
        for (int j = 0; j < nCols; j++)
        {
            tmpFreq = cenFreqOnAllCh + j * dF - 0.5 * nCols * dF;
            if (tmpFreq > detectedParam.shieldPhFreqSpan[2 * i] && tmpFreq < detectedParam.shieldPhFreqSpan[2 * i + 1])
            {
                shieldFreqInd[j] = 0;
            }
        }
    }
}

// int caliCenFreqBw(struct uavRow *allUavRow, float ***sumCorrAmp, float ***sumCorrPhase, int uavCh, int indLCali, int indRCali, int indCenFreq, struct UAVLib *UAVtypes, int uavIndex, int *caliFreqInd, float *caliBW, float dF, float dt, int freqIndexL, int freqIndexR)
// {
//     float averReal = 0, averImag = 0, thresh = 0.0, tmpAmp = 0, maxt = 0;
//     float averRealImag[2] = {0.0};
//     float *meanAmp = (float *)malloc(sizeof(float) * (indRCali - indLCali + 1));
//     int n = 0, indK = 0, count = 0, tmpCount = 0;
//     int uavIndL[2] = {-1}, uavIndR[2] = {-1};
//     float *averAmpCorr = (float *)malloc((indRCali - indLCali + 1) * sizeof(float));
//     float *noiseAmp = (float *)malloc((indRCali - indLCali + 1) * sizeof(float));
//     float *snrAmp = (float *)malloc((indRCali - indLCali + 1) * sizeof(float));
//     float *snrAmp_1 = (float *)malloc((indRCali - indLCali + 1) * sizeof(float));
//     // memset(averAmpCorr, 0.0,indRCali - indLCali + 1);
//     memset(noiseAmp, 0.0, indRCali - indLCali + 1);
//     memset(snrAmp, 0.0, indRCali - indLCali + 1);
//     memset(snrAmp_1, 0.0, indRCali - indLCali + 1);
//     for (int i = 0; i < indRCali - indLCali + 1; i++)
//     {
//         averAmpCorr[i] = 0.0;
//         meanAmp[i] = 0.0;
//     }
//     for (int j = 0; j < UAVtypes[uavIndex].nPulseBW; j++)
//     {
//         caliBW[j] = UAVtypes[uavIndex].pulseBW[j];
//     }

//     for (int i = 0; i < allUavRow->nw; i++)
//     {
//         n += allUavRow->w[i];
//     }

//     for (int i = 0; i < allUavRow->nw; i++)
//     {
//         for (int j = 0; j < allUavRow->w[i]; j++)
//         {
//             for (int k = 0; k < freqIndexR - freqIndexL + 1; k++)
//             {
//                 averReal += sumCorrAmp[uavCh][allUavRow->q1[i] + j][k] * cos(sumCorrPhase[uavCh][allUavRow->q1[i] + j][k]) / n;
//                 averImag += sumCorrAmp[uavCh][allUavRow->q1[i] + j][k] * sin(sumCorrPhase[uavCh][allUavRow->q1[i] + j][k]) / n;
//             }
//         }
//     }
//     averReal = averReal / (freqIndexR - freqIndexL + 1);
//     averImag = averImag / (freqIndexR - freqIndexL + 1);

//     int useAmpOrAmpPh = 0;
//     for (int k = 0; k <= indRCali - indLCali; k++)
//     {
//         for (int i = 0; i < allUavRow->nw; i++)
//         {
//             indK = k + indLCali;
//             averRealImag[0] = 0;
//             averRealImag[1] = 0;
//             for (int j = 0; j < allUavRow->w[i]; j++)
//             {
//                 averRealImag[0] += ((sumCorrAmp[uavCh][allUavRow->q1[i] + j][indK] * cos(sumCorrPhase[uavCh][allUavRow->q1[i] + j][indK]) - averReal)) / n;
//                 averRealImag[1] += ((sumCorrAmp[uavCh][allUavRow->q1[i] + j][indK] * sin(sumCorrPhase[uavCh][allUavRow->q1[i] + j][indK]) - averImag)) / n;
//                 averAmpCorr[k] += sumCorrAmp[uavCh][allUavRow->q1[i] + j][indK] / n;
//             }
//             meanAmp[k] += sqrt(averRealImag[0] * averRealImag[0] + averRealImag[1] * averRealImag[1]);
//         }
//         // printf("%f\n",meanAmp[k]);
//         if (averAmpCorr[k] >= 80) //高信噪比只用幅度比较，低信噪比采用复数比较
//             useAmpOrAmpPh++;
//     }

//     //通过有信号区域与无信号区域比较
//     //计算未检出区域整个累加的阈值
//     count = 0;
//     float maxSnrAmp = 0;
//     for (int k = 0; k <= indRCali - indLCali; k++)
//     {
//         indK = k + indLCali;
//         averRealImag[0] = 0;
//         averRealImag[1] = 0;
//         tmpAmp = 0;
//         n = 0;
//         noiseAmp[k] = 0;
//         for (int i = 0; i < allUavRow->nw - 1; i++)
//         {
//             for (int j = allUavRow->q1[i] + allUavRow->w[i] + 1; j < allUavRow->q1[i + 1]; j++)
//             {
//                 for (int nW = 0; nW < UAVtypes[uavIndex].nPulseT; nW++)
//                 {
//                     if (abs((allUavRow->q1[i + 1] - allUavRow->q1[i]) * dt - UAVtypes[uavIndex].pulseT[nW]) < UAVtypes[uavIndex].pulseTErr)
//                     {
//                         averRealImag[0] += ((sumCorrAmp[uavCh][j][indK] * cos(sumCorrPhase[uavCh][j][indK]) - averReal));
//                         averRealImag[1] += ((sumCorrAmp[uavCh][j][indK] * sin(sumCorrPhase[uavCh][j][indK]) - averImag));
//                         tmpAmp += sumCorrAmp[uavCh][j][indK];
//                         n++;
//                         break;
//                     }
//                 }
//             }
//         }
//         if (n == 0)
//         {
//             for (int i = 0; i < allUavRow->nw - 1; i++)
//             {
//                 for (int j = allUavRow->q1[i] + allUavRow->w[i] + 1; j < allUavRow->q1[i + 1]; j++)
//                 {
//                     averRealImag[0] += ((sumCorrAmp[uavCh][j][indK] * cos(sumCorrPhase[uavCh][j][indK]) - averReal));
//                     averRealImag[1] += ((sumCorrAmp[uavCh][j][indK] * sin(sumCorrPhase[uavCh][j][indK]) - averImag));
//                     tmpAmp += sumCorrAmp[uavCh][j][indK];
//                     n++;
//                 }
//             }
//         }
//         // 	if (useAmpOrAmpPh <= 2)
//         {
//             noiseAmp[k] = sqrt(averRealImag[0] * averRealImag[0] + averRealImag[1] * averRealImag[1]) / n;
//             snrAmp[k] = meanAmp[k]; // - noiseAmp[k];
//             snrAmp_1[k] = averAmpCorr[k];
//         }
//         // 	else
//         /*	{
//                 noiseAmp[k] = tmpAmp / n;
//                 snrAmp[k] = averAmpCorr[k] - noiseAmp[k];
//                 snrAmp_1[k] = averAmpCorr[k];
//             }*/

//         // printf("----noiseAmp = %f,meanAmp = %f,snrAmp = %f\n",noiseAmp[k],meanAmp[k],snrAmp[k]);
//         if (snrAmp[k] > maxSnrAmp)
//             maxSnrAmp = snrAmp[k];
//     }
//     smoothSingleSig(indRCali - indLCali + 1, snrAmp);
//     smoothSingleSig(indRCali - indLCali + 1, snrAmp_1);
//     smoothSingleSig(indRCali - indLCali + 1, meanAmp);

//     thresh = findThresh(snrAmp, indRCali - indLCali + 1, 0.2, 3, &maxt);

//     float errL = 0, errR = 0;

//     //     for(int k = 0 ; k <= indRCali - indLCali; k++)
//     //     {
//     // 	printf("%f ",snrAmp[k]);
//     //     }
//     //     printf("\n");
//     //     printf("1111111111111111111111111111111111111111\n");
//     //     printf("\n");
//     //     for(int k = 0 ; k <= indRCali - indLCali; k++)
//     //     {
//     // 	printf("%f ",snrAmp_1[k]);
//     //     }
//     //     printf("\n");
//     //     printf("1111111111111111111111111111111111111111\n");
//     //     printf("\n");
//     //

//     float sumBWamp[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
//     int indexBl[5] = {0, 0, 0, 0, 0};
//     int indexBr[5] = {0, 0, 0, 0, 0};
//     int IndexBW = 0;
//     float maxSumBW = 0;
//     for (int j = 0; j < UAVtypes[uavIndex].nPulseBW; j++)
//     {
//         sumBWamp[j] = -100.0;
//         float sumAmp = 0.0;
//         int nColBW = floor(UAVtypes[uavIndex].pulseBW[j] / dF);
//         for (int k = 0; k <= indRCali - indLCali; k++)
//         // for (k = IndexL; k < IndexR + 1; k++)
//         {
//             sumAmp = 0.0;
//             int colCount = 0;
//             for (int p = 0; p < nColBW; p++)
//             {
//                 if (k + p < indRCali - indLCali + 1)
//                 {
//                     sumAmp += snrAmp[k + p];
//                     colCount++;
//                 }
//             }
//             sumAmp /= (float)colCount;
//             if (sumAmp > sumBWamp[j])
//             {
//                 sumBWamp[j] = sumAmp;
//                 indexBl[j] = k;
//                 indexBr[j] = k + colCount;
//             }
//         }
//     }

//     IndexBW = 0;
//     printf("sumBWamp = %f,  ", sumBWamp[0]);
//     for (int j = 1; j < UAVtypes[uavIndex].nPulseBW; j++)
//     {
//         if (sumBWamp[j] > sumBWamp[0] * 0.68)
//             IndexBW = j;
//         printf("%f,  ", sumBWamp[j]);
//     }
//     printf("\n");
//     caliBW[0] = UAVtypes[uavIndex].pulseBW[IndexBW];
//     *caliFreqInd = indLCali + indexBl[IndexBW] + caliBW[0] / 2 / dF;

//     free(meanAmp);
//     free(averAmpCorr);
//     free(noiseAmp);
//     free(snrAmp);
//     free(snrAmp_1);
//     return 0;
// }

// int caliFreqBw(struct uavRow *allUavRow, float ***sumCorrAmp, float ***sumCorrPhase, int uavCh, int indLCali, int indRCali, int indCenFreq, struct UAVLib *UAVtypes, int uavIndex, int *caliFreqInd, float *caliBW, float dF, float dt)
// {
//     float averReal = 0, averImag = 0, thresh = 0.0, tmpAmp = 0, maxt = 0;
//     float averRealImag[2] = {0.0};
//     float *meanAmp = (float *)malloc(sizeof(float) * (indRCali - indLCali + 1));
//     int n = 0, indK = 0, count = 0, tmpCount = 0;
//     int uavIndL[2] = {-1}, uavIndR[2] = {-1};
//     float *averAmpCorr = (float *)malloc((indRCali - indLCali + 1) * sizeof(float));
//     float *noiseAmp = (float *)malloc((indRCali - indLCali + 1) * sizeof(float));
//     float *snrAmp = (float *)malloc((indRCali - indLCali + 1) * sizeof(float));
//     float *snrAmp_1 = (float *)malloc((indRCali - indLCali + 1) * sizeof(float));
//     // memset(averAmpCorr, 0.0,indRCali - indLCali + 1);
//     memset(noiseAmp, 0.0, indRCali - indLCali + 1);
//     memset(snrAmp, 0.0, indRCali - indLCali + 1);
//     memset(snrAmp_1, 0.0, indRCali - indLCali + 1);
//     for (int i = 0; i < indRCali - indLCali + 1; i++)
//     {
//         averAmpCorr[i] = 0.0;
//         meanAmp[i] = 0.0;
//     }
//     for (int j = 0; j < UAVtypes[uavIndex].nPulseBW; j++)
//     {
//         caliBW[j] = UAVtypes[uavIndex].pulseBW[j];
//     }

//     for (int i = 0; i < allUavRow->nw; i++)
//     {
//         n += allUavRow->w[i];
//     }

//     for (int i = 0; i < allUavRow->nw; i++)
//     {
//         for (int j = 0; j < allUavRow->w[i]; j++)
//         {
//             averReal += sumCorrAmp[uavCh][allUavRow->q1[i] + j][indCenFreq] * cos(sumCorrPhase[uavCh][allUavRow->q1[i] + j][indCenFreq]) / n;
//             averImag += sumCorrAmp[uavCh][allUavRow->q1[i] + j][indCenFreq] * sin(sumCorrPhase[uavCh][allUavRow->q1[i] + j][indCenFreq]) / n;
//         }
//     }

//     int useAmpOrAmpPh = 0;
//     for (int k = 0; k <= indRCali - indLCali; k++)
//     {
//         for (int i = 0; i < allUavRow->nw; i++)
//         {
//             indK = k + indLCali;
//             averRealImag[0] = 0;
//             averRealImag[1] = 0;
//             for (int j = 0; j < allUavRow->w[i]; j++)
//             {
//                 averRealImag[0] += ((sumCorrAmp[uavCh][allUavRow->q1[i] + j][indK] * cos(sumCorrPhase[uavCh][allUavRow->q1[i] + j][indK]) - averReal)) / n;
//                 averRealImag[1] += ((sumCorrAmp[uavCh][allUavRow->q1[i] + j][indK] * sin(sumCorrPhase[uavCh][allUavRow->q1[i] + j][indK]) - averImag)) / n;
//                 averAmpCorr[k] += sumCorrAmp[uavCh][allUavRow->q1[i] + j][indK] / n;
//             }
//             meanAmp[k] += sqrt(averRealImag[0] * averRealImag[0] + averRealImag[1] * averRealImag[1]);
//         }
//         // printf("%f\n",meanAmp[k]);
//         if (averAmpCorr[k] >= 80)
//             useAmpOrAmpPh++;
//     }

//     //通过有信号区域与无信号区域比较
//     //计算未检出区域整个累加的阈值
//     count = 0;
//     float maxSnrAmp = 0;
//     for (int k = 0; k <= indRCali - indLCali; k++)
//     {
//         indK = k + indLCali;
//         averRealImag[0] = 0;
//         averRealImag[1] = 0;
//         tmpAmp = 0;
//         n = 0;
//         noiseAmp[k] = 0;
//         for (int i = 0; i < allUavRow->nw - 1; i++)
//         {
//             for (int j = allUavRow->q1[i] + allUavRow->w[i] + 1; j < allUavRow->q1[i + 1]; j++)
//             {
//                 for (int nW = 0; nW < UAVtypes[uavIndex].nPulseT; nW++)
//                 {
//                     if (abs((allUavRow->q1[i + 1] - allUavRow->q1[i]) * dt - UAVtypes[uavIndex].pulseT[nW]) < UAVtypes[uavIndex].pulseTErr)
//                     {
//                         averRealImag[0] += ((sumCorrAmp[uavCh][j][indK] * cos(sumCorrPhase[uavCh][j][indK]) - averReal));
//                         averRealImag[1] += ((sumCorrAmp[uavCh][j][indK] * sin(sumCorrPhase[uavCh][j][indK]) - averImag));
//                         tmpAmp += sumCorrAmp[uavCh][j][indK];
//                         n++;
//                         break;
//                     }
//                 }
//             }
//         }
//         if (n == 0)
//         {
//             for (int i = 0; i < allUavRow->nw - 1; i++)
//             {
//                 for (int j = allUavRow->q1[i] + allUavRow->w[i] + 1; j < allUavRow->q1[i + 1]; j++)
//                 {
//                     averRealImag[0] += ((sumCorrAmp[uavCh][j][indK] * cos(sumCorrPhase[uavCh][j][indK]) - averReal));
//                     averRealImag[1] += ((sumCorrAmp[uavCh][j][indK] * sin(sumCorrPhase[uavCh][j][indK]) - averImag));
//                     tmpAmp += sumCorrAmp[uavCh][j][indK];
//                     n++;
//                 }
//             }
//         }
//         if (useAmpOrAmpPh <= 2)
//         {
//             noiseAmp[k] = sqrt(averRealImag[0] * averRealImag[0] + averRealImag[1] * averRealImag[1]) / n;
//             snrAmp[k] = noiseAmp[k] - meanAmp[k];
//             snrAmp_1[k] = averAmpCorr[k];
//         }
//         else
//         {
//             noiseAmp[k] = tmpAmp / n;
//             snrAmp[k] = averAmpCorr[k] - noiseAmp[k];
//             snrAmp_1[k] = averAmpCorr[k];
//         }

//         // printf("----noiseAmp = %f,meanAmp = %f,snrAmp = %f\n",noiseAmp[k],meanAmp[k],snrAmp[k]);
//         if (snrAmp[k] > maxSnrAmp)
//             maxSnrAmp = snrAmp[k];
//     }
//     smoothSingleSig(indRCali - indLCali + 1, snrAmp);
//     smoothSingleSig(indRCali - indLCali + 1, snrAmp_1);
//     smoothSingleSig(indRCali - indLCali + 1, meanAmp);

//     thresh = findThresh(snrAmp, indRCali - indLCali + 1, 0.2, 3, &maxt);

//     float errL = 0, errR = 0;
//     /*
//         for(int k = 0 ; k <= indRCali - indLCali; k++)
//         {
//         printf("bbb = %f\n",snrAmp[k]);
//         }
//         printf("\n");
//         printf("1111111111111111111111111111111111111111\n");
//         printf("\n");
//         for(int k = 0 ; k <= indRCali - indLCali; k++)
//         {
//         printf("aaa = %f\n",snrAmp_1[k]);
//         }
//         printf("\n");
//         printf("1111111111111111111111111111111111111111\n");
//         printf("\n");
//     */

//     for (int k = 0; k <= indRCali - indLCali; k++)
//     {
//         // 	printf("%f\n",snrAmp[k]);
//         if (thresh < snrAmp[k])
//         {
//             if (count == 0)
//             {
//                 uavIndL[1] = k;
//                 if (k != 0)
//                     errL = snrAmp[k] - snrAmp[k - 1];
//                 else
//                     errL = snrAmp[k];
//             }
//             count++;
//             if (k == indRCali - indLCali)
//             {
//                 uavIndR[1] = k;
//                 errR = snrAmp[k];
//             }
//         }
//         else
//         {
//             if (tmpCount == k - 1)
//             {
//                 if (count != 0)
//                 {
//                     if (k == 1)
//                     {
//                         uavIndR[1] = k - 1;
//                         errR = snrAmp[k - 1] - snrAmp[k];
//                     }
//                     else
//                     {
//                         uavIndR[1] = k - 2;
//                         errR = snrAmp[k - 2] - snrAmp[k - 1];
//                     }
//                     count = 0;
//                 }
//                 tmpCount = 0;
//             }
//             if (count != 0)
//                 tmpCount = k;
//         }
//         if (uavIndR[1] - uavIndL[1] > round(UAVtypes[uavIndex].pulseBW[0] / dF) / 3 && count == 0)
//         {
//             if (uavIndL[1] + indLCali <= indCenFreq && uavIndR[1] + indLCali >= indCenFreq)
//                 break;
//             else
//             {
//                 count = 0;
//                 uavIndR[1] = 0;
//                 uavIndL[1] = 0;
//             }
//         }
//     }

//     //     for(int j = 0; j < UAVtypes[uavIndex].nPulseBW;j++)
//     //     {
//     //         if(uavIndR[1] - uavIndL[1] <= ceil(UAVtypes[uavIndex].pulseBW[j] / dF + 1))
//     //         {
//     //             caliBW[0] = UAVtypes[uavIndex].pulseBW[j];
//     //             break;
//     //         }
//     //     }

//     //     printf("uavIndR =  %d, uavIndL = %d      -------\n",uavIndR[1],uavIndL[1]);
//     //     printf("errL = %f,errR = %f ------- \n",errL,errR);
//     //     printf("  bw = %f ---------- \n",caliBW[0]);
//     float snrAmpErr[5] = {0};
//     float maxErr = -100;
//     int maxErrInd[5] = {0};
//     int maxInd = 0;
//     int flag = 0;
//     //     char file[100] = "./log.txt";
//     //     FILE *fp = fopen(file,"a+");

//     if (errL > errR)
//     {
//         for (int j = 0; j < UAVtypes[uavIndex].nPulseBW; j++)
//         {
//             int indexR = uavIndL[1] + (int)(UAVtypes[uavIndex].pulseBW[j] / dF);
//             snrAmpErr[j] = -100;
//             for (int k = indexR - 1; k <= indexR + 2; k++)
//             {
//                 if (k > indRCali - indLCali)
//                     break;
//                 // 		printf("%f,%f,%d,   ",snrAmp_1[k] - snrAmp_1[k+1],meanAmp[k+1] - meanAmp[k],k);
//                 if (snrAmp_1[k] - snrAmp_1[k + 1] > snrAmpErr[j])
//                 {
//                     snrAmpErr[j] = snrAmp_1[k] - snrAmp_1[k + 1];
//                     maxErrInd[j] = k;
//                 }
//             }
//             // 	    printf("\n");
//             if (snrAmpErr[j] > maxErr)
//             {
//                 maxInd = j;
//                 maxErr = snrAmpErr[j];
//             }
//         }

//         for (int j = 0; j < UAVtypes[uavIndex].nPulseBW; j++)
//         {
//             if (j == maxInd)
//                 continue;
//             if (maxErr - snrAmpErr[j] > 2.5 || maxErr > 4)
//             {
//                 flag = 1;
//             }
//             else if (maxErr - snrAmpErr[j] < 2.5 && maxErr > 3)
//             {
//                 flag = 3; //判决待定
//             }
//             else
//             {
//                 flag = 0;
//                 break;
//             }
//         }
//         float tmpMaxErr = maxErr;
//         int tmpMaxErrInd = maxErrInd[maxInd];

//         // 	fprintf(fp,"%f,%f,%f,%d,    ",snrAmpErr[0],snrAmpErr[1],snrAmpErr[0]-snrAmpErr[1],flag);
//         //
//         // 	flag = 0;
//         if (UAVtypes[uavIndex].nPulseBW == 1 && maxErr > 2)
//             flag = 1;

//         if (flag == 0 || flag == 3)
//         {
//             for (int j = 0; j < UAVtypes[uavIndex].nPulseBW; j++)
//             {
//                 int indexR = uavIndL[1] + (int)(UAVtypes[uavIndex].pulseBW[j] / dF);
//                 snrAmpErr[j] = -100;
//                 for (int k = indexR - 1; k <= indexR + 2; k++)
//                 {
//                     if (k + 1 > indRCali - indLCali)
//                         break;
//                     // 		    printf("%f,%f,%d,   ",snrAmp[k] - snrAmp[k+1] > snrAmp[k] - snrAmp[k+2] ? snrAmp[k] - snrAmp[k+1]:snrAmp[k] - snrAmp[k+2],snrAmp[k] - snrAmp[k+1],k);
//                     if (snrAmp[k] - snrAmp[k + 1] > snrAmpErr[j] || snrAmp[k] - snrAmp[k + 2] > snrAmpErr[j])
//                     {
//                         snrAmpErr[j] = snrAmp[k] - snrAmp[k + 1] > snrAmp[k] - snrAmp[k + 2] ? snrAmp[k] - snrAmp[k + 1] : snrAmp[k] - snrAmp[k + 2];
//                         maxErrInd[j] = k;
//                     }
//                 }
//                 // 		printf("\n");
//                 if (snrAmpErr[j] > maxErr)
//                 {
//                     maxInd = j;
//                     maxErr = snrAmpErr[j];
//                 }
//             }

//             for (int j = 0; j < UAVtypes[uavIndex].nPulseBW; j++)
//             {
//                 if (j == maxInd)
//                     continue;
//                 // if(maxErr - snrAmpErr[j] > 10)
//                 if (maxErr > 10 && maxErr > snrAmpErr[j])
//                 {
//                     flag = 2;
//                 }
//                 else
//                 {
//                     break;
//                 }
//             }

//             if (UAVtypes[uavIndex].nPulseBW == 1 && maxErr > 10)
//                 flag = 2;
//         }
//         // 	fprintf(fp,"%f,%f,%f,%d\n",snrAmpErr[0],snrAmpErr[1],snrAmpErr[0]-snrAmpErr[1],flag);

//         if (flag == 0 || uavIndL[1] - 1 < 0)
//             return -1;
//         if (flag == 3)
//         {
//             maxErr = tmpMaxErr;
//             maxErrInd[maxInd] = tmpMaxErrInd;
//         }
//         caliBW[0] = UAVtypes[uavIndex].pulseBW[maxInd];

//         if ((maxErr > snrAmp_1[uavIndL[1]] - snrAmp_1[uavIndL[1] - 1] && flag == 1) || (maxErr > snrAmp[uavIndL[1]] - snrAmp[uavIndL[1] - 1] && flag == 2))
//         {
//             *caliFreqInd = indLCali + maxErrInd[maxInd] - caliBW[0] / 2 / dF + 1;
//         }
//         else
//         {
//             *caliFreqInd = indLCali + uavIndL[1] + caliBW[0] / 2 / dF + 1;
//         }
//     }
//     else if (errL <= errR)
//     {
//         for (int j = 0; j < UAVtypes[uavIndex].nPulseBW; j++)
//         {
//             int indexL = uavIndR[1] - (int)(UAVtypes[uavIndex].pulseBW[j] / dF);
//             snrAmpErr[j] = -100;
//             for (int k = indexL - 2; k <= indexL + 1; k++)
//             {
//                 if (k < 0)
//                     continue;
//                 // 		printf("%f,%f,%d,    ",snrAmp_1[k+1] - snrAmp_1[k],meanAmp[k]-meanAmp[k+1],k);
//                 if (snrAmp_1[k + 1] - snrAmp_1[k] > snrAmpErr[j])
//                 {
//                     snrAmpErr[j] = snrAmp_1[k + 1] - snrAmp_1[k];
//                     maxErrInd[j] = k;
//                 }
//             }
//             // 	    printf("\n");
//             if (snrAmpErr[j] > maxErr)
//             {
//                 maxInd = j;
//                 maxErr = snrAmpErr[j];
//             }
//         }

//         for (int j = 0; j < UAVtypes[uavIndex].nPulseBW; j++)
//         {
//             if (j == maxInd)
//                 continue;
//             if (maxErr - snrAmpErr[j] > 2.5 || maxErr > 4)
//             {
//                 flag = 1;
//             }
//             else if (maxErr - snrAmpErr[j] < 2.5 && maxErr > 3)
//             {
//                 flag = 3; //判决待定
//             }
//             else
//             {
//                 flag = 0;
//                 break;
//             }
//         }
//         float tmpMaxErr = maxErr;
//         int tmpMaxErrInd = maxErrInd[maxInd];

//         // 	fprintf(fp,"%f,%f,%f,%d,    ",snrAmpErr[0],snrAmpErr[1],snrAmpErr[0]-snrAmpErr[1],flag);
//         flag = 0;
//         if (UAVtypes[uavIndex].nPulseBW == 1 && maxErr > 2)
//             flag = 1;

//         if (flag == 0 || flag == 3)
//         {
//             for (int j = 0; j < UAVtypes[uavIndex].nPulseBW; j++)
//             {
//                 int indexL = uavIndR[1] - (int)(UAVtypes[uavIndex].pulseBW[j] / dF);
//                 snrAmpErr[j] = -100;
//                 for (int k = indexL - 2; k <= indexL + 1; k++)
//                 {
//                     if (k < 0)
//                         continue;
//                     // 		    printf("%f,%f,%d,    ",snrAmp[k+1] - snrAmp[k] > snrAmp[k+2] - snrAmp[k]?snrAmp[k+1] - snrAmp[k]:snrAmp[k+2] - snrAmp[k],snrAmp[k+1] - snrAmp[k],k);
//                     if (snrAmp[k + 1] - snrAmp[k] > snrAmpErr[j] || snrAmp[k + 2] - snrAmp[k] > snrAmpErr[j])
//                     {
//                         snrAmpErr[j] = snrAmp[k + 1] - snrAmp[k] > snrAmp[k + 2] - snrAmp[k] ? snrAmp[k + 1] - snrAmp[k] : snrAmp[k + 2] - snrAmp[k];
//                         maxErrInd[j] = k;
//                     }
//                 }
//                 // 		printf("\n");
//                 if (snrAmpErr[j] > maxErr)
//                 {
//                     maxInd = j;
//                     maxErr = snrAmpErr[j];
//                 }
//             }

//             for (int j = 0; j < UAVtypes[uavIndex].nPulseBW; j++)
//             {
//                 if (j == maxInd)
//                     continue;
//                 if (maxErr > 10 && maxErr > snrAmpErr[j])
//                 {
//                     flag = 2;
//                 }
//                 else
//                 {
//                     break;
//                 }
//             }
//             if (UAVtypes[uavIndex].nPulseBW == 1 && maxErr > 10)
//                 flag = 2;
//         }
//         // 	fprintf(fp,"%f,%f,%f,%d\n",snrAmpErr[0],snrAmpErr[1],snrAmpErr[0]-snrAmpErr[1],flag);

//         if (flag == 0 || uavIndR[1] - 1 < 0)
//             return -1;
//         if (flag == 3)
//         {
//             maxErr = tmpMaxErr;
//             maxErrInd[maxInd] = tmpMaxErrInd;
//         }
//         caliBW[0] = UAVtypes[uavIndex].pulseBW[maxInd];
//         if (uavIndL[1] - 1 < 0)
//         {
//             free(meanAmp);
//             free(averAmpCorr);
//             free(noiseAmp);
//             free(snrAmp);
//             free(snrAmp_1);
//             return -1;
//         }
//         if ((maxErr > snrAmp_1[uavIndR[1] - 1] - snrAmp_1[uavIndL[1]] && flag == 1) || (maxErr > snrAmp[uavIndL[1] - 1] - snrAmp[uavIndL[1]] && flag == 2))
//         {
//             *caliFreqInd = indLCali + maxErrInd[maxInd] + caliBW[0] / 2 / dF + 1;
//         }
//         else
//         {
//             *caliFreqInd = indLCali + uavIndR[1] - caliBW[0] / 2 / dF + 1;
//         }
//     }
//     //     fclose(fp);
//     //    printf("-----------------------      %d,%d,%f,%f,%f",uavIndL[1],uavIndR[1],errL,errR,caliBW[0]);

//     free(meanAmp);
//     free(averAmpCorr);
//     free(noiseAmp);
//     free(snrAmp);
//     free(snrAmp_1);
//     if (uavIndR[1] == indRCali - indLCali || uavIndL[1] <= 0)
//         return -1;
//     return 0;
// }

//无人机图传检测和识别函数
int vidBlocksDetect(
    struct uavRow *aUAVRow, float *s, int n, int *q1, int *w, int nw, float *colSnr, int antIndex, int colIndex, float cenFreqOnAllCh, float dt, float dF, struct UAVLib *UAVtypes, int nUAV)
{
    struct uavRow aUAVRow1;
    aUAVRow1 = *aUAVRow;
    int i, j, k, checkRTK;
    int isUAV;
    int isInFreqSpan;               //在频点的左右边界内
    int mPulseW[maxPulseWNumInLib]; //记录不同脉宽满足容限的次数
    int Indexq[n];// = (int *)malloc(n * sizeof(int));
    int nIndex;
    float freqL, freqR;         //无人机频点的上下边界
    int freqIndexL, freqIndexR; //无人机频点的上下边界索引
    int count = 0;

    int match[MaxVidPulse][MaxNumVidT]; //记录倍数周期上的脉冲编号
    int meetPulseTCount = 0;            //脉宽符合要求，且满足脉冲周期的脉冲个数
    float minSnr = 0.0;                 //识别出的凸起中最低信噪比

    //计算脉冲结束的时刻
    float t2[nw];//= (float *)malloc(nw * sizeof(float));
    for (i = 0; i < nw; i++)
    {
        t2[i] = (q1[i] + w[i]) * dt;
    }
    for (i = 0; i < nUAV; i++) //逐个无人机进行比对
    {
        if (UAVtypes[i].useMethod != 1) //过滤掉不使用图传检测方法检测的无人机
            continue;

        isUAV = true;
        checkRTK = 0;
        //筛选，列索引号必须在无人机频段内
        if (UAVtypes[i].isFixedFreq) //若是频点固定的图传
        {
            isInFreqSpan = false;
            for (j = 0; j < UAVtypes[i].nfreqp; j++)
            {
                freqL = UAVtypes[i].freqPoints[j] - UAVtypes[i].pulseBW[UAVtypes[i].nPulseBW - 1] / 2;
                freqR = UAVtypes[i].freqPoints[j] + UAVtypes[i].pulseBW[UAVtypes[i].nPulseBW - 1] / 2;
                freqIndexL = floor((freqL - cenFreqOnAllCh) / dF + NFFT / NSumCol / 2);
                freqIndexR = floor((freqR - cenFreqOnAllCh) / dF + NFFT / NSumCol / 2);
                if (colIndex > freqIndexL && colIndex <= freqIndexR) //若列索引在频率索引范围之内
                {
                    isInFreqSpan = true;
                    break;
                }
            }
            if (!isInFreqSpan) //若列索引不在任何一个频点的左右边界之内，则跳过
                continue;
        }
        else //若是频点不固定的图传
        {
            freqL = UAVtypes[i].freqPoints[0];
            freqR = UAVtypes[i].freqPoints[1];
            freqIndexL = floor((freqL - cenFreqOnAllCh) / dF + NFFT / NSumCol / 2);
            freqIndexR = floor((freqR - cenFreqOnAllCh) / dF + NFFT / NSumCol / 2);

            if (colIndex <= freqIndexL || colIndex > freqIndexR) //若列索引在频率索引范围之外，则跳过
                continue;
        }

        //判断条件，脉宽误差必须满足一定次数
        for (j = 0; j < maxPulseWNumInLib; j++)
            mPulseW[j] = 0;
        nIndex = 0;
        for (j = 0; j < nw; j++)
        {
            for (k = 0; k < UAVtypes[i].nPulseW; k++)
            {
                if (fabs(UAVtypes[i].pulseW[k] - w[j] * dt) < UAVtypes[i].pulseWErr[k])
                {
                    Indexq[nIndex] = j; //记录满足脉宽的q1、w索引
                    nIndex++;           //脉宽满足容限的总个数
                    mPulseW[k]++;       //若脉宽满足容限，则计数
                    break;
                }
            }

            if (strcmp(UAVtypes[i].name, "Phantom 4 RTK") == 0 && fabs(3.1 - w[j] * dt) < 0.3) //排除御中RTK无人机
            {
                checkRTK++;
            }
        }

        if (checkRTK >= 2)
        {
            isUAV = false;
            continue;
        }

        for (j = 0; j < UAVtypes[i].nPulseW; j++)
        {
            if (mPulseW[j] < UAVtypes[i].meetPulseW[j])
            {
                isUAV = false;
                break;
            }
        }

        //判断条件，若是周期脉冲，满足脉宽误差的脉冲必须周期出现一定次数
        if (isUAV && UAVtypes[i].nPulseT > 0 && UAVtypes[i].pulseT[0] > 0) //若脉冲有周期
        {
            isUAV = false;

            matchPulseT(match, t2, Indexq, nIndex, UAVtypes[i].pulseT, UAVtypes[i].nPulseT, UAVtypes[i].pulseTErr);

            for (j = 0; j < nIndex; j++)
            {
                if (j > MaxNumVidT - 1)
                    break;
                meetPulseTCount = 0;
                for (k = 0; k < MaxNumVidT; k++)
                {
                    if (match[j][k] > 0)
                    {
                        meetPulseTCount++;
                    }
                }
                if (meetPulseTCount > 0 && meetPulseTCount >= UAVtypes[i].meetHopp)
                {
                    isUAV = true;
                    break;
                }
            }
            if (isUAV)
            {
                nIndex = 0;
                for (k = 0; k < MaxNumVidT; k++)
                {
                    if (match[j][k] >= 0)
                    {
                        Indexq[nIndex] = match[j][k];
                        nIndex++;
                    }
                }
            }
        }

        //每个凸起的信噪比必须大于阈值
        if (isUAV)
        {
            minSnr = 1000.0;
            for (j = 0; j < nIndex; j++)
            {
                if (colSnr[Indexq[j]] < minSnr)
                    minSnr = colSnr[Indexq[j]];
                if (colSnr[Indexq[j]] < UAVtypes[i].SNR)
                {
                    isUAV = false;
                    break;
                }
            }
        }

        if (isUAV)
        {
            aUAVRow1.uavIndex = i;        //无人机在特征库内的编号
            aUAVRow1.antIndex = antIndex; //天线编号
            aUAVRow1.colIndex = colIndex; //列索引
            aUAVRow1.snr = minSnr;
            aUAVRow1.nCol = 0; //相同脉冲列的列索引个数
            aUAVRow1.meanColAmp = 0.0;
            count = 0;
            for (j = 0; j < nIndex; j++)
            {
                aUAVRow1.q1[j] = q1[Indexq[j]]; //脉冲起点索引
                aUAVRow1.w[j] = w[Indexq[j]];   //脉冲宽度索引
                for (k = aUAVRow1.q1[j]; k < aUAVRow1.q1[j] + aUAVRow1.w[j]; k++)
                {
                    aUAVRow1.meanColAmp += s[k];
                }
                count += aUAVRow1.w[j];
            }
            aUAVRow1.meanColAmp /= (float)count;
            aUAVRow1.nw = nIndex; //检测到的脉宽个数
            // free(Indexq);
            // Indexq = NULL;
            // free(t2);
            // t2 = NULL;
            *aUAVRow = aUAVRow1;
            return true;
        }
    }
    // free(Indexq);
    // Indexq = NULL;
    // free(t2);
    // t2 = NULL;
    *aUAVRow = aUAVRow1;
    return false;
}

// //剔除野值
// void robustAmp(float *colVecFilted, int n)
// {
//     int iter = 3;
//     float *probs = (float *)malloc(sizeof(float) * n);
//     float mean_ = 0, std_ = 0, sumProbs = 0, sumAmp = 0, sumStd = 0, minAmp = 1e5, maxAmp = 0;

//     for (int i = 0; i < n; i++)
//     {
//         probs[i] = 1;
//         if (colVecFilted[i] < minAmp)
//             minAmp = colVecFilted[i];
//         if (colVecFilted[i] > maxAmp)
//             maxAmp = colVecFilted[i];
//     }

//     for (int i = 0; i < iter; i++)
//     {
//         sumProbs = 0, sumAmp = 0, sumStd = 0;
//         for (int j = 0; j < n; j++)
//         {
//             sumAmp += probs[j] * colVecFilted[j];
//             sumProbs += probs[j];
//         }
//         mean_ = sumAmp / sumProbs;
//         // printf("mean_ = %f,sumAmp = %f, sumProbs = %f\n",mean_, sumAmp, sumProbs);
//         if (i == iter - 1)
//             break;
//         for (int j = 0; j < n; j++)
//         {
//             sumStd += pow(probs[i] - mean_, 2) * probs[j];
//         }
//         std_ = sqrt(sumStd) / sumProbs;
//         if (std_ == 0 || maxAmp == minAmp)
//             return;

//         for (int j = 0; j < n; j++)
//         {
//             probs[j] = 0.8 * exp(-pow((colVecFilted[i] - mean_), 2) / (2 * pow(std_, 2))) / sqrt(2 * PI) / std_;
//             probs[j] = probs[j] / (probs[j] + 0.2 / (maxAmp - minAmp));
//         }
//     }

//     for (int i = 0; i < n; i++)
//     {
//         if (abs(colVecFilted[i] - mean_) / std_ > 3)
//         {
//             colVecFilted[i] = mean_;
//         }
//     }
//     // printf("mean = %f\n",mean_);
//     free(probs);
//     return;
// }

void delFalseRTK(struct UAVLib *UAVtypes, struct uavRow *allUavRow, int *nUavRow)
{
    // int delUavN = 0;
    // std::vector<int> delInd;

    // for(int i = 0; i < nUavRow; i++)
    // {
    // if(strcmp(UAVtypes[allUavRow[i].uavIndex].name,"Mavic 2/Phantom 4P/Mavic Air2/Air2s/FPV") == 0)
    // {
    //     for(int j = 0; j < nUavRow; j++)
    //     {
    // 	if(j == i) continue;
    // 	if(strcmp(UAVtypes[allUavRow[j].uavIndex].name,"Phantom 4 RTK") == 0)
    // 	{
    // 	    //printf("%d,%d,%d,%d\n",(allUavRow[i].allColIndex[0] - 2) , allUavRow[j].allColIndex[0] , (allUavRow[i].allColIndex[allUavRow[i].nCol - 1] + 2) , allUavRow[j].allColIndex[allUavRow[j].nCol - 1]);
    // 	    if(((allUavRow[i].allColIndex[0] - 2) < allUavRow[j].allColIndex[0]) && ((allUavRow[i].allColIndex[allUavRow[i].nCol - 1] + 2) > allUavRow[j].allColIndex[allUavRow[j].nCol - 1]))
    // 	    {
    // 		delInd.push_back(j);
    // 	    }
    // 	}
    //     }
    // }
    // }
    // std::sort(delInd.begin(),delInd.end());
    // for(int i = 0; i < delInd.size(); i++)
    // {
    // for(int j = 0; j < nUavRow; j++)
    // {
    //     if(delInd.at(i) == j)
    //     {
    // 	for(int k = j; k < nUavRow - 1; k++)
    // 	{
    // 	    allUavRow[k] = allUavRow[k+1];
    // 	}
    // 	break;
    //     }
    // }
    // nUavRow = nUavRow - 1;
    // }
}
/*
   从频谱中每列检测是否含有无人机，形成脉冲列结构体，并合并不同列和不同通道的脉冲列
 */
void uavRowDetect(
    struct uavRow *allUavRow, int *nUavRow, float (*sumCorrAmp)[NFFT / NSumCol], float (*filtedAmp)[NFFT / NSumCol], float (*filtedAmpIntegral)[NFFT / NSumCol], int nRows, int nCols, float *cenFreqOnAllCh, float dt, float dF, struct detectParams detectedParam,
    struct UAVLib *UAVtypes, int nUAV)
{
    int nUavRow1 = *nUavRow;
    int i, j, k;
    int ii, jj, kk;
    int corrFlag = (detectedParam.doaTyp == 5) ? 2 : 1;
    int antennaNum = 1;
    float thresh, maxt = 0.0; //阈值
    int nw;                   //阈值分割后，q1和w中的数量
    float snr = 0;            //阈值分割得到的信噪比
    struct uavRow aUavRow;    //检测到一个无人机脉冲列
    int nqMeet = 0;           //记录q1比对时，满足的次数
    int isqMeet = false;      // q1是否能合并
    int isNewCol = false;     //是否是新的列索引
    int isNewq1w = false;     //是否是新的q1和w
    int id = 0;               //无人机脉冲ID编号，同一个无人机的不同通道的所有脉冲是同一个ID号
    int nSpan[2];             //频率列索引范围

    int maskW = detectedParam.maskW;
    int maskH = detectedParam.maskH;
    int q1IndexErr = detectedParam.q1IndexErr;
    float DT = detectedParam.DT;
    int MaxIter = detectedParam.MaxIter;
    int qMeetTimes = detectedParam.qMeetTimes;
    int minqMeet = 0;
    int Delt = detectedParam.Delt;
    int MinW = detectedParam.MinW;

//     float *colVecFilted = (float *)malloc(nRows * sizeof(float)), *colVec = (float *)malloc(nRows * sizeof(float));
//     int *q1 = (int *)malloc(nRows * sizeof(int));               //凸起的起点索引
//     int *w = (int *)malloc(nRows * sizeof(int));                //凸起的宽度索引
//     float *colMeanAmp = (float *)malloc(nRows * sizeof(float)); //凸起的幅度
//     float *colSnr = (float *)malloc(nRows * sizeof(float));     //凸起的信噪比
	
	float colVecFilted[NROWS]; //= (float *)malloc(nRows * sizeof(float));
	float colVec[NROWS]; // = (float *)malloc(nRows * sizeof(float));
    int q1[NROWS]; //(int *)malloc(nRows * sizeof(int));               //凸起的起点索引
    int w[NROWS]; //(int *)malloc(nRows * sizeof(int));                //凸起的宽度索引
    float colMeanAmp[NROWS]; //(float *)malloc(nRows * sizeof(float)); //凸起的幅度
    float colSnr[NROWS]; //(float *)malloc(nRows * sizeof(float));     //凸起的信噪比

    nUavRow1 = 0; //脉冲列初始化为0
    for (i = 0; i < antennaNum; i++)
    {
        int ind = 0;
        if (detectedParam.doaTyp == 5 && i >= 4)
            ind = (i + 1) % 5;
        else
            ind = i;
        nSpan[0] = detectedParam.vidIndexSpan[ind];
        nSpan[1] = detectedParam.vidIndexSpan[ind + NCorr];
        // for (j = (maskW + 1) / 2; j < NFFT / NSumCol - (maskW - 1) / 2; j++)
        for (j = nSpan[0]; j < nSpan[1]; j++)
        {
            if (j == 128)
                continue;
            //装入一列数据
            for (k = (maskH + 1) / 2; k < nRows - (maskH - 1) / 2; k++)
            {
                colVecFilted[k - (maskH + 1) / 2] = filtedAmp[k][j];
                colVec[k - (maskH + 1) / 2] = sumCorrAmp[k][j];
            }
            colVecFilted[nRows - maskH - 1] = filtedAmp[nRows - maskH - 2][j];

            // 	    for (k = (maskH + 1) / 2; k < nRows - (maskH - 1) / 2; k++)
            //             {
            // 		if(colVecFilted[k - (maskH + 1) / 2] > maxColAmp) maxColAmp = colVecFilted[k - (maskH + 1) / 2];
            // 		if(colVecFilted[k - (maskH + 1) / 2] < minColAmp) minColAmp = colVecFilted[k - (maskH + 1) / 2];
            // 	    }
            // 	    for (k = (maskH + 1) / 2; k < nRows - (maskH - 1) / 2; k++)
            //             {
            // 		colVecFilted[k - (maskH + 1) / 2] = log10((colVecFilted[k - (maskH + 1) / 2] - minColAmp)/(maxColAmp - minColAmp) * 1 + 1);
            // 	    }
            // 	    robustAmp(colVecFilted, nRows - maskH);

            thresh = findThresh(colVecFilted, nRows - maskH, DT, MaxIter, &maxt); //计算阈值
                                                                                  // 			printf("j = %d, thresh = %f\n", j, thresh);

            // 			if(maxt < detectedParam.filterThresh && detectedParam.filterMethod == 1)//
            // 			{
            // 				for (k = (maskH + 1) / 2; k < nRows - (maskH - 1) / 2; k++)
            // 				{
            // 					colVecFilted[k - (maskH + 1) / 2] = filtedAmpIntegral[i][k][j];
            // 				}
            // 				colVecFilted[nRows - maskH - 1] = filtedAmpIntegral[i][nRows - maskH - 2][j];
            // 				thresh = findThresh(colVecFilted, nRows - maskH, DT, MaxIter, maxt);                    //计算阈值
            // 			}
            //
            // 			if(thresh > detectedParam.filterThresh && detectedParam.filterMethod == 2)
            // 			{
            // 				robustAmp(colVecFilted, nRows - maskH);
            // 				thresh = findThresh(colVecFilted, nRows - maskH, DT, MaxIter, maxt);                    //计算阈值
            // 			}
            nw = threshSegm(colVecFilted, nRows - maskH, thresh, Delt, MinW, q1, w, colSnr); //阈值分割

            //补偿首端的索引号
            for (k = 0; k < nw; k++)
                q1[k] += (maskH + 1) / 2;
            //将检测到的无人机脉冲列装入数组，去重
            if (vidBlocksDetect(&aUavRow, colVec, nRows - maskH, q1, w, nw, colSnr, i, j, cenFreqOnAllCh[0], dt, dF, UAVtypes, nUAV))
            {
                isqMeet = false;

                //针对脉冲很少，比如少于qMeetTimes的情况，合并条件应该由最大满足脉宽次数决定
                minqMeet = 0;
                for (k = 0; k < UAVtypes[aUavRow.uavIndex].nPulseW; k++)
                {
                    if (UAVtypes[aUavRow.uavIndex].meetPulseW[k] > minqMeet)
                        minqMeet = UAVtypes[aUavRow.uavIndex].meetPulseW[k];
                }

                int q1MeetNum[MaxPulseInGroup];
                int maxNumIndex = -1;
                for (k = 0; k < nUavRow1; k++)
                {
                    q1MeetNum[k] = 0;
                    if (aUavRow.uavIndex != allUavRow[k].uavIndex)
                    {
                        continue;
                    }
                    nqMeet = 0;
                    for (ii = 0; ii < allUavRow[k].nw; ii++)
                    {
                        for (jj = 0; jj < aUavRow.nw; jj++)
                        {
                            if (abs(allUavRow[k].q1[ii] - aUavRow.q1[jj]) < q1IndexErr)
                            {
                                if (abs(allUavRow[k].w[ii] - aUavRow.w[jj]) < q1IndexErr)
                                    nqMeet++;
                            }
                        }
                    }

                    if (nqMeet >= qMeetTimes || nqMeet >= minqMeet)
                    {
                        q1MeetNum[k] = nqMeet;
                        if (maxNumIndex < 0)
                        {
                            maxNumIndex = k;
                        }
                        else if (q1MeetNum[maxNumIndex] < nqMeet)
                        {
                            maxNumIndex = k;
                        }
                    }
                }

                if (maxNumIndex > -1) //合并去重，幅度最大者保留
                {
                    k = maxNumIndex;
                    isqMeet = true;
                    allUavRow[k].onAnt[aUavRow.antIndex] = true;

                    isNewCol = true;
                    for (ii = 0; ii < allUavRow[k].nCol; ii++)
                    {
                        if (allUavRow[k].allColIndex[ii] == aUavRow.colIndex)
                        {
                            isNewCol = false;
                            break;
                        }
                    }
                    if (isNewCol) //插入新的脉冲列索引
                    {
                        if (allUavRow[k].allColIndex[allUavRow[k].nCol - 1] < aUavRow.colIndex)
                        {
                            allUavRow[k].allColIndex[allUavRow[k].nCol] = aUavRow.colIndex;
                            allUavRow[k].nCol++;
                        }
                        else
                        {
                            for (ii = 0; ii < allUavRow[k].nCol; ii++)
                            {
                                if (allUavRow[k].allColIndex[ii] > aUavRow.colIndex) //找到插入点
                                {
                                    for (jj = allUavRow[k].nCol; jj > ii; jj--)
                                    {
                                        allUavRow[k].allColIndex[jj] = allUavRow[k].allColIndex[jj - 1];
                                    }
                                    allUavRow[k].allColIndex[ii] = aUavRow.colIndex;
                                    allUavRow[k].nCol++;
                                    break;
                                }
                            }
                        }
                    }
                    //合并q1、w
                    for (ii = 0; ii < aUavRow.nw; ii++)
                    {
                        isNewq1w = true;
                        for (jj = 0; jj < allUavRow[k].nw; jj++)
                        {
                            if (aUavRow.q1[ii] < allUavRow[k].q1[jj])
                            {
                                if (aUavRow.q1[ii] + aUavRow.w[ii] > allUavRow[k].q1[jj])
                                {
                                    isNewq1w = false;
                                    break;
                                }
                            }
                            else
                            {
                                if (aUavRow.q1[ii] < allUavRow[k].q1[jj] + allUavRow[k].w[jj])
                                {
                                    isNewq1w = false;
                                    break;
                                }
                            }
                        }
                        if (isNewq1w) //插入新的q1和w
                        {
                            if (allUavRow[k].q1[allUavRow[k].nw - 1] < aUavRow.q1[ii])
                            {
                                allUavRow[k].q1[allUavRow[k].nw] = aUavRow.q1[ii];
                                allUavRow[k].w[allUavRow[k].nw] = aUavRow.w[ii];
                                allUavRow[k].nw++;
                            }
                            else
                            {
                                for (jj = 0; jj < allUavRow[k].nw; jj++)
                                {
                                    if (allUavRow[k].q1[jj] > aUavRow.q1[ii])
                                    {
                                        for (kk = allUavRow[k].nw; kk > jj; kk--)
                                        {
                                            allUavRow[k].q1[kk] = allUavRow[k].q1[kk - 1];
                                            allUavRow[k].w[kk] = allUavRow[k].w[kk - 1];
                                        }
                                        allUavRow[k].q1[jj] = aUavRow.q1[ii];
                                        allUavRow[k].w[jj] = aUavRow.w[ii];
                                        allUavRow[k].nw++;
                                        break;
                                    }
                                }
                            }
                        }
                        else
                        {
                            if (allUavRow[k].meanColAmp < aUavRow.meanColAmp)
                            {
                                //用幅度更强的q1和w更新
                                allUavRow[k].q1[jj] = aUavRow.q1[ii];
                                allUavRow[k].w[jj] = aUavRow.w[ii];
                            }
                        }
                    }
                    if (allUavRow[k].meanColAmp < aUavRow.meanColAmp)
                    {
                        //替换部分记录
                        allUavRow[k].antIndex = aUavRow.antIndex;
                        allUavRow[k].colIndex = aUavRow.colIndex;
                        allUavRow[k].meanColAmp = aUavRow.meanColAmp;
                        allUavRow[k].snr = aUavRow.snr;
                    }
                }

                if (!isqMeet && nUavRow1 < MaxUAV * antennaNum) //没有找到可以合并的脉冲列，则加入到数组
                {
                    //将aUavRow的值追加到allUavRow数组末尾
                    allUavRow[nUavRow1].antIndex = aUavRow.antIndex;
                    allUavRow[nUavRow1].colIndex = aUavRow.colIndex;
                    allUavRow[nUavRow1].uavIndex = aUavRow.uavIndex;
                    for (k = 0; k < antennaNum; k++)
                    {
                        if (k == aUavRow.antIndex)
                            allUavRow[nUavRow1].onAnt[k] = true;
                        else
                            allUavRow[nUavRow1].onAnt[k] = false;
                    }
                    allUavRow[nUavRow1].nw = aUavRow.nw;
                    allUavRow[nUavRow1].meanColAmp = aUavRow.meanColAmp;
                    for (k = 0; k < aUavRow.nw; k++)
                    {
                        allUavRow[nUavRow1].q1[k] = aUavRow.q1[k];
                        allUavRow[nUavRow1].w[k] = aUavRow.w[k];
                    }
                    // allUavRow数组末尾元素修改参数值
                    allUavRow[nUavRow1].nCol = 1;
                    allUavRow[nUavRow1].allColIndex[0] = aUavRow.colIndex;
                    allUavRow[nUavRow1].id = id;
                    allUavRow[nUavRow1].snr = aUavRow.snr;
                    id++;
                    nUavRow1++;
                }
            }
        }
    }
    *nUavRow = nUavRow1;
    delFalseRTK(UAVtypes, allUavRow, &nUavRow1);

//     //清理开辟的内存
//     free(colVecFilted);
//     colVecFilted = NULL;
//     free(colVec);
//     colVec = NULL;
//     free(q1);
//     q1 = NULL; //凸起的起点索引
//     free(w);
//     w = NULL; //凸起的宽度索引
//     free(colMeanAmp);
//     colMeanAmp = NULL;
//     free(colSnr);
//     colSnr = NULL;
}

/*
 * 从脉冲列生成脉冲串
 */
void genUavPulse(
    struct pulseGroup *uavPulse, int *nUavPulse, struct uavRow *allUavRow, int nUavRow, float *sumCorr, float (*sumCorrAmp)[NFFT / NSumCol], float (*sumCorrPhase)[NFFT / NSumCol], int nRows, int nCols, float *cenFreqOnAllCh,
    float dt, float dF, struct UAVLib *UAVtypes, struct detectParams detectedParam)
{
    int i, j, k, ii, jj;
    int nUavPulse1 = *nUavPulse;
    int uavIndex, cenCol;
    float colAmp;
    float pulseFreq;                            //脉冲中心频率
    int freqIndexL, freqIndexR, IndexL, IndexR; //脉冲带宽左右边界的列索引
    float minBW;                                //频点不固定的图传最小带宽
    float meanColAmp[NFFT / NSumCol] = {
        0,
        0,
    }; //搜索带宽时计算每列的平均幅度
    int corrFlag = (detectedParam.doaTyp == 5) ? 2 : 1;

    int uavCh; //检出无人机的天线编号
    int count, count1;
    int index, archiFlag = 1; //螺旋天线相位取返标志位
    float corrSumReal = 0.0, corrSumImage = 0.0;
    float corrSumReal1 = 0.0, corrSumImage1 = 0.0;
    float squareSumPhase = 0.0, sumPhase = 0.0;
    float squareSumPhaseWrap = 0.0, sumPhaseWrap = 0.0;
    float phaseWrap;
    float phaseVarBlock = 0.0, phaseVarWrapBlock = 0.0; //相位方差
    int phaseCount;

    float ampErr = detectedParam.ampErr;
    float initPhase[NCh], d[NCh];
    int antennaNum = 1;
    // float *sumReal = (float *)malloc(sizeof(float) * 40 * 1500);
    // float *sumImag = (float *)malloc(sizeof(float) * 40 * 1500);
    // float *sumReal1 = (float *)malloc(sizeof(float) * 40 * 1500);
    // float *sumImag1 = (float *)malloc(sizeof(float) * 40 * 1500);

    // int *shieldFreqInd = (int *)malloc(sizeof(int) * nCols);

    // getShieldFreqInd(shieldFreqInd, cenFreqOnAllCh[0], detectedParam, nCols, dF);

    nUavPulse1 = 0;
    for (i = 0; i < nUavRow; i++)
    {
        uavIndex = allUavRow[i].uavIndex;
        uavPulse[nUavPulse1].uavIndex = uavIndex;
        if (UAVtypes[uavIndex].isFixedFreq) //若频点固定，中心频率和带宽查询特征库即可
        {
            uavPulse[nUavPulse1].pulseBW = UAVtypes[uavIndex].pulseBW[0];

            cenCol = allUavRow[i].colIndex;

            //找到幅度最强的检出无人机的天线编号
            colAmp = -1000.0;
            for (j = 0; j < antennaNum; j++)
            {
                if (allUavRow[i].onAnt[j] && allUavRow[i].meanColAmp > colAmp)
                {
                    uavCh = j;
                    colAmp = allUavRow[i].meanColAmp;
                }
            }
            int ind = 0;
            if (detectedParam.doaTyp == 5 && uavCh >= 4)
                ind = (uavCh + 1) % 5;
            else
                ind = uavCh;

            //比较截面幅度在相邻信道内的能量
            colAmp = 0.0;
            for (j = 0; j < allUavRow[i].nCol; j++)
            {
                meanColAmp[j] = 0;
                count = 0;
                for (k = 0; k < allUavRow[i].nw; k++)
                {
                    for (ii = allUavRow[i].q1[k]; ii < allUavRow[i].q1[k] + allUavRow[i].w[k]; ii++)
                    {
                        meanColAmp[j] += sumCorrAmp[ii][allUavRow[i].allColIndex[j]];
                        count++;
                    }
                }
                if (count > 0)
                    meanColAmp[j] /= (float)count;
                colAmp += meanColAmp[j];
            }

            pulseFreq = cenFreqOnAllCh[0] + cenCol * dF - 0.5 * nCols * dF;
            for (j = 0; j < UAVtypes[uavIndex].nfreqp; j++)
            {
                if (pulseFreq <= UAVtypes[uavIndex].freqPoints[j] + UAVtypes[uavIndex].pulseBW[0] / 2)
                    break;
            }

            if (j < UAVtypes[uavIndex].nfreqp - 1 && pulseFreq > UAVtypes[uavIndex].freqPoints[j + 1] - UAVtypes[uavIndex].pulseBW[0] / 2) //若同时在右边相邻的信道内，则根据重叠率二选一
            {
                colAmp = 0;
                for (k = 0; k < allUavRow[i].nCol; k++) //与左右的信道比较，落入信道内的列索引计数
                {
                    pulseFreq = cenFreqOnAllCh[0] + allUavRow[i].allColIndex[k] * dF - 0.5 * nCols * dF;
                    if (pulseFreq > UAVtypes[uavIndex].freqPoints[j] - UAVtypes[uavIndex].pulseBW[0] / 2 && pulseFreq <= UAVtypes[uavIndex].freqPoints[j] + UAVtypes[uavIndex].pulseBW[0] / 2)
                        colAmp += meanColAmp[k];
                    if (pulseFreq > UAVtypes[uavIndex].freqPoints[j + 1] - UAVtypes[uavIndex].pulseBW[0] / 2 && pulseFreq <= UAVtypes[uavIndex].freqPoints[j + 1] + UAVtypes[uavIndex].pulseBW[0] / 2)
                        colAmp -= meanColAmp[k];
                }
                if (colAmp < 0) //若左边的信道计数小于右边，则判断落入右边的信道
                    j++;
            }
            for (k = 0; k < allUavRow[i].nw; k++)
                uavPulse[nUavPulse1].freq[k] = UAVtypes[uavIndex].freqPoints[j];

            freqIndexL = floor((uavPulse[nUavPulse1].freq[0] - uavPulse[nUavPulse1].pulseBW / 2 - (cenFreqOnAllCh[0] - 0.5 * nCols * dF)) / dF);
            freqIndexR = floor((uavPulse[nUavPulse1].freq[0] + uavPulse[nUavPulse1].pulseBW / 2 - (cenFreqOnAllCh[0] - 0.5 * nCols * dF)) / dF);
            //频谱边界限制
            if (freqIndexR >= NFFT / NSumCol - 5)
                freqIndexR = NFFT / NSumCol - 6;
            if (freqIndexL < 5)
                freqIndexL = 5;
        }
        else //无人机频点不固定，中心频率和带宽需要搜索
        {
            minBW = UAVtypes[uavIndex].pulseBW[0]; //该类无人机图传的最小带宽（特征库内第一个带宽）
                                                   //找到幅度最强的检出无人机的天线编号
                                                   //             colAmp = -1000.0;
                                                   //             for (j = 0; j < antennaNum; j++)
                                                   //             {
                                                   //                 if (allUavRow[i].onAnt[j] && allUavRow[i].meanColAmp > colAmp)
                                                   //                 {
                                                   //                     uavCh = j;
                                                   //                     colAmp = allUavRow[i].meanColAmp;
                                                   //                 }
                                                   //             }
            uavCh = allUavRow[i].antIndex;
            colAmp = allUavRow[i].meanColAmp;
            int ind = 0;
            if (detectedParam.doaTyp == 5 && uavCh >= 4)
                ind = (uavCh + 1) % 5;
            else
                ind = uavCh;

            //检测到无人机的列范围
            IndexL = allUavRow[i].allColIndex[0];
            IndexR = allUavRow[i].allColIndex[0];
            for (j = 0; j < allUavRow[i].nCol; j++)
            {
                if (allUavRow[i].allColIndex[j] < IndexL)
                    IndexL = allUavRow[i].allColIndex[j];
                if (allUavRow[i].allColIndex[j] > IndexR)
                    IndexR = allUavRow[i].allColIndex[j];
            }

            if ((IndexR - IndexL + 1) * dF < UAVtypes[uavIndex].pulseBW[0])
            {
                freqIndexR = IndexR;
                freqIndexL = IndexL;
            }
            else
            {
                for (j = IndexL; j < IndexR; j++)
                {
                    meanColAmp[j] = 0;
                    count = 0;
                    for (k = 0; k < allUavRow[i].nw; k++)
                    {
                        for (ii = allUavRow[i].q1[k]; ii < allUavRow[i].q1[k] + allUavRow[i].w[k]; ii++)
                        {
                            meanColAmp[j] += sumCorrAmp[ii][j];
                            count++;
                        }
                    }
                    if (count > 0)
                        meanColAmp[j] /= count;
                }

                cenCol = allUavRow[i].colIndex;
                float sumBWamp[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
                int indexBl[5] = {0, 0, 0, 0, 0};
                int indexBr[5] = {0, 0, 0, 0, 0};
                for (j = 0; j < UAVtypes[uavIndex].nPulseBW; j++)
                {
                    sumBWamp[j] = 0.0;
                    float sumAmp = 0.0;
                    int nColBW = floor(UAVtypes[uavIndex].pulseBW[j] / dF);
                    for (k = IndexL; k < IndexR + 1; k++)
                    {
                        sumAmp = 0.0;
                        int colCount = 0;
                        for (int p = 0; p < nColBW; p++)
                        {
                            if (k + p < IndexR + 1)
                            {
                                sumAmp += meanColAmp[k + p];
                                colCount++;
                            }
                        }
                        sumAmp /= (float)colCount;
                        if (sumAmp > sumBWamp[j])
                        {
                            sumBWamp[j] = sumAmp;
                            indexBl[j] = k;
                            indexBr[j] = k + colCount;
                        }
                    }
                }

                k = 0;
                for (j = 1; j < UAVtypes[uavIndex].nPulseBW; j++)
                {
                    if (abs(sumBWamp[j] - sumBWamp[0]) < ampErr)
                        k = j;
                }

                freqIndexL = indexBl[k];
                freqIndexR = indexBr[k];

                /*
                freqIndexL = -1;  //标记为-1
                for (j = IndexL; j < cenCol; j++)
                {
                    if (cenCol - j <= minBW / dF)
                    {
                        if (fabs(meanColAmp[j] - meanColAmp[cenCol]) < ampErr && fabs(meanColAmp[j + 1] - meanColAmp[cenCol]) < ampErr)
                        {
                            freqIndexL = j;
                            break;
                        }
                    }
                }
                freqIndexR = -1;  //标记为-1
                for (j = IndexR; j > cenCol; j--)
                {
                    if (j - cenCol <= minBW / dF)
                    {
                        if (fabs(meanColAmp[j] - meanColAmp[cenCol]) < ampErr && fabs(meanColAmp[j - 1] - meanColAmp[cenCol]) < ampErr)
                        {
                            freqIndexR = j;
                            break;
                        }
                    }
                }
                if (freqIndexR < 0)
                {
                    if (allUavRow[i].colIndex < NFFT / NSumCol - 6)
                        freqIndexR = allUavRow[i].colIndex + 1;
                    else
                        freqIndexR = allUavRow[i].colIndex;
                }
                if (freqIndexL < 0)
                {
                    if (allUavRow[i].colIndex < 6)
                        freqIndexL = allUavRow[i].colIndex - 1;
                    else
                        freqIndexL = allUavRow[i].colIndex;
                }
                */
            }

            //             uavPulse[nUavPulse].pulseBW = (freqIndexR + 1 - freqIndexL) * dF;
            //             uavPulse[nUavPulse].freq[0] = cenFreqOnAllCh[uavCh] - 0.5 * nCols*dF + (freqIndexL + freqIndexR) / 2.0 * dF;
            //             for (j = 1; j < allUavRow[i].nw; j++)
            //                 uavPulse[nUavPulse].freq[j] = uavPulse[nUavPulse].freq[0];

            // 重新计算频点、带宽
            float maxBW = 0;
            float caliBW[5] = {0.0};
            int caliFreqInd; //计算的频点位置
            for (int i = 0; i < UAVtypes[uavIndex].nPulseBW; i++)
            {
                if (UAVtypes[uavIndex].pulseBW[i] > maxBW)
                    maxBW = UAVtypes[uavIndex].pulseBW[i];
            }

            float calcFreq = cenFreqOnAllCh[0] - 0.5 * nCols * dF + (freqIndexL + freqIndexR) / 2.0 * dF; //重新计算前算到的中频
            int indFreq = (freqIndexL + freqIndexR) / 2.0;                                                //重新计算前的频点位置
            int indLCali = ((calcFreq - maxBW * 1.1) - (cenFreqOnAllCh[0] - 0.5 * nCols * dF)) / dF;      //频点左边界
            int indRCali = ((calcFreq + maxBW * 1.1) - (cenFreqOnAllCh[0] - 0.5 * nCols * dF)) / dF;      //频点右边界
            indLCali = indLCali >= 6 ? indLCali : 6;
            indRCali = indRCali <= nCols - 6 ? indRCali : nCols - 6;

            // 	    int ret;
            //
            // 	    printf("uavCh = %d\n ",uavCh);
            // 	    if(detectedParam.isCaliFreqBw == 3)
            // 		ret = caliFreqBw(&allUavRow[i], sumCorrAmp, sumCorrPhase, uavCh, indLCali, indRCali, indFreq, UAVtypes, uavIndex, &caliFreqInd,caliBW, dF,dt);//calcAverRealImag
            // 	    else if(detectedParam.isCaliFreqBw == 2)
            // 		ret = caliCenFreqBw(&allUavRow[i], sumCorrAmp, sumCorrPhase, uavCh, indLCali, indRCali, indFreq, UAVtypes, uavIndex, &caliFreqInd,caliBW, dF,dt,freqIndexL,freqIndexR);//calcAverRealImag
            // 	    else
            // 		ret = -1;
            // 	    printf("---------------      %d\n",ret);

            // 	    if (ret == -1)
            // 	    {
            uavPulse[nUavPulse1].pulseBW = (freqIndexR + 1 - freqIndexL) * dF;
            uavPulse[nUavPulse1].freq[0] = cenFreqOnAllCh[0] - 0.5 * nCols * dF + (freqIndexL + freqIndexR) / 2.0 * dF;
            for (j = 1; j < allUavRow[i].nw; j++)
                uavPulse[nUavPulse1].freq[j] = uavPulse[nUavPulse1].freq[0];
            // 	    }
            // 	    else
            // 	    {
            // 		uavPulse[nUavPulse].pulseBW = caliBW[0];
            // 		uavPulse[nUavPulse].freq[0] = cenFreqOnAllCh[0] - 0.5 * nCols*dF + caliFreqInd * dF;
            // 		for (j = 1; j < allUavRow[i].nw; j++)
            // 		    uavPulse[nUavPulse].freq[j] = uavPulse[nUavPulse].freq[0];
            // 	    }
        }
        //计算脉冲时间、脉宽、平均幅度、平均相位和相位方差
        // selAntParamNew(initPhase, uavPulse[nUavPulse].freq[0], detectedParam);
        selAntParamPro(initPhase, uavPulse[nUavPulse1].freq[0], detectedParam, d);
//        printf(
//            "initPhase=%f,%f,%f,%f,%f\n",
//            initPhase[0],
//            initPhase[1],
//            initPhase[2],
//            initPhase[3],
//            initPhase[4]);
        uavPulse[nUavPulse1].id = allUavRow[i].id;
        uavPulse[nUavPulse1].nPulse = allUavRow[i].nw;
        for (j = 0; j < antennaNum; j++)
        {
            uavPulse[nUavPulse1].onAnt[j] = allUavRow[i].onAnt[j];
        }
        for (j = 0; j < allUavRow[i].nw; j++)
        {
            uavPulse[nUavPulse1].pulseTime[j] = allUavRow[i].q1[j] * dt;
            uavPulse[nUavPulse1].pulseW[j] = allUavRow[i].w[j] * dt;
            for (k = 0; k < antennaNum; k++)
            {
                uavPulse[nUavPulse1].meanAmp[k][j] = 0.0;
                uavPulse[nUavPulse1].meanAmpComplex[k][j] = 0.0;
                for (ii = 0; ii < PhaseBinNum; ii++)
                {
                    uavPulse[nUavPulse1].phaseHist[k][j][ii] = 0;
                    uavPulse[nUavPulse1].weightedPhaseHist[k][j][ii] = 0;
                }
                corrSumReal = 0.0;
                corrSumImage = 0.0;
                corrSumReal1 = 0.0;
                corrSumImage1 = 0.0;
                squareSumPhase = 0.0, sumPhase = 0.0;
                squareSumPhaseWrap = 0.0, sumPhaseWrap = 0.0;
                count = 0, count1 = 0;
                // phaseCount=0;
                for (ii = allUavRow[i].q1[j]; ii < allUavRow[i].q1[j] + allUavRow[i].w[j]; ii++)
                {
                    for (jj = freqIndexL; jj < freqIndexR + 1; jj++)
                    {
                        uavPulse[nUavPulse1].meanAmp[k][j] += sumCorrAmp[ii][jj];
                        int n = 0;
                        if (corrFlag == 2)
                        {
                            n = (k % 2 == 1) ? (k - 1) : k;
                            index = (ii + nRows * (k % 2)) * NCorr * nCols * 2 + n / 2 * nCols * 2 + 2 * jj;
                        }
                        else
                            index = ii * NCorr * nCols * 2 + k * nCols * 2 + 2 * jj;

                        // corrSumReal += sumCorr[index] / 1000.0; //防止累加溢出，统一除
                        // corrSumImage += sumCorr[index + 1] / 1000.0;
                        // if (shieldFreqInd[jj] == 1)
                        // {
                        //     sumReal[count] = sumCorr[index] / 1000.0;
                        //     sumImag[count] = sumCorr[index + 1] / 1000.0;
                        //     count++;
                        // }
                        // sumReal1[count1] = sumCorr[index] / 1000.0;
                        // sumImag1[count1] = sumCorr[index + 1] / 1000.0;
                        // count1++;
                        // squareSumPhase+=sumCorrPhase[k][ii][jj]*sumCorrPhase[k][ii][jj];
                        // sumPhase+=sumCorrPhase[k][ii][jj];
                        // phaseWrap=fmod(sumCorrPhase[k][ii][jj]+3.0*PI,2.0*PI);
                        // squareSumPhaseWrap+=phaseWrap*phaseWrap;
                        // sumPhaseWrap+=phaseWrap;
                        // phaseCount++;
                        index = fmod(sumCorrPhase[ii][jj] - initPhase[k] + 9 * PI, 2 * PI) / (2 * PI) * PhaseBinNum;
                        uavPulse[nUavPulse1].phaseHist[k][j][index]++;
                        uavPulse[nUavPulse1].weightedPhaseHist[k][j][index] += sumCorrAmp[ii][jj];

                        // uavPulse[nUavPulse].meanPhase[k][j]+=sumCorrPhase[k][ii][jj];
                    }
                }
                //                 printf("-------- count = %d\n",  count);
                // if (count1 < 1)
                // {
                //     corrSumReal1 = sumReal[0];
                //     corrSumImage1 = sumImag[0];
                // }
                // else
                // {
                //     if (count > 1)
                //         robust_stat(sumReal, sumImag, count, &corrSumReal1, &corrSumImage1);
                //     else
                //         robust_stat(sumReal1, sumImag1, count1, &corrSumReal1, &corrSumImage1);
                // }
                // 		printf("---corrSumReal = %f, corrSumImage = %f\n",corrSumReal1,corrSumImage1);

                // phaseVarBlock=(squareSumPhase+phaseCount*(sumPhase/phaseCount)*(sumPhase/phaseCount)-2*(sumPhase/phaseCount)*sumPhase)/phaseCount;
                // phaseVarWrapBlock=(squareSumPhaseWrap+phaseCount*(sumPhaseWrap/phaseCount)*(sumPhaseWrap/phaseCount)-2*(sumPhaseWrap/phaseCount)*sumPhaseWrap)/phaseCount;
                // if (phaseVarBlock<phaseVarWrapBlock)
                //	uavPulse[nUavPulse].meanPhaseVar[k][j]=phaseVarBlock;
                // else
                //	uavPulse[nUavPulse].meanPhaseVar[k][j]=phaseVarWrapBlock;
                uavPulse[nUavPulse1].meanPhase[k][j] = 0;
                // archiFlag = 1;
                // if (detectedParam.doaTyp == 5 && k % 2 == 0)
                //     archiFlag = -1;
                // uavPulse[nUavPulse1].meanPhase[k][j] = fmod((archiFlag * atan2(corrSumImage1, corrSumReal1) - initPhase[k] + 7 * PI), 2 * PI) - PI; //平均相位
                // printf("uavPulse[nUavPulse].meanPhase[%d][%d] = %f\n",k,j,uavPulse[nUavPulse].meanPhase[k][j]);
                //  uavPulse[nUavPulse].meanPhase[k][j]/=phaseCount;

                phaseCount = 0;
                float x;
                uavPulse[nUavPulse1].phaseVar[k][j] = 0.0;
                for (ii = allUavRow[i].q1[j]; ii < allUavRow[i].q1[j] + allUavRow[i].w[j]; ii++)
                {
                    for (jj = freqIndexL; jj < freqIndexR + 1; jj++)
                    {
                        x = abs(sumCorrPhase[ii][jj] - uavPulse[nUavPulse1].meanPhase[k][j]);
                        if (x > PI)
                            x = 2 * PI - x;
                        uavPulse[nUavPulse1].phaseVar[k][j] += x * x;
                        phaseCount++;
                    }
                }
                uavPulse[nUavPulse1].phaseVar[k][j] /= phaseCount;
                uavPulse[nUavPulse1].meanAmpComplex[k][j] = 5 * log10(corrSumImage1 * corrSumImage1 + corrSumReal1 * corrSumReal1);
                uavPulse[nUavPulse1].meanAmp[k][j] /= (freqIndexR + 1 - freqIndexL) * allUavRow[i].w[j]; //平均幅度
            }
        }
        //计算检测置信度
        int edgePointNum = 0, lowerPointNum = 0;
        for (j = 0; j < allUavRow[i].nw; j++)
        {
            for (ii = allUavRow[i].q1[j] - 1; ii < allUavRow[i].q1[j] + allUavRow[i].w[j] + 1; ii++)
            {
                edgePointNum += 2;
                if (uavPulse[nUavPulse1].meanAmp[uavCh][j] > sumCorrAmp[ii][freqIndexL - 1])
                {
                    lowerPointNum++;
                }
                if (uavPulse[nUavPulse1].meanAmp[uavCh][j] > sumCorrAmp[ii][freqIndexR + 1])
                {
                    lowerPointNum++;
                }
            }
            for (ii = freqIndexL; ii < freqIndexR + 1; ii++)
            {
                edgePointNum += 2;
                if (uavPulse[nUavPulse1].meanAmp[uavCh][j] > sumCorrAmp[allUavRow[i].q1[j] - 1][ii])
                {
                    lowerPointNum++;
                }
                if (uavPulse[nUavPulse1].meanAmp[uavCh][j] > sumCorrAmp[allUavRow[i].q1[j] + allUavRow[i].w[j] + 1][ii])
                {
                    lowerPointNum++;
                }
            }
        }
        uavPulse[nUavPulse1].possibility = lowerPointNum * 50 / edgePointNum;

        if (UAVtypes[uavIndex].nPulseT > 0 && UAVtypes[uavIndex].pulseT[0] > 0)
        {
            float sumPulseT = 0.0;
            for (j = 0; j < UAVtypes[uavIndex].nPulseT; j++)
                sumPulseT += UAVtypes[uavIndex].pulseT[j];
            uavPulse[nUavPulse1].possibility += 50 * uavPulse[nUavPulse1].nPulse / (100 * UAVtypes[uavIndex].nPulseT / sumPulseT);
        }
        else
        {
            if (allUavRow[i].nCol / (UAVtypes[uavIndex].pulseBW[0] / dF) < 1.0)
                uavPulse[nUavPulse1].possibility += 30 * allUavRow[i].nCol / (UAVtypes[uavIndex].pulseBW[UAVtypes[uavIndex].nPulseBW - 1] / dF);
            else
                uavPulse[nUavPulse1].possibility += 30;
        }

        nUavPulse1++;

        if (nUavPulse1 >= MaxUAV)
            break;
    }
    *nUavPulse = nUavPulse1;
    // free(sumReal);
    // free(sumImag);
    // free(sumReal1);
    // free(sumImag1);
    // free(shieldFreqInd);
}

/*
   图传脉冲串，计算角度、距离等参数
 */
void calcUavPulse(struct pulseGroup *uavPulse, int nUavPulse, float *cenFreqOnAllCh, float *gainOnAllCh, float dt, float dF, struct UAVLib *UAVtypes, int nUAV, struct detectParams detectedParam)
{
    int corrFlag = (detectedParam.doaTyp == 5) ? 2 : 1;
    float onePulseMeanAmp[NCorr * corrFlag];        //临时存储比幅需要的平均幅度
    float onePulseMeanAmpUseDist[NCorr * corrFlag]; //临时存储比幅需要的平均幅度
    float onePulseMeanPhase[NCorr * corrFlag];      //存储一个脉冲的平均相位
    int onAnt[NCorr * corrFlag];
    float angle[NCorr * corrFlag]; //每组相关相位测角
    float phaseVar[NCorr * corrFlag];
    float allPulseAngle[MaxPulseInGroup]; //无人机脉冲串中所有脉冲的角度
    float range = 0.0;
    int antFace = 0;
    float dArray[2] = {detectedParam.d2450, detectedParam.d5800};
    float d[NCorr * corrFlag], initPhase[NCorr * corrFlag];

    int antennaNum = detectedParam.antennaNum;

    for (int i = 0; i < nUavPulse; i++)
    {
        // d = selAntParam(initPhase, uavPulse[i].freq[0], detectedParam);
        //  	d = selAntParamNew(initPhase, uavPulse[i].freq[0], detectedParam);
        selAntParamPro(initPhase, uavPulse[i].freq[0], detectedParam, d);

        //计算脉冲平均幅度,时间，脉宽，方位角和距离
        for (int j = 0; j < uavPulse[i].nPulse; j++)
        {
            for (int k = 0; k < antennaNum; k++)
            {
                // onePulseMeanAmp[k] = uavPulse[i].meanAmp[k][j] + detectedParam.offsetAmp[k];
                if ((detectedParam.doaTyp == 5 || detectedParam.doaTyp == 7) && uavPulse[i].freq[0] > 5000)
                    onePulseMeanAmp[k] = uavPulse[i].meanAmpComplex[k][j];
                else
                    onePulseMeanAmp[k] = uavPulse[i].meanAmp[k][j];
                onePulseMeanAmpUseDist[k] = uavPulse[i].meanAmp[k][j];
                onePulseMeanPhase[k] = uavPulse[i].meanPhase[k][j];
                // printf("onePulseMeanAmp[%d] = %f\n", k, onePulseMeanAmp[k]);
                if (detectedParam.ampSelAntType == 3 || detectedParam.ampSelAntType == 4)
                    uavPulse[i].onAnt[k] = 1;
                onAnt[k] = uavPulse[i].onAnt[k];
                phaseVar[k] = uavPulse[i].phaseVar[k][j];
            }

            // uavPulse[i].angle[j] = calcAngle(onePulseMeanPhase, onePulseMeanAmp, uavPulse[i].freq[0], d, onAnt, phaseVar, antFace, detectedParam);
            // uavPulse[i].angle[j] = calcAnglePro(onePulseMeanPhase, onePulseMeanAmp, uavPulse[i].freq[0], d, onAnt, phaseVar, antFace, detectedParam);
            uavPulse[i].angle[j] = 0;
            uavPulse[i].distance[j] = amp2dist(onePulseMeanAmpUseDist, uavPulse[i].onAnt, detectedParam.miu, gainOnAllCh);
            uavPulse[i].antFace[j] = antFace;
            range += uavPulse[i].distance[j];
            allPulseAngle[j] = uavPulse[i].angle[j];
        }
        uavPulse[i].azimuth = angleSelMean(allPulseAngle, uavPulse[i].nPulse); //最终输出的角度
        uavPulse[i].range = range / uavPulse[i].nPulse;
    }
    for (int index1 = 0; index1 < nUavPulse; index1++)
    {
        printf("UavPulse Amp=%f\n", uavPulse[index1].meanAmp[0][0]);
//        printf("wyc phase=%f,%f,%f,%f,%f\n", uavPulse[index1].meanPhase[0][0], uavPulse[index1].meanPhase[1][0], uavPulse[index1].meanPhase[2][0], uavPulse[index1].meanPhase[3][0], uavPulse[index1].meanPhase[4][0]);
    }
}

/*
   无人机图传信号检测和识别主函数
 */
void vidDetect(
    struct pulseGroup *uavPulse, int *nUavPulse, float *sumCorr, float (*sumCorrAmp)[128], float (*sumCorrPhase)[128], int nRows, int nCols, float *cenFreqOnAllCh, float *gainOnAllCh, float dt, float dF,
    struct UAVLib *UAVtypes, int nUAV, struct detectParams detectedParam)
{
    int nUavPulse1 = *nUavPulse;
    int antennaNum = 1;
    int corrFlag = (detectedParam.doaTyp == 5) ? 2 : 1;
    //定义图传脉冲列检测用变量
    struct uavRow allUavRow[MaxUAV];// = (struct uavRow *)malloc(MaxUAV * antennaNum * sizeof(struct uavRow)); //检测到的所有无人机脉冲列
    int nUavRow = 0;                                                                                 //记录检测到的所有无人机脉冲列数目
    //     //幅度高斯滤波
    //     float ***filtedAmp = createGrid(antennaNum, nRows, nCols);
    //     float ***filtedAmpIntegral = createGrid(antennaNum, nRows, nCols);
    //     float ***sumCorrReal = createGrid(antennaNum, nRows, nCols);
    //     float ***sumCorrImag = createGrid(antennaNum, nRows, nCols);

    float filtedAmp[NROWS][NFFT / NSumCol];// = createMatrix(nRows, nCols);
    //
    //     clock_t t1, t2;
    //     double  useTime;
    // 	float **filtedAmpIntegral = (float **)malloc(sizeof(float *) * nRows);
    // 	float **filtedAmp = (float **)malloc(sizeof(float *) * nRows);
    // 	for(int i = 0 ; i < nRows; i++)
    // 	{
    // 		float *filtedAmpIntegral[i] = (float *)malloc(sizeof(float) * nCols);
    // 		float *filtedAmp[i] = (float *)malloc(sizeof(float) * nCols);
    // 	}
    //
    // // 	createGrid(antennaNum, nRows, nCols);
    // 	int nSpan[2];//列索引范围
    //     nSpan[0]=detectedParam.vidIndexSpan[0];
    // 	nSpan[1]=detectedParam.vidIndexSpan[NCorr];
    // 	filter(sumCorrAmp, nRows, nCols, filtedAmp, detectedParam.mask, detectedParam.maskH, detectedParam.maskW, nSpan, 1);

    //     //单线程
    int nSpan[2]; //列索引范围
    // t1=clock();
    for (int i = 0; i < antennaNum; i++)
    {
        nSpan[0] = detectedParam.vidIndexSpan[i];
        nSpan[1] = detectedParam.vidIndexSpan[i + NCorr];
		for(int j = 0; j < nRows; j++)
		{
			for(int k = 0; k < nCols; k++)
			{
				filtedAmp[j][k] = sumCorrAmp[j][k];
			}
                }
//         filter(sumCorrAmp, nRows, nCols, filtedAmp, detectedParam.mask, detectedParam.maskH, detectedParam.maskW, nSpan, 1);
    }
    //     //t2=clock();
    //     //useTime=(double)(t2-t1)/CLOCKS_PER_SEC;
    //     //printf("vidfilt use time %f ms\n",useTime*1000);

    //     多线程
    //     t1=clock();
    //     pthread_t   tid[NCorr * corrFlag*2] = { 0 };
    //     filterParam filtParam[NCorr * corrFlag];
    //     int         err1,err2;

    //     for (int i = 0; i < antennaNum; i++)
    //     {
    //         int ind = 0;
    //         if(detectedParam.doaTyp == 5 && i >= 4)
    //             ind = (i+1)%5;
    //         else
    //             ind = i;

    //         int index,n = 0,frameHead;
    //         for (int j = 0; j < nRows; j++)
    //         {
    //             for(int k = 0; k < 6; k++)
    // 	    {
    // 		sumCorrReal[i][j][k] = 0;
    //                 sumCorrImag[i][j][k] = 0;
    // 	    }
    // 	    for (int k = 6; k < nCols - 6; k++)
    //             {
    //                 if(corrFlag == 2)
    //                 {
    //                     n = (i % 2 == 1)?(i - 1):i;
    //                     index = (j + nRows * (i % 2)) * NCorr * nCols * 2 + n / 2 * nCols * 2 + 2 * k;
    //                 }
    //                 else
    //                     index = j * NCorr * nCols * 2 + i * nCols * 2 + 2 * k;
    //                 sumCorrReal[i][j][k] = sumCorr[index];
    //                 sumCorrImag[i][j][k] = sumCorr[index + 1];
    //             }
    //             for(int k = nCols - 6; k < nCols; k++)
    // 	    {
    // 		sumCorrReal[i][j][k] = 0;
    //                 sumCorrImag[i][j][k] = 0;
    // 	    }
    //         }

    //         filtParam[i].m = nRows;
    //         filtParam[i].n = nCols;
    //         filtParam[i].mask = detectedParam.mask;
    //         filtParam[i].h = detectedParam.maskH;
    //         filtParam[i].w = detectedParam.maskW;
    //         filtParam[i].edgeFlag = 1;
    //         filtParam[i].n1 = detectedParam.vidIndexSpan[ind];
    //         filtParam[i].n2 = detectedParam.vidIndexSpan[ind + NCorr];

    //         switch (detectedParam.filterMethod)
    //         {
    //         case 0:
    //             filtParam[i].matIn = *(sumCorrAmp + i);
    //             filtParam[i].matOut = *(filtedAmp + i);
    //             err1 = pthread_create(&tid[i], NULL, filter_thread, (void *)&filtParam[i]);
    // // 	    err2 = 0;
    //             break;
    //         case 1:
    //             filtParam[i].matIn = *(sumCorrReal + i);
    //             filtParam[i].matIn2 = *(sumCorrImag + i);
    //             filtParam[i].matOut = *(filtedAmp + i);
    // 	        filtParam[i].matOut2 = *(filtedAmpIntegral + i);
    //             //ampAndIntegral_thread(&filtParam[i]);
    //     	    err1 = pthread_create(&tid[i], NULL, ampAndIntegral_thread, (void *)&filtParam[i]);
    //             break;
    //         case 2:
    // 	    filtParam[i].matIn = *(sumCorrReal + i);
    // 	    filtParam[i].matIn2 = *(sumCorrImag + i);
    // 	    filtParam[i].matOut = *(filtedAmp + i);
    // 	    //err1 = pthread_create(&tid[2*i], NULL, integral_thread, (void *)&filtParam[i]);
    // // 	    if (detectedParam.d800 < 1)
    // // 		integral_thread(&filtParam[i]);
    // // 	    else if(detectedParam.d800 >= 1)
    // //		integral_thread_cv(&filtParam[i]);

    // //             filtParam[i].matOut2 = *(filtedImag + i);
    // // 	    if (detectedParam.d800 < 1)
    // // 		integral_thread(&filtParam[i]);
    // // 	    else if(detectedParam.d800 >= 1)
    // 	    //integral_thread_cv2(&filtParam[i]);
    // 	    err1 = pthread_create(&tid[i], NULL, integral_thread_cv2, (void *)&filtParam[i]);
    //             break;
    //         default:
    //             break;
    //         }

    //         if (err1 != 0)//err1 != 0 || err2 != 0
    //         {
    // 	    for (int j = 0; j < i; j++)
    // 		pthread_join(tid[j], NULL);
    //             printf("Create thread failed!\n");
    //             free(allUavRow);
    //             allUavRow = NULL;
    //             freeGrid(filtedAmp, antennaNum, nRows, nCols);
    // 	        freeGrid(filtedAmpIntegral, antennaNum, nRows, nCols);
    //             freeGrid(sumCorrReal, antennaNum, nRows, nCols);
    //             freeGrid(sumCorrImag, antennaNum, nRows, nCols);
    // // 	    if(detectedParam.filterMethod == 1)
    // // 	    {
    // // 		freeGrid(filtedReal, antennaNum, nRows, nCols);
    // // 		freeGrid(filtedImag, antennaNum, nRows, nCols);
    // //
    // // 	    }
    //             nUavPulse = 0;
    //             return;
    //         }
    //     }

    // 	for (int i = 0; i < antennaNum; i++)
    // 	    pthread_join(tid[i], NULL);

    // 	if(detectedParam.filterMethod == 0)
    // 	{
    // 	      for(int i = 0; i < detectedParam.maskH; i++)
    // 	      {
    // 		  for(int j = 1; j < detectedParam.maskW; j++)
    // 		  {
    // 		      mask[i * w] += mask[j+ i * w];
    // 	  // 	    printf("i = %d,j = %d\n",i,j);
    // 		  }
    // 		  mask[i] = mask[i * w];
    // 	      }
    // 	}
    //
    //     if (detectedParam.filterMethod == 1 || detectedParam.filterMethod == 2)
    //     {
    // 	for (int i = 0; i < antennaNum; i++)
    // 	{
    // 	    for (int k = 0; k < nCols; k++)//5 * log10
    // 	    {
    // 		for (int j = 0; j < nRows; j++)
    // 		{
    // 		    filtedAmp[i][j][k] = 5*log10(pow(filtedReal[i][j][k],2) + pow(filtedImag[i][j][k], 2) +1);
    // 		}
    // 	    }
    // 	}
    //     }
    //
    //
    //
    //     for(int i = 0; i < antennaNum; i++)
    //     {
    // 	int ind = 0;
    //         if(detectedParam.doaTyp == 5 && i >= 4)
    //             ind = (i+1)%5;
    //         else
    //             ind = i;
    //
    // 	filtParam[i].m = nRows;
    //         filtParam[i].n = nCols;
    //         filtParam[i].mask = detectedParam.mask;
    //         filtParam[i].h = detectedParam.maskH;
    //         filtParam[i].w = detectedParam.maskW;
    //         filtParam[i].edgeFlag = 1;
    //         filtParam[i].n1 = detectedParam.vidIndexSpan[ind];
    //         filtParam[i].n2 = detectedParam.vidIndexSpan[ind + NCorr];
    //
    // 	filtParam[i].matIn = *(filtedAmp1 + i);
    //             filtParam[i].matOut = *(filtedAmp + i);
    //             err1 = pthread_create(&tid[i], NULL, filter_thread, (void *)&filtParam[i]);
    //     }
    //     for (int i = 0; i < antennaNum; i++)
    //     {
    // 	pthread_join(tid[i], NULL);
    //     }

    //     Mat dst = Mat::zeros(nRows, nCols, CV_32FC1);
    //     double maxA,minA;
    //     double *maxAmp = &maxA, *minAmp = &minA;
    //     short  g;
    //
    //     Mat spec(nRows, nCols, CV_8UC3, Scalar(0, 0, 0));  //定义总体图形
    //
    //     for (int j = 0; j < nRows; j++)
    //     {
    // 	for (int k = 0; k < nCols; k++)
    // 	{
    // 	    dst.at< float >(j, k) = filtedAmp[0][j][k];
    // 	}
    //     }
    //
    //     minMaxIdx(dst, minAmp, maxAmp);
    //     if (minA <= 20) minA = 20;
    //     printf("----%f,%f\n",maxA, minA);
    //
    //     for (int j = 0; j < nRows; j++)
    //     {
    // 	for (int k = 0; k < nCols; k++)
    // 	{
    // 	    g = (short)(dst.at< float >(j, k) - minA) / (maxA - minA) * 63;
    // 	    if(g < 0)	g==0;
    // 	    spec.at< Vec3b >(j, k) = Vec3b(jet_map1[g][2], jet_map1[g][1], jet_map1[g][0]);
    // 	}
    //     }
    //     imshow("outputIntegral",spec);

    //     imshow("outputImg",dst);
    //     t2=clock();
    //     useTime=(double)(t2-t1)/CLOCKS_PER_SEC;
    //     printf("vidfilt use time %f ms\n",useTime*1000);

    // detectedParam.antennaNum = 1;
    //脉冲列检测、识别和合并
    uavRowDetect(allUavRow, &nUavRow, sumCorrAmp, filtedAmp, filtedAmp, nRows, nCols, cenFreqOnAllCh, dt, dF, detectedParam, UAVtypes, nUAV); //时间方向处理，检测无人机图传

    //生成脉冲
    genUavPulse(uavPulse, &nUavPulse1, allUavRow, nUavRow, sumCorr, sumCorrAmp, sumCorrPhase, nRows, nCols, cenFreqOnAllCh, dt, dF, UAVtypes, detectedParam);
    //目标脉冲参数计算
    calcUavPulse(uavPulse, nUavPulse1, cenFreqOnAllCh, gainOnAllCh, dt, dF, UAVtypes, nUAV, detectedParam); //计算脉冲串内的角度、距离等参数

//     free(allUavRow);
//     allUavRow = NULL;
    //     freeGrid(sumCorrReal, antennaNum, nRows, nCols);
    //     freeGrid(sumCorrImag, antennaNum, nRows, nCols);
    //     freeGrid(filtedAmp, antennaNum, nRows, nCols);
    //     freeGrid(filtedAmpIntegral, antennaNum, nRows, nCols);
//     freeMatrix(filtedAmp, nRows);

    // if(detectedParam.filterMethod == 1)
    // {
    //  freeGrid(filtedReal, antennaNum, nRows, nCols);
    //  freeGrid(filtedImag, antennaNum, nRows, nCols);

    // }
    *nUavPulse = nUavPulse1;
//    printf("uavPulseN = %d\n",*nUavPulse);
}
