#include "uavDetect.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include "sharedFuncs.h"
// #include "opencv2/core.hpp"
// #include <opencv2/highgui.hpp>
// #include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
// #include <opencv2/highgui/highgui_c.h>

// 深度优先的递归搜索连通区域
bool DFS(float **CtrFiltedAmp, int ix, int iy, int **matMark, int label, int nRows, int nCols, int count, float ctrSNR)
{
    int i, ix1, iy1;
    int conn = 4;
    int direc[4][2] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}}; // 四连通

    // 幅度值低于25dB或者在频谱中心线左右1根谱线之内均不参与搜索
    if (matMark[ix][iy] > 0 || CtrFiltedAmp[ix][iy] < ctrSNR || count > MaxRecursionCount || (abs(iy - nCols / 2) < 2)) // && CtrFiltedAmp[ix][iy]<25))
    {
        return false;
    }
    else
    {
        matMark[ix][iy] = label;
        for (i = 0; i < conn; i++)
        {
            ix1 = ix + direc[i][0];
            iy1 = iy + direc[i][1];
            if (ix1 >= 0 && ix1 < nRows && iy1 >= 0 && iy1 < nCols)
            {
                DFS(CtrFiltedAmp, ix1, iy1, matMark, label, nRows, nCols, count, ctrSNR);
                count++;
            }
        }
    }
    return true;
}

/*
连通区域标记，使用深度递归算法
*/
int markConnected(float **CtrFiltedAmp, int **matMark, int nRows, int nCols, float ctrSNR)
{
    int i, j;
    int count = 0, label = 1;

    for (i = 0; i < nRows; i++)
    {
        *(matMark + i) = (int *)malloc(nCols * sizeof(int));
        memset(*(matMark + i), 0, nCols * sizeof(int)); // 标记初始化为0
    }
    for (i = 0; i < nRows; i++)
    {
        for (j = 10; j < nCols - 10; j++) // 频谱左右边界各10个频点不检测，因为是滤波器带宽范围之外
        {
            count = 0;
            if (DFS(CtrFiltedAmp, i, j, matMark, label, nRows, nCols, count, ctrSNR))
            {
                label++;
            }
        }
    }
    label--; // 减去最后一个多加的标号
    return label;
}

/*
连通区域标记，使用opencv库函数
*/
int markConnectedOpencv(float **CtrFiltedAmp, int **matMark, int nRows, int nCols, int n1, int n2, float ctrSNR)
{
    int i, j, label = 0;
    int connectivity = 4;
    cv::Mat markLabel;
    // 二值化
    cv::Mat filtedAmpThresh(nRows, nCols, CV_32FC1); // 从二维数组转化为mat
    for (i = 0; i < nRows; i++)
    {
        for (j = 0; j < n1; j++)
        {
            filtedAmpThresh.at<float>(i, j) = 0;
        }
        for (j = n1; j < n2; j++)
        {
            filtedAmpThresh.at<float>(i, j) = CtrFiltedAmp[i][j];
        }
        for (j = n2; j < nCols; j++)
        {
            filtedAmpThresh.at<float>(i, j) = 0;
        }
        filtedAmpThresh.at<float>(i, 64) = 50;
    }

    cv::threshold(filtedAmpThresh, filtedAmpThresh, ctrSNR, 1, cv::THRESH_BINARY);
    // 连通区域标记
    filtedAmpThresh.convertTo(filtedAmpThresh, CV_8UC1);
    label = connectedComponents(filtedAmpThresh, filtedAmpThresh, connectivity, CV_16U);

    // 将标记的mat转化为数组
    for (i = 0; i < nRows; i++)
    {
        for (j = 0; j < nCols; j++)
        {
            matMark[i][j] = filtedAmpThresh.at<ushort>(i, j);
        }
    }

    return label - 1;
}

// 连通标记后的矩阵，生成脉冲
int mark2Pulse(
    struct ctrPulse *pulse, int **matMark, float (*sumCorrAmp)[128], int nRows, int nCols, int n1, int n2, int label, float dt, float dF, int antIndex, float cenFreqOnAllCh, struct detectParams detectedParam)
{
    int i, j, k;

    int antennaNum = detectedParam.antennaNum;
    float minCtrPulseW = detectedParam.minCtrPulseW;
    float maxCtrPulseW = detectedParam.maxCtrPulseW;

    int nPulseParam;
    int marker;
    struct pulseParam allPulseParam[MaxPulseParam];
    // 初始化
    for (i = 0; i < MaxPulseParam; i++)
    {
        allPulseParam[i].antIndex = antIndex;
        allPulseParam[i].pulseTime = -1;
        allPulseParam[i].pulseBW = 0.0;
        allPulseParam[i].freq = 0.0;
        for (j = 0; j < antennaNum; j++)
        {
            allPulseParam[i].meanAmp[j] = 0.0;
            // 	    allPulseParam[i].maxAmp[j] = 0.0;
        }
    }
    int nPulse = 0; // 返回生成脉冲的个数

    // 参数累加
    if (label > MaxPulseParam)
        nPulseParam = MaxPulseParam;
    else
        nPulseParam = label;

    for (i = 0; i < nRows; i++)
    {
        for (j = n1; j < n2; j++)
        {
            marker = matMark[i][j] - 1;
            if (marker >= 0 && marker < nPulseParam)
            {
                if (allPulseParam[marker].pulseTime < 0)
                    allPulseParam[marker].pulseTime = i;
                allPulseParam[marker].pulseW = i;
                allPulseParam[marker].pulseBW++;
                allPulseParam[marker].freq += j * sumCorrAmp[i][j];
                for (k = 0; k < antennaNum; k++)
                {
                    // 		    if (allPulseParam[marker].maxAmp[k] < sumCorrAmp[k][i][j])
                    // 			allPulseParam[marker].maxAmp[k] = sumCorrAmp[k][i][j];

                    allPulseParam[marker].meanAmp[k] += sumCorrAmp[i][j]; // 幅度累加
                }
            }
        }
    }

    // 计算脉冲参数
    for (i = 0; i < nPulseParam; i++)
    {
        // 计算频率、带宽、脉宽等
        allPulseParam[i].freq = cenFreqOnAllCh - 0.5 * nCols * dF + 0.5 * dF + dF * allPulseParam[i].freq / allPulseParam[i].meanAmp[antIndex];
        for (j = 0; j < antennaNum; j++)
            //	    allPulseParam[i].meanAmp[j] = allPulseParam[i].maxAmp[j];
            allPulseParam[i].meanAmp[j] = allPulseParam[i].meanAmp[j] / allPulseParam[i].pulseBW;
        allPulseParam[i].pulseBW = allPulseParam[i].pulseBW / (allPulseParam[i].pulseW - allPulseParam[i].pulseTime + 1) * dF;
        allPulseParam[i].pulseW = (allPulseParam[i].pulseW - allPulseParam[i].pulseTime + 1) * dt;
        allPulseParam[i].pulseTime = allPulseParam[i].pulseTime * dt;
    }

    // 挑选合适的脉冲
    for (i = 0; i < nPulseParam; i++)
    {
        if (nPulse >= MaxCtrPulse)
            break;
        if (allPulseParam[i].pulseW > minCtrPulseW && allPulseParam[i].pulseW < maxCtrPulseW) // 脉宽和带宽必须在一定范围内
        {
            pulse[nPulse].antIndex = allPulseParam[i].antIndex;
            pulse[nPulse].freq = allPulseParam[i].freq;
            pulse[nPulse].pulseBW = allPulseParam[i].pulseBW;
            pulse[nPulse].pulseTime = allPulseParam[i].pulseTime;
            pulse[nPulse].pulseW = allPulseParam[i].pulseW;
            for (j = 0; j < antennaNum; j++)
                pulse[nPulse].meanAmp[j] = allPulseParam[i].meanAmp[j];
            nPulse++;
        }
    }

    return nPulse; // 返回脉冲的个数
}

// 将阈值分割后的每个连通区域生成脉冲，返回脉冲个数
int ctrPulseDetect(struct ctrPulse *pulse, float **CtrFiltedAmp, float (*sumCorrAmp)[128], int nRows, int nCols, float dt, float dF, int antIndex, float *cenFreqOnAllCh, struct detectParams detectedParam)
{
    int i, j, k;

    float ctrSNR = detectedParam.ctrSNR;
    int label;
    int nPulse = 0; // 返回生成脉冲的个数

    int ind = 0;
    if (detectedParam.doaTyp == 5 && antIndex >= 4)
        ind = (antIndex + 1) % 5;
    else
        ind = antIndex;
    int n1 = detectedParam.ctrIndexSpan[ind], n2 = detectedParam.ctrIndexSpan[ind + NCorr];
    // 创建一个标记矩阵
    int **matMark = (int **)malloc(nRows * sizeof(int *));
    for (i = 0; i < nRows; i++)
    {
        matMark[i] = (int *)malloc(nCols * sizeof(int));
    }
    int corrIndex; // 相关的数据索引

    // 连通区域标记
    label = markConnectedOpencv(CtrFiltedAmp, matMark, nRows, nCols, n1, n2, ctrSNR);

    // 由连通区域生成脉冲
    nPulse = mark2Pulse(pulse, matMark, sumCorrAmp, nRows, nCols, n1, n2, label, dt, dF, antIndex, cenFreqOnAllCh[0], detectedParam);

    // 清理临时数组变量
    for (i = 0; i < nRows; i++)
    {
        free(*(matMark + i));
    }
    free(matMark);
    matMark = NULL;

    return nPulse; // 返回脉冲的个数
}

/*
一般大带宽的脉冲只有两个端点被检出，合并两个端点成为一个完整的脉冲
*/
void mergeLargeBWpulse(struct ctrPulse *pulse, int &nCtrPulse, float dt, int antennaNum, int corrFlag)
{
    int i, j, k, p;

    for (i = 0; i < nCtrPulse; i++)
    {
        for (j = i + 1; j < nCtrPulse; j++)
        {
            // 若两个脉冲时间和脉宽都很接近，频率相差不超过12MHz，则将其合并成为一个大带宽的脉冲
            if (fabs(pulse[i].pulseTime - pulse[j].pulseTime) < 2 * dt / corrFlag && fabs(pulse[i].pulseW - pulse[j].pulseW) < 2 * dt / corrFlag && fabs(pulse[i].freq - pulse[j].freq) < 20)
            {
                // 合并脉冲参数
                pulse[i].pulseTime = (pulse[i].pulseTime + pulse[j].pulseTime) / 2.0;
                pulse[i].freq = (pulse[i].freq + pulse[j].freq) / 2.0;
                pulse[i].pulseBW = pulse[i].pulseBW + pulse[j].pulseBW + fabs(pulse[i].freq - pulse[j].freq);
                pulse[i].pulseW = (pulse[i].pulseW + pulse[j].pulseW) / 2.0;
                for (k = 0; k < antennaNum; k++)
                    pulse[i].meanAmp[k] = (pulse[i].meanAmp[k] + pulse[j].meanAmp[k]) / 2.0;
                // 删除被合并的脉冲
                for (k = j; k < nCtrPulse - 1; k++)
                {
                    pulse[k].antIndex = pulse[k + 1].antIndex;
                    pulse[k].freq = pulse[k + 1].freq;
                    pulse[k].pulseBW = pulse[k + 1].pulseBW;
                    pulse[k].pulseTime = pulse[k + 1].pulseTime;
                    pulse[k].pulseW = pulse[k + 1].pulseW;
                    for (p = 0; p < antennaNum; p++)
                        pulse[k].meanAmp[p] = pulse[k + 1].meanAmp[p];
                }
                j--;
                nCtrPulse--;
            }
        }
    }
}

/*
检查脉冲频率是否符合要求，分为频点固定和频点在某个范围两种情况。若符合要求checkedPulse[i]标记为1,同时记下频点
*/
int checkPulseFreq(struct ctrPulse *pulse, int nCtrPulse, int libIndex, int *freqIndex, int *checkedPulse, struct UAVLib *UAVtypes)
{
    int j, k;
    int nFreq, nChecked = 0;
    float freqErr = UAVtypes[libIndex].freqErr;
    float freqP, pulseBW0 = UAVtypes[libIndex].pulseBW[0];

    if (UAVtypes[libIndex].isFixedFreq) // 若遥控频点固定，挑出所有符合频点要求的脉冲
    {
        nFreq = UAVtypes[libIndex].nfreqp;
        for (j = 0; j < nCtrPulse; j++)
        {
            for (k = 0; k < nFreq; k++)
            {
                if (nChecked >= MaxCtrPulse)
                    return nChecked;
                freqP = UAVtypes[libIndex].freqPoints[k];
                if (pulseBW0 < 5)
                {
                    // 脉冲满足频点要求
                    if (fabs(freqP - pulse[j].freq) < freqErr)
                    {
                        checkedPulse[nChecked] = j; // 加入脉冲序号
                        freqIndex[nChecked] = k;    // 记录脉冲频率在跳频表内的编号
                        nChecked++;
                        break;
                    }
                }
                else
                {
                    // 对大带宽的无人机遥控，脉冲满足左右边界或中心的频率要求
                    if (fabs(freqP + pulseBW0 / 2 - pulse[j].freq) < freqErr || fabs(freqP - pulseBW0 / 2 - pulse[j].freq) < freqErr || fabs(freqP - pulse[j].freq) < freqErr)
                    {
                        checkedPulse[nChecked] = j; // 加入脉冲序号
                        freqIndex[nChecked] = k;    // 记录脉冲频率在跳频表内的编号
                        nChecked++;
                        break;
                    }
                }
            }
        }
    }
    else // 若频点不固定只是知道范围，则先挑选出范围内的频点
    {
        for (j = 0; j < nCtrPulse; j++)
        {
            // 挑出频点在频率范围内的脉冲
            if (pulse[j].freq > UAVtypes[libIndex].freqPoints[0] && pulse[j].freq < UAVtypes[libIndex].freqPoints[1])
            {
                checkedPulse[nChecked] = j; // 加入脉冲序号
                nChecked++;
            }
        }
    }
    return nChecked;
}

/*
检查脉冲的周期性和脉宽，只针对跳频在频率和时间方向都有周期的遥控脉冲
int checkPulseCycleAndWidth(int *ctrMatch, int **match, struct ctrPulse *pulse, int libIndex, int *checkedPulse, int nChecked, int *freqIndex, int *maxnT, struct UAVLib *UAVtypes)
{
    int j, k, p, q;
    float minPulseTime, maxPulseTime, sumPulseT;  //脉冲中最早和最晚的脉冲时间
    int nT;                          //脉冲时间跨度，多少个脉冲周期
    int pj, pk;                      //脉冲的临时索引号

    float **matchTimeErr = (float **)malloc(MaxCtrPulse * sizeof(float *));  //记录脉冲与倍数周期的时间差
    for (j = 0; j < MaxCtrPulse; j++)
        matchTimeErr[j] = (float *)malloc(MaxNumCtrT * sizeof(float));
    float **matchFreqErr = (float **)malloc(MaxCtrPulse * sizeof(float *));  //记录脉冲频点与跳频表频点的频率差
    for (j = 0; j < MaxCtrPulse; j++)
        matchFreqErr[j] = (float *)malloc(MaxNumCtrT * sizeof(float));

    int oneMatch[5][MaxNumCtrT];//记录倍数周期上的脉冲编号
    float oneMatchTimeErr[5][MaxNumCtrT];//记录倍数周期的时间偏差
    float oneMatchFreqErr[5][MaxNumCtrT];//

    float diffTime, TErr, freqErr;
    int   nFreq = UAVtypes[libIndex].nfreqp;
    int   hoppCount, pulseWCount[MaxPulseInGroup];
    int   nCtrMatch = 0;  //记录match中确定是遥控脉冲的总行数
    bool  isNewCtr, isMeetPulseW;
    int   sameIndex, pulseNum1, pulseNum2;

    //求取脉冲时间跨越多少个脉冲周期
    minPulseTime = pulse[checkedPulse[0]].pulseTime;
    maxPulseTime = pulse[checkedPulse[0]].pulseTime;
    for (j = 1; j < nChecked; j++)
    {
        pj = checkedPulse[j];
        if (pulse[pj].pulseTime > maxPulseTime)
            maxPulseTime = pulse[pj].pulseTime;
        if (pulse[pj].pulseTime < minPulseTime)
            minPulseTime = pulse[pj].pulseTime;
    }

    sumPulseT=0.0;
    for(j=0;j<UAVtypes[libIndex].nPulseT;j++)
    {
        sumPulseT+=UAVtypes[libIndex].pulseT[j];
    }
    *maxnT = floor(((maxPulseTime - minPulseTime) / sumPulseT+1)*UAVtypes[libIndex].nPulseT + 0.5);

    //标记跳频时间距离整数倍数周期最近，且距离跳频表中频点最近的脉冲
    for (j = 0; j < nChecked; j++)
    {
        pj = checkedPulse[j];
        match[j][0] = j;
        for (k = 1; k < MaxNumCtrT; k++)
        {
            match[j][k] = -1;
        }  //初始化
        for (k = 0; k < nChecked; k++)
        {
            if (j != k)
            {
                pk = checkedPulse[k];
                diffTime = pulse[pk].pulseTime - pulse[pj].pulseTime;
                if (diffTime < 0)
                    continue;
                nT = floor(diffTime / UAVtypes[libIndex].pulseT[0] + 0.5);
                TErr = abs(diffTime - nT * UAVtypes[libIndex].pulseT[0]);
                if (UAVtypes[libIndex].isHoppOnTime)  //若频点只在时间方向跳动，则频率误差只与该频点进行比较
                    freqErr = abs(UAVtypes[libIndex].freqPoints[freqIndex[j]] - pulse[pk].freq);
                else  //若频点是固定频率表跳动，则与跳频表进行比较
                    freqErr = abs(UAVtypes[libIndex].freqPoints[(freqIndex[j] + nT) % nFreq] - pulse[pk].freq);
                //若脉冲时间差至少大于一个脉冲周期，且与倍数周期时间差满足容差。若跳频有固定跳频表则频率偏差必须满足要求。
                if (nT > 0 && TErr < UAVtypes[libIndex].pulseTErr && (freqErr < UAVtypes[libIndex].freqErr || UAVtypes[libIndex].meetHopp < 1))
                {
                    if (match[j][nT] < 0)  //若是第一次找到整数倍的，则记下时间偏差和频率偏差
                    {
                        match[j][nT] = k;
                        matchTimeErr[j][nT] = TErr;
                        matchFreqErr[j][nT] = freqErr;
                    }
                    else
                    {
                        if (TErr < matchTimeErr[j][nT] && freqErr < matchFreqErr[j][nT])  //有更接近整数倍且更接近跳频表的，则替换
                        {
                            match[j][nT] = k;
                            matchTimeErr[j][nT] = TErr;
                            matchFreqErr[j][nT] = freqErr;
                        }
                    }
                }
            }
        }
    }
    //统计match中每行满足跳频周期和跳频频点的脉冲，必须要满足多个跳频点数
    for (j = 0; j < nChecked; j++)
    {
        //统计满足跳频表的频点数，和满足脉宽要求的频点数
        hoppCount = 0;
        for (k = 0; k < UAVtypes[libIndex].nPulseW; k++)
            pulseWCount[k] = 0;
        for (k = 0; k < (*maxnT) + 1; k++)
        {
            pk = match[j][k];
            if (match[j][k] >= 0)  //若该倍数周期存在脉冲
            {
                hoppCount++;
                for (int p = 0; p < UAVtypes[libIndex].nPulseW; p++)
                {
                    if (abs(UAVtypes[libIndex].pulseW[p] - pulse[checkedPulse[pk]].pulseW) < UAVtypes[libIndex].pulseWErr[p])
                    {
                        pulseWCount[p]++;
                        break;
                    }
                }
            }
        }
        //若满足跳频频点规律的脉冲个数大于最低限度，且至少有一个或多个脉宽符合要求，则将该脉冲串与记录在案的遥控脉冲进行去重
        isMeetPulseW = false;
        for (k = 0; k < UAVtypes[libIndex].nPulseW; k++)
        {
            if (pulseWCount[k] >= UAVtypes[libIndex].meetPulseW[k])
            {
                isMeetPulseW = true;
                break;
            }
        }

        //去重
        if (hoppCount >= UAVtypes[libIndex].meetHopp && isMeetPulseW)
        {
            isNewCtr = true;
            //与已记录的遥控脉冲去重
            for (k = 0; k < nCtrMatch; k++)
            {
                for (p = 0; p < (*maxnT) + 1; p++)
                {
                    pj = match[j][p];
                    for (q = 0; q < (*maxnT) + 1; q++)
                    {
                        pk = match[ctrMatch[k]][q];
                        if (pj >= 0 && pk >= 0)  //从两者中各挑出一个遥控脉冲，是否满足跳频周期
                        {
                            if (!UAVtypes[libIndex].isHoppOnTime)  //若只在时间方向跳频
                            {
                                if (fmod(abs(pulse[checkedPulse[pj]].pulseTime - pulse[checkedPulse[pk]].pulseTime), UAVtypes[libIndex].pulseT[0])
                                    < UAVtypes[libIndex].pulseTErr)  //若频点时间差是倍数周期，则是同一个遥控
                                {
                                    isNewCtr = false;
                                    sameIndex = k;
                                    break;
                                }
                            }
                            else  //若在时间和频率方向均跳频
                            {
                                if (abs(pulse[checkedPulse[pj]].freq - pulse[checkedPulse[pk]].freq) < UAVtypes[libIndex].freqErr
                                    && fmod(abs(pulse[checkedPulse[pj]].pulseTime - pulse[checkedPulse[pk]].pulseTime), UAVtypes[libIndex].pulseT[0])
                                           < UAVtypes[libIndex].pulseTErr)  //若频点接近且频点时间差是倍数周期，则是同一个遥控
                                {
                                    isNewCtr = false;
                                    sameIndex = k;
                                    break;
                                }
                                else if (abs(pulse[checkedPulse[pj]].pulseTime - pulse[checkedPulse[pk]].pulseTime) < UAVtypes[libIndex].pulseTErr)  //若时间很接近（可能是频谱泄漏），则是同一个遥控
                                {
                                    isNewCtr = false;
                                    sameIndex = k;
                                    break;
                                }
                            }
                        }
                    }
                    if (!isNewCtr)
                        break;
                }
                if (!isNewCtr)
                    break;
            }

            if (isNewCtr)  //若是一个新的遥控脉冲串，则追加到记录
            {
                ctrMatch[nCtrMatch] = j;
                nCtrMatch++;
                if (nCtrMatch >= MaxPulseInGroup)
                    break;
            }
            else  //若不是新的遥控脉冲串，则与跟它同类型的遥控脉冲串比较脉冲个数，脉冲个数多的留下来
            {
                pulseNum1 = 0;
                pulseNum2 = 0;
                for (k = 0; k < (*maxnT) + 1; k++)
                {
                    if (match[j][k] >= 0)
                        pulseNum1++;
                    if (match[ctrMatch[sameIndex]][k] >= 0)
                        pulseNum2++;
                }
                if (pulseNum1 > pulseNum2)  //若新的脉冲串遥控脉冲数量较多，则替换原有的脉冲串
                    ctrMatch[sameIndex] = j;
                else if (pulseNum1 == pulseNum2)  //若脉冲数量一样，则幅度强的留下
                {
                    if (pulse[checkedPulse[match[j][0]]].meanAmp[pulse[0].antIndex] > pulse[checkedPulse[match[ctrMatch[sameIndex]][0]]].meanAmp[pulse[0].antIndex])
                        ctrMatch[sameIndex] = j;
                }
            }
        }
    }

    for (j = 0; j < MaxCtrPulse; j++)
    {
        free(matchFreqErr[j]);
        matchFreqErr[j] = NULL;
        free(matchTimeErr[j]);
        matchTimeErr[j] = NULL;
    }
    free(matchFreqErr);
    matchFreqErr = NULL;
    free(matchTimeErr);
    matchTimeErr = NULL;

    return nCtrMatch;
}
 */

/*
 * 判断两个脉冲时间是否满足周期性
 * t1:脉冲1的时间
 * nt1:相对于首脉冲，是第几个周期时间
 * t1Index:首脉冲的脉冲周期索引
 * pulseT:脉冲周期
 * nPulseT:脉冲周期个数
 */
bool isMatchPulseT(float t1, int nt1, int t1Index, float t2, int nt2, int t2Index, float *pulseT, int nPulseT, float pulseTErr)
{
    float diffTime = 0, tErr = 0;
    float sumPulseT = 0.0;
    int nt = 0;
    for (int i = 0; i < nPulseT; i++)
        sumPulseT += pulseT[i];

    if (t1 <= t2)
    {
        diffTime = t2 - t1;
        diffTime = fmod(diffTime, sumPulseT);
        tErr = fabs(diffTime);
        while (1)
        {
            if (tErr < pulseTErr)
                return true;
            if (diffTime < 0)
                break;
            diffTime -= pulseT[(t1Index + nt1 + nt) % nPulseT];
            nt++;
        }
        return false;
    }
    else
        return isMatchPulseT(t2, nt2, t2Index, t1, nt1, t1Index, pulseT, nPulseT, pulseTErr);
}

/*
 * 根据幅度差异，判断两个脉冲是否是同一类
 */
bool isAmpPhaseDiff(struct ctrPulse pulse1, struct ctrPulse pulse2)
{
    float ampErr = 6; // 幅度容差

    if (fabs(pulse1.meanAmp[pulse1.antIndex] - pulse2.meanAmp[pulse2.antIndex]) > ampErr)
        return true;
    else
        return false;
}

/*
检查脉冲的周期性和脉宽，只针对跳频在频率和时间方向都有周期的遥控脉冲
*/
int checkPulseCycleAndWidth(int *ctrMatch, int **match, struct ctrPulse *pulse, int libIndex, int *checkedPulse, int nChecked, int *freqIndex, int *maxnT, struct UAVLib *UAVtypes)
{
    int j, k, p, q;
    float minPulseTime, maxPulseTime, sumPulseT; // 脉冲中最早和最晚的脉冲时间
    int pj, pk;                                  // 脉冲的临时索引号

    int oneMatch[5][MaxNumCtrT];          // 记录倍数周期上的脉冲编号
    float oneMatchTimeErr[5][MaxNumCtrT]; // 记录倍数周期的时间偏差
    float oneMatchFreqErr[5][MaxNumCtrT]; //
    int indexPulseT[MaxCtrPulse] = {
        0,
    };

    float diffTime, resTime, TErr, freqErr;
    int nt = 0, nT = 0, nBigT = 0;
    int count = 0, maxCount = 0, maxIndex = 0;

    int nFreq = UAVtypes[libIndex].nfreqp;
    int hoppCount, pulseWCount[MaxPulseInGroup];
    int nCtrMatch = 0; // 记录match中确定是遥控脉冲的总行数
    bool isNewCtr, isMeetPulseW;
    int sameIndex, pulseNum1, pulseNum2;

    // 求取脉冲时间跨越多少个脉冲周期
    minPulseTime = pulse[checkedPulse[0]].pulseTime;
    maxPulseTime = pulse[checkedPulse[0]].pulseTime;
    for (j = 1; j < nChecked; j++)
    {
        pj = checkedPulse[j];
        if (pulse[pj].pulseTime > maxPulseTime)
            maxPulseTime = pulse[pj].pulseTime;
        if (pulse[pj].pulseTime < minPulseTime)
            minPulseTime = pulse[pj].pulseTime;
    }

    sumPulseT = 0.0;
    for (j = 0; j < UAVtypes[libIndex].nPulseT; j++)
    {
        sumPulseT += UAVtypes[libIndex].pulseT[j];
    }
    *maxnT = floor(((maxPulseTime - minPulseTime) / sumPulseT + 1) * UAVtypes[libIndex].nPulseT + 0.5);
    if (*maxnT > 99)
        *maxnT = 99;

    // 标记跳频时间距离整数倍数周期最近，且距离跳频表中频点最近的脉冲
    for (j = 0; j < nChecked; j++)
    {
        pj = checkedPulse[j];

        for (p = 0; p < UAVtypes[libIndex].nPulseT; p++)
        {
            oneMatch[p][0] = j;
            for (q = 1; q < MaxNumCtrT; q++)
                oneMatch[p][q] = -1;

            for (k = 0; k < nChecked; k++)
            {
                if (j != k)
                {
                    pk = checkedPulse[k];

                    if (isAmpPhaseDiff(pulse[pk], pulse[pj])) // 两个脉冲幅度相差太大
                        continue;

                    diffTime = pulse[pk].pulseTime - pulse[pj].pulseTime;
                    if (diffTime < 0)
                        continue;

                    if (UAVtypes[libIndex].hoppType == 1) // 若频点只在时间方向跳动，则频率误差只与该频点进行比较
                        freqErr = fabs(UAVtypes[libIndex].freqPoints[freqIndex[j]] - pulse[pk].freq);

                    nBigT = floor(diffTime / sumPulseT);
                    resTime = diffTime - nBigT * sumPulseT;

                    nt = 0;
                    while (1)
                    {
                        TErr = fabs(resTime);
                        if (TErr < UAVtypes[libIndex].pulseTErr)
                        {
                            nT = UAVtypes[libIndex].nPulseT * nBigT + nt;
                            if (nT > 99)
                                continue;

                            if (UAVtypes[libIndex].hoppType == 0) // 若频点是固定频率表跳动，则与跳频表进行比较
                                freqErr = fabs(UAVtypes[libIndex].freqPoints[(freqIndex[j] + nT) % nFreq] - pulse[pk].freq);

                            if (nT > 0 && nT < MaxNumCtrT && (freqErr < UAVtypes[libIndex].freqErr || UAVtypes[libIndex].meetHopp < 1))
                            {
                                if (oneMatch[p][nT] < 0)
                                {
                                    oneMatch[p][nT] = k;
                                    oneMatchTimeErr[p][nT] = TErr;
                                    oneMatchFreqErr[p][nT] = freqErr;
                                }
                                else if (TErr < oneMatchTimeErr[p][nT] && freqErr < oneMatchFreqErr[p][nT])
                                {
                                    oneMatch[p][nT] = k;
                                    oneMatchTimeErr[p][nT] = TErr;
                                    oneMatchFreqErr[p][nT] = freqErr;
                                }
                            }
                            break;
                        }
                        if (resTime < 0)
                            break;
                        resTime -= UAVtypes[libIndex].pulseT[(p + nt) % UAVtypes[libIndex].nPulseT];
                        nt++;
                    }
                }
            }
        }
        // 统计每种起点的周期匹配次数，选择匹配次数最大的
        maxCount = 0;
        maxIndex = 0;
        for (k = 0; k < UAVtypes[libIndex].nPulseT; k++)
        {
            count = 0;
            for (p = 0; p < MaxNumCtrT; p++)
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
        indexPulseT[j] = maxIndex;
        for (k = 0; k < MaxNumCtrT; k++)
            match[j][k] = oneMatch[maxIndex][k];
    }

    // 统计match中每行满足跳频周期和跳频频点的脉冲，必须要满足多个跳频点数
    for (j = 0; j < nChecked; j++)
    {
        // 统计满足跳频表的频点数，和满足脉宽要求的频点数
        hoppCount = 0;
        for (k = 0; k < UAVtypes[libIndex].nPulseW; k++)
            pulseWCount[k] = 0;
        for (k = 0; k < (*maxnT) + 1; k++)
        {
            pk = match[j][k];
            if (match[j][k] >= 0) // 若该倍数周期存在脉冲
            {
                hoppCount++;
                for (int p = 0; p < UAVtypes[libIndex].nPulseW; p++)
                {
                    if (fabs(UAVtypes[libIndex].pulseW[p] - pulse[checkedPulse[pk]].pulseW) < UAVtypes[libIndex].pulseWErr[p])
                    {
                        pulseWCount[p]++;
                        break;
                    }
                }
            }
        }
        // 若满足跳频频点规律的脉冲个数大于最低限度，且至少有一个或多个脉宽符合要求，则将该脉冲串与记录在案的遥控脉冲进行去重
        isMeetPulseW = false;
        for (k = 0; k < UAVtypes[libIndex].nPulseW; k++)
        {
            if (pulseWCount[k] >= UAVtypes[libIndex].meetPulseW[k])
            {
                isMeetPulseW = true;
                break;
            }
        }

        // 去重
        if (hoppCount >= UAVtypes[libIndex].meetHopp && isMeetPulseW)
        {
            isNewCtr = true;
            // 与已记录的遥控脉冲去重
            for (k = 0; k < nCtrMatch; k++)
            {
                for (p = 0; p < (*maxnT) + 1; p++)
                {
                    pj = match[j][p];
                    for (q = 0; q < (*maxnT) + 1; q++)
                    {
                        pk = match[ctrMatch[k]][q];
                        if (pj >= 0 && pk >= 0) // 从两者中各挑出一个遥控脉冲，是否满足跳频周期
                        {
                            if (UAVtypes[libIndex].hoppType == 1) // 若只在时间方向跳频
                            {
                                // if (fmod(abs(pulse[checkedPulse[pj]].pulseTime - pulse[checkedPulse[pk]].pulseTime), UAVtypes[libIndex].pulseT[0])
                                //    < UAVtypes[libIndex].pulseTErr)  //若频点时间差是倍数周期，则是同一个遥控
                                if (isMatchPulseT(
                                        pulse[checkedPulse[pj]].pulseTime, p, indexPulseT[j], pulse[checkedPulse[pk]].pulseTime, q, indexPulseT[ctrMatch[k]], UAVtypes[libIndex].pulseT,
                                        UAVtypes[libIndex].nPulseT, UAVtypes[libIndex].pulseTErr))
                                {
                                    isNewCtr = false;
                                    sameIndex = k;
                                    break;
                                }
                            }
                            else // 若在时间和频率方向均跳频
                            {
                                if (fabs(pulse[checkedPulse[pj]].freq - pulse[checkedPulse[pk]].freq) < UAVtypes[libIndex].freqErr && isMatchPulseT(
                                                                                                                                          pulse[checkedPulse[pj]].pulseTime, p, indexPulseT[j], pulse[checkedPulse[pk]].pulseTime, q, indexPulseT[ctrMatch[k]], UAVtypes[libIndex].pulseT,
                                                                                                                                          UAVtypes[libIndex].nPulseT, UAVtypes[libIndex].pulseTErr))
                                // 若频点接近且频点时间差是倍数周期，则是同一个遥控
                                {
                                    isNewCtr = false;
                                    sameIndex = k;
                                    break;
                                }
                                else if (fabs(pulse[checkedPulse[pj]].pulseTime - pulse[checkedPulse[pk]].pulseTime) < UAVtypes[libIndex].pulseTErr) // 若时间很接近（可能是频谱泄漏），则是同一个遥控
                                {
                                    isNewCtr = false;
                                    sameIndex = k;
                                    break;
                                }
                            }
                        }
                    }
                    if (!isNewCtr)
                        break;
                }
                if (!isNewCtr)
                    break;
            }

            if (isNewCtr) // 若是一个新的遥控脉冲串，则追加到记录
            {
                ctrMatch[nCtrMatch] = j;
                nCtrMatch++;
                if (nCtrMatch >= MaxPulseInGroup)
                    break;
            }
            else // 若不是新的遥控脉冲串，则与跟它同类型的遥控脉冲串比较脉冲个数，脉冲个数多的留下来
            {
                pulseNum1 = 0;
                pulseNum2 = 0;
                for (k = 0; k < (*maxnT) + 1; k++)
                {
                    if (match[j][k] >= 0)
                        pulseNum1++;
                    if (match[ctrMatch[sameIndex]][k] >= 0)
                        pulseNum2++;
                }
                if (pulseNum1 > pulseNum2) // 若新的脉冲串遥控脉冲数量较多，则替换原有的脉冲串
                    ctrMatch[sameIndex] = j;
                else if (pulseNum1 == pulseNum2) // 若脉冲数量一样，则幅度强的留下
                {
                    if (pulse[checkedPulse[match[j][0]]].meanAmp[pulse[0].antIndex] > pulse[checkedPulse[match[ctrMatch[sameIndex]][0]]].meanAmp[pulse[0].antIndex])
                        ctrMatch[sameIndex] = j;
                }
            }
        }
    }

    return nCtrMatch;
}

/*
检查脉冲的周期性和脉宽，只针对跳频在时间方向有周期,频率不固定的遥控脉冲
*/
int checkPulseTAndWidth(int *ctrMatch, int **match, struct ctrPulse *pulse, int libIndex, int *checkedPulse, int nChecked, int *freqIndex, int *maxnT, struct UAVLib *UAVtypes)
{
    int j, k, p, q;
    int minPulseTime, maxPulseTime; // 脉冲中最早和最晚的脉冲时间
    int pj, pk;                     // 脉冲的临时索引号

    float diffFreq;
    int nFreq = UAVtypes[libIndex].nfreqp;
    int hoppCount, pulseWCount[MaxPulseInGroup];
    int nCtrMatch = 0; // 记录match中确定是遥控脉冲的总行数
    bool isNewCtr, isMeetPulseW;
    int sameIndex, pulseNum1, pulseNum2, pulseNum1Ind, pulseNum2Ind, pulseNumInd;

    int oneMatch[5][MaxNumCtrT];          // 记录倍数周期上的脉冲编号
    float oneMatchTimeErr[5][MaxNumCtrT]; // 记录倍数周期的时间偏差
    int indexPulseT[MaxCtrPulse] = {
        0,
    };

    float diffTime, resTime, TErr, sumPulseT = 0.0;
    int nt = 0, nT = 0, nBigT = 0;
    int count = 0, maxCount = 0, maxIndex = 0, matchBFlag = 0;

    // 求取脉冲时间跨越多少个脉冲周期
    minPulseTime = pulse[checkedPulse[0]].pulseTime;
    maxPulseTime = pulse[checkedPulse[0]].pulseTime;
    for (j = 1; j < nChecked; j++)
    {
        pj = checkedPulse[j];
        if (pulse[pj].pulseTime > maxPulseTime)
            maxPulseTime = pulse[pj].pulseTime;
        if (pulse[pj].pulseTime < minPulseTime)
            minPulseTime = pulse[pj].pulseTime;
    }

    sumPulseT = 0.0;
    for (j = 0; j < UAVtypes[libIndex].nPulseT; j++)
    {
        sumPulseT += UAVtypes[libIndex].pulseT[j];
    }
    *maxnT = floor(((maxPulseTime - minPulseTime) / sumPulseT + 1) * UAVtypes[libIndex].nPulseT + 0.5);

    // 标记跳频时间距离整数倍数周期最近，且距离跳频表中频点最近的脉冲
    for (j = 0; j < nChecked; j++)
    {
        pj = checkedPulse[j];
        matchBFlag = 0;
        for (p = 0; p < UAVtypes[libIndex].nPulseW; p++)
        {
            if (fabs(UAVtypes[libIndex].pulseW[p] - pulse[pj].pulseW) < UAVtypes[libIndex].pulseWErr[p])
            {
                matchBFlag = 1;
                break;
            }
        }
        for (p = 0; p < UAVtypes[libIndex].nPulseT; p++)
        {
            oneMatch[p][0] = j;
            for (q = 1; q < MaxNumCtrT; q++)
                oneMatch[p][q] = -1;

            if (matchBFlag == 0)
                continue;
            for (k = 0; k < nChecked; k++)
            {
                if (j != k)
                {
                    pk = checkedPulse[k];
                    matchBFlag = 0;
                    for (int p1 = 0; p1 < UAVtypes[libIndex].nPulseW; p1++)
                    {
                        if (fabs(UAVtypes[libIndex].pulseW[p1] - pulse[pk].pulseW) < UAVtypes[libIndex].pulseWErr[p1])
                        {
                            matchBFlag = 1;
                            break;
                        }
                    }
                    if (matchBFlag == 0)
                        continue;

                    if (isAmpPhaseDiff(pulse[pk], pulse[pj])) // 两个脉冲幅度相差太大
                        continue;

                    if (fabs(pulse[pk].freq - pulse[pj].freq) < 1)
                        continue;

                    diffTime = pulse[pk].pulseTime - pulse[pj].pulseTime;
                    if (diffTime < 0)
                        continue;

                    nBigT = floor(diffTime / sumPulseT);
                    resTime = diffTime - nBigT * sumPulseT;

                    nt = 0;
                    while (1)
                    {
                        TErr = fabs(resTime);
                        if (TErr < UAVtypes[libIndex].pulseTErr)
                        {
                            nT = UAVtypes[libIndex].nPulseT * nBigT + nt;

                            if (nT > 0 && nT < MaxNumCtrT)
                            {
                                if (oneMatch[p][nT] < 0)
                                {
                                    oneMatch[p][nT] = k;
                                    oneMatchTimeErr[p][nT] = TErr;
                                }
                                else if (TErr < oneMatchTimeErr[p][nT])
                                {
                                    oneMatch[p][nT] = k;
                                    oneMatchTimeErr[p][nT] = TErr;
                                }
                            }
                            break;
                        }
                        if (resTime < 0)
                            break;

                        resTime -= UAVtypes[libIndex].pulseT[(p + nt) % UAVtypes[libIndex].nPulseT];
                        nt++;
                    }
                }
            }
        }
        // 统计每种起点的周期匹配次数，选择匹配次数最大的
        maxCount = 0;
        maxIndex = 0;
        for (k = 0; k < UAVtypes[libIndex].nPulseT; k++)
        {
            count = 0;
            for (p = 0; p < MaxNumCtrT; p++)
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
        indexPulseT[j] = maxIndex;
        for (k = 0; k < MaxNumCtrT; k++)
            match[j][k] = oneMatch[maxIndex][k];
    }

    // 统计match中每行满足跳频周期的脉冲，必须要满足多个跳频点数
    for (j = 0; j < nChecked; j++)
    {
        // 统计满足跳频表的频点数，和满足脉宽要求的频点数
        hoppCount = 0;
        for (k = 0; k < UAVtypes[libIndex].nPulseW; k++)
            pulseWCount[k] = 0;
        for (k = 0; k < (*maxnT) + 1; k++)
        {
            pk = match[j][k];
            if (match[j][k] >= 0) // 若该倍数周期存在脉冲
            {
                hoppCount++;
                for (int p = 0; p < UAVtypes[libIndex].nPulseW; p++)
                {
                    if (fabs(UAVtypes[libIndex].pulseW[p] - pulse[checkedPulse[pk]].pulseW) < UAVtypes[libIndex].pulseWErr[p])
                    {
                        pulseWCount[p]++;
                        break;
                    }
                }
            }
        }
        // 若满足跳频频点规律的脉冲个数大于最低限度，且至少有一个或多个脉宽符合要求，则将该脉冲串与记录在案的遥控脉冲进行去重
        isMeetPulseW = false;
        for (k = 0; k < UAVtypes[libIndex].nPulseW; k++)
        {
            if (pulseWCount[k] >= UAVtypes[libIndex].meetPulseW[k])
            {
                isMeetPulseW = true;
                break;
            }
        }

        // 去重
        if (hoppCount >= UAVtypes[libIndex].meetHopp && isMeetPulseW)
        {
            isNewCtr = true;
            // 与已记录的遥控脉冲去重
            for (k = 0; k < nCtrMatch; k++)
            {
                for (p = 0; p < (*maxnT) + 1; p++)
                {
                    pj = match[j][p];
                    for (q = 0; q < (*maxnT) + 1; q++)
                    {
                        pk = match[ctrMatch[k]][q];
                        if (pj >= 0 && pk >= 0) // 从两者中各挑出一个遥控脉冲，是否满足跳频周期
                        {
                            if (isMatchPulseT(
                                    pulse[checkedPulse[pj]].pulseTime, p, indexPulseT[j], pulse[checkedPulse[pk]].pulseTime, q, indexPulseT[ctrMatch[k]], UAVtypes[libIndex].pulseT,
                                    UAVtypes[libIndex].nPulseT, UAVtypes[libIndex].pulseTErr))
                            // 若频点时间差是倍数周期，则是同一个遥控
                            {
                                isNewCtr = false;
                                sameIndex = k;
                                break;
                            }
                            else if (fabs(pulse[checkedPulse[pj]].pulseTime - pulse[checkedPulse[pk]].pulseTime) < UAVtypes[libIndex].pulseTErr) // 若时间很接近（可能是频谱泄漏），则是同一个遥控
                            {
                                isNewCtr = false;
                                sameIndex = k;
                                break;
                            }
                        }
                    }
                    if (!isNewCtr)
                        break;
                }
                if (!isNewCtr)
                    break;
            }

            if (isNewCtr) // 若是一个新的遥控脉冲串，则追加到记录
            {
                ctrMatch[nCtrMatch] = j;
                nCtrMatch++;
                if (nCtrMatch >= MaxPulseInGroup)
                    break;
            }
            else // 若不是新的遥控脉冲串，则与跟它同类型的遥控脉冲串比较脉冲个数，脉冲个数多的留下来
            {
                pulseNum1 = 0;
                pulseNum2 = 0;
                pulseNum1Ind = 0;
                pulseNum2Ind = 0;
                pulseNumInd = 0;
                for (k = 0; k < (*maxnT) + 1; k++)
                {
                    if (match[j][k] >= 0)
                    {
                        pulseNum1++;
                        pulseNum1Ind = k;
                    }
                    if (match[ctrMatch[sameIndex]][k] >= 0)
                    {
                        pulseNum2++;
                        pulseNum2Ind = k;
                    }
                    if (pulseNum1Ind == pulseNum2Ind && pulseNumInd == 0)
                    {
                        pulseNumInd = pulseNum1Ind;
                    }
                }
                // printf("%d,%d\n",pulseNum1 , pulseNum2);
                if (pulseNum1 > pulseNum2) // 若新的脉冲串遥控脉冲数量较多，则替换原有的脉冲串
                    ctrMatch[sameIndex] = j;
                else if (pulseNum1 == pulseNum2) // 若脉冲数量一样，则幅度强的留下
                {
                    if (pulseNumInd != 0)
                    {
                        pulseNum1Ind = pulseNumInd;
                        pulseNum2Ind = pulseNumInd;
                    }
                    // printf("%d,%d\n",match[j][pulseNum1Ind],match[ctrMatch[sameIndex]][pulseNum2Ind]);
                    // printf("%d,%d\n",checkedPulse[match[j][pulseNum1Ind]],checkedPulse[match[ctrMatch[sameIndex]][pulseNum2Ind]]);
                    if (pulse[checkedPulse[match[j][pulseNum1Ind]]].meanAmp[pulse[0].antIndex] > pulse[checkedPulse[match[ctrMatch[sameIndex]][pulseNum2Ind]]].meanAmp[pulse[0].antIndex])
                        ctrMatch[sameIndex] = j;
                }
            }
        }
    }

    return nCtrMatch;
}

/*
检查脉冲的脉宽，附加了带宽的要求
*/
int checkPulseW(int *pulseWMatch, int *checkedFreqIndex, struct ctrPulse *pulse, int libIndex, int *checkedPulse, int nChecked, int *freqIndex, struct UAVLib *UAVtypes)
{
    int j, k;
    int pj;
    int pulseWCount[maxPulseWNumInLib]; // 记录满足脉宽的脉冲个数
    int nPulseWMatch = 0;               // 脉宽匹配的次数
    bool isNewCtr;

    // 脉宽计数初始化为0
    for (j = 0; j < UAVtypes[libIndex].nPulseW; j++)
        pulseWCount[j] = 0;

    for (j = 0; j < nChecked; j++)
    {
        pj = checkedPulse[j];
        for (k = 0; k < UAVtypes[libIndex].nPulseW; k++)
        {
            // 若脉宽满足误差范围
            if (fabs(pulse[pj].pulseW - UAVtypes[libIndex].pulseW[k]) < UAVtypes[libIndex].pulseWErr[k] && pulse[pj].pulseBW > 0.75 * UAVtypes[libIndex].pulseBW[0])
            {
                pulseWCount[k]++;
                pulseWMatch[nPulseWMatch] = pj;
                checkedFreqIndex[nPulseWMatch] = freqIndex[j];
                nPulseWMatch++;
            }
        }
    }
    // 判断脉宽满足的次数是否达标
    isNewCtr = true;
    for (int j = 0; j < UAVtypes[libIndex].nPulseW; j++)
    {
        if (pulseWCount[j] < UAVtypes[libIndex].meetPulseW[j])
        {
            isNewCtr = false;
            break;
        }
    }
    if (!isNewCtr) // 若脉宽满足的次数不达标，则返回0
        nPulseWMatch = 0;
    return nPulseWMatch;
}

/*
使用符合要求的脉冲，生成遥控脉冲
*/
int genUavCtrPulse(struct ctrPulse *pulse, int *checkCtrPulse, int nChecked, int *freqIndex, int libIndex, struct pulseGroup *uavCtrPulse, int nUavCtrPulse, struct UAVLib *UAVtypes, int antennaNum)
{
    int i, j, k, pj;

    if (nUavCtrPulse >= MaxUAV) // 若大于最大无人机遥控的检测数量，则不再搜索
        return nUavCtrPulse;

    uavCtrPulse[nUavCtrPulse].uavIndex = libIndex;
    uavCtrPulse[nUavCtrPulse].pulseBW = UAVtypes[libIndex].pulseBW[0]; // 指定库内的遥控带宽，遥控一般只有1种带宽，所以索引号为0
    for (k = 0; k < NCh; k++)
        uavCtrPulse[nUavCtrPulse].onAnt[k] = false;
    uavCtrPulse[nUavCtrPulse].onAnt[pulse[checkCtrPulse[0]].antIndex] = true;

    if (nChecked > MaxPulseInGroup)
        uavCtrPulse[nUavCtrPulse].nPulse = MaxPulseInGroup;
    else
        uavCtrPulse[nUavCtrPulse].nPulse = nChecked;
    for (j = 0; j < nChecked; j++)
    {
        if (j >= MaxPulseInGroup)
            break;
        pj = checkCtrPulse[j];
        if (UAVtypes[libIndex].hoppType == 1 && UAVtypes[libIndex].pulseBW[0] > 5) // 若该类型的无人机仅在时间方向跳频，且带宽较大，则脉冲频率直接使用库内的中心频点，因为脉冲中心频率不够准确
            uavCtrPulse[nUavCtrPulse].freq[j] = UAVtypes[libIndex].freqPoints[freqIndex[j]];
        else
            uavCtrPulse[nUavCtrPulse].freq[j] = pulse[pj].freq;
        uavCtrPulse[nUavCtrPulse].pulseTime[j] = pulse[pj].pulseTime;
        uavCtrPulse[nUavCtrPulse].pulseW[j] = pulse[pj].pulseW;
        for (k = 0; k < antennaNum; k++)
        {
            uavCtrPulse[nUavCtrPulse].meanAmp[k][j] = pulse[pj].meanAmp[k];
            for (i = 0; i < PhaseBinNum; i++)
            {
                uavCtrPulse[nUavCtrPulse].phaseHist[k][j][i] = 0;         // pulse[pj].phaseHist[k][i];
                uavCtrPulse[nUavCtrPulse].weightedPhaseHist[k][j][i] = 0; // pulse[pj].weightedPhaseHist[k][i];
            }
        }
    }
    nUavCtrPulse++;
    return nUavCtrPulse;
}

/*
遥控脉冲的无人机型号识别，识别完成后加入到脉冲串，每个无人机型号的所有脉冲是一个脉冲串
*/
int checkCtrPulse(struct ctrPulse *pulse, int nCtrPulse, struct pulseGroup *chCtrPulse, struct UAVLib *UAVtypes, int nUav, int antennaNum)
{
    if (nCtrPulse < 1) // 若输入的单个通道检出的脉冲数为0，则直接返回
    {
        return 0;
    }

    int i, j, k;
    int nFreq;                     // 某种无人机的特征频点个数
    int checkedPulse[MaxCtrPulse]; // 记录经过每个步骤筛选的脉冲索引号
    int nChecked;                  // 记录通过初步筛选的脉冲个数
    int pj, pk;                    // 脉冲的临时索引号
    int **match = (int **)malloc(MaxCtrPulse * sizeof(int *));
    for (i = 0; i < MaxCtrPulse; i++)
        match[i] = (int *)malloc(MaxNumCtrT * sizeof(int));

    int freqIndex[MaxCtrPulse];        // 筛选出合理的频率的脉冲频点在跳频表内的编号
    int checkedFreqIndex[MaxCtrPulse]; // 最终筛选后的跳频表内编号
    int maxnT, nT;

    int nCtrMatch;                  // 记录match中确定是遥控脉冲的总行数
    int ctrMatch[MaxCtrPulse];      // 记录match中确定是遥控脉冲的行号
    int nPulseWMatch;               // 记录脉宽满足的总次数
    int pulseWMatch[MaxCtrPulse];   // 记录脉宽达标的脉冲序号
    int nPulseC_WMatch;             // 脉冲周期和脉宽满足次数
    int PulseC_WMatch[MaxCtrPulse]; // 记录脉宽达标的脉冲序号

    int nChCtrPulse = 0; // 遥控脉冲串个数初始化为0
    int count = 0;       // 计数

    for (i = 0; i < nUav; i++)
    {
        if (UAVtypes[i].useMethod != 2) // 过滤
            continue;
        nFreq = UAVtypes[i].nfreqp;
        nCtrMatch = 0;

        // 检查频点是否在范围内
        nChecked = checkPulseFreq(pulse, nCtrPulse, i, freqIndex, checkedPulse, UAVtypes);
        if (nChecked < 1)
            continue;

        if (UAVtypes[i].nPulseT > 0 && UAVtypes[i].pulseT[0] > 0) // 若脉冲周期个数>0，表示脉冲存在周期性，根据脉冲的周期容差判断
        {
            if (UAVtypes[i].isFixedFreq && UAVtypes[i].hoppType < 2) // 若遥控频点固定,且是按照频点和周期跳频，则要满足跳频表
            {
                nCtrMatch = checkPulseCycleAndWidth(ctrMatch, match, pulse, i, checkedPulse, nChecked, freqIndex, &maxnT, UAVtypes);
                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                // 特殊处理，针对脉冲很少容易误判的情况
                if (nCtrMatch >= 1 && UAVtypes[i].pulseT[0] > 30) // 若该种无人机遥控脉冲周期很大，如果该信道出现很多个其它脉冲，则认为是误判
                {
                    for (j = 0; j < nCtrMatch; j++)
                    {
                        count = 0;
                        pj = ctrMatch[j];
                        for (k = 0; k < nChecked; k++)
                        {
                            pk = checkedPulse[k];
                            nT = floor(fabs(pulse[checkedPulse[pj]].pulseTime - pulse[pk].pulseTime) / UAVtypes[i].pulseT[0] + 0.5);
                            if (fabs(UAVtypes[i].freqPoints[freqIndex[k]] - pulse[pk].freq) < UAVtypes[i].freqErr && fabs(fabs(pulse[checkedPulse[pj]].pulseTime - pulse[pk].pulseTime) - nT * UAVtypes[i].pulseT[0]) > 5)
                            {
                                count++;
                            }
                        }
                        if (count > 2)
                        {
                            nCtrMatch = 0;
                            break;
                        }
                    }
                }
            }
            else // 若频点不固定，只要满足周期
            {
                nCtrMatch = checkPulseTAndWidth(ctrMatch, match, pulse, i, checkedPulse, nChecked, freqIndex, &maxnT, UAVtypes);
            }
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            for (j = 0; j < nCtrMatch; j++)
            {
                nPulseC_WMatch = 0;
                for (k = 0; k < maxnT; k++)
                {
                    pk = match[ctrMatch[j]][k];

                    if (pk >= 0)
                    {
                        PulseC_WMatch[nPulseC_WMatch] = checkedPulse[pk]; // pulse中的索引
                        checkedFreqIndex[nPulseC_WMatch] = freqIndex[pk];
                        nPulseC_WMatch++;
                    }
                }
                nChCtrPulse = genUavCtrPulse(pulse, PulseC_WMatch, nPulseC_WMatch, checkedFreqIndex, i, chCtrPulse, nChCtrPulse, UAVtypes, antennaNum);
            }
        }
        else // 若频点不固定只是知道范围，判断脉宽是否满足条件
        {
            nPulseWMatch = checkPulseW(pulseWMatch, checkedFreqIndex, pulse, i, checkedPulse, nChecked, freqIndex, UAVtypes);
            if (nPulseWMatch > 0) // 确认是该类型无人机，生成脉冲串
            {
                nChCtrPulse = genUavCtrPulse(pulse, pulseWMatch, nPulseWMatch, checkedFreqIndex, i, chCtrPulse, nChCtrPulse, UAVtypes, antennaNum);
            }
        }
    }

    for (i = 0; i < MaxCtrPulse; i++)
    {
        free(match[i]);
        match[i] = NULL;
    }
    free(match);
    match = NULL;

    return nChCtrPulse;
}

/*
合并不同通道的遥控脉冲串
*/
void mergeUavCtrPulse(struct pulseGroup *uavCtrPulse, int *nUavCtrPulse1, struct pulseGroup *chCtrPulse, int nChCtrPulse, struct UAVLib *UAVtypes, int antennaNum)
{
    int j, k, p, q, r;

    bool isNew;                      // 记录脉冲串是否能够合并，若不能则为true
    int uavCtrIndex;                 // 记录可以合并的遥控脉冲串编号
    int antCtrIndex;                 // 记录新加入的脉冲串的通道序号
    int mergedFlag[MaxPulseInGroup]; // 记录脉冲串内的脉冲是否被合并掉，若最后都没有被合并则是新的脉冲，需加入到记录中
    bool isMerged;                   // 记录脉冲是否被加入到记录中
    float sumPulseT = 0.0;
    int nUavCtrPulse = *nUavCtrPulse1;

    for (j = 0; j < nChCtrPulse; j++)
    {
        isNew = true;
        for (k = 0; k < nUavCtrPulse; k++)
        {
            // 能够合并的条件
            if (chCtrPulse[j].uavIndex == uavCtrPulse[k].uavIndex) // 若无人机型号一致
            {
                for (p = 0; p < UAVtypes[chCtrPulse[j].uavIndex].nPulseT; p++)
                {
                    sumPulseT += UAVtypes[chCtrPulse[j].uavIndex].pulseT[p];
                }
                for (p = 0; p < chCtrPulse[j].nPulse; p++)
                {
                    for (q = 0; q < uavCtrPulse[k].nPulse; q++)
                    {
                        // 若两组脉冲串中有任何一个脉冲的脉冲时间和频率很接近，则可以合并
                        if (fabs(chCtrPulse[j].pulseTime[p] - uavCtrPulse[k].pulseTime[q]) < 2 && fabs(chCtrPulse[j].freq[p] - uavCtrPulse[k].freq[q]) < 1)
                        {
                            uavCtrIndex = k;
                            isNew = false;
                            break;
                        }
                        // 若脉冲串是有周期的，则只需要两组脉冲任何两个脉冲的时间差是周期的整数倍，则可以合并。若只在时间方向跳频，则还需要频点很接近。
                        if (UAVtypes[chCtrPulse[j].uavIndex].nPulseT > 0 && UAVtypes[chCtrPulse[j].uavIndex].pulseT[0] > 0)
                        {
                            if (fmod(fabs(chCtrPulse[j].pulseTime[p] - uavCtrPulse[k].pulseTime[q]), sumPulseT) < UAVtypes[chCtrPulse[j].uavIndex].pulseTErr)
                            {
                                if (UAVtypes[chCtrPulse[j].uavIndex].hoppType == 1) // 若只在时间方向跳频，合并的条件则更严格，需要频点接近
                                {
                                    if (fabs(chCtrPulse[j].freq[p] - uavCtrPulse[k].freq[q]) < 1)
                                    {
                                        uavCtrIndex = k;
                                        isNew = false;
                                        break;
                                    }
                                }
                                else
                                {
                                    uavCtrIndex = k;
                                    isNew = false;
                                    break;
                                }
                            }
                        }
                    }
                    if (!isNew)
                        break;
                }
                if (!isNew)
                    break;
            }
        }
        if (isNew) // 追加新的遥控脉冲串
        {
            uavCtrPulse[nUavCtrPulse].id = chCtrPulse[j].id;
            uavCtrPulse[nUavCtrPulse].nPulse = chCtrPulse[j].nPulse;
            uavCtrPulse[nUavCtrPulse].pulseBW = chCtrPulse[j].pulseBW;
            uavCtrPulse[nUavCtrPulse].uavIndex = chCtrPulse[j].uavIndex;
            for (k = 0; k < antennaNum; k++)
            {
                uavCtrPulse[nUavCtrPulse].onAnt[k] = chCtrPulse[j].onAnt[k];
            }
            for (k = 0; k < chCtrPulse[j].nPulse; k++)
            {
                uavCtrPulse[nUavCtrPulse].freq[k] = chCtrPulse[j].freq[k];
                uavCtrPulse[nUavCtrPulse].pulseTime[k] = chCtrPulse[j].pulseTime[k];
                uavCtrPulse[nUavCtrPulse].pulseW[k] = chCtrPulse[j].pulseW[k];
                for (p = 0; p < antennaNum; p++)
                {
                    uavCtrPulse[nUavCtrPulse].meanAmp[p][k] = chCtrPulse[j].meanAmp[p][k];
                    uavCtrPulse[nUavCtrPulse].meanPhase[p][k] = chCtrPulse[j].meanPhase[p][k];
                    uavCtrPulse[nUavCtrPulse].phaseVar[p][k] = chCtrPulse[j].phaseVar[p][k];
                }
            }
            nUavCtrPulse++;
        }
        else // 合并脉冲串
        {
            for (k = 0; k < antennaNum; k++)
            {
                // 增加一个通道存在遥控的标记
                if (chCtrPulse[j].onAnt[k])
                {
                    uavCtrPulse[uavCtrIndex].onAnt[k] = true;
                    antCtrIndex = k;
                    break;
                }
            }
            uavCtrPulse[uavCtrIndex].pulseBW = (uavCtrPulse[uavCtrIndex].pulseBW + chCtrPulse[j].pulseBW) / 2.0; // 带宽平均
            for (k = 0; k < MaxPulseInGroup; k++)
                mergedFlag[k] = -1;
            // 对脉冲串内，脉冲时间和频率都很接近的脉冲，脉冲参数取平均
            for (k = 0; k < uavCtrPulse[uavCtrIndex].nPulse; k++)
            {
                for (p = 0; p < chCtrPulse[j].nPulse; p++)
                {
                    if (UAVtypes[uavCtrPulse[uavCtrIndex].uavIndex].isFixedFreq > 0)
                    {
                        if (mergedFlag[p] < 0 && fabs(uavCtrPulse[uavCtrIndex].pulseTime[k] - chCtrPulse[j].pulseTime[p]) < 2 && fabs(uavCtrPulse[uavCtrIndex].freq[k] - chCtrPulse[j].freq[p]) < 1)
                        {
                            mergedFlag[p] = 1;
                            uavCtrPulse[uavCtrIndex].pulseTime[k] = (uavCtrPulse[uavCtrIndex].pulseTime[k] + chCtrPulse[j].pulseTime[p]) / 2.0;
                            uavCtrPulse[uavCtrIndex].pulseW[k] = (uavCtrPulse[uavCtrIndex].pulseW[k] + chCtrPulse[j].pulseW[p]) / 2.0;
                            uavCtrPulse[uavCtrIndex].freq[k] = (uavCtrPulse[uavCtrIndex].freq[k] + chCtrPulse[j].freq[p]) / 2.0;
                            uavCtrPulse[uavCtrIndex].meanAmp[antCtrIndex][k] = (uavCtrPulse[uavCtrIndex].meanAmp[antCtrIndex][k] + chCtrPulse[j].meanAmp[antCtrIndex][p]) / 2.0;
                            uavCtrPulse[uavCtrIndex].meanPhase[antCtrIndex][k] = (uavCtrPulse[uavCtrIndex].meanPhase[antCtrIndex][k] + chCtrPulse[j].meanPhase[antCtrIndex][p]) / 2.0;
                            uavCtrPulse[uavCtrIndex].phaseVar[antCtrIndex][k] = (uavCtrPulse[uavCtrIndex].phaseVar[antCtrIndex][k] + chCtrPulse[j].phaseVar[antCtrIndex][p]) / 2.0;
                        }
                    }
                    else
                    {
                        if (mergedFlag[p] < 0 && fabs(uavCtrPulse[uavCtrIndex].pulseTime[k] - chCtrPulse[j].pulseTime[p]) < 2)
                        {
                            mergedFlag[p] = 1;
                            if (uavCtrPulse[uavCtrIndex].meanAmp[antCtrIndex][k] < chCtrPulse[j].meanAmp[antCtrIndex][p])
                            {
                                uavCtrPulse[uavCtrIndex].pulseTime[k] = chCtrPulse[j].pulseTime[p];
                                uavCtrPulse[uavCtrIndex].pulseW[k] = chCtrPulse[j].pulseW[p];
                                uavCtrPulse[uavCtrIndex].freq[k] = chCtrPulse[j].freq[p];
                                uavCtrPulse[uavCtrIndex].meanAmp[antCtrIndex][k] = chCtrPulse[j].meanAmp[antCtrIndex][p];
                                uavCtrPulse[uavCtrIndex].meanPhase[antCtrIndex][k] = chCtrPulse[j].meanPhase[antCtrIndex][p];
                                uavCtrPulse[uavCtrIndex].phaseVar[antCtrIndex][k] = chCtrPulse[j].phaseVar[antCtrIndex][p];
                            }
                        }
                    }
                }
            }
            // 对脉冲串内，新出现的脉冲，则直接插入到记录中
            for (k = 0; k < chCtrPulse[j].nPulse; k++)
            {
                if (uavCtrPulse[uavCtrIndex].nPulse >= MaxPulseInGroup)
                {
                    break;
                }
                if (mergedFlag[k] < 0) // 若脉冲串内有脉冲没有被合并，则插入到记录中
                {
                    isMerged = false;
                    for (p = 0; p < uavCtrPulse[uavCtrIndex].nPulse; p++)
                    {
                        if (uavCtrPulse[uavCtrIndex].pulseTime[p] > chCtrPulse[j].pulseTime[k])
                        {
                            isMerged = true;
                            if (uavCtrPulse[uavCtrIndex].nPulse >= MaxPulseInGroup)
                            {
                                break;
                            }
                            uavCtrPulse[uavCtrIndex].nPulse++; // 脉冲串内脉冲数+1
                            // 往后挪动脉冲
                            for (q = uavCtrPulse[uavCtrIndex].nPulse - 1; q > p; q--)
                            {
                                uavCtrPulse[uavCtrIndex].pulseTime[q] = uavCtrPulse[uavCtrIndex].pulseTime[q - 1];
                                uavCtrPulse[uavCtrIndex].pulseW[q] = uavCtrPulse[uavCtrIndex].pulseW[q - 1];
                                uavCtrPulse[uavCtrIndex].freq[q] = uavCtrPulse[uavCtrIndex].freq[q - 1];
                                for (r = 0; r < antennaNum; r++)
                                {
                                    uavCtrPulse[uavCtrIndex].meanAmp[r][q] = uavCtrPulse[uavCtrIndex].meanAmp[r][q - 1];
                                    uavCtrPulse[uavCtrIndex].meanPhase[r][q] = uavCtrPulse[uavCtrIndex].meanPhase[r][q - 1];
                                    uavCtrPulse[uavCtrIndex].phaseVar[r][q] = uavCtrPulse[uavCtrIndex].phaseVar[r][q - 1];
                                }
                            }
                            // 插入新的脉冲
                            uavCtrPulse[uavCtrIndex].pulseTime[p] = chCtrPulse[j].pulseTime[k];
                            uavCtrPulse[uavCtrIndex].pulseW[p] = chCtrPulse[j].pulseW[k];
                            uavCtrPulse[uavCtrIndex].freq[p] = chCtrPulse[j].freq[k];
                            for (r = 0; r < antennaNum; r++)
                            {
                                uavCtrPulse[uavCtrIndex].meanAmp[r][p] = chCtrPulse[j].meanAmp[r][k];
                                uavCtrPulse[uavCtrIndex].meanPhase[r][p] = chCtrPulse[j].meanPhase[r][k];
                                uavCtrPulse[uavCtrIndex].phaseVar[r][p] = chCtrPulse[j].phaseVar[r][k];
                            }
                            break;
                        }
                    }
                    if (!isMerged) // 若脉冲没有被插入，则插入到脉冲串尾部
                    {
                        if (uavCtrPulse[uavCtrIndex].nPulse >= MaxPulseInGroup)
                        {
                            break;
                        }
                        uavCtrPulse[uavCtrIndex].pulseTime[uavCtrPulse[uavCtrIndex].nPulse] = chCtrPulse[j].pulseTime[k];
                        uavCtrPulse[uavCtrIndex].pulseW[uavCtrPulse[uavCtrIndex].nPulse] = chCtrPulse[j].pulseW[k];
                        uavCtrPulse[uavCtrIndex].freq[uavCtrPulse[uavCtrIndex].nPulse] = chCtrPulse[j].freq[k];
                        for (r = 0; r < antennaNum; r++)
                        {
                            uavCtrPulse[uavCtrIndex].meanAmp[r][uavCtrPulse[uavCtrIndex].nPulse] = chCtrPulse[j].meanAmp[r][k];
                            uavCtrPulse[uavCtrIndex].meanPhase[r][uavCtrPulse[uavCtrIndex].nPulse] = chCtrPulse[j].meanPhase[r][k];
                            uavCtrPulse[uavCtrIndex].phaseVar[r][uavCtrPulse[uavCtrIndex].nPulse] = chCtrPulse[j].phaseVar[r][k];
                        }
                        uavCtrPulse[uavCtrIndex].nPulse++; // 脉冲串内脉冲数+1
                    }
                }
            }
        }
    }
    *nUavCtrPulse1 = nUavCtrPulse;
}

/*
 * 计算无人机遥控脉冲的置信度
 */
char calcUavCtrPulsePr(struct pulseGroup uavCtrPulse, struct UAVLib *UAVtypes, int antennaNum)
{
    float pr = 0.0;
    int uavIndex = uavCtrPulse.uavIndex;

    // 跳频规律估计部分置信度
    if (UAVtypes[uavIndex].pulseT[0] > 0 && UAVtypes[uavIndex].nPulseT > 0)
    {
        if (UAVtypes[uavIndex].isFixedFreq) // 跳频在时间和频率方向有周期
        {
            pr += 0.5 * uavCtrPulse.nPulse / (106 / UAVtypes[uavIndex].pulseT[0] + 1.0);
        }
        else // 跳频只在时间方向有周期
        {
            pr += 0.3 * uavCtrPulse.nPulse / (106 / UAVtypes[uavIndex].pulseT[0] + 1.0);
            float minFreq = UAVtypes[uavIndex].freqPoints[1], maxFreq = UAVtypes[uavIndex].freqPoints[0];
            for (int i = 0; i < uavCtrPulse.nPulse; i++)
            {
                if (uavCtrPulse.freq[i] < minFreq)
                    minFreq = uavCtrPulse.freq[i];
                if (uavCtrPulse.freq[i] > maxFreq)
                    maxFreq = uavCtrPulse.freq[i];
            }
            pr += 0.2 * (maxFreq - minFreq) / (UAVtypes[uavIndex].freqPoints[1] - UAVtypes[uavIndex].freqPoints[0]);
        }
    }
    else
    {
        float minFreq = UAVtypes[uavIndex].freqPoints[1], maxFreq = UAVtypes[uavIndex].freqPoints[0];
        for (int i = 0; i < uavCtrPulse.nPulse; i++)
        {
            if (uavCtrPulse.freq[i] < minFreq)
                minFreq = uavCtrPulse.freq[i];
            if (uavCtrPulse.freq[i] > maxFreq)
                maxFreq = uavCtrPulse.freq[i];
        }
        pr += 0.5 * (maxFreq - minFreq) / (UAVtypes[uavIndex].freqPoints[1] - UAVtypes[uavIndex].freqPoints[0]);
    }

    // 幅度向量一致性估计置信度
    float ampVectorCenter[NCh], maxAmp = 0.0;
    for (int i = 0; i < antennaNum; i++)
    {
        ampVectorCenter[i] = 0.0;
        for (int j = 0; j < uavCtrPulse.nPulse; j++)
        {
            ampVectorCenter[i] += uavCtrPulse.meanAmp[i][j];
        }
        ampVectorCenter[i] /= uavCtrPulse.nPulse;
        if (ampVectorCenter[i] > maxAmp)
            maxAmp = ampVectorCenter[i];
    }
    float dist = 0.0;
    for (int i = 0; i < antennaNum; i++)
    {
        for (int j = 0; j < uavCtrPulse.nPulse; j++)
        {
            dist += pow(uavCtrPulse.meanAmp[i][j] - ampVectorCenter[i], 2);
        }
    }
    dist /= NCorr * uavCtrPulse.nPulse;
    pr += 0.5 * (1 - exp(-sqrt(maxAmp * maxAmp / dist) / 10));

    char result = pr * 100;
    return result;
}

/*
标记id号，计算遥控脉冲的角度、距离和置信度
*/
void calcUavCtrPulse(
    struct pulseGroup *uavCtrPulse, int nUavCtrPulse, float *sumCorr, float (*sumCorrAmp)[128], float (*sumCorrPhase)[128], int nRows, int nCols, float dt, float dF, float *cenFreqOnAllCh, float *gainOnAllCh,
    struct detectParams detectedParam, struct UAVLib *UAVtypes)
{
    int i, j, k, ii, jj;
    int freqIndexL, freqIndexR, q1, q2;
    int corrIndex;
    float corrSumReal, corrSumImage;
    float squareSump, sump, squareSumpWrap, sumpWrap;
    float phaseVarBlock, phaseVarWrapBlock;
    float phasePoint, phaseCount;

    float miu = detectedParam.miu;
    int antennaNum = detectedParam.antennaNum;
    float d[NCh] = {0.0}, initPhase[NCh] = {
                              0.0,
                          };
    float onePulseMeanAmp[NCh] = {0.0}; // 临时存储一个脉冲的平均幅度
    float meanPhase[NCh] = {0.0};       // 存储一个脉冲的平均相位
    float angle[NCh] = {0.0};           // 所有相位测角
    // float d[NCorr] = {0.0}, initPhase[NCorr] = {0.0,};
    // float onePulseMeanAmp[5] = { 0.0 };  //临时存储一个脉冲的平均幅度
    // float meanPhase[5] = { 0.0 };        //存储一个脉冲的平均相位
    // float angle[5] = { 0.0 };            //所有相位测角
    float freq;
    float phaseVar[NCh] = {0.0};
    float allPulseAngle[MaxPulseInGroup];
    float range = 0.0;
    int antFace = 0;
    int corrFlag = (detectedParam.doaTyp == 5) ? 2 : 1;
    int archiFlag, n;

    for (i = 0; i < nUavCtrPulse; i++)
    {
        for (j = 0; j < uavCtrPulse[i].nPulse; j++)
        {
            freq = uavCtrPulse[i].freq[j];
            // d = selAntParamNew(initPhase, freq, detectedParam);
            selAntParamPro(initPhase, freq, detectedParam, d);
            q1 = floor(uavCtrPulse[i].pulseTime[j] / dt + 0.5);                              // 脉冲起点时间列索引
            q2 = floor((uavCtrPulse[i].pulseTime[j] + uavCtrPulse[i].pulseW[j]) / dt + 0.5); // 脉冲终点时间列索引

            if (q1 >= nRows)
                q1 = nRows - 1;
            if (q2 >= nRows)
                q2 = nRows - 1;

            for (k = 0; k < antennaNum; k++)
            {
                int ind = 0;
                if (corrFlag == 2 && k >= 4)
                    ind = (k + 1) % 5;
                else
                    ind = k;

                if (corrFlag == 2 && k % 2 == 1)
                    uavCtrPulse[i].onAnt[k] = uavCtrPulse[i].onAnt[k - 1];

                freqIndexL = floor((uavCtrPulse[i].freq[j] - uavCtrPulse[i].pulseBW / 2 - cenFreqOnAllCh[0]) / dF + nCols / 2 + 0.5);
                freqIndexR = floor((uavCtrPulse[i].freq[j] + uavCtrPulse[i].pulseBW / 2 - cenFreqOnAllCh[0]) / dF + nCols / 2 + 0.5);

                if (freqIndexL < 5)
                    freqIndexL = 5;
                if (freqIndexR > nCols - 6)
                    freqIndexR = nCols - 6;

                corrSumReal = 0.0;
                corrSumImage = 0.0;
                squareSump = 0.0, sump = 0.0, squareSumpWrap = 0.0, sumpWrap = 0.0;
                phaseCount = 0;
                for (ii = 0; ii < PhaseBinNum; ii++)
                {
                    uavCtrPulse[i].phaseHist[k][j][ii] = 0.0;
                    uavCtrPulse[i].weightedPhaseHist[k][j][ii] = 0.0;
                }
                for (ii = q1; ii <= q2; ii++)
                {
                    for (jj = freqIndexL; jj <= freqIndexR; jj++)
                    {
                        phaseCount++;
                        if (corrFlag == 2)
                        {
                            n = (k % 2 == 1) ? (k - 1) : k;
                            corrIndex = (ii + nRows * (k % 2)) * NCorr * nCols * 2 + n / 2 * nCols * 2 + 2 * jj;
                        }
                        else
                            corrIndex = ii * NCorr * nCols * 2 + k * nCols * 2 + 2 * jj;

                        //                         corrSumReal += sumCorr[corrIndex];
                        //                         corrSumImage += sumCorr[corrIndex + 1];

                        phasePoint = sumCorrPhase[ii][jj];
                        int index = fmod(phasePoint - initPhase[k] + 9 * PI, 2 * PI) / (2 * PI) * PhaseBinNum;
                        uavCtrPulse[i].phaseHist[k][j][index]++;
                        uavCtrPulse[i].weightedPhaseHist[k][j][index] += sumCorrAmp[ii][jj];

                        squareSump += phasePoint * phasePoint;
                        sump += phasePoint;
                        phasePoint = fmod(phasePoint + 3.0 * PI, 2.0 * PI) - PI;
                        squareSumpWrap += phasePoint * phasePoint;
                        sumpWrap += phasePoint;
                    }
                }
                archiFlag = 1;
                if (detectedParam.doaTyp == 5 && k % 2 == 0)
                    archiFlag = -1;

                //                 uavCtrPulse[i].meanPhase[k][j] = fmod(archiFlag *atan2(corrSumImage, corrSumReal) - initPhase[k] + 7* PI, 2 * PI) - PI;

                phaseVarBlock = (squareSump + phaseCount * (sump / phaseCount) * (sump / phaseCount) - 2 * (sump / phaseCount) * sump) / phaseCount;
                phaseVarWrapBlock = (squareSumpWrap + phaseCount * (sumpWrap / phaseCount) * (sumpWrap / phaseCount) - 2 * (sumpWrap / phaseCount) * sumpWrap) / phaseCount;
                if (phaseVarBlock < phaseVarWrapBlock)
                    uavCtrPulse[i].phaseVar[k][j] = phaseVarBlock;
                else
                    uavCtrPulse[i].phaseVar[k][j] = phaseVarWrapBlock;

                onePulseMeanAmp[k] = uavCtrPulse[i].meanAmp[k][j];
                //                 meanPhase[k] = uavCtrPulse[i].meanPhase[k][j];
                //                 phaseVar[k] = uavCtrPulse[i].phaseVar[k][j];

                if (detectedParam.ampSelAntType == 2 || detectedParam.ampSelAntType == 4)
                    uavCtrPulse[i].onAnt[k] = 1;

                /*
                //使用直方图选天线
                float maxHist=0,sumHist=0;
                for(ii=0;ii<PhaseBinNum;ii++)
                {
                    sumHist+=uavCtrPulse[i].phaseHist[k][j][ii];
                    if(uavCtrPulse[i].phaseHist[k][j][ii]>maxHist)
                        maxHist=uavCtrPulse[i].phaseHist[k][j][ii];
                }
                phaseVar[k]=sumHist/maxHist;
                */
            }

            // uavCtrPulse[i].angle[j] = calcAngle(meanPhase, onePulseMeanAmp, freq, d, uavCtrPulse[i].onAnt, phaseVar, antFace, detectedParam);
            uavCtrPulse[i].angle[j] = 0; // calcAnglePro(meanPhase, onePulseMeanAmp, freq, d, uavCtrPulse[i].onAnt, phaseVar, antFace, detectedParam);

            uavCtrPulse[i].distance[j] = amp2dist(onePulseMeanAmp, uavCtrPulse[i].onAnt, miu, gainOnAllCh);
            uavCtrPulse[i].antFace[j] = antFace;
            range += uavCtrPulse[i].distance[j];

            allPulseAngle[j] = uavCtrPulse[i].angle[j];
        }

        uavCtrPulse[i].azimuth = angleSelMean(allPulseAngle, uavCtrPulse[i].nPulse); // 最终输出的角度
        uavCtrPulse[i].range = range / uavCtrPulse[i].nPulse;
        uavCtrPulse[i].possibility = calcUavCtrPulsePr(uavCtrPulse[i], UAVtypes, antennaNum);

        uavCtrPulse[i].id = i;
    }
}

// 遥控检测主函数
void ctrDetect(
    struct pulseGroup *uavCtrPulse, int *nUavCtrPulse1, float *sumCorr, float (*sumCorrAmp)[128], float (*sumCorrPhase)[128], int nRows, int nCols, float *cenFreqOnAllCh, float *gainOnAllCh, float dt, float dF,
    struct UAVLib *UAVtypes, int nUAV, struct detectParams detectedParam)
{
    int corrIndex;

    float ctrSNR = detectedParam.ctrSNR;
    int antennaNum = detectedParam.antennaNum;

    struct ctrPulse pulse[MaxCtrPulse];
    int nPulse;                           // 记录单个通道脉冲数量
    struct pulseGroup chCtrPulse[MaxUAV]; // 存储单个通道的无人机遥控脉冲串
    int nChCtrPulse = 0;                  // 记录单个通道遥控脉冲串的个数
    int nUavCtrPulse = 0;
    int corrFlag = (detectedParam.doaTyp == 5) ? 2 : 1;

    // 滤波后的幅度，用于图传或遥控
    //     float ***filtedAmp = createGrid(antennaNum, nRows, nCols);
    float filtedAmp1[NROWS][NFFT / NSumCol];

    for (int i = 0; i < antennaNum; i++)
    {
        for (int j = 0; j < nRows; j++)
        {
            for (int k = 0; k < nCols; k++)
            {
                filtedAmp1[j][k] = sumCorrAmp[j][k];
            }
        }
    }
    //     单线程
    int nSpan[2];
    // clock_t t1,t2;
    // t1=clock();
    nSpan[0] = detectedParam.ctrIndexSpan[0];
    nSpan[1] = detectedParam.ctrIndexSpan[5];
    filter(sumCorrAmp, nRows, nCols, filtedAmp1, detectedParam.hpMask, detectedParam.hpMaskH, detectedParam.hpMaskW, nSpan, 1);
    float **filtedAmp = createMatrix(nRows, nCols);
    for (int i = 0; i < antennaNum; i++)
    {
        for (int j = 0; j < nRows; j++)
        {
            for (int k = 0; k < nCols; k++)
            {
                filtedAmp[j][k] = filtedAmp1[j][k];
            }
        }
    }
    // t2=clock();
    // double useTime=(double)(t2-t1)/CLOCKS_PER_SEC;
    // printf("ctrfilt use time %f ms\n",useTime*1000);

    // 多线程
    //     pthread_t   tid[NCh] = { 0 };
    //     filterParam filtParam[NCh];
    //     int         err;
    //     for (int i = 0; i < antennaNum; i++)
    //     {
    //         int ind = 0;
    // 	if(detectedParam.doaTyp == 5 && i >= 4)
    // 	    ind = (i+1)%5;
    // 	else
    // 	    ind = i;
    // 	filtParam[i].m = nRows;
    //         filtParam[i].n = nCols;
    //         filtParam[i].mask = detectedParam.hpMask;
    //         filtParam[i].h = detectedParam.hpMaskH;
    //         filtParam[i].w = detectedParam.hpMaskW;
    //         filtParam[i].edgeFlag = 1;
    //         filtParam[i].matIn = *(sumCorrAmp + i);
    //         filtParam[i].matOut = *(filtedAmp + i);
    //         filtParam[i].n1 = detectedParam.ctrIndexSpan[ind];
    //         filtParam[i].n2 = detectedParam.ctrIndexSpan[ind + NCorr];
    //         err = pthread_create(&tid[i], NULL, filter_thread, (void *)&filtParam[i]);
    //
    //         if (err != 0)
    //         {
    //             for (int j = 0; j < i; j++)
    //                 pthread_join(tid[j], NULL);
    //             printf("Create thread failed!\n");
    //             freeGrid(filtedAmp, antennaNum, nRows, nCols);
    //             *nUavCtrPulse = 0;
    //             return;
    //         }
    //     }
    //
    //     for (int i = 0; i < antennaNum; i++)
    //         pthread_join(tid[i], NULL);

    // 遥控脉冲检测
    for (int i = 0; i < antennaNum; i++)
    {
        // 检测所有遥控脉冲
        nPulse = ctrPulseDetect(pulse, filtedAmp, sumCorrAmp, nRows, nCols, dt, dF, i, cenFreqOnAllCh, detectedParam); // 每组相关幅度的脉冲检测
        /*
        static int fp_num = 0;
        char fp_name[24]= {0};
        printf("fp_num:%d i:%d NCorr:%d\n",fp_num,i,NCorr);
        sprintf(fp_name,"./data/%d_pulse_%d.csv",fp_num,i);
        int fp = open(fp_name,O_RDWR | O_CREAT,0666);
        if(fp<0)
        {
            perror("open pulse");
        }
        else
        {
            for(j=0;j<nPulse;j++)
            {
                dprintf(fp,"%d,%f,%f,%f,%f",pulse[j].antIndex,pulse[j].freq,pulse[j].pulseTime,pulse[j].pulseW,pulse[j].pulseBW);
                for(int z=0;z<NCorr;z++)
                {
                    dprintf(fp,",%f",pulse[z].meanAmp[z]);
                }
                dprintf(fp,"\n");
            }
            close(fp);
            if(i==NCorr-1)
            {
                fp_num++;
            }
        }*/

        // 合并大脉冲
        mergeLargeBWpulse(pulse, nPulse, dt, antennaNum, corrFlag);
        // 单个通道的遥控脉冲识别和参数计boxNum算
        nChCtrPulse = checkCtrPulse(pulse, nPulse, chCtrPulse, UAVtypes, nUAV, antennaNum);

        // 将单个通道新生成的脉冲串与合并后的脉冲串逐个进行合并
        mergeUavCtrPulse(uavCtrPulse, &nUavCtrPulse, chCtrPulse, nChCtrPulse, UAVtypes, antennaNum);
    }

    // 计算遥控脉冲的角度、距离等参数
    calcUavCtrPulse(uavCtrPulse, nUavCtrPulse, sumCorr, sumCorrAmp, sumCorrPhase, nRows, nCols, dt, dF, cenFreqOnAllCh, gainOnAllCh, detectedParam, UAVtypes);

    *nUavCtrPulse1 = nUavCtrPulse;
    freeMatrix(filtedAmp, nRows);
    //     freeGrid(filtedAmp, antennaNum, nRows, nCols);  //清除大数组
}
