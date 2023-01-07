#include <time.h>
// #include"relay.h"
#include "uavDetect.h"
#include "paramRead.h"
// extern "C"{
// #include "user_udp.h"
// }
#include <sys/stat.h>
#include "readDetectParam.h"
#include "main.h"
/*
获取系统时间
 */
int getTime(char *timeNow)
{
    time_t timep;
    struct tm *p;
    time(&timep);
    p = localtime(&timep);
    if (p == NULL)
    {
        return -1;
    }
    sprintf(timeNow, "%d-%02d-%02d %02d:%02d:%02d", p->tm_year + 1900, p->tm_mon + 1, p->tm_mday, p->tm_hour, p->tm_min, p->tm_sec);
    return 0;
}

/*
 * 检测磁盘空间
 * free_space： 允许写到的剩余空间(单位G)，filename：哪个文件所在的磁盘
 */
int check_free_space2(const int free_space, char *filename)
{

    char buf[1024] = {0};
    FILE *pf = NULL;
    char fileSystem[20] = "";
    long int allSize = -1, usedSize = -1, availableSize = -1;
    char orderbuf[256] = {0};
    sprintf(orderbuf, "df %s", filename);
    // 获取文件所属磁盘空间
    if ((pf = popen(orderbuf, "r")) == NULL)
    {
        sprintf(buf, "popen '%s' failed ", orderbuf);
        // send_monitor(5, buf);
        return -1;
    }

    memset(buf, 0, 1024);
    fgets(buf, sizeof(buf), pf);
    fgets(buf, sizeof(buf), pf);

    buf[strlen(buf) - 1] = '\0'; // delete the '\n'
    // printf("--%s--\n",buf);
    sscanf(buf, "%s %ld %ld %ld", fileSystem, &allSize, &usedSize, &availableSize);
    // printf("fileSystem:%s,all:%ld,used:%ld,available:%ld\n",fileSystem,allSize,usedSize,availableSize);

    pclose(pf);

    if (availableSize < free_space * 1000 * 1000)
    {
        memset(buf, 0, 1024);
        sprintf(buf, "the %s less than %d G", filename, free_space);
        // send_monitor(5, buf);
        return -2;
    }

    return availableSize;
}

/*
  解析相关谱帧头和帧尾
*/
int trackFrame(float *sumCorr, int *nRows, int *nSumRow, struct deviceParam *devParam, struct detectParams detectedParam)
{
    int i, j, k;
    int corrIndex;
    int corrFlag = (detectedParam.doaTyp == 5) ? 2 : 1;
    unsigned long long int freqAllCh[5]; // 5个接收通道的频率
    unsigned long long int gainAllch[5]; // 5个接收通道增益

    unsigned char *bit64 = NULL; // 64位临时存储

    unsigned int *bit1_63_32;
    unsigned int *frameCount; // 帧计数
    unsigned int *bit2_63_32;
    unsigned int *bit2_31_0;
    unsigned long long int *timeStamp = NULL; // 数据时戳
    unsigned long long int *freq = NULL;      // 接收通道频率
    unsigned char *gain = NULL;               // 接收通道增益

    unsigned short *enableTr = NULL;          // 发射通道使能
    unsigned char *nSumRowPtr = NULL;         // 累加次数
    unsigned long long int *freqTrPtr = NULL; // 指针，发射频率
    unsigned long long int freqTr = 0;        // 发射频率
    unsigned long long int *bit253_63_24 = NULL;
    char *childBoardT = NULL; // 子板节温
    char *A_BoardT = NULL;    // 子板节温
    char *B_BoardT = NULL;    // 子板节温
    unsigned short *bit254_63_48 = NULL;
    unsigned char *lngLatValid = NULL; // 经纬度有效指示（0x55有效）
    unsigned short *lngDegPtr = NULL;  // 指针，经度中的度
    unsigned short lngDeg = 0;         // 经度中的度
    unsigned int *lngMinutePtr = NULL; // 指针，经度中的分
    unsigned int lngMinute = 0;        // 经度中的分
    unsigned char *latDegPtr = NULL;   // 指针，纬度中的度
    unsigned char latDeg = 0;          // 纬度中的度
    unsigned int *latMinutePtr = NULL; // 指针，纬度中的分
    unsigned int latMinute = 0;        // 纬度中的分
    unsigned char *bit255_31_24 = NULL;
    unsigned char *angleValid = NULL;    // 寻北仪方位角有效指示
    unsigned short *compassAng = NULL;   // 寻北仪方位角
    unsigned short *turntableAng = NULL; // 转台角
    unsigned int *frameCheck = NULL;     // 帧校验
    unsigned int *bit256_31_0 = NULL;    // 帧尾

    // 读取FPGA版本号，分AB板
    char FPGA_BoardA_Version[12];
    FPGA_BoardA_Version[0] = '2';
    FPGA_BoardA_Version[1] = '0';
    char FPGA_BoardB_Version[12];
    FPGA_BoardB_Version[0] = '2';
    FPGA_BoardB_Version[1] = '0';

    corrIndex = 1 * 512 + 252 * 2;
    bit64 = (unsigned char *)(sumCorr + corrIndex);
    for (i = 0; i < 4; i++)
        sprintf(FPGA_BoardA_Version + 2 + 2 * i, "%02x", *(bit64 + 7 - i));
    strncpy(devParam->FPGA_BoardA_Version, FPGA_BoardA_Version, 10);

    corrIndex = 4 * 512 + 252 * 2;
    bit64 = (unsigned char *)(sumCorr + corrIndex);
    for (i = 0; i < 4; i++)
        sprintf(FPGA_BoardB_Version + 2 + 2 * i, "%02x", *(bit64 + 7 - i));
    strncpy(devParam->FPGA_BoardB_Version, FPGA_BoardB_Version, 10);

    // GPS时间
    k = 0;
    corrIndex = k * 512 + 4;
    timeStamp = (unsigned long long int *)(sumCorr + corrIndex);
    bit64 = (unsigned char *)timeStamp;
    devParam->gpsTime.hour = *(bit64 + 6);
    devParam->gpsTime.minute = *(bit64 + 5);
    devParam->gpsTime.seconds = *(bit64 + 4);
    devParam->gpsTime.nanoSecond = *((unsigned int *)bit64);

    // 转台角
    k = 0;
    corrIndex = k * 512 + 5 * 2;
    bit64 = (unsigned char *)(sumCorr + corrIndex);
    turntableAng = (unsigned short *)bit64;
    devParam->turntableAngle = *turntableAng;

    // 经纬度
    k = 0;
    corrIndex = k * 512 + 253 * 2;
    bit64 = (unsigned char *)(sumCorr + corrIndex);
    lngLatValid = bit64 + 5;

    lngDegPtr = (unsigned short *)(bit64 + 3);
    lngDeg = *lngDegPtr & ((1u << 15u) - 1);
    unsigned char lngDegSign = (*lngDegPtr & (1u << 15u)) >> 15u;

    lngMinutePtr = (unsigned int *)bit64;

    lngMinute = (*lngMinutePtr) << 8;
    lngMinute = lngMinute >> 8;
    // printf("lngLatValid=0x%x\n", *lngLatValid);
    // printf("lngDeg=%d\n",lngDeg);
    // printf("lngMinute=%d\n",lngMinute);

    corrIndex = k * 512 + 254 * 2;
    bit64 = (unsigned char *)(sumCorr + corrIndex);
    latDegPtr = (unsigned char *)(bit64 + 7);
    latDeg = *latDegPtr & ((1u << 7u) - 1);
    unsigned char latDegSign = (*latDegPtr & (1u << 7u)) >> 7u;

    latMinutePtr = (unsigned int *)(bit64 + 4);
    latMinute = (*latMinutePtr) << 8;
    latMinute = latMinute >> 8;
    // 		printf("latDeg=%d\n",latDeg);
    // 		printf("latMinute=%d\n",latMinute);

    angleValid = (unsigned char *)(bit64 + 2);
    compassAng = (unsigned short *)bit64;

    unsigned char *charPtr = (unsigned char *)(sumCorr + 8);
    devParam->decim = *(charPtr + 2);
    devParam->timeCount = *((unsigned int *)(charPtr + 4));
    if (devParam->decim < 2)
        devParam->decim = 1;
    if (devParam->timeCount == 0)
    {
        devParam->timeCount = *((unsigned int *)sumCorr);
        devParam->timeCountVersion = 1;
    }
    else
    {
        devParam->timeCountVersion = 2;
    }

    devParam->frameCount = *((unsigned int *)sumCorr);
    // 读取第一帧5个通道的所有参数
    for (k = 0; k < 5; k++)
    {
        /*
        corrIndex = k * 512 + 1;
        bit1_63_32 = (unsigned int *)(sumCorr + corrIndex);
        // printf("h55444648=0x%x\n", *bit1_63_32);

        corrIndex = k * 512 + 4;
        timeStamp = (unsigned long long int *)(sumCorr + corrIndex);
        bit64 = (unsigned char *)timeStamp;
        devParam->gpsTime.hour = *(bit64 + 6) + 8;
        devParam->gpsTime.minute = *(bit64 + 5);
        devParam->gpsTime.seconds = *(bit64 + 4);
        devParam->gpsTime.nanoSecond = *((unsigned int *)bit64);
        */

        corrIndex = k * 512 + 6;
        freq = (unsigned long long int *)(sumCorr + corrIndex);
        freqAllCh[k] = *freq;
        // printf("freq=%lld\n", *freq);

        corrIndex = k * 512 + 8;
        gain = (unsigned char *)(sumCorr + corrIndex);
        gainAllch[k] = *gain;
        // printf("gain=%lld\n", *gain);

        // 帧尾
        corrIndex = k * 512 + 251 * 2;
        bit64 = (unsigned char *)(sumCorr + corrIndex);
        enableTr = (unsigned short *)(bit64 + 6);
        nSumRowPtr = bit64 + 5;
        *nSumRow = *nSumRowPtr;
        freqTrPtr = (unsigned long long int *)bit64;
        freqTr = (*freqTrPtr) >> 24;

        corrIndex = k * 512 + 252 * 2;
        bit64 = (unsigned char *)(sumCorr + corrIndex);
        childBoardT = (char *)(bit64 + 2);
        devParam->childBoardTemperature[k] = *childBoardT;
        // printf("childBoardTemperature=%d\n", *childBoardT);

        A_BoardT = (char *)(bit64 + 1);
        // printf("A_BoardTemperature=%d\n", *A_BoardT);

        B_BoardT = (char *)bit64;
        // printf("B_BoardTemperature=%d\n", *B_BoardT);

        /*
        corrIndex = k * 512 + 253 * 2;
        bit64 = (unsigned char *)(sumCorr + corrIndex);
        lngLatValid = bit64 + 5;

        lngDegPtr = (unsigned short *)(bit64 + 3);
        lngDeg = *lngDegPtr;
        if (lngDeg > 180)
            lngDeg = 32768 - lngDeg;
        lngMinutePtr = (unsigned int *)bit64;

        lngMinute = (*lngMinutePtr) << 8;
        lngMinute = lngMinute >> 8;
        // printf("lngLatValid=0x%x\n", *lngLatValid);
        // printf("lngDeg=%d\n",lngDeg);
        // printf("lngMinute=%d\n",lngMinute);

        corrIndex = k * 512 + 254 * 2;
        bit64 = (unsigned char *)(sumCorr + corrIndex);
        latDegPtr = (unsigned char *)(bit64 + 7);
        latDeg = *latDegPtr;
        if (latDeg > 90)
            latDeg = latDeg - 128;
        else
            latDeg = -latDeg;

        latMinutePtr = (unsigned int *)(bit64 + 4);
        latMinute = (*latMinutePtr) << 8;
        latMinute = latMinute >> 8;
        // 		printf("latDeg=%d\n",latDeg);
        // 		printf("latMinute=%d\n",latMinute);
        */

        // angleValid = (unsigned char *)(bit64 + 2);
        // compassAng = (unsigned short *)bit64;

        // printf("compassAngle=%d\n", *compassAng);

        // corrIndex = k * 512 + 255 * 2;
        // bit64 = (unsigned char *)(sumCorr + corrIndex);
        // frameCheck = (unsigned int *)(bit64 + 4);
        // printf("frameCheck=0x%x\n", *frameCheck);

        // bit256_31_0 = (unsigned int *)bit64;
        // printf("bit256_31_0=0x%x\n", *bit256_31_0);
    }

    if (*nSumRow > 50 || *nSumRow < 1) // 检查参数。数据传输出错时，把不合理的值去掉
        *nSumRow = 8;

    //*nRows = floor(detectedParam.dataTime * Freqs * 1000.0 / NFFT / (*nSumRow));  //相关复数的行数
    *nRows = 2000;

    // FILE *fp=fopen("framCount.txt","w");
    // 检查每个通道每一行的频率是否相同且不为0，并且每一行的增益相同。若不同，则返回-1

    for (k = 0; k < NCorr - 1; k++)
    {
        int ind = 0;
        if ((detectedParam.doaTyp == 5 || detectedParam.frameNum == 8000) && k == 4)
            continue;
        corrIndex = 512 * 5 * i + k * 512 + 6;
        freq = (unsigned long long int *)(sumCorr + corrIndex);
        corrIndex = 512 * 5 * i + k * 512 + 8;
        gain = (unsigned char *)(sumCorr + corrIndex);

        if (*freq <= 0 || freqAllCh[ind] != *freq || gainAllch[ind] != *gain)
        {
            return -1;
        }

        // 帧计数
        //  corrIndex = 512 * 5 * i + k * 512;
        //  frameCount=(unsigned int *)(sumCorr + corrIndex);
        //  fprintf(fp,"%d,",*frameCount);
    }
    // fprintf(fp,"\n");

    // 检查是否出现多行的数据相同，若出现，则返回-2
    /*
    int  sameCount = 0;
    int sameFlag = false;
    if (detectedParam.writeSumCorr == 2)
    {
        for (i = 0; i < 5; i++)
        {
            for (j = 0; j < *nRows - 1; j++)
            {
                sameFlag = true;
                for (k = 10; k < 256 - 10; k++)
                {
                    corrIndex = 512 * 5 * j + i * 512 + 2 * k;
                    if (sumCorr[corrIndex] != sumCorr[corrIndex + 512 * 5] || sumCorr[corrIndex + 1] != sumCorr[corrIndex + 512 * 5 + 1])
                    {
                        sameFlag = false;
                        break;
                    }
                }
                if (sameFlag)
                    sameCount++;
            }
        }
    }
    if (sameCount > 5)
        return -2;
    */

    // fclose(fp);

    // 对每个通道的中心频率、增益赋值
    for (k = 0; k < 5; k++)
    {
        if (k == 4)
        {
            devParam->cenFreqOnAllCh[4] = devParam->cenFreqOnAllCh[3]; // 从Hz转化为MHz
            devParam->gainOnAllCh[4] = devParam->gainOnAllCh[3];
            continue;
        }
        devParam->cenFreqOnAllCh[k] = freqAllCh[k] / 1.0e6; // 从Hz转化为MHz
        devParam->gainOnAllCh[k] = gainAllch[k];
    }
    // 给出AB板节温、经纬度、寻北仪数值
    devParam->A_BoardTemperature = *A_BoardT;
    devParam->B_BoardTemperature = *B_BoardT;

    if (lngDegSign > 0)
        devParam->longitude = -lngDeg - lngMinute / 60.0e5;
    else
        devParam->longitude = lngDeg + lngMinute / 60.0e5;
    if (latDegSign > 0)
        devParam->latitude = latDeg + latMinute / 60.0e5;
    else
        devParam->latitude = -latDeg - latMinute / 60.0e5;

    if (*lngLatValid == 0x55)
        devParam->lngLatValid = 0;
    else if (*lngLatValid == 0x66)
        devParam->lngLatValid = 1;
    else if (*lngLatValid == 0x77)
        devParam->lngLatValid = 2;
    else
        devParam->lngLatValid = -1;

    // devParam->compassAngle = *compassAng / 100.0;
    if (*angleValid == 0x55)
        devParam->angleValid = 0;
    else if (*angleValid == 0x56)
        devParam->angleValid = 1;
    else if (*angleValid == 0x66)
        devParam->angleValid = 2;
    else if (*angleValid == 0x67)
        devParam->angleValid = 3;
    else if (*angleValid == 0x77)
        devParam->angleValid = 5;
    else if (*angleValid == 0x88)
        devParam->angleValid = 4;
    else if (*angleValid == 0x99)
        devParam->angleValid = 6;
    else
    {
        devParam->angleValid = -1;
        devParam->compassAngle = 0.0;
    }

    /*
    printf("lngLatValid=0x%x\n", *lngLatValid);
    printf("lngDeg=%d\n",lngDeg);
    printf("lngMinute=%d\n",lngMinute);
    printf("latDeg=%d\n",latDeg);
    printf("latMinute=%d\n",latMinute);

    printf("enabelTr=%d,nSumRow=%d,freqTr=%ld\n",*enableTr,*nSumRow,freqTr);
    printf("lng=%f,lat=%f\n",devParam->longitude,devParam->latitude);
    printf("compassAng=%.3f\n",devParam->compassAngle);
    printf("timeStamp=%d:%d:%d:%d\n", devParam->gpsTime.hour,devParam->gpsTime.minute,devParam->gpsTime.seconds,devParam->gpsTime.nanoSecond);
    */
    return 0;
}

void calcAmpPhaseOneCh(float *sumCorr, int nRows, int nCols, float **sumCorrAmp, float **sumCorrPhase, int corrFlag)
{
    int i, j, k;
    int index, n = 0;

    for (i = 0; i < nRows; i++)
    {
        for (j = 0; j < nCols; j++)
        {
            // 			index = i * nCols * 2 + 2 * j;
            index = i * NCorr * nCols * 2 + 2 * j;
            sumCorrAmp[i][j] = 5 * log10(sumCorr[index] * sumCorr[index] + sumCorr[index + 1] * sumCorr[index + 1]);
            sumCorrPhase[i][j] = atan2(sumCorr[index + 1], sumCorr[index]);
        }
    }
}

void getAmpPhaseOneChSumCorr(float *sumCorr, int nRows, int nCols, float **sumCorrAmp, float **sumCorrPhase, int corrFlag)
{
    int i, j, k;
    int index, n = 0;
    int validCols = nCols - 12;

    for (i = 0; i < nRows; i++)
    {
        for (j = 6; j < nCols - 6; j++)
        {
            // 			index = i * nCols * 2 + 2 * j;
            index = i * validCols + j;
            sumCorrAmp[i][j] = 10 * log10(sumCorr[index] + 1);
            sumCorrPhase[i][j] = 0;
        }
        for (j = 0; j < 6; j++)
        {
            sumCorrAmp[i][j] = 0.0;
            sumCorrPhase[i][j] = 0.0;
        }
        for (j = 250; j < 256; j++)
        {
            sumCorrAmp[i][j] = 0.0;
            sumCorrPhase[i][j] = 0.0;
        }
    }
}

int getAmpPhaseCorr(int *sumCorr, int nRows, int nCols, float (*sumCorrAmp)[NFFT / NSumCol], float (*sumCorrPhase)[NFFT / NSumCol], int *udpData, int corrFlag)
{
    float *p = (float *)(sumCorr);

    for (int i = 0; i < nRows; i++)
    {
        for (int k = 0; k < nCols; k++)
        {
            if (k > 3 && k < 126)
            {
                float am = 10 * log10(sqrt(p[0] * p[0] + p[1] * p[1]));
                if (am >= 0 && am < 1000)
                    sumCorrAmp[i][k] = am;
                udpData[i * nCols + k] = sumCorrAmp[i][k];
                float ph = atan2(p[0], p[1]);
                sumCorrPhase[i][k] = ph;
            }
            if (k <= 3 || k >= 126)
            {
                sumCorrAmp[i][k] = 0;
                sumCorrPhase[i][k] = 0;
            }
            p += 2;
        }
    }
    return 0;
}
int getAmpPhaseOneCh(int *sumCorr, int nRows, int nCols, float (*sumCorrAmp)[NFFT / NSumCol], float (*sumCorrPhase)[NFFT / NSumCol], int *udpData, int corrFlag)
{
    int i, j, k = 0;
    int index = 0, n = 0, offsetInd = 0, offsetIndInit = 0, indOffset = 0;
    float centerFreq;
    // 	int validCols = nCols - 12;
    //    xil_printf("%d%d \r\n", nRows,nCols);
    for (i = 0; i < 256; i++)
    {
        if (sumCorr[i] == 0x70209361)
        {
            //               printf("freq = %d\n",sumCorr[i + 127]);
            centerFreq = (float)sumCorr[i + 127];
            offsetInd = i;
            offsetIndInit = i;
            break;
        }
    }
    for (i = 0; i < nRows; i++)
    {
        for (j = 0; j < nCols; j++)
        {
            index = i * nCols + 127 - j + offsetInd;
            indOffset = i * nCols + j + offsetInd;
            sumCorrAmp[i][j] = 10 * log10(sumCorr[index] + 1.0);
            if (j <= 1 || j >= nCols - 2)
                udpData[i * nCols + j] = sumCorr[index + j - 127 + j];
            else
                udpData[i * nCols + j] = sumCorrAmp[i][j];
            //			xil_printf("%d ", sumCorr[index]);
            sumCorrPhase[i][j] = 0;
        }

        if (sumCorr[indOffset + 1] == 0x70209361)
        {
            continue;
        }

        for (k = 0; k < 128; k++)
        {
            if (sumCorr[k + indOffset + 1] == 0x70209361)
            {
                offsetInd = offsetInd + k;
                break;
            }
        }
    }
    return offsetIndInit;
}

/*
   相关求幅度和相位
 */
void calcAmpPhase(float *sumCorr, int nRows, int nCols, float ***sumCorrAmp, float ***sumCorrPhase, int corrFlag)
{
    int i, j, k;
    int index, n = 0;
    float tmpAmp = 0.0;
    // float amp , phase;
    // 正常读取数据（按照上下各一半）
    for (i = 0; i < NCorr * corrFlag; i++)
    {
        for (j = 0; j < nRows; j++)
        {
            for (k = 6; k < nCols - 6; k++)
            {
                if (corrFlag == 2)
                {
                    n = (i % 2 == 1) ? (i - 1) : i;
                    index = (j + nRows * (i % 2)) * NCorr * nCols * 2 + n / 2 * nCols * 2 + 2 * k;
                }
                else
                    index = j * NCorr * nCols * 2 + i * nCols * 2 + 2 * k;
                tmpAmp = 5 * log10(sumCorr[index] * sumCorr[index] + sumCorr[index + 1] * sumCorr[index + 1]);
                if (tmpAmp > 0 && tmpAmp < 200)
                    sumCorrAmp[i][j][k] = tmpAmp;
                else
                    sumCorrAmp[i][j][k] = 50;
                //                 sumCorrAmp[i][j][k] = 5 * log10(sumCorr[index] * sumCorr[index] + sumCorr[index + 1] * sumCorr[index + 1]);
                sumCorrPhase[i][j][k] = atan2(sumCorr[index + 1], sumCorr[index]);
            }
            for (k = 0; k < 6; k++)
            {
                sumCorrAmp[i][j][k] = 0.0;
                sumCorrPhase[i][j][k] = 0.0;
            }
            for (k = 250; k < 256; k++)
            {
                sumCorrAmp[i][j][k] = 0.0;
                sumCorrPhase[i][j][k] = 0.0;
            }
        }
    }

    // FILE *fid = fopen("./dataFile/sumCorrPHASE", "w");
    // printf("sumCorrPHASE = %f, %f\n", sumCorrPHASE[0], sumCorrPHASE[5555]);
    // printf("***************************1***********************************\n");
    // fwrite(sumCorrPHASE, sizeof(float), 5 * nRows, fid);
    // printf("***************************2***********************************\n");
    // fclose(fid);

    // FILE *fid1 = fopen("./dataFile/sumCorrAMP", "w");
    // printf("sumCorrAMP = %f, %f\n", sumCorrAMP[0], sumCorrAMP[5555]);
    // printf("***************************3***********************************\n");
    // fwrite(sumCorrAMP, sizeof(float), 5 * nRows, fid1);
    // printf("***************************4***********************************\n");
    // fclose(fid1);
    // //间隔读取
    //     for (i = 0; i < NCorr * corrFlag; i++)
    //     {
    //         for (j = 0; j < nRows; j++)
    //         {
    //             for (k = 0; k < nCols; k++)
    //             {
    // 		if(corrFlag == 2)
    // 		{
    // 		    n = (i % 2 == 1)?(i - 1):i;
    // 		    index = ( i % 2 + j * 2 )* NCorr * nCols * 2 + n * nCols + 2 * k;
    // 		}
    // 		else
    // 		    index = j * NCorr * nCols * 2 + i * nCols * 2 + 2 * k;
    //                 sumCorrAmp[i][j][k] = 5 * log10(sumCorr[index] * sumCorr[index] + sumCorr[index + 1] * sumCorr[index + 1]);
    //                 sumCorrPhase[i][j][k] = atan2(sumCorr[index + 1], sumCorr[index]);
    //             }
    //         }
    //     }
}

/*
 * 从幅度和相位数据，分离出幅度和相位
 */
void getAmpAndPhase(float ***sumCorrAmp, float ***sumCorrPhase, float *ampPhase, int corrFlag)
{
    int offset = 5 * 2000 * 246;
    int n = 0;
    for (int i = 0; i < NCorr * corrFlag; i++)
    {
        for (int j = 0; j < 2000 / corrFlag; j++)
        {
            for (int k = 5; k < 251; k++)
            {
                if (corrFlag == 2)
                {
                    n = (i % 2 == 1) ? (i - 1) : i;
                    sumCorrAmp[i][j][k] = ampPhase[n / 2 * 246 + (j + 1000 * (i % 2)) * 5 * 246 + k - 5];
                    sumCorrPhase[i][j][k] = ampPhase[offset + n / 2 * 246 + (j + 1000 * (i % 2)) * 5 * 246 + k - 5];
                    //                     NewSaveAMP[i*2000/corrFlag*256+j*256+k] = u_int8_t(round(((ampPhase[n/2*246+(j+1000*(i%2))*5*246+k-5]))/130*255));
                    //                     NewSavePHASE[i*2000/corrFlag*256+j*256+k] = u_int8_t(round(((ampPhase[offset+n/2*246+(j+1000*(i%2))*5*246+k-5]+PI)/(2*PI)*255)));
                }
                else
                {
                    sumCorrAmp[i][j][k] = ampPhase[i * 246 + j * 5 * 246 + k - 5];
                    sumCorrPhase[i][j][k] = ampPhase[offset + i * 246 + j * 5 * 246 + k - 5];
                    //                     NewSaveAMP[i*2000/corrFlag*256+j*256+k] = u_int8_t(round(((ampPhase[i*246+j*5*246+k-5]))/130*255));
                    //                     NewSavePHASE[i*2000/corrFlag*256+j*256+k] = u_int8_t(round(((ampPhase[offset+i*246+j*5*246+k-5]+PI)/(2*PI)*255)));
                }
            }
            for (int k = 0; k < 6; k++)
            {
                sumCorrAmp[i][j][k] = 0.0;
                sumCorrPhase[i][j][k] = 0.0;
                //                 NewSaveAMP[i*2000/corrFlag*256+j*256+k] = 0.0;
                //                 NewSavePHASE[i*2000/corrFlag*256+j*256+k] = 0.0;
            }
            for (int k = 250; k < 256; k++)
            {
                sumCorrAmp[i][j][k] = 0.0;
                sumCorrPhase[i][j][k] = 0.0;
                //                 NewSaveAMP[i*2000/corrFlag*256+j*256+k] = 0.0;
                //                 NewSavePHASE[i*2000/corrFlag*256+j*256+k] = 0.0;
            }
        }
    }
}

// /*
//  * 标校信号幅度和相位
//  */
// void calcRefAmpPhase(float refFreq, float *cenFreqOnAllCh, float dF, float ***sumCorrAmp, float ***sumCorrPhase, int nRows, int nCols, float *refAmp, float *refPhase)
// {
//     int i, j;
//     int refIndex;

//     for (i = 0; i < NCorr; i++)
//     {
//         refIndex = floor((refFreq - cenFreqOnAllCh[i]) / dF + nCols / 2 + 0.5);
//         refAmp[i] = -1.0;
//         refPhase[i] = -10.0;
//         if (refIndex > 4 && refIndex < nCols - 5)
//         {
//             refAmp[i] = 0.0;
//             refPhase[i] = 0.0;
//             for (j = 0; j < nRows; j++)
//             {
//                 refAmp[i] += sumCorrAmp[i][j][refIndex];
//                 refPhase[i] += sumCorrPhase[i][j][refIndex];
//             }
//             refAmp[i] /= nRows;
//             refPhase[i] /= nRows;
//         }
//     }
// }

/*
 * 相关数据写文件
 */
// void writeSumCorrToFile(float *sumCorr, int nRows, char *filePrefix, int SumCorrsign, int SumCorrsize)
// {
//
//     static int num = 0;
//     static int Newnum = 0;
//     static int newnum = 0;
//     static int delnum = 0;
//     char       fileName[100] = { 0 };
//     // char timeNow[100];
//     // getTime(timeNow);
//     switch (SumCorrsign)
//     {
//         case 0:
//         {
//             if (!mkdir("./dataFile", 0777))
//                 return;
//
//             num++;
//             sprintf(fileName, "./dataFile/%s_%s_%d", filePrefix, "sumCorr", num);
//             FILE *fp = fopen(fileName, "w");
//             fwrite(sumCorr, sizeof(float), 512 * 5 * nRows, fp);
//             fclose(fp);
//             break;
//         }
//         case 1:
//         {
//             if (!mkdir("./NewdataFile", 0777))
//             return;
//
//             newnum++;
//             sprintf(fileName, "./NewdataFile/%s_%s_%d", filePrefix, "sumCorr", newnum);
//             FILE *fp = fopen(fileName, "w");
//             fwrite(sumCorr, sizeof(float), 512 * 5 * nRows, fp);
//             fclose(fp);
//
//             if ( newnum - delnum > SumCorrsize ){
//                     delnum++;
//                     sprintf(fileName, "./NewdataFile/%s_%s_%d", filePrefix, "sumCorr", delnum);
//                     remove(fileName);
//                     while(newnum - delnum != SumCorrsize){
//                         delnum++;
//                         sprintf(fileName, "./NewdataFile/%s_%s_%d", filePrefix, "sumCorr", delnum);
//                         remove(fileName);
//                     }
//                 }
//             break;
//         }
//         case 2:
//         {
//             if (!mkdir("./Newdata", 0777))
//             return;
//             float *inform = (float *)malloc(24 * sizeof(float));
//             for (int i = 0; i < 24; i++){
//                 inform[i] = sumCorr[i];
//                 if (i >= 12)
//                     inform[i] = sumCorr[i+488];
//             }
//
//             Newnum ++;
//
//             sprintf(fileName, "./Newdata/%s%s_%d", filePrefix, "NewSumCorr", Newnum);
//             FILE *fp = fopen(fileName, "w");
//             fwrite(inform, sizeof(float), 24*1, fp);
//
//             // fwrite(NewSaveAMP, sizeof(u_int8_t), 256 * 8000, fp);
//             // fwrite(NewSavePHASE, sizeof(u_int8_t), 256 * 8000, fp);
//             fclose(fp);
//             free(inform);
//             break;
//         }
//
//         default:
//             break;
//     }
// }

/*
打印图传脉冲串信息
*/
void printUavPulse(struct pulseGroup *uavPulse, int nUavPulse, struct UAVLib *UAVtypes)
{
    int i, j;
    char timeNow[100];
    getTime(timeNow);
    printf("\n%s\n", timeNow);
    for (i = 0; i < nUavPulse; i++)
    {
        printf("*******************************************\n");
        printf("ID=%d,%s\n", uavPulse[i].id, UAVtypes[uavPulse[i].uavIndex].name);
        printf("cenFreq=%.1f,pulseBW=%.1f,azimuth=%.1f\n", uavPulse[i].freq[i], uavPulse[i].pulseBW, uavPulse[i].azimuth);
        printf("OnAnt:");
        for (j = 0; j < NCorr; j++)
        {
            if (uavPulse[i].onAnt[j])
                printf("%d,", j + 1);
        }
        printf("\n");
        for (j = 0; j < uavPulse[i].nPulse; j++)
        {
            printf("pulseTime=%.1f,pulseW=%2.1f,angle=%.1f,dist=%.0f\n", uavPulse[i].pulseTime[j], uavPulse[i].pulseW[j], uavPulse[i].angle[j], uavPulse[i].distance[j]);
        }
    }
    printf("*******************************************\n");
}

/*
图传脉冲检测结果写日志文件
*/
// void writeLogUavPulse(struct pulseGroup *uavPulse, int nUavPulse, struct UAVLib *UAVtypes, struct deviceParam devParam,     struct detectParams   &detectedParam)
// {
//     int        i, j, k;
//     char       timeNow[100];
//     static int num = 0;
//     float initPhase[NCh],d[NCh];
//
//
//     if (!mkdir("./log", 0777))
//         return;
//
// 	char filePath[]="./";
// 	int spaceFree=check_free_space2(FreeSpace, filePath);
// 	if(spaceFree<0)
// 	{
// 		printf("Free space on disk is smaller than %d G! Can not write log.\n", FreeSpace);
// 		return;
// 	}
//
//     FILE *fp = fopen("./log/VidPulseResult.txt", "a");
//     if (fp != NULL)
//     {
//         getTime(timeNow);
//         num++;
//
//         for (i = 0; i < nUavPulse; i++)
//         {
// 	    // selAntParamPro(initPhase, uavPulse[i].freq[0], detectedParam, d);
//             for (j = 0; j < uavPulse[i].nPulse; j++)
//             {
// 		fprintf(fp, "frameCount=%d,compassAng=%f,turntableAng=%d,%02d:%02d:%02d %0.2f,", devParam.frameCount,devParam.compassAngle,devParam.turntableAngle,devParam.gpsTime.hour, devParam.gpsTime.minute, devParam.gpsTime.seconds, devParam.gpsTime.nanoSecond/1e6);
//                 fprintf(fp, "antFace=%d,",uavPulse[i].antFace[j]);
// 		fprintf(fp, "initPhase=");
// 		for (k = 0; k < detectedParam.antennaNum; k++)
// 		{
// 		    fprintf(fp,"%.2f,",initPhase[k]);
// 		}
// 		for (k = detectedParam.antennaNum; k < 8; k++)
// 		{
// 		    fprintf(fp,"0.0,");
// 		}
// 		fprintf(fp, "d=");
// 		for (k = 0; k < detectedParam.antennaNum; k++)
// 		{
// 		    fprintf(fp,"%.3f,",d[k]);
// 		}
// 		for (k = detectedParam.antennaNum; k < 8; k++)
// 		{
// 		    fprintf(fp,"0.0,");
// 		}
// //		fprintf(fp, "initPhase=%.2f,%.2f,%.2f,%.2f,%.2f,d=%.3f,%.3f,%.3f,%.3f,%.3f,",initPhase[0],initPhase[1],initPhase[2],initPhase[3],initPhase[4],d[0],d[1],d[2],d[3],d[4]);
// 		fprintf(fp, "num=%d,%s,", num, timeNow);
//                 fprintf(fp, "ID=%d,name=%s,pr=%0.1f,", uavPulse[i].id, UAVtypes[uavPulse[i].uavIndex].name, (int)uavPulse[i].possibility + detectedParam.validSwInd / 10.0);
//                 fprintf(fp, "freq=%.1f,pulseBW=%.1f,azimuth=%.1f,range=%.1f,", uavPulse[i].freq[0], uavPulse[i].pulseBW, uavPulse[i].azimuth, uavPulse[i].range);
//                 fprintf(fp, "OnAnt:");
//                 for (k = 0; k < detectedParam.antennaNum; k++)
//                 {
//                     if (uavPulse[i].onAnt[k])
//                         fprintf(fp, "%d,", 1);
//                     else
//                         fprintf(fp, "%d,", 0);
//                 }
//                 for (k = detectedParam.antennaNum; k < 8; k++)
//                 {
//                         fprintf(fp, "%d,", 0);
//                 }
//                 fprintf(fp, "pulseTime=%.1f,pulseW=%2.1f,angle=%.1f,", uavPulse[i].pulseTime[j], uavPulse[i].pulseW[j], uavPulse[i].angle[j]);
//
// 		fprintf(fp, "meanPhase=");
// 		for (k = 0; k < detectedParam.antennaNum; k++)
// 		{
// 		    fprintf(fp,"%.2f,",uavPulse[i].meanPhase[k][j]);
// 		}
// 		for (k = detectedParam.antennaNum; k < 8; k++)
// 		{
// 		    fprintf(fp,"0.0,");
// 		}
//
// 		fprintf(fp, "meanAmp=");
// 		for (k = 0; k < detectedParam.antennaNum; k++)
// 		{
// 		    fprintf(fp,"%.1f,",uavPulse[i].meanAmp[k][j]);
// 		}
// 		for (k = detectedParam.antennaNum; k < 8; k++)
// 		{
// 		    fprintf(fp,"0.0,");
// 		}
//
// 		fprintf(fp, "phaseVar=");
// 		for (k = 0; k < detectedParam.antennaNum; k++)
// 		{
// 		    if(k == 7)
// 		    {
// 			fprintf(fp,"%.2f\n",uavPulse[i].phaseVar[k][j]);
// 			continue;
// 		    }
// 		    fprintf(fp,"%.2f,",uavPulse[i].phaseVar[k][j]);
// 		}
// 		for (k = detectedParam.antennaNum; k < 8; k++)
// 		{
// 		    if(k == 7)
// 		    {
// 			fprintf(fp,"0.0\n");
// 			continue;
// 		    }
// 		    fprintf(fp,"0.0,");
// 		}
//
// // 		fprintf(
// //                     fp, "meanPhase=%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,", uavPulse[i].meanPhase[0][j], uavPulse[i].meanPhase[1][j], uavPulse[i].meanPhase[2][j], uavPulse[i].meanPhase[3][j],
// //                     uavPulse[i].meanPhase[4][j], uavPulse[i].meanPhase[5][j], uavPulse[i].meanPhase[6][j], uavPulse[i].meanPhase[7][j]);
// //                 fprintf(fp, "meanAmp=%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,", uavPulse[i].meanAmp[0][j], uavPulse[i].meanAmp[1][j], uavPulse[i].meanAmp[2][j], uavPulse[i].meanAmp[3][j], uavPulse[i].meanAmp[4][j], uavPulse[i].meanAmp[5][j], uavPulse[i].meanAmp[6][j], uavPulse[i].meanAmp[7][j]);
// //                 fprintf(
// //                     fp, "phaseVar=%.2f,%.2f,%.2f,%.2f,%.2f\n", uavPulse[i].phaseVar[0][j], uavPulse[i].phaseVar[1][j], uavPulse[i].phaseVar[2][j], uavPulse[i].phaseVar[3][j],
// //                     uavPulse[i].phaseVar[4][j]);
//             }
//         }
//         fclose(fp);
//     }
// }

/*
打印遥控脉冲串信息
*/
void printCtrPulse(struct pulseGroup *uavCtrPulse, int nUavCtrPulse, struct UAVLib *UAVtypes)
{
    int i, j;
    char timeNow[100];
    getTime(timeNow);
    printf("\n%s\n", timeNow);
    for (i = 0; i < nUavCtrPulse; i++)
    {
        printf("*******************************************\n");
        printf("ID=%d,%s\n", uavCtrPulse[i].id, UAVtypes[uavCtrPulse[i].uavIndex].name);
        printf("pulseBW=%.1f,azimuth=%.1f\n", uavCtrPulse[i].pulseBW, uavCtrPulse[i].azimuth);
        printf("OnAnt:");
        for (j = 0; j < NCorr; j++)
        {
            if (uavCtrPulse[i].onAnt[j])
                printf("%d,", j + 1);
        }
        printf("\n");
        for (j = 0; j < uavCtrPulse[i].nPulse; j++)
        {
            printf(
                "freq=%.1f,pulseTime=%.1f,pulseW=%.2f,angle=%.1f,dist=%.0f\n", uavCtrPulse[i].freq[j], uavCtrPulse[i].pulseTime[j], uavCtrPulse[i].pulseW[j], uavCtrPulse[i].angle[j],
                uavCtrPulse[i].distance[j]);
        }
    }
    printf("*******************************************\n");
}

// /*
// 遥控脉冲检测结果写日志文件
// */
// void writeLogUavCtrPulse(struct pulseGroup *uavCtrPulse, int nUavCtrPulse, struct UAVLib *UAVtypes, struct deviceParam devParam, struct detectParams  &detectedParam)
// {
//     int        i, j, k;
//     char       timeNow[100];
//     static int num = 0;
//     float initPhase[NCh],d[NCh];
//
//     if (!mkdir("./log", 0777))
//         return;
//
// 	char filePath[]="./";
// 	int spaceFree=check_free_space2(FreeSpace, filePath);
// 	if(spaceFree<0)
// 	{
// 		printf("Free space on disk is smaller than %d G! Can not write log.\n", FreeSpace);
// 		return;
// 	}
//
//     FILE *fp = fopen("./log/CtrPulseResult.txt", "a");
//
//     if (fp != NULL)
//     {
//         getTime(timeNow);
//         num++;
//
//         for (i = 0; i < nUavCtrPulse; i++)
//         {
//             // selAntParamPro(initPhase, uavCtrPulse[i].freq[0], detectedParam, d);
// 	    for (j = 0; j < uavCtrPulse[i].nPulse; j++)
//             {
//                 fprintf(fp, "frameCount=%d,compassAng=%f,turntableAng=%d,%02d:%02d:%02d %0.2f,", devParam.frameCount,devParam.compassAngle,devParam.turntableAngle,devParam.gpsTime.hour, devParam.gpsTime.minute, devParam.gpsTime.seconds, devParam.gpsTime.nanoSecond/1e6);
//                 fprintf(fp, "antFace=%d,",uavCtrPulse[i].antFace[j]);
// 		fprintf(fp, "initPhase=");
// 		for (k = 0; k < detectedParam.antennaNum; k++)
// 		{
// 		    fprintf(fp,"%.2f,",initPhase[k]);
// 		}
// 		for (k = detectedParam.antennaNum; k < 8; k++)
// 		{
// 		    fprintf(fp,"0.0,");
// 		}
// 		fprintf(fp, "d=");
// 		for (k = 0; k < detectedParam.antennaNum; k++)
// 		{
// 		    fprintf(fp,"%.3f,",d[k]);
// 		}
// 		for (k = detectedParam.antennaNum; k < 8; k++)
// 		{
// 		    fprintf(fp,"0.0,");
// 		}
// 		//fprintf(fp, "initPhase=%.2f,%.2f,%.2f,%.2f,%.2f,d=%.3f,%.3f,%.3f,%.3f,%.3f,",initPhase[0],initPhase[1],initPhase[2],initPhase[3],initPhase[4],d[0],d[1],d[2],d[3],d[4]);
// 		fprintf(fp, "num=%d,%s,", num, timeNow);
//                 fprintf(fp, "ID=%d,name=%s,pr=%0.1f,", uavCtrPulse[i].id, UAVtypes[uavCtrPulse[i].uavIndex].name,(int) uavCtrPulse[i].possibility + detectedParam.validSwInd / 10.0);
//                 fprintf(fp, "freq=%.1f,pulseBW=%.1f,azimuth=%.1f,range=%.1f,", uavCtrPulse[i].freq[j], uavCtrPulse[i].pulseBW, uavCtrPulse[i].azimuth, uavCtrPulse[i].range);
//                 fprintf(fp, "OnAnt:");
//                 for (k = 0; k < detectedParam.antennaNum; k++)
//                 {
//                     if (uavCtrPulse[i].onAnt[k])
//                         fprintf(fp, "%d,", 1);
//                     else
//                         fprintf(fp, "%d,", 0);
//                 }
//                 for (k = detectedParam.antennaNum; k < 8; k++)
//                 {
//                         fprintf(fp, "%d,", 0);
//                 }
//                 fprintf(fp, "pulseTime=%.1f,pulseW=%2.1f,angle=%.1f,", uavCtrPulse[i].pulseTime[j], uavCtrPulse[i].pulseW[j], uavCtrPulse[i].angle[j]);
//
// 		fprintf(fp, "meanPhase=");
// 		for (k = 0; k < detectedParam.antennaNum; k++)
// 		{
// 		    fprintf(fp,"%.2f,",uavCtrPulse[i].meanPhase[k][j]);
// 		}
// 		for (k = detectedParam.antennaNum; k < 8; k++)
// 		{
// 		    fprintf(fp,"0.0,");
// 		}
//
// 		fprintf(fp, "meanAmp=");
// 		for (k = 0; k < detectedParam.antennaNum; k++)
// 		{
// 		    fprintf(fp,"%.1f,",uavCtrPulse[i].meanAmp[k][j]);
// 		}
// 		for (k = detectedParam.antennaNum; k < 8; k++)
// 		{
// 		    fprintf(fp,"0.0,");
// 		}
//
// 		fprintf(fp, "phaseVar=");
// 		for (k = 0; k < detectedParam.antennaNum; k++)
// 		{
// 		    if(k == 7)
// 		    {
// 			fprintf(fp,"%.2f\n",uavCtrPulse[i].phaseVar[k][j]);
// 			continue;
// 		    }
// 		    fprintf(fp,"%.2f,",uavCtrPulse[i].phaseVar[k][j]);
// 		}
// 		for (k = detectedParam.antennaNum; k < 8; k++)
// 		{
// 		    if(k == 7)
// 		    {
// 			fprintf(fp,"0.0\n");
// 			continue;
// 		    }
// 		    fprintf(fp,"0.0,");
// 		}
//
// // 		fprintf(
// //                     fp, "meanPhase=%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,", uavCtrPulse[i].meanPhase[0][j], uavCtrPulse[i].meanPhase[1][j], uavCtrPulse[i].meanPhase[2][j], uavCtrPulse[i].meanPhase[3][j],
// //                     uavCtrPulse[i].meanPhase[4][j], uavCtrPulse[i].meanPhase[5][j], uavCtrPulse[i].meanPhase[6][j], uavCtrPulse[i].meanPhase[7][j]);
// //                 fprintf(
// //                     fp, "meanAmp=%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,", uavCtrPulse[i].meanAmp[0][j], uavCtrPulse[i].meanAmp[1][j], uavCtrPulse[i].meanAmp[2][j], uavCtrPulse[i].meanAmp[3][j],
// //                     uavCtrPulse[i].meanAmp[4][j], uavCtrPulse[i].meanAmp[5][j], uavCtrPulse[i].meanAmp[6][j], uavCtrPulse[i].meanAmp[7][j]);
// //                 fprintf(
// //                     fp, "phaseVar=%.2f,%.2f,%.2f,%.2f,%.2f\n", uavCtrPulse[i].phaseVar[0][j], uavCtrPulse[i].phaseVar[1][j], uavCtrPulse[i].phaseVar[2][j], uavCtrPulse[i].phaseVar[3][j],
// //                     uavCtrPulse[i].phaseVar[4][j]);
//             }
//         }
//         fclose(fp);
//     }
// }
//
// /*
// 跟踪结果写文件
// */
void writeLogObj(struct message *messagePtr, struct deviceParam devParam)
{
    int i;

    static int num = 0;

    if (!mkdir("./log", 0777))
        return;

    char filePath[] = "./";
    int spaceFree = check_free_space2(FreeSpace, filePath);
    if (spaceFree < 0)
    {
        printf("Free space on disk is smaller than %f G! Can not write log.\n", FreeSpace);
        return;
    }

    FILE *fp = fopen("./log/objResult.txt", "a");

    time_t timep;
    struct tm *p;
    time(&timep);
    p = localtime(&timep);
    if (p == NULL)
    {
        if (fp != NULL)
            fclose(fp);
        return;
    }

    if (fp != NULL)
    {
        if (messagePtr->amount > 0)
            num++;
        for (i = 0; i < messagePtr->amount; i++)
        {
            fprintf(fp, "framCount=%d,", devParam.frameCount);
            fprintf(
                fp, "num=%d,%04d-%02d-%02d %02d:%02d:%02d,ID=%d,name=%s,pr=%d,", num, p->tm_year + 1900, p->tm_mon + 1, p->tm_mday, messagePtr->obj[i].wHour, messagePtr->obj[i].wMinute,
                messagePtr->obj[i].wSecond, messagePtr->obj[i].objID, messagePtr->obj[i].name, messagePtr->obj[i].possibility);
            if (messagePtr->obj[i].typ == 2)
                fprintf(fp, "vid,");
            else if (messagePtr->obj[i].typ == 1)
                fprintf(fp, "ctr,");
            fprintf(
                fp, "freq=%.1f,pulseBW=%.1f,pulseW=%.2f,pulseAmp=%.1f,azimuth=%.1f,range=%.1f,", messagePtr->obj[i].freq / 1e3, messagePtr->obj[i].pulseBW / 1e3, messagePtr->obj[i].pulseW,
                messagePtr->obj[i].pulseAmp, messagePtr->obj[i].azimuth, messagePtr->obj[i].range);
            fprintf(fp, "longitude=%.6f,latitude=%.6f,height=%d\n", messagePtr->obj[i].longitude, messagePtr->obj[i].latitude, (int)messagePtr->obj[i].height);
        }
        fclose(fp);
    }
}

/*
 * 自动标校用日志
 */
// void writeLogAutoCalibration(struct message *messagePtr,struct pulseGroup *uavPulse, int nUavPulse,struct pulseGroup *uavCtrPulse, int nUavCtrPulse, struct UAVLib *UAVtypes, struct deviceParam devParam, struct detectParams   detectedParam)
// {
// 	int        i, j, k;
// 	char       timeNow[100];
// 	static int num = 0;
// 	float initPhase[NCh],d[NCh];
// 	struct pulseGroup tmpPulse;
// 	if (!mkdir("/home/bekl/urd360/log", 0777))
// 	    return;
//
// 	char filePath[]="/home/bekl/urd360/";
// 	int spaceFree=check_free_space2(FreeSpace, filePath);
// 	if(spaceFree<0)
// 	{
// 		printf("Free space on disk is smaller than %d G! Can not write log.\n", FreeSpace);
// 		return;
// 	}
//
// 	FILE *fpCtr = fopen("/home/bekl/urd360/log/CtrPulseResult.txt", "a");
// 	static int numCtr = 0;
// 	if (fpCtr != NULL)
// 	{
//         getTime(timeNow);
//         numCtr++;
//
//         for (i = 0; i < nUavCtrPulse; i++)
//         {
//             // selAntParamPro(initPhase, uavCtrPulse[i].freq[0], detectedParam, d);
// 	    for (j = 0; j < uavCtrPulse[i].nPulse; j++)
//             {
//                 fprintf(fpCtr, "frameCount=%d,compassAng=%f,turntableAng=%d,%02d:%02d:%02d %0.2f,", devParam.frameCount,devParam.compassAngle,devParam.turntableAngle,devParam.gpsTime.hour, devParam.gpsTime.minute, devParam.gpsTime.seconds, devParam.gpsTime.nanoSecond/1e6);
//                 fprintf(fpCtr, "antFace=%d,",uavCtrPulse[i].antFace[j]);
// 		fprintf(fpCtr, "initPhase=%.2f,%.2f,%.2f,%.2f,%.2f,d=%.3f,%.3f,%.3f,%.3f,%.3f,",initPhase[0],initPhase[1],initPhase[2],initPhase[3],initPhase[4],d[0],d[1],d[2],d[3],d[4]);
// 		fprintf(fpCtr, "num=%d,%s,", numCtr, timeNow);
//                 fprintf(fpCtr, "ID=%d,name=%s,pr=%d,", uavCtrPulse[i].id, UAVtypes[uavCtrPulse[i].uavIndex].name, uavCtrPulse[i].possibility);
//                 fprintf(fpCtr, "freq=%.1f,pulseBW=%.1f,azimuth=%.1f,range=%.1f,", uavCtrPulse[i].freq[j], uavCtrPulse[i].pulseBW, uavCtrPulse[i].azimuth, uavCtrPulse[i].range);
//                 fprintf(fpCtr, "OnAnt:");
//                 for (k = 0; k < NCorr; k++)
//                 {
//                     if (uavCtrPulse[i].onAnt[k])
//                         fprintf(fpCtr, "%d,", 1);
//                     else
//                         fprintf(fpCtr, "%d,", 0);
//                 }
//                 fprintf(fpCtr, "pulseTime=%.1f,pulseW=%2.1f,angle=%.1f,", uavCtrPulse[i].pulseTime[j], uavCtrPulse[i].pulseW[j], uavCtrPulse[i].angle[j]);
//
// 		fprintf(fpCtr, "meanPhase=");
// 		for (k = 0; k < NCorr; k++)
// 		{
// 		    fprintf(fpCtr,"%.2f,",uavCtrPulse[i].meanPhase[k][j]);
// 		}
// 		fprintf(fpCtr, "meanAmp=");
// 		for (k = 0; k < NCorr; k++)
// 		{
// 		    fprintf(fpCtr,"%.1f,",uavCtrPulse[i].meanAmp[k][j]);
// 		}
// 		fprintf(fpCtr, "phaseVar=");
// 		for (k = 0; k < NCorr; k++)
// 		{
// 		    fprintf(fpCtr,"%.2f,",uavCtrPulse[i].phaseVar[k][j]);
// 		    if (k == NCorr - 1)
// 			fprintf(fpCtr,"\n");
// 		}
//             }
//         }
//         fclose(fpCtr);
// 	}
//
//
// 	FILE *fpVid = fopen("/home/bekl/urd360/log/VidPulseResult.txt", "a");
// 	static int numVid = 0;
//     if (fpVid != NULL)
//     {
//         getTime(timeNow);
//         numVid++;
//
//         for (i = 0; i < nUavPulse; i++)
//         {
// 	    // selAntParamPro(initPhase, uavPulse[i].freq[0], detectedParam, d);
//             for (j = 0; j < uavPulse[i].nPulse; j++)
//             {
// 		fprintf(fpVid, "frameCount=%d,compassAng=%f,%02d:%02d:%02d %0.2f,", devParam.frameCount,devParam.compassAngle,devParam.gpsTime.hour, devParam.gpsTime.minute, devParam.gpsTime.seconds, devParam.gpsTime.nanoSecond/1e6);
//                 fprintf(fpVid, "antFace=%d,",uavPulse[i].antFace[j]);
// 		fprintf(fpVid, "initPhase=%.2f,%.2f,%.2f,%.2f,%.2f,d=%.3f,%.3f,%.3f,%.3f,%.3f,",initPhase[0],initPhase[1],initPhase[2],initPhase[3],initPhase[4],d[0],d[1],d[2],d[3],d[4]);
// 		fprintf(fpVid, "num=%d,%s,", numVid, timeNow);
//                 fprintf(fpVid, "ID=%d,name=%s,pr=%d,", uavPulse[i].id, UAVtypes[uavPulse[i].uavIndex].name, uavPulse[i].possibility);
//                 fprintf(fpVid, "freq=%.1f,pulseBW=%.1f,azimuth=%.1f,range=%.1f,", uavPulse[i].freq[0], uavPulse[i].pulseBW, uavPulse[i].azimuth, uavPulse[i].range);
//                 fprintf(fpVid, "OnAnt:");
//                 for (k = 0; k < NCorr; k++)
//                 {
//                     if (uavPulse[i].onAnt[k])
//                         fprintf(fpVid, "%d,", 1);
//                     else
//                         fprintf(fpVid, "%d,", 0);
//                 }
//                 fprintf(fpVid, "pulseTime=%.1f,pulseW=%2.1f,angle=%.1f,", uavPulse[i].pulseTime[j], uavPulse[i].pulseW[j], uavPulse[i].angle[j]);
//
// // 		fprintf(fpVid, "meanPhase=");
// // 		for (k = 0; k < detectedParam.antennaNum; k++)
// // 		{
// // 		    fprintf(fpVid,"%.2f,",uavPulse[i].meanPhase[k][j]);
// // 		}
// // 		for (k = detectedParam.antennaNum; k < 8; k++)
// // 		{
// // 		    fprintf(fpVid,"0.0,");
// // 		}
// //
// // 		fprintf(fpVid, "meanAmp=");
// // 		for (k = 0; k < detectedParam.antennaNum; k++)
// // 		{
// // 		    fprintf(fpVid,"%.1f,",uavPulse[i].meanAmp[k][j]);
// // 		}
// // 		for (k = detectedParam.antennaNum; k < 8; k++)
// // 		{
// // 		    fprintf(fpVid,"0.0,");
// // 		}
// //
// // 		fprintf(fpVid, "phaseVar=");
// // 		for (k = 0; k < detectedParam.antennaNum; k++)
// // 		{
// // 		    if(k == 7)
// // 		    {
// // 			fprintf(fpVid,"%.2f\n",uavPulse[i].phaseVar[k][j]);
// // 			continue;
// // 		    }
// // 		    fprintf(fpVid,"%.2f,",uavPulse[i].phaseVar[k][j]);
// // 		}
// // 		for (k = detectedParam.antennaNum; k < 8; k++)
// // 		{
// // 		    if(k == 7)
// // 		    {
// // 			fprintf(fpVid,"0.0\n");
// // 			continue;
// // 		    }
// // 		    fprintf(fpVid,"0.0,");
// // 		}
//
// 		fprintf(fpVid, "meanPhase=%.2f,%.2f,%.2f,%.2f,%.2f,", uavPulse[i].meanPhase[0][j], uavPulse[i].meanPhase[1][j], uavPulse[i].meanPhase[2][j], uavPulse[i].meanPhase[3][j],uavPulse[i].meanPhase[4][j]);
//                 fprintf(fpVid, "meanAmp=%.1f,%.1f,%.1f,%.1f,%.1f,", uavPulse[i].meanAmp[0][j], uavPulse[i].meanAmp[1][j], uavPulse[i].meanAmp[2][j], uavPulse[i].meanAmp[3][j], uavPulse[i].meanAmp[4][j]);
//                 fprintf(fpVid, "phaseVar=%.2f,%.2f,%.2f,%.2f,%.2f\n", uavPulse[i].phaseVar[0][j], uavPulse[i].phaseVar[1][j], uavPulse[i].phaseVar[2][j], uavPulse[i].phaseVar[3][j],uavPulse[i].phaseVar[4][j]);
//             }
//         }
//         fclose(fpVid);
//     }
//
// 	FILE *fp = fopen("/home/bekl/urd360/log/pulseResult.txt", "a");
// 	if (fp != NULL)
// 	{
//         getTime(timeNow);
//         num++;
//
//         for (i = 0; i < nUavCtrPulse; i++)
//         {
//             // selAntParamPro(initPhase, uavCtrPulse[i].freq[0], detectedParam, d);
// 	    for (j = 0; j < uavCtrPulse[i].nPulse; j++)
//             {
//                 fprintf(fp, "frameCount=%d,compassAng=%f,%02d:%02d:%02d %0.2f,", devParam.frameCount,devParam.compassAngle,devParam.gpsTime.hour, devParam.gpsTime.minute, devParam.gpsTime.seconds, devParam.gpsTime.nanoSecond/1e6);
//                 fprintf(fp, "antFace=%d,",uavCtrPulse[i].antFace[j]);
// 		fprintf(fp, "initPhase=");
// 		for (k = 0; k < detectedParam.antennaNum; k++)
// 		{
// 		    fprintf(fp,"%.2f,",initPhase[k]);
// 		}
// 		for (k = detectedParam.antennaNum; k < 8; k++)
// 		{
// 		    fprintf(fp,"0.0,");
// 		}
// 		fprintf(fp, "d=");
// 		for (k = 0; k < detectedParam.antennaNum; k++)
// 		{
// 		    fprintf(fp,"%.3f,",d[k]);
// 		}
// 		for (k = detectedParam.antennaNum; k < 8; k++)
// 		{
// 		    fprintf(fp,"0.0,");
// 		}
// 		//fprintf(fp, "initPhase=%.2f,%.2f,%.2f,%.2f,%.2f,d=%.3f,%.3f,%.3f,%.3f,%.3f,",initPhase[0],initPhase[1],initPhase[2],initPhase[3],initPhase[4],d[0],d[1],d[2],d[3],d[4]);
// 		fprintf(fp, "num=%d,%s,", num, timeNow);
//                 fprintf(fp, "ID=%d,name=%s,pr=%d,", uavCtrPulse[i].id, UAVtypes[uavCtrPulse[i].uavIndex].name, uavCtrPulse[i].possibility);
//                 fprintf(fp, "freq=%.1f,pulseBW=%.1f,azimuth=%.1f,range=%.1f,", uavCtrPulse[i].freq[j], uavCtrPulse[i].pulseBW, uavCtrPulse[i].azimuth, uavCtrPulse[i].range);
//                 fprintf(fp, "OnAnt:");
//                 for (k = 0; k < detectedParam.antennaNum; k++)
//                 {
//                     if (uavCtrPulse[i].onAnt[k])
//                         fprintf(fp, "%d,", 1);
//                     else
//                         fprintf(fp, "%d,", 0);
//                 }
//                 for (k = detectedParam.antennaNum; k < 8; k++)
//                 {
//                         fprintf(fp, "%d,", 0);
//                 }
//                 fprintf(fp, "pulseTime=%.1f,pulseW=%2.1f,angle=%.1f,", uavCtrPulse[i].pulseTime[j], uavCtrPulse[i].pulseW[j], uavCtrPulse[i].angle[j]);
//
// 		fprintf(fp, "meanPhase=");
// 		for (k = 0; k < detectedParam.antennaNum; k++)
// 		{
// 		    fprintf(fp,"%.2f,",uavCtrPulse[i].meanPhase[k][j]);
// 		}
// 		for (k = detectedParam.antennaNum; k < 8; k++)
// 		{
// 		    fprintf(fp,"0.0,");
// 		}
//
// 		fprintf(fp, "meanAmp=");
// 		for (k = 0; k < detectedParam.antennaNum; k++)
// 		{
// 		    fprintf(fp,"%.1f,",uavCtrPulse[i].meanAmp[k][j]);
// 		}
// 		for (k = detectedParam.antennaNum; k < 8; k++)
// 		{
// 		    fprintf(fp,"0.0,");
// 		}
//
// 		fprintf(fp, "phaseVar=");
// 		for (k = 0; k < detectedParam.antennaNum; k++)
// 		{
// 		    if(k == 7)
// 		    {
// 			fprintf(fp,"%.2f\n",uavCtrPulse[i].phaseVar[k][j]);
// 			continue;
// 		    }
// 		    fprintf(fp,"%.2f,",uavCtrPulse[i].phaseVar[k][j]);
// 		}
// 		for (k = detectedParam.antennaNum; k < 8; k++)
// 		{
// 		    if(k == 7)
// 		    {
// 			fprintf(fp,"0.0\n");
// 			continue;
// 		    }
// 		    fprintf(fp,"0.0,");
// 		}
//
//         }
//         }
//         fclose(fp);
// 	}
//
//
// 	static int num1 = 0;
//     FILE *fp1 = fopen("/home/bekl/urd360/log/objResult.txt", "a");
// //
//     time_t     timep;
//     struct tm *p;
//     time(&timep);
//     p = localtime(&timep);
//     if (p == NULL)
//     {
// 		if(fp1!=NULL)
// 			fclose(fp1);
//         return;
//     }
// //
//     if (fp1 != NULL)
//     {
//         if (messagePtr->amount > 0)
//             num1++;
//         for (i = 0; i < messagePtr->amount; i++)
//         {
//             fprintf(fp1,"framCount=%d,",devParam.frameCount);
// 	    	fprintf(
//                 fp1, "num=%d,%04d-%02d-%02d %02d:%02d:%02d,ID=%d,name=%s,pr=%d,", num1, p->tm_year + 1900, p->tm_mon + 1, p->tm_mday, messagePtr->obj[i].wHour, messagePtr->obj[i].wMinute,
//                 messagePtr->obj[i].wSecond, messagePtr->obj[i].objID, messagePtr->obj[i].name, messagePtr->obj[i].possibility);
//             if (messagePtr->obj[i].typ == 2)
//                 fprintf(fp1, "vid,");
//             else if (messagePtr->obj[i].typ == 1)
//                 fprintf(fp1, "ctr,");
//             fprintf(
//                 fp1, "freq=%.1f,pulseBW=%.1f,pulseW=%.2f,pulseAmp=%.1f,azimuth=%.1f,range=%.1f,", messagePtr->obj[i].freq / 1e3, messagePtr->obj[i].pulseBW / 1e3, messagePtr->obj[i].pulseW,
//                 messagePtr->obj[i].pulseAmp, messagePtr->obj[i].azimuth, messagePtr->obj[i].range);
//             fprintf(fp1, "longitude=%.6f,latitude=%.6f,height=%d\n", messagePtr->obj[i].longitude, messagePtr->obj[i].latitude, messagePtr->obj[i].height);
//         }
//         fclose(fp1);
//     }
//
// }

// /*
// 标校信号平均幅度和相位写日志文件
// */
// void writeLogRefAmpPhase(float *refAmp, float *refPhase)
// {
//     if (!mkdir("./log", 0777))
//         return;

//     FILE *fp = fopen("./log/refAmpPhase.txt", "a");
//     if (fp != NULL)
//     {
//         fprintf(
//             fp, "refAmp=%.1f,%.1f,%.1f,%.1f,%.1f, refPhase=%.2f,%.2f,%.2f,%.2f,%.2f\n", refAmp[0], refAmp[1], refAmp[2], refAmp[3], refAmp[4], refPhase[0], refPhase[1], refPhase[2], refPhase[3],
//             refPhase[4]);
//         fclose(fp);
//     }
// }

/*
打印目标信息
*/
void printObj(struct message msg)
{
    int i;

    printf("*******************************************\n");
    for (i = 0; i < msg.amount; i++)
    {
        printf("objID=%d\n", msg.obj[i].objID);             // 目标ID编号
        printf("UAV Name=%s\n", msg.obj[i].name);           // 无人机名称
        printf("vidOrCtr=%d\n", msg.obj[i].typ);            // 类型，图传2,遥控1,其他0
        printf("freq=%.1f\n", msg.obj[i].freq / 1e3);       // 中心频率,MHz
        printf("pulseBW=%.1f\n", msg.obj[i].pulseBW / 1e3); // 带宽,MHz
        printf("pulseAmp=%f\n", msg.obj[i].pulseAmp);       // 信号强度，dB
        printf("azimuth=%.1f\n", msg.obj[i].azimuth);       // 方位角,度
        printf("range=%.1f\n", msg.obj[i].range);           // 无人机的距离,m
        printf("longitude=%f\n", msg.obj[i].longitude);     // 无人机所在的经度，度
        printf("latitude=%f\n", msg.obj[i].latitude);       // 无人机所在的纬度，度
        printf("height=%d\n", (int)msg.obj[i].height);      // 无人机的海拔高度，m
        printf("possibility=%f\n", (float)msg.obj[i].possibility);
        printf("detected time, H:%d,M:%d,S:%d,mS:%d\n", msg.obj[i].wHour, msg.obj[i].wMinute, msg.obj[i].wSecond, msg.obj[i].wMilliseconds);
        printf("duration time,%.0f seconds\n", floor(msg.obj[i].objDuration / 1e6 + 0.5));
        //        printf("mrm=%d\n", msg.obj[i].mode);

        printf("*******************************************\n");
    }
}

/*
打印目标信息
*/
void printObjLog(struct message msg)
{
    int i;
    FILE *fp = fopen("./log/deblurLog.txt", "a+");
    fprintf(fp, "*******************************************\n");
    for (i = 0; i < msg.amount; i++)
    {
        fprintf(fp, "objID=%d\n", msg.obj[i].objID);             // 目标ID编号
        fprintf(fp, "UAV Name=%s\n", msg.obj[i].name);           // 无人机名称
        fprintf(fp, "vidOrCtr=%d\n", msg.obj[i].typ);            // 类型，图传2,遥控1,其他0
        fprintf(fp, "freq=%.1f\n", msg.obj[i].freq / 1e3);       // 中心频率,MHz
        fprintf(fp, "pulseBW=%.1f\n", msg.obj[i].pulseBW / 1e3); // 带宽,MHz
        fprintf(fp, "pulseAmp=%f\n", msg.obj[i].pulseAmp);       // 信号强度，dB
        fprintf(fp, "azimuth=%.1f\n", msg.obj[i].azimuth);       // 方位角,度
        fprintf(fp, "range=%.1f\n", msg.obj[i].range);           // 无人机的距离,m
        fprintf(fp, "longitude=%f\n", msg.obj[i].longitude);     // 无人机所在的经度，度
        fprintf(fp, "latitude=%f\n", msg.obj[i].latitude);       // 无人机所在的纬度，度
        fprintf(fp, "height=%d\n", msg.obj[i].height);           // 无人机的海拔高度，m
        fprintf(fp, "detected time, H:%d,M:%d,S:%d,mS:%d\n", msg.obj[i].wHour, msg.obj[i].wMinute, msg.obj[i].wSecond, msg.obj[i].wMilliseconds);
        fprintf(fp, "duration time,%.0f seconds\n", floor(msg.obj[i].objDuration / 1e6 + 0.5));

        fprintf(fp, "*******************************************\n");
    }
    fclose(fp);
}

/*
 * 计算当前时间和日期的相差秒数
 */
int getDiffTime(struct tm *p, struct dateTime t)
{
    int diffTime = 0;
    diffTime += (p->tm_year + 1900 - t.year) * 365 * 12 * 24 * 60 * 60;
    diffTime += (p->tm_mon + 1 - t.month) * 12 * 24 * 60 * 60;
    diffTime += (p->tm_mday - t.day) * 24 * 60 * 60;
    diffTime += (p->tm_hour - t.hour) * 60 * 60;
    diffTime += (p->tm_min - t.minute) * 60;
    diffTime += p->tm_sec - t.seconds;
    return diffTime;
}

/*
 * 计算两个日期的相差秒数
 */
int getDiffDateTime(struct dateTime t1, struct dateTime t2)
{
    int diffTime = 0;
    diffTime += (t1.year - t2.year) * 365 * 12 * 24 * 60 * 60;
    diffTime += (t1.month - t2.month) * 12 * 24 * 60 * 60;
    diffTime += (t1.day - t2.day) * 24 * 60 * 60;
    diffTime += (t1.hour - t2.hour) * 60 * 60;
    diffTime += (t1.minute - t2.minute) * 60;
    diffTime += t1.seconds - t2.seconds;
    return diffTime;
}

/*
 * 给定当前时间到日期
 */
void setDateTime(struct tm *p, struct dateTime *t)
{
    t->year = p->tm_year + 1900;
    t->month = p->tm_mon + 1;
    t->day = p->tm_mday;
    t->hour = p->tm_hour;
    t->minute = p->tm_min;
    t->seconds = p->tm_sec;
    t->nanoSecond = 0;
}

/*
 * 根据目标每次检测的置信度和出现的次数以及时间，估计该目标的总体置信度
 */
char evalPr(struct objInfo obj)
{
    time_t timep;
    struct tm *p;
    time(&timep);
    p = localtime(&timep);

    float pr = 0.0;
    // int diffTime;
    // float weight = 0.0;
    // for (int i = 0; i < obj.num; i++)
    // {
    //     diffTime = getDiffTime(p, obj.detectedTime[i]); //距离首次检出的时间
    //     weight = pow(0.9, diffTime + 1.0 + 0.5 * i);
    //     pr += weight * obj.possibility[i] / 100.0;
    // }
    // pr /= diffTime + 2.0 + obj.num * 0.5;
    // pr = pow(pr, 1.0 / obj.num);
    // if (pr > 1.0)
    pr = 1.0;
    char result = floor(pr * 100);
    return result;
}

/*
 * 从脉冲时间，寻找最佳的起始周期索引
 */
int findIndexPulseT(float *pulseTime, int n, float *pulseT, int nPulseT)
{
    int indexPulseT = 0, nt = 0;
    float diffTime;
    float sumPulseT = 0.0, sumTimeErr = 0.0, timeErr = 0.0, tErr;
    float minSumTimeErr = 1000;
    for (int i = 0; i < nPulseT; i++)
        sumPulseT += pulseT[i];

    for (int i = 0; i < nPulseT; i++)
    {
        sumTimeErr = 0.0;
        for (int j = 1; j < n; j++)
        {
            diffTime = pulseTime[j] - pulseTime[0];
            diffTime = fmod(diffTime, sumPulseT);
            nt = 0;
            timeErr = 1000;
            for (int k = 0; k < 1000; k++)
            {
                tErr = fabs(diffTime);
                if (tErr < timeErr)
                    timeErr = tErr;
                if (diffTime < 0)
                    break;
                diffTime -= pulseT[(i + nt) % nPulseT];
                nt++;
            }
            sumTimeErr += timeErr;
        }
        if (sumTimeErr < minSumTimeErr)
        {
            minSumTimeErr = sumTimeErr;
            indexPulseT = i;
        }
    }
    return indexPulseT;
}

/*
 * 以某个脉冲时间为基准，两组脉冲时间周期匹配
 */
int isPulseTMatch(int pulseTIndex, float pulseTime1_0, float *pulseTime2, int n2, float *pulseT, int nPulseT, float pulseTErr, float *sumTErr)
{
    float sumPulseT = 0.0, diffTime, tErr;
    int nt = 0, count = 0;
    int minCount = 0;
    int sumTErr1 = *sumTErr;
    sumTErr1 = 0.0;

    for (int i = 0; i < nPulseT; i++)
        sumPulseT += pulseT[i];

    for (int i = 0; i < n2; i++)
    {
        diffTime = pulseTime2[i] - pulseTime1_0;
        diffTime = fmod(diffTime, sumPulseT);
        nt = 0;
        for (int k = 0; k < 1000; k++)
        {
            tErr = fabs(diffTime);
            if (tErr < pulseTErr)
            {
                sumTErr1 += tErr;
                count++;
                break;
            }
            if (diffTime < 0)
                break;
            diffTime -= pulseT[(nt + pulseTIndex) % nPulseT];
            nt++;
        }
    }
    if (sumTErr1 == 0 && count == 0)
        return false;
    else
    {
        sumTErr1 = sumTErr1 / count;
    }

    *sumTErr = sumTErr1;
    if (n2 < 3)
        minCount = 1;
    else
        minCount = n2 / 2;

    if (count >= minCount)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/*
 * 根据两次帧计数，计算相差的时间ms
 */
float calcDiffTime(unsigned int lastFrameCount, unsigned int currentFrameCount)
{
    float diffFrameTime = -1;
    for (int i = 0; i < 2000; i++)
    {
        if (fabs(lastFrameCount + 9375 * (i + 1) - currentFrameCount) < 2)
        {
            diffFrameTime = (i + 1) * 500;
            break;
        }
    }

    return diffFrameTime;
}

// 时间计数转化为毫秒
float timeCount2ms(unsigned int timeCount, char timeCountVersion)
{
    float t = 0.0; // ms
    unsigned int lastTimeCount = timeCount;
    if (timeCountVersion == 1)
    {
        t = timeCount / 18.750;
        if (lastTimeCount > timeCount)
        {
            t += 229064922.4;
        }
    }
    else if (timeCountVersion == 2)
    {
        t = timeCount / 100.0;
        if (lastTimeCount > timeCount)
        {
            t += 42949672.95;
        }
    }
    lastTimeCount = timeCount;

    return t;
}

/*
 * 对于有周期的信号，根据帧计数，对上次的脉冲时间和本次脉冲时间进行周期匹配
 */
int pulseTRelated(float lastCountTime, float *lastPulseTime, int n1, float currentCountTime, float *currentPulseTime, int n2, struct UAVLib uavType, float *TErr)
{
    int TErr1 = *TErr;
    int IndexPulseT = 0;
    int count = 0;
    float diffFrameTime = -1;
    float *pulseT = uavType.pulseT;
    float pulseTErr = uavType.pulseTErr;
    int nPulseT = uavType.nPulseT;

    float tmpTErr = 0.0;
    diffFrameTime = currentCountTime - lastCountTime;

    if (diffFrameTime < 0)
        return false;

    /*
    printf("pulseTime1:");
    for(int i=0;i<n1;i++)
    {
        printf("%.1f,",lastPulseTime[i]);
    }
    printf("\n");

    printf("pulseTime2:");
    for(int i=0;i<n2;i++)
    {
        printf("%.1f,",currentPulseTime[i]);
    }
    printf("\n");
    */
    TErr1 = 0.0;
    for (int i = 0; i < n1; i++)
    {
        IndexPulseT = findIndexPulseT(lastPulseTime + i, n1 - i, pulseT, nPulseT);
        //        printf("i =%d,n1 =%d\n",i,n1);
        if (isPulseTMatch(IndexPulseT, lastPulseTime[i] - diffFrameTime, currentPulseTime, n2, pulseT, nPulseT, pulseTErr, &tmpTErr))
        {
            TErr1 += tmpTErr;
            count++;
        }
    }
    if (count > 0)
    {
        TErr1 = TErr1 / (count * count);
        *TErr = TErr1;
        return true;
    }
    else
    {
        TErr1 = 10000;
        *TErr = TErr1;
        return false;
    }
}

int checkRelated(int nObj, int nPulse, int *matchedIndex, int *isRelated, float *sumTErr)
{
    float minSumTErr = 10000;
    int minIndex = -1;
    for (int i = 0; i < nObj; i++)
    {
        minSumTErr = 10000;
        minIndex = -1;
        for (int j = 0; j < nPulse; j++)
        {
            if (isRelated[j] == 0)
                continue;
            if (matchedIndex[j] == i)
            {
                if (minSumTErr > sumTErr[j])
                {
                    if (minIndex >= 0)
                        isRelated[minIndex] = 0;
                    minSumTErr = sumTErr[j];
                    minIndex = j;
                }
            }
        }
    }
    return 0;
}

int updateRecordedObj(float countTime, unsigned int frameCount, struct objInfo *recordedObj, int *nObj, struct pulseGroup onePulse, int pulseIndex, struct detectParams detectedParam, int matchedIndex, int *isRelated, int sIndex, int index, float *sumTErr)
{

    int nObj1 = *nObj;
    float minFreq = onePulse.freq[0], maxFreq = onePulse.freq[0];
    static int objID = 2;
    float angleVar = 0, x = 0;
    int angleCount = 0, maxAmpInd = 0;

    time_t timep;
    struct tm *p;
    time(&timep);
    p = localtime(&timep);

    for (int i = 0; i < onePulse.nPulse; i++)
    {
        x = fabs(onePulse.azimuth - onePulse.angle[i]);
        if (x > 180)
            x = 360 - 180;
        angleVar += x * x;
        angleCount++;
    }
    angleVar /= angleCount;

    // 匹配成功
    if (isRelated[pulseIndex] == 1)
    {
        // 更新目标信息
        setDateTime(p, &recordedObj[matchedIndex].detectedTime[index + 1]);

        recordedObj[matchedIndex].countTime = countTime;
        recordedObj[matchedIndex].frameCount = frameCount;
        for (int i = 0; i < onePulse.nPulse; i++)
        {
            if (onePulse.freq[i] < minFreq)
                minFreq = onePulse.freq[i];
            if (onePulse.freq[i] > maxFreq)
                maxFreq = onePulse.freq[i];
        }
        //        printf("------    minFreq = %f,maxFreq = %f\n", minFreq, maxFreq);
        if (detectedParam.bwFreqType == 0)
        {
            recordedObj[matchedIndex].freq[index + 1] = onePulse.freq[0];
            recordedObj[matchedIndex].pulseBW[index + 1] = onePulse.pulseBW;
        }
        else
        {
            recordedObj[matchedIndex].freq[index + 1] = (maxFreq - minFreq) / 2 + minFreq;
            recordedObj[matchedIndex].pulseBW[index + 1] = maxFreq - minFreq + onePulse.pulseBW;
        }
        recordedObj[matchedIndex].azimuth[index + 1] = onePulse.azimuth;
        recordedObj[matchedIndex].range[index + 1] = onePulse.range;
        recordedObj[matchedIndex].pitchAngle[index + 1] = angleVar;
        // recordedObj[matchedIndex].pulseBW[index + 1] = onePulse.pulseBW;
        recordedObj[matchedIndex].pulseBW[index + 1] = maxFreq - minFreq + onePulse.pulseBW;
        recordedObj[matchedIndex].pulseAmp[index + 1] = onePulse.meanAmp[maxAmpInd][0];
        for (int j = 0; j < MaxPulseTimeNum; j++)
        {
            recordedObj[matchedIndex].pulseTime[index + 1][j] = -1;
            recordedObj[matchedIndex].pulseW[index + 1][j] = -1;
        }
        for (int j = 0; j < onePulse.nPulse; j++)
        {
            if (j < MaxPulseTimeNum)
            {
                recordedObj[matchedIndex].pulseTime[index + 1][j] = onePulse.pulseTime[j];
                recordedObj[matchedIndex].pulseW[index + 1][j] = onePulse.pulseW[j];
            }
            else
                break;
        }
        recordedObj[matchedIndex].possibility[index + 1] = onePulse.possibility;

        recordedObj[matchedIndex].isUpdated = true;
        recordedObj[matchedIndex].num++;

        // if (recordedObj[matchedIndex].count < MaxHistoryNum)
        //{
        recordedObj[matchedIndex].count++;
        //}
        if (recordedObj[matchedIndex].num == MaxHistoryNum) // 若历史记录数量存满，则往后移动一次
        {
            for (int j = 0; j < recordedObj[matchedIndex].num - 1; j++)
            {
                recordedObj[matchedIndex].detectedTime[j] = recordedObj[matchedIndex].detectedTime[j + 1];
                recordedObj[matchedIndex].freq[j] = recordedObj[matchedIndex].freq[j + 1];
                recordedObj[matchedIndex].azimuth[j] = recordedObj[matchedIndex].azimuth[j + 1];
                recordedObj[matchedIndex].range[j] = recordedObj[matchedIndex].range[j + 1];
                recordedObj[matchedIndex].pitchAngle[j] = recordedObj[matchedIndex].pitchAngle[j + 1];
                recordedObj[matchedIndex].pulseBW[j] = recordedObj[matchedIndex].pulseBW[j + 1];
                recordedObj[matchedIndex].pulseAmp[j] = recordedObj[matchedIndex].pulseAmp[j + 1];
                for (int k = 0; k < MaxPulseTimeNum; k++)
                {
                    recordedObj[matchedIndex].pulseTime[j][k] = recordedObj[matchedIndex].pulseTime[j + 1][k];
                    recordedObj[matchedIndex].pulseW[j][k] = recordedObj[matchedIndex].pulseW[j + 1][k];
                }
                recordedObj[matchedIndex].possibility[j] = recordedObj[matchedIndex].possibility[j + 1];
            }
            recordedObj[matchedIndex].num--;
        }
        recordedObj[matchedIndex].sourceIndex = sIndex;
        recordedObj[matchedIndex].pulseIndex = pulseIndex;
    }
    else // 没有匹配成功的目标新加入
    {
        if (nObj1 < MaxObjNum)
        {
            recordedObj[nObj1].countTime = countTime;
            recordedObj[nObj1].frameCount = frameCount;
            setDateTime(p, &recordedObj[nObj1].firstDetectedTime);
            setDateTime(p, &recordedObj[nObj1].detectedTime[0]);

            recordedObj[nObj1].objID = objID;
            objID++;

            for (int i = 0; i < onePulse.nPulse; i++)
            {
                if (onePulse.freq[i] < minFreq)
                    minFreq = onePulse.freq[i];
                if (onePulse.freq[i] > maxFreq)
                    maxFreq = onePulse.freq[i];
            }

            recordedObj[nObj1].uavIndex = onePulse.uavIndex;
            if (detectedParam.bwFreqType == 0)
            {
                recordedObj[nObj1].freq[0] = onePulse.freq[0];
                recordedObj[nObj1].pulseBW[0] = onePulse.pulseBW;
            }
            else
            {
                recordedObj[nObj1].freq[0] = (maxFreq - minFreq) / 2 + minFreq;
                recordedObj[nObj1].pulseBW[0] = maxFreq - minFreq + onePulse.pulseBW;
            }
            recordedObj[nObj1].azimuth[0] = onePulse.azimuth;
            recordedObj[nObj1].range[0] = onePulse.range;
            recordedObj[nObj1].pitchAngle[0] = angleVar;
            // recordedObj[nObj].pulseBW[0] = onePulse.pulseBW;
            // recordedObj[nObj].pulseBW[index + 1] = maxFreq - minFreq + onePulse.pulseBW;
            recordedObj[nObj1].pulseAmp[0] = onePulse.meanAmp[maxAmpInd][0];
            recordedObj[nObj1].isUpdated = true;
            for (int j = 0; j < MaxPulseTimeNum; j++)
            {
                recordedObj[nObj1].pulseTime[0][j] = -1;
                recordedObj[nObj1].pulseW[0][j] = -1;
            }

            for (int j = 0; j < onePulse.nPulse; j++)
            {
                if (j < MaxPulseTimeNum)
                {
                    recordedObj[nObj1].pulseTime[0][j] = onePulse.pulseTime[j];
                    recordedObj[nObj1].pulseW[0][j] = onePulse.pulseW[j];
                }
                else
                    break;
            }
            recordedObj[nObj1].possibility[0] = onePulse.possibility;

            recordedObj[nObj1].num = 1;
            recordedObj[nObj1].count = 1;
            recordedObj[nObj1].sourceIndex = sIndex;
            recordedObj[nObj1].pulseIndex = pulseIndex;

            nObj1++;
        }
    }
    *nObj = nObj1;
    return 0;
}

/*
 * 无人机目标关联
 */
int objRelated(float countTime, unsigned int frameCount, struct objInfo *recordedObj, int *nObj, struct pulseGroup onePulse, int pulseIndex, struct UAVLib *UAVtypes, struct detectParams detectedParam, int *matchedIndex, int *isRelated, int *selIndex, float *sumTErr)
{
    float sumTErr1 = *sumTErr;
    int selIndex1 = *selIndex;
    int isRelated1 = *isRelated;
    int matchedIndex1 = *matchedIndex;
    int nObj1 = *nObj;
    isRelated1 = 0.0;
    int index = 0;
    int uavIndex = onePulse.uavIndex;
    //     static int objID = 2;
    float pulseEndTime1[MaxPulseTimeNum], pulseEndTime2[MaxPulseTimeNum];
    int nTime1, nTime2;

    float maxUavSpeed = detectedParam.maxUavSpeed; // 无人机最大速度
    float maxCorrTime = detectedParam.maxCorrTime; // 最大关联时间
    float timeOut = detectedParam.timeOut;         // 目标过期时间（s），过期则清除
    float freqErr = detectedParam.freqErr;         // 中心频率误差

    time_t timep;
    struct tm *p;
    time(&timep);
    p = localtime(&timep);

    int diffTime;
    float angleErr, tmpMaxAmp = onePulse.meanAmp[0][0];
    float angleVar = 0, x = 0, tmpTErr = 1000;
    int angleCount = 0, maxAmpInd = 0;
    for (int i = 0; i < onePulse.nPulse; i++)
    {
        x = fabs(onePulse.azimuth - onePulse.angle[i]);
        if (x > 180)
            x = 360 - 180;
        angleVar += x * x;
        angleCount++;
    }
    angleVar /= angleCount;
    for (int i = 0; i < detectedParam.antennaNum; i++)
    {
        if (onePulse.meanAmp[i][0] > tmpMaxAmp)
        {
            tmpMaxAmp = onePulse.meanAmp[i][0];
            maxAmpInd = i;
        }
    }
    if (UAVtypes[uavIndex].useMethod == 1)
    {
        angleErr = detectedParam.vidAngleErr;
    }
    else
    {
        angleErr = detectedParam.ctrAngleErr;
    }

    sumTErr1 = 1000;
    // 与记录的目标进行匹配
    for (int i = 0; i < nObj1; i++)
    {
        index = recordedObj[i].num - 1;
        if (recordedObj[i].uavIndex == uavIndex) // 无人机型号是否一致
        {
            if (UAVtypes[uavIndex].nPulseT > 0 && UAVtypes[uavIndex].pulseT[0] > 0 & 0) // 无人机脉冲有周期，根据周期关联
            {
                if (UAVtypes[uavIndex].useMethod == 1)
                {
                    //
                    for (int j = 0; j < MaxPulseTimeNum; j++)
                    {
                        if (recordedObj[i].pulseTime[index][j] >= 0)
                        {

                            pulseEndTime1[j] = recordedObj[i].pulseTime[index][j] + recordedObj[i].pulseW[index][j];
                            nTime1 = j + 1;
                        }
                        if (j < onePulse.nPulse)
                        {
                            pulseEndTime2[j] = onePulse.pulseTime[j] + onePulse.pulseW[j];
                            nTime2 = j + 1;
                        }
                    }
                }
                else
                {
                    for (int j = 0; j < MaxPulseTimeNum; j++)
                    {
                        if (recordedObj[i].pulseTime[index][j] >= 0)
                        {
                            pulseEndTime1[j] = recordedObj[i].pulseTime[index][j];
                            nTime1 = j + 1;
                        }
                        if (j < onePulse.nPulse)
                        {
                            pulseEndTime2[j] = onePulse.pulseTime[j];
                            nTime2 = j + 1;
                        }
                    }
                }
                if (pulseTRelated(recordedObj[i].countTime, pulseEndTime1, nTime1, countTime, pulseEndTime2, nTime2, UAVtypes[uavIndex], &tmpTErr))
                {
                    if (tmpTErr < sumTErr1)
                    {
                        sumTErr1 = tmpTErr;
                        isRelated1 = 1;
                        matchedIndex1 = i;
                        selIndex1 = index;
                    }
                }
            }
            else // 无周期的，根据中心频率进行关联
            {
                //                printf("UAVtypes[uavIndex].isFixedFreq = %d, %f\n",UAVtypes[uavIndex].isFixedFreq,fabs(recordedObj[i].freq[index] - onePulse.freq[0]));
                if (!UAVtypes[uavIndex].isFixedFreq || (UAVtypes[uavIndex].isFixedFreq && fabs(recordedObj[i].freq[index] - onePulse.freq[0]) < freqErr)) // 若是固定频点的无人机，判断频点是否一致
                {
                    // 关联时间条件
                    diffTime = getDiffTime(p, recordedObj[i].detectedTime[index]);
                    //                    diffTime = 1;
                    //                    printf("diffTime = %d, %f\n",diffTime,maxCorrTime);
                    if (diffTime > maxCorrTime)
                        continue;

                    if (fabs(onePulse.azimuth - recordedObj[i].azimuth[index]) < angleErr)
                    {
                        selIndex1 = index;
                        isRelated1 = 1;
                        matchedIndex1 = i;
                        *sumTErr = sumTErr1;
                        *selIndex = selIndex1;
                        *isRelated = isRelated1;
                        *matchedIndex = matchedIndex1;
                        *nObj = nObj1;
                        //                        printf("selIndex = %d,%d,%d\n",selIndex1,isRelated1,matchedIndex1);
                        break;
                    }
                }
            }
        }
        *sumTErr = sumTErr1;
        *selIndex = selIndex1;
        *isRelated = isRelated1;
        *matchedIndex = matchedIndex1;
        *nObj = nObj1;
        //        printf("selIndex11111 = %d,%d,%d\n",*selIndex,*isRelated,*matchedIndex);
    }

    //     float minFreq = onePulse.freq[0],maxFreq = onePulse.freq[0];
    //     //匹配成功
    //     if (isRelated)
    //     {
    //         //更新目标信息
    //         setDateTime(p, &recordedObj[matchedIndex].detectedTime[index + 1]);
    //         recordedObj[matchedIndex].countTime = countTime;
    // 	recordedObj[matchedIndex].frameCount = frameCount;
    // 	for (int i = 0; i < onePulse.nPulse; i++)
    // 	{
    // 	    if(onePulse.freq[i] < minFreq)
    // 		minFreq = onePulse.freq[i];
    // 	    if(onePulse.freq[i] > maxFreq)
    // 		maxFreq = onePulse.freq[i];
    // 	}
    // 	printf("------    minFreq = %f,maxFreq = %f\n",minFreq,maxFreq);
    // 	if(detectedParam.bwFreqType == 0)
    // 	{
    // 	    recordedObj[matchedIndex].freq[index + 1] = onePulse.freq[0];
    // 	    recordedObj[matchedIndex].pulseBW[index + 1] = onePulse.pulseBW;
    // 	}
    // 	else
    // 	{
    // 	    recordedObj[matchedIndex].freq[index + 1] = (maxFreq - minFreq)/2 + minFreq;
    // 	    recordedObj[matchedIndex].pulseBW[index + 1] = maxFreq - minFreq + onePulse.pulseBW;
    // 	}
    //         recordedObj[matchedIndex].azimuth[index + 1] = onePulse.azimuth;
    //         recordedObj[matchedIndex].range[index + 1] = onePulse.range;
    //         recordedObj[matchedIndex].pitchAngle[index + 1] = angleVar;
    //         //recordedObj[matchedIndex].pulseBW[index + 1] = onePulse.pulseBW;
    // 	    recordedObj[matchedIndex].pulseBW[index + 1] = maxFreq - minFreq + onePulse.pulseBW;
    //         recordedObj[matchedIndex].pulseAmp[index + 1] = onePulse.meanAmp[maxAmpInd][0];
    //         for (int j = 0; j < MaxPulseTimeNum; j++)
    //         {
    //             recordedObj[matchedIndex].pulseTime[index + 1][j] = -1;
    //             recordedObj[matchedIndex].pulseW[index + 1][j] = -1;
    //         }
    //         for (int j = 0; j < onePulse.nPulse; j++)
    //         {
    //             if (j < MaxPulseTimeNum)
    //             {
    //                 recordedObj[matchedIndex].pulseTime[index + 1][j] = onePulse.pulseTime[j];
    //                 recordedObj[matchedIndex].pulseW[index + 1][j] = onePulse.pulseW[j];
    //             }
    //             else
    //                 break;
    //         }
    //         recordedObj[matchedIndex].possibility[index + 1] = onePulse.possibility;
    //
    //         recordedObj[matchedIndex].isUpdated = true;
    //         recordedObj[matchedIndex].num++;
    //
    //         //if (recordedObj[matchedIndex].count < MaxHistoryNum)
    //         //{
    //         recordedObj[matchedIndex].count++;
    //         //}
    //         if (recordedObj[matchedIndex].num == MaxHistoryNum)  //若历史记录数量存满，则往后移动一次
    //         {
    //             for (int j = 0; j < recordedObj[matchedIndex].num - 1; j++)
    //             {
    // 				recordedObj[matchedIndex].detectedTime[j] = recordedObj[matchedIndex].detectedTime[j + 1];
    //                 recordedObj[matchedIndex].freq[j] = recordedObj[matchedIndex].freq[j + 1];
    //                 recordedObj[matchedIndex].azimuth[j] = recordedObj[matchedIndex].azimuth[j + 1];
    //                 recordedObj[matchedIndex].range[j] = recordedObj[matchedIndex].range[j + 1];
    //                 recordedObj[matchedIndex].pitchAngle[j] = recordedObj[matchedIndex].pitchAngle[j + 1];
    //                 recordedObj[matchedIndex].pulseBW[j] = recordedObj[matchedIndex].pulseBW[j + 1];
    //                 recordedObj[matchedIndex].pulseAmp[j] = recordedObj[matchedIndex].pulseAmp[j + 1];
    //                 for (int k = 0; k < MaxPulseTimeNum; k++)
    //                 {
    //                     recordedObj[matchedIndex].pulseTime[j][k] = recordedObj[matchedIndex].pulseTime[j + 1][k];
    //                     recordedObj[matchedIndex].pulseW[j][k] = recordedObj[matchedIndex].pulseW[j + 1][k];
    //                 }
    //                 recordedObj[matchedIndex].possibility[j] = recordedObj[matchedIndex].possibility[j + 1];
    //             }
    //             recordedObj[matchedIndex].num--;
    //         }
    //         recordedObj[matchedIndex].sourceIndex = sIndex;
    //         recordedObj[matchedIndex].pulseIndex = pulseIndex;
    //     }
    //     else  //没有匹配成功的目标新加入
    //     {
    //         if (nObj < MaxObjNum)
    //         {
    //             recordedObj[nObj].countTime = countTime;
    // 			recordedObj[nObj].frameCount=frameCount;
    //             setDateTime(p, &recordedObj[nObj].firstDetectedTime);
    //             setDateTime(p, &recordedObj[nObj].detectedTime[0]);
    //
    //             recordedObj[nObj].objID = objID;
    //             objID++;
    //
    // 	    for (int i = 0; i < onePulse.nPulse; i++)
    // 	    {
    // 		if(onePulse.freq[i] < minFreq)
    // 		    minFreq = onePulse.freq[i];
    // 		if(onePulse.freq[i] > maxFreq)
    // 		    maxFreq = onePulse.freq[i];
    // 	    }
    //
    //             recordedObj[nObj].uavIndex = onePulse.uavIndex;
    // 	    if(detectedParam.bwFreqType == 0)
    // 	    {
    // 		recordedObj[nObj].freq[0] = onePulse.freq[0];
    // 		recordedObj[nObj].pulseBW[0] = onePulse.pulseBW;
    // 	    }
    // 	    else
    // 	    {
    // 		recordedObj[nObj].freq[0] = (maxFreq - minFreq)/2 + minFreq;
    // 		recordedObj[nObj].pulseBW[0] = maxFreq - minFreq + onePulse.pulseBW;
    // 	    }
    //             recordedObj[nObj].azimuth[0] = onePulse.azimuth;
    //             recordedObj[nObj].range[0] = onePulse.range;
    //             recordedObj[nObj].pitchAngle[0] = angleVar;
    //             //recordedObj[nObj].pulseBW[0] = onePulse.pulseBW;
    // 	    // recordedObj[nObj].pulseBW[index + 1] = maxFreq - minFreq + onePulse.pulseBW;
    //             recordedObj[nObj].pulseAmp[0] = onePulse.meanAmp[maxAmpInd][0];
    //             recordedObj[nObj].isUpdated = true;
    //             for (int j = 0; j < MaxPulseTimeNum; j++)
    //             {
    //                 recordedObj[nObj].pulseTime[0][j] = -1;
    //                 recordedObj[nObj].pulseW[0][j] = -1;
    //             }
    //
    //             for (int j = 0; j < onePulse.nPulse; j++)
    //             {
    //                 if (j < MaxPulseTimeNum)
    //                 {
    //                     recordedObj[nObj].pulseTime[0][j] = onePulse.pulseTime[j];
    //                     recordedObj[nObj].pulseW[0][j] = onePulse.pulseW[j];
    //                 }
    //                 else
    //                     break;
    //             }
    //             recordedObj[nObj].possibility[0] = onePulse.possibility;
    //
    //             recordedObj[nObj].num = 1;
    //             recordedObj[nObj].count = 1;
    //             recordedObj[nObj].sourceIndex = sIndex;
    //             recordedObj[nObj].pulseIndex = pulseIndex;
    //
    //             nObj++;
    //         }
    //     }
}

void mean_std(float *Real, float *Imag, int vec_len, int *mask, float *mean_Real, float *mean_Imag, float *std_)
{
    int valid_vec_len = 0;
    // 计算平均复向量
    for (int i = 0; i < vec_len; i++)
    {
        if (mask[i])
        {
            *mean_Real += Real[i];
            *mean_Imag += Imag[i];
            valid_vec_len++;
        }
    }
    *mean_Real /= valid_vec_len;
    *mean_Imag /= valid_vec_len;

    // 计算平均复向量
    float sum_diff_from_mean = 0.0;
    for (int i = 0; i < vec_len; i++)
    {
        if (mask[i])
        {
            sum_diff_from_mean += pow((Real[i] - *mean_Real), 2) + pow((Imag[i] - *mean_Imag), 2);
        }
    }
    sum_diff_from_mean /= (valid_vec_len - 1);
    *std_ = sqrt(sum_diff_from_mean);
}

// void robust_stat(float *Real, float *Imag, int vec_len, float *mean_Real, float *mean_Imag)
// {
//     int iter_steps = 5;
//     float std_ = 0.0;
//     float outlier_threshold_relative_std_ = 2.0;
//     float outlier_threshold = 0.0;
//     float meanR = 0, meanI = 0;

//     float *diff_from_mean = (float *)malloc(sizeof(float) * vec_len);
//     int *mask = (int *)malloc(sizeof(int) * vec_len);

//     for (int i = 0; i < vec_len; i++)
//     {
//         mask[i] = true;
//         diff_from_mean[i] = 0.0;
//     }
//     mean_std(Real, Imag, vec_len, mask, &meanR, &meanI, &std_);
//     // printf("---mean_Real = %f, mean_Imag = %f\n",*mean_Real,*mean_Imag);
//     int count = 0;
//     for (int iter = 0; iter < iter_steps; iter++)
//     {
//         outlier_threshold = outlier_threshold_relative_std_ * std_;
//         // 查找野值点
//         count = 0;
//         for (int i = 0; i < vec_len; i++)
//         {
//             diff_from_mean[i] = sqrt(pow((Real[i] - meanR), 2) + pow((Imag[i] - meanI), 2));
//             if (diff_from_mean[i] > outlier_threshold)
//             {
//                 mask[i] = false;
//             }
//             else
//             {
//                 mask[i] = true;
//                 count++;
//             }
//         }
//         if (count == 0)
//             printf("\n");
//         mean_std(Real, Imag, vec_len, mask, &meanR, &meanI, &std_);
//         // 	printf("---mean_Real = %f, mean_Imag = %f,iter_steps = %d,count = %d\n",meanR,meanI,iter,count);
//     }
//     *mean_Real = meanR;
//     *mean_Imag = meanI;
//     free(diff_from_mean);
//     free(mask);
// }

/*
 * 距离均值滤波
 */
float meanFiltRange(float *x, int n, int nFit)
{
    float sumX = 0.0;
    float count = 0.0;
    for (int i = 0; i < nFit; i++)
    {
        if (n - i - 1 >= 0)
        {
            sumX += x[n - i - 1];
            count++;
        }
        else
            break;
    }
    if (count > 0)
        return sumX / count;
    else
        return 0.0;
}

/*
 * 角度均值滤波
 */
float meanFiltAzimuth(float *x, int n, int nFit)
{
    float sumX = 0.0;
    float sumY = 0.0;
    float count = 0.0;
    for (int i = 0; i < nFit; i++)
    {
        if (n - i - 1 >= 0)
        {
            sumX += cos(x[n - i - 1] / 180 * PI);
            sumY += sin(x[n - i - 1] / 180 * PI);
            count++;
        }
        else
            break;
    }
    if (count > 0)
        return fmod(atan2(sumY, sumX) / PI * 180 + 360, 360);
    else
        return 0.0;
}

/*
 * 无人机目标跟踪
 */
int objTrack(
    float countTime, unsigned int frameCount, struct objInfo *recordedObj, int *nObj, struct objInfo *objOut, struct pulseGroup *uavPulse, int nUavPulse, struct pulseGroup *uavCtrPulse, int nUavCtrPulse,
    struct UAVLib *UAVtypes, struct detectParams detectedParam)
{
    int nObjOut = 0;
    int nObj0 = *nObj;
    int usefulCount = detectedParam.usefulCount;           // 出现次数大于此限度，认为目标有效
    int confidenceThresh = detectedParam.confidenceThresh; // 置信度大于此限度，认为目标有效，可以输出
    int vidFitPoints = detectedParam.vidFitPoints;         // 图传平滑点数
    int ctrFitPoints = detectedParam.ctrFitPoints;         // 遥控平滑点数
    float timeOut = detectedParam.timeOut;                 // 目标过期时间

    int matchedIndex[MaxUAV];
    int isRelated[MaxUAV];
    int sIndex[MaxUAV];
    int index[MaxUAV];
    float sumTErr[MaxUAV];
    time_t timep;
    struct tm *p;
    time(&timep);
    p = localtime(&timep);

    int disapperTime = 0;

    // 更新字段重置为false
    for (int i = 0; i < nObj0; i++)
    {
        recordedObj[i].isUpdated = false;
    }

    // 加入使用图传检测方法检测的无人机信息
    for (int i = 0; i < nUavPulse; i++)
    {
        // printf("selIndex0 = %d,%d\n",isRelated[0],matchedIndex[0]);
        objRelated(countTime, frameCount, recordedObj, &nObj0, uavPulse[i], i, UAVtypes, detectedParam, &(matchedIndex[i]), &isRelated[i], &index[i], &sumTErr[i]);
    }
    // printf("selIndex1 = %d,%d\n",isRelated[0],matchedIndex[0]);
    checkRelated(nObj0, nUavPulse, matchedIndex, isRelated, sumTErr);
    for (int i = 0; i < nUavPulse; i++)
    {
        updateRecordedObj(countTime, frameCount, recordedObj, &nObj0, uavPulse[i], i, detectedParam, matchedIndex[i], isRelated, 2, index[i], sumTErr);
    }

    // 加入使用遥控检测方法得到的无人机信息
    for (int i = 0; i < nUavCtrPulse; i++)
    {
        objRelated(countTime, frameCount, recordedObj, &nObj0, uavCtrPulse[i], i, UAVtypes, detectedParam, &matchedIndex[i], &isRelated[i], &index[i], &sumTErr[i]);
    }
    checkRelated(nObj0, nUavCtrPulse, matchedIndex, isRelated, sumTErr);
    for (int i = 0; i < nUavCtrPulse; i++)
    {
        updateRecordedObj(countTime, frameCount, recordedObj, &nObj0, uavCtrPulse[i], i, detectedParam, matchedIndex[i], isRelated, 1, index[i], sumTErr);
    }

    // printf("nObj=%d\n", nObj0);

    // 清理过期的目标
    for (int i = 0; i < nObj0; i++)
    {
        disapperTime = getDiffTime(p, recordedObj[i].detectedTime[recordedObj[i].num - 1]);
        //        printf(" disapper = %d, %d\n",disapperTime,recordedObj[i].num);

        if (disapperTime > timeOut)
        {
            for (int j = i; j < nObj0 - 1; j++)
            {
                recordedObj[j] = recordedObj[j + 1];
            }
            nObj0--;
            --i;
        }
    }

    nObjOut = 0;
    for (int i = 0; i < nObj0; i++)
    {
        if (recordedObj[i].count > usefulCount)
        {
            char possibility = evalPr(recordedObj[i]);
            if (possibility > confidenceThresh)
            {
                objOut[nObjOut] = recordedObj[i];
                objOut[nObjOut].possibility[0] = possibility;

                if (strncmp(UAVtypes[objOut[nObjOut].uavIndex].vidOrCtr, "vid", 3) == 0)
                {
                    for (int j = 0; j < objOut[nObjOut].num; j++)
                    {
                        objOut[nObjOut].azimuth[j] = meanFiltAzimuth(recordedObj[i].azimuth, j + 1, vidFitPoints);
                        objOut[nObjOut].range[j] = meanFiltRange(recordedObj[i].range, j + 1, vidFitPoints);
                    }
                }
                else
                {
                    for (int j = 0; j < objOut[nObjOut].num; j++)
                    {
                        objOut[nObjOut].azimuth[j] = meanFiltAzimuth(recordedObj[i].azimuth, j + 1, ctrFitPoints);
                        objOut[nObjOut].range[j] = meanFiltRange(recordedObj[i].range, j + 1, ctrFitPoints);
                    }
                }

                for (int j = 0; j < MaxCompassAngNum; j++)
                {
                    objOut[nObjOut].compCorrAmp[j] = recordedObj[i].compCorrAmp[j];
                }
                objOut[nObjOut].validAmpN = recordedObj[i].validAmpN;
                nObjOut++;
            }
        }
    }
    *nObj = nObj0;
    return nObjOut;
}

/*
 * 生成发送到qt的信息
 */
void genObjForUi(struct ObjForUI *objforUi, struct message *messagePtr, struct objInfo *objOut, int nObj, struct pulseGroup *uavPulse, struct pulseGroup *uavCtrPulse)
{
    int count = 0;
    for (int i = 0; i < nObj; i++)
    {
        if (count >= MAX_PLANE_AMOUNT)
            break;
        if (objOut[i].isUpdated)
        {
            objforUi->obj[count].objID = messagePtr->obj[count].objID;
            strcpy(objforUi->obj[count].name, messagePtr->obj[count].name);
            objforUi->obj[count].azimuth = messagePtr->obj[count].azimuth;
            objforUi->obj[count].pitchAngle = messagePtr->obj[count].height;
            objforUi->obj[count].range = messagePtr->obj[count].range;
            objforUi->obj[count].longitude = messagePtr->obj[count].longitude;
            objforUi->obj[count].latitude = messagePtr->obj[count].latitude;
            objforUi->obj[count].height = 100; // messagePtr->obj[count].height;
            objforUi->obj[count].freq = messagePtr->obj[count].freq / 1.0e3;
            objforUi->obj[count].pulseBW = messagePtr->obj[count].pulseBW / 1.0e3;
            objforUi->obj[count].pulseW = messagePtr->obj[count].pulseW;
            objforUi->obj[count].pulseAmp = messagePtr->obj[count].pulseAmp;
            objforUi->obj[count].possibility = objOut[i].possibility[0];
            objforUi->obj[count].objDuration = getDiffDateTime(objOut[i].detectedTime[objOut[i].num - 1], objOut[i].firstDetectedTime);
            objforUi->obj[count].wHour = messagePtr->obj[count].wHour;
            objforUi->obj[count].wMinute = messagePtr->obj[count].wMinute;
            objforUi->obj[count].wSecond = messagePtr->obj[count].wSecond;
            objforUi->obj[count].typ = messagePtr->obj[count].typ;

            if (objOut[i].sourceIndex == 1) // 目标信息来自遥控脉冲串
            {
                int num = 0;
                for (int j = 0; j < uavCtrPulse[objOut[i].pulseIndex].nPulse; j++)
                {
                    if (j >= MaxVidPulse)
                        break;
                    objforUi->obj[count].pulseTime[j] = uavCtrPulse[objOut[i].pulseIndex].pulseTime[j];
                    objforUi->obj[count].pulseWs[j] = uavCtrPulse[objOut[i].pulseIndex].pulseW[j];
                    objforUi->obj[count].freqPoints[j] = uavCtrPulse[objOut[i].pulseIndex].freq[j];
                    num++;
                }
                objforUi->obj[count].nPulse = num;

                // 特殊处理id号，界面灰色显示已经失效的目标
                if (uavCtrPulse[objOut[i].pulseIndex].nPulse == 0)
                    objforUi->obj[count].objID = uavCtrPulse[objOut[i].pulseIndex].id;
            }
            else if (objOut[i].sourceIndex == 2) // 目标信息来自图传脉冲串
            {
                int num = 0;
                for (int j = 0; j < uavPulse[objOut[i].pulseIndex].nPulse; j++)
                {
                    if (j >= MaxVidPulse)
                        break;
                    objforUi->obj[count].pulseTime[j] = uavPulse[objOut[i].pulseIndex].pulseTime[j];
                    objforUi->obj[count].pulseWs[j] = uavPulse[objOut[i].pulseIndex].pulseW[j];
                    objforUi->obj[count].freqPoints[j] = uavPulse[objOut[i].pulseIndex].freq[0];
                    num++;
                }
                objforUi->obj[count].nPulse = num;
            }
            count++;
        }
    }
    objforUi->amount = count;
}

void ColPivot(float *x, float *a, int n)
{
    int i, j, t, k;
    float *c, p;
    c = (float *)malloc(n * (n + 1) * sizeof(float));
    for (i = 0; i <= n - 1; i++)
    {
        for (j = 0; j <= n; j++)
            *(c + i * (n + 1) + j) = (*(a + i * (n + 1) + j));
    }
    for (i = 0; i <= n - 2; i++)
    {
        k = i;
        for (j = i + 1; j <= n - 1; j++)
        {
            if (fabs(*(c + j * (n + 1) + i)) > (fabs(*(c + k * (n + 1) + i))))
                k = j;
        }
        if (k != i)
        {
            for (j = i; j <= n; j++)
            {
                p = *(c + i * (n + 1) + j);
                *(c + i * (n + 1) + j) = *(c + k * (n + 1) + j);
                *(c + k * (n + 1) + j) = p;
            }
        }
        for (j = i + 1; j <= n - 1; j++)
        {
            p = (*(c + j * (n + 1) + i)) / (*(c + i * (n + 1) + i));
            for (t = i; t <= n - 1; t++)
                *(c + j * (n + 1) + t) = *(c + j * (n + 1) + t) - p * (*(c + i * (n + 1) + t));
            *(c + j * (n + 1) + n) -= *(c + i * (n + 1) + n) * p;
        }
    }
    for (i = n - 1; i >= 0; i--)
    {
        for (j = n - 1; j >= i + 1; j--)
            (*(c + i * (n + 1) + n)) -= x[j] * (*(c + i * (n + 1) + j));
        x[i] = *(c + i * (n + 1) + n) / (*(c + i * (n + 1) + i));
    }
    free(c);
}

float power(int i, float v)
{
    float a = 1.0;
    while (i--)
        a *= v;
    return a;
}

/*
 * 多项式拟合
 */
void polyfit(float *a, float *x, float *y, int m, int n)
{
    float *c;
    int i, j, t;
    c = (float *)malloc((n + 1) * (n + 2) * sizeof(float));
    for (i = 0; i <= n; i++)
    {
        for (j = 0; j <= n; j++)
        {
            *(c + i * (n + 2) + j) = 0.0;
            for (t = 0; t <= m - 1; t++)
                *(c + i * (n + 2) + j) += power(i + j, x[t]);
        }
        *(c + i * (n + 2) + n + 1) = 0.0;
        for (j = 0; j <= m - 1; j++)
            *(c + i * (n + 2) + n + 1) += y[j] * power(i, x[j]);
    }
    ColPivot(a, c, n + 1);
    free(c);
}

// 盾项目的测向版本
// float calcAngleShield(struct objInfo *recordedObj, int nObjRecord, struct objInfo *objOut, int nObj)
// {
//     //    printf("begin calc Angle shield\n");
//     // int index = (int)compassAngle;
//     int spanN = 8;
//     float tmpMaxAmp = 0, tmpFitAmp = 0;
//     int tmpMaxInd = 0, refAngle = 0;
//     int calcFlag = 0, dataN = 0, n = 5; // n为多项式系数阶数
//     float p[5] = {0};
//     float x[MaxCompassAngNum] = {0};
//     float y[MaxCompassAngNum] = {0};
//     float tmpCompCorrAmp[MaxCompassAngNum] = {0};
//     // FILE *fp = fopen("ShieldResultLog.txt","a+");
//     for (int i = 0; i < nObjRecord; i++)
//     {
//         if (recordedObj[i].isUpdated)
//         {
//             if (recordedObj[i].compCorrAmp[index] <= 0)
//                 recordedObj[i].validAmpN++;
//             recordedObj[i].compCorrAmp[index] = recordedObj[i].pulseAmp[recordedObj[i].num - 1];
//             //            for(int j = 0; j < recordedObj[i].num; j++)
//             //            {
//             //                printf("%d, %f ",recordedObj[i].num, recordedObj[i].pulseAmp[j]);
//             //            }
//         }
//         //        printf("\n");
//     }

//     for (int i = 0; i < nObj; i++)
//     {
//         if (objOut[i].isUpdated)
//         {
//             // fprintf(fp,"%d, %d\n",objOut[i].objID, objOut[i].uavIndex);
//             if (objOut[i].compCorrAmp[index] <= 0)
//                 objOut[i].validAmpN++;
//             objOut[i].compCorrAmp[index] = objOut[i].pulseAmp[objOut[i].num - 1];

//             calcFlag = 0;
//             tmpMaxInd = 0;
//             tmpMaxAmp = 0;
//             dataN = 0;
//             refAngle = 0;

//             for (int j = 0; j < MaxCompassAngNum; j++)
//             {
//                 tmpCompCorrAmp[j] = 0;
//                 //                printf("%f, ",objOut[i].compCorrAmp[j]);
//                 // fprintf(fp,"%f,",objOut[i].compCorrAmp[j]);
//                 if (objOut[i].compCorrAmp[j] > 0 && objOut[i].compCorrAmp[j] > tmpMaxAmp)
//                 {
//                     tmpMaxAmp = objOut[i].compCorrAmp[j];
//                     tmpMaxInd = j;
//                 }
//             }
//             // fprintf(fp,"\n");
//             tmpMaxAmp = 0;
//             if (objOut[i].validAmpN > spanN) // amp fit, calc angle
//             {
//                 calcFlag = 1;
//                 for (int j = 0; j < MaxCompassAngNum; j++)
//                 {
//                     if (objOut[i].compCorrAmp[j] > 0)
//                     {
//                         x[dataN] = float(fmod(j + tmpMaxInd + 180, 360));
//                         y[dataN] = objOut[i].compCorrAmp[j];
//                         dataN++;
//                     }
//                 }
//                 polyfit(p, x, y, dataN, n);
//                 for (int j = 90; j <= MaxCompassAngNum - 90; j++)
//                 {

//                     tmpFitAmp = p[4] * power(4, float(j)) + p[3] * power(3, float(j)) + p[2] * power(2, float(j)) + p[1] * power(1, float(j)) + p[0];
//                     if (tmpFitAmp > tmpMaxAmp)
//                     {
//                         refAngle = j;
//                         tmpMaxAmp = tmpFitAmp;
//                     }
//                 }
//             }
//             if (calcFlag)
//             {
//                 // fprintf(fp,"calcFlag = %d,p = %f,%f,%f,%f,%f, refAngle = %d, tmpMaxInd = %d\n",calcFlag,p[0],p[1],p[2],p[3],p[4],refAngle,tmpMaxInd);
//                 refAngle = (int)fmod(refAngle - 180 + tmpMaxInd + 360, 360);
//                 objOut[i].angleConfi = (int)(log10(0.2 * (objOut[i].validAmpN + 5)) * 100);
//             }
//             else
//             {
//                 // fprintf(fp,"calcFlag = %d,p = 0.0,0.0,0.0,0.0,0.0, refAngle = -1, tmpMaxInd = -1\n",calcFlag);
//                 refAngle = tmpMaxInd;
//                 objOut[i].angleConfi = (int)(objOut[i].validAmpN / (2 * spanN * 1.0) * 100);
//             }
//             if (objOut[i].angleConfi > 95)
//                 objOut[i].angleConfi = 95;
//             objOut[i].azimuth[objOut[i].num - 1] = refAngle - compassAngle;
//             // fprintf(fp,"refAngle = %d, azimuth = %f, compassAngle = %f\n\n\n",refAngle, compassAngle, objOut[i].azimuth[objOut[i].num - 1]);
//         }
//     }
//     // fclose(fp);
//     return 0;
// }

float calibrateAmp(float freq, float *calibAmp)
{
    int i = 0;
    int ind = 0;
    float minErrFreq = 6000;
    float errFreq = 0.0;
    while (calibAmp[i] > 0.0)
    {
        errFreq = fabs(freq - calibAmp[i]);
        if (errFreq < minErrFreq)
        {
            minErrFreq = errFreq;
            ind = i;
        }
        i = i + 2;
    }
    return calibAmp[ind + 1];
}

int checkBw(struct UAVLib *UAVtypes, struct objInfo *objOut)
{
    int ret = 0;
    for (int i = 0; i < UAVtypes[objOut->uavIndex].nPulseBW; i++)
    {
        if (objOut->pulseBW[objOut->num - 1] == UAVtypes[objOut->uavIndex].pulseBW[i])
            ret = 1;
    }
    return ret;
}

/*
生成目标信息
*/
void genObj(struct message *messagePtr, struct objInfo *objOut, int nObj, struct UAVLib *UAVtypes, struct deviceParam devParam, struct detectParams detectedParam)
{

    int i, j;
    int count = 0;
    float longitude, latitude, compassAng;
    float d, theta, beta;

    time_t timep;
    struct tm *p;
    time(&timep);
    p = localtime(&timep);
    float caliAmp = 0.0;

    // gps
    if (devParam.lngLatValid > -1)
    {
        longitude = devParam.longitude;
        latitude = devParam.latitude;
        messagePtr->stationInfo.gpsStatus = devParam.lngLatValid;
        messagePtr->stationInfo.longitude = longitude;
        messagePtr->stationInfo.latitude = latitude;
    }
    else
    {
        messagePtr->stationInfo.gpsStatus = -1;
        longitude = 0.0;
        latitude = 0.0;
    }
    // 罗盘
    if (devParam.angleValid > -1)
    {
        compassAng = devParam.compassAngle;
        messagePtr->stationInfo.compassStatus = devParam.angleValid;
        messagePtr->stationInfo.angle = compassAng;
    }
    else
    {
        compassAng = -1;
        messagePtr->stationInfo.compassStatus = -1;
        messagePtr->stationInfo.angle = compassAng;
    }
    messagePtr->stationInfo.height = 100;
    messagePtr->stationInfo.status = 1;
    // 当前更新的目标填写进待发送信息
    for (i = 0; i < nObj; i++)
    {
        if (count >= MAX_PLANE_AMOUNT)
            break;
        // caliAmp = calibrateAmp(objOut[i].freq[objOut[i].num - 1],detectedParam.calibAmp);
        // objOut[i].pulseAmp[objOut[i].num - 1] -= caliAmp;
        //        printf("objOut[i].isUpdated =  %d!!!!!!!!!!!!!!!!!\n",objOut[i].isUpdated);
        if (objOut[i].isUpdated)
        {
            int ret = 1;
            messagePtr->obj[count].objID = objOut[i].objID; // 目标ID编号
            if (strncmp(UAVtypes[objOut[i].uavIndex].vidOrCtr, "vid", 3) == 0 && UAVtypes[objOut[i].uavIndex].isFixedFreq == 0 && detectedParam.isCaliFreqBw == 1)
                ret = checkBw(UAVtypes, &objOut[i]);

            if (ret == 0)
            {
                sprintf(messagePtr->obj[count].name, "%s%s", detectedParam.objPrefix, UAVtypes[objOut[i].uavIndex].name);
            }
            else if (ret == 1)
                strcpy(messagePtr->obj[count].name, UAVtypes[objOut[i].uavIndex].name); // 无人机名称

            if (strncmp(UAVtypes[objOut[i].uavIndex].vidOrCtr, "vid", 3) == 0)
                messagePtr->obj[count].typ = 2; // 图传或遥控或其它疑似目标，图传2，遥控1，框选3
            else if (strncmp(UAVtypes[objOut[i].uavIndex].vidOrCtr, "ctr", 3) == 0)
                messagePtr->obj[count].typ = 1;
            messagePtr->obj[count].freq = objOut[i].freq[objOut[i].num - 1] * 1e3;       // 中心频率,kHz
            messagePtr->obj[count].pulseBW = objOut[i].pulseBW[objOut[i].num - 1] * 1e3; // 带宽,kHz
            messagePtr->obj[count].pulseW = objOut[i].pulseW[objOut[i].num - 1][0];      // 脉宽,ms
            messagePtr->obj[count].pulseAmp = objOut[i].pulseAmp[objOut[i].num - 1];     // 信号强度，dBm
            messagePtr->obj[count].possibility = objOut[i].possibility[0];
            for (int j = 0; j < 16; j++)
                messagePtr->obj[count].pulesTime[j] = 0.0;
            for (int j = 0; j < objOut[i].num; j++)
            {
                if (j < 16 && objOut[i].pulseTime[objOut[i].num - 1][j] >= 0.0)
                    messagePtr->obj[count].pulesTime[j] = floor(objOut[i].pulseTime[objOut[i].num - 1][j] / 0.053 + 0.5);
                else
                    break;
            }
            messagePtr->obj[count].frameCount = objOut[i].frameCount;

            // if(devParam.lngLatValid==0)
            //{
            //	messagePtr->obj[count].pulseTime[0]=devParam.gpsTime.seconds*1000+floor(devParam.gpsTime.nanoSecond/1e6+0.5);

            //}

            messagePtr->obj[count].azimuth = fmod(objOut[i].azimuth[objOut[i].num - 1] + compassAng + 720, 360); // 方位角,度
            messagePtr->obj[count].range = objOut[i].range[objOut[i].num - 1];                                   // 无人机的距离,m
            theta = messagePtr->obj[count].azimuth;
            d = messagePtr->obj[count].range;
            messagePtr->obj[count].latitude = latitude + d * cos(theta / 180 * PI) / (PI * EarthRadius) * 180.0; // 无人机所在的纬度，度
            beta = messagePtr->obj[count].latitude;
            messagePtr->obj[count].longitude = longitude + d * sin(theta / 180 * PI) / (PI * EarthRadius * cos(beta / 180 * PI)) * 180.0; // 无人机所在的经度，度
            messagePtr->obj[count].height = objOut[i].pitchAngle[objOut[i].num - 1];                                                      // 站点的海拔高度，m

            if (devParam.lngLatValid == 0)
            {
                messagePtr->obj[count].wHour = devParam.gpsTime.hour; // 使用gps时间
                messagePtr->obj[count].wMinute = devParam.gpsTime.minute;
                messagePtr->obj[count].wSecond = devParam.gpsTime.seconds;
                messagePtr->obj[count].wMilliseconds = floor(devParam.gpsTime.nanoSecond / 1e6 + 0.5);
            }
            else
            {
                messagePtr->obj[count].wHour = objOut[i].detectedTime[objOut[i].num - 1].hour;
                messagePtr->obj[count].wMinute = objOut[i].detectedTime[objOut[i].num - 1].minute;
                messagePtr->obj[count].wSecond = objOut[i].detectedTime[objOut[i].num - 1].seconds;
                messagePtr->obj[count].wMilliseconds = objOut[i].detectedTime[objOut[i].num - 1].nanoSecond / 1e6;
            }
            messagePtr->obj[count].objDuration = 0;
            messagePtr->obj[count].mode = (unsigned char)(UAVtypes[objOut[i].uavIndex].mrm);

            count++;
        }
    }
    // 填写目标个数
    if (count <= MAX_PLANE_AMOUNT)
        messagePtr->amount = count;
    else
        messagePtr->amount = MAX_PLANE_AMOUNT;
}

// //检查框选目标的频率是否在本次频谱的带宽内
// int checkBoxObj(struct boxObjEvent boxObjs)
// {
//     if (boxObjs.nBoxObj > MaxBoxObj || boxObjs.nBoxObj < 0)
//         return -1;
//     for (int i = 0; i < boxObjs.nBoxObj; i++)
//     {
//         if (boxObjs.boxObjInfo[i].boxNum > MaxBoxNum || boxObjs.boxObjInfo[i].boxNum < 1)
//             return -1;
//         for (int j = 0; j < boxObjs.boxObjInfo[i].boxNum; j++)
//         {
//             if (boxObjs.boxObjInfo[i].boxInfo[j].x1 < 6 || boxObjs.boxObjInfo[i].boxInfo[j].x1 > 250)
//                 return -1;
//             if (boxObjs.boxObjInfo[i].boxInfo[j].y1 < 0 || boxObjs.boxObjInfo[i].boxInfo[j].y1 > 2000)
//                 return -1;
//         }
//     }
//     return 0;
// }

// /*
//  * 将框选脉冲信息加入到目标信息和发送给界面的信息结构体中
//  */
// void boxPulseToObj(struct boxPulse *uavBoxPulse, int nBoxPulse, struct message *messagePtr, struct ObjForUI *objForUi, int antennaNum)
// {
// 	int n1=messagePtr->amount;
// 	int n2=objForUi->amount,maxAmpInd;
// 	float latitude,longitude;
// 	float d,theta,beta,tmpMaxAmp;

// 	latitude=messagePtr->stationInfo.latitude;
// 	longitude=messagePtr->stationInfo.longitude;

// 	time_t     timep;
//     struct tm *p;
//     time(&timep);
//     p = localtime(&timep);

// 	for(int i=0;i<nBoxPulse;i++)
// 	{
// 		if(uavBoxPulse[i].meanAmp[0][0] <=0)
// 			break;
// 		for(int ii = 0; ii < antennaNum; ii++)
// 		{
// 		    //printf("antennaNum  = %d,uavBoxPulse[i].meanAmp[ii][0] = %f \n",antennaNum , uavBoxPulse[i].meanAmp[ii][0]);
// 		    if(uavBoxPulse[i].meanAmp[ii][0] > tmpMaxAmp)
// 		    {
// 			tmpMaxAmp = uavBoxPulse[i].meanAmp[ii][0];
// 			maxAmpInd = ii;
// 		    }
// 		}
// 		if(uavBoxPulse[i].nPulse>0)//用来通知界面删除框选目标的信息不加入到目标中
// 		{
// 			if(n1<MAX_PLANE_AMOUNT)
// 			{
// 				messagePtr->obj[n1].azimuth=fmod(uavBoxPulse[i].azimuth+messagePtr->stationInfo.angle+360,360);
// 				messagePtr->obj[n1].range=uavBoxPulse[i].range;
// 				messagePtr->obj[n1].freq=uavBoxPulse[i].freq[0]*1e3;
// 				messagePtr->obj[n1].height=100;
// 				d=messagePtr->obj[n1].range;
// 				theta=messagePtr->obj[n1].azimuth;
// 				messagePtr->obj[n1].latitude=latitude + d * cos(theta / 180 * PI) / (PI * EarthRadius) * 180.0;;
// 				beta=messagePtr->obj[n1].latitude;
// 				messagePtr->obj[n1].longitude=longitude + d * sin(theta / 180 * PI) / (PI * EarthRadius * cos(beta / 180 * PI)) * 180.0;
// 				strcpy(messagePtr->obj[n1].name,uavBoxPulse[i].name);
// 				messagePtr->obj[n1].objDuration=0;
// 				messagePtr->obj[n1].objID=uavBoxPulse[i].id;
// 				messagePtr->obj[n1].possibility=uavBoxPulse[i].possibility;
// 				messagePtr->obj[n1].pulseAmp=uavBoxPulse[i].meanAmp[maxAmpInd][0];
// 				messagePtr->obj[n1].pulseBW=uavBoxPulse[i].pulseBW*1e3;
// 				messagePtr->obj[n1].pulseW=uavBoxPulse[i].pulseW[0];
// 				messagePtr->obj[n1].typ=uavBoxPulse[i].vidOrCtr;

// 				messagePtr->obj[n1].wHour=p->tm_hour;
// 				messagePtr->obj[n1].wMinute=p->tm_min;
// 				messagePtr->obj[n1].wSecond=p->tm_sec;
// 				messagePtr->obj[n1].wMilliseconds=0;
// 				n1++;
// 			}
// 		}

// 		if(n2<MAX_PLANE_AMOUNT)
// 		{
// 			objForUi->obj[n2].azimuth=uavBoxPulse[i].azimuth;
// 			objForUi->obj[n2].freq=uavBoxPulse[i].freq[0];
// 			objForUi->obj[n2].height=100;
// 			if(uavBoxPulse[i].nPulse>0)
// 			{
// 				d=uavBoxPulse[i].range;
// 				theta=fmod(uavBoxPulse[i].azimuth+messagePtr->stationInfo.angle+360,360);
// 				objForUi->obj[n2].latitude=latitude + d * cos(theta / 180 * PI) / (PI * EarthRadius) * 180.0;;
// 				beta=objForUi->obj[n2].latitude;
// 				objForUi->obj[n2].longitude=longitude + d * sin(theta / 180 * PI) / (PI * EarthRadius * cos(beta / 180 * PI)) * 180.0;
// 			}
// 			else
// 			{
// 				objForUi->obj[n2].latitude=0.0;
// 				objForUi->obj[n2].longitude=0.0;
// 			}

// 			strcpy(objForUi->obj[n2].name,uavBoxPulse[i].name);
// 			objForUi->obj[n2].nPulse=uavBoxPulse[i].nPulse;
// 			objForUi->obj[n2].objDuration=0;
// 			objForUi->obj[n2].objID=uavBoxPulse[i].id;
// 			objForUi->obj[n2].pitchAngle=0;
// 			objForUi->obj[n2].possibility=uavBoxPulse[i].possibility;
// 			objForUi->obj[n2].pulseAmp=uavBoxPulse[i].meanAmp[maxAmpInd][0];
// 			objForUi->obj[n2].pulseBW=uavBoxPulse[i].pulseBW;
// 			objForUi->obj[n2].pulseW=uavBoxPulse[i].pulseW[0];
// 			objForUi->obj[n2].range=uavBoxPulse[i].range;
// 			objForUi->obj[n2].typ=uavBoxPulse[i].vidOrCtr;
// 			objForUi->obj[n2].wHour=p->tm_hour;
// 			objForUi->obj[n2].wMinute=p->tm_min;
// 			objForUi->obj[n2].wSecond=p->tm_sec;

// 			for(int j=0;j<uavBoxPulse[i].nPulse;j++)
// 			{
// 				objForUi->obj[n2].freqPoints[j]=uavBoxPulse[i].freq[j];
// 				objForUi->obj[n2].pulseTime[j]=uavBoxPulse[i].pulseTime[j];
// 				objForUi->obj[n2].pulseWs[j]=uavBoxPulse[i].pulseW[j];
// 			}
// 			n2++;
// 		}
// 	}
// 	messagePtr->amount=n1;
// 	objForUi->amount=n2;
// }

// /*
//  * 将框选检测到的脉冲加入到遥控结果中，便于opencv加边框显示
//  */
// void appendBoxToCtrPulse(struct pulseGroup *uavCtrPulse, int &nUavCtrPulse, struct boxPulse *uavBoxPulse, int nBoxPulse, struct UAVLib *UAVtypes, int &nUAV)
// {
//     for(int i=0;i<nBoxPulse;i++)
//     {
// 		if(uavBoxPulse[i].nPulse==0)
// 			continue;
//     	if(nUavCtrPulse<MaxUAV)
//     	{
//     		uavCtrPulse[nUavCtrPulse].id=uavBoxPulse[i].id;
//     		uavCtrPulse[nUavCtrPulse].uavIndex=nUAV;
// 			strcpy(UAVtypes[nUAV].name,uavBoxPulse[i].name);
// 			nUAV++;
//     		uavCtrPulse[nUavCtrPulse].azimuth=uavBoxPulse[i].azimuth;
//     		uavCtrPulse[nUavCtrPulse].range=uavBoxPulse[i].range;
// 			uavCtrPulse[nUavCtrPulse].pulseBW=uavBoxPulse[i].pulseBW;
// 			uavCtrPulse[nUavCtrPulse].possibility=uavBoxPulse[i].possibility;
//     		uavCtrPulse[nUavCtrPulse].nPulse=uavBoxPulse[i].nPulse;

// 			for(int j=0;j<NCh;j++)
// 			{
// 				uavCtrPulse[nUavCtrPulse].onAnt[j]=uavBoxPulse[i].onAnt[j];
// 				for(int k=0;k<MaxBoxNum;k++)
// 				{
// 					uavCtrPulse[nUavCtrPulse].meanAmp[j][k]=uavBoxPulse[i].meanAmp[j][k];
// 					uavCtrPulse[nUavCtrPulse].meanPhase[j][k]=uavBoxPulse[i].meanPhase[j][k];
// 					uavCtrPulse[nUavCtrPulse].phaseVar[j][k]=uavBoxPulse[i].phaseVar[j][k];
// 					for(int p=0;p<PhaseBinNum;p++)
// 					{
// 						uavCtrPulse[nUavCtrPulse].phaseHist[j][k][p]=0.0;
// 						uavCtrPulse[nUavCtrPulse].weightedPhaseHist[j][k][p]=0.0;
// 					}
// 				}
// 			}

//     		for(int j=0;j<MaxBoxNum;j++)
// 			{
// 				uavCtrPulse[nUavCtrPulse].antFace[j]=uavBoxPulse[i].antFace[j];
// 				uavCtrPulse[nUavCtrPulse].freq[j]=uavBoxPulse[i].freq[j];
// 				uavCtrPulse[nUavCtrPulse].pulseTime[j]=uavBoxPulse[i].pulseTime[j];
// 				uavCtrPulse[nUavCtrPulse].pulseW[j]=uavBoxPulse[i].pulseW[j];
// 				uavCtrPulse[nUavCtrPulse].angle[j]=uavBoxPulse[i].angle[j];
// 				uavCtrPulse[nUavCtrPulse].distance[j]=uavBoxPulse[i].distance[j];
// 			}

//     		nUavCtrPulse++;
//     	}
//     	else
//     		break;
//     }
// }

/*
 * 根据不同抽取比例，更新无人机库参数。1G以下由于数据时长是其它频段的4倍，需要加严要求避免虚警升高
 */
void updateUavlib(struct UAVLib *UAVtypes, int nUav, int decim)
{
    for (int i = 0; i < nUav; i++)
    {
        if (strcmp(UAVtypes[i].name, "Xinghuo433") == 0 || strcmp(UAVtypes[i].name, "Mavlink433Drone") == 0 || strcmp(UAVtypes[i].name, "Mavlink915Wide") == 0)
            continue;
        for (int j = 0; j < UAVtypes[i].nPulseW; j++)
            UAVtypes[i].meetPulseW[j] *= decim / 1.5;

        if (UAVtypes[i].nPulseT > 0)
            UAVtypes[i].meetHopp *= decim / 2;
    }
}

/*
 * 不同的天线类型，修改算法参数（填充的最大沟壑点数，最小脉宽的点数，误差点数容限）
 */
void updateParam(struct detectParams *detectedParam, int corrFlag)
{
    detectedParam->Delt = ceil(detectedParam->Delt / 2);
    detectedParam->MinW = ceil(detectedParam->MinW / 2);
    detectedParam->q1IndexErr = ceil(detectedParam->q1IndexErr / 2);
    detectedParam->qMeetTimes = ceil(detectedParam->qMeetTimes / 2);
}

/**
 *@brief 无人机图传和遥控检测函数
 *@param[in] detectParamFilePath// 检测算法和显示参数配置文件路径
 *@param[in] uavFeatureFilePath//	无人机特征库配置文件路径
 *@param[in] sumCorr    //	相关实部和虚部
 */
int uavDetect(char *udp_send_ip, int *sumCorr, float freqcenter, int *udpData, struct UAVLib *UAVtypes, struct detectParams detectedParam, int nUAV, int workmode)
{
    //    udp_data_send(sumCorr,sizeof (sumCorr));

    char timeNow[100];
    static int NTime = 0;
    getTime(timeNow);
    printf("\ntimeNow = %d , %s\n", NTime, timeNow);
    NTime++;
    struct message messagePtr;
    struct ObjForUI objForUi;
    int nCalib;
    //    char detectParamFilePath[] = "./detectParams.ini";
    //    char uavFeatureFilePath[] = "./uavFeature.ini";
    //    struct detectParams detectedParam;
    //    struct UAVLib UAVtypes[MaxUAVinLib];
    static struct objInfo recordedObj[MaxObjNum];
    static struct objInfo objOut[MaxObjNum];
    static int nObjRecord = 0;
    int nObj = 0;

    struct deviceParam devParam;
    int nRows = NROWS;          // 相关复数的行数
    int nCols = NFFT / NSumCol; // 相关复数的列数
    float dF = Freqs / NFFT * NSumCol;
    int trackSuccess = 0;
    int SumCorrsize = 0; // 续写最近N个相关数据个数
    int SumCorrsign = 0; // 写相关种类标志，0：写老相关 1：续写最近N个老相关 2：写新相关
    clock_t t1, t2;
    int nSumRow = NSumRow;
    //    t1 =clock();
    //    if (getDetectParam(&detectedParam, detectParamFilePath) < 0) //读取检测算法的参数
    //        return -1;
    //    t2 =clock();
    //    printf("getDetectParam use time = %f ms\n",(double)(t2 - t1) / CLOCKS_PER_SEC*1000);
    detectedParam.doaTyp = 3;
    detectedParam.antennaNum = 1;
    // 定义图传脉冲用的变量
    struct pulseGroup uavPulse[MaxUAV];    // = (struct pulseGroup *)malloc(MaxUAV * sizeof(struct pulseGroup));    //存储无人机图传脉冲串
    int nUavPulse = 0;                     // 记录无人机图传脉冲串的个数
    struct pulseGroup uavCtrPulse[MaxUAV]; // = (struct pulseGroup *)malloc(MaxUAV * sizeof(struct pulseGroup)); //无人机遥控脉冲串
    int nUavCtrPulse = 0;                  // 无人机遥控脉冲串个数

    //     float ***sumCorrAmp, ***sumCorrPhase;  //相关幅度矩阵和相位矩阵
    struct detectParams param = detectedParam;

    float sumCorrAmp[NROWS][NFFT / NSumCol], sumCorrPhase[NROWS][NFFT / NSumCol];

    //     clock_t t1, t2;
    // double useTime;
    // nCalib = getCalibLibPro(&detectedParam);
    // nCalib = getCalibLib(&detectedParam);

    // if(detectedParam.calibType == 1)
    // {
    //     memset(&detectedParam.calibParamNew,0,sizeof(detectedParam.calibParamNew));
    //     int retCalibNum = getCalibSql(&detectedParam);
    // }
    //     t1 =clock();
    //     nUAV = getUavlib(uavFeatureFilePath, UAVtypes); //读取无人机特征库
    //     t2=clock();
    //     printf("getUavlib use time = %f ms\n",(double)(t2 - t1) / CLOCKS_PER_SEC*1000);
    //     printf("----- %d\n", nUAV);
    // printf("----- %d\n", sizeof(detectedParam));
    // 相关谱每个通道的帧头和帧尾数据解析
    // trackSuccess = trackFrame(sumCorr, &nRows, &nSumRow, &devParam, detectedParam);
    devParam.cenFreqOnAllCh[0] = freqcenter;
    devParam.decim = 1;
    //     printf("************trackSuccess finish**********\n");
    nRows = (detectedParam.doaTyp == 5) ? nRows / 2 : nRows;
    int corrFlag = (detectedParam.doaTyp == 5) ? 2 : 1;
    float dt = NFFT * nSumRow / Freqs * devParam.decim / 1000.0 * corrFlag;
    dF /= (float)devParam.decim;
    detectedParam.centFreq = devParam.cenFreqOnAllCh[0];

    // if(devParam.decim>1)
    // 	updateUavlib(UAVtypes, nUAV, devParam.decim);
    //     if (corrFlag > 1)
    updateParam(&detectedParam, corrFlag);

    // 分析特征库频率范围，给出列索引范围
    //  printf("************getFreqSpan **********\n");
    //    t1 =clock();
    getFreqSpan(UAVtypes, nUAV, devParam.cenFreqOnAllCh, dF, detectedParam.vidIndexSpan, detectedParam.ctrIndexSpan);
    //    t2=clock();
    //    printf("getFreqSpan use time = %f ms\n",(double)(t2 - t1) / CLOCKS_PER_SEC*1000);
    // printf("************getFreqSpan done **********\n");
    //     sumCorrAmp = createGrid(NCorr * corrFlag, nRows, nCols);
    //     sumCorrPhase = createGrid(NCorr * corrFlag, nRows, nCols);

    //     sumCorrAmp = createMatrix(nRows, nCols);
    //     sumCorrPhase = createMatrix(nRows, nCols);
    /*
    u_int8_t *NewSaveAMP = (u_int8_t *)malloc(NCorr * corrFlag * nRows * nCols * sizeof(u_int8_t));
    u_int8_t *NewSavePHASE = (u_int8_t *)malloc(NCorr * corrFlag * nRows * nCols * sizeof(u_int8_t));*/

    // 	if(ampPhase!=NULL)
    // 		getAmpAndPhase(sumCorrAmp, sumCorrPhase, ampPhase, corrFlag);
    // 	else
    // 		calcAmpPhase(sumCorr, nRows, nCols, sumCorrAmp, sumCorrPhase, corrFlag);
    // printf("************calcAmpPhaseOneCh**********\n");

    // sumCorr数据
    // calcAmpPhaseOneCh(sumCorr, nRows, nCols, sumCorrAmp, sumCorrPhase, corrFlag);
    // getAmpPhaseOneChSumCorr(sumCorr, nRows, nCols, sumCorrAmp, sumCorrPhase, corrFlag);
    //    int udpData[NROWS * NFFT / NSumCol];
    //    t1 =clock();
    //    FILE *fp = fopen("sumcorr.dat","w");
    //    fwrite(sumCorr, sizeof(int), 128 * 800, fp);
    //    fclose(fp);

    int offsetInd = 0;
    // printf("workmode = %d, freqcenter = %f ,compassAngle = %f\n", workmode, freqcenter, compassAngle);
    if (workmode == 1)
        offsetInd = getAmpPhaseCorr(sumCorr, nRows, nCols, sumCorrAmp, sumCorrPhase, udpData, corrFlag);
    else if (workmode == 2)
        offsetInd = getAmpPhaseOneCh(sumCorr, nRows, nCols, sumCorrAmp, sumCorrPhase, udpData, corrFlag);
    else
        return 0;

    //    t2=clock();
    //    printf("getAmpPhaseOneCh use time = %f ms\n",(double)(t2 - t1) / CLOCKS_PER_SEC*1000);
    //    printf("uavDetect freqcenter = %f\n",freqcenter);
    //    FILE *fp ;
    //    char file[50];
    //    fp= fopen(file, "w");
    //    fp= fopen("sumcorrdata.dat", "w");
    //    if (fp)
    //        {
    //        fwrite(sumCorr,sizeof(int),800*128,fp);
    //        fflush(fp);
    //        fclose(fp);
    //        }
    // 标校信号幅度和相位
    // float refAmp[NCorr * corrFlag], refPhase[NCorr * corrFlag];
    // if (detectedParam.refFreq > 0)
    // {
    //     calcRefAmpPhase(detectedParam.refFreq, devParam.cenFreqOnAllCh, dF, sumCorrAmp, sumCorrPhase, nRows, nCols, refAmp, refPhase);
    //     // printf("refAmp=%.1f,%.1f,%.1f,%.1f,%.1f, refPhase=%.2f,%.2f,%.2f,%.2f,%.2f\n",refAmp[0],refAmp[1],refAmp[2],refAmp[3],refAmp[4],refPhase[0],refPhase[1],refPhase[2],refPhase[3],refPhase[4]);
    //     if (detectedParam.writeRef > 0)
    //         writeLogRefAmpPhase(refAmp, refPhase);
    // }

    // 写相关数据
    //      if (detectedParam.writeSumCorr == 1)
    //      {
    //          SumCorrsign = 0;
    //          writeSumCorrToFile(sumCorr, nRows*corrFlag, detectedParam.filePrefix, SumCorrsign, SumCorrsize);
    //      }
    //  //续写最近N个相关数据
    //  if (detectedParam.SumCorrContinuation == 1)
    //  {
    //      SumCorrsize = detectedParam.SumCorrsize;
    //      SumCorrsign = 1;
    //      writeSumCorrToFile(sumCorr, nRows*corrFlag, detectedParam.filePrefix, SumCorrsign ,SumCorrsize);
    //  }
    // 写新相关数据
    //      if (detectedParam.writeNewSumCorr == 1)
    //      {
    //          SumCorrsign = 2;
    //          writeSumCorrToFile(sumCorr, nRows*corrFlag, detectedParam.filePrefix, SumCorrsign ,SumCorrsize, NewSaveAMP, NewSavePHASE);
    //      }

    // 	float **CorrAmp = sumCorrAmp[0];
    // 	float **CorrPhase = sumCorrPhase[0];
    if (trackSuccess == 0) // 若解帧正确，则调用无人机检测算法
                           //      if(1)//若解帧正确，则调用无人机检测算法
    {
        // t1 = clock();
        // 图传检测
        //         printf("************图传检测**********\n");
        //        t1 =clock();
        vidDetect(uavPulse, &nUavPulse, (float *)sumCorr, sumCorrAmp, sumCorrPhase, nRows, nCols, devParam.cenFreqOnAllCh, devParam.gainOnAllCh, dt, dF, UAVtypes, nUAV, detectedParam);
        //        t2=clock();
        //        printf("vidDetect use time = %f ms\n",(double)(t2 - t1) / CLOCKS_PER_SEC*1000);
        // t2 = clock();
        // useTime = (double)(t2 - t1) / CLOCKS_PER_SEC;
        // printf("vidDetect use time %f ms\n", useTime * 1000);
        // printUavPulse(uavPulse, nUavPulse, UAVtypes);//打印图传检测结果
        // if (detectedParam.writeVidPulse > 0)
        //     writeLogUavPulse(uavPulse, nUavPulse, UAVtypes, devParam, detectedParam);  //图传检测结果写日志文件

        t1 = clock();
        // 遥控检测
        ctrDetect(uavCtrPulse, &nUavCtrPulse, (float *)sumCorr, sumCorrAmp, sumCorrPhase, nRows, nCols, devParam.cenFreqOnAllCh, devParam.gainOnAllCh, dt, dF, UAVtypes, nUAV, detectedParam);
        t2 = clock();
        // useTime = (double)(t2 - t1) / CLOCKS_PER_SEC;
        // printf("ctrDetect use time %f ms\n", useTime * 1000);
        // printCtrPulse(uavCtrPulse, nUavCtrPulse, UAVtypes);//打印遥控检测结果

        // if (checkBoxObj(boxObjs) == 0)
        // {
        //     //框选检测
        //     t1 = clock();
        //     boxDetect(uavBoxPulse, nBoxPulse, UAVtypes, nUAV, sumCorr, sumCorrAmp, sumCorrPhase, nRows, nCols, boxObjs, devParam.cenFreqOnAllCh, devParam.gainOnAllCh, dt, dF, detectedParam);
        //     t2 = clock();
        //     useTime = (double)(t2 - t1) / CLOCKS_PER_SEC;
        //     printf("boxDetect use time %f ms\n", useTime * 1000);
        // }
    }
    // 目标跟踪
    float countTime = timeCount2ms(devParam.timeCount, devParam.timeCountVersion);
    //    t1 =clock();

    // 调试用
    //    float compassAng[360] = {339,344,86,24,91,80,2,329,0,12};
    ////    float sigAmp[360] = {102.74,103.95,101.47,109.88,100.16,102.98,107.60,100.161,107.267,108.987};
    //    static int ind = 0;
    //    compassAngle = compassAng[ind];
    //    struct pulseGroup uavPulse1;

    //    uavPulse1.id = 1001;                                                     //无人机脉冲id
    //    uavPulse1.uavIndex = 0;                                               //无人机在特征库内的编号
    //    uavPulse1.azimuth = 0;                                              //无人机遥控方位角
    //    uavPulse1.range = 20;                                                //无人机遥控距离
    //    uavPulse1.onAnt[0] = 1;                                             //是否在相应天线上检出
    //    uavPulse1.antFace[0] = 1;                               //面对的天线，测角选择的天线
    //    uavPulse1.freq[0] = 5800;                                //跳频点脉冲频率,MHz
    //    uavPulse1.pulseBW = 1;                                              //带宽,MHz
    //    uavPulse1.pulseTime[0] = 1;                           //脉冲时间,ms
    //    uavPulse1.pulseW[0] = 2.2;                              //脉宽,ms
    //    uavPulse1.meanAmp[0][0] = sigAmp[ind];                        //所有天线上的所有脉冲平均幅度,dB
    //    uavPulse1.meanAmpComplex[0][0] = sigAmp[ind];                 //所有天线上的所有脉冲复数平均幅度,dB
    //    uavPulse1.meanPhase[0][0] = 1;                      //所有天线上的所有脉冲的平均相位，rad
    //    uavPulse1.phaseVar[0][0] = 1;                       //所有天线上所有脉冲的相位方差
    //    uavPulse1.phaseHist[0][0][PhaseBinNum] = 1;         //相位直方图
    //    uavPulse1.weightedPhaseHist[0][0][PhaseBinNum] = 1; //加权相位直方图
    //    uavPulse1.angle[0] = 0;                               //脉冲的方位角,度
    //    uavPulse1.distance[0] = 100;                            //脉冲计算的距离,m
    //    uavPulse1.possibility = 80;                                           //目标的置信度，%                                                //脉冲个数
    //    uavPulse1.nPulse = 1;
    //    ind++;
    //    if(ind == 11) ind = 0;
    //    uavPulse[0] = uavPulse1;
    //    nUavPulse = 1;
    //    printf("amp = %f\n", uavPulse[0].meanAmp[0][0]);
    nObj = objTrack(countTime, devParam.frameCount, recordedObj, &nObjRecord, objOut, uavPulse, nUavPulse, uavCtrPulse, nUavCtrPulse, UAVtypes, detectedParam);
    //    t2=clock();
    //    printf("objTrack use time = %f ms\n",(double)(t2 - t1) / CLOCKS_PER_SEC*1000);
    // printf("objTrack finished\n");
    printf("nObj =%d\n", nObj);
    // 整合目标信息
    //  printf("整合目标信息\n");
    //    t1 =clock();

    // devParam.angleValid = 0;
    // devParam.compassAngle = compassAngle;
    // calcAngleShield(recordedObj, nObjRecord, objOut, nObj);

    genObj(&messagePtr, objOut, nObj, UAVtypes, devParam, detectedParam);
    //    t2=clock();
    //    printf("genObj use time = %f ms\n",(double)(t2 - t1) / CLOCKS_PER_SEC*1000);
    //     if(detectedParam.writeDeblurLog >= 1)
    //         printObjLog(*messagePtr);
    //     else
    printObj(messagePtr);

    /***************************************组帧****************************************************/
    // for (int n = 0; n < 800; n++)
    // {
    //     for (int m = 0; m < 128; m++)
    //     {
    //         if (m == 1)
    //             udpData[n * 128 + m] = n + 1;
    //         if (m == 127)
    //             udpData[n * 128 + m] = (int)freqcenter + 1;
    //         //                if(m<126&&m>1)
    //         //                    udpData[n*128+m] =(int)10*log10(udpData[n*128+m]);
    //     }
    // }

    //        FILE *fp;
    //        char filepath[100] = "UavudpData.dat";
    //        fp = fopen(filepath,"w");
    //        if(fp)
    //        {
    //            fwrite(udpData,sizeof (int),800*128,fp);
    //        }
    //        fclose(fp);

    /***********************************用UDP发送数据************************************************/
    // udp_data_send_start(udp_send_ip);
    // for (int n = 0; n < 400; n++)
    // {
    //     udp_data_send_ip((uint8_t *)(udpData + n * 128 * 2), sizeof(uint8_t) * 128 * 4 * 2, udp_send_ip);
    // }
    // udp_data_send_end(udp_send_ip);
    // if (nObj > 0)
    //     udp_data_send_ip((uint8_t *)(&messagePtr), sizeof(uint8_t) * 1232, udp_send_ip); // udp发送目标信息

    /*----------------------------------发送到ui的信息----------------------------------------------*/
    // printf("发送到ui的信息\n");
    genObjForUi(&objForUi, &messagePtr, objOut, nObj, uavPulse, uavCtrPulse);
    objForUi.frameCount = devParam.frameCount;

    //    udp_data_send((char * )&messagePtr,sizeof (messagePtr));

    // printf("*******发送到ui的信息完成********\n");
    // //加入框选目标信息
    // boxPulseToObj(uavBoxPulse, nBoxPulse, messagePtr, objForUi, detectedParam.antennaNum);
    /*******************************跟踪结果写文件*******************************/
    if (detectedParam.writeObj > 0)
        writeLogObj(&messagePtr, devParam);
    printf("*******uavend**\n");
    //
    //
    // 	// appendBoxToCtrPulse(uavCtrPulse, nUavCtrPulse, uavBoxPulse, nBoxPulse, UAVtypes, nUAV);
    //     if (detectedParam.writeCtrPulse > 0)
    //             writeLogUavCtrPulse(uavCtrPulse, nUavCtrPulse, UAVtypes, devParam, detectedParam);

    // if (detectedParam.writeCaliPulse > 0)
    // 	writeLogAutoCalibration(messagePtr,uavPulse, nUavPulse,uavCtrPulse, nUavCtrPulse, UAVtypes, devParam, detectedParam);

    // 显示主界面
    //  if (detectedParam.showImage > 0  || detectedParam.saveImage>0)
    //  {
    //      //主界面
    //      showMainWindow(objOut, nObj, uavPulse, nUavPulse, uavCtrPulse, nUavCtrPulse, UAVtypes, devParam, detectedParam);
    //      // printf("showMainWindow finished\n");
    //      //频谱
    //      showImg(sumCorrAmp, sumCorrPhase, nRows, nCols, devParam.cenFreqOnAllCh, dt, dF, uavPulse, nUavPulse, uavCtrPulse, nUavCtrPulse, detectedParam);
    //      // printf("showImg finished\n");
    //  }
    //  else
    //      destroyAllWindows();

    // 清除动态内存
    //      freeGrid(sumCorrAmp, NCorr * corrFlag, nRows, nCols);
    //      freeGrid(sumCorrPhase, NCorr * corrFlag, nRows, nCols);
    //     freeMatrix(sumCorrAmp, nRows);
    //     freeMatrix(sumCorrPhase, nRows);
    // free(detectedParam.mask);
    // detectedParam.mask = NULL;
    // free(detectedParam.hpMask);
    // detectedParam.hpMask = NULL;
    //     free(uavPulse);
    //     uavPulse = NULL;
    //     free(uavCtrPulse);
    //     uavCtrPulse = NULL;
    //     free(NewSaveAMP);
    //     free(NewSavePHASE);
    return 0;
}
