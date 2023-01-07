#include "readDetectParam.h"
#include "paramRead.h"
#include "uavDetect.h"
#include "string.h"
// #include "mysql/mysql.h"
// #define INI_PARAM
// #ifndef SQL_LIB
// 	MYSQL my_connection;
// 	MYSQL calibsqlconnection;
// #endif

/*
 * 根据天线AntennaShape标记确定天线的组数和测向类型。
 * 测向类型：0表示2.4G相位，5.8G幅度;1表示2.4G相位，5.8G相位;2表示只支持800M的4天线相位;3表示表示800-6000M相位;4表示双层天线;5表示阿基米德螺旋天线;6表示侦诱一体天线(只用一组数据)
 */
int checkAntennaTyp(int *antennaNum, int *doaTyp, int antFlag)
{
    int antennaNum0 = *antennaNum;
    int doaTyp0 = *doaTyp;
    if (antFlag == 0)
    {
        doaTyp0 = 0;
        antennaNum0 = 5;
    }
    else if (antFlag == 1)
    {
        doaTyp0 = 1;
        antennaNum0 = 5;
    }
    else if (antFlag == 2)
    {
        doaTyp0 = 2;
        antennaNum0 = 4;
    }
    else if (antFlag == 3)
    {
        doaTyp0 = 3;
        antennaNum0 = 4;
    }
    else if (antFlag == 4)
    {
        doaTyp0 = 4;
        antennaNum0 = 4;
    }
    else if (antFlag == 5)
    {
        doaTyp0 = 5;
        antennaNum0 = 8;
    }
    else if (antFlag == 6)
    {
        doaTyp0 = 6;
        antennaNum0 = 1;
    }
    else if (antFlag == 7)
    {
        doaTyp0 = 7;
        antennaNum0 = 4;
    }
    else
    {
        *antennaNum = antennaNum0;
        *doaTyp = doaTyp0;
        return -1;
    }
    *antennaNum = antennaNum0;
    *doaTyp = doaTyp0;
    return 0;
}

/*
 * 打印机型库参数
 */
void printUavlib(struct UAVLib *UAVtypes, int nUAV)
{
    int i = 0, j = 0;
    for (i = 0; i < nUAV; i++)
    {
        printf("id=%d\n", i);
        printf("name=%s\n", UAVtypes[i].name);
        printf("vidOrCtr=%s\n", UAVtypes[i].vidOrCtr);
        printf("freqPoints=");
        for (j = 0; j < UAVtypes[i].nfreqp; j++)
            printf("%.1f,", UAVtypes[i].freqPoints[j]);
        printf("\n");
        printf("nfreqp=%d\n", UAVtypes[i].nfreqp);
        printf("isFixedFreq=%d\n", UAVtypes[i].isFixedFreq);
        printf("hoppType=%d\n", UAVtypes[i].hoppType);
        printf("pulseW=");
        for (j = 0; j < UAVtypes[i].nPulseW; j++)
            printf("%.2f,", UAVtypes[i].pulseW[j]);
        printf("\n");
        printf("isFixedPulseW=%d\n", UAVtypes[i].isFixedPulseW);
        printf("pulseT=");
        for (j = 0; j < UAVtypes[i].nPulseT; j++)
            printf("%.2f,", UAVtypes[i].pulseT[j]);
        printf("\n");
        printf("nPulseT=%d\n", UAVtypes[i].nPulseT);
        printf("pulseBW=");
        for (j = 0; j < UAVtypes[i].nPulseBW; j++)
            printf("%.1f,", UAVtypes[i].pulseBW[j]);
        printf("\n");
        printf("pulseWErr=");
        for (j = 0; j < UAVtypes[i].nPulseW; j++)
            printf("%.2f,", UAVtypes[i].pulseWErr[j]);
        printf("\n");
        printf("meetPulseW=");
        for (j = 0; j < UAVtypes[i].nPulseW; j++)
            printf("%d,", UAVtypes[i].meetPulseW[j]);
        printf("\n");
        printf("pulseTErr=%.1f\n", UAVtypes[i].pulseTErr);
        printf("meetHopp=%d\n", UAVtypes[i].meetHopp);
        printf("freqErr=%.1f\n", UAVtypes[i].freqErr);
        printf("useMethod=%d\n", UAVtypes[i].useMethod);
        printf("SNR=%.1f\n", UAVtypes[i].SNR);
        printf("onOroff=%d\n\n", UAVtypes[i].onOroff);
    }
}

#ifdef SQL_LIB
int readCalibLibSql(struct detectParams *detectedParam, char *server_host, char *sql_user_name, char *sql_password, char *db_name)
{
    MYSQL_RES *res_ptr;
    MYSQL_ROW sqlrow;
    int res;
    int field_count = 0;
    int num = 0;
    int sqlType = 0;
    float freq;
    int freqNum = 0;
    int chInd = 0;
    int newFreqPFlag = 0;
    int antennaNum = detectedParam->antennaNum;
    static char connect_flag = 0;
    if (connect_flag == 0)
    {
        if (mysql_init(&calibsqlconnection) == NULL)
            return -1;
        if (mysql_real_connect(&calibsqlconnection, server_host, sql_user_name, sql_password, db_name, 0, NULL, 0))
        {
            connect_flag = 1;
        }
    }
    // mysql_init(&my_connection);
    if (connect_flag)
    {
        // printf("------------------connection success\n");
        mysql_set_character_set(&calibsqlconnection, "utf8");
        res = mysql_query(
            &calibsqlconnection, "SELECT\
                    id,freq, Pitch_angle, channels, channel_number, ph_ref_azimuth, db, init_phase, ph_comp_poly_P4, ph_comp_poly_P3, ph_comp_poly_P2,\
                    ph_comp_poly_P1, ph_comp_poly_P0, ph_comp_fit_accuracy, ph_comp_lower_limit, ph_comp_upper_limit, amp_comp_poly_P4, amp_comp_poly_P3,\
                    amp_comp_poly_P2, amp_comp_poly_P1, amp_comp_poly_P0, amp_comp_fit_accuracy, amp_comp_lower_limit, amp_comp_upper_limit, amp_comp_beam_center,\
                    amp_comp_beam_width, max_amp_of_pattern, pattern_fit_accuracy, slope_with_freq FROM calibration");

        if (res)
        {
            sqlType = 1;
            res = mysql_query(
                &calibsqlconnection, "SELECT\
                        id,freq, Pitch_angle, channels, channel_number, ph_ref_azimuth, db, init_phase, ph_comp_poly_P4, ph_comp_poly_P3, ph_comp_poly_P2,\
                        ph_comp_poly_P1, ph_comp_poly_P0, ph_comp_fit_accuracy, ph_comp_lower_limit, ph_comp_upper_limit, amp_comp_poly_P4, amp_comp_poly_P3,\
                        amp_comp_poly_P2, amp_comp_poly_P1, amp_comp_poly_P0, amp_comp_fit_accuracy, amp_comp_lower_limit, amp_comp_upper_limit, amp_comp_beam_center,\
                        amp_comp_beam_width, max_amp_of_pattern, pattern_fit_accuracy, slope_with_freq FROM calibration");
        }
        if (res)
        {
            printf("Select Error: %s\n", mysql_error(&calibsqlconnection));
            mysql_close(&calibsqlconnection);
            connect_flag = 0;
        }
        else
        {
            res_ptr = mysql_use_result(&calibsqlconnection);
            if (res_ptr)
            {
                while ((sqlrow = mysql_fetch_row(res_ptr)))
                {
                    freq = atof(sqlrow[1]);
                    chInd = atoi(sqlrow[4]) - 1;
                    for (int i = 0; i < freqNum + 1; i++)
                    {
                        if (detectedParam->calibParamNew[i].calibFreq == freq || (freqNum == 0 && newFreqPFlag == 0) || (freqNum == i))
                        {
                            detectedParam->calibParamNew[i].calibFreq = freq;
                            detectedParam->calibParamNew[i].phRefAzi[chInd] = atof(sqlrow[5]);
                            detectedParam->calibParamNew[i].calibD[chInd] = atof(sqlrow[6]);
                            detectedParam->calibParamNew[i].calibInitPh[chInd] = atof(sqlrow[7]);
                            detectedParam->calibParamNew[i].phPoly[chInd][0] = atof(sqlrow[8]);
                            detectedParam->calibParamNew[i].phPoly[chInd][1] = atof(sqlrow[9]);
                            detectedParam->calibParamNew[i].phPoly[chInd][2] = atof(sqlrow[10]);
                            detectedParam->calibParamNew[i].phPoly[chInd][3] = atof(sqlrow[11]);
                            detectedParam->calibParamNew[i].phPoly[chInd][4] = atof(sqlrow[12]);
                            detectedParam->calibParamNew[i].phFitAccuracy[chInd] = atof(sqlrow[13]);
                            detectedParam->calibParamNew[i].phLowUpLimit[chInd][0] = atof(sqlrow[14]);
                            detectedParam->calibParamNew[i].phLowUpLimit[chInd][1] = atof(sqlrow[15]);
                            detectedParam->calibParamNew[i].ampPoly[chInd][0] = atof(sqlrow[16]);
                            detectedParam->calibParamNew[i].ampPoly[chInd][1] = atof(sqlrow[17]);
                            detectedParam->calibParamNew[i].ampPoly[chInd][2] = atof(sqlrow[18]);
                            detectedParam->calibParamNew[i].ampPoly[chInd][3] = atof(sqlrow[19]);
                            detectedParam->calibParamNew[i].ampPoly[chInd][4] = atof(sqlrow[20]);
                            detectedParam->calibParamNew[i].ampFitAccuracy[chInd] = atof(sqlrow[21]);
                            detectedParam->calibParamNew[i].ampLowUpLimit[chInd][0] = atof(sqlrow[22]);
                            detectedParam->calibParamNew[i].ampLowUpLimit[chInd][1] = atof(sqlrow[23]);
                            detectedParam->calibParamNew[i].ampBeamCent[chInd] = atof(sqlrow[24]);
                            detectedParam->calibParamNew[i].ampBeamW[chInd] = atof(sqlrow[25]);
                            detectedParam->calibParamNew[i].maxAmpPattern[chInd] = atof(sqlrow[26]);
                            detectedParam->calibParamNew[i].patternFitAccuracy[chInd] = atof(sqlrow[27]);
                            detectedParam->calibParamNew[i].slopeFreq[chInd] = atof(sqlrow[28]);
                            if (freqNum == 0 || (freqNum == i))
                            {
                                newFreqPFlag = 1;
                            }
                            break;
                        }
                        // if (detectedParam->calibParamNew[i].calibFreq <= 0)
                        //     break;
                    }
                    if (newFreqPFlag == 1)
                    {
                        freqNum++;
                        newFreqPFlag = 0;
                    }
                }
                mysql_free_result(res_ptr);
            }
            // mysql_close(&calibsqlconnection);

            // printUavlib(UAVtypes, nUAV);

            return 1;
        }
    }
    else
    {
        printf("calibrationSql connection failed!\n");
        if (mysql_errno(&calibsqlconnection))
        {
            printf("Connection error %d: %s\n", mysql_errno(&calibsqlconnection), mysql_error(&calibsqlconnection));
        }
    }

    return 0;
}
#endif

#ifdef SQL_LIB
int getCalibSql(struct detectParams *detectedParam)
{
    char server_host[20] = "127.0.0.1";
    char sql_user_name[20] = "root";
    char sql_password[20] = "bekl077";
    char db_name[50] = "calibLib";

    readCalibLibSql(detectedParam, server_host, sql_user_name, sql_password, db_name);
    return 0;

    // printf("nUAV=%d\n", nUAV);
}
#endif

#ifdef INI_LIB
int getUavlib(char *fileName, struct UAVLib *UAVtypes)
{
    //    printf("***************ndef INI_PARAM getUavlib***********\n");
    //    printf("******************%s****************\n", fileName);
    int i, j;
    int nUAV = 0;
    int printUavFeature = 0;
    int allSwitch = 0;

    // 机型字段
    char name[50];                       // 名称
    char vidOrCtr[20];                   // 图传还是遥控，vid或者ctr
    float freqPoints[100];               // 频点,MHz
    int nfreqp;                          // freqPoints频点个数
    int isFixedFreq;                     // 是否固定频点,>0则是固定频点，=0则只取freqPoints[0]和freqPoints[1]作为频带范围
    int hoppType;                        // 跳频类型，仅对useMethod=2有效。0则要按照freqPoints顺序跳频，1则目标的所有频点频率一致，2则只需要在频段内按周期跳频
    float pulseW[maxPulseWNumInLib];     // 脉宽，ms
    int nPulseW;                         // 脉宽种类数
    int isFixedPulseW;                   // 脉宽是否固定，已经不产生作用（废弃状态）
    float pulseT[5];                     // 脉冲出现的周期,ms。可以有多个，比如（3,4），表示脉冲1与2起始时间相差3ms，脉冲2与3相差4ms,然后类推，脉冲3与4相差3ms
    int nPulseT;                         // 周期个数，如pulseT=(3,4,5)，则nPulseT=3
    float pulseBW[5];                    // 带宽,MHz。针对比如mavic带宽可变的情况
    int nPulseBW;                        // pulseBW总带宽的个数，若大于1则useMethod=1时带宽需要额外搜索确定
    float pulseWErr[maxPulseWNumInLib];  // 脉宽误差,ms。比如pulseW=(3,4)，pulseWErr=(0.1,0.2)则，脉宽范围是3±0.1和4±0.2
    float meetPulseW[maxPulseWNumInLib]; // 满足脉宽的最小次数，>=。比如pulseW=(3,4)，meetPulseW=(2,5)，则必须出现3ms的脉宽不少于2次，且4ms的脉宽不少于5次
    float pulseTErr;                     // 脉冲起始时间满足周期规律的误差,ms
    int meetHopp;                        // 满足周期的脉冲最小个数，>=
    float freqErr;                       // 频率误差范围,MHz。仅对useMethod=2有效
    int useMethod;                       // 列方向分割方法1，主要用于检测图传。带宽较小的信号检测方法2,主要用于遥控检测
    float SNR;                           // 信噪比阈值，仅对useMethod=1有效，高于此信噪比才会被检测到
    int onOroff;                         // 开关,>0则打开
    int flag;

    char sTitle[50]; // 字段标题
    char key[30] = {0};

    // 打印
    char printControl[] = "printControl";
    strcpy(key, "printUavFeature");
    printUavFeature = GetIniKeyInt(printControl, key, fileName);
    strcpy(key, "allSwitch");
    allSwitch = GetIniKeyInt(printControl, key, fileName);

    // 读取机型参数
    for (i = 0; i < 100; i++)
    {
        sprintf(sTitle, "UAV%d", i + 1);
        strcpy(key, "name");
        flag = GetIniKeyString(name, sTitle, key, fileName);
        if (flag != 0)
            continue;
        strcpy(key, "vidOrCtr");
        GetIniKeyString(vidOrCtr, sTitle, key, fileName);
        strcpy(key, "nfreqp");
        nfreqp = GetIniKeyInt(sTitle, key, fileName);
        strcpy(key, "freqPoints");
        GetIniKeyFloatArray(sTitle, key, freqPoints, nfreqp, fileName);
        strcpy(key, "isFixedFreq");
        isFixedFreq = GetIniKeyInt(sTitle, key, fileName) > 0;
        strcpy(key, "hoppType");
        hoppType = GetIniKeyInt(sTitle, key, fileName);
        strcpy(key, "nPulseW");
        nPulseW = GetIniKeyInt(sTitle, key, fileName);
        strcpy(key, "pulseW");
        GetIniKeyFloatArray(sTitle, key, pulseW, nPulseW, fileName);
        strcpy(key, "isFixedPulseW");
        isFixedPulseW = GetIniKeyInt(sTitle, key, fileName) > 0;
        strcpy(key, "nPulseT");
        nPulseT = GetIniKeyInt(sTitle, key, fileName);
        strcpy(key, "pulseT");
        GetIniKeyFloatArray(sTitle, key, pulseT, nPulseT, fileName);
        strcpy(key, "nPulseBW");
        nPulseBW = GetIniKeyInt(sTitle, key, fileName);
        strcpy(key, "pulseBW");
        GetIniKeyFloatArray(sTitle, key, pulseBW, nPulseBW, fileName);
        strcpy(key, "pulseWErr");
        GetIniKeyFloatArray(sTitle, key, pulseWErr, nPulseW, fileName);
        strcpy(key, "meetPulseW");
        GetIniKeyFloatArray(sTitle, key, meetPulseW, nPulseW, fileName);
        strcpy(key, "pulseTErr");
        pulseTErr = GetIniKeyFloat(sTitle, key, fileName);
        strcpy(key, "meetHopp");
        meetHopp = GetIniKeyInt(sTitle, key, fileName);
        strcpy(key, "freqErr");
        freqErr = GetIniKeyFloat(sTitle, key, fileName);
        strcpy(key, "useMethod");
        useMethod = GetIniKeyInt(sTitle, key, fileName);
        strcpy(key, "SNR");
        SNR = GetIniKeyFloat(sTitle, key, fileName);
        strcpy(key, "onOroff");
        onOroff = GetIniKeyInt(sTitle, key, fileName);

        if (flag == 0 && (allSwitch > 0 || onOroff > 0))
        {
            // 赋值
            strcpy(UAVtypes[nUAV].name, name);
            strcpy(UAVtypes[nUAV].vidOrCtr, vidOrCtr);
            for (j = 0; j < nfreqp; j++)
                UAVtypes[nUAV].freqPoints[j] = freqPoints[j];
            UAVtypes[nUAV].nfreqp = nfreqp;
            UAVtypes[nUAV].isFixedFreq = isFixedFreq;
            UAVtypes[nUAV].hoppType = hoppType;
            for (j = 0; j < nPulseW; j++)
            {
                UAVtypes[nUAV].pulseW[j] = pulseW[j];
                UAVtypes[nUAV].pulseWErr[j] = pulseWErr[j];
                UAVtypes[nUAV].meetPulseW[j] = meetPulseW[j];
            }
            UAVtypes[nUAV].nPulseW = nPulseW;
            UAVtypes[nUAV].isFixedPulseW = isFixedPulseW;
            UAVtypes[nUAV].nPulseT = nPulseT;
            for (j = 0; j < nPulseT; j++)
                UAVtypes[nUAV].pulseT[j] = pulseT[j];
            for (j = 0; j < nPulseBW; j++)
                UAVtypes[nUAV].pulseBW[j] = pulseBW[j];
            UAVtypes[nUAV].nPulseBW = nPulseBW;
            UAVtypes[nUAV].pulseTErr = pulseTErr;
            UAVtypes[nUAV].meetHopp = meetHopp;
            UAVtypes[nUAV].freqErr = freqErr;
            UAVtypes[nUAV].useMethod = useMethod;
            UAVtypes[nUAV].SNR = SNR;
            UAVtypes[nUAV].onOroff = onOroff;
            nUAV++;
        }
    }

    // 打印
    if (printUavFeature > 0)
    {
        printUavlib(UAVtypes, nUAV);
    }

    return nUAV;
}

#endif
/*
   读取算法参数配置文件
 */
#ifdef INI_PARAM
int getDetectParam(struct detectParams *param0, char *fileName)
{
    int i, j;
    struct detectParams param;
    char printControl[] = "printControl";
    char filter[] = "filter";
    char segment[] = "segment";
    char angleMeasure[] = "angleMeasure";
    char objTrack[] = "objTrack";
    char show[] = "show";
    char box[] = "box";
    char deblur[] = "deblur";
    char key[30] = {0};

    ////printControl
    // 打印读出的配置参数
    strcpy(key, "printParam");

    param.printParam = GetIniKeyInt(printControl, key, fileName);
    if (param.printParam < 0)
    {
        printf("detectParams.ini not found!\n");
        return -1;
    }

    strcpy(key, "writeIQ");
    param.writeIQ = GetIniKeyInt(printControl, key, fileName);
    strcpy(key, "corrOnOff");
    param.corrOnOff = GetIniKeyInt(printControl, key, fileName);
    strcpy(key, "gfskOnOff");
    param.gfskOnOff = GetIniKeyInt(printControl, key, fileName);
    strcpy(key, "wifiOnOff");
    param.wifiOnOff = GetIniKeyInt(printControl, key, fileName);

    // 相关文件名前缀
    strcpy(key, "filePrefix");
    GetIniKeyString(param.filePrefix, printControl, key, fileName);
    // 相关写文件，等于1每次都写文件，等于2则相关数据出错才写文件
    strcpy(key, "writeSumCorr");
    param.writeSumCorr = GetIniKeyInt(printControl, key, fileName);
    // 写新相关文件
    strcpy(key, "writeNewSumCorr");
    param.writeNewSumCorr = GetIniKeyInt(printControl, key, fileName);
    // 标校信号结果写文件
    strcpy(key, "writeRef");
    param.writeRef = GetIniKeyInt(printControl, key, fileName);
    // 删除相关文件，配置容量大小
    strcpy(key, "SumCorrsize");
    param.SumCorrsize = GetIniKeyInt(printControl, key, fileName);
    strcpy(key, "SumCorrContinuation");
    param.SumCorrContinuation = GetIniKeyInt(printControl, key, fileName);
    // 无人机图传脉冲检测结果写文件
    strcpy(key, "writeVidPulse");
    param.writeVidPulse = GetIniKeyInt(printControl, key, fileName);
    // 无人机遥控脉冲检测结果写文件
    strcpy(key, "writeCtrPulse");
    param.writeCtrPulse = GetIniKeyInt(printControl, key, fileName);
    // 外场自动标校用日志
    strcpy(key, "writeCaliPulse");
    param.writeCaliPulse = GetIniKeyInt(printControl, key, fileName);
    // 无人机目标写文件
    strcpy(key, "writeObj");
    param.writeObj = GetIniKeyInt(printControl, key, fileName);
    // 解模糊日志写文件
    strcpy(key, "writeDeblurLog");
    param.writeDeblurLog = GetIniKeyInt(printControl, key, fileName);
    // 截图保存，若等于1,则保存每张界面截图。若大于2，则只在检测到无人机图传或遥控时保存界面截图，循环保存这么多图片
    strcpy(key, "saveImage");
    param.saveImage = GetIniKeyInt(printControl, key, fileName);
    // 疑似目标(测频测带宽不准的真实目标)名称前缀
    strcpy(key, "objPrefix");
    GetIniKeyString(param.objPrefix, printControl, key, fileName);

    ////filter
    // 高斯滤波模板高度
    strcpy(key, "maskH");
    param.maskH = GetIniKeyInt(filter, key, fileName);
    // 模板宽度
    strcpy(key, "maskW");
    param.maskW = GetIniKeyInt(filter, key, fileName);
    // 模板参数
    strcpy(key, "mask");
    param.mask = (float *)malloc(param.maskH * param.maskW * sizeof(float));
    GetIniKeyFloatArray(filter, key, param.mask, param.maskH * param.maskW, fileName);
    // 高通滤波模板高度
    strcpy(key, "hpMaskH");
    param.hpMaskH = GetIniKeyInt(filter, key, fileName);
    // 模板宽度
    strcpy(key, "hpMaskW");
    param.hpMaskW = GetIniKeyInt(filter, key, fileName);
    // 模板参数
    strcpy(key, "hpMask");
    param.hpMask = (float *)malloc(param.hpMaskH * param.hpMaskW * sizeof(float));
    GetIniKeyFloatArray(filter, key, param.hpMask, param.hpMaskH * param.hpMaskW, fileName);
    // 剔除野值再滤波门限
    strcpy(key, "filterThresh");
    param.filterThresh = GetIniKeyFloat(filter, key, fileName);

    ////segment
    // 算法需要的数据时长,ms
    strcpy(key, "dataTime");
    param.dataTime = GetIniKeyFloat(segment, key, fileName);
    // 阈值更新的最小变化量
    strcpy(key, "DT");
    param.DT = GetIniKeyFloat(segment, key, fileName);
    // 最大迭代次数
    strcpy(key, "MaxIter");
    param.MaxIter = GetIniKeyInt(segment, key, fileName);
    // 填充的最大沟壑点数
    strcpy(key, "Delt");
    param.Delt = GetIniKeyInt(segment, key, fileName);
    // 最小脉宽的点数，40表示2ms
    strcpy(key, "MinW");
    param.MinW = GetIniKeyInt(segment, key, fileName);
    // 无人机脉冲列合并时，q1起点的误差点数容限
    strcpy(key, "q1IndexErr");
    param.q1IndexErr = GetIniKeyInt(segment, key, fileName);
    // q1满足误差容限的最低累计次数，无人机脉冲列合并条件
    strcpy(key, "qMeetTimes");
    param.qMeetTimes = GetIniKeyInt(segment, key, fileName);
    // 对于中心频点不固定的图传，搜索带宽时平均幅度的容差,dB
    strcpy(key, "ampErr");
    param.ampErr = GetIniKeyFloat(segment, key, fileName);
    strcpy(key, "checkBWAmpErr");
    param.checkBWAmpErr = GetIniKeyFloat(segment, key, fileName);
    // 遥控器检测的信噪比门限
    strcpy(key, "ctrSNR");
    param.ctrSNR = GetIniKeyFloat(segment, key, fileName);
    // 遥控器检测的信噪比门限
    strcpy(key, "archimedeanSNR");
    param.archimedeanSNR = GetIniKeyFloat(segment, key, fileName);
    // 遥控器检测最小脉宽
    strcpy(key, "minCtrPulseW");
    param.minCtrPulseW = GetIniKeyFloat(segment, key, fileName);
    // 遥控器检测最大脉宽
    strcpy(key, "maxCtrPulseW");
    param.maxCtrPulseW = GetIniKeyFloat(segment, key, fileName);

    ////angleMeasure
    // 天线1中心线的方位角
    strcpy(key, "ant1Angle");
    param.ant1Angle = GetIniKeyFloat(angleMeasure, key, fileName);

    char filePackageIni[] = "./package.ini";
    char angleParam[] = "ANGLE";
    char devParam[] = "DEVICE";
    char recvParam[] = "RECV";
    // 读取测向参数
    strcpy(key, "miu");
    param.miu = GetIniKeyFloat(angleParam, key, filePackageIni);
    if (param.miu < 0)
    {
        printf("package.ini not found!\n");
        param.printParam = -1;
        return -1;
    }
    // 侧向结果后处理
    strcpy(key, "Lateral");
    param.Lateral = GetIniKeyInt(angleParam, key, filePackageIni);
    strcpy(key, "initial_azi");
    param.initial_azi = GetIniKeyFloat(angleParam, key, filePackageIni);
    strcpy(key, "azi_step");
    param.azi_step = GetIniKeyFloat(angleParam, key, filePackageIni);
    strcpy(key, "rand_err");
    param.rand_err = GetIniKeyFloat(angleParam, key, filePackageIni);

    // 过渡带角度
    strcpy(key, "overlapAngle");
    param.overlapAngle = GetIniKeyFloat(angleParam, key, filePackageIni);
    // 是否
    strcpy(key, "isUseAmpLib");
    param.isUseAmpLib = GetIniKeyFloat(angleParam, key, filePackageIni);
    // 是否启动波素角度偏移修正
    strcpy(key, "isUseAziLib");
    param.isUseAziLib = GetIniKeyFloat(angleParam, key, filePackageIni);
    // 是否优化测频测带宽
    strcpy(key, "isCaliFreqBw");
    param.isCaliFreqBw = GetIniKeyFloat(angleParam, key, filePackageIni);
    // 遥控带宽中频上报类型
    strcpy(key, "bwFreqType");
    param.bwFreqType = GetIniKeyFloat(angleParam, key, filePackageIni);
    // 选天线方案
    strcpy(key, "ampSelAntType");
    param.ampSelAntType = GetIniKeyInt(angleParam, key, filePackageIni);
    // 对周解模糊方案
    strcpy(key, "antShape7UseMethod");
    param.antShape7UseMethod = GetIniKeyFloat(angleParam, key, filePackageIni);
    // 滤波方法，0为幅度滤波，1为复数滤波，2为积分图像滤波
    strcpy(key, "filterMethod");
    param.filterMethod = GetIniKeyFloat(angleParam, key, filePackageIni);
    strcpy(key, "archi433AziMethod");
    param.archi433AziMethod = GetIniKeyFloat(angleParam, key, filePackageIni);
    // 标校类型选择
    strcpy(key, "calibType");
    param.calibType = GetIniKeyFloat(angleParam, key, filePackageIni);
    // 拟合测角开关
    strcpy(key, "fitPh");
    param.fitPh = GetIniKeyFloat(angleParam, key, filePackageIni);
    // 拟合测角频段范围
    strcpy(key, "fitFreqSpan");
    GetIniKeyFloatArray(angleParam, key, param.fitFreqSpan, 2, filePackageIni);
    // 屏蔽相位的频段个数
    strcpy(key, "shieldPhN");
    param.shieldPhN = GetIniKeyFloat(angleParam, key, filePackageIni);
    // 屏蔽相位的频段范围
    strcpy(key, "shieldPhFreqSpan");
    GetIniKeyFloatArray(angleParam, key, param.shieldPhFreqSpan, 40, filePackageIni);
    // 天线分层标校选择
    strcpy(key, "antSwOnOff");
    param.antSwOnOff = GetIniKeyInt(angleParam, key, filePackageIni);

    // 433M
    strcpy(key, "d433");
    param.d433 = GetIniKeyFloat(angleParam, key, filePackageIni);
    // 800M
    strcpy(key, "d800");
    param.d800 = GetIniKeyFloat(angleParam, key, filePackageIni);
    // 1800M
    strcpy(key, "d1800");
    param.d1800 = GetIniKeyFloat(angleParam, key, filePackageIni);
    // 2450MHz天线间距
    strcpy(key, "d2450");
    param.d2450 = GetIniKeyFloat(angleParam, key, filePackageIni);
    // 5800MHz天线间距
    strcpy(key, "d5800");
    param.d5800 = GetIniKeyFloat(angleParam, key, filePackageIni);
    // 910-935M天线间距
    strcpy(key, "d920coef");
    GetIniKeyFloatArray(angleParam, key, param.d920coef, 3, filePackageIni);
    // 初始相位433MHz
    strcpy(key, "initPhase433");
    GetIniKeyFloatArray(angleParam, key, param.initPhase433, 4, filePackageIni);
    // 初始相位800MHz
    strcpy(key, "initPhase800");
    GetIniKeyFloatArray(angleParam, key, param.initPhase800, 4, filePackageIni);
    // 800MHz四天线（夹角）拟合参数
    strcpy(key, "fitCoef800");
    GetIniKeyFloatArray(angleParam, key, param.fitCoef800, 24, filePackageIni);
    // 初始相位1800MHz
    strcpy(key, "initPhase1800");
    GetIniKeyFloatArray(angleParam, key, param.initPhase1800, 4, filePackageIni);
    // 初始相位2450MHz
    strcpy(key, "initPhase2450");
    GetIniKeyFloatArray(angleParam, key, param.initPhase2450, 5, filePackageIni);
    // 2450MHz四天线（夹角）拟合参数
    strcpy(key, "fitCoef2450");
    GetIniKeyFloatArray(angleParam, key, param.fitCoef2450, 24, filePackageIni);
    // 初始相位5800MHz
    strcpy(key, "initPhase5800");
    GetIniKeyFloatArray(angleParam, key, param.initPhase5800, 5, filePackageIni);
    // 5800MHz四天线（夹角）拟合参数
    strcpy(key, "fitCoef5800");
    GetIniKeyFloatArray(angleParam, key, param.fitCoef5800, 24, filePackageIni);
    // 幅度补偿系数
    strcpy(key, "offsetAmp433");
    GetIniKeyFloatArray(angleParam, key, param.offsetAmp433, 8, filePackageIni);
    strcpy(key, "offsetAmp800");
    GetIniKeyFloatArray(angleParam, key, param.offsetAmp800, 8, filePackageIni);
    strcpy(key, "offsetAmp1800");
    GetIniKeyFloatArray(angleParam, key, param.offsetAmp1800, 8, filePackageIni);
    strcpy(key, "offsetAmp2450");
    GetIniKeyFloatArray(angleParam, key, param.offsetAmp2450, 8, filePackageIni);
    strcpy(key, "offsetAmp5800");
    GetIniKeyFloatArray(angleParam, key, param.offsetAmp5800, 8, filePackageIni);

    // 单边幅度补偿系数
    strcpy(key, "offsetAmp433LOrR");
    GetIniKeyFloatArray(angleParam, key, param.offsetAmp433LOrR, 8, filePackageIni);
    strcpy(key, "offsetAmp800LOrR");
    GetIniKeyFloatArray(angleParam, key, param.offsetAmp800LOrR, 8, filePackageIni);
    strcpy(key, "offsetAmp915LOrR");
    GetIniKeyFloatArray(angleParam, key, param.offsetAmp915LOrR, 8, filePackageIni);
    strcpy(key, "offsetAmp1800LOrR");
    GetIniKeyFloatArray(angleParam, key, param.offsetAmp1800LOrR, 8, filePackageIni);
    strcpy(key, "offsetAmp2450LOrR");
    GetIniKeyFloatArray(angleParam, key, param.offsetAmp2450LOrR, 8, filePackageIni);
    strcpy(key, "offsetAmp5800LOrR");
    GetIniKeyFloatArray(angleParam, key, param.offsetAmp5800LOrR, 16, filePackageIni);

    // strcpy(key, "calibrationLib");
    // GetIniKeyFloatArray(angleParam, key, param.calibLib, 100*17, filePackageIni);
    // strcpy(key, "calibrationLibPro");
    // GetIniKeyFloatArray(angleParam, key, param.calibLibNew, 100*17, filePackageIni);

    // strcpy(key, "offsetAmpLib");
    // GetIniKeyFloatArray(angleParam, key, param.offsetAmpLib, 100*9, filePackageIni);
    // strcpy(key, "offsetAziLib");
    // GetIniKeyFloatArray(angleParam, key, param.offsetAziLib, 100*9, filePackageIni);

    strcpy(key, "sumcorrFreamNum");
    param.frameNum = GetIniKeyInt(recvParam, key, filePackageIni);

    // 天线种类
    strcpy(key, "AntennaShape");
    int antennaTyp = GetIniKeyInt(devParam, key, filePackageIni);
    // if(checkAntennaTyp(param.antennaNum, param.doaTyp, antennaTyp)!=0)
    // {
    // 	printf("check antenna type failed\n");
    // 	return -1;
    // }
    strcpy(key, "LPDAType");
    param.LPDAType = GetIniKeyInt(devParam, key, filePackageIni);

    // 标校信号频率MHz
    strcpy(key, "refFreq");
    param.refFreq = GetIniKeyFloat(angleMeasure, key, fileName);
    // 比幅测向参数
    strcpy(key, "coef");
    GetIniKeyFloatArray(angleMeasure, key, param.coef, 15, fileName);
    // 测向拟合开关
    strcpy(key, "fit");
    param.fit = GetIniKeyInt(angleMeasure, key, fileName);

    float maxUavSpeed;     // 无人机飞行最大速度
    float maxCorrTime;     // 最大关联时间
    int usefulCount;       // 出现次数大于此限度，认为目标有效
    char confidenceThresh; // 置信度（百分数）大于此限度，认为目标有效
    float vidAngleErr;     // 图传角度偏差
    float ctrAngleErr;     // 遥控角度偏差
    int vidFitPoints;      // 图传平滑点数
    int ctrFitPoints;      // 遥控平滑点数
    float timeOut;         // 目标过期时间（s），过期则清除
    float freqErr;         // 关联最大频率偏差，MHz

    ////objTrack
    strcpy(key, "maxUavSpeed");
    param.maxUavSpeed = GetIniKeyFloat(objTrack, key, fileName); // 无人机最大速度
    strcpy(key, "maxCorrTime");
    param.maxCorrTime = GetIniKeyFloat(objTrack, key, fileName); // 最大关联时间
    strcpy(key, "usefulCount");
    param.usefulCount = GetIniKeyInt(objTrack, key, fileName); // 出现次数大于此限度，认为目标有效
    strcpy(key, "confidenceThresh");
    param.confidenceThresh = GetIniKeyInt(objTrack, key, fileName); // 置信度（百分数）大于此限度，认为目标有效
    strcpy(key, "vidAngleErr");
    param.vidAngleErr = GetIniKeyFloat(objTrack, key, fileName); // 图传角度偏差
    strcpy(key, "ctrAngleErr");
    param.ctrAngleErr = GetIniKeyFloat(objTrack, key, fileName); // 遥控角度偏差
    strcpy(key, "vidFitPoints");
    param.vidFitPoints = GetIniKeyInt(objTrack, key, fileName); // 图传平滑点数
    strcpy(key, "ctrFitPoints");
    param.ctrFitPoints = GetIniKeyInt(objTrack, key, fileName); // 遥控平滑点数
    strcpy(key, "timeOut");
    param.timeOut = GetIniKeyFloat(objTrack, key, fileName); // 目标过期时间（s），过期则清除
    strcpy(key, "freqErr");
    param.freqErr = GetIniKeyFloat(objTrack, key, fileName); // 中心频率误差

    // box
    strcpy(key, "freqSpan");
    param.freqSpan = GetIniKeyFloat(box, key, fileName); // 搜索的带宽范围，MHz
    strcpy(key, "boxAmpErr");
    param.boxAmpErr = GetIniKeyFloat(box, key, fileName); // 搜索时的幅度误差范围，dB
    strcpy(key, "boxPhaseErr");
    param.boxPhaseErr = GetIniKeyFloat(box, key, fileName); // 搜索时相位误差
    strcpy(key, "boxTimeOut");
    param.boxTimeOut = GetIniKeyFloat(box, key, fileName); // 框选目标失效时间
    strcpy(key, "boxCmdTimeOut");
    param.boxCmdTimeOut = GetIniKeyFloat(box, key, fileName); // 失去界面跟踪指令后，维持跟踪的时间

    ////show
    strcpy(key, "colorMapSel");
    param.colorMapSel = GetIniKeyInt(show, key, fileName);
    // 画图，0表示不画，1表示画相关谱
    strcpy(key, "showImage");
    param.showImage = GetIniKeyInt(show, key, fileName);
    // 画图抽样，每sampPlot个点抽取一点。越大，则瀑布图高度越小
    strcpy(key, "nSampPlot");
    param.nSampPlot = GetIniKeyInt(show, key, fileName);
    // 幅度阈值，高于阈值的则显示，若为0则自动调整
    strcpy(key, "ampThreshShow");
    param.ampThreshShow = GetIniKeyFloat(show, key, fileName);
    // 幅度谱最小和最大值,用于颜色映射。若设为0,0表示颜色自动调节
    strcpy(key, "ampMinMax");
    GetIniKeyFloatArray(show, key, param.ampMinMax, 2, fileName);
    // 是否标记图传脉冲
    strcpy(key, "markVidPulse");
    param.markVidPulse = GetIniKeyInt(show, key, fileName);
    // 是否标记遥控脉冲
    strcpy(key, "markCtrPulse");
    param.markCtrPulse = GetIniKeyInt(show, key, fileName);
    // 直方图，0：不显示，1：图传的相位，2：图传的幅度加权相位，3：遥控的相位，4：遥控的幅度加权相位
    strcpy(key, "showHist");
    param.showHist = GetIniKeyInt(show, key, fileName);
    // 需要显示直方图的无人机ID号
    strcpy(key, "whichUavShowHist");
    param.whichUavShowHist = GetIniKeyInt(show, key, fileName);
    // 是否显示历史轨迹
    strcpy(key, "plotPath");
    param.plotPath = GetIniKeyInt(show, key, fileName);
    // 保留历史轨迹的点数,<1000
    strcpy(key, "historyPointNum");
    param.historyPointNum = GetIniKeyInt(show, key, fileName);
    // 是否显示日志信息
    strcpy(key, "showLog");
    param.showLog = GetIniKeyInt(show, key, fileName);
    // 雷达图最大半径,大于0有效
    strcpy(key, "maxRadius");
    param.maxRadius = GetIniKeyFloat(show, key, fileName);
    // 显示频谱信号截面，若大於0,则点左键显示频率方向，点右键显示时间方向
    strcpy(key, "showSlice");
    param.showSlice = GetIniKeyInt(show, key, fileName);

    // 波束宽度
    strcpy(key, "beam_width");
    param.beam_width = GetIniKeyFloat(deblur, key, fileName);
    // 比幅计算方差参数，幅度容差(单位dB)
    strcpy(key, "amp_obs_std");
    param.amp_obs_std = GetIniKeyFloat(deblur, key, fileName);
    // 相位容差(5/180×pi)
    strcpy(key, "phi_obs_std");
    param.phi_obs_std = GetIniKeyFloat(deblur, key, fileName);
    // 合理相位的最大角度
    strcpy(key, "angleSpan");
    param.angleSpan = GetIniKeyFloat(deblur, key, fileName);
    // 相位先验信息参数
    strcpy(key, "probMultiply");
    param.probMultiply = GetIniKeyFloat(deblur, key, fileName);

    // char filePhMouldIni[] = "./detect_wyc_ini/phMould.ini";
    // char phMould[] = "Mould";
    // strcpy(key, "phMould433");
    // GetIniKeyFloatArray(phMould, key, param.phMould433, 360*4, filePhMouldIni);
    // strcpy(key, "phMould315");
    // GetIniKeyFloatArray(phMould, key, param.phMould315, 360*4, filePhMouldIni);

    // char fileCalibAmpIni[] = "./detect_wyc_ini/ampCalib.ini";
    // char calibAmp[] = "ampCalib";
    // strcpy(key, "ampCalib");
    // GetIniKeyFloatArray(calibAmp, key, param.calibAmp, 100, fileCalibAmpIni);

    //    char answerIni[] = "./answer/answer.ini";
    //    strcpy(key, "ShapeEnable");
    //    param.shapeEnable = GetIniKeyInt("FPGA", key, answerIni);
    //    strcpy(key, "antSwNum");
    //    param.antSwNum = GetIniKeyInt("AntSwitch", key, answerIni);

    //    strcpy(key, "freq");
    //    char antSw[30] = {0};
    //    int num  = 0;
    //    for (num = 0; num < param.antSwNum; num++)
    //    {
    //        sprintf(antSw, "antSw%d", num);
    //        param.antSwitchFreq[num] = GetIniKeyFloat(antSw, key, answerIni);
    //    }

    //    char wifiFilePath[50] = "./detectCw_ini/fastWifiUavLib.ini";
    //    strcpy(key, "matchCoef");
    //    param.matchCoef = GetIniKeyFloat("THRESH", key, wifiFilePath);

    // 打印参数
    if (param.printParam == 1)
    {
        printf("-------------------------------------------------------\n");
        printf("[printControl]\n");
        printf("printParam=%d\n", param.printParam);
        printf("filePrefix=%s\n", param.filePrefix);
        printf("writeSumCorr=%d\n", param.writeSumCorr);
        printf("writeRef=%d\n", param.writeRef);
        printf("writeVidPulse=%d\n", param.writeVidPulse);
        printf("writeCtrPulse=%d\n", param.writeCtrPulse);
        printf("writeObj=%d\n", param.writeObj);
        printf("saveImage=%d\n", param.saveImage);
        printf("\n");

        printf("[filter]\n");
        printf("maskH=%d\n", param.maskH);
        printf("maskW=%d\n", param.maskW);
        printf("mask=\n");

        for (i = 0; i < param.maskH; i++)
        {
            for (j = 0; j < param.maskW; j++)
            {
                printf("%f,", param.mask[i * param.maskW + j]);
            }
            printf("\n");
        }
        printf("hpMaskH=%d\n", param.hpMaskH);
        printf("hpMaskW=%d\n", param.hpMaskW);
        printf("hpMask=\n");
        for (i = 0; i < param.hpMaskH; i++)
        {
            for (j = 0; j < param.hpMaskW; j++)
            {
                printf("%.2f,", param.hpMask[i * param.hpMaskW + j]);
            }
            printf("\n");
        }
        printf("\n");

        printf("[segment]\n");
        printf("dataTime=%.1f ms\n", param.dataTime);
        printf("DT=%.1f\n", param.DT);
        printf("MaxIter=%d\n", param.MaxIter);
        printf("Delt=%d\n", param.Delt);
        printf("MinW=%d\n", param.MinW);
        printf("q1IndexErr=%d\n", param.q1IndexErr);
        printf("qMeetTimes=%d\n", param.qMeetTimes);
        printf("ampErr=%.1f\n", param.ampErr);
        printf("ctrSNR=%.1f\n", param.ctrSNR);
        printf("\n");

        printf("[angleMeasure]\n");
        printf("miu=%.1f\n", param.miu);
        printf("ant1Angle=%.1f\n", param.ant1Angle);
        printf("d2450=%.3f\n", param.d2450);
        printf("initPhase2450=");
        for (i = 0; i < 8; i++)
        {
            printf("%.2f,", param.initPhase2450[i]);
        }
        printf("\n");
        printf("d5800=%.3f\n", param.d5800);
        printf("initPhase5800=");
        for (i = 0; i < 8; i++)
        {
            printf("%.2f,", param.initPhase5800[i]);
        }
        printf("\n");
        printf("refFreq=%.1f\n", param.refFreq);
        printf("offsetAmp433=");
        for (i = 0; i < 8; i++)
        {
            printf("%.2f,", param.offsetAmp433[i]);
        }
        printf("\noffsetAmp800=");
        for (i = 0; i < 8; i++)
        {
            printf("%.2f,", param.offsetAmp800[i]);
        }
        printf("\noffsetAmp2450=");
        for (i = 0; i < 8; i++)
        {
            printf("%.2f,", param.offsetAmp2450[i]);
        }
        printf("\noffsetAmp5800=");
        for (i = 0; i < 8; i++)
        {
            printf("%.2f,", param.offsetAmp5800[i]);
        }
        printf("\n");
        printf("coef=");
        for (i = 0; i < 15; i++)
        {
            printf("%.2f,", param.coef[i]);
        }
        printf("\n");
        printf("overlapAngle=%.1f\n", param.overlapAngle);

        printf("[objTrack]\n");
        printf("maxUavSpeed=%.1f\n", param.maxUavSpeed);
        printf("maxCorrTime=%.1f\n", param.maxCorrTime);
        printf("usefulCount=%d\n", param.usefulCount);
        printf("confidenceThresh=%d\n", param.confidenceThresh);
        printf("vidAngleErr=%.1f\n", param.vidAngleErr);
        printf("ctrAngleErr=%.1f\n", param.ctrAngleErr);
        printf("vidFitPoints=%d\n", param.vidFitPoints);
        printf("ctrFitPoints=%d\n", param.ctrFitPoints);
        printf("timeOut=%.1f\n", param.timeOut);
        printf("freqErr=%.1f\n", param.freqErr);

        printf("[box]\n");
        printf("freqSpan=%.1f\n", param.freqSpan);
        printf("boxAmpErr=%.1f\n", param.boxAmpErr);
        printf("boxPhaseErr=%.1f\n", param.boxPhaseErr);
        printf("boxTimeOut=%.1f\n", param.boxTimeOut);
        printf("boxCmdTimeOut=%.1f\n", param.boxCmdTimeOut);

        printf("[show]\n");
        printf("colorMapSel=%d\n", param.colorMapSel);
        printf("showImage=%d\n", param.showImage);
        printf("nSampPlot=%d\n", param.nSampPlot);
        printf("ampThreshShow=%.1f\n", param.ampThreshShow);
        printf("ampMinMax=%.1f,%.1f\n", param.ampMinMax[0], param.ampMinMax[1]);
        printf("markVidPulse=%d\n", param.markVidPulse);
        printf("markCtrPulse=%d\n", param.markCtrPulse);
        printf("showHist=%d\n", param.showHist);
        printf("whichUavShowHist=%d\n", param.whichUavShowHist);
        printf("plotPath=%d\n", param.plotPath);
        printf("historyPointNum=%d\n", param.historyPointNum);
        printf("showLog=%d\n", param.showLog);
        printf("maxRadius=%.0f\n", param.maxRadius);
        printf("showSlice=%d\n", param.showSlice);
        printf("-------------------------------------------------------\n");
    }
    *param0 = param;
    return 0;
}
#endif

/*
   读取算法参数配置文件
 */
#ifndef INI_PARAM
int getDetectParam(struct detectParams *param, char *fileName)
{
    // struct detectParams param;
    // printf("***************getDetectParam***************\n");
    int i, j;
    float mask[] = {0.0106971325264857, 0.0155642359870698, 0.0176365899319151, 0.0155642359870698, 0.0106971325264857, 0.0199848745987236, 0.0290778209633642, 0.0329494878431903, 0.0290778209633642, 0.0199848745987236, 0.0290778209633642, 0.0423079798575001, 0.0479412219279087, 0.0423079798575001, 0.0290778209633642,
                    0.0329494878431903, 0.0479412219279087, 0.0543245214657432, 0.0479412219279087, 0.0329494878431903,
                    0.0290778209633642, 0.0423079798575001, 0.0479412219279087, 0.0423079798575001, 0.0290778209633642,
                    0.0199848745987236, 0.0290778209633642, 0.0329494878431903, 0.0290778209633642, 0.0199848745987236,
                    0.0106971325264857, 0.0155642359870698, 0.0176365899319151, 0.0155642359870698, 0.0106971325264857};
    float hpMask[10] = {-0.25, -0.25, -0.25, 0.25, 1.00, 0.25, -0.25, -0.25, -0.25};

    param->printParam = 0;
    param->writeIQ = 0;   // 是否写入IQ文件
    param->corrOnOff = 0; // mavic检测开关，0关闭，1打开
    param->gfskOnOff = 0; // gfsk检测开关，0关闭，1打开
    param->wifiOnOff = 0; // wifi检测开关，0关闭，1打开
    // param->filePrefix = 0;
    param->writeSumCorr = 0;        // 相关写文件，等于1每次都写文件
    param->writeNewSumCorr = 0;     // 写新相关文件
    param->writeRef = 0;            // 标校信号结果写文件
    param->SumCorrsize = 0;         // 连续写相关文件
    param->SumCorrContinuation = 0; // SumCorrsize(续写个数)
    param->writeVidPulse = 0;       // 无人机图传脉冲检测结果写文件
    param->writeCtrPulse = 0;       // 无人机遥控脉冲检测结果写文件
    param->writeCaliPulse = 0;      // 外场自动标校写文件
    param->writeObj = 0;            // 无人机目标写文件
    param->writeDeblurLog = 0;      // 解模糊过程写文件
    param->saveImage = 0;           // 截图保存，若等于1,则保存每张界面截图。若大于2，则只在检测到无人机图传或遥控时保存界面截图，循环保存这么多图片
    // param->objPrefix = 0;
    param->maskH = 7; // 模板高度
    param->maskW = 5; // 模板宽度
    // param->mask = mask; //高斯滤波模板

    // param->mask = (float *)malloc(param->maskH * param->maskW * sizeof(float));
    memcpy(param[0].mask, mask, param->maskH * param->maskW * sizeof(float));
    // printf("mask=\n");

    // for (i = 0; i < param->maskH * param->maskW; i++)
    // {
    // 	param->mask[i] = mask[i];
    // }

    // printf("\n");
    param->hpMaskH = 1; // 模板高度
    param->hpMaskW = 9; // 模板宽度
    //     param->hpMask = hpMask;    //高通滤波模板
    // param->hpMask = (float *)malloc(param->hpMaskH * param->hpMaskW * sizeof(float));
    memcpy(param->hpMask, hpMask, param->hpMaskH * param->hpMaskW * sizeof(float));
    param->filterThresh = 60;  // 剔除野值再滤波门限
    param->dataTime = 106.7;   // 算法需要的数据时长,ms
    param->DT = 0.2;           // 阈值更新的最小变化量
    param->MaxIter = 10;       // 最大迭代次数
    param->Delt = 5;           // 填充的最大沟壑点数
    param->MinW = 30;          // 最小脉宽的点数，40表示2ms
    param->q1IndexErr = 10;    // 无人机脉冲列合并时，q1起点的误差点数容限
    param->qMeetTimes = 2;     // q1满足误差容限的最低累计次数，无人机脉冲列合并条件
    param->ampErr = 6;         // 对于中心频点不固定的图传，搜索带宽时平均幅度的容差,dB
    param->checkBWAmpErr = 10; // 对于中心频点不固定的图传，优化测频测带宽时平均幅度的容差,(复数dB)
    param->ctrSNR = 10;        // 遥控器检测的信噪比门限
    param->archimedeanSNR = 0;
    param->minCtrPulseW = 0.1; // 遥控器检测最小脉宽
    param->maxCtrPulseW = 13;  // 遥控器检测最大脉宽

    param->ant1Angle = 0; // 天线1中心线的方位角
    param->miu = 6.5;     // 幅度估计距离的系数，越大则距离越大
    // 侧向结果后处理
    param->Lateral = 0;      // 开关
    param->initial_azi = 0;  // 上一次角度
    param->azi_step = 30;    // 角度步进
    param->rand_err = 1;     // 随机误差
    param->overlapAngle = 0; // overlapAngle
    param->isUseAmpLib = 0;  // 全频段幅度标校列表开关
    param->isUseAziLib = 1;  // 全频段幅度标校列表开关
    param->isCaliFreqBw = 1; // 是否优化测评测带宽 0表示不优化(按算法测到中频带宽)，1表示不同脉冲带框内卷积比较, 2表示不同带宽内卷积比较(列累加,专业模式),3表示搜索上升下降沿优化(专业模式)

    param->bwFreqType = 1;         // 遥控信号带宽与中频类型，0表示遥控信号脉冲块默认中频类型，1表示遥控调频范围带宽与调频范围中频
    param->ampSelAntType = 0;      // 天线选择类型(0为选附近)
    param->antShape7UseMethod = 1; // 天线类型7测向方法(测向方法0,1(后续优化版本))
    param->filterMethod = 1;       // 滤波方法：0为幅度滤波，1为复数滤波，2为积分图像滤波
    param->archi433AziMethod = 0;  // 螺旋天线433频点测向方法(0为常规，1为处理波束偏移)
    param->calibType = 1;          // 标校类型选择(0为配置文件方式，1为数据库方式)
    param->fitPh = 0;              // 拟合测角使用开关
    param->fitFreqSpan[0] = 590;   // 拟合测角使用频段(freq1,freq2格式，freq1<freq2)
    param->fitFreqSpan[1] = 6050;
    param->shieldPhN = 1;             // 屏蔽相位的频段个数(配合shieldPhFreqSpan使用，shieldPhFreqSpan中前shieldPhN组范围生效,最大支持20组)
    param->shieldPhFreqSpan[0] = 825; // 屏蔽相位的频段范围(freq1,freq2,freq3,freq4...格式，两个频点为一组，组内频点freq1<freq2;例：824,827表示在824到827存在干扰，信号在此范围内不参与相位计算))
    param->shieldPhFreqSpan[1] = 826.5;
    param->antSwOnOff = 0;              // 天线分层标校选择(0为选择所有标校频点，1为选择按天线层数选择标校频点)
    param->sumcorrFreamNum = 8000;      // 默认为10000帧或者8000帧 重启生效
    param->AntennaShape = 5;            // 0表示平行形态，1表示五角星形态,2表示800M专用天线,3表示四天线800-6000MHZ,4表示双层天线,5表示阿基米德螺旋天线,6表示侦诱一体天线,7表示对周2.0天线
    param->LPDAType = 0;                // 0表示对周天线，1表示对周2.0天线
    param->maxUavSpeed = 20;            // 无人机最大速度
    param->maxCorrTime = 20;            // 最大关联时间
    param->usefulCount = 0;             // 出现次数大于此限度，认为目标有效
    param->confidenceThresh = (char)40; // 置信度（百分数）大于此限度，认为目标有效
    param->vidAngleErr = 10;            // 图传角度偏差
    param->ctrAngleErr = 20;            // 遥控角度偏差
    param->vidFitPoints = 1;            // 图传平滑点数
    param->ctrFitPoints = 10;           // 遥控平滑点数
    param->timeOut = 25;                // 目标过期时间（s），过期则清除
    param->freqErr = 10;                // 中心频率误差
    param->beam_width = 60;             // 波束宽度
    param->amp_obs_std = 2;             // 比幅计算方差参数，幅度容差(单位dB)
    param->phi_obs_std = 20;            // 相位容差(5/180×pi)
    param->angleSpan = 60;              // 合理相位的最大角度
    param->probMultiply = 2;            // 相位先验信息参数

    // *param0 = param;
    return 0;
}
#endif
/*
 *  读取ini文件中的机型库
 */

// #ifndef Code_LIB
// int getUavlib(char *fileName, struct UAVLib *UAVtypes)
// {
//     printf("******************def SQL_LIB  getUavlib****************\n");
//     printf("******************%s****************\n", fileName);
//     int i, j;
//     char name[50];                       //名称
//     char vidOrCtr[20];                   //图传还是遥控，vid或者ctr
//     float freqPoints[100];               //频点,MHz
//     int nfreqp;                          // freqPoints频点个数
//     int isFixedFreq;                     //是否固定频点,>0则是固定频点，=0则只取freqPoints[0]和freqPoints[1]作为频带范围
//     int hoppType;                        //跳频类型，仅对useMethod=2有效。0则要按照freqPoints顺序跳频，1则目标的所有频点频率一致，2则只需要在频段内按周期跳频
//     float pulseW[maxPulseWNumInLib];     //脉宽，ms
//     int nPulseW;                         //脉宽种类数
//     int isFixedPulseW;                   //脉宽是否固定，已经不产生作用（废弃状态）
//     float pulseT[5];                     //脉冲出现的周期,ms。可以有多个，比如（3,4），表示脉冲1与2起始时间相差3ms，脉冲2与3相差4ms,然后类推，脉冲3与4相差3ms
//     int nPulseT;                         //周期个数，如pulseT=(3,4,5)，则nPulseT=3
//     float pulseBW[5];                    //带宽,MHz。针对比如mavic带宽可变的情况
//     int nPulseBW;                        // pulseBW总带宽的个数，若大于1则useMethod=1时带宽需要额外搜索确定
//     float pulseWErr[maxPulseWNumInLib];  //脉宽误差,ms。比如pulseW=(3,4)，pulseWErr=(0.1,0.2)则，脉宽范围是3±0.1和4±0.2
//     float meetPulseW[maxPulseWNumInLib]; //满足脉宽的最小次数，>=。比如pulseW=(3,4)，meetPulseW=(2,5)，则必须出现3ms的脉宽不少于2次，且4ms的脉宽不少于5次
//     float pulseTErr;                     //脉冲起始时间满足周期规律的误差,ms
//     int meetHopp;                        //满足周期的脉冲最小个数，>=
//     float freqErr;                       //频率误差范围,MHz。仅对useMethod=2有效
//     int useMethod;                       //列方向分割方法1，主要用于检测图传。带宽较小的信号检测方法2,主要用于遥控检测
//     float SNR;                           //信噪比阈值，仅对useMethod=1有效，高于此信噪比才会被检测到
//     int onOroff;                         //开关,>0则打开

//     strcpy(name, "Mavic 2/Phantom4 Pro V2.0");
//     strcpy(vidOrCtr, "vid");
//     freqPoints[0] = 5710;
//     freqPoints[1] = 5850;
//     nfreqp = 2;
//     isFixedFreq = 0;
//     hoppType = 0;
//     pulseW[0] = 2;
//     pulseW[1] = 3.1;
//     pulseW[2] = 5.1;
//     nPulseW = 3;
//     isFixedPulseW = 0;
//     pulseT[0] = 4;
//     pulseT[1] = 6;
//     nPulseT = 2;
//     pulseBW[0] = 10;
//     pulseBW[1] = 18;
//     nPulseBW = 2;
//     pulseWErr[0] = 0.2;
//     pulseWErr[1] = 0.3;
//     pulseWErr[2] = 0.5;
//     meetPulseW[0] = 1;
//     meetPulseW[1] = 3;
//     meetPulseW[2] = 5;
//     pulseTErr = 0.5;
//     meetHopp = 8;
//     freqErr = 0;
//     useMethod = 1;
//     SNR = 0;
//     onOroff = 1;
//     strcpy(UAVtypes[0].name, name);
//     strcpy(UAVtypes[0].vidOrCtr, vidOrCtr);
//     for (j = 0; j < nfreqp; j++)
//         UAVtypes[0].freqPoints[j] = freqPoints[j];
//     UAVtypes[0].nfreqp = nfreqp;
//     UAVtypes[0].isFixedFreq = isFixedFreq;
//     UAVtypes[0].hoppType = hoppType;
//     for (j = 0; j < nPulseW; j++)
//     {
//         UAVtypes[0].pulseW[j] = pulseW[j];
//         UAVtypes[0].pulseWErr[j] = pulseWErr[j];
//         UAVtypes[0].meetPulseW[j] = meetPulseW[j];
//     }
//     UAVtypes[0].nPulseW = nPulseW;
//     UAVtypes[0].isFixedPulseW = isFixedPulseW;
//     UAVtypes[0].nPulseT = nPulseT;
//     for (j = 0; j < nPulseT; j++)
//         UAVtypes[0].pulseT[j] = pulseT[j];
//     for (j = 0; j < nPulseBW; j++)
//         UAVtypes[0].pulseBW[j] = pulseBW[j];
//     UAVtypes[0].nPulseBW = nPulseBW;
//     UAVtypes[0].pulseTErr = pulseTErr;
//     UAVtypes[0].meetHopp = meetHopp;
//     UAVtypes[0].freqErr = freqErr;
//     UAVtypes[0].useMethod = useMethod;
//     UAVtypes[0].SNR = SNR;
//     UAVtypes[0].onOroff = onOroff;
//     // char temp[LINE_MAX];
//     // FILE *fp = NULL;
//     // fp = fopen(fileName, "r");
//     // const char s[2] = "'"; //分隔字符串
//     // const char d[] = ",";  //分隔字符串
//     // char *str = NULL;
//     // char *part = NULL;
//     // char buf[LINE_MAX];
//     // int nUavLib = 0;
//     // int i, j, k;
//     // int len_line;

//     // if (NULL == (fp))
//     // {
//     //     perror("fopen");
//     //     printf("file %s open failed", fileName);
//     //     return -1;
//     // }
//     // printf("file %s open \n", fileName);
//     // while (NULL != fgets(temp, LINE_MAX, fp))
//     // {

//     //     printf("*********the line char is****************\n ");
//     //     printf("%s\n", temp); //文件中每一行的文本
//     //     nUavLib++;
//     //     i = 0;
//     //     // len_line = strlen(temp);

//     //     for (str = strtok(temp, s); str != NULL; str = strtok(NULL, s))
//     //     {
//     //         printf("%s\n", str); //每一行分离开的参数
//     //         strcpy(buf, str);
//     //         i++;
//     //         printf("*********str char is %s **这是第%d项****************\n ", buf, i);
//     //     }
//     // }

//     // fclose(fp);
//     // printf("file %s close \n", fileName);
//     // return nUavLib;
// }
// #endif

#ifdef Code_LIB
int getUavlib(char *fileName, struct UAVLib *UAVtypes)
{
    int nUav = 4;
    char uavLibStr[][400] = {"'Phantom 4A/Inspire/Pro V1.0/M600','vid','2406.5,2416.5,2426.5,2436.5,2446.5,2456.5,2466.5,2476.5',8,1,0,'9.8',1,1,'14',1,'10',1,'1','4',1,4,0,1,0,1,0,'OFDM'",
                             "'Mavic 2/Phantom 4P/Mavic Air2/Air2s/FPV','vid','2400,2480',2,0,0,'2,3.1,5.1',3,0,'4,6',2,'9,18',2,'0.2,0.3,0.4','1,3,5',0.3,8,0,1,0,1,0,'OFDM'",
                             "'Phantom4 Pro V1.0','vid','5730,5735,5740,5745,5750,5755,5760,5765,5770,5775,5780,5785,5790,5795,5800,5805,5810,5815,5820,5825,5830,5835,5840',23,1,0,'9.8',1,1,'14',1,'10',1,'1','4',1,3,0,1,0,1,0,'OFDM'",
                             "'Mavic 2/Phantom 4P/Mavic Air2/Air2s/FPV','vid','5710,5850',2,0,0,'2,3.1,5.1',3,0,'4,6',2,'9,18',2,'0.2,0.3,0.5','1,3,5',0.3,8,0,1,0,1,0,'OFDM'"};
    //     char uavLibStr[] = "('Phantom 4A/Inspire/Pro V1.0/M600','vid','2406.5,2416.5,2426.5,2436.5,2446.5,2456.5,2466.5,2476.5',8,1,0,'9.8',1,1,'14',1,'10',1,'1','4',1,4,0,1,0,1,0,'OFDM'),\
    // ('Mavic 2/Phantom 4P/Mavic Air2/Air2s/FPV','vid','2400,2480',2,0,0,'2,3.1,5.1',3,0,'4,6',2,'9,18',2,'0.2,0.3,0.4','1,3,5',0.3,8,0,1,0,1,0,'OFDM'),\
    // ('Phantom4 Pro V1.0','vid','5730,5735,5740,5745,5750,5755,5760,5765,5770,5775,5780,5785,5790,5795,5800,5805,5810,5815,5820,5825,5830,5835,5840',23,1,0,'9.8',1,1,'14',1,'10',1,'1','4',1,3,0,1,0,1,0,'OFDM'),\
    // ('Mavic 2/Phantom 4P/Mavic Air2/Air2s/FPV','vid','5710,5850',2,0,0,'2,3.1,5.1',3,0,'4,6',2,'9,18',2,'0.2,0.3,0.5','1,3,5',0.3,8,0,1,0,1,0,'OFDM'),\
    // ('Phantom 4A','ctr','2468,2462,2456,2450,2444,2438,2432,2426,2420,2414,2408,2470,2464,2458,2452,2446,2440,2434,2428,2422,2416,2410,2404,2466,2460,2454,2448,2442,2436,2430,2424,2418,2412,2406',34,1,0,'2.2,3.2',2,1,'14',1,'1.2',1,'0.3,0.2','2,2',1,4,0.8,2,0,1,0,'OFDM'),\
    // ('Phantom 4A','ctr','2406,2412,2418,2424,2430,2436,2442,2448,2454,2460,2466,2404,2410,2416,2422,2428,2434,2440,2446,2452,2458,2464,2470,2408,2414,2420,2426,2432,2438,2444,2450,2456,2462,2468',34,1,0,'2.2,3.2',2,1,'14',1,'1.2',1,'0.3,0.2','2,2',1,4,0.8,2,0,1,0,'OFDM'),\
    // ('Phantom 4 Pro V1.0','ctr','2455,2431,2407,2460,2436,2412,2465,2441,2417,2470,2447,2422,2476,2452,2428,2404,2457,2433,2409,2462,2438,2414,2467,2443,2419,2472,2448,2424,2477,2453,2429,2405,2458,2435,2411,2464,2439,2415,2468,2444',40,1,0,'2.2',1,1,'14',1,'1.2',1,'0.3','2',1,4,0.8,2,0,1,0,'OFDM'),\
    // ('Phantom 4A','ctr','2410,2420,2430,2440,2450,2460,2470,2412,2422,2432,2442,2452,2462,2404,2414,2424,2434,2444,2454,2464,2406,2416,2426,2436,2446,2456,2466,2408,2418,2428,2438,2448,2458,2468',34,1,0,'2.2,3.2',2,1,'14',1,'1.2',1,'0.3,0.2','2,2',1,4,0.8,2,0,1,0,'OFDM'),\
    // ('Phantom 4A','ctr','2420,2480',2,0,0,'2.2,3.2',2,1,'14',1,'1.2',1,'0.3,0.2','2,2',1,4,0.8,2,0,1,0,'OFDM'),\
    // ('MavicAir2','ctr','2400,2480',2,0,0,'0.52',1,1,'4,6',2,'1.5',1,'0.06','12',0.2,8,0,2,0,1,0,'OFDM'),\
    // ('Mavic2','ctr','5720,5810',2,0,0,'0.52',1,1,'4,6',2,'1.5',1,'0.06','12',0.2,8,0,2,0,1,0,'OFDM'),\
    // ('FlySky(FS-i6S)','ctr','2467,2455,2442.5,2450.5,2432,2429,2423,2440,2446,2474.5,2457.5,2421.5,2426.5,2419.5,2452.5,2416.5',16,1,0,'1.35',1,1,'3.85',1,'1.8',1,'0.1','5',0.3,6,1,2,0,0,0,'2FSK'),\
    // ('FlySky(FS-i6)','ctr','2405,2444.5,2441,2466.5,2408,2419,2434.5,2454.5,2458,2425.5,2446,2413.5,2429,2432,2416,2411',16,1,2,'1.35',1,1,'3.85',1,'1.8',1,'0.1','6',0.3,6,1,2,0,0,0,'2FSK'),\
    // ('Futaba','ctr','2407,2409,2411,2413,2415,2417,2419,2421,2423,2425,2427,2429,2431,2433,2435,2437,2439,2441,2443,2445,2447,2449,2451,2453,2455,2457,2459,2461,2463,2465,2467',31,1,2,'1.5',1,1,'15',1,'1.5',1,'0.1','3',1,4,0.8,2,0,0,0,'2FSK'),\
    // ('xiaomi/xiaomi 4K','ctr','5745,5765,5785,5805,5825',5,1,1,'0.25',1,1,'3.06',1,'10',1,'0.15','10',0.3,20,1.5,2,6,0,0,'OFDM'),\
    // ('xiaomi','vid','5745,5765,5785,5805,5825',5,1,1,'1',1,1,'3.06',1,'10',1,'0.2','5',0.3,5,1.5,2,0,0,0,'OFDM'),\
    // ('iDol','ctr','2429,2439,2449,2459',4,1,0,'0.84',1,1,'4.96',1,'2.5',1,'0.1','5',0.5,10,0.8,2,0,0,0,'2FSK'),\
    // ('Phantom4 Pro V1.0','ctr','5732,5862',2,0,0,'2.25',1,1,'14',1,'1.8',1,'0.5','4',0.5,4,1,2,0,1,0,'OFDM'),\
    // ('p900','ctr','900,930',2,0,0,'1.6',1,1,'20',1,'1.2',1,'0.3','4',1,4,1,2,0,0,0,'2FSK'),\
    // ('p900-2','ctr','900,930',2,0,0,'2.1',1,1,'20',1,'1.2',1,'0.3','4',1,4,1,2,0,0,0,'2FSK'),\
    // ('RadioLink','ctr','2405,2480',2,0,0,'2.19',1,1,'6',1,'3',1,'0.2','10',1,0,0,2,0,0,0,'QPSK'),\
    // ('Fengying','ctr','2491,2491,2500,2500,2515,2515',6,1,0,'1.4',1,1,'2.5',1,'1',1,'0.12','10',0.6,15,0.8,2,0,0,0,'2FSK'),\
    // ('Xinghuo433','ctr','417,421',2,0,0,'3.75',1,1,'0',0,'1',1,'0.1','1',0,1,0,1,20,0,0,'2FSK'),\
    // ('X-ROCK433','ctr','433.3,433.9,434.5',3,1,0,'4.48,3.75',2,0,'0',0,'1',1,'0.1,0.1','1,1',0,0,0.5,1,6,0,0,'2FSK'),\
    // ('X-ROCK900M','vid','915,928',2,0,0,'3.75',1,1,'0',0,'1',1,'0.1','1',0,0,0.5,1,10,0,0,'2FSK'),\
    // ('attop(yade)','ctr','2430,2450,2461',3,1,0,'0.5',1,1,'7.7',1,'1.2',1,'0.1','5',0.5,8,1,2,0,0,0,'2FSK'),\
    // ('udi(youdi)','vid','2412,2427,2415,2430,2430,2433,2436,2418,2427,2421,2421,2436,2415,2412,2430,2433,2427,2430,2427,2433',20,1,0,'11.5',1,1,'13.5',1,'6',1,'1','3',1,4,2,2,0,0,0,'2FSK'),\
    // ('syma','ctr','2457,2457,2469,2469,2454,2454,2466,2466',8,1,0,'0.7',1,1,'4',1,'1.2',1,'0.2','10',1,10,1,2,0,0,0,'2FSK'),\
    // ('sjrc/holy stone','ctr','2454,2454,2474,2474,2459,2459,2478,2478',8,1,0,'1.15',1,1,'3.2,11.8',2,'1.2',1,'0.3','4',1,5,1,2,0,0,0,'2FSK'),\
    // ('jjpro','ctr','2457,2448,2439,2430,2421,2460,2460,2451,2442,2433,2424,2454,2445,2436,2427,2418',16,1,0,'1.6',1,1,'11.6',1,'1.2',1,'0.3','5',1,5,1,2,0,0,0,'UNKNOWN'),\
    // ('hr','ctr','2470,2456,2466',3,1,0,'0.5',1,1,'7.75',1,'1.2',1,'0.2','6',1,6,1,2,0,0,0,'2FSK'),\
    // ('dwi','ctr','2446',1,1,1,'1.38',1,1,'6,9',2,'1.8',1,'0.3','6',0.8,8,1,2,0,0,0,'2FSK'),\
    // ('RadioLink(T8FB)','ctr','435,442',2,0,0,'10.7',1,1,'25',1,'1.2',1,'0.8','3',3,3,1,2,0,0,0,'2FSK'),\
    // ('RadioLink(T8FB)','ctr','2435,2485',2,0,0,'9.3',1,1,'20',1,'1.2',1,'1','3',1,3,1,2,0,0,0,'2FSK'),\
    // ('RadioLink(T8FB)','ctr','5805',1,1,0,'9.3',1,1,'20',1,'1.2',1,'1','3',1,3,1,2,0,0,0,'2FSK'),\
    // ('FPV(MS-750_1200TVL)','vid','5805',1,1,0,'18.3',1,1,'20',1,'1.8',1,'1','3',1,3,0,1,0,0,0,'OFDM'),\
    // ('F1000(Feima)','ctr','840,845',2,0,0,'5.1',1,1,'19.9',1,'1.2',1,'0.8','3',1,3,0.8,2,0,0,0,'UNKNOWN'),\
    // ('F1000(Feima)','vid','840,845',2,0,0,'2.6',1,1,'19.9',1,'1.2',1,'0.5','3',1,3,0.8,2,0,0,0,'UNKNOWN'),\
    // ('Phantom 3Pro','ctr','2436,2434,2432,2430,2428,2426,2424,2422,2420,2418,2416,2414,2412,2410,2408,2406,2404,2470,2468,2466,2464,2462,2460,2458,2456,2454,2452,2450,2448,2446,2444,2442,2440,2438',34,1,0,'2.2,3.2',2,1,'14',1,'1.2',1,'0.3,0.2','2,2',1,4,0.8,2,0,0,0,'UNKNOWN'),\
    // ('Immersion 433','vid','433.9',1,1,0,'22.9',1,1,'24',1,'1',1,'1','2',1,2,0,1,0,1,0,'2FSK'),\
    // ('Phantom4Pro','ctr','2406,2408,2412,2418,2423,2428,2433,2435,2439,2445,2450,2455,2457,2462,2467,2472,2477',17,1,2,'2.2',1,1,'14',1,'2',1,'0.2','5',1,6,1,2,0,0,0,'OFDM'),\
    // ('PowerEgg','vid','2431',1,1,0,'1.2,4,8',3,0,'0',0,'5',1,'0.2,2,3','2,4,2',0,0,1,2,0,1,0,'UNKNOWN'),\
    // ('AMIMON','vid','5736,5814',2,0,0,'14.2',1,1,'16',1,'40',1,'1','5',1,5,1,1,0,1,0,'UNKNOWN'),\
    // ('845_SIGNAL','ctr','839,846',2,0,2,'2.1',1,1,'20',1,'0.3',1,'0.2','3',1,3,0,2,0,1,0,'2FSK'),\
    // ('AirLink','ctr','1766,1784',2,1,0,'0.56',1,1,'0.48,19.52',2,'1',1,'0.1','6',0.2,5,1,2,0,0,0,'OFDM'),\
    // ('Phantom 4 RTK','vid','5710,5850',2,0,0,'2.03,5.07',2,0,'2.99,17.01',2,'10',1,'0.2,0.3','2,2',0.3,6,0,1,0,1,0,'OFDM'),\
    // ('dahua1.4G','vid','1417,1472',2,0,0,'0.9',1,1,'1',1,'10',1,'0.2','30',0.05,20,0,1,0,1,0,'UNKNOWN'),\
    // ('Mavic Air2/Air2s/mini 2','vid','2400,2480',2,0,0,'1.07,2.03',2,0,'2,4',2,'10,18',2,'0.1,0.2','10,3',0.3,10,0,1,0,1,0,'OFDM'),\
    // ('Mavic Air2/Air2s/mini 2','vid','5710,5850',2,0,0,'1.07,2.03',2,0,'2,4',2,'10,18',2,'0.1,0.2','10,3',0.3,10,0,1,0,1,0,'OFDM'),\
    // ('Industry ZD680','vid','2392,2510',2,0,2,'1',1,1,'0',0,'0.3',1,'0.1','40',0,0,0,2,0,1,1,'UNKNOWN'),\
    // ('Mavlink915Wide','vid','330,433,845,915',2,0,2,'115',1,1,'0',0,'0.6',1,'6','1',0,0,0,1,8,1,0,'UNKNOWN'),\
    // ('Mavlink433Drone','vid','434',1,1,0,'3.7,4.5',2,0,'230.2,5.12',2,'0.3',1,'0.1,0.2','1,1',2,1,1,2,0,1,0,'UNKNOWN'),\
    // ('vid1440','vid','1420,1480',2,0,0,'0.9',1,0,'2,3',2,'10.2',1,'0.1','40',0.3,20,0.6,1,0,1,0,'UNKNOWN'),\
    // ('vid915','vid','900,950',2,0,0,'15.2',1,0,'22',1,'9.5',1,'1.5','12',2.2,6,0.6,1,0,1,1,'UNKNOWN'),\
    // ('vid815','vid','800,850',2,0,0,'0.8,2,3.6',3,0,'2.1,3.1,5.1',3,'9',1,'0.2,0.3,0.6','1,1,10',0.5,10,0.6,1,0,1,1,'UNKNOWN'),\
    // ('unknown1-1','vid','1350,1450',2,0,0,'15.1',1,0,'22.2',1,'9',1,'0.5','3',1.5,2,0,1,0,1,0,'OFDM'),\
    // ('unknown1-2','vid','1350,1450',2,0,0,'19.2',1,0,'22.2',1,'9',1,'1','3',1,3,0,1,5,1,0,'OFDM'),\
    // ('unknown1-2','ctr','1350,1450',2,0,0,'4.1',1,0,'22.2',1,'9',1,'0.5','4',1,4,0,1,5,1,0,'OFDM'),\
    // ('unknown2-1','vid','800,850',2,0,0,'15.1',1,0,'22.2',1,'9',1,'1','3',1,3,0,1,0,1,0,'OFDM'),\
    // ('unknown2-2','vid','900,950',2,0,0,'15.1',1,0,'22.2',1,'9',1,'1','3',1,3,0,1,0,1,0,'OFDM'),\
    // ('unknown2-1','vid','800,850',2,0,0,'19.2',1,0,'22.2',1,'9',1,'1','3',1,3,0,1,0,1,0,'OFDM'),\
    // ('unknown2-2','vid','900,950',2,0,0,'19.2',1,0,'22.2',1,'9',1,'1','3',1,3,0,1,0,1,0,'OFDM'),\
    // ('unknown2-1','ctr','800,850',2,0,0,'4.1',1,0,'22.2',1,'9',1,'0.5','3',1,0,3,1,5,1,0,'OFDM'),\
    // ('unknown2-2','ctr','900,950',2,0,0,'4.1',1,0,'22.2',1,'9',1,'0.5','3',1,0,3,1,5,1,0,'OFDM');";

    // printf("%s\n",uavLibStr);

    int ind = 0, i = 0;
    unsigned char tmp;
    while (1)
    {
        // printf("111111111111111111111111111111\n");
        if (nUav <= i)
            break;
        char *str1 = strtok(*(uavLibStr + i), ",'");
        // 		printf("%s\n",str1);

        //         str1 = strtok(NULL, ",'");
        strcpy(UAVtypes[i].name, str1);
        //         printf("%s\n",str1);
        str1 = strtok(NULL, ",'");
        strcpy(UAVtypes[i].vidOrCtr, str1);
        //         printf("%s\n",str1);
        str1 = strtok(NULL, "'");
        ind = 0;
        while (1)
        {
            str1 = strtok(NULL, ",");
            //             printf("%s\n",str1);
            // if(str1.find("'"))
            if (strchr(str1, '\''))
            {
                tmp = (unsigned char)(strchr(str1, '\'') - str1);
                UAVtypes[i].freqPoints[ind] = atof(strndup(str1, tmp));
                ind++;
                break;
            }
            UAVtypes[i].freqPoints[ind] = atof(str1);
            ind++;
        }
        // str1 = strtok(NULL, "'");
        //         printf("11111   %s\n",str1);
        str1 = strtok(NULL, ",");
        UAVtypes[i].nfreqp = atoi(str1);
        //         printf("%s\n",str1);
        str1 = strtok(NULL, ",");
        UAVtypes[i].isFixedFreq = atoi(str1);
        //         printf("%s\n",str1);
        str1 = strtok(NULL, ",");
        UAVtypes[i].hoppType = atoi(str1);
        //         printf(" %s\n",str1);
        ind = 0;
        while (1)
        {
            str1 = strtok(NULL, ",");
            if (ind == 0)
                str1 = str1 + 1;
            //             printf("1111 %s\n",str1);
            // if(str1.find("'"))
            if (strchr(str1, '\''))
            {
                tmp = (unsigned char)(strchr(str1, '\'') - str1);
                UAVtypes[i].pulseW[ind] = atof(strndup(str1, tmp));
                ind++;
                break;
            }
            UAVtypes[i].pulseW[ind] = atof(str1);
            ind++;
            if (ind >= 5)
                break;
        }
        // str1 = strtok(NULL, "'");
        // printf("%s\n",str1);
        str1 = strtok(NULL, ",");
        UAVtypes[i].nPulseW = atoi(str1);
        // printf("%s\n",str1);
        str1 = strtok(NULL, ",");
        UAVtypes[i].isFixedPulseW = atoi(str1);
        // printf("%s\n",str1);
        ind = 0;
        while (1)
        {
            str1 = strtok(NULL, ",");
            if (ind == 0)
                str1 = str1 + 1;
            // printf("1111 %s\n",str1);
            // if(str1.find("'"))
            if (strchr(str1, '\''))
            {
                tmp = (unsigned char)(strchr(str1, '\'') - str1);
                UAVtypes[i].pulseT[ind] = atof(strndup(str1, tmp));
                ind++;
                break;
            }
            UAVtypes[i].pulseT[ind] = atof(str1);
            ind++;
            if (ind >= 5)
                break;
        }
        str1 = strtok(NULL, ",");
        UAVtypes[i].nPulseT = atoi(str1);
        // printf("%s\n",str1);
        ind = 0;
        while (1)
        {
            str1 = strtok(NULL, ",");
            if (ind == 0)
                str1 = str1 + 1;
            // printf("1111 %s\n",str1);
            // if(str1.find("'"))
            if (strchr(str1, '\''))
            {
                tmp = (unsigned char)(strchr(str1, '\'') - str1);
                UAVtypes[i].pulseBW[ind] = atof(strndup(str1, tmp));
                ind++;
                break;
            }
            UAVtypes[i].pulseBW[ind] = atof(str1);
            ind++;
            if (ind >= 5)
                break;
        }
        // str1 = strtok(NULL, "'");
        // printf("%s\n",str1);
        str1 = strtok(NULL, ",");
        UAVtypes[i].nPulseBW = atoi(str1);
        // printf("%s\n",str1);

        ind = 0;
        while (1)
        {
            str1 = strtok(NULL, ",");
            if (ind == 0)
                str1 = str1 + 1;
            // printf("1111 %s\n",str1);
            // if(str1.find("'"))
            if (strchr(str1, '\''))
            {
                tmp = (unsigned char)(strchr(str1, '\'') - str1);
                UAVtypes[i].pulseWErr[ind] = atof(strndup(str1, tmp));
                ind++;
                break;
            }
            UAVtypes[i].pulseWErr[ind] = atof(str1);
            ind++;
            if (ind >= 5)
                break;
        }
        // str1 = strtok(NULL, "'");
        // printf("%s\n",str1);

        ind = 0;
        while (1)
        {
            str1 = strtok(NULL, ",");
            if (ind == 0)
                str1 = str1 + 1;
            // printf("1111 %s\n",str1);
            // if(str1.find("'"))
            if (strchr(str1, '\''))
            {
                tmp = (unsigned char)(strchr(str1, '\'') - str1);
                UAVtypes[i].meetPulseW[ind] = atoi(strndup(str1, tmp));
                ind++;
                break;
            }
            UAVtypes[i].meetPulseW[ind] = atoi(str1);
            ind++;
        }

        // str1 = strtok(NULL, "'");
        // str1 = strtok(NULL, "'");
        str1 = strtok(NULL, ",");
        UAVtypes[i].pulseTErr = atof(str1);
        str1 = strtok(NULL, ",");
        UAVtypes[i].meetHopp = atoi(str1);
        str1 = strtok(NULL, ",");
        UAVtypes[i].freqErr = atof(str1);
        str1 = strtok(NULL, ",");
        UAVtypes[i].useMethod = atoi(str1);
        str1 = strtok(NULL, ",");
        UAVtypes[i].SNR = atof(str1);
        str1 = strtok(NULL, ",");
        UAVtypes[i].onOroff = atoi(str1);
        str1 = strtok(NULL, ",");
        str1 = strtok(NULL, "',");
        if (!strcmp(str1, "AM"))
            UAVtypes[i].mrm = MRM_AM;
        if (!strcmp(str1, "ASK"))
            UAVtypes[i].mrm = MRM_ASK;
        if (!strcmp(str1, "BPSK"))
            UAVtypes[i].mrm = MRM_BPSK;
        if (!strcmp(str1, "QPSK"))
            UAVtypes[i].mrm = MRM_QPSK;
        if (!strcmp(str1, "8PSK"))
            UAVtypes[i].mrm = MRM_8PSK;
        if (!strcmp(str1, "2FSK"))
            UAVtypes[i].mrm = MRM_2FSK;
        if (!strcmp(str1, "4FSK"))
            UAVtypes[i].mrm = MRM_4FSK;
        if (!strcmp(str1, "UNKNOWN"))
            UAVtypes[i].mrm = MRM_UNKNOWN;
        if (!strcmp(str1, "OFDM"))
            UAVtypes[i].mrm = MRM_OFDM;
        //         UAVtypes[nUav].mrm = atoi(str1);
        //         printf("%s\n",str1);
        //         str1 = strtok(NULL, "',");

        // while (strcmp(str1,")") && strcmp(str1,");")) //str1 != NULL)
        // {
        //     printf("%s\n",str1);
        //     str1 = strtok(NULL, ",'");
        //     // printf("222 %s\n",str1);

        //     // printf("222 %s\n",str1);
        // }
        // printf("%s\n",str1);
        // printf("comp = %d\n",~strcmp(str1,");"));
        i++;
        //         if (!strcmp(str1, ");"))
        //         {
        //             break;
        //         }
        //         str1 = strtok(NULL, "'");
    }
    //     printUavlib(UAVtypes, nUav);
    return nUav;
}
#endif

/*
 *  读取ini文件中的机型库
 */
int getUavlibCW(char *fileName, struct UAVLib *UAVtypes)
{
    int i, j;
    int nUAV = 0;
    int printUavFeature = 0;
    int allSwitch = 0;

    // 机型字段
    char name[50];                       // 名称
    char vidOrCtr[20];                   // 图传还是遥控，vid或者ctr
    float freqPoints[100];               // 频点,MHz
    int nfreqp;                          // freqPoints频点个数
    int isFixedFreq;                     // 是否固定频点,>0则是固定频点，=0则只取freqPoints[0]和freqPoints[1]作为频带范围
    int hoppType;                        // 跳频类型，仅对useMethod=2有效。0则要按照freqPoints顺序跳频，1则目标的所有频点频率一致，2则只需要在频段内按周期跳频
    float pulseW[maxPulseWNumInLib];     // 脉宽，ms
    int nPulseW;                         // 脉宽种类数
    int isFixedPulseW;                   // 脉宽是否固定，已经不产生作用（废弃状态）
    float pulseT[5];                     // 脉冲出现的周期,ms。可以有多个，比如（3,4），表示脉冲1与2起始时间相差3ms，脉冲2与3相差4ms,然后类推，脉冲3与4相差3ms
    int nPulseT;                         // 周期个数，如pulseT=(3,4,5)，则nPulseT=3
    float pulseBW[5];                    // 带宽,MHz。针对比如mavic带宽可变的情况
    int nPulseBW;                        // pulseBW总带宽的个数，若大于1则useMethod=1时带宽需要额外搜索确定
    float pulseWErr[maxPulseWNumInLib];  // 脉宽误差,ms。比如pulseW=(3,4)，pulseWErr=(0.1,0.2)则，脉宽范围是3±0.1和4±0.2
    float meetPulseW[maxPulseWNumInLib]; // 满足脉宽的最小次数，>=。比如pulseW=(3,4)，meetPulseW=(2,5)，则必须出现3ms的脉宽不少于2次，且4ms的脉宽不少于5次
    float pulseTErr;                     // 脉冲起始时间满足周期规律的误差,ms
    int meetHopp;                        // 满足周期的脉冲最小个数，>=
    float freqErr;                       // 频率误差范围,MHz。仅对useMethod=2有效
    int useMethod;                       // 列方向分割方法1，主要用于检测图传。带宽较小的信号检测方法2,主要用于遥控检测
    float SNR;                           // 信噪比阈值，仅对useMethod=1有效，高于此信噪比才会被检测到
    int onOroff;                         // 开关,>0则打开
    int flag;

    char sTitle[50]; // 字段标题
    char key[30] = {0};

    // 打印
    char printControl[] = "printControl";
    strcpy(key, "printUavFeature");
    printUavFeature = GetIniKeyInt(printControl, key, fileName);
    strcpy(key, "allSwitch");
    allSwitch = GetIniKeyInt(printControl, key, fileName);

    // 读取机型参数
    for (i = 0; i < MaxUAVinLib; i++)
    {
        sprintf(sTitle, "CW%d", i + 1);

        strcpy(key, "name");
        flag = GetIniKeyString(name, sTitle, key, fileName);
        if (flag != 0)
            continue;
        strcpy(key, "vidOrCtr");
        GetIniKeyString(vidOrCtr, sTitle, key, fileName);
        strcpy(key, "nfreqp");
        nfreqp = GetIniKeyInt(sTitle, key, fileName);
        strcpy(key, "freqPoints");
        GetIniKeyFloatArray(sTitle, key, freqPoints, nfreqp, fileName);
        strcpy(key, "isFixedFreq");
        isFixedFreq = GetIniKeyInt(sTitle, key, fileName) > 0;
        strcpy(key, "hoppType");
        hoppType = GetIniKeyInt(sTitle, key, fileName);
        strcpy(key, "nPulseW");
        nPulseW = GetIniKeyInt(sTitle, key, fileName);
        strcpy(key, "pulseW");
        GetIniKeyFloatArray(sTitle, key, pulseW, nPulseW, fileName);
        strcpy(key, "isFixedPulseW");
        isFixedPulseW = GetIniKeyInt(sTitle, key, fileName) > 0;
        strcpy(key, "nPulseT");
        nPulseT = GetIniKeyInt(sTitle, key, fileName);
        strcpy(key, "pulseT");
        GetIniKeyFloatArray(sTitle, key, pulseT, nPulseT, fileName);
        strcpy(key, "nPulseBW");
        nPulseBW = GetIniKeyInt(sTitle, key, fileName);
        strcpy(key, "pulseBW");
        GetIniKeyFloatArray(sTitle, key, pulseBW, nPulseBW, fileName);
        strcpy(key, "pulseWErr");
        GetIniKeyFloatArray(sTitle, key, pulseWErr, nPulseW, fileName);
        strcpy(key, "meetPulseW");
        GetIniKeyFloatArray(sTitle, key, meetPulseW, nPulseW, fileName);
        strcpy(key, "pulseTErr");
        pulseTErr = GetIniKeyFloat(sTitle, key, fileName);
        strcpy(key, "meetHopp");
        meetHopp = GetIniKeyInt(sTitle, key, fileName);
        strcpy(key, "freqErr");
        freqErr = GetIniKeyFloat(sTitle, key, fileName);
        strcpy(key, "useMethod");
        useMethod = GetIniKeyInt(sTitle, key, fileName);
        strcpy(key, "SNR");
        SNR = GetIniKeyFloat(sTitle, key, fileName);
        strcpy(key, "onOroff");
        onOroff = GetIniKeyInt(sTitle, key, fileName);

        if (flag == 0 && (allSwitch > 0 || onOroff > 0))
        {
            // 赋值
            strcpy(UAVtypes[nUAV].name, name);
            strcpy(UAVtypes[nUAV].vidOrCtr, vidOrCtr);
            for (j = 0; j < nfreqp; j++)
                UAVtypes[nUAV].freqPoints[j] = freqPoints[j];
            UAVtypes[nUAV].nfreqp = nfreqp;
            UAVtypes[nUAV].isFixedFreq = isFixedFreq;
            UAVtypes[nUAV].hoppType = hoppType;
            for (j = 0; j < nPulseW; j++)
            {
                UAVtypes[nUAV].pulseW[j] = pulseW[j];
                UAVtypes[nUAV].pulseWErr[j] = pulseWErr[j];
                UAVtypes[nUAV].meetPulseW[j] = meetPulseW[j];
            }
            UAVtypes[nUAV].nPulseW = nPulseW;
            UAVtypes[nUAV].isFixedPulseW = isFixedPulseW;
            UAVtypes[nUAV].nPulseT = nPulseT;
            for (j = 0; j < nPulseT; j++)
                UAVtypes[nUAV].pulseT[j] = pulseT[j];
            for (j = 0; j < nPulseBW; j++)
                UAVtypes[nUAV].pulseBW[j] = pulseBW[j];
            UAVtypes[nUAV].nPulseBW = nPulseBW;
            UAVtypes[nUAV].pulseTErr = pulseTErr;
            UAVtypes[nUAV].meetHopp = meetHopp;
            UAVtypes[nUAV].freqErr = freqErr;
            UAVtypes[nUAV].useMethod = useMethod;
            UAVtypes[nUAV].SNR = SNR;
            UAVtypes[nUAV].onOroff = onOroff;
            nUAV++;
        }

        char wifiFilePath[50] = "./detectCw_ini/fastWifiUavLib.ini";

        // 读取机型参数
        for (i = 0; i < MaxUAVinLib; i++)
        {
            sprintf(sTitle, "WIFI%d", i + 1);

            strcpy(key, "name");
            flag = GetIniKeyString(name, sTitle, key, wifiFilePath);
            if (flag != 0)
                continue;
            strcpy(key, "vidOrCtr");
            GetIniKeyString(vidOrCtr, sTitle, key, wifiFilePath);
            strcpy(key, "nfreqp");
            nfreqp = GetIniKeyInt(sTitle, key, wifiFilePath);
            strcpy(key, "freqPoints");
            GetIniKeyFloatArray(sTitle, key, freqPoints, nfreqp, wifiFilePath);
            strcpy(key, "isFixedFreq");
            isFixedFreq = GetIniKeyInt(sTitle, key, wifiFilePath) > 0;
            strcpy(key, "hoppType");
            hoppType = GetIniKeyInt(sTitle, key, wifiFilePath);
            strcpy(key, "nPulseW");
            nPulseW = GetIniKeyInt(sTitle, key, wifiFilePath);
            strcpy(key, "pulseW");
            GetIniKeyFloatArray(sTitle, key, pulseW, nPulseW, wifiFilePath);
            strcpy(key, "isFixedPulseW");
            isFixedPulseW = GetIniKeyInt(sTitle, key, wifiFilePath) > 0;
            strcpy(key, "nPulseT");
            nPulseT = GetIniKeyInt(sTitle, key, wifiFilePath);
            strcpy(key, "pulseT");
            GetIniKeyFloatArray(sTitle, key, pulseT, nPulseT, wifiFilePath);
            strcpy(key, "nPulseBW");
            nPulseBW = GetIniKeyInt(sTitle, key, wifiFilePath);
            strcpy(key, "pulseBW");
            GetIniKeyFloatArray(sTitle, key, pulseBW, nPulseBW, wifiFilePath);
            strcpy(key, "pulseWErr");
            GetIniKeyFloatArray(sTitle, key, pulseWErr, nPulseW, wifiFilePath);
            strcpy(key, "meetPulseW");
            GetIniKeyFloatArray(sTitle, key, meetPulseW, nPulseW, wifiFilePath);
            strcpy(key, "pulseTErr");
            pulseTErr = GetIniKeyFloat(sTitle, key, wifiFilePath);
            strcpy(key, "meetHopp");
            meetHopp = GetIniKeyInt(sTitle, key, wifiFilePath);
            strcpy(key, "freqErr");
            freqErr = GetIniKeyFloat(sTitle, key, wifiFilePath);
            strcpy(key, "useMethod");
            useMethod = GetIniKeyInt(sTitle, key, wifiFilePath);
            strcpy(key, "SNR");
            SNR = GetIniKeyFloat(sTitle, key, wifiFilePath);
            strcpy(key, "onOroff");
            onOroff = GetIniKeyInt(sTitle, key, wifiFilePath);

            if (flag == 0)
            {
                // 赋值
                strcpy(UAVtypes[nUAV].name, name);
                strcpy(UAVtypes[nUAV].vidOrCtr, "wifi");
                for (j = 0; j < nfreqp; j++)
                    UAVtypes[nUAV].freqPoints[j] = freqPoints[j];
                UAVtypes[nUAV].nfreqp = nfreqp;
                UAVtypes[nUAV].isFixedFreq = isFixedFreq;
                UAVtypes[nUAV].hoppType = 0;
                for (j = 0; j < nPulseW; j++)
                {
                    UAVtypes[nUAV].pulseW[j] = 0;
                    UAVtypes[nUAV].pulseWErr[j] = 0;
                    UAVtypes[nUAV].meetPulseW[j] = 0;
                }
                UAVtypes[nUAV].nPulseW = 0;
                UAVtypes[nUAV].isFixedPulseW = 0;
                UAVtypes[nUAV].nPulseT = 0;
                for (j = 0; j < nPulseT; j++)
                    UAVtypes[nUAV].pulseT[j] = 0;
                for (j = 0; j < nPulseBW; j++)
                    UAVtypes[nUAV].pulseBW[j] = pulseBW[j];
                UAVtypes[nUAV].nPulseBW = nPulseBW;
                UAVtypes[nUAV].pulseTErr = pulseTErr;
                UAVtypes[nUAV].meetHopp = 0;
                UAVtypes[nUAV].freqErr = 0;
                UAVtypes[nUAV].useMethod = 0;
                UAVtypes[nUAV].SNR = 0;
                UAVtypes[nUAV].onOroff = 1;
                nUAV++;
            }
        }

        // 打印
        if (printUavFeature > 0)
        {
            printUavlib(UAVtypes, nUAV);
        }

        return nUAV;
    }
    return nUAV;
}
/*
 * 字符转float数组
 */
void str2floatArray(float *arr, int n, char *str)
{
    char *p;
    char delimiter[2] = ",";
    int num = 0;
    int i = 0;
    for (i = 0; i < n; i++)
        arr[i] = 0.0;

    p = strtok(str, delimiter);
    while (p)
    {
        arr[num] = atof(p);
        num++;
        if (num > n)
            break;
        p = strtok(NULL, delimiter);
    }
}

#ifdef SQL_LIB
/*
 * 读取数据库里的机型参数
 */
int readUavlibSql(struct UAVLib *UAVtypes, int &nUAV, char *server_host, char *sql_user_name, char *sql_password, char *db_name)
{
    MYSQL_RES *res_ptr;
    MYSQL_ROW sqlrow;
    int res;
    int field_count = 0;
    int num = 0;
    int sqlType = 0;

    nUAV = 0;

    // 机型库字段
    char name[50];                       // 名称
    char vidOrCtr[20];                   // 图传还是遥控，vid或者ctr
    float freqPoints[100];               // 频点,MHz
    int nfreqp;                          // freqPoints频点个数
    int isFixedFreq;                     // 是否固定频点,>0则是固定频点，=0则只取freqPoints[0]和freqPoints[1]作为频带范围
    int hoppType;                        // 跳频类型，仅对useMethod=2有效。0则要按照freqPoints顺序跳频，1则目标的所有频点频率一致，2则只需要在频段内按周期跳频
    float pulseW[maxPulseWNumInLib];     // 脉宽，ms
    int nPulseW;                         // 脉宽种类数
    int isFixedPulseW;                   // 脉宽是否固定，已经不产生作用（废弃状态）
    float pulseT[5];                     // 脉冲出现的周期,ms。可以有多个，比如（3,4），表示脉冲1与2起始时间相差3ms，脉冲2与3相差4ms,然后类推，脉冲3与4相差3ms
    int nPulseT;                         // 周期个数，如pulseT=(3,4,5)，则nPulseT=3
    float pulseBW[5];                    // 带宽,MHz。针对比如mavic带宽可变的情况
    int nPulseBW;                        // pulseBW总带宽的个数，若大于1则useMethod=1时带宽需要额外搜索确定
    float pulseWErr[maxPulseWNumInLib];  // 脉宽误差,ms。比如pulseW=(3,4)，pulseWErr=(0.1,0.2)则，脉宽范围是3±0.1和4±0.2
    float meetPulseW[maxPulseWNumInLib]; // 满足脉宽的最小次数，>=。比如pulseW=(3,4)，meetPulseW=(2,5)，则必须出现3ms的脉宽不少于2次，且4ms的脉宽不少于5次
    float pulseTErr;                     // 脉冲起始时间满足周期规律的误差,ms
    int meetHopp;                        // 满足周期的脉冲最小个数，>=
    float freqErr;                       // 频率误差范围,MHz。仅对useMethod=2有效
    int useMethod;                       // 列方向分割方法1，主要用于检测图传。带宽较小的信号检测方法2,主要用于遥控检测
    float SNR;                           // 信噪比阈值，仅对useMethod=1有效，高于此信噪比才会被检测到
    int onOroff;                         // 开关,>0则打开
    char mrm[10];                        // 调制识别结果

    static char connect_flag = 0;
    if (connect_flag == 0)
    {
        if (mysql_init(&my_connection) == NULL)
            return -1;
        if (mysql_real_connect(&my_connection, server_host, sql_user_name, sql_password, db_name, 0, NULL, 0))
        {
            connect_flag = 1;
        }
    }
    // mysql_init(&my_connection);
    if (connect_flag)
    {
        // printf("------------------connection success\n");
        mysql_set_character_set(&my_connection, "utf8");
        res = mysql_query(
            &my_connection, "SELECT\
                    id,name,vidOrCtr,freqPoints,nfreqp,isFixedFreq,hoppType,pulseW,nPulseW,isFixedPulseW,pulseT,nPulseT,pulseBW,nPulseBW,pulseWErr,meetPulseW,pulseTErr,\
                    meetHopp,freqErr,useMethod,SNR,onOroff,mrm FROM list");

        if (res)
        {
            sqlType = 1;
            res = mysql_query(
                &my_connection, "SELECT\
                        id,name,vidOrCtr,freqPoints,nfreqp,isFixedFreq,hoppType,pulseW,nPulseW,isFixedPulseW,pulseT,nPulseT,pulseBW,nPulseBW,pulseWErr,meetPulseW,pulseTErr,\
                        meetHopp,freqErr,useMethod,SNR,onOroff FROM list");
        }
        if (res)
        {
            printf("Select Error: %s\n", mysql_error(&my_connection));
            mysql_close(&my_connection);
            connect_flag = 0;
        }
        else
        {
            res_ptr = mysql_use_result(&my_connection);
            if (res_ptr)
            {
                while ((sqlrow = mysql_fetch_row(res_ptr)))
                {
                    num = atoi(sqlrow[0]);
                    strcpy(name, sqlrow[1]);
                    strcpy(vidOrCtr, sqlrow[2]);
                    nfreqp = atoi(sqlrow[4]);
                    str2floatArray(freqPoints, nfreqp, sqlrow[3]);
                    isFixedFreq = atoi(sqlrow[5]) > 0;
                    hoppType = atoi(sqlrow[6]);
                    nPulseW = atoi(sqlrow[8]);
                    str2floatArray(pulseW, nPulseW, sqlrow[7]);
                    isFixedPulseW = atoi(sqlrow[9]) > 0;
                    nPulseT = atoi(sqlrow[11]);
                    str2floatArray(pulseT, nPulseT, sqlrow[10]);
                    nPulseBW = atoi(sqlrow[13]);
                    str2floatArray(pulseBW, nPulseBW, sqlrow[12]);
                    str2floatArray(pulseWErr, nPulseW, sqlrow[14]);
                    str2floatArray(meetPulseW, nPulseW, sqlrow[15]);
                    pulseTErr = atof(sqlrow[16]);
                    meetHopp = atoi(sqlrow[17]);
                    freqErr = atof(sqlrow[18]);
                    useMethod = atoi(sqlrow[19]);
                    SNR = atof(sqlrow[20]);
                    onOroff = atoi(sqlrow[21]);
                    if (sqlType == 1)
                        strcpy(mrm, "UNKNOWN");
                    else if (sqlType == 0)
                        strcpy(mrm, sqlrow[22]);

                    if (nUAV < MaxUAVinLib && onOroff > 0)
                    {
                        // 赋值
                        strcpy(UAVtypes[nUAV].name, name);
                        strcpy(UAVtypes[nUAV].vidOrCtr, vidOrCtr);
                        for (int j = 0; j < nfreqp; j++)
                            UAVtypes[nUAV].freqPoints[j] = freqPoints[j];
                        UAVtypes[nUAV].nfreqp = nfreqp;
                        UAVtypes[nUAV].isFixedFreq = isFixedFreq;
                        UAVtypes[nUAV].hoppType = hoppType;
                        for (int j = 0; j < nPulseW; j++)
                        {
                            UAVtypes[nUAV].pulseW[j] = pulseW[j];
                            UAVtypes[nUAV].pulseWErr[j] = pulseWErr[j];
                            UAVtypes[nUAV].meetPulseW[j] = meetPulseW[j];
                        }
                        UAVtypes[nUAV].nPulseW = nPulseW;
                        UAVtypes[nUAV].isFixedPulseW = isFixedPulseW;
                        UAVtypes[nUAV].nPulseT = nPulseT;
                        for (int j = 0; j < nPulseT; j++)
                            UAVtypes[nUAV].pulseT[j] = pulseT[j];
                        for (int j = 0; j < nPulseBW; j++)
                            UAVtypes[nUAV].pulseBW[j] = pulseBW[j];
                        UAVtypes[nUAV].nPulseBW = nPulseBW;
                        UAVtypes[nUAV].pulseTErr = pulseTErr;
                        UAVtypes[nUAV].meetHopp = meetHopp;
                        UAVtypes[nUAV].freqErr = freqErr;
                        UAVtypes[nUAV].useMethod = useMethod;
                        UAVtypes[nUAV].SNR = SNR;
                        UAVtypes[nUAV].onOroff = onOroff;
                        UAVtypes[nUAV].mrm = MRM_UNKNOWN;
                        if (!strcmp(mrm, "AM"))
                            UAVtypes[nUAV].mrm = MRM_AM;
                        if (!strcmp(mrm, "ASK"))
                            UAVtypes[nUAV].mrm = MRM_ASK;
                        if (!strcmp(mrm, "BPSK"))
                            UAVtypes[nUAV].mrm = MRM_BPSK;
                        if (!strcmp(mrm, "QPSK"))
                            UAVtypes[nUAV].mrm = MRM_QPSK;
                        if (!strcmp(mrm, "8PSK"))
                            UAVtypes[nUAV].mrm = MRM_8PSK;
                        if (!strcmp(mrm, "2FSK"))
                            UAVtypes[nUAV].mrm = MRM_2FSK;
                        if (!strcmp(mrm, "4FSK"))
                            UAVtypes[nUAV].mrm = MRM_4FSK;
                        if (!strcmp(mrm, "UNKNOWN"))
                            UAVtypes[nUAV].mrm = MRM_UNKNOWN;
                        if (!strcmp(mrm, "OFDM"))
                            UAVtypes[nUAV].mrm = MRM_OFDM;

                        nUAV++;
                    }
                    /*
                                field_count=0;
                                while(field_count<mysql_field_count(&my_connection))
                                {
                                    UAVtypes[nUAV].name,
                                    printf("%s  ",sqlrow[field_count]);
                                    field_count++;
                                }
                                printf("\n");
                                */
                }
                mysql_free_result(res_ptr);
            }
            // mysql_close(&my_connection);

            // printUavlib(UAVtypes, nUAV);

            return 1;
        }
    }
    else
    {
        printf("UavLib connection failed!\n");
        if (mysql_errno(&my_connection))
        {
            printf("Connection error %d: %s\n", mysql_errno(&my_connection), mysql_error(&my_connection));
        }
    }

    return 0;
}

#endif

#ifdef SQL_LIB

// /*
//  *  读取机型数据库
//  */
int getUavlib(char *fileName, struct UAVLib *UAVtypes)
{
    int nUAV = 0;
    char server_host[20] = "127.0.0.1";
    char sql_user_name[20] = "root";
    char sql_password[20] = "bekl077";
    char db_name[50] = "uavLib";

    readUavlibSql(UAVtypes, nUAV, server_host, sql_user_name, sql_password, db_name);

    // printf("nUAV=%d\n", nUAV);
    return nUAV;
}
// #else

#endif

// 提取机型库中所有机型覆盖的频率范围
void getFreqSpan(struct UAVLib *UAVtypes, int nUav, float *cenFreqOnAllCh, float dF, int *vidIndex, int *ctrIndex)
{
    int i, j, k;
    int minIndex[2], maxIndex[2];
    int tp, index, indexL, indexR;

    for (i = 0; i < 5; i++)
    {
        minIndex[0] = NFFT;
        minIndex[1] = NFFT;
        maxIndex[0] = 0;
        maxIndex[1] = 0;
        for (j = 0; j < nUav; j++)
        {
            if (UAVtypes[j].useMethod == 1)
                tp = 0;
            else
                tp = 1;

            if (UAVtypes[j].isFixedFreq) // 频率是固定信道
            {
                for (k = 0; k < UAVtypes[j].nfreqp; k++)
                {
                    indexL = floor((UAVtypes[j].freqPoints[k] - cenFreqOnAllCh[i] - UAVtypes[j].pulseBW[UAVtypes[j].nPulseBW - 1] / 2.0) / dF + NFFT / NSumCol / 2);
                    indexR = floor((UAVtypes[j].freqPoints[k] - cenFreqOnAllCh[i] + UAVtypes[j].pulseBW[UAVtypes[j].nPulseBW - 1] / 2.0) / dF + NFFT / NSumCol / 2);
                    if (indexR >= 5 && indexL < NFFT / NSumCol - 5)
                    {
                        if (indexL < minIndex[tp])
                            minIndex[tp] = indexL;
                        if (indexR > maxIndex[tp])
                            maxIndex[tp] = indexR;
                    }
                }
            }
            else // 不固定，频率范围
            {
                indexL = floor((UAVtypes[j].freqPoints[0] - cenFreqOnAllCh[i]) / dF + NFFT / NSumCol / 2);
                indexR = floor((UAVtypes[j].freqPoints[1] - cenFreqOnAllCh[i]) / dF + NFFT / NSumCol / 2 + 1);
                if (indexR >= 5 && indexL < NFFT / NSumCol - 5)
                {
                    if (indexL < minIndex[tp])
                        minIndex[tp] = indexL;
                    if (indexR > maxIndex[tp])
                        maxIndex[tp] = indexR;
                }
            }
        }
        minIndex[0] = (minIndex[0] < 5) ? 5 : minIndex[0];
        minIndex[1] = (minIndex[1] < 5) ? 5 : minIndex[1];
        maxIndex[0] = (maxIndex[0] > (NFFT / NSumCol - 5)) ? (NFFT / NSumCol - 5) : maxIndex[0];
        maxIndex[1] = (maxIndex[1] > (NFFT / NSumCol - 5)) ? (NFFT / NSumCol - 5) : maxIndex[1];

        if (minIndex[0] == NFFT)
        {
            vidIndex[i] = 5;
            if (maxIndex[0] == 0)
                vidIndex[i + 5] = 5;
            else
                vidIndex[i + 5] = maxIndex[0];
        }
        else
        {
            vidIndex[i] = minIndex[0];
            if (maxIndex[0] == 0)
                vidIndex[i + 5] = 5;
            else
                vidIndex[i + 5] = maxIndex[0];
        }

        if (minIndex[1] == NFFT)
        {
            ctrIndex[i] = 5;
            if (maxIndex[1] == 0)
                ctrIndex[i + 5] = 5;
            else
                ctrIndex[i + 5] = maxIndex[1];
        }
        else
        {
            ctrIndex[i] = minIndex[1];
            if (maxIndex[1] == 0)
                ctrIndex[i + 5] = 5;
            else
                ctrIndex[i + 5] = maxIndex[1];
        }
    }
}

/* 获取各频点标校参数,多个d值
 */
// int getCalibLibPro(struct detectParams *detectedParam) // struct calibrateParam *calibParam,
// {
//     int nCalib = 0;
//     int antennaNum = detectedParam->antennaNum;
//     for (int i = 0; i < MaxCalibFreqNum; i++)
//     {
//         detectedParam->calibParamNew[i].calibFreq = detectedParam->calibLibNew[i * (antennaNum * 2 + 1)];
//         for (int j = 0; j < antennaNum; j++)
//         {
//             detectedParam->calibParamNew[i].calibD[j] = detectedParam->calibLibNew[i * (antennaNum * 2 + 1) + 1 + j];
//             detectedParam->calibParamNew[i].calibInitPh[j] = detectedParam->calibLibNew[i * (antennaNum * 2 + 1) + 1 + antennaNum + j];
//         }
//         if (detectedParam->calibParamNew[i].calibFreq <= 0)
//             break;
//         nCalib++;
//     }

//     for (int i = 0; i < MaxCalibFreqNum; i++)
//     {
//         detectedParam->offsetAmp[i].calibFreq = detectedParam->offsetAmpLib[i * (antennaNum + 1)];
//         for (int j = 0; j < antennaNum; j++)
//         {
//             detectedParam->offsetAmp[i].calibAmp[j] = detectedParam->offsetAmpLib[i * (antennaNum + 1) + 1 + j];
//         }
//         if (detectedParam->offsetAmp[i].calibFreq <= 0)
//             break;
//     }

//     for (int i = 0; i < MaxCalibFreqNum; i++)
//     {
//         detectedParam->offsetAzi[i].calibFreq = detectedParam->offsetAziLib[i * (antennaNum + 1)];
//         for (int j = 0; j < antennaNum; j++)
//         {
//             detectedParam->offsetAzi[i].calibAzi[j] = detectedParam->offsetAziLib[i * (antennaNum + 1) + 1 + j];
//         }
//         if (detectedParam->offsetAzi[i].calibFreq <= 0)
//             break;
//     }
//     return nCalib;
// }

// /*
//  * 获取各频点标校参数
//  */
// int getCalibLib(struct detectParams *detectedParam) // struct calibrateParam *calibParam,
// {
//     int nCalib = 0;
//     int antennaNum = detectedParam->antennaNum;
//     for (int i = 0; i < MaxCalibFreqNum; i++)
//     {
//         detectedParam->calibParam[i].calibFreq = detectedParam->calibLib[i * (antennaNum + 2)];
//         detectedParam->calibParam[i].calibD = detectedParam->calibLib[i * (antennaNum + 2) + 1];
//         for (int j = 0; j < antennaNum; j++)
//         {
//             detectedParam->calibParam[i].calibInitPh[j] = detectedParam->calibLib[i * (antennaNum + 2) + 2 + j];
//         }
//         if (detectedParam->calibParam[i].calibFreq <= 0)
//             break;
//         nCalib++;
//     }
//     return nCalib;
// }
