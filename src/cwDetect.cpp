#include "pscanctrdetect.h"
#include "pscanviddetect.h"
#include "main.h"
// extern "C"{
//     #include "user_udp.h"
// }
/**************变量声明********************/
extern int workmode;
/*
   动态申请一个大的三维数组
 */
float ***createGrid(int m, int n, int k)
{
    int i, j;
    float ***p = NULL;
    p = (float ***)malloc(m * sizeof(float **));
    for (i = 0; i < m; i++)
    {
        p[i] = (float **)malloc(n * sizeof(float *));
        for (j = 0; j < n; j++)
        {
            p[i][j] = (float *)malloc(k * sizeof(float));
        }
    }
    return p;
}

/*
   释放大的三维数组的内存
 */
void freeGrid(float ***p, int m, int n, int k)
{
    int i, j;
    if (p != NULL)
    {
        for (i = 0; i < m; i++)
        {
            for (j = 0; j < n; j++)
            {
                free(p[i][j]);
            }
            free(p[i]);
        }
        free(p);
    }
}
/**
 * @brief 数据格式整理
 * @param[in] data 原数据
 * @param[in] nrows 数据行数
 * @param[in] ncols 除去帧头帧尾的列数
 * @param[out] dataAmp 幅度
 */
int shortsumcorr_cpu(int *data, float ***Amp, int nrows, int ncols,
                     float *dataAmp)
{
    FILE *fp;
    char filepath[100] = "data.dat";
    fp = fopen(filepath, "w");
    if (fp)
    {
        fwrite(data, sizeof(int), 128 * 800, fp); // 按照128*800读取再转置
    }
    fclose(fp);

    int n = 0;
    for (int i = 0; i < NROWS; i++)
    {
        for (int j = 2; j < 126; j++)
        {
            dataAmp[n] = 10 * log10(data[i * NCOLS + j] / 1.0);
            n++;
        }
    }
    for (int i = 0; i < 1; i++)
    {
        for (int j = 0; j < NROWS; j++)
        {
            for (int k = 0; k < NCOLS - 4; k++)
            {
                Amp[i][j][k] = dataAmp[i * (NCOLS - 4) * NROWS + j * (NCOLS - 4) + k];
            }
        }
    }

    char filepath2[100] = "dataAmp.dat";
    fp = fopen(filepath2, "w");
    if (fp)
    {
        fwrite(dataAmp, sizeof(float), NROWS * (NCOLS - 4), fp);
    }
    fclose(fp);
}
/**
 * @brief 单频检测算法
 * @param[in] sumCorr 原数据带帧头帧尾
 * @param[in] freqcenter 中频单位Mhz
 * @param[in] workmode 选择
 * @param[out] dataAmp 幅度
 */
int cwDetect(int *sumCorr, float freqcenter, int *udpData, vector<struct detect_pulse> outcwpulse, int workmode)
{
    SHORT_DATA_HEAD shortdatahead;
    int antennaNum = 1;
    shortdatahead.cols = NCOLS - 4;
    shortdatahead.rows = NROWS;
    shortdatahead.resolution = Freqs * (1e6) / NFFT; // MHz
    shortdatahead.timeRresol = (double)106 / NROWS;  // ms
    shortdatahead.freq = freqcenter * (1e6);
    shortdatahead.gain = 50;
    float ***integralAmp = createGrid(antennaNum, shortdatahead.rows, shortdatahead.cols);
    float *dataAmp = (float *)malloc(antennaNum * shortdatahead.cols * shortdatahead.rows * sizeof(float));

    int offsetInd = 0;
    int nRows = NROWS;
    int nCols = NCOLS;
    float sumCorrAmp[nRows][nCols], sumCorrPhase[nRows][nCols];

    shortsumcorr_cpu(sumCorr, integralAmp, shortdatahead.rows, shortdatahead.cols, dataAmp);
    vector<struct detect_pulse> outctrpulse;
    vector<struct detect_pulse> outvidtarget;
    vector<struct detect_pulse> outvidpulse;
    vector<struct TEMPLATE> Template;

    mkdir("./log", 0777);
    timeBegin();
    ctr_detect(dataAmp, shortdatahead, outctrpulse, outcwpulse);
    vid_detect(Template, dataAmp, shortdatahead, outvidtarget, outvidpulse);
    timeEnd("end ctr_detect");

    cout << "outctrpulse.size() = " << outctrpulse.size() << endl;
    cout << "outcwpulse.size() = " << outcwpulse.size() << endl;
    cout << "outvidtarget.size() = " << outvidtarget.size() << endl;
    cout << "outvidpulse.size() = " << outvidpulse.size() << endl;
    // 单频检测信息
    for (int i = 0; i < outcwpulse.size(); i++)
    {
        cout << "mun: " << i << endl;
        printf("freq = %.2f, time = %.2f, pw = %.2f, bw = %.2f, snr = %.2f, pwm = %.2f\n",
               outcwpulse[i].fre, outcwpulse[i].time, outcwpulse[i].pw, outcwpulse[i].bw, outcwpulse[i].SNR, outcwpulse[i].PWM);
    }

    freeGrid(integralAmp, antennaNum, shortdatahead.rows, shortdatahead.cols);
    free(dataAmp);

    return 0;
}
