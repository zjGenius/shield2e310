#ifndef _READDETECTPARAM_H
#define _READDETECTPARAM_H

#include "main.h"
enum mrm_type
{
    MRM_UNKNOWN = 0,
    MRM_AM,
    MRM_ASK,
    MRM_NOISE,
    MRM_BPSK,
    MRM_QPSK,
    MRM_8PSK,
    MRM_2FSK,
    MRM_4FSK,
    MRM_OFDM,
};

// 定义标校参数
struct calibrateParam
{
    float calibFreq;
    float calibD;
    float calibInitPh[8];
};

// 定义标校参数(多个d值)
struct calibrateParamPro
{
    float calibFreq;               // 频点
    float phRefAzi[NCh];           // 相位参考方位角
    float calibD[NCh];             // d值
    float calibInitPh[NCh];        // 初相
    float phPoly[NCh][5];          // 比相测角拟合参数
    float phFitAccuracy[NCh];      // 比相测角拟合精度
    float phLowUpLimit[NCh][2];    // 相位上下边界
    float ampPoly[NCh][5];         // 比幅测角拟合参数
    float ampFitAccuracy[NCh];     // 比幅测角拟合精度
    float ampLowUpLimit[NCh][2];   // 幅度上下边界
    float ampBeamCent[NCh];        // 波束中心指向
    float ampBeamW[NCh];           // 波束中心带宽
    float maxAmpPattern[NCh];      // 方向图最大幅度
    float patternFitAccuracy[NCh]; // 方向图拟合精度
    float slopeFreq[NCh];          // 基准相位随频率变化斜率    //20220919,参数修改为天线层数。
};

// 定义幅度标校参数(多个d值)
struct offsetAmpParam
{
    float calibFreq;
    float calibAmp[NCh];
};

// 定义幅度标校参数(多个d值)
struct offsetAziParam
{
    float calibFreq;
    float calibAzi[NCh];
};

// 检测算法参数
struct detectParams
{
    int writeIQ;   // 是否写入IQ文件
    int corrOnOff; // mavic检测开关，0关闭，1打开
    int gfskOnOff; // gfsk检测开关，0关闭，1打开
    int wifiOnOff; // wifi检测开关，0关闭，1打开
    // 打印和文件保存
    int printParam;          // 是否打印读取的参数
    char filePrefix[100];    // 相关数据存文件，文件名前缀
    int writeSumCorr;        // 保存相关数据的开关，>0则保存
    int writeNewSumCorr;     // 保存相关数据的开关，>0则保存
    int SumCorrsize;         // 相关数据容量
    int SumCorrContinuation; // 删除相关数据的开关，>0则开始删除容量满后的最开始数据
    int writeRef;            // 开关，保存单频参考信号的幅度和相位，>0则保存
    int writeVidPulse;       // 保存图传脉冲信息的开关，>0则保存
    int writeCtrPulse;       // 保存遥控脉冲信息的开关，>0则保存
    int writeCaliPulse;      // 外场自动标校用日志，>0则保存
    int writeObj;            // 保存目标信息的开关，>0则保存
    int writeDeblurLog;      // 保存解模糊过程，>0则保存
    int saveImage;           // 保存频谱截图的开关，>0则保存
    char objPrefix[100];     // 疑似目标(测频测带宽不准的真实目标)名称前缀

    // 检测参数
    //    float mask[35];          //图传检测的滤波模板
    float *mask;
    int maskH; // 模板行数
    int maskW; // 模板列数
    //    float hpMask[9];        //高通滤波模板
    float *hpMask;
    int hpMaskH;          // 模板行数
    int hpMaskW;          // 模板列数
    float filterThresh;   // 剔除野值再滤波门限
    float dataTime;       // 数据时长，ms
    float DT;             // 阈值更新的最小变化量
    int MaxIter;          // 截面求分割阈值的最大迭代次数
    int Delt;             // 填充的最大沟壑点数
    int MinW;             // 分割的最小脉冲宽度，小于此脉宽的则丢弃，ms
    int q1IndexErr;       // 判断是否是同一个无人机，截面边缘对齐的最大偏差行数
    int qMeetTimes;       // 满足截面边缘对齐的最小次数，有多个边缘可以对齐则认为是同一个无人机
    float ampErr;         // 对于中心频点不固定的图传，搜索带宽时平均幅度的容差,dB
    float checkBWAmpErr;  ////对于中心频点不固定的图传，优化测频测带宽时平均幅度的容差,(复数dB)
    float ctrSNR;         // 遥控脉冲分割的最小信噪比
    float archimedeanSNR; // 八面螺旋解模糊门限
    float minCtrPulseW;   // 遥控脉冲的最小脉宽限制
    float maxCtrPulseW;   // 遥控脉冲的最大脉宽限制

    // 测向参数
    int antennaNum;              // 天线组数
    int doaTyp;                  // 测向类型：0表示2.4G相位，5.8G幅度;1表示2.4G相位，5.8G相位;2表示只支持800M的4天线相位;3表示表示800-6000M相位;4表示双层天线;5表示阿基米德螺旋天线
    int LPDAType;                // 对周天线类型，1表示拉大间距天线，0表示未拉大
    int isUseAmpLib;             // 幅度标校
    int isUseAziLib;             // 波束指向标校
    int isCaliFreqBw;            // 是否优化测评测带宽 0表示不优化，按算法本身中频带宽，1表示优化
    int bwFreqType;              // 遥控信号上报类型，0为脉冲块带宽中频，1为跳频范围带宽中频
    int ampSelAntType;           // 选天线方案
    int antShape7UseMethod;      // 对周解模糊方案
    int filterMethod;            // 滤波方法，0为幅度滤波，1为复数滤波，2为积分图像滤波
    int archi433AziMethod;       // 螺旋天线433频点测向方法(0为常规，1为处理波束偏移)
    int calibType;               // 标校类型选择
    float fitFreqSpan[2];        // 拟合测角频段范围
    int fitPh;                   // 拟合测角开关
    float shieldPhFreqSpan[40];  // 屏蔽相位的频段范围
    int shieldPhN;               // 屏蔽相位的频段个数
    int Lateral;                 // 侧向结果处理
    float initial_azi;           // 前⼀个上报⽅位⾓
    float azi_step;              // 角度步进
    float rand_err;              // 随机误差
    float d433;                  // 433M天线间距m
    float initPhase433[5];       // 433M初始相位
    float d800;                  // 800M天线间距m
    float initPhase800[5];       // 800M初始相位
    float fitCoef800[24];        // 800M四天线（夹角）拟合参数
    float d920coef[3];           //
    float d1800;                 // 1.8G的天线距离,m
    float initPhase1800[5];      // 1.8G的初始相位
    float d2450;                 // 2.4G的天线距离，m
    float initPhase2450[5];      // 2.4G初始相位
    float fitCoef2450[24];       // 2450四天线（夹角）拟合参数
    float d5800;                 // 5.8G的天线距离，m
    float initPhase5800[5];      // 5.8G的初始相位
    float fitCoef5800[24];       // 5800四天线（夹角）拟合参数
    float miu;                   // 距离参数，越大目标的距离越大
    float ant1Angle;             // 设置天线1的角度，默认为0
    float refFreq;               // 注入标校用的连续波频率，MHz
    float offsetAmp433[8];       // 433M幅度补偿
    float offsetAmp800[8];       // 800M幅度补偿
    float offsetAmp1800[8];      // 1800M幅度补偿
    float offsetAmp2450[8];      // 2450M幅度补偿
    float offsetAmp5800[8];      // 5800M幅度补偿
    float offsetAmp433LOrR[8];   // 433M幅度补偿(单边)
    float offsetAmp800LOrR[8];   // 800M幅度补偿
    float offsetAmp915LOrR[8];   // 915M幅度补偿
    float offsetAmp1800LOrR[8];  // 1800M幅度补偿
    float offsetAmp2450LOrR[8];  // 2450M幅度补偿
    float offsetAmp5800LOrR[16]; // 5800M幅度补偿
    float coef[15];              // 比幅测角系数
    int fit;                     // 测向是否拟合
    float overlapAngle;          // 两组天线之间过渡带角度
    int frameNum;

    // 目标关联参数
    float maxUavSpeed;     // 无人机最大速度，m/s
    float maxCorrTime;     // 最大关联时间
    int usefulCount;       // 出现次数大于此限度，认为目标有效
    char confidenceThresh; // 置信度（百分数）大于此限度，认为目标有效
    float vidAngleErr;     // 图传角度偏差
    float ctrAngleErr;     // 遥控角度偏差
    int vidFitPoints;      // 图传平滑点数
    int ctrFitPoints;      // 遥控平滑点数
    float timeOut;         // 目标过期时间（s），过期则清除
    float freqErr;         // 中心频率误差

    float freqSpan;      // 搜索的带宽范围，MHz
    float boxAmpErr;     // 搜索时的幅度误差范围，dB
    float boxPhaseErr;   // 搜索时相位误差
    float boxTimeOut;    // 框选目标失效时间
    float boxCmdTimeOut; // 失去界面跟踪指令后，维持跟踪的时间

    // 画图
    int colorMapSel;      // 颜色图,0:jet,1:hot,2:cool,3:gray,4:summer,5:winter
    int showImage;        // 画图，0表示不画，1表示画相关谱
    int nSampPlot;        // 画图抽样，每sampPlot个点抽取一点。越大，则瀑布图高度越小
    float ampThreshShow;  // 幅度阈值，高于阈值的则显示
    float ampMinMax[2];   // 幅度谱最小和最大值,用于颜色映射。若设为0,0表示颜色自动调节
    int markVidPulse;     // 是否标记图传脉冲
    int markCtrPulse;     // 是否标记遥控脉冲
    int showHist;         // 直方图，0：不显示，1：图传的相位，2：图传的幅度加权相位，3：遥控的相位，4：遥控的幅度加权相位
    int whichUavShowHist; // 需要显示直方图的无人机ID号
    int plotPath;         // 是否显示历史轨迹
    int historyPointNum;  // 保留历史轨迹的点数,<1000
    int showLog;          // 是否显示日志信息
    float maxRadius;      // 雷达图最大半径,大于0有效
    int showSlice;        // 显示频谱信号截面，若大於0,则点左键显示频率方向，点右键显示时间方向

    float beam_width;   // 波束宽度
    float amp_obs_std;  // 比幅计算方差参数，幅度容差(单位dB)
    float phi_obs_std;  // 相位容差(5/180×pi)
    float angleSpan;    // 合理相位的最大角度
    float probMultiply; // 相位先验信息参数

    int vidIndexSpan[10]; // 图传搜索的频带范围
    int ctrIndexSpan[10]; // 遥控搜索的频带范围

    // 偶极子天线标校
    //  float phMould433[mouldNum * 4]; //相位模板433MHz
    //  float phMould315[mouldNum * 4]; //相位模板315MHz

    // 天线标校数据
    int calibrationType;
    // float calibLib[MaxCalibFreqNum * 17];
    // float calibLibNew[MaxCalibFreqNum * 17];
    // struct calibrateParam calibParam[MaxCalibFreqNum];
    // struct calibrateParamPro calibParamNew[MaxCalibFreqNum];
    // float offsetAmpLib[MaxCalibFreqNum * 9];
    // float offsetAziLib[MaxCalibFreqNum * 9];
    // struct offsetAmpParam offsetAmp[MaxCalibFreqNum];
    // struct offsetAziParam offsetAzi[MaxCalibFreqNum];
    // float calibAmp[MaxCalibFreqNum];

    // 天线分层信息数据
    int shapeEnable;                  // 分层信息是否有效
    int antSwNum;                     // 天线分段数，即天线层数
    float antSwitchFreq[NSwitchFreq]; // 天线分层频点
    int calibLibInd;                  // 选择的天线标校数据序号
    float centFreq;                   // 扫频的中心频率
    int validSwInd;                   // 选择的层数，0为未选择，
    int antSwOnOff;                   // 天线选择方法开关

    float matchCoef;     // 快速wifi的相关系数
    int sumcorrFreamNum; // 帧数
    int AntennaShape;    // 天线类型
};

// 无人机库
struct UAVLib
{
    char name[50];                      // 名称
    char vidOrCtr[20];                  // 图传还是遥控，vid或者ctr
    float freqPoints[100];              // 频点,MHz
    int nfreqp;                         // freqPoints频点个数
    int isFixedFreq;                    // 是否固定频点,>0则是固定频点，=0则只取freqPoints[0]和freqPoints[1]作为频带范围
    int hoppType;                       // 跳频类型，仅对useMethod=2有效。0则要按照freqPoints顺序跳频，1则目标的所有频点频率一致，2则只需要在频段内按周期跳频
    float pulseW[maxPulseWNumInLib];    // 脉宽，ms
    int nPulseW;                        // 脉宽种类数
    int isFixedPulseW;                  // 脉宽是否固定，已经不产生作用（废弃状态）
    float pulseT[5];                    // 脉冲出现的周期,ms。可以有多个，比如（3,4），表示脉冲1与2起始时间相差3ms，脉冲2与3相差4ms,然后类推，脉冲3与4相差3ms
    int nPulseT;                        // 周期个数，如pulseT=(3,4,5)，则nPulseT=3
    float pulseBW[5];                   // 带宽,MHz。针对比如mavic带宽可变的情况
    int nPulseBW;                       // pulseBW总带宽的个数，若大于1则useMethod=1时带宽需要额外搜索确定
    float pulseWErr[maxPulseWNumInLib]; // 脉宽误差,ms。比如pulseW=(3,4)，pulseWErr=(0.1,0.2)则，脉宽范围是3±0.1和4±0.2
    int meetPulseW[maxPulseWNumInLib];  // 满足脉宽的最小次数，>=。比如pulseW=(3,4)，meetPulseW=(2,5)，则必须出现3ms的脉宽不少于2次，且4ms的脉宽不少于5次
    float pulseTErr;                    // 脉冲起始时间满足周期规律的误差,ms
    int meetHopp;                       // 满足周期的脉冲最小个数，>=
    float freqErr;                      // 频率误差范围,MHz。仅对useMethod=2有效
    int useMethod;                      // 列方向分割方法1，主要用于检测图传。带宽较小的信号检测方法2,主要用于遥控检测
    float SNR;                          // 信噪比阈值，仅对useMethod=1有效，高于此信噪比才会被检测到
    int onOroff;                        // 开关,>0则打开
    int mrm;                            // 调制识别类型
};

int checkAntennaTyp(int *antennaNum, int *doaTyp, int antFlag);       // 检查天线类型
int getDetectParam(struct detectParams *param, char *fileName);       // 读取算法参数文件
int getUavlib(char *, struct UAVLib *);                               // 读取无人机库
int getUavlibCW(char *, struct UAVLib *);                             // 读取无人机库
int getCalibSql(struct detectParams *);                               // 读取sql标校
void getFreqSpan(struct UAVLib *, int, float *, float, int *, int *); // 估计计算的频带范围
// int getCalibLib(struct detectParams *);                               // struct calibrateParam *calibParam,
// int getCalibLibPro(struct detectParams *);                            // struct calibrateParam *calibParam,

#endif
