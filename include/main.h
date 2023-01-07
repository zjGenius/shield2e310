/***********************************************/
/******************General**********************/
/***********************************************/
#define NROWS 800   // 数据行数
#define NCOLS 128   // 数据列数
#define Freqs 61.44 // Mhz
/***********************************************/
/******************Uavalgorithm*****************/
/***********************************************/
#define PI 3.1415926535898    // 圆周率
#define LightSpeed 299792458  // 光速，m/s
#define EarthRadius 6371393   // 地球半径，m
#define NFFT 128              // FFT点数
#define NSumCol 1             // 频谱累加列数
#define NCorr 5               // 相关组数
#define NCh 10                // 单天线最大通道
#define MaxUAVinLib 200       // 无人机库内最大机型数量
#define MaxPulseWNumInLib 5   // 无人机特征库内脉宽的最大个数
#define MaxPulseInGroup 120   // 脉冲串内的最大脉冲数
#define MaxUAV 5              // 最大检测的无人机个数
#define MaxRecursionCount 500 // 寻找连通区域递归搜索时的最大递归层数
#define MaxCtrPulse 2000      // 单次频谱内的某种型号无人机的遥控检测出的最大脉冲个数
#define MaxNumCtrT 100        // 单次频谱内的某种型号无人机遥控的最大脉冲周期个数
#define MaxVidPulse 50        // 单个脉冲列内检出的满足脉宽条件的最大脉冲个数
#define MaxNumVidT 50         // 单个脉冲列内某种型号无人机图传的最大脉冲周期个数
#define MaxPulseInObj 100     // 每个目标结构体中最大脉冲个数
#define MaxPulseParam 2000    // 脉冲相位参数最大个数
#define MAX_PLANE_AMOUNT 20   // 发送的最大目标信息条数
#define MaxObjNum 20          // 最大目标个数
#define PhaseBinNum 18        // 相位直方图区间个数
#define MaxHistoryNum 1000    // 目标跟踪最大记录条数
#define MaxPulseTimeNum 16    // 最大脉冲时间个数，用于双站关联
#define NSumRow 64            // 频域累加次数
#define MaxCompassAngNum 360  //

/***********************************************/
/******************readDetectParam**************/
/***********************************************/

#define maxPulseWNumInLib 5 // 最大脉宽种类数
#define mouldNum 360        // 定义低频检测模板角度的个数
#define MaxCalibFreqNum 300 // 最大标校频段的个数
#define NSwitchFreq 10      // 最大天线分层数
#define LINE_MAX 1024       // 每行长度最大值
/***********************************************/
/******************paramRead********************/
/***********************************************/
#define BUFFSIZE 8192