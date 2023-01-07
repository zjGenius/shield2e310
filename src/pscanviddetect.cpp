#include "pscanviddetect.h"
#include "pscanDetectCommon.h"
// extern "C"
// {
// #include"relay.h"
// }
// extern int Send_KP;
// extern int uartfd;
int PWM_detect(Mat Amp, vector<TEMPLATE> &Template, int nRows, float cfreq, float freqResolution)
{
	float row_snr = 0;
	int index = 0;
	for (int i = 0; i < Template.size(); i++)
	{
		if (cfreq - Template[i].LeftFreq > 30.7)
			continue;

		Mat champ = Amp.rowRange(Template[i].CH * nRows, (Template[i].CH + 1) * nRows);
		int count = 0;
		int lp = Template[i].Left_point;
		int rp = Template[i].Right_point;
		for (int j = 0; j < nRows; j++)
		{
			Mat onerowamp = champ.row(j).colRange(lp - 9, rp + 9).clone();
			Mat snr_t = onerowamp.clone();
			snr_t = onerowamp - mean(onerowamp)[0];
			snr_t = snr_t / norm(snr_t);
			snr_t = snr_t / (sum(abs(snr_t))[0] / 2);
			row_snr = sum(onerowamp.dot(snr_t))[0];
			//                         printf(" %f - %f = %f(%d)\n",row_snr,Template[i].Snr,row_snr - Template[i].Snr,j+1);
			if (row_snr + 2 - Template[i].Snr > 1)
			{
				Template[i].Timeind.push_back(j);
				count++;
			}
		}
		Template[i].percentage = count * 1.0 / nRows;
		index++;
	}
	return index;
}

int Gaussianfilter(int Tempsize, float sigma, Mat &signal)
{
	//  [功能]		 :对一维信号的高斯滤波，头尾r/2的信号不进行滤波
	//  Tempsize    :高斯模板的大小推荐奇数
	//  sigma 		:标准差
	//  signal     	:需要进行高斯滤波的序列

	Mat GaussTemp(1, Tempsize * 2 - 1, CV_32F);
	// 生成一维高斯滤波模板
	for (int i = 1; i < Tempsize * 2; i++)
	{
		GaussTemp.at<float>(0, i - 1) = exp(-((i - Tempsize) * (i - Tempsize)) / (2 * sigma * sigma)) / (sigma * sqrt(2 * 3.141592));
	}
	//  高斯滤波
	for (int i = Tempsize - 1; i < signal.cols - Tempsize + 1; i++)
	{
		// cout << signal.colRange(i - Tempsize + 1, i + Tempsize) * (GaussTemp.t()) << endl;
		signal.at<float>(0, i) = Mat(signal.colRange(i - Tempsize + 1, i + Tempsize) * (GaussTemp.t())).at<float>(0);
	}
}

int isContinuous(vector<int> index, vector<int> &ChInd, int ant)
{
	ChInd.resize(0);
	int num1 = 0, endind = 0;

	while (num1 < index.size())
	{
		int num2 = 0;
		while (num1 + num2 + 1 <= index.size() && index[num1] + num2 + 1 == index[num1 + num2 + 1])
		{
			num2 = num2 + 1;
		}
		if (num2 >= 1)
		{
			for (int i = num1; i <= num1 + num2; i++)
			{
				endind += index[i];
			}
			ChInd.push_back(round(endind / (num2 + 1)));
			endind = 0;
		}
		else
		{
			ChInd.push_back(index[num1]);
		}
		num1 = num1 + num2 + 1;
	}

	return 0;
}

int Template_detect(vector<TEMPLATE> &Template, Mat Amp, int nRows, int nCols, int antennaNum, float cfreq, float freqResolution, int gain)
{
	Mat RowSumAmp, OneChAmp;
	Mat Binary = Mat::zeros(1, nCols, CV_32F);
	vector<int> index;
	vector<int> ChInd;
	struct TEMPLATE temp_late;

	for (int i = 0; i < antennaNum; i++)
	{
		index.resize(0);
		OneChAmp = Amp.rowRange(i * nRows, (i + 1) * nRows);
		RowSumAmp = Mat::zeros(1, nCols, CV_32F);

		reduce(OneChAmp, RowSumAmp, 0, 1);
		// printf("ch[%d]amp = %f\n",i,sum(RowSumAmp.colRange(49,449))[0]/400);
		Gaussianfilter(11, 1.5, RowSumAmp);
		// Gaussianfilter(5,5,RowSumAmp);
		if (sum(RowSumAmp.colRange(10, 114))[0] / 104 - (gain - 195) / 2 > 65)
		{
			for (int j = 10; j < 114; j++)
			{
				if (abs(RowSumAmp.at<float>(0, j) - RowSumAmp.at<float>(0, j - 4)) > 6 && abs(RowSumAmp.at<float>(0, j) - RowSumAmp.at<float>(0, j + 4)) < 6)
				{
					Binary.at<float>(0, j) = 1;
					index.push_back(j);
				}
			}
		}
		else
		{
			for (int j = 10; j < 114; j++)
			{
				if (abs(RowSumAmp.at<float>(0, j) - RowSumAmp.at<float>(0, j - 5)) > 3 && abs(RowSumAmp.at<float>(0, j) - RowSumAmp.at<float>(0, j + 5)) < 3)
				{
					Binary.at<float>(0, j) = 1;
					index.push_back(j);
				}
			}
		}
		isContinuous(index, ChInd, i);
		if (ChInd.size() < 2)
			continue;
		for (int j = 1; j < ChInd.size(); j++)
		{
			if (ChInd[j] - ChInd[j - 1] > 14 && ChInd[j] - ChInd[j - 1] < 130)
			{
				//                                 printf("ch = %d ** ind(%d , %d)\t",i,ChInd[j-1],ChInd[j]);
				temp_late.Left_point = ChInd[j - 1];
				temp_late.Right_point = ChInd[j];
				temp_late.LeftFreq = (cfreq - (nCols / 2) * freqResolution) + ChInd[j - 1] * freqResolution;
				temp_late.RightFreq = (cfreq - (nCols / 2) * freqResolution) + ChInd[j] * freqResolution;
				reduce(OneChAmp, RowSumAmp, 0, 1);
				temp_late.signal = RowSumAmp.colRange(ChInd[j - 1] - 5, ChInd[j] + 5).clone();
				Mat snr_t = temp_late.signal.clone();
				snr_t = snr_t - mean(snr_t)[0];
				snr_t = snr_t / norm(snr_t);
				snr_t = snr_t / (sum(abs(snr_t))[0] / 2);
				temp_late.Snr = sum(temp_late.signal.dot(snr_t))[0];
				// printf("SNR = %.2f\n",temp_late.Snr);
				temp_late.CH = i;
				Template.push_back(temp_late);
			}
		}
		// printf("\n");
	}
	return 0;
}

int TargetAndPulse(vector<struct TEMPLATE> &Template, vector<struct detect_pulse> &outvidtarget,
				   vector<struct detect_pulse> &outvidpulse, Mat Amp, float cfreq, float freqResolution, float timeResolution)
{
	struct detect_pulse tempvidtarget;

	FILE *fid = fopen("./log/pscanshortsumcorr.log", "a");
	for (int i = 0; i < Template.size(); i++)
	{
		tempvidtarget.fre = (Template[i].LeftFreq + Template[i].RightFreq) / 2;
		tempvidtarget.start_fre = Template[i].LeftFreq;
		tempvidtarget.end_fre = Template[i].RightFreq;
		tempvidtarget.SNR = Template[i].Snr;
		tempvidtarget.bw = Template[i].RightFreq - Template[i].LeftFreq;
		tempvidtarget.PWM = Template[i].percentage;

		if (tempvidtarget.PWM > 0)
		{
			Mat Amp_ch = Amp.rowRange(Template[i].CH * (Amp.rows / 1), (Template[i].CH + 1) * (Amp.rows / 1));
			//			Mat Phase_ch = Phase.rowRange(Template[i].CH * (Amp.rows/8), (Template[i].CH+1) * (Amp.rows/8));
			int x = round((tempvidtarget.fre - (cfreq - 30.7)) / freqResolution);
			int count = 0;

			if (tempvidtarget.PWM > 0.35)
			{
				int y = Template[i].Timeind[round(Template[i].Timeind.size() / 2)];
				tempvidtarget.meanAmp = Amp_ch.at<float>(y, x);
				//				tempvidtarget.meanPhase = Phase_ch.at<float>(y,x);
				outvidtarget.push_back(tempvidtarget);
				// printf("vidtaget:(freq = %.2f,bw = %.2f,pwm = %.2f,snr = %.2f,meanAmp = %.2f,meanPhase = %.2f) CH = %d\n",
				// 		tempvidtarget.fre, tempvidtarget.bw, tempvidtarget.PWM, tempvidtarget.SNR, tempvidtarget.meanAmp, tempvidtarget.meanPhase, Template[i].CH);
				// fprintf(fid,"vidtaget:(freq = %.2f,bw = %.2f,pwm = %.2f,snr = %.2f,meanAmp = %.2f,meanPhase = %.2f)\n",
				// 		tempvidtarget.fre, tempvidtarget.bw, tempvidtarget.PWM, tempvidtarget.SNR, tempvidtarget.meanAmp, tempvidtarget.meanPhase);
				continue;
			}

			// cout << Template[i].Timeind.size() << endl;
			for (int k = 1; k < Template[i].Timeind.size(); k++)
			{
				if (Template[i].Timeind[k - 1] + 1 != Template[i].Timeind[k])
				{
					// printf("11111111111\n");
					tempvidtarget.start_time = Template[i].Timeind[k - count - 1] * timeResolution;
					tempvidtarget.end_time = Template[i].Timeind[k - 1] * timeResolution;
					// printf("t1 = %d \t t2 = %d\n",Template[i].Timeind[k-count-1],Template[i].Timeind[k-1]);
					tempvidtarget.time = (tempvidtarget.end_time + tempvidtarget.start_time) / 2;
					// cout << tempvidtarget.start_time << "\t" << tempvidtarget.end_time << "\t" <<tempvidtarget.time << endl;
					tempvidtarget.pw = tempvidtarget.end_time - tempvidtarget.start_time;
					int y = round(tempvidtarget.time / timeResolution);
					// cout << tempvidtarget.time << endl;
					tempvidtarget.meanAmp = Amp_ch.at<float>(y, x);
					//					tempvidtarget.meanPhase = Phase_ch.at<float>(y,x);
					// printf("x = %d \t y = %d \t count = %d\n",x,y,count);
					count = 0;
					//					printf("vidpulse:(freq = %.2f,time = %.2f,bw = %.2f,pwm = %.2f,snr = %.2f,meanAmp = %.2f,meanPhase = %.2f) CH = %d\n",
					//							tempvidtarget.fre, tempvidtarget.time, tempvidtarget.bw, tempvidtarget.PWM,
					//							tempvidtarget.SNR, tempvidtarget.meanAmp, tempvidtarget.meanPhase, Template[i].CH);
					fprintf(fid, "vidpulse:(freq = %.2f,time = %.2f,bw = %.2f,pwm = %.2f,snr = %.2f,meanAmp = %.2f,meanPhase = %.2f)\n",
							tempvidtarget.fre, tempvidtarget.time, tempvidtarget.bw, tempvidtarget.PWM,
							tempvidtarget.SNR, tempvidtarget.meanAmp, tempvidtarget.meanPhase);
					outvidpulse.push_back(tempvidtarget);
				}
				else
				{
					count++;
				}
				if (k == Template[i].Timeind.size() - 1 && count > 0)
				{
					// printf("22222222222\n");
					tempvidtarget.start_time = Template[i].Timeind[k - count - 1] * timeResolution;
					tempvidtarget.end_time = Template[i].Timeind[k - 1] * timeResolution;
					// printf("t1 = %d \t t2 = %d\n",Template[i].Timeind[k-count-1],Template[i].Timeind[k-1]);
					tempvidtarget.time = (tempvidtarget.end_time + tempvidtarget.start_time) / 2;
					// cout << tempvidtarget.start_time << "\t" << tempvidtarget.end_time << "\t" <<tempvidtarget.time << endl;
					tempvidtarget.pw = tempvidtarget.end_time - tempvidtarget.start_time;
					int y = round(tempvidtarget.time / timeResolution);
					tempvidtarget.meanAmp = Amp_ch.at<float>(y, x);
					//					tempvidtarget.meanPhase = Phase_ch.at<float>(y,x);
					// printf("x = %d \t y = %d \t count = %d\n",x,y,count);
					count = 0;
					//					printf("vidpulse:(freq = %.2f,time = %.2f,bw = %.2f,pwm = %.2f,snr = %.2f,meanAmp = %.2f,meanPhase = %.2f) CH = %d\n",
					//							tempvidtarget.fre, tempvidtarget.time, tempvidtarget.bw, tempvidtarget.PWM,
					//							tempvidtarget.SNR, tempvidtarget.meanAmp, tempvidtarget.meanPhase, Template[i].CH);
					fprintf(fid, "vidpulse:(freq = %.2f,time = %.2f,bw = %.2f,pwm = %.2f,snr = %.2f,meanAmp = %.2f,meanPhase = %.2f)\n",
							tempvidtarget.fre, tempvidtarget.time, tempvidtarget.bw, tempvidtarget.PWM,
							tempvidtarget.SNR, tempvidtarget.meanAmp, tempvidtarget.meanPhase);
					outvidpulse.push_back(tempvidtarget);
				}
			}
		}
	}
	fclose(fid);
	return 0;
}

bool comp_freq_amp(const detect_pulse &a, const detect_pulse &b)
{
	if (a.fre < b.fre)
		return true;
	else if (a.fre == b.fre && a.meanAmp < b.meanAmp)
		return true;
	else
		return false;
}

int merge(vector<struct detect_pulse> &outvidpulse)
{
	vector<struct detect_pulse> temptargets(outvidpulse);
	outvidpulse.resize(0);

	// cout << "outvidpulse.size = " << outvidpulse.size() << endl;
	// cout << "temptargets.size = " << temptargets.size() << endl;

	sort(temptargets.begin(), temptargets.end(), comp_freq_amp);
	vector<int> index;
	int ind = 0;
	for (int i = 0; i < temptargets.size() - 1; i++)
	{
		if (temptargets[i + 1].fre - temptargets[i].fre < 1)
		{
			// printf("i = %d, ind = %d\n",i,ind);
			if (temptargets[i + 1].meanAmp > temptargets[ind].meanAmp)
			{
				ind = i + 1;
			}
		}
		else
		{
			// printf("i = %d, ind = %d\n",i,ind);
			outvidpulse.push_back(temptargets[ind]);
			ind = i + 1;
		}
		if (i == temptargets.size() - 2)
		{
			// printf("i = %d, ind = %d\n",i,ind);
			outvidpulse.push_back(temptargets[ind]);
			ind = i + 1;
		}
	}
}

int vid_detect(vector<struct TEMPLATE> &Template, float *dataAmp,
			   struct SHORT_DATA_HEAD shortdatahead, vector<struct detect_pulse> &outvidtarget, vector<struct detect_pulse> &outvidpulse)
{
	// if (Template.size() == 0) return 0;
	printf("\n\t\t\t*************  VID_DETECT BEGIN  *************\n\n");
	int time_sample_rate = 1;
	int freq_sample_rate = 1;

	int antennaNum = 1;
	int gain = shortdatahead.gain;
	int nRows = shortdatahead.rows;
	int nCols = shortdatahead.cols;
	int nRowsSample = nRows / time_sample_rate;
	//	int nColsSample = nCols / freq_sample_rate;
	float freqResolution = float(shortdatahead.resolution / 1e6);
	float timeResolution = shortdatahead.timeRresol;
	float cfreq = float(shortdatahead.freq / 1e6);
	printf("cfreq = %f\tgain = %d\tantennaNum = %d\n", cfreq, gain, antennaNum);
	//	printf("nRowsSample = %d\tnColsSample = %d\n",nRows ,nCols);
	//	printf("freqResolution = %f\ttimeResolution = %f\n",freqResolution ,timeResolution);

	if (cfreq == 0 || nRows == 0 || nCols == 0)
		return 0;

	Mat Amp(nRows * antennaNum, nCols, CV_32F, dataAmp);
	//	Mat Phase(nRows * antennaNum, nCols, CV_32F, dataPhase);

	Template_detect(Template, Amp, nRows, nCols, antennaNum, cfreq, freqResolution, gain);
	PWM_detect(Amp, Template, nRows, cfreq, freqResolution);
	if (Template.size() == 0)
		return 0;
	// for (int i = 0; i < Template.size(); i++)
	// {
	// 	printf("CH = %d \t lf = %f \t rf = %f \t lp = %d \t rp = %d \t snr = %f \t pwm = %f\n",
	// 			Template[i].CH,Template[i].LeftFreq,Template[i].RightFreq,Template[i].Left_point,
	// 			Template[i].Right_point,Template[i].Snr,Template[i].percentage);
	// 	cout << Template[i].signal << endl;
	// }
	TargetAndPulse(Template, outvidtarget, outvidpulse, Amp, cfreq, freqResolution, timeResolution);
	if (outvidtarget.size() > 1)
		merge(outvidtarget);
	printf("\n");
	FILE *fid = fopen("./log/pscanshortsumcorr.log", "a");
	for (int i = 0; i < outvidtarget.size(); i++)
	{
		outvidtarget[i].fre = outvidtarget[i].fre + 1;
		printf("vidtaget:(freq = %.2f,bw = %.2f,pwm = %.2f,snr = %.2f,meanAmp = %.2f,meanPhase = %.2f)\n",
			   outvidtarget[i].fre, outvidtarget[i].bw, outvidtarget[i].PWM, outvidtarget[i].SNR, outvidtarget[i].meanAmp, outvidtarget[i].meanPhase);
		fprintf(fid, "vidtaget:(freq = %.2f,bw = %.2f,pwm = %.2f,snr = %.2f,meanAmp = %.2f,meanPhase = %.2f)\n",
				outvidtarget[i].fre, outvidtarget[i].bw, outvidtarget[i].PWM, outvidtarget[i].SNR, outvidtarget[i].meanAmp, outvidtarget[i].meanPhase);
	}
	fclose(fid);
	printf("\n\t\t\t *************  VID_DETECT END  *************\n\n");
}
