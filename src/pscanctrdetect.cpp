#include <stdio.h>
#include "pscanctrdetect.h"
#include "pscanDetectCommon.h"
// extern "C"
// {
// #include"relay.h"
// }
struct timeval timep1, timep2;
int tm_sec;
struct tm *p1, *p2;
extern int uartfd;
extern int Send_CW;
void timeBegin()
{

	gettimeofday(&timep1, NULL);
	p1 = localtime(&timep1.tv_sec);
	tm_sec = p1->tm_sec;
}

void timeEnd(std::string s)
{
	gettimeofday(&timep2, NULL);
	p2 = localtime(&timep2.tv_sec);
	printf("end Detect %02d:%02d:%02d.%03d\n", p2->tm_hour, p2->tm_min, p2->tm_sec, (int)timep2.tv_usec / 1000);
	printf("-----------------------------%s diff time = %.2f ms\n\n", s.c_str(), (p2->tm_min - p1->tm_min) * 60 * 1000 + (p2->tm_sec - tm_sec) * 1000 + timep2.tv_usec / 1000.0 - timep1.tv_usec / 1000.0);
}

int max_ampch(float *data)
{
	int ind;
	float miny = 0;
	for (int i = 0; i < 8; i++)
	{
		if (data[i] > miny)
		{
			miny = data[i];
			ind = i;
		}
	}
	return ind;
}

int ch_sel(vector<int> data, int len)
{
	float x[8] = {0};
	int y;
	for (int i = 0; i < len; i++)
	{
		x[data[i]] = x[data[i]] + 1;
	}
	int ind = max_ampch(x);
	return ind;
}

int cw_sel(Mat data, vector<int> &cwindex, vector<float> &rowsum)
{
	int index = 0;
	for (int i = 0; i < data.cols; i++)
	{
		if (sum(data.col(i))[0] > data.rows * 0.6)
		{
			rowsum.push_back(sum(data.col(i))[0] / data.rows);
			cwindex.push_back(i);
		}
	}
	return 0;
}

bool comp(const detect_pulse &a, const detect_pulse &b)
{
	if (a.start_fre < b.start_fre)
		return true;
	else if (a.start_fre == b.start_fre && a.start_time < b.start_time)
		return true;
	else
		return false;
}

int cwtaget_merge(vector<detect_pulse> &outcwtarget)
{
	vector<detect_pulse> tempcwtarget(outcwtarget);
	outcwtarget.resize(0);

	if (tempcwtarget.size() < 2)
		return 0;

	for (int i = 1; i < tempcwtarget.size(); i++)
	{
		if (fabs(tempcwtarget[i].fre - tempcwtarget[i - 1].fre) < 0.5)
		{
			tempcwtarget[i].pw = tempcwtarget[i].pw + tempcwtarget[i - 1].pw;
			tempcwtarget[i].PWM = tempcwtarget[i].PWM + tempcwtarget[i - 1].PWM;
			if (i == tempcwtarget.size() - 1)
			{
				if (tempcwtarget[i].PWM > 1)
					tempcwtarget[i].PWM = 1;
				outcwtarget.push_back(tempcwtarget[i]);
			}
		}
		else
		{
			if (tempcwtarget[i - 1].PWM > 1)
				tempcwtarget[i - 1].PWM = 1;
			outcwtarget.push_back(tempcwtarget[i - 1]);
			if (i == tempcwtarget.size() - 1)
				outcwtarget.push_back(tempcwtarget[i]);
		}
	}
}

int merge_pulses(vector<ctrPulses> ctrpulse, int end_ch, vector<detect_pulse> &outctrpulse)
{
	int count = 1;
	float sumamp = 0, sumphase = 0, sumsnr = 0;
	struct detect_pulse tmpctr;
	vector<detect_pulse> midctrpulse;
	// freq merge
	for (int i = 1; i < ctrpulse.size(); i++)
	{
		if (fabs(ctrpulse[i].start_time - ctrpulse[i - 1].start_time) < 0.1 && fabs(ctrpulse[i].start_fre - ctrpulse[i - 1].start_fre) < 0.4)
		{
			count++;
			if (i == ctrpulse.size() - 1)
			{
				tmpctr.start_time = ctrpulse[i - count + 1].start_time;
				tmpctr.start_fre = ctrpulse[i - count + 1].start_fre;
				tmpctr.end_time = ctrpulse[i].end_time;
				tmpctr.end_fre = ctrpulse[i].end_fre;
				tmpctr.PWM = ctrpulse[i].PWM;
				tmpctr.fre = (tmpctr.end_fre + tmpctr.start_fre) / 2;
				tmpctr.time = (tmpctr.end_time + tmpctr.start_time) / 2;
				tmpctr.bw = tmpctr.end_fre - tmpctr.start_fre;
				tmpctr.pw = tmpctr.end_time - tmpctr.start_time;
				for (int j = i - count + 1; j < i + 1; j++)
				{
					sumamp += ctrpulse[j].meanAmp[end_ch];
					sumphase += ctrpulse[j].meanPhase[end_ch];
					sumsnr += ctrpulse[j].SNR;
				}
				tmpctr.meanAmp = sumamp / count;
				tmpctr.meanPhase = sumphase / count;
				tmpctr.SNR = sumsnr / count;
				tmpctr.Prob = 0.9;
				midctrpulse.push_back(tmpctr);
				sumamp = 0;
				sumphase = 0;
				sumsnr = 0;
				count = 1;
			}
		}
		else
		{
			tmpctr.start_time = ctrpulse[i - count].start_time;
			tmpctr.start_fre = ctrpulse[i - count].start_fre;
			tmpctr.end_time = ctrpulse[i - 1].end_time;
			tmpctr.end_fre = ctrpulse[i - 1].end_fre;
			tmpctr.PWM = ctrpulse[i].PWM;
			tmpctr.fre = (tmpctr.end_fre + tmpctr.start_fre) / 2;
			tmpctr.time = (tmpctr.end_time + tmpctr.start_time) / 2;
			tmpctr.bw = tmpctr.end_fre - tmpctr.start_fre;
			tmpctr.pw = tmpctr.end_time - tmpctr.start_time;
			for (int j = i - count; j < i; j++)
			{
				sumamp += ctrpulse[j].meanAmp[end_ch];
				sumphase += ctrpulse[j].meanPhase[end_ch];
				sumsnr += ctrpulse[j].SNR;
			}
			tmpctr.meanAmp = sumamp / count;
			tmpctr.meanPhase = sumphase / count;
			tmpctr.SNR = sumsnr / count;
			tmpctr.Prob = 0.9;
			midctrpulse.push_back(tmpctr);
			sumamp = 0;
			sumphase = 0;
			sumsnr = 0;
			count = 1;
		}
	}
	sort(midctrpulse.begin(), midctrpulse.end(), comp);
	// time merge
	if (midctrpulse.size() == 1)
	{
		outctrpulse.push_back(midctrpulse[0]);
		return 0;
	}
	// cout << "\nend ctr_pulse:" << endl;
	for (int i = 1; i < midctrpulse.size(); i++)
	{
		if (fabs(midctrpulse[i].start_fre - midctrpulse[i - 1].start_fre) < midctrpulse[i - 1].bw && fabs(midctrpulse[i].start_time - midctrpulse[i - 1].start_time) < 0.4)
		{
			count++;
			if (i == midctrpulse.size() - 1)
			{
				if (count == 1)
					continue;
				tmpctr.start_time = midctrpulse[i - count + 1].start_time;
				tmpctr.start_fre = midctrpulse[i - count + 1].start_fre;
				tmpctr.end_time = midctrpulse[i].end_time;
				tmpctr.end_fre = midctrpulse[i].end_fre;
				tmpctr.PWM = midctrpulse[i].PWM;
				tmpctr.fre = (tmpctr.end_fre + tmpctr.start_fre) / 2;
				tmpctr.time = (tmpctr.end_time + tmpctr.start_time) / 2;
				tmpctr.bw = tmpctr.end_fre - tmpctr.start_fre;
				tmpctr.pw = tmpctr.end_time - tmpctr.start_time;
				for (int j = i - count + 1; j < i + 1; j++)
				{
					sumamp += midctrpulse[j].meanAmp;
					sumphase += midctrpulse[j].meanPhase;
					sumsnr += midctrpulse[j].SNR;
				}
				tmpctr.meanAmp = sumamp / count;
				tmpctr.meanPhase = sumphase / count;
				tmpctr.SNR = sumsnr / count;
				tmpctr.Prob = 0.9;
				// printf("freq = %.2f\ttime = %.2f\t",tmpctr.fre,tmpctr.time);
				// printf("sf = %.2f\tef = %.2f\tst = %.2f\tet = %.2f\t",tmpctr.start_fre,tmpctr.end_fre,tmpctr.start_time,tmpctr.end_time);
				// printf("pw = %.2f\tbw = %.2f\tmeanamp = %.2f\tmeanphase = %.2f\n",tmpctr.pw,tmpctr.bw,tmpctr.meanAmp,tmpctr.meanPhase);
				outctrpulse.push_back(tmpctr);
				sumamp = 0;
				sumphase = 0;
				sumsnr = 0;
				count = 1;
			}
		}
		else
		{
			if (count == 1)
			{
				outctrpulse.push_back(midctrpulse[i - 1]);
				continue;
			}
			tmpctr.start_time = midctrpulse[i - count].start_time;
			tmpctr.start_fre = midctrpulse[i - count].start_fre;
			tmpctr.end_time = midctrpulse[i - 1].end_time;
			tmpctr.end_fre = midctrpulse[i - 1].end_fre;
			tmpctr.PWM = midctrpulse[i].PWM;
			tmpctr.fre = (tmpctr.end_fre + tmpctr.start_fre) / 2;
			tmpctr.time = (tmpctr.end_time + tmpctr.start_time) / 2;
			tmpctr.bw = tmpctr.end_fre - tmpctr.start_fre;
			tmpctr.pw = tmpctr.end_time - tmpctr.start_time;
			for (int j = i - count; j < i; j++)
			{
				sumamp += midctrpulse[j].meanAmp;
				sumphase += midctrpulse[j].meanPhase;
				sumsnr += midctrpulse[j].SNR;
			}
			tmpctr.meanAmp = sumamp / count;
			tmpctr.meanPhase = sumphase / count;
			tmpctr.SNR = sumsnr / count;
			tmpctr.Prob = 0.9;
			// printf("freq = %.2f\ttime = %.2f\t",tmpctr.fre,tmpctr.time);
			// printf("sf = %.2f\tef = %.2f\tst = %.2f\tet = %.2f\t",tmpctr.start_fre,tmpctr.end_fre,tmpctr.start_time,tmpctr.end_time);
			// printf("pw = %.2f\tbw = %.2f\tmeanamp = %.2f\tmeanphase = %.2f\n",tmpctr.pw,tmpctr.bw,tmpctr.meanAmp,tmpctr.meanPhase);
			// printf("meanAmp = %.2f\tmeanPhase = %.2f\n\n",tmpctr.meanAmp,tmpctr.meanPhase);
			outctrpulse.push_back(tmpctr);
			sumamp = 0;
			sumphase = 0;
			sumsnr = 0;
			count = 1;
		}
	}

	return 0;
}

int rectwave_filter_Opencv(Mat &this_rectwave_heatmap, float *integralAmp, int rect_window, int dim, int nRowsSample, int nColsSample)
{
	// 方波两边的大小
	int edge_window = int(rect_window / 2.0 + 0.5);
	int whole_window = rect_window + 2 * edge_window;
	float rect_weight_1 = 1 / (rect_window * 1.0);
	float rect_weight_2 = 1 / ((whole_window - rect_window) * 1.0);
	// 边缘两边大小
	int left_window = int(whole_window / 2.0 + 0.5);
	int right_window = whole_window - left_window;
	float edge_weight_1 = 1 / (right_window * 1.0);
	float edge_weight_2 = 1 / ((whole_window - right_window) * 1.0);
	int effect_size = nColsSample - whole_window;

	Mat tmpamp(nRowsSample, nColsSample, CV_32F, integralAmp);
	// cout << tmpamp << endl;

	// Mat t_amp = tmpamp.t();
	Mat cum_vals = Mat::zeros(nRowsSample, nColsSample, CV_32F);

	for (int i = 1; i < nColsSample; i++)
	{
		cum_vals.col(i) = tmpamp.col(i - 1) + cum_vals.col(i - 1);
	}
	if (dim == 1)
	{
	}
	else if (dim == 2)
	{
		Mat rect_whole_hm = Mat::zeros(nRowsSample, effect_size, CV_32F);
		Mat rect_hm = Mat::zeros(nRowsSample, effect_size, CV_32F);
		Mat edge_hm = Mat::zeros(nRowsSample, effect_size, CV_32F);
		Mat effect_rectwave_hm = Mat::zeros(nRowsSample, effect_size, CV_32F);
		Mat rectwave_hm = Mat::zeros(nRowsSample, effect_size + 8, CV_32F);

		rect_whole_hm = cum_vals.colRange(whole_window, whole_window + effect_size) - cum_vals.colRange(0, effect_size);
		rect_hm = cum_vals.colRange(edge_window + rect_window, edge_window + rect_window + effect_size) - cum_vals.colRange(edge_window, edge_window + effect_size);
		edge_hm = cum_vals.colRange(whole_window, whole_window + effect_size) - cum_vals.colRange(left_window, left_window + effect_size);

		rect_hm = (rect_hm * rect_weight_1) - ((rect_whole_hm - rect_hm) * rect_weight_2);
		edge_hm = (edge_hm * edge_weight_1) - ((rect_whole_hm - edge_hm) * edge_weight_2);

		effect_rectwave_hm = rect_hm - cv::abs(edge_hm);
		for (int i = 0; i < 4; i++)
		{
			rectwave_hm.col(i) = effect_rectwave_hm.col(0) * 1;
			rectwave_hm.col(effect_size + 4 + i) = effect_rectwave_hm.col(effect_size - 1) * 1;
		}
		rectwave_hm.colRange(4, 4 + effect_size) = effect_rectwave_hm.colRange(0, effect_size) * 1;
		this_rectwave_heatmap = rectwave_hm;
	}
	return 0;
}

int narrowband_pulse_detect_v2(float *dataAmp, vector<ctrPulses> &ctrpulse, vector<ctrPulses> &cwpulse,
							   int antennaNum, int nRowsSample, int nColsSample, double cfreq, double freqResolution, double timeResolution)
{
	int s_ch = 0;

	float hm_threshold = 4, rect_windows = 3;
	int dim = 2;

	Mat this_rectwave_heatmap = Mat::zeros(nRowsSample, nColsSample, CV_32F);
	Mat binary_segs = Mat::zeros(antennaNum * nRowsSample, nColsSample, CV_32F);
	for (int i = 0; i < antennaNum; i++)
	{
		rectwave_filter_Opencv(this_rectwave_heatmap, dataAmp + (nRowsSample * nColsSample * i), rect_windows, dim, nRowsSample, nColsSample);

		for (int k = 0; k < nRowsSample; k++)
		{
			for (int j = 0; j < nColsSample; j++)
			{
				if (this_rectwave_heatmap.at<float>(k, j) > hm_threshold)
					binary_segs.at<float>((i * nRowsSample) + k, j) = 1;
			}
		}
	}
	struct index inds;
	int sum_binary_col = 0;
	Mat fused_binary_segs = Mat::zeros(nRowsSample, nColsSample, CV_32F);

	float *cdata = (float *)malloc(nRowsSample * nColsSample * sizeof(float));

	for (int j = 0; j < nRowsSample; j++)
	{
		for (int k = 0; k < nColsSample; k++)
		{
			for (int i = 0; i < antennaNum; i++)
			{
				sum_binary_col += binary_segs.at<float>((i * nRowsSample) + j, k);
			}
			if (sum_binary_col > s_ch)
			{
				fused_binary_segs.at<float>(j, k) = 1;
				inds.x.push_back(j);
				inds.y.push_back(k);
				inds.num++;
			}
			cdata[j * nColsSample + k] = fused_binary_segs.at<float>(j, k);
			sum_binary_col = 0;
		}
	}
	FILE *fp = fopen("./checkdata", "w");
	fwrite(cdata, sizeof(float), nRowsSample * nColsSample, fp);
	fclose(fp);
	free(cdata);
	// cout << fused_binary_segs.colRange(150,400) << endl;
	// imshow("2",fused_binary_segs);
	// waitKey(10);
	// while (1)
	// {
	// 	/* code */
	// }

	struct ctrPulses tmpctrp;
	ctrpulse.resize(0);
	cwpulse.resize(0);
	Mat SNR_A(1, 7, CV_32F);
	Mat SNR_B(1, 7, CV_32F);
	vector<int> ch_ind;
	vector<int> cwindex;
	vector<float> rowsum;

	cw_sel(fused_binary_segs, cwindex, rowsum);

	for (int i = 0; i < inds.num; i++)
	{
		tmpctrp.start_time = inds.x[i] * timeResolution;
		tmpctrp.end_time = (inds.x[i] + 1) * timeResolution;
		// tmpctrp.start_fre = (cfreq - 76.8/cfreqdim + freqResolution * 5) + ((inds.y[i]) * freqResolution);
		// tmpctrp.end_fre = (cfreq - 76.8/cfreqdim + freqResolution * 5) + ((inds.y[i] + 1) * freqResolution);
		tmpctrp.start_fre = (cfreq - 30.7) + ((inds.y[i]) * freqResolution);
		tmpctrp.end_fre = (cfreq - 30.7) + ((inds.y[i] + 1) * freqResolution);

		tmpctrp.time = (tmpctrp.end_time + tmpctrp.start_time) / 2;
		tmpctrp.fre = (tmpctrp.end_fre + tmpctrp.start_fre) / 2;
		tmpctrp.pw = tmpctrp.end_time - tmpctrp.start_time;
		// tmpctrp.bw = tmpctrp.end_fre - tmpctrp.start_fre + freqResolution + exp(log((rand()%(2-0+1)+0) + rand()%10/10.0));
		tmpctrp.bw = tmpctrp.end_fre - tmpctrp.start_fre;
		// if (inds.y[i] == cwindex && (maxsum * 1.0 / nColsSample) > 0.75)
		// {
		// 	tmpctrp.bw = tmpctrp.end_fre - tmpctrp.start_fre;
		// }
		tmpctrp.Prob = 0.9;
		for (int j = 0; j < antennaNum; j++)
		{
			tmpctrp.meanAmp[j] = dataAmp[j * nRowsSample * nColsSample + inds.x[i] * nColsSample + inds.y[i]];
			//			tmpctrp.meanPhase[j] = dataPhase[j*nRowsSample * nColsSample + inds.x[i]* nColsSample + inds.y[i]];
			// tmpctrp.meanAmp[j] = integralAmp[j][inds.x[i]][inds.y[i]];
			// tmpctrp.meanPhase[j] = integralPhase[j][inds.x[i]][inds.y[i]];
			for (int n = 0; n < 7; n++)
			{
				SNR_A.at<float>(0, n) = dataAmp[j * nRowsSample * nColsSample + inds.x[i] * nColsSample + inds.y[i] - 3 + n];
			}
			SNR_B = SNR_A - mean(SNR_A)[0];
			SNR_B = SNR_B / norm(SNR_B);
			SNR_B = SNR_B / (sum(abs(SNR_B))[0] / 2);
			tmpctrp.SNR = sum(SNR_A.dot(SNR_B))[0];
		}
		int dim = 0;

		for (int m = 0; m < cwindex.size(); m++)
		{
			if (fabs(inds.y[i] - cwindex[m]) < 2)
			{
				tmpctrp.PWM = rowsum[m];
				cwpulse.push_back(tmpctrp);
				ch_ind.push_back(max_ampch(tmpctrp.meanAmp));
				dim = 1;
			}
		}
		if (dim == 1)
		{
			continue;
		}
		ctrpulse.push_back(tmpctrp);
		ch_ind.push_back(max_ampch(tmpctrp.meanAmp));
	}
	// FILE *fp = fopen("./log/pscanshortsumcorr.log", "a");
	// fprintf(fp, "ctr pulses:%d \t cw pulses:%d\n",ctrpulse.size(),cwpulse.size());
	// fclose(fp);
	int end_ch = ch_sel(ch_ind, inds.num);
	printf("end_ch = %d\n", end_ch);
	return end_ch;
}

int ctr_detect(float *dataAmp, SHORT_DATA_HEAD shortdatahead, vector<detect_pulse> &outctrpulse, vector<detect_pulse> &outcwpulse)
{
	printf("\n\t\t\t*************  CTR_DETECT BEGIN  *************\n\n");

	static int num = 0;
	struct timeval timep;
	struct tm *p;
	gettimeofday(&timep, NULL);
	p = localtime(&timep.tv_sec);
	num++;

	FILE *fp = fopen("./log/pscanshortsumcorr.log", "a");
	fprintf(fp, "\n[%d]Time : %d-%d-%d --- %d:%d:%d\n", num, p->tm_year, p->tm_mon, p->tm_mday, p->tm_hour, p->tm_min, p->tm_sec);
	fprintf(fp, "receve freq:%.2f\n", float(shortdatahead.freq / 1e6));
	fclose(fp);

	int time_sample_rate = 1;
	int freq_sample_rate = 1;

	int antennaNum = 1;
	float cfreq = float(shortdatahead.freq / 1e6);
	int nRows = shortdatahead.rows;
	int nCols = shortdatahead.cols;
	int nRowsSample = nRows / time_sample_rate;
	int nColsSample = nCols / freq_sample_rate;
	float freqResolution = float(shortdatahead.resolution / 1e6);
	float timeResolution = shortdatahead.timeRresol;
	printf("cfreq = %f\tantennaNum = %d\n", cfreq, antennaNum);
	//        printf("nRowsSample = %d\tnColsSample = %d\n",nRows ,nCols);
	//        printf("freqResolution = %f\ttimeResolution = %f\n",freqResolution ,timeResolution);
	if (cfreq == 0 || nRows == 0 || nCols == 0)
		return 0;

	// narrowband pulse detect
	vector<ctrPulses> ctrpulse;
	vector<ctrPulses> cwpulse;

	// timeBegin();
	int end_ch = narrowband_pulse_detect_v2(dataAmp, ctrpulse, cwpulse, antennaNum, nRowsSample, nColsSample, cfreq, freqResolution, timeResolution);
	// timeEnd("end narrowband_pulse_detect_v2");

	// cout << "ctrpulse.size() = " << ctrpulse.size() << endl;
	// cout << "cwpulse.size() = " << cwpulse.size() << endl;

	if (cwpulse.size() > 0)
	{
		merge_pulses(cwpulse, end_ch, outcwpulse);
		cwtaget_merge(outcwpulse);
	}

	if (ctrpulse.size() > 0)
		merge_pulses(ctrpulse, end_ch, outctrpulse);
	// cout << "\noutctrpulse.size() = " << outctrpulse.size() << endl;

	//	printf("\n");
	// FILE *fid = fopen("./log/pscanshortsumcorr.log", "a");
	//	for (int i = 0; i < outctrpulse.size(); i++)
	//	{
	//		printf("ctrpulse:(freq=%.2f,time=%.2f,pw=%.2f,bw=%.2f,SNR=%.2f,meanAmp=%.2f,meanPhase=%.2f)\n"
	//				,outctrpulse[i].fre,outctrpulse[i].time,outctrpulse[i].pw,outctrpulse[i].bw,outctrpulse[i].SNR,outctrpulse[i].meanAmp,outctrpulse[i].meanPhase);
	//		fprintf(fid, "ctrpulse:(freq=%.2f,time=%.2f,pw=%.2f,bw=%.2f,SNR=%.2f,meanAmp=%.2f,meanPhase=%.2f)\n"
	//				,outctrpulse[i].fre,outctrpulse[i].time,outctrpulse[i].pw,outctrpulse[i].bw,outctrpulse[i].SNR,outctrpulse[i].meanAmp,outctrpulse[i].meanPhase);
	//	}
	//	printf("\n");
	// fprintf(fid, "\n");
	for (int i = 0; i < outcwpulse.size(); i++)
	{
		outcwpulse[i].fre = outcwpulse[i].fre + 1;
		printf("cwtarget:(freq=%.2f,pw=%.2f,pmw=%.2f,bw=%.2f,SNR=%.2f,meanAmp=%.2f,meanPhase=%.2f)\n", outcwpulse[i].fre, outcwpulse[i].pw, outcwpulse[i].PWM, outcwpulse[i].bw, outcwpulse[i].SNR, outcwpulse[i].meanAmp, outcwpulse[i].meanPhase);
		// fprintf(fid, "cwtarget:(freq=%.2f,pw=%.2f,pwm=%.2f,bw=%.2f,SNR=%.2f,meanAmp=%.2f,meanPhase=%.2f)\n", outcwpulse[i].fre, outcwpulse[i].pw, outcwpulse[i].PWM, outcwpulse[i].bw, outcwpulse[i].SNR, outcwpulse[i].meanAmp, outcwpulse[i].meanPhase);
	}
	// printf("\n");
	// fprintf(fid, "\n");
	// fclose(fid);
	printf("\n\t\t\t *************  CTR_DETECT END  *************\n");
	return 0;
}
