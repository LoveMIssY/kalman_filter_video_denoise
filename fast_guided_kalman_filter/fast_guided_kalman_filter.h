#define _CRT_SECURE_NO_DEPRECATE     //_crt_secure_no_deprecare
#ifndef STMKF_H
#define STMKF_H


#include <iostream>
#include <omp.h>
#include <io.h>   //遍历文件夹

#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv;

class STMKF
{
public:
	STMKF(Mat firstFrame, float q, float r, int maskSize, int d, double sigmaValue);
	Mat newFrame(Mat frame, bool isBilateral, bool isFastGuide);
	int frameCount;
	double bilateralTime, blurTime, modifiedKalmanTime;


private:
	Mat xPredicted;
	Mat pPredicted;
	Mat xCorrection;
	Mat pCorrection;
	Mat K;
	Mat previous_blured;  //上一帧的均值滤波结果
	Mat current_blured;   //当前帧的均值滤波结果
	Mat floatFrame;       //观测值，即当前帧转化为浮点数
	Mat delta_Q;          //系统误差矩阵


	Mat bilateral_or_fastguided_filtered;
	Mat kalman_filterd;
	Mat R;

	float q;
	double sigmas;
	int d;
	int maskSize;
	int height, width;
};


#endif
#pragma once

