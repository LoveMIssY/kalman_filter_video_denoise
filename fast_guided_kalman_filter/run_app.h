#include "fast_guided_kalman_filter.h"
#include <video_tools/video_tools.hpp>            //自己实现的处理视频video相关的，包括各种文件读取
#include <ssim_psnr/psnr.hpp>
#include <ssim_psnr/metric_tools.hpp>


class RunApp
{
public:
	RunApp(int h, int w)
	{
		Height = h;
		Width = w;
	}

	/*
	这个函数式针对原始的YUV文件的
	*/
	void runSTMKF_With_Bilateral_or_FastGuided(string fileFolder, string fileExtention, string output_filename, bool isBilateral, bool isFastGuide);

	/*
	这个函数是针对于自己要进行评价的Y4M视频格式而言的,它可以输出两个视频，一个是添加高斯噪声之后的视频，
	一个是 经过卡尔曼滤波 处理过后的视频
	*/
	void runSTMKF_With_Y4M(string filename, string output_noise, string output_denoise, bool isBilateral, bool isFastGuide, double delta);


	double runVideoAverageMetric(string filename,string filename_processed,string metricName);

	int Height;
	int Width;
};

#pragma once
