#include "fast_guided_kalman_filter.h"
#include <video_tools/video_tools.hpp>            //�Լ�ʵ�ֵĴ�����Ƶvideo��صģ����������ļ���ȡ
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
	�������ʽ���ԭʼ��YUV�ļ���
	*/
	void runSTMKF_With_Bilateral_or_FastGuided(string fileFolder, string fileExtention, string output_filename, bool isBilateral, bool isFastGuide);

	/*
	���������������Լ�Ҫ�������۵�Y4M��Ƶ��ʽ���Ե�,���������������Ƶ��һ������Ӹ�˹����֮�����Ƶ��
	һ���� �����������˲� ����������Ƶ
	*/
	void runSTMKF_With_Y4M(string filename, string output_noise, string output_denoise, bool isBilateral, bool isFastGuide, double delta);


	double runVideoAverageMetric(string filename,string filename_processed,string metricName);

	int Height;
	int Width;
};

#pragma once
