#include "run_app.h"


/*
运行，两种类型的滤波，根据传入的参数自行选择
“双边+卡尔曼滤波”
“快速导向+卡尔曼滤波”
params:
    fileFolder:      包含所有的YUV文件的文件夹
	fileExtention:   文件的拓展名称 .yuv
	output_filename: 输出的降噪的视频名称，如 output.avi
*/
void RunApp::runSTMKF_With_Bilateral_or_FastGuided(string fileFolder, string fileExtention, string output_filename, bool isBilateral, bool isFastGuide)
{
	VideoTools videotools = VideoTools(Height, Width);  //创建VideoTools对象
	Mat filtered_v, filtered_u;
	float q = 0.005f;
	float r = 1;
	int maskSize = 5;
	int bilateralKernelSize = 5;  // 3;
	float bilateralSigmas = 50;   // 50.0;

	clock_t startTime, endTime;
	double time_mean = 0.0, times_;

	int frameCount = 0;
	Mat filtered;
	Mat blur_filtered, bilateral_filtered;
	Mat pre_img;

	vector<string> v_filenames; //遍历所有的YUV图片，得到所有的图片名称
	v_filenames = videotools.getFiles(fileFolder, fileExtention);

	try {
		//写入视频相关的操作
		VideoWriter writer;
		Size size = Size(videotools.Width, videotools.Height * 2);
		int fourcc = writer.fourcc('X', 'V', 'I', 'D');
		writer.open(output_filename, fourcc, 25.0, size, true);

		vector<Mat> yuv_img = videotools.readYUV(fileFolder, v_filenames[frameCount]);
		//构建stmkf对象，得到一些初始化的值,分别对U,V通道进行滤波，不用考虑Y通道
		STMKF stmkf_v = STMKF(yuv_img[1], q, r, maskSize, bilateralKernelSize, bilateralSigmas);
		STMKF stmkf_u = STMKF(yuv_img[2], q, r, maskSize, bilateralKernelSize, bilateralSigmas);

		while (1)
		{
			Mat yuv_whole_before = videotools.Y_V_U_2_YVU(yuv_img[0], yuv_img[1], yuv_img[2]);
			pre_img = videotools.YUV_2_BGR(yuv_whole_before,cv::COLOR_YUV2BGRA_NV21);   //将前一帧转化成彩色，这是为了方便对比效果

			startTime = clock();
			filtered_v = stmkf_v.newFrame(yuv_img[1], isBilateral, isFastGuide);  //进入卡尔曼滤波函数
			filtered_u = stmkf_u.newFrame(yuv_img[2], isBilateral, isFastGuide);  //进入卡尔曼滤波函数
			endTime = clock();

			Mat yuv_whole_after = videotools.Y_V_U_2_YVU(yuv_img[0], filtered_v, filtered_u);
			Mat bgr_img = videotools.YUV_2_BGR(yuv_whole_after, cv::COLOR_YUV2BGRA_NV21); //将滤波之后的YUV图像转化成BGR彩色图像			
			times_ = (double)(endTime - startTime) / CLOCKS_PER_SEC;
			std::cout << "The single frame time is: " << times_ << "s" << endl;	//单帧的处理时间			
			time_mean += times_;  //求平均时间

			cv::Mat combine_img;
			cv::vconcat(pre_img, bgr_img, combine_img); //将前面两张图片合并
			//rotate(combine_img, combine_img, ROTATE_90_CLOCKWISE);  //手机视频需要旋转一下

			//显示处理之后的图像以及视频保存
			cv::imshow("preFilterd_Filterd", combine_img);
			cv::waitKey(1);
			writer.write(combine_img);

			frameCount++;
			
			yuv_img = videotools.readYUV(fileFolder, v_filenames[frameCount]);    //读取下一帧

			if (yuv_img.empty())
			{
				std::cout << endl << "Video ended!" << endl;
				break;
			}
		}
		std::cout << "The run mean time is: " << (double)time_mean / frameCount << "s" << endl;  //最终的平均时间
	}
	catch (const Exception & ex)
	{
		std::cout << "Error: " << ex.what() << endl;
	}
}

/*
同时输出噪声视频、处理之后的视频
*/
void RunApp::runSTMKF_With_Y4M(string filename, string output_noise, string output_denoise, bool isBilateral, bool isFastGuide, double delta)
{
	MetricTools metrictools = MetricTools(filename);  //创建一个对象
	float q = 0.005;
	float r = 1;
	int bilateralKernelSize = 5;  // 3;
	float bilateralSigmas = 50;   // 50.0;

	clock_t startTime, endTime;
	double time_mean = 0.0, times_;

	int frameCount = 0;
	Mat frame, filtered;
	Mat blur_filtered, bilateral_filtered;
	Mat gaussianframe;

	vector<Mat> filtered_vector;
	Mat filtered_1, filtered_2, filtered_3;
	vector<Mat> channels;
	vector<Mat> gaussian_channels;

	double single_frame_psnr, total_frame_psnr = 0, mean_psnr;

	try {
		VideoCapture videoSource;
		if (!videoSource.open(filename))
		{
			cout << "Error on load video..." << endl;
		}
		//写入  噪声视频  相关的操作
		VideoWriter writer1;
		Size size1 = Size(videoSource.get(CAP_PROP_FRAME_WIDTH), videoSource.get(CAP_PROP_FRAME_HEIGHT));
		int fourcc1 = writer1.fourcc('X', 'V', 'I', 'D');
		writer1.open(output_noise, fourcc1, 25.0, size1, true);

		//写入  降噪之后的的视频  相关的操作
		VideoWriter writer2;
		Size size2 = Size(videoSource.get(CAP_PROP_FRAME_WIDTH), videoSource.get(CAP_PROP_FRAME_HEIGHT));
		int fourcc2 = writer2.fourcc('X', 'V', 'I', 'D');
		writer2.open(output_denoise, fourcc2, 25.0, size2, true);

		videoSource >> frame;   //读取一帧图像

		//构建stmkf对象，得到一些初始化的值
		cv::split(frame, channels);
		STMKF stmkf_1 = STMKF(channels[0], q, r, 5, bilateralKernelSize, bilateralSigmas);
		STMKF stmkf_2 = STMKF(channels[1], q, r, 5, bilateralKernelSize, bilateralSigmas);
		STMKF stmkf_3 = STMKF(channels[2], q, r, 5, bilateralKernelSize, bilateralSigmas);

		while (1) 
		{
			startTime = clock();
			gaussianframe = metrictools.addGaussianNoise(frame, 0, delta); //给每一帧添加高斯噪声
			writer1.write(gaussianframe);
			cv::split(gaussianframe, gaussian_channels);

			filtered_1 = stmkf_1.newFrame(gaussian_channels[0], isBilateral,isFastGuide);  //进入滤波函数 (均值-双边-卡尔曼)
			filtered_vector.push_back(filtered_1);
			filtered_2 = stmkf_2.newFrame(gaussian_channels[1], isBilateral, isFastGuide);
			filtered_vector.push_back(filtered_2);
			filtered_3 = stmkf_3.newFrame(gaussian_channels[2], isBilateral, isFastGuide);
			filtered_vector.push_back(filtered_3);
			cv::merge(filtered_vector, filtered);
			
			endTime = clock();
			times_ = (double)(endTime - startTime) / CLOCKS_PER_SEC;
			cout << "The single frame time is: " << times_ << "s" << endl;	//单帧的处理时间
	
			time_mean += times_;  //求平均时间


			cv::Mat combine_img;
			vconcat(gaussianframe, filtered, combine_img); //将前面两张图片合并
			//rotate(filtered, filtered, ROTATE_90_CLOCKWISE);  //手机视频需要旋转一下

			//显示处理之后的图像以及视频保存
			cv::namedWindow("combine_img", cv::WINDOW_NORMAL);
			imshow("combine_img", combine_img);
			waitKey(1);

			writer2.write(filtered);

			videoSource >> frame;  //读取下一帧			
			frameCount++;

			if (frame.empty())
			{
				cout << endl << "Video ended!" << endl;
				break;
			}
			cv::split(frame, channels);
			filtered.release();
			filtered_vector.clear();  //每次循环之后需要释放，否则会每次累加一个通道，filtered通道数会依次为3、6、9、12.。。。
			
		}
		cout << "The run mean time is: " << (double)time_mean / frameCount << "s" << endl;  //最终的平均时间

	}
	catch (const Exception& ex) 
	{
		cout << "Error: " << ex.what() << endl;
	}
}


/*
获取处理之后的视频的平均评价指标
*/
double RunApp::runVideoAverageMetric(string filename, string filename_processed, string metricName)
{
	MetricTools metricTools = MetricTools(filename);
	double metric;
	if (metricName == "psnr")
	{
		metric = metricTools.getAveragePSNR(filename, filename_processed);
		return metric;
	}
	else if (metricName == "ssim")
	{
		metric = metricTools.getAverageSSIM(filename, filename_processed);
		return metric;
	}
	else
	{
		return -1;
	}
}