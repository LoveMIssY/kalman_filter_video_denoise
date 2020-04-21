#include "run_app.h"


/*
���У��������͵��˲������ݴ���Ĳ�������ѡ��
��˫��+�������˲���
�����ٵ���+�������˲���
params:
    fileFolder:      �������е�YUV�ļ����ļ���
	fileExtention:   �ļ�����չ���� .yuv
	output_filename: ����Ľ������Ƶ���ƣ��� output.avi
*/
void RunApp::runSTMKF_With_Bilateral_or_FastGuided(string fileFolder, string fileExtention, string output_filename, bool isBilateral, bool isFastGuide)
{
	VideoTools videotools = VideoTools(Height, Width);  //����VideoTools����
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

	vector<string> v_filenames; //�������е�YUVͼƬ���õ����е�ͼƬ����
	v_filenames = videotools.getFiles(fileFolder, fileExtention);

	try {
		//д����Ƶ��صĲ���
		VideoWriter writer;
		Size size = Size(videotools.Width, videotools.Height * 2);
		int fourcc = writer.fourcc('X', 'V', 'I', 'D');
		writer.open(output_filename, fourcc, 25.0, size, true);

		vector<Mat> yuv_img = videotools.readYUV(fileFolder, v_filenames[frameCount]);
		//����stmkf���󣬵õ�һЩ��ʼ����ֵ,�ֱ��U,Vͨ�������˲������ÿ���Yͨ��
		STMKF stmkf_v = STMKF(yuv_img[1], q, r, maskSize, bilateralKernelSize, bilateralSigmas);
		STMKF stmkf_u = STMKF(yuv_img[2], q, r, maskSize, bilateralKernelSize, bilateralSigmas);

		while (1)
		{
			Mat yuv_whole_before = videotools.Y_V_U_2_YVU(yuv_img[0], yuv_img[1], yuv_img[2]);
			pre_img = videotools.YUV_2_BGR(yuv_whole_before,cv::COLOR_YUV2BGRA_NV21);   //��ǰһ֡ת���ɲ�ɫ������Ϊ�˷���Ա�Ч��

			startTime = clock();
			filtered_v = stmkf_v.newFrame(yuv_img[1], isBilateral, isFastGuide);  //���뿨�����˲�����
			filtered_u = stmkf_u.newFrame(yuv_img[2], isBilateral, isFastGuide);  //���뿨�����˲�����
			endTime = clock();

			Mat yuv_whole_after = videotools.Y_V_U_2_YVU(yuv_img[0], filtered_v, filtered_u);
			Mat bgr_img = videotools.YUV_2_BGR(yuv_whole_after, cv::COLOR_YUV2BGRA_NV21); //���˲�֮���YUVͼ��ת����BGR��ɫͼ��			
			times_ = (double)(endTime - startTime) / CLOCKS_PER_SEC;
			std::cout << "The single frame time is: " << times_ << "s" << endl;	//��֡�Ĵ���ʱ��			
			time_mean += times_;  //��ƽ��ʱ��

			cv::Mat combine_img;
			cv::vconcat(pre_img, bgr_img, combine_img); //��ǰ������ͼƬ�ϲ�
			//rotate(combine_img, combine_img, ROTATE_90_CLOCKWISE);  //�ֻ���Ƶ��Ҫ��תһ��

			//��ʾ����֮���ͼ���Լ���Ƶ����
			cv::imshow("preFilterd_Filterd", combine_img);
			cv::waitKey(1);
			writer.write(combine_img);

			frameCount++;
			
			yuv_img = videotools.readYUV(fileFolder, v_filenames[frameCount]);    //��ȡ��һ֡

			if (yuv_img.empty())
			{
				std::cout << endl << "Video ended!" << endl;
				break;
			}
		}
		std::cout << "The run mean time is: " << (double)time_mean / frameCount << "s" << endl;  //���յ�ƽ��ʱ��
	}
	catch (const Exception & ex)
	{
		std::cout << "Error: " << ex.what() << endl;
	}
}

/*
ͬʱ���������Ƶ������֮�����Ƶ
*/
void RunApp::runSTMKF_With_Y4M(string filename, string output_noise, string output_denoise, bool isBilateral, bool isFastGuide, double delta)
{
	MetricTools metrictools = MetricTools(filename);  //����һ������
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
		//д��  ������Ƶ  ��صĲ���
		VideoWriter writer1;
		Size size1 = Size(videoSource.get(CAP_PROP_FRAME_WIDTH), videoSource.get(CAP_PROP_FRAME_HEIGHT));
		int fourcc1 = writer1.fourcc('X', 'V', 'I', 'D');
		writer1.open(output_noise, fourcc1, 25.0, size1, true);

		//д��  ����֮��ĵ���Ƶ  ��صĲ���
		VideoWriter writer2;
		Size size2 = Size(videoSource.get(CAP_PROP_FRAME_WIDTH), videoSource.get(CAP_PROP_FRAME_HEIGHT));
		int fourcc2 = writer2.fourcc('X', 'V', 'I', 'D');
		writer2.open(output_denoise, fourcc2, 25.0, size2, true);

		videoSource >> frame;   //��ȡһ֡ͼ��

		//����stmkf���󣬵õ�һЩ��ʼ����ֵ
		cv::split(frame, channels);
		STMKF stmkf_1 = STMKF(channels[0], q, r, 5, bilateralKernelSize, bilateralSigmas);
		STMKF stmkf_2 = STMKF(channels[1], q, r, 5, bilateralKernelSize, bilateralSigmas);
		STMKF stmkf_3 = STMKF(channels[2], q, r, 5, bilateralKernelSize, bilateralSigmas);

		while (1) 
		{
			startTime = clock();
			gaussianframe = metrictools.addGaussianNoise(frame, 0, delta); //��ÿһ֡��Ӹ�˹����
			writer1.write(gaussianframe);
			cv::split(gaussianframe, gaussian_channels);

			filtered_1 = stmkf_1.newFrame(gaussian_channels[0], isBilateral,isFastGuide);  //�����˲����� (��ֵ-˫��-������)
			filtered_vector.push_back(filtered_1);
			filtered_2 = stmkf_2.newFrame(gaussian_channels[1], isBilateral, isFastGuide);
			filtered_vector.push_back(filtered_2);
			filtered_3 = stmkf_3.newFrame(gaussian_channels[2], isBilateral, isFastGuide);
			filtered_vector.push_back(filtered_3);
			cv::merge(filtered_vector, filtered);
			
			endTime = clock();
			times_ = (double)(endTime - startTime) / CLOCKS_PER_SEC;
			cout << "The single frame time is: " << times_ << "s" << endl;	//��֡�Ĵ���ʱ��
	
			time_mean += times_;  //��ƽ��ʱ��


			cv::Mat combine_img;
			vconcat(gaussianframe, filtered, combine_img); //��ǰ������ͼƬ�ϲ�
			//rotate(filtered, filtered, ROTATE_90_CLOCKWISE);  //�ֻ���Ƶ��Ҫ��תһ��

			//��ʾ����֮���ͼ���Լ���Ƶ����
			cv::namedWindow("combine_img", cv::WINDOW_NORMAL);
			imshow("combine_img", combine_img);
			waitKey(1);

			writer2.write(filtered);

			videoSource >> frame;  //��ȡ��һ֡			
			frameCount++;

			if (frame.empty())
			{
				cout << endl << "Video ended!" << endl;
				break;
			}
			cv::split(frame, channels);
			filtered.release();
			filtered_vector.clear();  //ÿ��ѭ��֮����Ҫ�ͷţ������ÿ���ۼ�һ��ͨ����filteredͨ����������Ϊ3��6��9��12.������
			
		}
		cout << "The run mean time is: " << (double)time_mean / frameCount << "s" << endl;  //���յ�ƽ��ʱ��

	}
	catch (const Exception& ex) 
	{
		cout << "Error: " << ex.what() << endl;
	}
}


/*
��ȡ����֮�����Ƶ��ƽ������ָ��
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