#include "fast_guided_kalman_filter.h"
#include <omp.h>
#include <sstream>
#include <string>
#include <fast_guided/fast_guided_filter.hpp>          //�Լ�ʵ�ֵĿ��ٵ����˲���ͷ�ļ����������˾�̬��fastGF.lib�ļ�
#include <video_tools/video_tools.hpp>                 //�Լ�ʵ�ֵĴ�����Ƶvideo��صģ����������ļ���ȡ


STMKF::STMKF(Mat firstFrame, float q, float r, int maskSize, int d, double sigmaValue) 
{
	height = firstFrame.rows;
	width = firstFrame.cols;

	xPredicted = Mat(height, width, CV_32F);   //xPredicted ȫ����ʼ��Ϊ1
	xPredicted.setTo(1);

	pPredicted = Mat(height, width, CV_32F);   //pPredicted ȫ����ʼ��Ϊ1
	pPredicted.setTo(1);

	firstFrame.convertTo(xCorrection, CV_32F);   // xCorrection = firstFrame ������һ֡�����Ե�һ֡ʵ����Ҳ�ǽ����˿������˲��ģ�

	pCorrection = Mat(height, width, CV_32F);  // pCorrection ȫ����ʼ��Ϊ1
	pCorrection.setTo(1);

	K = Mat(height, width, CV_32F);            // ���������� ȫ����ʼ��Ϊ 0.5
	K.setTo(1);

	previous_blured = Mat(height, width, CV_32F); // ��ʾ��һ֡������ֵ�˲�֮��Ľ��������Ϊϵͳ���ļ�������,�ʼ��ʼ��Ϊ0
	previous_blured.setTo(0);

	this->R = Mat(height, width, CV_32F);      //r ȫ�����ó�1
	this->R.setTo(r);

	this->q = q;                //0.026
	this->maskSize = maskSize;  //5 , ��ֵ�˲��ĺ˴�С
	this->d = d;                //3 , ˫���˲��Ĳ��� d
	this->sigmas = sigmaValue;  //50, ˫���˲��ĵ��ġ��������������sigmaColor��sigmaSpace

	//һЩ��������
	delta_Q = Mat(height, width, CV_32F);
	current_blured = Mat(height, width, CV_32F);    //�洢��ǰ֡��ֵ�˲��Ľ��
	bilateral_or_fastguided_filtered = Mat(height, width, CV_32F);//�洢   ˫���˲�  ������  ���ٵ����˲� �Ľ��
	kalman_filterd = Mat(height, width, CV_8U);     //�������յ�KFת��֮��Ľ��

	frameCount = 0;      //֡��
	bilateralTime = 0.0; //�ۻ���˫���˲���ʱ��
	blurTime = 0.0;      //�ۼƵľ�ֵ�˲���ʱ��
	modifiedKalmanTime = 0.0; //�ۼƵ�KF�˲���ʱ��
}

/*
   �����ǶԵ�ͨ�������˲�����Ϊ�Ƕ� Y U V �ֱ�����˲�
*/
Mat STMKF::newFrame(Mat frame, bool isBilateral, bool isFastGuide)
{
	frameCount++;  //������ԣ��ۼƶ���֡
	frame.convertTo(floatFrame, CV_32F);   //��ÿһ֡ת���ɸ���������Ϊֻ�и��������ܽ����˲����㣬����Ϊ��ǰ֡�Ĺ۲�ֵ
	if (isBilateral)
	{
		blur(floatFrame, current_blured, Size(5, 5));
		bilateralFilter(floatFrame, bilateral_or_fastguided_filtered, d, sigmas, sigmas); //˫���˲��������bilateral_filtered
	}
	else if (isFastGuide)
	{
		int r = 7;
		int s = 4;
		double eps = 0.01 * 255 * 255;
		cv::ximgproc::fastGuidedFilter(floatFrame, floatFrame, current_blured, r, eps, 4);
		bilateral_or_fastguided_filtered = current_blured.clone();
	}


	delta_Q = current_blured - previous_blured; //Ϊ�˹���ϵͳ�����Q
	R = 1 + (R) / (1 + R);      //Ϊ�˹���۲����R

	//KF�ĵ�һ��
	xPredicted = xCorrection.clone();

	//�ڶ���
	pPredicted = pCorrection + q * delta_Q.mul(delta_Q);

	//������������������
	K = pPredicted / (pPredicted + R);

	//���Ĳ�
	xCorrection = (1 - K).mul(xPredicted + K.mul(floatFrame - xPredicted)) + K.mul(bilateral_or_fastguided_filtered);

	//���岽
	pCorrection = pPredicted.mul(1 - K);

	previous_blured = current_blured.clone();
	xCorrection.convertTo(kalman_filterd, CV_8U);

	return kalman_filterd;
}





/*
��֡�ǵĽ��룬������ʱ������������
*/
//void runSpatialGuidedFilter(string fileFolder, string fileExtention, string output_filename)
//{
//	int r = 7;
//	double eps = 0.001 * 255 * 255;  //��ȡֵ������Ҫ
//	int s = 6;
//
//	clock_t startTime, endTime;
//	double time_mean = 0.0, times_, times2;
//
//	int frameCount = 0;
//	Mat filtered;
//	Mat pre_img;
//
//	vector<string> v_filenames; //�������е�YUVͼƬ���õ����е�ͼƬ����
//	v_filenames = getFiles(fileFolder, fileExtention);
//
//	try {
//		//д����Ƶ��صĲ���
//		VideoWriter writer;
//		Size size = Size(Width, Height * 2);
//		int fourcc = writer.fourcc('X', 'V', 'I', 'D');
//		writer.open(output_filename, fourcc, 25.0, size, true);
//
//		vector<Mat> yuv;
//		yuv = readYUV(fileFolder, v_filenames[frameCount]);  //��ȡ��һ֡,���ص���һ��vector
//		Mat yuv_img;
//
//
//
//		while (1)
//		{
//			yuv_img = Y_V_U_2_YVU(yuv[0], yuv[1], yuv[2]);     // ��Y ��U ��Vת���� YUV
//			pre_img = YUV_2_BGR(yuv_img);                    //��ǰһ֡ת���ɲ�ɫ������Ϊ�˷���Ա�Ч��
//
//			//���ٰ�Ŀ��ĵ����˲�  vs  û�м��ٰ�Ŀ��ĵ����˲�
//			startTime = clock();
//			cv::ximgproc::fastGuidedFilter(yuv[0], yuv[0], filtered, r, eps, s);  //�����Ŀռ����˲�,�õ�������Y	
//			//filtered = fastGuidedFilterWithNonSpeed(yuv[0], yuv[0], r, eps, s, cv::INTER_LINEAR);  //�����Ŀռ����˲�,�õ�������Y	
//
//
//			endTime = clock();
//
//			//�����е�ĳ���뱣����������һ��Ч����
//
//
//			times_ = (double)(endTime - startTime) / CLOCKS_PER_SEC;
//
//			Mat yuv_filtered = Y_V_U_2_YVU(filtered, yuv[1], yuv[2]);
//			Mat bgr_img = YUV_2_BGR(yuv_filtered); //���˲�֮���YUVͼ��ת����BGR��ɫͼ��	Mat bgr_img2 = YUV_2_BGR(yuv_filtered2); //���˲�֮���YUVͼ��ת����BGR��ɫͼ��	
//
//			cv::putText(bgr_img, std::to_string(times_), cv::Point(50, 50), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 255, 230), 1, 8, false); //��ÿһ֡��ʱ��д��ȥ
//			std::cout << "The single frame time is: " << times_ << "s" << endl;	//��֡�Ĵ���ʱ��			
//			time_mean += times_;  //��ƽ��ʱ��
//
//			cv::Mat combine_img;
//			vconcat(pre_img, bgr_img, combine_img); //��ǰ������ͼƬ�ϲ�
//			//rotate(combine_img2, combine_img2, ROTATE_90_CLOCKWISE);  //�ֻ���Ƶ��Ҫ��תһ��
//
//			//��ʾ����֮���ͼ���Լ���Ƶ����
//			cv::imshow("preFilterd_Filterd", combine_img);
//			cv::waitKey(1);
//			writer.write(combine_img);
//
//			frameCount++;
//			yuv = readYUV(fileFolder, v_filenames[frameCount]);  //��ȡ��һ֡
//
//			if (yuv_img.empty())
//			{
//				std::cout << endl << "Video ended!" << endl;
//				break;
//			}
//		}
//		std::cout << "The run mean time is: " << (double)time_mean / frameCount << "s" << endl;  //���յ�ƽ��ʱ��
//		getchar();
//	}
//	catch (const Exception& ex)
//	{
//		std::cout << "Error: " << ex.what() << endl;
//	}
//}

