#include "fast_guided_kalman_filter.h"
#include <omp.h>
#include <sstream>
#include <string>
#include <fast_guided/fast_guided_filter.hpp>          //自己实现的快速导向滤波的头文件，制作成了静态库fastGF.lib文件
#include <video_tools/video_tools.hpp>                 //自己实现的处理视频video相关的，包括各种文件读取


STMKF::STMKF(Mat firstFrame, float q, float r, int maskSize, int d, double sigmaValue) 
{
	height = firstFrame.rows;
	width = firstFrame.cols;

	xPredicted = Mat(height, width, CV_32F);   //xPredicted 全部初始化为1
	xPredicted.setTo(1);

	pPredicted = Mat(height, width, CV_32F);   //pPredicted 全部初始化为1
	pPredicted.setTo(1);

	firstFrame.convertTo(xCorrection, CV_32F);   // xCorrection = firstFrame ，即第一帧，所以第一帧实际上也是进行了卡尔曼滤波的，

	pCorrection = Mat(height, width, CV_32F);  // pCorrection 全部初始化为1
	pCorrection.setTo(1);

	K = Mat(height, width, CV_32F);            // 卡尔曼增益 全部初始化为 0.5
	K.setTo(1);

	previous_blured = Mat(height, width, CV_32F); // 表示上一帧经过均值滤波之后的结果，是作为系统误差的计算依据,最开始初始化为0
	previous_blured.setTo(0);

	this->R = Mat(height, width, CV_32F);      //r 全部设置成1
	this->R.setTo(r);

	this->q = q;                //0.026
	this->maskSize = maskSize;  //5 , 均值滤波的核大小
	this->d = d;                //3 , 双边滤波的参数 d
	this->sigmas = sigmaValue;  //50, 双边滤波的第四、第五个参数，即sigmaColor和sigmaSpace

	//一些附加数据
	delta_Q = Mat(height, width, CV_32F);
	current_blured = Mat(height, width, CV_32F);    //存储当前帧均值滤波的结果
	bilateral_or_fastguided_filtered = Mat(height, width, CV_32F);//存储   双边滤波  或者是  快速导向滤波 的结果
	kalman_filterd = Mat(height, width, CV_8U);     //这是最终的KF转换之后的结果

	frameCount = 0;      //帧数
	bilateralTime = 0.0; //累积的双边滤波的时间
	blurTime = 0.0;      //累计的均值滤波的时间
	modifiedKalmanTime = 0.0; //累计的KF滤波的时间
}

/*
   这里是对单通道进行滤波，因为是对 Y U V 分别进行滤波
*/
Mat STMKF::newFrame(Mat frame, bool isBilateral, bool isFastGuide)
{
	frameCount++;  //类的属性，累计多少帧
	frame.convertTo(floatFrame, CV_32F);   //将每一帧转化成浮点数，因为只有浮点数才能进行滤波运算，会作为当前帧的观测值
	if (isBilateral)
	{
		blur(floatFrame, current_blured, Size(5, 5));
		bilateralFilter(floatFrame, bilateral_or_fastguided_filtered, d, sigmas, sigmas); //双边滤波，结果是bilateral_filtered
	}
	else if (isFastGuide)
	{
		int r = 7;
		int s = 4;
		double eps = 0.01 * 255 * 255;
		cv::ximgproc::fastGuidedFilter(floatFrame, floatFrame, current_blured, r, eps, 4);
		bilateral_or_fastguided_filtered = current_blured.clone();
	}


	delta_Q = current_blured - previous_blured; //为了构造系统的误差Q
	R = 1 + (R) / (1 + R);      //为了构造观测误差R

	//KF的第一步
	xPredicted = xCorrection.clone();

	//第二步
	pPredicted = pCorrection + q * delta_Q.mul(delta_Q);

	//第三步：卡尔曼增益
	K = pPredicted / (pPredicted + R);

	//第四步
	xCorrection = (1 - K).mul(xPredicted + K.mul(floatFrame - xPredicted)) + K.mul(bilateral_or_fastguided_filtered);

	//第五步
	pCorrection = pPredicted.mul(1 - K);

	previous_blured = current_blured.clone();
	xCorrection.convertTo(kalman_filterd, CV_8U);

	return kalman_filterd;
}





/*
单帧是的降噪，不考虑时间域上面的情况
*/
//void runSpatialGuidedFilter(string fileFolder, string fileExtention, string output_filename)
//{
//	int r = 7;
//	double eps = 0.001 * 255 * 255;  //该取值至关重要
//	int s = 6;
//
//	clock_t startTime, endTime;
//	double time_mean = 0.0, times_, times2;
//
//	int frameCount = 0;
//	Mat filtered;
//	Mat pre_img;
//
//	vector<string> v_filenames; //遍历所有的YUV图片，得到所有的图片名称
//	v_filenames = getFiles(fileFolder, fileExtention);
//
//	try {
//		//写入视频相关的操作
//		VideoWriter writer;
//		Size size = Size(Width, Height * 2);
//		int fourcc = writer.fourcc('X', 'V', 'I', 'D');
//		writer.open(output_filename, fourcc, 25.0, size, true);
//
//		vector<Mat> yuv;
//		yuv = readYUV(fileFolder, v_filenames[frameCount]);  //读取第一帧,返回的是一个vector
//		Mat yuv_img;
//
//
//
//		while (1)
//		{
//			yuv_img = Y_V_U_2_YVU(yuv[0], yuv[1], yuv[2]);     // 将Y 、U 、V转化成 YUV
//			pre_img = YUV_2_BGR(yuv_img);                    //将前一帧转化成彩色，这是为了方便对比效果
//
//			//加速版的快四导向滤波  vs  没有加速版的快四导向滤波
//			startTime = clock();
//			cv::ximgproc::fastGuidedFilter(yuv[0], yuv[0], filtered, r, eps, s);  //单独的空间域滤波,得到仅仅是Y	
//			//filtered = fastGuidedFilterWithNonSpeed(yuv[0], yuv[0], r, eps, s, cv::INTER_LINEAR);  //单独的空间域滤波,得到仅仅是Y	
//
//
//			endTime = clock();
//
//			//将其中的某几针保存下来，看一下效果啊
//
//
//			times_ = (double)(endTime - startTime) / CLOCKS_PER_SEC;
//
//			Mat yuv_filtered = Y_V_U_2_YVU(filtered, yuv[1], yuv[2]);
//			Mat bgr_img = YUV_2_BGR(yuv_filtered); //将滤波之后的YUV图像转化成BGR彩色图像	Mat bgr_img2 = YUV_2_BGR(yuv_filtered2); //将滤波之后的YUV图像转化成BGR彩色图像	
//
//			cv::putText(bgr_img, std::to_string(times_), cv::Point(50, 50), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 255, 230), 1, 8, false); //将每一帧的时间写上去
//			std::cout << "The single frame time is: " << times_ << "s" << endl;	//单帧的处理时间			
//			time_mean += times_;  //求平均时间
//
//			cv::Mat combine_img;
//			vconcat(pre_img, bgr_img, combine_img); //将前面两张图片合并
//			//rotate(combine_img2, combine_img2, ROTATE_90_CLOCKWISE);  //手机视频需要旋转一下
//
//			//显示处理之后的图像以及视频保存
//			cv::imshow("preFilterd_Filterd", combine_img);
//			cv::waitKey(1);
//			writer.write(combine_img);
//
//			frameCount++;
//			yuv = readYUV(fileFolder, v_filenames[frameCount]);  //读取下一帧
//
//			if (yuv_img.empty())
//			{
//				std::cout << endl << "Video ended!" << endl;
//				break;
//			}
//		}
//		std::cout << "The run mean time is: " << (double)time_mean / frameCount << "s" << endl;  //最终的平均时间
//		getchar();
//	}
//	catch (const Exception& ex)
//	{
//		std::cout << "Error: " << ex.what() << endl;
//	}
//}

