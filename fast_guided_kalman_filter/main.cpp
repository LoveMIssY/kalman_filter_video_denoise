#include <iostream>
#include "run_app.h"


using namespace cv;
using namespace std;

int main(int argc, char* argv[]) {

	bool isBilateral = false;  //输出名称中包含gaussianbilateral
	bool isFastGuide = false;  //输出名称中包含fastguide

	string output_filename = "YUV_Output/arc6_denoise_spatial_fastguided.avi";
	string fileFolder = "F:\\low_light_video_enhancement\\YUV\\arc6\\";
	string fileExtention = "*.yuv";

	//构建 run APP 的对象
	RunApp runapp = RunApp(1088, 1920);
	//runapp.runSTMKF_With_Bilateral_or_FastGuided(fileFolder, fileExtention, output_filename, false, true);
	
	
	//string source = "evaluate_videos/source_videos/football_cif.y4m";
	//string noise = "evaluate_videos/noise_videos/footbal_noise_10.avi";
	//string denoise = "evaluate_videos/process_videos/footbal_denoise_10_bilateral.avi";
	//runapp.runSTMKF_With_Y4M(source, noise, denoise, false, true, 10);

	

	double metric;
	string filename = "F:/low_light_video_enhancement/STMKF_TEST/fast_guided_kalman_filter_sourceYUV_2017_v1.0/fast_guided_kalman_filter/fast_guided_kalman_filter/evaluate_videos/source_videos/football_cif.y4m";
	string filename_processed = "F:/low_light_video_enhancement/STMKF_TEST/fast_guided_kalman_filter_sourceYUV_2017_v1.0/fast_guided_kalman_filter/fast_guided_kalman_filter/evaluate_videos/process_videos/ghost/footbal_denoise_10_bilateral.avi";
	metric = runapp.runVideoAverageMetric(filename, filename_processed, "ssim");
    std::cout << "平均峰值信噪比为 : " << metric << std::endl;

	getchar();
	return 0;
}
