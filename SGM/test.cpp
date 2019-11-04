#include<string>
#include<iostream>
#include<vector>
#include<opencv2/core.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<chrono>
#include"StereoSGMPara.h"
#include"CostCalculate.h"

int main(int argc,char** argv)
{
	if (argc != 3)
	{
		std::cerr << "Usage: " << argv[0] << " <leftImage><rightImage>" << std::endl;
		return -1;
	}

	std::string file1 = argv[1];
	std::string file2 = argv[2];
	cv::Mat left_image = cv::imread(file1, CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat right_image = cv::imread(file2, CV_LOAD_IMAGE_GRAYSCALE);

	int minD = 0;
	int maxD = 64;
	int CensusWindowSize = 5;
	int uniquenessRatio = 10;
	int p1 = 3;
	int p2 = 20;
	int disp12MaxDiff = 2;
	SGM::SGMPara para(minD, maxD, CensusWindowSize,	uniquenessRatio, p1, p2, disp12MaxDiff);
	
	//计时
	auto t_begin = std::chrono::system_clock::now();

	int height = left_image.rows, width = left_image.cols;
	int D = maxD - minD;
	int blur_radius = 3;
	//Initial Smoothing
	cv::GaussianBlur(left_image, left_image, cv::Size(blur_radius, blur_radius), 0, 0);
	cv::GaussianBlur(right_image, right_image, cv::Size(blur_radius, blur_radius),0,0);
	//Census Tansform
	std::vector<std::vector<unsigned int>> census_left(height, std::vector<unsigned int>(width, 0));
	std::vector<std::vector<unsigned int>> census_right(height, std::vector<unsigned int>(width, 0));
	SGM::CensusTransform(left_image, para, census_left);
	SGM::CensusTransform(right_image, para, census_right);
	//Census Cost
	std::vector<std::vector<std::vector<ushort>>> C(height, std::vector<std::vector<ushort>>(width, std::vector<ushort>(D, 0)));
	SGM::CensusCost(census_left, census_right, para, C);

	cv::Mat disparity = cv::Mat::zeros(left_image.size(), CV_32FC1);
	SGM::GetDisparity(para, C, disparity);
	//计时
	auto t_end = std::chrono::system_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_begin);
	std::cout << "The SGM algorithm spent " << duration.count() << " milliseconds." << std::endl;
	cv::Mat disparity_output(disparity.size(), CV_8UC1);
	cv::normalize(disparity, disparity_output, 0, 255, cv::NORM_MINMAX);
	cv::imwrite("disparity_output.png", disparity_output);
	return 0;
}