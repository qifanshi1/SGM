#pragma once
#ifndef CostCalculate
#define CostCalculate

#include"StereoSGMPara.h"
#include<opencv2/core.hpp>
#include<vector>

namespace SGM
{
	//对图像进行Census变换，得到各像素的Census变换值
	void CensusTransform(const cv::Mat& img, const SGM::SGMPara& para,
		std::vector<std::vector<unsigned int>>& Census);

	//计算两幅图像的Census匹配代价
	void CensusCost(const std::vector<std::vector<unsigned int>>& census_left,
		const std::vector<std::vector<unsigned int>>& census_right,
		const SGM::SGMPara& para,
		std::vector<std::vector<std::vector<ushort>>>& C);

	//八路径代价聚合
	void GetDisparity(const SGM::SGMPara& para,
		const std::vector<std::vector<std::vector<ushort>>>& C,
		cv::Mat& disparity);

	float DispRefine(const std::vector<int>& x, const std::vector<int>& y);
}

#endif // !CostCalculate
