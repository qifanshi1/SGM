#pragma once
#ifndef CostCalculate
#define CostCalculate

#include"StereoSGMPara.h"
#include<opencv2/core.hpp>
#include<vector>

namespace SGM
{
	//��ͼ�����Census�任���õ������ص�Census�任ֵ
	void CensusTransform(const cv::Mat& img, const SGM::SGMPara& para,
		std::vector<std::vector<unsigned int>>& Census);

	//��������ͼ���Censusƥ�����
	void CensusCost(const std::vector<std::vector<unsigned int>>& census_left,
		const std::vector<std::vector<unsigned int>>& census_right,
		const SGM::SGMPara& para,
		std::vector<std::vector<std::vector<ushort>>>& C);

	//��·�����۾ۺ�
	void GetDisparity(const SGM::SGMPara& para,
		const std::vector<std::vector<std::vector<ushort>>>& C,
		cv::Mat& disparity);

	float DispRefine(const std::vector<int>& x, const std::vector<int>& y);
}

#endif // !CostCalculate
