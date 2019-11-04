#pragma once
#ifndef StereoSGMPara
#define StereoSGMPara

namespace SGM
{
	struct SGMPara
	{
		//默认构造函数
		SGMPara()
		{
			minDisparity = maxDisparity = 0;
			CensusWindowSize = 0;
			uniquenessRatio = 0;
			p1 = p2 = 0;
			disp12MaxDiff = 0;
			//mode = true;
		}

		//带参数的构造函数
		SGMPara(int _minDisparity,int _maxDisparity,int _CensusWindowSize,
			int _uniquenessRatio,int _p1,int _p2,int _disp12MaxDiff)
		{
			minDisparity = _minDisparity;
			maxDisparity = _maxDisparity;
			CensusWindowSize = _CensusWindowSize;
			uniquenessRatio = _uniquenessRatio;
			p1 = _p1;
			p2 = _p2;
			disp12MaxDiff = _disp12MaxDiff;
			//mode = _mode;
		}
		int minDisparity; //最小视差
		int maxDisparity; //最大视差
		int CensusWindowSize; //Census窗口大小
		int uniquenessRatio; //唯一性检查
		int p1; //惩罚系数p1
		int p2; //惩罚系数p2，p2>p1
		int disp12MaxDiff; //左右一致性检查，左右图像的最大允许视差
		//bool mode; //以左图像作为原始图像(mode=true)，或者右图像作为原始图像(mode=false)
		           //为了进行左右一致性检查
	};
}

#endif // !StereoSGMPara
