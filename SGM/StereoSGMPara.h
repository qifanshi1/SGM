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
			speckleWindowSize = 0;
			speckleRange = 0;
		}

		//带参数的构造函数
		SGMPara(int _minDisparity,int _maxDisparity,int _CensusWindowSize,
			int _uniquenessRatio,int _p1,int _p2,int _disp12MaxDiff,
			int _speckleWindowSize,int _speckleRange)
		{
			minDisparity = _minDisparity;
			maxDisparity = _maxDisparity;
			CensusWindowSize = _CensusWindowSize;
			uniquenessRatio = _uniquenessRatio;
			p1 = _p1;
			p2 = _p2;
			disp12MaxDiff = _disp12MaxDiff;
			speckleWindowSize = _speckleWindowSize;
			speckleRange = _speckleRange;
			
		}
		int minDisparity; //最小视差
		int maxDisparity; //最大视差
		int CensusWindowSize; //Census窗口大小
		int uniquenessRatio; //唯一性检查
		int p1; //惩罚系数p1
		int p2; //惩罚系数p2，p2>p1
		int disp12MaxDiff; //左右一致性检查，左右图像的最大允许视差
		int speckleWindowSize;//连通区域的阈值，连通区域小于该值即认为是误匹配点
		int speckleRange;//视差相差该像素的点位于同一连通区域
	};
}

#endif // !StereoSGMPara
