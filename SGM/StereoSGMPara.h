#pragma once
#ifndef StereoSGMPara
#define StereoSGMPara

namespace SGM
{
	struct SGMPara
	{
		//Ĭ�Ϲ��캯��
		SGMPara()
		{
			minDisparity = maxDisparity = 0;
			CensusWindowSize = 0;
			uniquenessRatio = 0;
			p1 = p2 = 0;
			disp12MaxDiff = 0;
			//mode = true;
		}

		//�������Ĺ��캯��
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
		int minDisparity; //��С�Ӳ�
		int maxDisparity; //����Ӳ�
		int CensusWindowSize; //Census���ڴ�С
		int uniquenessRatio; //Ψһ�Լ��
		int p1; //�ͷ�ϵ��p1
		int p2; //�ͷ�ϵ��p2��p2>p1
		int disp12MaxDiff; //����һ���Լ�飬����ͼ�����������Ӳ�
		//bool mode; //����ͼ����Ϊԭʼͼ��(mode=true)��������ͼ����Ϊԭʼͼ��(mode=false)
		           //Ϊ�˽�������һ���Լ��
	};
}

#endif // !StereoSGMPara
