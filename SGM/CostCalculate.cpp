#include"CostCalculate.h"
#include<opencv2/core.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include<stack>

void SGM::CensusTransform(const cv::Mat& img, const SGM::SGMPara& para,
	std::vector<std::vector<unsigned int>>& Census)
{
	int height = img.rows;
	int width = img.cols;
	int window_size = para.CensusWindowSize;
	int half_win_size = window_size / 2;

	//cv::Mat Census_image = cv::Mat::zeros(img.size(), CV_8UC1);
	for (int row = half_win_size; row < height-half_win_size; row++)
	{
		for (int col = half_win_size; col < width - half_win_size; col++)
		{
			uchar value = img.at<uchar>(row, col);
			unsigned int record = 0;
			for (int y = row - half_win_size; y <= row + half_win_size; y++)
			{
				for (int x = col - half_win_size; x <= col + half_win_size; x++)
				{
					record = record << 1;
					uchar cur_value = img.at<uchar>(y, x);
					if (cur_value > value)
						record += 1;
				}
			}
			Census[row][col] = record;
			//unsigned int max_value = std::pow(2, window_size*window_size + 1) - 1;
			//Census_image.at<uchar>(row, col) = 255 * record / max_value;
		}
	}
	//cv::imwrite("Census_img.png", Census_image);
}

void SGM::CensusCost(const std::vector<std::vector<unsigned int>>& census_left,
	const std::vector<std::vector<unsigned int>>& census_right,
	const SGM::SGMPara& para,
	std::vector<std::vector<std::vector<ushort>>>& C,
	const bool& flag)
{
	int height = census_left.size();
	int width = census_left[0].size();
	int window_size = para.CensusWindowSize;
	int minD = std::max(para.minDisparity, 0);
	int maxD = std::max(para.maxDisparity, 0);
	int half_win_size = window_size / 2;
	
	//����Hamming���룬��������ټ����������1�ĸ���
	int begin, end;//���㷶Χ
	if (flag)
	{
		begin = std::max(maxD + half_win_size, 0);
		end = width + ((minD - half_win_size) > 0 ? 0 : (minD - half_win_size));
	}
	else
	{
		begin = half_win_size;
		end = width - maxD - half_win_size;
	}
	//�����۾ۺϼ����Ӳ�ͼ
	cv::Mat disparity = cv::Mat::zeros(height, width, CV_8UC1);
	for (int row = half_win_size; row < height - half_win_size; row++)
	{
		for (int col = begin; col < end; col++)
		{
			unsigned int value_left = census_left[row][col];		
			ushort min_value = window_size * window_size + 1;//�����������ֵΪwindowsize*windowsize
			int best_dis;
			for (int d = minD; d < maxD; d++)
			{
				ushort count = 0;
				/*if (col - d < 0)
				{
					count = window_size * window_size + 1;
					break;
				}*/	
				unsigned int value_right;
				if (flag)
					value_right = census_right[row][col - d];
				else
				{
					value_right = census_right[row][col + d];
				}			
				unsigned int value = value_left ^ value_right;				
				while (value)
				{
					count++;
					value = value & (value - 1);
				}
				C[row][col][d - minD] = count;
				if (count < min_value)
				{
					min_value = count;
					best_dis = d;
				}
			}
			disparity.at<uchar>(row, col) = (uchar)(best_dis * 255 / maxD);
		}
	}
	cv::imwrite("disparity_before.png", disparity);
}

float SGM::DispRefine(const std::vector<int>& x, const std::vector<int>& y)
{
	Eigen::Matrix3d A;
	Eigen::Vector3d B, P;
	int size = x.size();

	for (int i = 0; i < size; i++)
	{
		A(i, 0) = x[i] * x[i];
		A(i, 1) = x[i];
		A(i, 2) = 1;
		B(i) = y[i];
	}
	P = A.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(B);
	//P = (A.transpose()*A).inverse()*(A.transpose())*B;
	float res = -P(1) / (2 * P(0));
	return res;
}

void SGM::GetDisparity(const SGM::SGMPara& para,
	const std::vector<std::vector<std::vector<ushort>>>& C,
	cv::Mat& disparity, const bool& flag)
{

	int window_size = para.CensusWindowSize;
	int minD = std::max(para.minDisparity, 0);
	int maxD = std::max(para.maxDisparity, 0);
	int half_win_size = window_size / 2;
	int height = C.size();
	int width = C[0].size();
	ushort p1 = (ushort)para.p1;
	ushort p2 = (ushort)para.p2;
	int uniquenessRatio = para.uniquenessRatio;
	int D = maxD - minD;

	int begin_row, end_row, begin_col, end_col, width1,minX;
	begin_row = half_win_size;
	end_row = height - half_win_size;
	if (flag)
	{
		begin_col = std::max(maxD + half_win_size, 0);
		end_col = width + ((minD - half_win_size) > 0 ? 0 : (minD - half_win_size));		
	}
	else
	{
		begin_col = half_win_size;
		end_col = width - maxD - half_win_size;
	}
	width1 = end_col - begin_col;
	minX = begin_col;
	std::vector<std::vector<std::vector<ushort>>> S(height, std::vector<std::vector<ushort>>(width, std::vector<ushort>(D, 0)));

	int nPath = 2;	//��·�����۾ۺϣ�����ֳ�������·������
	int dx, dy;//������������������
	const int MAX_COST = std::pow(2, 17) - 1;//S�����ֵΪ2^17-1

	for (int path = 1; path <= nPath; path++)
	{
		if (path == 1)
		{
			//begin_row = half_win_size;
			//end_row = height - half_win_size;
			dx = dy = 1;//�������
		}
		else
		{
			/*begin_row = height - half_win_size - 1;
			end_row = half_win_size - 1;
			begin_col = end_col - 1;
			end_col = std::max(maxD + half_win_size, 0) - 1;*/
			std::swap(begin_col, end_col);
			std::swap(begin_row, end_row);
			begin_col--, end_col--, begin_row--, end_row--;
			dx = dy = -1;//�������
		}
		//һ�μ�������·���Ĵ���ֵ���ֱ�Ϊ���Ϸ�����·����minLr0��ʾ��һ�е���С���۾ۺ�ֵ,minLr1��ʾ��ǰ�е���С���۾ۺ�ֵ
		//Lr0��ʾ��һ�еĴ��۾ۺ�ֵ��Lr1��ʾ��ǰ�еĴ��۾ۺ�ֵ����ʼΪ0
		std::vector<std::vector<ushort>> minLr0(width1, std::vector<ushort>(4, 0)), minLr1(width1, std::vector<ushort>(4, 0));
		std::vector<std::vector<std::vector<ushort>>> Lr0(width1, std::vector<std::vector<ushort>>(D, std::vector<ushort>(4, 0)));
		std::vector<std::vector<std::vector<ushort>>> Lr1(width1, std::vector<std::vector<ushort>>(D, std::vector<ushort>(4, 0)));
	
		for (int row = begin_row; row != end_row; row += dy)
		{
			for (int col = begin_col; col != end_col; col += dx)
			{
				ushort minL0 = MAX_COST, minL1 = MAX_COST, minL2 = MAX_COST, minL3 = MAX_COST;
				int index_col_Lr = col - minX;
				for (int d = minD; d < maxD; d++)
				{
					int index_d = d - minD;
					ushort Cpd = C[row][col][index_d], L0, L1, L2, L3;
					if (dx == 1 && (row == begin_row || row == end_row - 1 || col == begin_col || col == end_col - 1))
					{//ǰ����·��
						L0 = Cpd;
						L1 = Cpd;
						L2 = Cpd;
						L3 = Cpd;//�߽��
					}
					else if (dx == -1 && (row == begin_row || row == end_row + 1 || col == begin_col || col == end_col + 1))
					{//�����·��
						L0 = Cpd;
						L1 = Cpd;
						L2 = Cpd;
						L3 = Cpd;//�߽��
					}
					else
					{
						ushort temp01, temp02, temp11, temp12, temp21, temp22, temp31, temp32;
						if (d - 1 < minD)
						{
							temp01 = Lr0[index_col_Lr - dx][index_d + 1][0];//��ǰ�У�·��0
							temp11 = Lr1[index_col_Lr - dx][index_d + 1][1];//��һ�У�·��1
							temp21 = Lr1[index_col_Lr][index_d + 1][2];//��һ�У�·��2
							temp31 = Lr1[index_col_Lr + dx][index_d + 1][3];//��һ�У�·��3��path=2ʱ��Ϊ·��4,5,6,7
						}
						else if (d + 1 >= maxD)
						{
							temp01 = Lr0[index_col_Lr - dx][index_d - 1][0];
							temp11 = Lr1[index_col_Lr - dx][index_d - 1][1];
							temp21 = Lr1[index_col_Lr][index_d - 1][2];
							temp31 = Lr1[index_col_Lr + dx][index_d - 1][3];
						}
						else
						{
							temp01 = std::min(Lr0[index_col_Lr - dx][index_d - 1][0], Lr0[index_col_Lr - dx][index_d + 1][0]);
							temp11 = std::min(Lr1[index_col_Lr - dx][index_d - 1][1], Lr1[index_col_Lr - dx][index_d + 1][1]);
							temp21 = std::min(Lr1[index_col_Lr][index_d - 1][2], Lr1[index_col_Lr][index_d + 1][2]);
							temp31 = std::min(Lr1[index_col_Lr + dx][index_d - 1][3], Lr1[index_col_Lr + dx][index_d + 1][3]);
						}
						temp02 = std::min(Lr0[index_col_Lr - dx][index_d][0], (ushort)(minLr0[index_col_Lr - dx][0] + p2));
						temp12 = std::min(Lr1[index_col_Lr - dx][index_d][1], (ushort)(minLr1[index_col_Lr - dx][1] + p2));
						temp22 = std::min(Lr1[index_col_Lr][index_d][2], (ushort)(minLr1[index_col_Lr][2] + p2));
						temp32 = std::min(Lr1[index_col_Lr + dx][index_d][3], (ushort)(minLr1[index_col_Lr + dx][3] + p2));
						L0 = Cpd + std::min(ushort(temp01 + p1), temp02) - minLr0[index_col_Lr - dx][0];
						L1 = Cpd + std::min(ushort(temp11 + p1), temp12) - minLr1[index_col_Lr - dx][1];
						L2 = Cpd + std::min(ushort(temp21 + p1), temp22) - minLr1[index_col_Lr][2];
						L3 = Cpd + std::min(ushort(temp31 + p1), temp32) - minLr1[index_col_Lr + dx][3];

					}
				
					//��Lr0��ֵ
					Lr0[index_col_Lr][index_d][0] = L0;
					minL0 = std::min(minL0, L0);

					Lr0[index_col_Lr][index_d][1] = L1;
					minL1 = std::min(minL1, L1);

					Lr0[index_col_Lr][index_d][2] = L2;
					minL2 = std::min(minL2, L2);

					Lr0[index_col_Lr][index_d][3] = L3;
					minL3 = std::min(minL3, L3);

					S[row][col][index_d] += (L0 + L1 + L2 + L3);
				}
				minLr0[index_col_Lr][0] = minL0;
				minLr0[index_col_Lr][1] = minL1;
				minLr0[index_col_Lr][2] = minL2;
				minLr0[index_col_Lr][3] = minL3;
			}//������һ�к�,Lr0,Lr1�Լ�minLr0,minLr1����
			std::swap(minLr0, minLr1);
			std::swap(Lr0, Lr1);
			if (path == nPath)
			{
				//��·�����۾ۺ���ɣ������Ӳ�
				for (size_t x = begin_col; x !=end_col; x+=dx)
				{
					int minS = MAX_COST;
					int bestdisp = -1;
					for (int d = minD; d < maxD; d++)
					{
						int Cur_S = S[row][x][d - minD];
						if (Cur_S<minS)
						{
							minS = Cur_S;
							bestdisp = d;
						}
					}
					//Ψһ��Լ��
					int d;
					for (d = minD; d < maxD; d++)
					{
						if (S[row][x][d - minD] * (100 - uniquenessRatio) < minS * 100 && std::abs(bestdisp - d) > 1)
							break;//������Ψһ��Լ������
					}
					if (d < maxD)
						continue;//������Ψһ��Լ��
					d = bestdisp;
					//�Ӳ��Ż�
					float opti_d = float(d);
					if (d > minD&&d < maxD - 1)//d=0,or d=maxD-1,�޷��Ż�
					{
						std::vector<int> vec_d = { d - 1,d,d + 1 };
						std::vector<int> vec_S = { S[row][x][d - minD - 1],S[row][x][d - minD],S[row][x][d - minD + 1] };
						opti_d = DispRefine(vec_d, vec_S);
					}
					disparity.at<float>(row, x) = opti_d;
				}
			}
		}
	}
}

//�����Ӳ�
void SGM::DisparityComputation(const SGM::SGMPara& para,
	const std::vector<std::vector<std::vector<ushort>>>& C,
	cv::Mat& disparity)
{
	int window_size = para.CensusWindowSize;
	int minD = std::max(para.minDisparity, 0);
	int maxD = std::max(para.maxDisparity, 0);
	int half_win_size = window_size / 2;
	int height = C.size();
	int width = C[0].size();
	ushort p1 = (ushort)para.p1;
	ushort p2 = (ushort)para.p2;
	int uniquenessRatio = para.uniquenessRatio;
	int D = maxD - minD;
	int disp12MaxDiff = para.disp12MaxDiff;

	int begin_row = half_win_size;
	int end_row = height - half_win_size;
	int begin_col = std::max(maxD + half_win_size, 0);
	int end_col = width + ((minD - half_win_size) > 0 ? 0 : (minD - half_win_size));
	int width1 = end_col - begin_col;
	int minX = begin_col;
	int maxX = end_col;

	const int MAX_COST = std::pow(2, 17) - 1;//S�����ֵΪ2^17-1
	std::vector<std::vector<std::vector<ushort>>> S_left(height, std::vector<std::vector<ushort>>(width, std::vector<ushort>(D, 0)));
	std::vector<std::vector<ushort>> Cost_right(height, std::vector<ushort>(width, MAX_COST));
	int nPath = 2;	//��·�����۾ۺϣ�����ֳ�������·������
	int dx, dy;//������������������
	cv::Mat disparity_right = cv::Mat::zeros(disparity.size(), CV_32FC1);

	for (int path = 1; path <= nPath; path++)
	{
		if (path == 1)
		{
			dx = dy = 1;//�������
		}
		else
		{
			std::swap(begin_col, end_col);
			std::swap(begin_row, end_row);
			begin_col--, end_col--, begin_row--, end_row--;
			dx = dy = -1;//�������
		}
		//һ�μ�������·���Ĵ���ֵ���ֱ�Ϊ���Ϸ�����·����minLr0��ʾ��һ�е���С���۾ۺ�ֵ,minLr1��ʾ��ǰ�е���С���۾ۺ�ֵ
		//Lr0��ʾ��һ�еĴ��۾ۺ�ֵ��Lr1��ʾ��ǰ�еĴ��۾ۺ�ֵ����ʼΪ0
		std::vector<std::vector<ushort>> minLr0(width1, std::vector<ushort>(4, 0)), minLr1(width1, std::vector<ushort>(4, 0));
		std::vector<std::vector<std::vector<ushort>>> Lr0(width1, std::vector<std::vector<ushort>>(D, std::vector<ushort>(4, 0)));
		std::vector<std::vector<std::vector<ushort>>> Lr1(width1, std::vector<std::vector<ushort>>(D, std::vector<ushort>(4, 0)));

		for (int row = begin_row; row != end_row; row += dy)
		{
			for (int col = begin_col; col != end_col; col += dx)
			{
				ushort minL0 = MAX_COST, minL1 = MAX_COST, minL2 = MAX_COST, minL3 = MAX_COST;
				int index_col_Lr = col - minX;
				for (int d = minD; d < maxD; d++)
				{
					int index_d = d - minD;
					ushort Cpd = C[row][col][index_d], L0, L1, L2, L3;
					if (dx == 1 && (row == begin_row || row == end_row - 1 || col == begin_col || col == end_col - 1))
					{//ǰ����·��
						L0 = Cpd;
						L1 = Cpd;
						L2 = Cpd;
						L3 = Cpd;//�߽��
					}
					else if (dx == -1 && (row == begin_row || row == end_row + 1 || col == begin_col || col == end_col + 1))
					{//�����·��
						L0 = Cpd;
						L1 = Cpd;
						L2 = Cpd;
						L3 = Cpd;//�߽��
					}
					else
					{
						ushort temp01, temp02, temp11, temp12, temp21, temp22, temp31, temp32;
						if (d - 1 < minD)
						{
							temp01 = Lr0[index_col_Lr - dx][index_d + 1][0];//��ǰ�У�·��0
							temp11 = Lr1[index_col_Lr - dx][index_d + 1][1];//��һ�У�·��1
							temp21 = Lr1[index_col_Lr][index_d + 1][2];//��һ�У�·��2
							temp31 = Lr1[index_col_Lr + dx][index_d + 1][3];//��һ�У�·��3��path=2ʱ��Ϊ·��4,5,6,7
						}
						else if (d + 1 >= maxD)
						{
							temp01 = Lr0[index_col_Lr - dx][index_d - 1][0];
							temp11 = Lr1[index_col_Lr - dx][index_d - 1][1];
							temp21 = Lr1[index_col_Lr][index_d - 1][2];
							temp31 = Lr1[index_col_Lr + dx][index_d - 1][3];
						}
						else
						{
							temp01 = std::min(Lr0[index_col_Lr - dx][index_d - 1][0], Lr0[index_col_Lr - dx][index_d + 1][0]);
							temp11 = std::min(Lr1[index_col_Lr - dx][index_d - 1][1], Lr1[index_col_Lr - dx][index_d + 1][1]);
							temp21 = std::min(Lr1[index_col_Lr][index_d - 1][2], Lr1[index_col_Lr][index_d + 1][2]);
							temp31 = std::min(Lr1[index_col_Lr + dx][index_d - 1][3], Lr1[index_col_Lr + dx][index_d + 1][3]);
						}
						temp02 = std::min(Lr0[index_col_Lr - dx][index_d][0], (ushort)(minLr0[index_col_Lr - dx][0] + p2));
						temp12 = std::min(Lr1[index_col_Lr - dx][index_d][1], (ushort)(minLr1[index_col_Lr - dx][1] + p2));
						temp22 = std::min(Lr1[index_col_Lr][index_d][2], (ushort)(minLr1[index_col_Lr][2] + p2));
						temp32 = std::min(Lr1[index_col_Lr + dx][index_d][3], (ushort)(minLr1[index_col_Lr + dx][3] + p2));
						L0 = Cpd + std::min(ushort(temp01 + p1), temp02) - minLr0[index_col_Lr - dx][0];
						L1 = Cpd + std::min(ushort(temp11 + p1), temp12) - minLr1[index_col_Lr - dx][1];
						L2 = Cpd + std::min(ushort(temp21 + p1), temp22) - minLr1[index_col_Lr][2];
						L3 = Cpd + std::min(ushort(temp31 + p1), temp32) - minLr1[index_col_Lr + dx][3];

					}

					//��Lr0��ֵ
					Lr0[index_col_Lr][index_d][0] = L0;
					minL0 = std::min(minL0, L0);

					Lr0[index_col_Lr][index_d][1] = L1;
					minL1 = std::min(minL1, L1);

					Lr0[index_col_Lr][index_d][2] = L2;
					minL2 = std::min(minL2, L2);

					Lr0[index_col_Lr][index_d][3] = L3;
					minL3 = std::min(minL3, L3);

					S_left[row][col][index_d] += (L0 + L1 + L2 + L3);
				}
				minLr0[index_col_Lr][0] = minL0;
				minLr0[index_col_Lr][1] = minL1;
				minLr0[index_col_Lr][2] = minL2;
				minLr0[index_col_Lr][3] = minL3;
			}//������һ�к�,Lr0,Lr1�Լ�minLr0,minLr1����
			std::swap(minLr0, minLr1);
			std::swap(Lr0, Lr1);
			if (path == nPath)
			{
				//��·�����۾ۺ���ɣ������Ӳ�
				for (size_t x = begin_col; x != end_col; x += dx)
				{
					int minS = MAX_COST;
					int bestdisp = -1;
					for (int d = minD; d < maxD; d++)
					{
						int Cur_S = S_left[row][x][d - minD];
						if (Cur_S < minS)
						{
							minS = Cur_S;
							bestdisp = d;
						}
					}
					//Ψһ��Լ��
					int d;
					for (d = minD; d < maxD; d++)
					{
						if (S_left[row][x][d - minD] * (100 - uniquenessRatio) < minS * 100 && std::abs(bestdisp - d) > 1)
							break;//������Ψһ��Լ������
					}
					if (d < maxD)
						continue;//������Ψһ��Լ��
					d = bestdisp;
					//������ͼ�Ӳ�
					int x2 = x - d;
					if (Cost_right[row][x2] > minS)
					{
						Cost_right[row][x2] = minS;
						disparity_right.at<float>(row, x2) = d;
					}
					//�Ӳ��Ż�
					float opti_d = float(d);
					if (d > minD&&d < maxD - 1)//d=0,or d=maxD-1,�޷��Ż�
					{
						std::vector<int> vec_d = { d - 1,d,d + 1 };
						std::vector<int> vec_S = { S_left[row][x][d - minD - 1],S_left[row][x][d - minD],S_left[row][x][d - minD + 1] };
						opti_d = DispRefine(vec_d, vec_S);
					}
					disparity.at<float>(row, x) = opti_d;
				}
				//L-R Check
				for (size_t x = minX; x < maxX; x++)
				{
					float d = disparity.at<float>(row, x);
					if (std::abs(d) < 1e-8)
						continue;
					int d_lower = std::max((int)d, minD);
					int d_upper = std::min((int)(d + 1), maxD);
					int x_lower = x - d_lower;
					int x_upper = x - d_upper;
					if (x_lower >= 0 && x_lower < width && disparity_right.at<float>(row, x_lower) >= minD &&
						std::abs(disparity_right.at<float>(row, x_lower) - d_lower) > disp12MaxDiff && 
						x_upper >= 0 && x_upper < width && disparity_right.at<float>(row, x_upper) >= minD &&
						std::abs(disparity_right.at<float>(row, x_upper) - d_upper) > disp12MaxDiff)
						disparity.at<float>(row, x) = 0.0;//����������һ����ԭ��
				}
			}
		}
	}
	cv::Mat disparity_right_out = cv::Mat::zeros(disparity_right.size(), CV_8UC1);
	cv::normalize(disparity_right, disparity_right_out, 0, 255, cv::NORM_MINMAX);
	cv::imwrite("disparity_right.png", disparity_right_out);
}

//��ͨ����ȥ����ƥ��
void SGM::FilterSpeckle(cv::Mat& disparity, int MaxSpeckleSize,
	int maxDiff)
{
	int height = disparity.rows, width = disparity.cols, num_pixels = height * width;
	std::vector<std::vector<int>> labels(height, std::vector<int>(width, 0));//��¼ÿ�����ص�label
	std::vector<int> rtype(num_pixels, 0);//��¼ÿ��label����ͨ�����С�Ƿ�С����ֵ
	int dstep = (int)(disparity.step / sizeof(float));//disparity��������ݱ���Ϊfloat����

	int cur_label = 0;
	for (int row = 0; row < height; row++)
	{
		for (int col = 0; col < width; col++)
		{
			float d = disparity.at<float>(row, col);
			if (std::abs(d) < 1e-8)
				continue;//�Ӳ�Ϊ0����Ч�Ӳ�
			else
			{
				if (labels[row][col])
				{
					if (rtype[labels[row][col]])
					{
						disparity.at<float>(row, col) = 0.0;
					}
				}
				else
				{
					cv::Point2i p(col, row);
					cur_label++;//next label
					int count = 0;
					labels[row][col] = cur_label;
					std::stack<cv::Point2i> record;
					record.push(p);

					while (!record.empty())
					{
						count++;
						float* dp = &disparity.at<float>(p.y, p.x);
						float d = *dp;
						if (p.y < height - 1 && !labels[p.y + 1][p.x] &&
							std::abs(dp[+dstep])>0.0 &&
							std::abs(d - dp[+dstep]) <= maxDiff)
						{
							labels[p.y + 1][p.x] = cur_label;
							record.push(cv::Point2i(p.x, p.y + 1));
						}
						if (p.y > 0 && !labels[p.y - 1][p.x] &&
							std::abs(dp[-dstep])>0.0 &&
							std::abs(d - dp[-dstep]) <= maxDiff)
						{
							labels[p.y - 1][p.x] = cur_label;
							record.push(cv::Point2i(p.x, p.y - 1));
						}
						if (p.x < width - 1 && !labels[p.y][p.x + 1] &&
							std::abs(dp[+1])>0.0 &&
							std::abs(d - dp[+1]) <= maxDiff)
						{
							labels[p.y][p.x + 1] = cur_label;
							record.push(cv::Point2i(p.x + 1, p.y));
						}
						if (p.x > 0 && !labels[p.y][p.x - 1] &&
							std::abs(dp[-1])>0.0 &&
							std::abs(d - dp[-1]) <= maxDiff)
						{
							labels[p.y][p.x - 1] = cur_label;
							record.push(cv::Point2i(p.x - 1, p.y));
						}

						p = record.top();
						record.pop();
					}
					if (count <= MaxSpeckleSize)
					{
						rtype[labels[row][col]] = 1;
						disparity.at<float>(row, col) = 0.0;
					}
					else
					{
						rtype[labels[row][col]] = 0;
					}
				}
			}
		}
	}
}