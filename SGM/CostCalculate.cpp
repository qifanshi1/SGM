#include"CostCalculate.h"
#include<opencv2/core.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<Eigen/Core>
#include<Eigen/Geometry>

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
	std::vector<std::vector<std::vector<ushort>>>& C)
{
	int height = census_left.size();
	int width = census_left[0].size();
	int window_size = para.CensusWindowSize;
	int minD = std::max(para.minDisparity, 0);
	int maxD = std::max(para.maxDisparity, 0);
	int half_win_size = window_size / 2;

	//计算Hamming距离，先异或处理，再计算异或结果中1的个数
	int begin = std::max(maxD+half_win_size, 0);
	//int begin = std::max(minD + half_win_size, 0);
	int end = width + ((minD - half_win_size) > 0 ? 0 : (minD - half_win_size));
	//不代价聚合计算视差图
	cv::Mat disparity = cv::Mat::zeros(height, width, CV_8UC1);
	for (int row = half_win_size; row < height - half_win_size; row++)
	{
		for (int col = begin; col < end; col++)
		{
			unsigned int value_left = census_left[row][col];		
			ushort min_value = window_size * window_size + 1;//汉明距离最大值为windowsize*windowsize
			int best_dis;
			for (int d = minD; d < maxD; d++)
			{
				ushort count = 0;
				/*if (col - d < 0)
				{
					count = window_size * window_size + 1;
					break;
				}*/				
				unsigned int value_right = census_right[row][col - d];				
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
	cv::imwrite("disparity.png", disparity);
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
	int begin_col = std::max(maxD + half_win_size, 0);
	int end_col = width + ((minD - half_win_size) > 0 ? 0 : (minD - half_win_size));
	int width1 = end_col - begin_col;
	int D = maxD - minD;
	int minX = begin_col;

	std::vector<std::vector<std::vector<ushort>>> S(height, std::vector<std::vector<ushort>>(width, std::vector<ushort>(D, 0)));

	int nPath = 2;	//八路径代价聚合，计算分成两个四路径进行
	int dx, dy, begin_row, end_row;//正向遍历或者逆向遍历
	const int MAX_COST = std::pow(2, 17) - 1;//S的最大值为2^17-1

	for (int path = 1; path <= nPath; path++)
	{
		if (path == 1)
		{
			begin_row = half_win_size;
			end_row = height - half_win_size;
			dx = dy = 1;//正向遍历
		}
		else
		{
			begin_row = height - half_win_size - 1;
			end_row = half_win_size - 1;
			begin_col = end_col - 1;
			end_col = std::max(maxD + half_win_size, 0) - 1;
			dx = dy = -1;//逆向遍历
		}
		//一次计算四条路径的代价值，分别为左上方四条路径，minLr0表示上一行的最小代价聚合值,minLr1表示当前行的最小代价聚合值
		//Lr0表示上一行的代价聚合值，Lr1表示当前行的代价聚合值，初始为0
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
					{//前四条路径
						L0 = Cpd;
						L1 = Cpd;
						L2 = Cpd;
						L3 = Cpd;//边界点
					}
					else if (dx == -1 && (row == begin_row || row == end_row + 1 || col == begin_col || col == end_col + 1))
					{//后八条路径
						L0 = Cpd;
						L1 = Cpd;
						L2 = Cpd;
						L3 = Cpd;//边界点
					}
					else
					{
						ushort temp01, temp02, temp11, temp12, temp21, temp22, temp31, temp32;
						if (d - 1 < minD)
						{
							temp01 = Lr0[index_col_Lr - dx][index_d + 1][0];//当前行，路径0
							temp11 = Lr1[index_col_Lr - dx][index_d + 1][1];//上一行，路径1
							temp21 = Lr1[index_col_Lr][index_d + 1][2];//上一行，路径2
							temp31 = Lr1[index_col_Lr + dx][index_d + 1][3];//上一行，路径3，path=2时则为路径4,5,6,7
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
				
					//给Lr0赋值
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
			}//计算完一行后,Lr0,Lr1以及minLr0,minLr1交换
			std::swap(minLr0, minLr1);
			std::swap(Lr0, Lr1);
			if (path == nPath)
			{
				//八路径代价聚合完成，计算视差
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
					//唯一性约束
					int d;
					for (d = minD; d < maxD; d++)
					{
						if (S[row][x][d - minD] * (100 - uniquenessRatio) < minS * 100 && std::abs(bestdisp - d) > 1)
							break;//不满足唯一性约束条件
					}
					if (d < maxD)
						continue;//不满足唯一性约束
					d = bestdisp;
					//视差优化
					float opti_d = float(d);
					if (d > minD&&d < maxD - 1)//d=0,or d=maxD-1,无法优化
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