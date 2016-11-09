#include "IOfunc.h"

void writeR(std::ofstream& out, const cv::Mat &R)
{
   	out<<' '<<R.rows<<' '<<R.cols<<' ';

	for (int i = 0; i < R.rows; ++i)
	{
		for (int j = 0; j < R.cols; ++j)
		{
			out<<R.at<double>(j,i)<<' ';
		}
	}

}

void writeT(std::ofstream& out, const cv::Vec3d &t)
{
   out << t[0] << ' ' << t[1] << ' '<<t[2]<<' ' ;
}


void readR(std::ifstream &inp,  cv::Mat &R)
{
   	int rows , cols;
   	inp>>rows;
   	inp>>cols;

   	if (rows > 0  && cols > 0)
   	{
   		
	   	R = cv::Mat(rows , cols , CV_64F, 0.0);
	   	double temp;

	   	for (int i = 0; i < R.rows; ++i)
		{
			for (int j = 0; j < R.cols; ++j)
			{
				inp>>R.at<double>(j,i);
			}
		}

   	}

}

void readT(std::ifstream& inp, cv::Vec3d &t)
{
   inp >> t[0]>>t[1]>>t[2];
}


