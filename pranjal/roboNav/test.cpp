#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iomanip>
#include <iostream>

using namespace std;

void print(cv::Mat mat, int prec)
{      
    for(int i=0; i<mat.size().height; i++)
    {
        cout << "[";
        for(int j=0; j<mat.size().width; j++)
        {
            cout << setprecision(prec) << mat.at<int>(i,j);
            if(j != mat.size().width-1)
                cout << ", ";
            else
                cout << "]" << endl; 
        }
    }
}



int main(int argc, char const *argv[])
{
	// cv::Mat test;// = cv::Mat(3,4,CV_64F);
	cv::Mat test = cv::Mat::zeros(30,40, CV_8U);

	//(y,x)
	test.at<char>(0,0) = 0;
	test.at<char>(0,1) = 1;
	test.at<char>(0,2) = 2;
	test.at<char>(0,3) = 3;

	test.at<char>(2,2) = 200;
	std::cout << test.rows << " " << test.cols << std::endl;
    cout << "test = " << endl << " " << test << endl << endl;
    // print(test,5);

    //(y,x)
	test.at<char>(cv::Point2f(0,0)) = 0;
	test.at<char>(cv::Point2f(1,0)) = 100;
	test.at<char>(cv::Point2f(2,0)) = 200;
	test.at<char>(cv::Point2f(3,0)) = 255;

	test.at<char>(cv::Point2f(1,1)) = 150;
	std::cout << test.rows << " " << test.cols << std::endl;
    cout << "test = " << endl << " " << test << endl << endl;
    // print(test,5);

    cv::namedWindow( "Display window", cv::WINDOW_NORMAL );// Create a window for display.
    cv::imshow( "Display window", test );
    // cv::imwrite("test.jpg",test);

    cv::waitKey(0); 

	return 0;
}