#include <bits/stdc++.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

int main(int argc, char const *argv[])
{
    Mat img, img_hsi; Mat img_split[3];
    img = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    cvtColor(img, img_hsi, CV_BGR2HSV);
    imwrite("floor_hsi.jpg", img_hsi);
    split(img_hsi, img_split);
    imwrite("floor_hsi_h.jpg", img_split[0]);
    imwrite("floor_hsi_s.jpg", img_split[1]);
    imwrite("floor_hsi_v.jpg", img_split[2]);
}
