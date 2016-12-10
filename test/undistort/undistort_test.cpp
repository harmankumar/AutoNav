#include "undistort.h"
#include "fishcam.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types_c.h>

using namespace cv;

int main(int argc, char const *argv[]) {
  std::string s = "calib_results_flycap.txt";
  FishOcam f;
  f.init(s);
  int hout;
  const int wout = f.width;
  double hfov, vfov, focal;
  f.createPerspectiveWarp(hout, hfov, vfov, focal, 1280, 1024, 1280, true);
  std::cout << "Focal is " << focal << "\n";
  std::cout << "Hout is " << hout << "\n";
  Size S = Size(wout, hout);
  Mat image = imread(argv[1], CV_LOAD_IMAGE_COLOR);
  Mat image2(Size(wout, hout), CV_8UC3);

  f.WarpImage(image, image2);   // undistort image

  cv::imwrite(argv[2], image2);
}
