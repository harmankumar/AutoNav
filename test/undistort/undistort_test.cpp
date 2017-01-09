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
  std::cout << "Focal is " << f.focal << "\n";
  std::cout << "Wout(from config file) is " << f.wout << "\n";
  std::cout << "Hout is " << f.hout << "\n";
  Size S = Size(f.wout, f.hout);
  Mat image = imread(argv[1], CV_LOAD_IMAGE_COLOR);
  Mat image2;
  f.WarpImage(image, image2);   // undistort image

  cv::imwrite(argv[2], image2);
}
