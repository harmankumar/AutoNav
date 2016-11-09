#include <bits/stdc++.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <lcm/lcm-cpp.hpp>
#include "exlcm/example_t.hpp"

using namespace cv;
using namespace std;

vector<Point> floorPoints;
const int WindowSize = 200;
const int StepSize = 40;
const int FLOOR_DIFF_THRES = 15;
const int FLOOR_COLOR_THRES = 100;
const float EMPTY_THRES = 0.05;

Mat img;
int imagewidth, imageheight;

int currImage;

class Handler
 {
     public:
         ~Handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan,
                const exlcm::example_t* msg)
        {
            currImage = (int)msg->imageId;
        }
};

// Check if file exists
inline bool exists (const std::string& name) {
  struct stat buffer;
  return (stat (name.c_str(), &buffer) == 0);
}


int main(int argc, char const *argv[])
{
    lcm::LCM lcm;
    if(!lcm.good())
       return 1;

    exlcm::example_t data;
    Handler handlerObject;
    lcm.subscribe("PROCESSING_SEND", &Handler::handleMessage, &handlerObject);

    int i = 0;
    string fileName;
    while(true) {
      lcm.handle();
      fileName = to_string(currImage) + ".jpg";
      if(exists(fileName)) {
        img = imread(fileName, CV_LOAD_IMAGE_COLOR);
        imagewidth = img.cols;
        imageheight = img.rows;

        checkGround();
        data.distance = findDistance();
        data.angle = getZAngle()
        lcm.publish("PROCESSING_RECEIVE", &data);
        i++;
      }

    }

    return 0;
}
