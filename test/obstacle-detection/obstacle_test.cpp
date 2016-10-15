#include <bits/stdc++.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

void checkGround() {
    Mat img, gray;
    img = imread("floor_4.jpg", CV_LOAD_IMAGE_COLOR);
    // cvtColor(img, gray, CV_BGR2GRAY);

    const int imagewidth = img.cols;
    const int imageheight = img.rows;
    // cout << imagewidth << " " << imageheight << endl;

    const int WindowSize = 200;
    const int StepSize = 40;
    const int FLOOR_DIFF_THRES = 15;
    const int FLOOR_COLOR_THRES = 100;
    const float EMPTY_THRES = 0.05;

    const int N = WindowSize / StepSize;
    const int w = imagewidth / StepSize;
    const int h = imageheight / StepSize;

    int fCount[w][h];   // floor count dp in boxes
    int nfCount[w][h];  // non-floor count dp in boxes
    memset(fCount, -1, sizeof(fCount));
    memset(nfCount, -1, sizeof(nfCount));

    int countFloor = 0, countNFloor = 0;
    int start_i, end_i, start_j, end_j;
    Vec3b pixel; int pixel_b, pixel_g, pixel_r, pixel_avg;
    // Count number of floor and non-floor pixels in boxes of image
    for (int i = 0; i < w; i++) {
        for (int j = 0; j < h; j++) {
            countFloor = 0; countNFloor = 0;
            start_i = i * StepSize; end_i = start_i + StepSize;
            start_j = j * StepSize; end_j = start_j + StepSize;
            for (int k = start_i; k < end_i; k++) {
                for (int l = start_j; l < end_j; l++) {
                    pixel = img.at<Vec3b>(l, k);
                    pixel_b = (int)pixel.val[0];
                    pixel_g = (int)pixel.val[1];
                    pixel_r = (int)pixel.val[2];
                    pixel_avg = (pixel_b + pixel_g + pixel_r)/3;
                    if(pixel_b > FLOOR_COLOR_THRES and pixel_g > FLOOR_COLOR_THRES and pixel_r > FLOOR_COLOR_THRES and abs(pixel_b - pixel_avg) < FLOOR_DIFF_THRES and abs(pixel_g - pixel_avg) < FLOOR_DIFF_THRES and abs(pixel_r - pixel_avg) < FLOOR_DIFF_THRES) {
                        countFloor++;
                    }
                    else {
                        countNFloor++;
                    }
                }
            }
            fCount[i][j] = countFloor;
            nfCount[i][j] = countNFloor;
            // cout << countFloor << "," << countNFloor << " ";
        }
        // cout << endl;
    }


    // Find floor points in image
    vector<Point> floorPoints;
    for (int i = 0; i < w-N; i++) {
        for (int j = 0; j < h-N; j++) {
            countFloor = 0; countNFloor = 0;
            for (int k = i; k < i+N; k++) {
                for (int l = j; l < j+N; l++) {
                    countFloor += fCount[k][l];
                    countNFloor += nfCount[k][l];
                }
            }
            float floorRatio = ((float)countNFloor)/((float)countFloor + (float)countNFloor);
            // cout << floorRatio << " ";
            if(floorRatio < EMPTY_THRES) {
                floorPoints.push_back(Point((i+N/2) * StepSize, (j+N/2) * StepSize) );
            }
        }
        // cout << endl;
    }

    cout << "No. of floor points: " << floorPoints.size() << endl;

    // Draw points on image
    for(auto point: floorPoints) {
        // cout << point;
        circle(img, point, 10, Scalar(0, 255, 0), -1);
    }

    imwrite("floor_final.jpg", img);
}

int main(int argc, char const *argv[]) {
    checkGround();

    return 0;
}
