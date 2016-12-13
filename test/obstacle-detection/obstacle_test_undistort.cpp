#include <bits/stdc++.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "../undistort/undistort.h"
#include "../undistort/fishcam.h"

using namespace cv;
using namespace std;

typedef pair<int, int> floorpoint;

vector<floorpoint> floorPoints;
vector<floorpoint> floorPointsCoarse;
set<floorpoint> boundaryPoints;
const int WindowSize = 300;
const int WindowSizeCoarse = 500;
const int StepSize = 40;
const float EMPTY_THRES = 0.9;

map<floorpoint, int> ComponentNumber;
int numComponents = 0;

Mat img;
int imagewidth, imageheight;
Mat img_hsi;

void constructHSIImage() {
    img_hsi = Mat(img.rows, img.cols, img.type());

    float r, g, b, h, s, in;

    for(int i = 0; i < img.rows; i++)
    {
        for(int j = 0; j < img.cols; j++)
        {
            b = img.at<Vec3b>(i, j)[0];
            g = img.at<Vec3b>(i, j)[1];
            r = img.at<Vec3b>(i, j)[2];

            in = (b + g + r) / 3;

            int min_val = 0;
            min_val = std::min(r, std::min(b,g));

            s = 1 - 3*(min_val/(b + g + r));
            if(s < 0.00001)
            {
                s = 0;
            } else if(s > 0.99999){
                s = 1;
            }

            if(s != 0)
            {
                h = 0.5 * ((r - g) + (r - b)) / sqrt(((r - g)*(r - g)) + ((r - b)*(g - b)));
                h = acos(h);

                if(b <= g)
                {
                    h = h;
                } else {
                    h = ((360 * 3.14159265) / 180.0) - h;
                }
            }

            img_hsi.at<Vec3b>(i, j)[0] = (h * 180) / 3.14159265;
            img_hsi.at<Vec3b>(i, j)[1] = s*100;
            img_hsi.at<Vec3b>(i, j)[2] = in;
        }
    }
    imwrite("floor_hsi.jpg", img_hsi);
}


// bool isNeighbour(Point a, Point b)
// {
//     return (abs(a.x - b.x) <= StepSize or abs(a.y - b.y) <= StepSize);
// }

// dfs through points in image
void dfs(floorpoint currPoint, set<floorpoint>& visited, set<floorpoint>& allPoints) {
    int dx[4] = {0, 0, StepSize, -StepSize};
    int dy[4] = {StepSize, -StepSize, 0, 0};

    // if(visited.count(currPoint))
    //     return;

    ComponentNumber[currPoint] = numComponents;
    visited.insert(currPoint);

    for(int k=0; k<4; k++)
    {
        floorpoint newPoint = make_pair(currPoint.first + dx[k], currPoint.second + dy[k]);

        if(allPoints.count(newPoint) and (!visited.count(newPoint)))
            dfs(newPoint, visited, allPoints);
    }

}


// Mark connected components in point vector
void mark(vector<floorpoint>& v) {
    set<floorpoint> allPoints;
    set<floorpoint> visited;

    numComponents = 0;

    for(auto it: v)
        allPoints.insert(it);

    for(auto it: allPoints)
    {
        if(visited.count(it))
            continue;
        numComponents++;
        dfs(it, visited, allPoints);
    }
}

// Get yaw angle of bot
// TODO: test
double getZAngle(Point point, double imagewidth, double focalLength) {
    double theta = atan((double)(point.x - imagewidth)/focalLength);
    return theta;
}

// Mean point of largest component - to follow
floorpoint getMeanLargestComp(int num) {
    int* numPoints = new int[num+1];
    // Set to 0
    for(int i = 1; i <= num; ++i) {
        numPoints[i] = 0;
    }
    int maxComponent, maxPoints;
    for(auto iterator = ComponentNumber.begin(); iterator != ComponentNumber.end(); iterator++) {
        // cout << iterator->second;
        numPoints[iterator->second]++;
    }

    // Find component having max number of points
    maxPoints = 0;
    for(int i = 1; i <= num; ++i) {
        int numPoint = numPoints[i];
        if(maxPoints < numPoint) {
            maxPoints = numPoint;
            maxComponent = i;
        }
    }
    cout << "Max component: " << maxComponent << endl;
    cout << "Number of points: " << maxPoints << endl;

    map<int, vector<int> > relevantComponent;
    // find center of connected component
    double target_x = 0, target_y = 0; int n = 0;
    map<int, int > meanPath;

    for(auto iterator = ComponentNumber.begin(); iterator != ComponentNumber.end(); iterator++) {
        if(iterator->second == maxComponent) {
            n++;
            // cout << iterator->first.first << " " << iterator->first.second << endl;
            /* Forming relevant component */
            auto point = iterator->first;
            relevantComponent[point.second].push_back(point.first);

            /* Mean Calculation */
            target_x += point.first;
            target_y += point.second;
        }
    }


    for(auto it: relevantComponent)
    {
        int mean  = 0;
        for(auto it1 : it.second)
            mean += it1;

        mean /= (it.second).size();
        meanPath[it.first] = mean;
    }

    /* Draw mean path */
    auto it = meanPath.begin();
    int pathCount = 0;
    const int SKIP = 3;
    while(it != meanPath.end()) {
        // // Draw points
        // circle(img, Point(it->second, it->first), 10, Scalar(255, 0, 0), -1);
        // Draw path
        auto itNext = it;
        if(pathCount + SKIP < meanPath.size()) {
            advance(itNext, SKIP);
            if(itNext != meanPath.end()) {
                auto currPoint = Point(it->second, it->first);
                auto nextPoint = Point(itNext->second, itNext->first);
                line(img, currPoint, nextPoint, Scalar(255, 0 ,0), 5);
            }
        }
        else {
            break;
        }
        advance(it, SKIP);
        pathCount += SKIP;
        if(pathCount >= meanPath.size())
            break;
    }

    cout << n << endl;
    target_x /= numPoints[maxComponent];
    target_y /= numPoints[maxComponent];
    return make_pair((int)target_x, (int)target_y);
}

void checkGround(bool coarse) {
    int N;
    if(coarse) {
        N = WindowSizeCoarse / StepSize;
    }
    else {
        N = WindowSize / StepSize;
    }
    const int w = imagewidth / StepSize;
    const int h = imageheight / StepSize;

    const int FLOOR_DIFF_THRES = 15;
    const int FLOOR_COLOR_THRES = 130;

    int fCount[w][h];   // floor count dp in boxes
    int nfCount[w][h];  // non-floor count dp in boxes
    memset(fCount, -1, sizeof(fCount));
    memset(nfCount, -1, sizeof(nfCount));

    int countFloor = 0, countNFloor = 0;
    int start_i, end_i, start_j, end_j;
    Vec3b pixel_bgr; int pixel_b, pixel_g, pixel_r, pixel_avg;
    // Count number of floor and non-floor pixels in boxes of image
    for (int i = 0; i < w; i++) {
        for (int j = 0; j < h; j++) {
            countFloor = 0; countNFloor = 0;
            start_i = i * StepSize; end_i = start_i + StepSize;
            start_j = j * StepSize; end_j = start_j + StepSize;
            for (int k = start_i; k < end_i; k++) {
                for (int l = start_j; l < end_j; l++) {
                    pixel_bgr = img.at<Vec3b>(l, k);
                    pixel_b = (int)pixel_bgr.val[0];
                    pixel_g = (int)pixel_bgr.val[1];
                    pixel_r = (int)pixel_bgr.val[2];
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
            //cout << floorRatio << " ";
            if(floorRatio < EMPTY_THRES) {
                if(coarse) {
                    floorPointsCoarse.push_back(make_pair((i+N/2) * StepSize, (j+N/2) * StepSize) );
                }
                else {
                    floorPoints.push_back(make_pair((i+N/2) * StepSize, (j+N/2) * StepSize) );
                }
            }
        }
        // cout << endl;
    }
}

void checkGroundHSI(bool coarse) {
    int N;
    if(coarse) {
        N = WindowSizeCoarse / StepSize;
    }
    else {
        N = WindowSize / StepSize;
    }
    const int w = imagewidth / StepSize;
    const int h = imageheight / StepSize;

    const float HUE_THRES = 190;
    const float SAT_THRES = 40;
    const float INTENSITY_THRES = 150;

    int fCount[w][h];   // floor count dp in boxes
    int nfCount[w][h];  // non-floor count dp in boxes
    memset(fCount, -1, sizeof(fCount));
    memset(nfCount, -1, sizeof(nfCount));

    int countFloor = 0, countNFloor = 0;
    int start_i, end_i, start_j, end_j;
    Vec3b pixel_bgr; int pixel_b, pixel_g, pixel_r;
    Vec3b pixel_hsi; float pixel_h, pixel_s, pixel_i;
    // Count number of floor and non-floor pixels in boxes of image
    for (int i = 0; i < w; i++) {
        for (int j = 0; j < h; j++) {
            countFloor = 0; countNFloor = 0;
            start_i = i * StepSize; end_i = start_i + StepSize;
            start_j = j * StepSize; end_j = start_j + StepSize;
            for (int k = start_i; k < end_i; k++) {
                for (int l = start_j; l < end_j; l++) {
                    pixel_bgr = img.at<Vec3b>(l, k);
                    pixel_b = (int)pixel_bgr.val[0];
                    pixel_g = (int)pixel_bgr.val[1];
                    pixel_r = (int)pixel_bgr.val[2];
                    pixel_hsi = img_hsi.at<Vec3f>(l, k);
                    pixel_h = (float)pixel_hsi[2];
                    pixel_s = (float)pixel_hsi[1];
                    pixel_i = (float)pixel_hsi[0];

                    const int FLOOR_COLOR_THRES = 150;
                    if(/*(pixel_s < SAT_THRES) and */ ((int)pixel_h > HUE_THRES)) {
                        cout << pixel_hsi.val[0] << " " << pixel_hsi.val[1] << " " << pixel_hsi.val[2] << endl;
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
                if(coarse) {
                    floorPointsCoarse.push_back(make_pair((i+N/2) * StepSize, (j+N/2) * StepSize) );
                }
                else {
                    floorPoints.push_back(make_pair((i+N/2) * StepSize, (j+N/2) * StepSize) );
                }
            }
        }
        // cout << endl;
    }
}

// Get boundary points
void getBoundaryPoints() {
    const int Y_GAP = 10 * StepSize;
    int xBound = floorPoints[0].first, yBound = floorPoints[0].second;
    // cout << "Floor points: " << endl;
    for(auto rit = floorPoints.rbegin(); rit != floorPoints.rend(); rit++) {
        // cout << point.first << " " << point.second << endl;
        if(rit->first != xBound) {
            boundaryPoints.insert(make_pair(xBound, yBound));
            xBound = rit->first;
            yBound = rit->second;
        }
        else {
            if(rit->second < yBound and (yBound - rit->second < Y_GAP)) {
                yBound = rit->second;
            }
        }
    }
}

void findTileLines() {
    Mat gray, filt, thres;
    // Convert to grayscale
    cvtColor(img, gray, CV_BGR2GRAY);
    // Median filter
    medianBlur(gray, filt, 5);
    // Threshold image
    // adaptiveThreshold(filt, thres, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 11, 2);
    Canny(filt, thres, 50, 125);
    imwrite("floor_thres.jpg", thres);
    // Find hough transform
    vector<Vec2f> lines;
    HoughLines(thres, lines, 1, CV_PI/180, 500, 0, 0 );
    cout << lines.size() << endl;

    vector<pair<float, float> > line_params;
    for(auto tile_line:lines)
    {
        line_params.push_back(make_pair(tile_line[0], tile_line[1]));
    }
    sort(line_params.begin(), line_params.end());

    // Reduce lines by choosing only 1 from each cluster
    vector<pair<float, float> > line_params_unique;
    float cluster_thres = 10.0;
    float current_cluster_rho = line_params[0].first;
    line_params_unique.push_back(line_params[0]);

    for(auto params:line_params) {
        if(params.first - current_cluster_rho > cluster_thres) {
            current_cluster_rho = params.first;
            line_params_unique.push_back(params);
            cluster_thres = max((float)(0.05 * current_cluster_rho), cluster_thres);
        }
    }

    // draw lines
    const int len = 3000;

    for(auto tile_param: line_params_unique)
    {
        float rho = tile_param.first, theta = tile_param.second;
        cout << rho << " " << theta << endl;
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        //Adding stuff!
        pt1.x = cvRound(x0 + len*(-b));
        pt1.y = cvRound(y0 + len*(a));
        pt2.x = cvRound(x0 - len*(-b));
        pt2.y = cvRound(y0 - len*(a));
        line( img, pt1, pt2, Scalar(0,0,255), 2, CV_AA);
    }
    imwrite("floor_lines.jpg", img);
}

// finds distance(pixels) of obstacle in given direction
// Direction of bot is currently assumed to be same as camera
int findDistance() {
    const int X_SPAN = 6 * StepSize;  // Why?

    vector<int> pointYVec;
    for(auto point: floorPoints) {
        if(abs(point.first - imagewidth) < X_SPAN) {
            pointYVec.push_back(point.first);
        }
    }
    sort(pointYVec.begin(), pointYVec.end());

    return 0;
}

// dispHSVImage(Mat img) {
//     for(int i=0; i<img.rows; i++) {
//         for(int j=0; j<img.cols; j++) {
//             // You can now access the pixel value with cv::Vec3b
//             std::cout << img.at<cv::Vec3f>(i,j)[0] << " " << img.at<cv::Vec3f>(i,j)[1] << " " << img.at<cv::Vec3f>(i,j)[2] << "\t";
//         }
//     }
// }

int main(int argc, char const *argv[])
{
    // std::string s = "../undistort/calib_results_flycap.txt";
    // FishOcam f;
    // f.init(s);
    // int hout;
    // const int wout = f.width;
    // double hfov, vfov, focal;
    // f.createPerspectiveWarp(hout, hfov, vfov, focal, 1280, 1024, 1280, true);
    // Size S = Size(wout, hout);
    // img = Mat(S, CV_8UC3);

    // Undistort image
    Mat img_in = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    // f.WarpImage(img_in, img);
    // cout << "Done undistort" << endl;

    // img = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    // constructHSIImage();    // construct img_hsi
    cvtColor(img, img_hsi, CV_BGR2HSV);

    // imwrite("floor_hsi.jpg", img_hsi);
    imagewidth = img.cols;
    imageheight = img.rows;
    cout << imagewidth << " " << imageheight << endl;
    imwrite("undistort.jpg", img);
    imwrite("undistort_hsi.jpg", img_hsi);

    checkGroundHSI(false);     // find points using fine window
    checkGroundHSI(true);     // find points using coarse window
    // findTileLines();

    // Draw points on image
    cout << "No. of coarse floor points: " << floorPointsCoarse.size() << endl;
    for(auto point: floorPointsCoarse) {
        // cout << point;
        circle(img, Point(point.first, point.second), 7, Scalar(0, 255, 0), -1);
    }

    // Mark connected components in points
    mark(floorPointsCoarse);
    cout << "Marked: " << ComponentNumber.size() << endl;
    cout << "Number of components: " << numComponents << endl;

    floorpoint target = getMeanLargestComp(numComponents);
    // cout << target.first << " " << target.second << endl;
    Point targetPoint(target.first, target.second);
    circle(img, targetPoint, 15, Scalar(0, 0, 255), -1);

    // Get Boundary points
    getBoundaryPoints();
    for(auto it = boundaryPoints.begin(); it!=boundaryPoints.end(); it++) {
        // circle(img, boundaryPoint, 10, Scalar(255, 0, 0), -1);
        /* Draw boundary line */
        auto itNext = it;
        itNext++;
        if(itNext != boundaryPoints.end()) {
            line(img, Point(it->first, it->second), Point(itNext->first, itNext->second), Scalar(255, 0, 255), 5);
        }
    }

    imwrite(argv[2], img);

    // int distanceToObstacle = findDistance();
    // cout << "Distance to obstacle: " << distanceToObstacle << " pixels" << endl;

    return 0;
}
