#include <bits/stdc++.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <lcm/lcm-cpp.hpp>
#include "exlcm/example_t.hpp"
#include "undistort/undistort.h"
#include "undistort/fishcam.h"

using namespace cv;
using namespace std;
typedef pair<int, int> floorpoint;

int currImage;

vector<floorpoint> floorPoints;
vector<floorpoint> floorPointsCoarse;
vector<floorpoint> floorPointsFiltered;
map<int, int> boundaryPoints;
const int WindowSize = 75;
const int WindowSizeCoarse = 100;
const int StepSize = 25;
const float EMPTY_THRES = 0.4;

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

    assert(ComponentNumber.empty());
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
double getZAngle(Point point, double imagewidth, double focalLength) {
    double x = (double)(point.x - imagewidth/2);
    double y = (double)(imageheight/2 - point.y);
    double theta = atan(x /(focalLength + y));    // height of bot approx. equal to distance from centre of mass
    // atan chosen as return value in range [-pi/2, pi/2]
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
    assert(not ComponentNumber.empty());
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

void checkGroundHSI(bool coarse) {
    if(coarse) {
        assert(floorPointsCoarse.empty());
    }
    else {
        assert(floorPoints.empty());
    }

    int N;
    if(coarse) {
        N = WindowSizeCoarse / StepSize;
    }
    else {
        N = WindowSize / StepSize;
    }
    const int w = imagewidth / StepSize;
    const int h = imageheight / StepSize;

    const float HUE_THRES = 170;
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
                    pixel_hsi = img_hsi.at<Vec3b>(l, k);
                    pixel_i = (int)pixel_hsi.val[0];
                    pixel_s = (int)pixel_hsi.val[1];
                    pixel_h = (int)pixel_hsi.val[2];
                    // cout << pixel_h << " " << pixel_s << " " << pixel_i << " " << "\t";
                    const int FLOOR_COLOR_THRES = 150;
                    if(/* (pixel_s < SAT_THRES) and */ (pixel_h > HUE_THRES)) {
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
    assert(boundaryPoints.empty());
    const int Y_GAP = 5 * StepSize;
    const int ZERO_GAP = 5 * StepSize;
    bool ignoreColumn = false;
    int xBound = floorPoints[0].first, yBound = floorPoints[0].second;
    // cout << "Floor points: " << endl;
    for(auto rit = floorPoints.rbegin(); rit != floorPoints.rend(); rit++) {
        // cout << rit->first << " " << rit->second << endl;
        if(rit->first != xBound) {  // New column start
            boundaryPoints[xBound] = yBound;   // Prev column added to boundary

            if(imageheight - rit->second > ZERO_GAP) {
                ignoreColumn = true;    // Ignore column if free space far from image bottom
            }
            else {
                ignoreColumn = false;
            }

            xBound = rit->first;
            yBound = rit->second;

        }
        else {      // Old column
            if((not ignoreColumn) and rit->second < yBound and (yBound - rit->second < Y_GAP)) {
                yBound = rit->second;
            }
        }
    }
}

void filterFloorPoints() {
    assert(floorPointsFiltered.empty());
    for(auto point: floorPointsCoarse) {
        if(point.second > boundaryPoints[point.first]) {
            floorPointsFiltered.push_back(point);
        }
    }
}

// finds distance(pixels) of obstacle in given direction
// Direction of bot is currently assumed to be same as camera
int findDistance() {
    int minYpoint;
    // for(auto point: boundaryPoints) {
    //
    // }

    return minYpoint;
}

// Check if file exists
inline bool exists (const std::string& name) {
    if (FILE *file = fopen(name.c_str(), "r")) {
          fclose(file);
          return true;
      } else {
          return false;
      }
}

class Handler
 {
     public:
         ~Handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan,
                const exlcm::example_t* msg)
        {
            currImage = (int)msg->currImage;
        }
};

int main(int argc, char const *argv[])
{
    const double focalLength = 806.74987791;
    lcm::LCM lcm;
    if(!lcm.good())
       return 1;

    exlcm::example_t data;
    Handler handlerObject;
    lcm.subscribe("PROCESSING_SEND", &Handler::handleMessage, &handlerObject);

    // std::string s = "undistort/calib_results_flycap.txt";
    // FishOcam f;
    // f.init(s);
    // int hout;
    // const int wout = f.width;
    // double hfov, vfov, focal;
    // f.createPerspectiveWarp(hout, hfov, vfov, focal, 1280, 1024, 1280, true);
    // Size S = Size(wout, hout);
    // img = Mat(S, CV_8UC3);

    int i = 0;
    string fileName;
    while(true) {
      lcm.handle();
      cout << "Received" << endl;
      fileName = "share/" + to_string(currImage) + ".jpg";
      cerr << "Filename: " << fileName << endl;
      if(exists(fileName)) {
        img = imread(fileName, CV_LOAD_IMAGE_COLOR);
        cout << "Image read successful" << endl;

        // Undistort image
        // Mat img = imread(fileName, CV_LOAD_IMAGE_COLOR);
        // f.WarpImage(img_in, img);

        imagewidth = img.cols;
        imageheight = img.rows;

        // Convert to hsv color space
        cvtColor(img, img_hsi, CV_BGR2HSV);

        checkGroundHSI(false);
        checkGroundHSI(true);

        if(!floorPointsCoarse.empty()) {
            assert(not floorPointsCoarse.empty());
            getBoundaryPoints();
            filterFloorPoints();
            mark(floorPointsFiltered);
            if(!floorPointsFiltered.empty()) {
                data.distance = imageheight - findDistance();
                floorpoint target = getMeanLargestComp(numComponents);
                Point targetPoint(target.first, target.second);
                circle(img, targetPoint, 15, Scalar(0, 0, 255), -1);
                data.angle = getZAngle(targetPoint, imagewidth, focalLength);
                cerr << "Angle sent: " << data.angle << endl;
                i++;                
            }
            else {
                data.angle = 0;
                cerr << "No floor points found" << endl;
            }
        }
        else {
            data.angle = 0;
            cerr << "No floor points found" << endl;
        }

        // Draw floorpoints on image
        for(auto point: floorPointsFiltered) {
            circle(img, Point(point.first, point.second), 7, Scalar(0, 255, 0), -1);
        }

        imwrite("share/" + to_string(currImage) + "-processed.jpg", img);
        lcm.publish("PROCESSING_RECEIVE", &data);
     }
      else {
          cerr << "File not found" << endl;
      }

    floorPoints.clear();
    floorPointsCoarse.clear();
    floorPointsFiltered.clear();
    boundaryPoints.clear();
    ComponentNumber.clear();
    numComponents = 0;
    }

    return 0;
}
